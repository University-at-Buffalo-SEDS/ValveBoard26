"""Tank-pressure acquisition liveness across boot and a Resync out of Launch.

The production data_acq_thread.c (whole file), the pressure driver settle logic,
overpressure.c and main_thread.c's recovery_prepare_outputs run together under
a deterministic cooperative scheduler on one virtual millisecond clock. The
control pseudo-thread feeds overpressure_step every tick as the main thread
does. Inert HAL: no MCU timing, no transducer physics, no RTOS preemption.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def function(source, name):
    match = re.search(r'^[^\n;]*\b' + re.escape(name) + r'\([^;]*?\)\s*\{', source, re.M)
    assert match, name
    start = match.start(); end, depth = source.index('{', start) + 1, 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}'); end += 1
    return source[start:end]


HAL = r'''#pragma once
#include <stdint.h>
#include <stddef.h>
typedef enum{HAL_OK=0,HAL_ERROR=1,HAL_BUSY=2,HAL_TIMEOUT=3}HAL_StatusTypeDef;
typedef struct{unsigned counter;}DMA_HandleTypeDef;
typedef struct{DMA_HandleTypeDef *DMA_Handle;}ADC_HandleTypeDef;
typedef struct{int unused;}TIM_HandleTypeDef;
typedef struct{int unused;}I2C_HandleTypeDef;
typedef struct{uint32_t Channel,Rank,SamplingTime,SingleDiff,OffsetNumber,Offset;}ADC_ChannelConfTypeDef;
#define ADC_CHANNEL_5 5U
#define ADC_REGULAR_RANK_1 1U
#define ADC_SAMPLETIME_47CYCLES_5 4U
#define ADC_SINGLE_ENDED 0U
#define ADC_OFFSET_NONE 0U
#define DMA_IT_HT 1U
#define DMA_IT_TC 2U
#define __HAL_DMA_GET_COUNTER(d) ((d)->counter)
#define __HAL_DMA_ENABLE_IT(d,i) ((void)(d),(void)(i))
#define __HAL_TIM_GET_COUNTER(t) 0U
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef*);HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef*);
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef*,uint32_t);
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef*,ADC_ChannelConfTypeDef*);
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef*,uint32_t*,uint32_t);
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef*);
uint32_t HAL_ADC_GetState(ADC_HandleTypeDef*);uint32_t HAL_ADC_GetError(ADC_HandleTypeDef*);
uint32_t HAL_TIM_Base_GetState(TIM_HandleTypeDef*);
'''

TX = r'''#pragma once
#include <stdint.h>
typedef unsigned UINT;typedef unsigned long ULONG;
typedef struct{int unused;}TX_THREAD;typedef struct{int unused;}TX_BYTE_POOL;
typedef struct{int unused;}TX_EVENT_FLAGS_GROUP;typedef struct{int unused;}TX_QUEUE;
#define TX_NULL ((void*)0)
#define TX_SUCCESS 0U
#define TX_NO_TIME_SLICE 0UL
#define TX_AUTO_START 1U
#define TX_TIMER_TICKS_PER_SECOND 1000U
#define TX_INTERRUPT_SAVE_AREA
#define TX_DISABLE
#define TX_RESTORE
ULONG tx_time_get(void);UINT tx_thread_sleep(ULONG);
UINT tx_thread_create(TX_THREAD*,const char*,void(*)(ULONG),ULONG,void*,ULONG,UINT,UINT,ULONG,UINT);
'''

VB_THREADS = r'''#pragma once
#include "tx_api.h"
#include "overpressure.h"
void data_acq_thread_entry(ULONG initial_input);
UINT create_data_acq_thread(TX_BYTE_POOL *byte_pool);
void data_acq_latest_pressure(overpressure_sample_t *out);
'''

TELEMETRY = r'''#pragma once
#include <stddef.h>
typedef int SedsResult;
#define SEDS_OK 0
#define SEDS_DT_FUEL_TANK_PRESSURE 77U
SedsResult log_telemetry_asynchronous(unsigned type,const void*data,size_t n,size_t size);
'''

LTC = r'''#pragma once
#include <stdint.h>
#include "stm32g4xx_hal.h"
typedef struct{int unused;}LTC2990_Handle_t;
#define LTC2990_I2C_ADDRESS_CURRENT (0x4C)
#define LTC2990_I2C_ADDRESS_VOLTAGE (0x4D)
enum{LTC2990_ROLE_VOLTAGE,LTC2990_ROLE_CURRENT};
int8_t LTC2990_Init(LTC2990_Handle_t*,I2C_HandleTypeDef*,uint8_t,int);
void LTC2990_Get_Values(LTC2990_Handle_t*,float*);
void LTC2990_Set_Wait_Service(void(*)(void));
void telemetry_ltc2990_update_voltage(LTC2990_Handle_t*);
void telemetry_ltc2990_update_current(LTC2990_Handle_t*);
'''

POWER = r'''#pragma once
#include <stdbool.h>
#include "stm32g4xx_hal.h"
bool power_i2c_needs_recovery(I2C_HandleTypeDef*);HAL_StatusTypeDef power_i2c_recover(I2C_HandleTypeDef*);
'''

PRESSURE_SD = r'''#pragma once
#include "pressure_transducer_driver.h"
UINT pressure_sd_init(pressure_transducer_t*);
'''

HARNESS = r'''
#define _XOPEN_SOURCE 700
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <ucontext.h>
#include "stm32g4xx_hal.h"
#include "tx_api.h"
#include "telemetry.h"
#include "ltc2990.h"
#include "power_i2c_recovery.h"
#include "pressure_transducer_driver.h"
#include "overpressure.h"
void data_acq_thread_entry(ULONG);
void data_acq_latest_pressure(overpressure_sample_t*);

/* Virtual clock and a two-thread cooperative scheduler: a thread runs until it
 * sleeps; the clock advances only when nothing is runnable. */
static uint32_t now;
uint32_t HAL_GetTick(void){return now;}
ULONG tx_time_get(void){return now;}
static ucontext_t sched,ctx[2];static uint32_t wake[2];static int current;
static char stacks[2][1<<16];
UINT tx_thread_sleep(ULONG t){wake[current]=now+(t?t:1U);swapcontext(&ctx[current],&sched);return 0;}
UINT tx_thread_create(TX_THREAD*a,const char*b,void(*c)(ULONG),ULONG d,void*e,ULONG f,UINT g,UINT h,ULONG i,UINT j)
{(void)a;(void)b;(void)c;(void)d;(void)e;(void)f;(void)g;(void)h;(void)i;(void)j;return 0;}

/* Inert ADC: the DMA ring always ends at slot 15. */
static uint16_t *ring;static uint16_t raw=400U;
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef*a){(void)a;return HAL_OK;}
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef*a){(void)a;return HAL_OK;}
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef*a,uint32_t b){(void)a;(void)b;return HAL_OK;}
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef*a,ADC_ChannelConfTypeDef*b){(void)a;(void)b;return HAL_OK;}
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef*a,uint32_t*b,uint32_t c){(void)a;(void)c;ring=(uint16_t*)b;return HAL_OK;}
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef*a){(void)a;return HAL_OK;}
uint32_t HAL_ADC_GetState(ADC_HandleTypeDef*a){(void)a;return 0;}
uint32_t HAL_ADC_GetError(ADC_HandleTypeDef*a){(void)a;return 0;}
uint32_t HAL_TIM_Base_GetState(TIM_HandleTypeDef*a){(void)a;return 0;}
static DMA_HandleTypeDef dma={16U};
ADC_HandleTypeDef hadc3={&dma};TIM_HandleTypeDef htim3;I2C_HandleTypeDef hi2c2;

/* Publication and power sensors. A power read may stall the acquisition loop. */
static unsigned published;static uint32_t first_published_at;static uint32_t stall_at,stall_ms;
SedsResult log_telemetry_asynchronous(unsigned type,const void*d,size_t n,size_t s)
{assert(type==SEDS_DT_FUEL_TANK_PRESSURE&&n==1U&&s==sizeof(float));(void)d;if(!published++)first_published_at=now;return SEDS_OK;}
int8_t LTC2990_Init(LTC2990_Handle_t*h,I2C_HandleTypeDef*i,uint8_t a,int r){(void)h;(void)i;(void)a;(void)r;return 0;}
void LTC2990_Get_Values(LTC2990_Handle_t*h,float*v){(void)h;(void)v;}
/* Inert driver: its waits are the stub sleeps below, never serviced. */
void LTC2990_Set_Wait_Service(void(*s)(void)){(void)s;}
void telemetry_ltc2990_update_voltage(LTC2990_Handle_t*h){(void)h;tx_thread_sleep(15U);
 if(stall_ms&&now>=stall_at){uint32_t ms=stall_ms;stall_ms=0U;tx_thread_sleep(ms);}}
void telemetry_ltc2990_update_current(LTC2990_Handle_t*h){(void)h;tx_thread_sleep(15U);}
bool power_i2c_needs_recovery(I2C_HandleTypeDef*h){(void)h;return false;}
UINT pressure_sd_init(pressure_transducer_t*s){(void)s;return 0;}
HAL_StatusTypeDef power_i2c_recover(I2C_HandleTypeDef*h){(void)h;return HAL_OK;}

/* recovery_prepare_outputs from main_thread.c with inert output drivers. */
typedef struct{int unused;}Solenoid_t;
static Solenoid_t pilot_solenoid;static uint8_t g_outputs_initialized=1U;static unsigned moves;
#define BOARD_RECOVERY_SAFE_OUTPUTS 2U
static uint32_t disturb_until;static uint16_t disturb_raw;
static void no_servo_open(void){moves++;}
static void dump_servo_open(void){moves++;disturb_until=now+300U;}
static void solenoidOff(Solenoid_t*s){(void)s;moves++;}
static uint8_t recovery_read_outputs(uint32_t*mask){*mask=6U;return BOARD_RECOVERY_SAFE_OUTPUTS;}
RECOVERY_PREPARE

static overpressure_t op;static unsigned stale,trips,faults;static uint32_t resync_at,last_seq,last_seq_at,max_gap,valid_before_settle;
static uint32_t boot_offset;
static void control(void){
 for(;;){
  if(resync_at&&now>=resync_at){resync_at=0U;assert(recovery_prepare_outputs(1U)&&moves==3U);}
  /* Model a transducer/ADC disturbance from the servo move (hypothesis a). */
  if(ring)ring[15]=disturb_until&&now<disturb_until?disturb_raw:raw;
  overpressure_sample_t s;data_acq_latest_pressure(&s);
  if(s.seq!=last_seq){if(last_seq&&now-last_seq_at>max_gap)max_gap=now-last_seq_at;last_seq=s.seq;last_seq_at=now;
   if(s.valid&&now<boot_offset+2000U)valid_before_settle++;}
  const uint32_t act=overpressure_step(&op,&s,now,0U);
  if((act&OVERPRESSURE_ACT_FAULT)&&op.fault==OVERPRESSURE_FAULT_STALE)stale++;
  if(act&OVERPRESSURE_ACT_FAULT)faults++;
  if(act&OVERPRESSURE_ACT_REPORT)trips++;
  tx_thread_sleep(1U);
 }
}
static void acq(void){data_acq_thread_entry(0);}
static void run(uint32_t until){
 while(now<until){
  int ran=0;
  for(int t=0;t<2;t++)if((int32_t)(now-wake[t])>=0){current=t;ran=1;swapcontext(&sched,&ctx[t]);}
  if(!ran){uint32_t next=wake[0];if((int32_t)(wake[1]-next)<0)next=wake[1];now=next;}
 }
}
int main(int argc,char**argv){
 const char*mode=argc>1?argv[1]:"";setvbuf(stdout,NULL,_IONBF,0);
 for(int t=0;t<2;t++){getcontext(&ctx[t]);ctx[t].uc_stack.ss_sp=stacks[t];ctx[t].uc_stack.ss_size=sizeof stacks[t];ctx[t].uc_link=&sched;}
 makecontext(&ctx[0],control,0);makecontext(&ctx[1],acq,0);
 now=boot_offset=5U;
 if(!strcmp(mode,"boot")){
  /* Boot: control starts its stale clock first, as main_thread does. The
   * transducer reads near full scale while settling. */
  raw=3000U;run(1500U);raw=400U;run(6000U);
  printf("boot stale=%u faults=%u trips=%u published=%u first=%u valid_before_settle=%u\n",
         stale,faults,trips,published,first_published_at,valid_before_settle);
  assert(published&&first_published_at>=2005U&&valid_before_settle==0U);
  assert(trips==0U&&op.stage==OVERPRESSURE_IDLE);
  assert(stale==0U&&faults==0U&&op.fault==OVERPRESSURE_FAULT_NONE);
 }else if(!strcmp(mode,"resync")||!strcmp(mode,"brownout_low")||!strcmp(mode,"brownout_rail")){
  run(8000U);unsigned before=stale;max_gap=0U;faults=0U;
  resync_at=10000U;disturb_raw=!strcmp(mode,"brownout_low")?0U:!strcmp(mode,"brownout_rail")?4095U:raw;
  run(16000U);
  printf("%s stale=%u faults=%u fault=%u gap=%u\n",mode,stale-before,faults,op.fault,max_gap);
  assert(moves==3U);
  /* The apply never stops acquisition: no report gap beyond the power read. */
  assert(max_gap<=60U&&stale==before);
  /* A transducer/ADC disturbance is an invalid or implausible reading, never stale. */
  if(!strcmp(mode,"resync"))assert(faults==0U);
  if(!strcmp(mode,"brownout_low"))assert(faults==1U);
  if(!strcmp(mode,"brownout_rail"))assert(faults==1U);
 }else if(!strcmp(mode,"deadboot")){
  /* An acquisition thread that never starts is still stale. */
  wake[1]=0x40000000U;run(4000U);
  printf("deadboot stale=%u\n",stale);
  assert(stale==1U&&op.fault==OVERPRESSURE_FAULT_STALE);
 }else if(!strcmp(mode,"stall")){
  run(8000U);unsigned before=stale;stall_at=9000U;stall_ms=2500U;run(14000U);
  printf("stall stale=%u gap=%u\n",stale-before,max_gap);
  /* A genuinely stopped acquisition is still reported stale. */
  assert(stale==before+1U);
 }else if(!strcmp(mode,"rise")){
  /* A genuine rise right after settling still trips, on consecutive readings only. */
  run(2100U);for(uint16_t r=400U;r<=1400U;r+=5U){raw=r;run(now+1U);}run(now+600U);
  printf("rise trips=%u stage=%u\n",trips,op.stage);
  assert(trips==1U&&op.stage==OVERPRESSURE_RELIEF);
 }else return 2;
 return 0;
}
'''


class PressureAcquisitionLiveness(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        tmp = Path(cls.tmp.name)
        for name, text in {'stm32g4xx_hal.h': HAL, 'tx_api.h': TX, 'VB-Threads.h': VB_THREADS,
                           'telemetry.h': TELEMETRY, 'ltc2990.h': LTC, 'power_i2c_recovery.h': POWER,
                           'pressure_sd.h': PRESSURE_SD}.items():
            (tmp / name).write_text(text)
        control = (ROOT / 'Core/Src/main_thread.c').read_text()
        (tmp / 'harness.c').write_text(HARNESS.replace(
            'RECOVERY_PREPARE', function(control, 'recovery_prepare_outputs')))
        cls.binary = tmp / 'liveness'
        subprocess.run(['cc', '-std=c11', '-D_XOPEN_SOURCE=700', '-DTELEMETRY_ENABLED', '-Wall', '-Wextra',
                        '-Werror', '-Wno-deprecated-declarations', '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-I', str(tmp), '-I', str(ROOT / 'Core/Inc'), '-I', str(ROOT / 'Drivers/Pressure'),
                        str(tmp / 'harness.c'), str(ROOT / 'Core/Src/data_acq_thread.c'),
                        str(ROOT / 'Drivers/Pressure/pressure_transducer_driver.c'),
                        str(ROOT / 'Core/Src/overpressure.c'), '-lm', '-o', str(cls.binary)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_mode(self, mode):
        result = subprocess.run([str(self.binary), mode], capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_boot_settle_window_is_not_reported_stale(self): self.run_mode('boot')
    def test_resync_apply_keeps_acquisition_running(self): self.run_mode('resync')
    def test_transducer_dropout_is_implausible_never_stale(self): self.run_mode('brownout_low')
    def test_transducer_rail_is_invalid_never_stale(self): self.run_mode('brownout_rail')
    def test_stopped_acquisition_is_still_stale(self): self.run_mode('stall')
    def test_acquisition_that_never_starts_is_stale(self): self.run_mode('deadboot')
    def test_genuine_rise_after_settling_still_trips(self): self.run_mode('rise')


if __name__ == '__main__':
    unittest.main()
