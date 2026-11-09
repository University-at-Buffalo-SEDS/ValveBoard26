"""Tank-pressure sampling stays live while the power sensors misbehave.

The production data_acq_thread.c and Drivers/ltc2990.c (whole files), the
pressure driver, overpressure.c and main_thread.c's recovery_prepare_outputs
run on one virtual millisecond clock. The two LTC2990 power monitors are a
register-level model behind HAL_I2C_*: a trigger completes the configured
conversions 55 ms later, an I2C failure busy-waits its HAL timeout without
yielding, and a power-on reset returns CONTROL to 0 (internal temperature only),
so voltage/current status bits never set again.

At 10 s a Resync apply moves both servos. The fault modes model what that
transient can do to the power monitors without resetting the MCU. The pressure
sampler shares its thread with the LTC2990 reads, so every gap in its 50 Hz
readings is measured: as the control owner sees it (overpressure stale clock)
and as published to the station. Cooperative scheduler: no MCU timing, no
transducer physics, no preemption; a HAL busy-wait advances the clock without
letting any other thread run, as on the board.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
MAX_SAMPLE_GAP_MS = 500


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
#define I2C_MEMADD_SIZE_8BIT 1U
#define __HAL_DMA_GET_COUNTER(d) ((d)->counter)
#define __HAL_DMA_ENABLE_IT(d,i) ((void)(d),(void)(i))
#define __HAL_TIM_GET_COUNTER(t) 0U
uint32_t HAL_GetTick(void);void HAL_Delay(uint32_t);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef*);HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef*);
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef*,uint32_t);
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef*,ADC_ChannelConfTypeDef*);
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef*,uint32_t*,uint32_t);
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef*);
uint32_t HAL_ADC_GetState(ADC_HandleTypeDef*);uint32_t HAL_ADC_GetError(ADC_HandleTypeDef*);
uint32_t HAL_TIM_Base_GetState(TIM_HandleTypeDef*);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef*,uint16_t,uint32_t,uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef*,uint16_t,uint16_t,uint16_t,uint8_t*,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef*,uint16_t,uint16_t,uint16_t,uint8_t*,uint16_t,uint32_t);
'''

TX = r'''#pragma once
#include <stdint.h>
typedef unsigned UINT;typedef unsigned long ULONG;typedef void VOID;
typedef struct{int unused;}TX_THREAD;typedef struct{int unused;}TX_BYTE_POOL;
typedef struct{int unused;}TX_EVENT_FLAGS_GROUP;typedef struct{int unused;}TX_QUEUE;
typedef struct{int unused;}TX_MUTEX;
#define TX_NULL ((void*)0)
#define TX_SUCCESS 0U
#define TX_WAIT_FOREVER 0xFFFFFFFFUL
#define TX_NO_TIME_SLICE 0UL
#define TX_AUTO_START 1U
#define TX_TIMER_TICKS_PER_SECOND 1000U
#define TX_INTERRUPT_SAVE_AREA
#define TX_DISABLE
#define TX_RESTORE
ULONG tx_time_get(void);UINT tx_thread_sleep(ULONG);TX_THREAD*tx_thread_identify(void);
UINT tx_mutex_get(TX_MUTEX*,ULONG);UINT tx_mutex_put(TX_MUTEX*);
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
#include <stdint.h>
typedef int SedsResult;typedef uint32_t SedsDataType;
#define SEDS_OK 0
#define SEDS_DT_FUEL_TANK_PRESSURE 77U
#define SEDS_DT_BATTERY_VOLTAGE 78U
#define SEDS_DT_BATTERY_CURRENT 79U
SedsResult log_telemetry_asynchronous(SedsDataType type,const void*data,size_t n,size_t size);
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
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <ucontext.h>
#include "stm32g4xx_hal.h"
#include "tx_api.h"
#include "telemetry.h"
#include "power_i2c_recovery.h"
#include "pressure_transducer_driver.h"
#include "overpressure.h"
void data_acq_thread_entry(ULONG);
void data_acq_latest_pressure(overpressure_sample_t*);

/* Virtual clock and a two-thread cooperative scheduler: a thread runs until it
 * sleeps; the clock advances only when nothing is runnable, or inside a HAL
 * busy-wait, which blocks every other thread of its priority on the board. */
static uint32_t now;
uint32_t HAL_GetTick(void){return now;}
ULONG tx_time_get(void){return now;}
void HAL_Delay(uint32_t ms){now+=ms;}
static ucontext_t sched,ctx[2];static uint32_t wake[2];static int current;
static char stacks[2][1<<16];static TX_THREAD threads[2];
UINT tx_thread_sleep(ULONG t){wake[current]=now+(t?t:1U);swapcontext(&ctx[current],&sched);return 0;}
TX_THREAD*tx_thread_identify(void){return &threads[current];}
UINT tx_mutex_get(TX_MUTEX*m,ULONG w){(void)m;(void)w;return 0;}
UINT tx_mutex_put(TX_MUTEX*m){(void)m;return 0;}
UINT tx_thread_create(TX_THREAD*a,const char*b,void(*c)(ULONG),ULONG d,void*e,ULONG f,UINT g,UINT h,ULONG i,UINT j)
{(void)a;(void)b;(void)c;(void)d;(void)e;(void)f;(void)g;(void)h;(void)i;(void)j;return 0;}

/* Inert ADC: the DMA ring always ends at slot 15. */
static uint16_t *ring;static const uint16_t raw=400U;
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef*a){(void)a;return HAL_OK;}
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef*a){(void)a;return HAL_OK;}
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef*a,uint32_t b){(void)a;(void)b;return HAL_OK;}
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef*a,ADC_ChannelConfTypeDef*b){(void)a;(void)b;return HAL_OK;}
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef*a,uint32_t*b,uint32_t c){(void)a;(void)c;ring=(uint16_t*)b;ring[15]=raw;return HAL_OK;}
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef*a){(void)a;return HAL_OK;}
uint32_t HAL_ADC_GetState(ADC_HandleTypeDef*a){(void)a;return 0;}
uint32_t HAL_ADC_GetError(ADC_HandleTypeDef*a){(void)a;return 0;}
uint32_t HAL_TIM_Base_GetState(TIM_HandleTypeDef*a){(void)a;return 0;}
static DMA_HandleTypeDef dma={16U};
ADC_HandleTypeDef hadc3={&dma};TIM_HandleTypeDef htim3;I2C_HandleTypeDef hi2c2;
bool power_i2c_needs_recovery(I2C_HandleTypeDef*h){(void)h;return false;}
HAL_StatusTypeDef power_i2c_recover(I2C_HandleTypeDef*h){(void)h;return HAL_OK;}
UINT pressure_sd_init(pressure_transducer_t*s){(void)s;return 0;}

/* Two LTC2990s (7-bit 0x4C current, 0x4D voltage). A trigger completes the
 * conversions enabled by CONTROL[4:3]=11 after 55 ms (TINT first). POR leaves
 * CONTROL=0: internal temperature only, so no voltage/current bit ever sets. */
typedef struct{uint8_t control;uint32_t triggered;uint8_t pending;}Ltc;
static Ltc ltc[2];
static uint32_t bus_fault_until; /* every transaction busy-waits its timeout */
static Ltc*device(uint16_t address){return (address>>1)==0x4CU?&ltc[0]:(address>>1)==0x4DU?&ltc[1]:NULL;}
static HAL_StatusTypeDef bus(uint32_t timeout){
 if(bus_fault_until&&now<bus_fault_until){now+=timeout;return HAL_ERROR;}
 return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef*h,uint16_t a,uint32_t trials,uint32_t timeout)
{(void)h;(void)trials;return device(a)?bus(timeout):HAL_ERROR;}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef*h,uint16_t a,uint16_t reg,uint16_t size,uint8_t*data,uint16_t n,uint32_t timeout){
 (void)h;(void)size;(void)n;Ltc*d=device(a);if(!d)return HAL_ERROR;
 if(bus(timeout)!=HAL_OK)return HAL_ERROR;
 if(reg==0x01U)d->control=*data;
 if(reg==0x02U){d->triggered=now;d->pending=1U;}
 return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef*h,uint16_t a,uint16_t reg,uint16_t size,uint8_t*data,uint16_t n,uint32_t timeout){
 (void)h;(void)size;(void)n;Ltc*d=device(a);if(!d)return HAL_ERROR;
 if(bus(timeout)!=HAL_OK)return HAL_ERROR;
 const uint8_t done=d->pending&&now-d->triggered>=55U;
 if(reg==0x00U)*data=(uint8_t)(done?(((d->control>>3)&3U)==3U?0x3EU:0x02U):0x01U);
 else if(reg==0x01U)*data=d->control;
 else if(reg>=0x06U&&reg<=0x0DU)*data=(reg&1U)?0x40U:0x8AU; /* valid, mid-scale */
 else *data=0U;
 return HAL_OK;
}

/* Publication: pressure and both power reports, with their gaps. */
static uint32_t published_pressure,last_pressure_at,max_published_gap,voltage_reports,current_reports;
static uint8_t measuring;
SedsResult log_telemetry_asynchronous(SedsDataType type,const void*d,size_t n,size_t s){
 assert(n==1U&&s==sizeof(float));
 if(type==SEDS_DT_FUEL_TANK_PRESSURE){
  if(measuring&&published_pressure&&now-last_pressure_at>max_published_gap)max_published_gap=now-last_pressure_at;
  published_pressure++;last_pressure_at=now;
 }else if(type==SEDS_DT_BATTERY_VOLTAGE){assert(isfinite(*(const float*)d));voltage_reports++;}
 else if(type==SEDS_DT_BATTERY_CURRENT){assert(isfinite(*(const float*)d));current_reports++;}
 else assert(0);
 return SEDS_OK;
}

/* recovery_prepare_outputs from main_thread.c. The servo move is the fault. */
typedef struct{int unused;}Solenoid_t;
static Solenoid_t pilot_solenoid;static uint8_t g_outputs_initialized=1U;static unsigned moves;
#define BOARD_RECOVERY_SAFE_OUTPUTS 2U
static const char*mode="";
static void servo_transient(void){
 if(!strncmp(mode,"por",3)){ltc[0].control=ltc[1].control=0U;ltc[0].pending=ltc[1].pending=0U;}
 if(!strcmp(mode,"bus"))bus_fault_until=now+1000U;
}
static void no_servo_open(void){moves++;servo_transient();}
static void dump_servo_open(void){moves++;}
static void solenoidOff(Solenoid_t*s){(void)s;moves++;}
static uint8_t recovery_read_outputs(uint32_t*mask){*mask=6U;return BOARD_RECOVERY_SAFE_OUTPUTS;}
RECOVERY_PREPARE

static overpressure_t op;static unsigned stale;static uint32_t resync_at,last_seq,last_seq_at,max_gap;
static void control(void){
 for(;;){
  if(resync_at&&now>=resync_at){resync_at=0U;assert(recovery_prepare_outputs(1U)&&moves==3U);}
  overpressure_sample_t s;data_acq_latest_pressure(&s);
  if(s.seq!=last_seq){if(measuring&&last_seq&&now-last_seq_at>max_gap)max_gap=now-last_seq_at;last_seq=s.seq;last_seq_at=now;}
  const uint32_t act=overpressure_step(&op,&s,now,0U);
  if((act&OVERPRESSURE_ACT_FAULT)&&op.fault==OVERPRESSURE_FAULT_STALE)stale++;
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
 mode=argc>1?argv[1]:"";setvbuf(stdout,NULL,_IONBF,0);
 if(strcmp(mode,"normal")&&strcmp(mode,"por")&&strcmp(mode,"por_reconfig")&&strcmp(mode,"bus"))return 2;
 for(int t=0;t<2;t++){getcontext(&ctx[t]);ctx[t].uc_stack.ss_sp=stacks[t];ctx[t].uc_stack.ss_size=sizeof stacks[t];ctx[t].uc_link=&sched;}
 makecontext(&ctx[0],control,0);makecontext(&ctx[1],acq,0);
 now=5U;run(8000U);
 /* Healthy power monitors are configured and reporting before the Resync. */
 assert(((ltc[0].control>>3)&3U)==3U&&((ltc[1].control>>3)&3U)==3U);
 assert(voltage_reports>=5U&&current_reports>=5U);
 measuring=1U;const unsigned before=stale;resync_at=10000U;run(10000U);
 const unsigned voltage_at_move=voltage_reports,current_at_move=current_reports;run(16000U);
 printf("%s gap=%u published_gap=%u stale=%u voltage=%u current=%u\n",
        mode,max_gap,max_published_gap,stale-before,voltage_reports,current_reports);
 assert(moves==3U&&stale==before);
 assert(max_gap<MAX_SAMPLE_GAP_MS&&max_published_gap<MAX_SAMPLE_GAP_MS);
 if(!strcmp(mode,"por_reconfig")){
  /* A monitor back in its power-on mode is configured again and reports. */
  printf("reconfig control=%02x/%02x voltage=+%u current=+%u\n",ltc[1].control,ltc[0].control,
         voltage_reports-voltage_at_move,current_reports-current_at_move);
  assert(((ltc[0].control>>3)&3U)==3U&&((ltc[1].control>>3)&3U)==3U);
  assert(voltage_reports-voltage_at_move>=3U&&current_reports-current_at_move>=3U);
 }
 return 0;
}
'''


class PressureSamplerPowerStall(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        tmp = Path(cls.tmp.name)
        for name, text in {'stm32g4xx_hal.h': HAL, 'tx_api.h': TX, 'VB-Threads.h': VB_THREADS,
                           'telemetry.h': TELEMETRY, 'power_i2c_recovery.h': POWER,
                           'pressure_sd.h': PRESSURE_SD, 'main.h': '#pragma once\n#include "stm32g4xx_hal.h"\n',
                           'sedsnet_config.h': '#pragma once\n'}.items():
            (tmp / name).write_text(text)
        control = (ROOT / 'Core/Src/main_thread.c').read_text()
        (tmp / 'harness.c').write_text(HARNESS.replace(
            'RECOVERY_PREPARE', function(control, 'recovery_prepare_outputs')))
        cls.binary = tmp / 'sampler'
        subprocess.run(['cc', '-std=c11', '-D_XOPEN_SOURCE=700', '-DTELEMETRY_ENABLED',
                        f'-DMAX_SAMPLE_GAP_MS={MAX_SAMPLE_GAP_MS}U', '-Wall', '-Wextra',
                        '-Werror', '-Wno-deprecated-declarations', '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-I', str(tmp), '-I', str(ROOT / 'Core/Inc'), '-I', str(ROOT / 'Core/Inc/Drivers'),
                        '-I', str(ROOT / 'Drivers/Pressure'),
                        str(tmp / 'harness.c'), str(ROOT / 'Core/Src/data_acq_thread.c'),
                        str(ROOT / 'Core/Src/Drivers/ltc2990.c'),
                        str(ROOT / 'Drivers/Pressure/pressure_transducer_driver.c'),
                        str(ROOT / 'Core/Src/overpressure.c'), '-lm', '-o', str(cls.binary)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_mode(self, mode):
        result = subprocess.run([str(self.binary), mode], capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_resync_with_healthy_power_monitors_keeps_sampling(self): self.run_mode('normal')
    def test_power_monitor_reset_by_servo_move_does_not_stall_sampling(self): self.run_mode('por')
    def test_power_monitor_reset_by_servo_move_is_reconfigured(self): self.run_mode('por_reconfig')
    def test_power_i2c_timeouts_during_servo_move_do_not_stall_sampling(self): self.run_mode('bus')


if __name__ == '__main__':
    unittest.main()
