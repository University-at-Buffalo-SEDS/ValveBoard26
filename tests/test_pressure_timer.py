"""Execute the actual Cube-generated TIM3 initializer against the ADC contract."""
from pathlib import Path
import subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class PressureTimerTests(unittest.TestCase):
    def test_actual_tim3_initializer_requests_500hz_adc_update_triggers(self):
        source=(ROOT/'Core/Src/main.c').read_text()
        start=source.index('static void MX_TIM3_Init(void)\n{')
        function=source[start:source.index('\n}',start)+2]
        harness=r'''#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#define HAL_OK 0
#define TIM3 ((void*)0x40000400)
#define TIM_COUNTERMODE_UP 0
#define TIM_CLOCKDIVISION_DIV1 0
#define TIM_AUTORELOAD_PRELOAD_DISABLE 0
#define TIM_CLOCKSOURCE_INTERNAL 0
#define TIM_TRGO_RESET 0
#define TIM_TRGO_UPDATE 0x20
#define TIM_MASTERSLAVEMODE_DISABLE 0
typedef struct{uint32_t Prescaler,CounterMode,Period,ClockDivision,AutoReloadPreload;}init_t;
typedef struct{void *Instance;init_t Init;}TIM_HandleTypeDef;
typedef struct{unsigned ClockSource;}TIM_ClockConfigTypeDef;
typedef struct{unsigned MasterOutputTrigger,MasterSlaveMode;}TIM_MasterConfigTypeDef;
static TIM_HandleTypeDef htim3;static unsigned trigger;
int HAL_TIM_Base_Init(TIM_HandleTypeDef*t){assert(t->Instance==TIM3);return HAL_OK;}
int HAL_TIM_ConfigClockSource(TIM_HandleTypeDef*t,TIM_ClockConfigTypeDef*c){(void)t;assert(c->ClockSource==TIM_CLOCKSOURCE_INTERNAL);return HAL_OK;}
int HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef*t,TIM_MasterConfigTypeDef*c){(void)t;trigger=c->MasterOutputTrigger;return HAL_OK;}
void Error_Handler(void){abort();}
''' + function + r'''
int main(void){MX_TIM3_Init();assert(htim3.Init.Prescaler==1699);assert(htim3.Init.Period==199);
assert(170000000u/((htim3.Init.Prescaler+1u)*(htim3.Init.Period+1u))==500);
assert(trigger==TIM_TRGO_UPDATE);return 0;}
'''
        with tempfile.TemporaryDirectory() as temp:
            exe=Path(temp)/'pressure-timer'
            subprocess.run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror','-x','c','-','-o',str(exe)],input=harness,text=True,check=True)
            subprocess.run([str(exe)],check=True)
