import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

class PowerI2cBusRecovery(unittest.TestCase):
    def test_interrupted_byte_and_permanent_stuck_lines(self):
        header = r"""
#pragma once
#include <stdint.h>
#include <stddef.h>
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
#define HAL_ERROR 1
typedef struct {void *Instance;} I2C_HandleTypeDef;
typedef struct {uint32_t Pin,Mode,Pull,Speed;} GPIO_InitTypeDef;
#define I2C2 ((void*)2)
#define GPIOA ((void*)1)
#define GPIO_PIN_8 256U
#define GPIO_PIN_9 512U
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define GPIO_MODE_OUTPUT_OD 7
#define GPIO_NOPULL 0
#define GPIO_SPEED_FREQ_LOW 0
#define I2C_ANALOGFILTER_ENABLE 1
#define I2C_FLAG_BUSY 1
extern unsigned busy;
#define __HAL_I2C_GET_FLAG(h,f) ((void)(h),(void)(f),busy)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
int HAL_GPIO_ReadPin(void*,unsigned);
void HAL_GPIO_WritePin(void*,unsigned,int);
void HAL_GPIO_Init(void*,GPIO_InitTypeDef*);
int HAL_I2C_DeInit(I2C_HandleTypeDef*);
int HAL_I2C_Init(I2C_HandleTypeDef*);
int HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef*,unsigned);
int HAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef*,unsigned);
void HAL_Delay(unsigned);
"""
        tx = r"""
#pragma once
#define TX_NULL NULL
#define TX_TIMER_TICKS_PER_SECOND 1000U
void *tx_thread_identify(void);
void tx_thread_sleep(unsigned);
"""
        harness = r"""
#include <assert.h>
#include "power_i2c_recovery.h"
unsigned busy;
static unsigned edges,delays,init_calls,deinit_calls,filters;
static unsigned release_after;
static int sda_low,scl_low,scl_stuck,init_failed,filter_failed,thread_active;
static I2C_HandleTypeDef handle={I2C2};
int HAL_GPIO_ReadPin(void *port,unsigned pin){
 assert(port==GPIOA);
 if(pin==GPIO_PIN_9)return !scl_low&&!scl_stuck;
 assert(pin==GPIO_PIN_8);return !sda_low&&(release_after==0||edges>=release_after);
}
void HAL_GPIO_WritePin(void *port,unsigned pins,int state){
 assert(port==GPIOA&&(pins&~(GPIO_PIN_8|GPIO_PIN_9))==0);
 if(pins&GPIO_PIN_8)sda_low=state==GPIO_PIN_RESET;
 if(pins&GPIO_PIN_9){
  if(scl_low&&state==GPIO_PIN_SET&&!sda_low)edges++;
  scl_low=state==GPIO_PIN_RESET;
 }
}
void HAL_GPIO_Init(void *port,GPIO_InitTypeDef *p){
 assert(port==GPIOA&&p->Pin==(GPIO_PIN_8|GPIO_PIN_9));
 assert(p->Mode==GPIO_MODE_OUTPUT_OD&&p->Pull==GPIO_NOPULL);
}
int HAL_I2C_DeInit(I2C_HandleTypeDef *h){assert(h==&handle);deinit_calls++;return HAL_OK;}
int HAL_I2C_Init(I2C_HandleTypeDef *h){assert(h==&handle);init_calls++;busy=0;return init_failed?HAL_ERROR:HAL_OK;}
int HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef *h,unsigned x){assert(h==&handle&&x==1);filters++;return filter_failed?HAL_ERROR:HAL_OK;}
int HAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef *h,unsigned x){assert(h==&handle&&x==0);filters++;return HAL_OK;}
void HAL_Delay(unsigned n){delays+=n;assert(delays<40);}
void *tx_thread_identify(void){return thread_active?(void*)1:NULL;}
void tx_thread_sleep(unsigned n){assert(n>0);HAL_Delay(n);}
static void reset(void){busy=edges=delays=init_calls=deinit_calls=filters=release_after=0;
 sda_low=scl_low=scl_stuck=init_failed=filter_failed=0;thread_active=1;}
int main(void){
 reset();assert(!power_i2c_needs_recovery(&handle));
 // Captured failure: controller BUSY, target holds SDA, SCL released.
 busy=1;release_after=3;assert(power_i2c_needs_recovery(&handle));
 assert(power_i2c_recover(&handle)==HAL_OK&&edges==3&&!busy);
 assert(init_calls==1&&deinit_calls==1&&filters==2&&!sda_low&&!scl_low);
 reset();busy=1;release_after=9;assert(power_i2c_recover(&handle)==HAL_OK&&edges==9);
 // A permanently low SDA cannot create an unbounded pulse loop.
 reset();release_after=100;assert(power_i2c_recover(&handle)==HAL_ERROR&&edges==9&&init_calls==1);
 // Never try to drive a held-low SCL high or wait forever for stretching.
 reset();scl_stuck=1;assert(power_i2c_recover(&handle)==HAL_ERROR&&edges==0&&init_calls==1);
 reset();busy=1;thread_active=0;assert(power_i2c_recover(&handle)==HAL_OK&&edges==0);
 reset();init_failed=1;assert(power_i2c_recover(&handle)==HAL_ERROR&&init_calls==1&&filters==0);
 reset();filter_failed=1;assert(power_i2c_recover(&handle)==HAL_ERROR&&filters==2);
 I2C_HandleTypeDef wrong={NULL};reset();assert(power_i2c_recover(NULL)==HAL_ERROR);
 assert(power_i2c_recover(&wrong)==HAL_ERROR&&init_calls==0);
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)
            (p/'main.h').write_text(header)
            (p/'tx_api.h').write_text(tx)
            (p/'power_i2c_recovery.h').write_text((ROOT/'Core/Inc/power_i2c_recovery.h').read_text())
            (p/'test.c').write_text(harness)
            exe=p/'test'
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
                '-I',tmp,str(ROOT/'Core/Src/power_i2c_recovery.c'),str(p/'test.c'),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
