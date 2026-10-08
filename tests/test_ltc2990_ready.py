import re
import subprocess
import tempfile
import unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
class ConversionReady(unittest.TestCase):
    def test_delayed_result_bus_failure_timeout_and_clock_wrap(self):
        s=(ROOT/'Core/Src/Drivers/ltc2990.c').read_text()
        f=re.search(r"uint8_t LTC2990_ADC_Read_New_Data\(.*?\n\}",s,re.S).group()
        code=r"""
#include <stdint.h>
#include <assert.h>
#define TX_NULL 0
#define STATUS_REG 0
#define LTC2990_DATA_READY_TIMEOUT_MS 100U
typedef struct {int unused;} LTC2990_Handle_t;
static uint32_t now,start,ready_at;static int failed;
static uint32_t HAL_GetTick(void){return now;}
static void ltc2990_sleep_ms(uint32_t ms){now+=ms;}
static uint8_t status_bit_from_msb(uint8_t reg){return reg==6?2:0xff;}
static int LTC2990_Read_Register(LTC2990_Handle_t*h,uint8_t reg,uint8_t*out){
 (void)h;if(failed)return 1;
 *out=reg==0?((uint32_t)(now-start)>=ready_at?4:0):(reg==6?0x80:1);return 0;
}
"""+f+r"""
int main(void){LTC2990_Handle_t h={0};uint16_t raw=0;int8_t valid=0;
 start=now=10;ready_at=55;assert(!LTC2990_ADC_Read_New_Data(&h,6,&raw,&valid));
 assert(now==65&&raw==1&&valid);
 start=now=UINT32_MAX-20;ready_at=55;
 assert(!LTC2990_ADC_Read_New_Data(&h,6,&raw,&valid));assert(now==34);
 start=now=20;ready_at=1000;assert(LTC2990_ADC_Read_New_Data(&h,6,&raw,&valid));assert(now==120);
 failed=1;start=now=20;assert(LTC2990_ADC_Read_New_Data(&h,6,&raw,&valid));assert(now==20);
 assert(LTC2990_ADC_Read_New_Data(&h,9,&raw,&valid));
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);(p/'test.c').write_text(code)
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',str(p/'test.c'),'-o',str(p/'test')],check=True)
            subprocess.run([str(p/'test')],check=True)
