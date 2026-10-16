"""Exercise the actual acquisition function against transient/persistent faults."""
import re
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class TransientConversion(unittest.TestCase):
    def test_retry_freshness_and_bounded_failure(self):
        source = (ROOT / 'Core/Src/Drivers/ltc2990.c').read_text()
        function = re.search(r'void LTC2990_Step\(.*?\n\}', source, re.S).group()
        code = r'''
#include <stdint.h>
#include <assert.h>
#include <math.h>
#define TX_NULL 0
#define LTC2990_ROLE_VOLTAGE 0
#define V1_MSB_REG 6
#define V2_MSB_REG 8
#define V3_MSB_REG 10
#define V4_MSB_REG 12
typedef struct { int role; float last_values[4]; } LTC2990_Handle_t;
static uint32_t now; static unsigned triggers, reads[4]; static int mode;
static uint32_t HAL_GetTick(void) { return now; }
static void ltc2990_sleep_ms(uint32_t ms) { now += ms; }
static int LTC2990_Trigger_Conversion(LTC2990_Handle_t *h) {
 (void)h; triggers++; if(mode==3) { now+=25; return 1; } return 0;
}
static int LTC2990_ADC_Read_New_Data(LTC2990_Handle_t *h, uint8_t reg, uint16_t *raw, int8_t *valid) {
 (void)h; unsigned channel=(reg-6)/2; reads[channel]++;
 if(channel==1 && (mode==1 || mode==4 || (mode==0 && reads[channel]==1))) {
  if(mode==4) now+=100; *valid=0; return 1;
 }
 if(channel==0 && mode==5 && reads[channel]==1) { *valid=0; return 1; }
 *valid=1; *raw=channel+10; return 0;
}
static float LTC2990_Code_To_Single_Ended_Voltage(LTC2990_Handle_t *h,uint16_t raw,uint8_t channel) {
 (void)h; (void)channel; return raw;
}
static float LTC2990_Code15_To_CurrentA(uint16_t raw) { return -(float)raw; }
''' + function + r'''
static void reset(int selected) {mode=selected;now=0;triggers=0;for(int i=0;i<4;i++)reads[i]=0;}
int main(void) {
 (void)HAL_GetTick();
 LTC2990_Handle_t h={.role=0,.last_values={99,99,99,99}};
 reset(0);LTC2990_Step(&h);assert(triggers==2&&h.last_values[1]==11&&reads[0]==1&&reads[1]==2);
 reset(1);LTC2990_Step(&h);assert(triggers==2&&isnan(h.last_values[1])&&h.last_values[0]==10);
 reset(3);LTC2990_Step(&h);assert(triggers==2&&now==50);for(int i=0;i<4;i++)assert(isnan(h.last_values[i]));
 reset(4);LTC2990_Step(&h);assert(triggers==1&&isnan(h.last_values[1]));
 reset(5);h.role=1;LTC2990_Step(&h);assert(triggers==2&&h.last_values[0]==-10&&h.last_values[1]==-12);
 assert(isnan(h.last_values[2])&&isnan(h.last_values[3]));
 reset(2);now=UINT32_MAX-5;LTC2990_Step(&h);assert(triggers==1&&h.last_values[0]==-10);
 LTC2990_Step(TX_NULL);
}
'''
        # Keep test stubs warning-clean as well as checking production C.
        code = code.replace('if(mode==4) now+=100; *valid=0;', 'if(mode==4) { now+=100; } *valid=0;')
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            (path / 'test.c').write_text(code)
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined', str(path / 'test.c'), '-o', str(path / 'test')], check=True)
            subprocess.run([str(path / 'test')], check=True)
