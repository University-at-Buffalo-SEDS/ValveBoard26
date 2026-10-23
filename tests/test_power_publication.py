import re
import subprocess
import tempfile
import unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
class PowerPublication(unittest.TestCase):
    def test_invalid_read_and_rejected_publication_are_distinct(self):
        source=(ROOT/'Core/Src/Drivers/ltc2990.c').read_text()
        start=source.index('typedef struct {')
        end=source.index('static inline void ltc2990_sleep_ms',start)
        code=r"""
#include <assert.h>
#include <stdint.h>
#include <math.h>
typedef int SedsResult;
typedef int SedsDataType;
#define SEDS_OK 0
static int result,calls;
static uint32_t tx_time_get(void){return 123;}
static SedsResult log_telemetry_asynchronous(SedsDataType type,const void*p,unsigned n,unsigned size){
 assert(type==7&&n==1&&size==sizeof(float));assert(*(const float*)p==16.25f);calls++;return result;
}
"""+source[start:end]+r"""
int main(void){
 report_power_sample(0,7,NAN);report_power_sample(0,7,INFINITY);
 assert(calls==0&&g_valve_power_publish[0].invalid==2);
 result=-2;report_power_sample(0,7,16.25f);
 assert(calls==1&&g_valve_power_publish[0].rejected==1&&g_valve_power_publish[0].last_result==-2);
 result=0;report_power_sample(0,7,16.25f);
 assert(calls==2&&g_valve_power_publish[0].accepted==1);
 assert(g_valve_power_publish[0].samples==4&&g_valve_power_publish[0].last_tick==123);
 assert(g_valve_power_publish[1].samples==0);
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);(p/'t.c').write_text(code)
            subprocess.run(['cc','-DTELEMETRY_ENABLED=1','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',str(p/'t.c'),'-o',str(p/'t')],check=True)
            subprocess.run([str(p/'t')],check=True)
