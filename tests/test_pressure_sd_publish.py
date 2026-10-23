"""Execute the real bounded admission retry helper, including tick rollover."""
from pathlib import Path
import subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class PublishTests(unittest.TestCase):
    def test_refused_enqueue_retries_original_bytes_but_never_replays_acceptance(self):
        source=(ROOT/'Core/Src/pressure_sd.c').read_text()
        helper=source[source.index('static SedsResult publish_batch('):source.index('static void run(')]
        harness=r'''#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <string.h>
typedef int SedsResult;
#define SEDS_OK 0
#define SEDS_IO -14
#define SEDS_DT_VALVE_PRESSURE_SD_V1 157797238
#define PRESSURE_SD_BYTES 44
#define TX_TIMER_TICKS_PER_SECOND 1000
static uint32_t now,g_pressure_sd_admission_retries;
static unsigned calls,sleeps,refusals;
static int failure;
static uint8_t expected[44];
uint32_t HAL_GetTick(void){return now;}
void tx_thread_sleep(unsigned ticks){assert(ticks==1);++sleeps;now+=ticks;}
SedsResult log_telemetry_asynchronous(int ty,const void *wire,size_t n,size_t width){
 assert(ty==SEDS_DT_VALVE_PRESSURE_SD_V1 && n==44 && width==1);
 assert(memcmp(wire,expected,44)==0);++calls;
 return calls<=refusals?SEDS_IO:failure;
}
''' + helper + r'''
int main(void){
 for(unsigned i=0;i<44;++i)expected[i]=(uint8_t)i;
 refusals=3;assert(publish_batch(expected)==SEDS_OK);
 assert(calls==4 && sleeps==3 && g_pressure_sd_admission_retries==3);
 /* Successful enqueue ends the retry loop immediately. */
 calls=sleeps=0;refusals=0;failure=-2;
 assert(publish_batch(expected)==-2 && calls==1 && sleeps==0);
 calls=sleeps=0;failure=0;refusals=1000;now=UINT32_MAX-100;
 assert(publish_batch(expected)==SEDS_IO);
 assert(calls==201 && sleeps==200 && now==99);
 return 0;
}'''
        with tempfile.TemporaryDirectory() as d:
            exe=Path(d)/'publish'
            subprocess.run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',str(exe)],input=harness,text=True,check=True)
            subprocess.run([str(exe)],check=True)
