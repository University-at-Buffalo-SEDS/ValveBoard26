"""All exact automatic report bytes and retained-origin retries; no SDK surrogate claim."""
from pathlib import Path
import re,subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class AbortReportPayloadTests(unittest.TestCase):
 def test_every_reason_timestamp_and_retry_byte(self):
  source=(ROOT/'Core/Src/telemetry_thread.c').read_text()
  reporter=re.search(r"static void telemetry_report_safety_abort\(void\)\s*\{.*?\n\}",source,re.S).group()
  prefix=r'''
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "safety_diagnostics.h"
typedef uint32_t ULONG;
#define TX_TIMER_TICKS_PER_SECOND 1000U
#define SEDS_DT_ABORT 9
#define SEDS_OK 0
typedef struct {uint64_t original_utc;uint32_t reason,epoch,lease;} thread_comm_safety_report_t;
static thread_comm_safety_report_t pending={.epoch=1};static uint32_t serial,active;
static uint8_t thread_comm_safety_report_claim(thread_comm_safety_report_t*out){if(!pending.reason||active)return 0;*out=pending;out->lease=++serial;active=out->lease;return 1;}
static void thread_comm_safety_report_finish(uint32_t lease,uint8_t retained){assert(active==lease);active=0;if(retained)pending.reason=0;}

static ULONG ticks;
static unsigned calls;
static int result=-1;
static uint64_t original;
static const char*expected;
static ULONG tx_time_get(void){return ticks;}
static int log_telemetry_string_asynchronous_at(int type,const char*text,uint64_t timestamp){assert(type==SEDS_DT_ABORT&&timestamp==original);assert(strlen(text)==strlen(expected)&&!memcmp(text,expected,strlen(expected)+1U));calls++;return result;}
'''
  main=r'''
int main(int argc,char**argv){assert(argc==3);unsigned reason=(unsigned)strtoul(argv[1],NULL,10);original=strtoull(argv[2],NULL,10);
 switch(reason){case SAFETY_ABORT_HEARTBEAT:expected="SEDS_AUTO_ABORT_V1:heartbeat timeout";break;case SAFETY_ABORT_INITIAL_HEARTBEAT:expected="SEDS_AUTO_ABORT_V1:initial heartbeat missing";break;case SAFETY_ABORT_CONTINUITY:expected="SEDS_AUTO_ABORT_V1:continuity lost";break;case SAFETY_ABORT_LAUNCH_CONTINUITY:expected="SEDS_AUTO_ABORT_V1:launch continuity timeout";break;case SAFETY_ABORT_ADC_START:expected="SEDS_AUTO_ABORT_V1:ADC start failed";break;default:expected="SEDS_AUTO_ABORT_V1:local";break;}
 telemetry_report_safety_abort();assert(!calls);pending.original_utc=original;pending.reason=reason;ticks=UINT32_MAX-500U;telemetry_report_safety_abort();if(!reason){assert(!calls);return 0;}assert(calls==1);ticks+=999U;telemetry_report_safety_abort();assert(calls==1);ticks++;result=SEDS_OK;telemetry_report_safety_abort();assert(calls==2);ticks+=10000U;telemetry_report_safety_abort();assert(calls==2);return 0;}
'''
  with tempfile.TemporaryDirectory(prefix='abort-report-bytes-')as temp:
   p=Path(temp);(p/'test.c').write_text(prefix+reporter+main);exe=p/'test'
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(ROOT/'Core/Inc'),str(p/'test.c'),'-o',str(exe)],check=True)
   for reason in [0,1,2,3,4,5,255]:
    for utc in [0,9000,(1<<64)-1]:
     with self.subTest(reason=reason,utc=utc):subprocess.run([str(exe),str(reason),str(utc)],check=True)
