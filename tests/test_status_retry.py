import pathlib,re,subprocess,tempfile,unittest,os
ROOT=pathlib.Path(__file__).resolve().parents[1]
class StatusRetryTests(unittest.TestCase):
 def test_current_owner_stop_and_newer_state_supersede_refused_or_unknown_observation(self):
  header="\n".join(line for line in(ROOT/"Core/Inc/status_report_retry.h").read_text().splitlines()if not line.startswith("#include"))
  enum=re.search(r"typedef enum \{[^}]+\} TelemetryStatusSubmit;",(ROOT/"Core/Inc/telemetry.h").read_text()).group()
  code=r"""
#include <assert.h>
#include <stdint.h>
typedef uint32_t ULONG;typedef int SedsResult;
#define SEDS_OK 0
#define SEDS_IO -14
#define TX_TIMER_TICKS_PER_SECOND 1000
static ULONG now;static unsigned sdk_calls,last_id,last_state,snapshot_reads,inside_sdk;
static uint8_t physical_pilot=1;static int ready,refuse,remote_applied;
static ULONG tx_time_get(void){return now;}
static uint8_t current_owner(uint8_t id,uint8_t*state){assert(inside_sdk);snapshot_reads++;if(id!=0)return 0;*state=physical_pilot;return 1;}
"""+enum+r"""
static TelemetryStatusSubmit telemetry_submit_umbilical_snapshot(uint8_t id,uint8_t(*snapshot)(uint8_t,uint8_t*),SedsResult*out){
 uint8_t state=0;
 inside_sdk=1;*out=SEDS_IO;
 if(!ready||!snapshot||!snapshot(id,&state)){inside_sdk=0;return TELEMETRY_STATUS_DEFERRED;}
 sdk_calls++;last_id=id;last_state=state;
 if(refuse){inside_sdk=0;return TELEMETRY_STATUS_REFUSED;}
 *out=SEDS_OK;inside_sdk=0;return TELEMETRY_STATUS_ACCEPTED_LOCAL;
}
"""+header+r"""
int main(void){
 assert(publish_umbilical_status(0,1)==SEDS_IO&&sdk_calls==0&&status_report_pending==1&&status_report_deferred==1&&status_report_failures==0);
 now=99;retry_pending_status_reports();assert(!sdk_calls&&!snapshot_reads);
 now=100;retry_pending_status_reports();assert(status_report_pending==1&&!sdk_calls&&status_report_deferred==2);
 status_report_bind_snapshot(current_owner);
 /* A real Stop changes the owner without publishing/updating any cached value. */
 physical_pilot=0;ready=1;refuse=1;now=200;retry_pending_status_reports();
 assert(sdk_calls==1&&last_id==0&&last_state==0&&status_report_pending==1&&status_report_failures==1);
 /* A later owner transition must be observed again after local refusal. */
 physical_pilot=1;refuse=0;now=300;retry_pending_status_reports();
 assert(sdk_calls==2&&last_state==1&&status_report_pending==0&&status_report_recovered==1&&remote_applied==0);
 now=400;retry_pending_status_reports();assert(sdk_calls==2&&snapshot_reads==2);
 /* Another refused open followed by Stop cannot replay the cached open. */
 refuse=1;assert(publish_umbilical_status(0,1)==SEDS_IO);physical_pilot=0;refuse=0;now=500;retry_pending_status_reports();
 assert(last_state==0&&status_report_pending==0&&status_report_recovered==2&&status_report_failures==2);
 return 0;
}
"""
  with tempfile.TemporaryDirectory()as tmp:
   exe=str(pathlib.Path(tmp)/"status-retry");san=["-fsanitize=address,undefined","-fno-omit-frame-pointer"]if os.environ.get("VB_STATUS_SANITIZERS")=="1"else[]
   subprocess.run(["cc","-DTELEMETRY_ENABLED=1","-std=c11","-Wall","-Wextra","-Werror",*san,"-x","c","-","-o",exe],input=code,text=True,check=True)
   subprocess.run([exe],check=True,env={**os.environ,"ASAN_OPTIONS":"detect_leaks=1:halt_on_error=1","UBSAN_OPTIONS":"halt_on_error=1","LSAN_OPTIONS":"exitcode=23"})
