"""Compile the actual publisher bodies with explicit SDK/local-owner controls."""
from pathlib import Path
import os,re,subprocess,tempfile
from checked_router_fixture import checked_hooks
from router_lifecycle_fixture import function
ROOT=Path(__file__).resolve().parents[1]
def code():
 source=(ROOT/"Core/Src/telemetry.c").read_text()
 enum=re.search(r"typedef enum \{[^}]+\} TelemetryStatusSubmit;",(ROOT/"Core/Inc/telemetry.h").read_text()).group()
 diagnostic=re.search(r"typedef struct \{.*?\} ValveStatusDiagnostic;",source,re.S).group()
 scaffold=r"""
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#define TELEMETRY_ENABLED 1
#define SEDS_FIRMWARE_SIM_TEST 1
typedef int SedsResult;
enum {SEDS_OK=0,SEDS_ERR=-1,SEDS_BAD_ARG=-10,SEDS_IO=-14,SEDS_DT_UMBILICAL_STATUS=242848455,SEDS_EK_UNSIGNED=0};
static struct {void*r;uint8_t created;}g_router={(void*)1,1};
static unsigned checked_depth;
static void(*test_after_lock)(void),(*test_before_sdk)(void);
static void telemetry_lock(void){assert(!checked_depth);++checked_depth;if(test_after_lock)test_after_lock();}
static void telemetry_unlock(void){assert(checked_depth==1);--checked_depth;}
static uint64_t utc=1000,change_clock_after_read;
static unsigned sdk_calls,clock_reads,last_id,last_state;
static uint64_t last_stamp;
static int sdk_result;
static uint64_t telemetry_unix_ms_locked(void){assert(checked_depth==1);clock_reads++;uint64_t original=utc;if(change_clock_after_read)utc=change_clock_after_read;return original;}
static unsigned tx_time_get(void){return 42;}
static SedsResult seds_router_log_typed_ex(void*r,int type,const void*data,size_t count,size_t width,int kind,const uint64_t*stamp,bool queue){assert(checked_depth==1&&r&&type==SEDS_DT_UMBILICAL_STATUS&&count==2&&width==1&&kind==SEDS_EK_UNSIGNED&&stamp&&queue);const uint8_t*p=data;assert(p[1]<=1);if(test_before_sdk)test_before_sdk();sdk_calls++;last_id=p[0];last_state=p[1];last_stamp=*stamp;return sdk_result;}
volatile uint32_t g_sim_umbilical_status_ok,g_sim_umbilical_status_fail;
"""
 return scaffold+"typedef uint8_t (*TelemetryStatusSnapshot)(uint8_t,uint8_t*);\n"+enum+"\n"+diagnostic+"\nvolatile ValveStatusDiagnostic g_valve_status_publish[7];\n"+checked_hooks(ROOT)+"\n"+"\n".join(function(source,n)for n in ("telemetry_publish_umbilical_snapshot_locked","telemetry_publish_umbilical_status_locked","telemetry_submit_umbilical_snapshot","telemetry_submit_umbilical_status","telemetry_publish_umbilical_status"))
def run(main):
 with tempfile.TemporaryDirectory(prefix="status-submit-")as tmp:
  exe=str(Path(tmp)/"case");flags=["-fsanitize=address,undefined","-fno-omit-frame-pointer"]if os.environ.get("VB_STATUS_SANITIZERS")=="1"else[]
  subprocess.run(["cc","-std=c11","-Wall","-Wextra","-Werror",*flags,"-x","c","-","-o",exe],input=code()+main,text=True,check=True)
  subprocess.run([exe],check=True,env={**os.environ,"ASAN_OPTIONS":"detect_leaks=1:halt_on_error=1","UBSAN_OPTIONS":"halt_on_error=1","LSAN_OPTIONS":"exitcode=23"})
