"""Valve-state reports keep their cadence and admission across network time steps.

Bench evidence (SEDSnet 4.3.14): after a clock step the Valve Board's reports
were refused or stopped; later the board stamped reports from a GS-heartbeat
estimate. With SEDSnet 6 every report carries the router's own network clock
and its time epoch: a follower steps to the leader's new epoch within one
announce, and the GS judges a report's age with the core age API, which
translates a stamp of the previous epoch across the step. The board needs no
heartbeat clock of its own.

The actual reporter, retry, submit, network-health and report-stamp C runs
tick by tick against a modeled SEDSnet follower clock (stepping to the GS's
new epoch at its next announce, every 2 s) and a GS that judges each report's
age across the step. Each case steps the GS clock once.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from router_lifecycle_fixture import function

ROOT = Path(__file__).resolve().parents[1]


def harness():
    telemetry = (ROOT / "Core/Src/telemetry.c").read_text()
    main = (ROOT / "Core/Src/main_thread.c").read_text()
    retry = (ROOT / "Core/Inc/status_report_retry.h").read_text()
    retry = "\n".join(l for l in retry.splitlines() if not l.startswith("#include"))
    period = re.search(r"^#define UMBILICAL_STATUS_PERIOD_TICKS .*$", main, re.M).group(0)
    reporter = main[main.index("static void publish_all_umbilical_statuses("):
                    main.index("static void publish_abort_umbilical_statuses")]
    scaffold = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define TELEMETRY_ENABLED 1
#define TX_TIMER_TICKS_PER_SECOND 1000U
#define TX_SUCCESS 0U
typedef unsigned long ULONG;
typedef int SedsResult;typedef int SedsRouter;
enum {SEDS_OK=0,SEDS_ERR=-1,SEDS_BAD_ARG=-10,SEDS_IO=-14,SEDS_DT_UMBILICAL_STATUS=7,SEDS_EK_UNSIGNED=0};
enum {CMD_PILOT_OPEN=0,CMD_NORMALLY_OPEN_OPEN=1,CMD_DUMP_OPEN=2,CMD_SEQUENCE=6};
typedef enum {TELEMETRY_STATUS_DEFERRED,TELEMETRY_STATUS_ACCEPTED_LOCAL,TELEMETRY_STATUS_REFUSED} TelemetryStatusSubmit;
typedef uint8_t (*TelemetryStatusSnapshot)(uint8_t,uint8_t*);
typedef struct {uint32_t calls,accepted,failed;uint8_t last_state;ULONG last_tick;SedsResult last_result;} ValveStatusDiagnostic;
static volatile ValveStatusDiagnostic g_valve_status_publish[7];
#define BASE UINT64_C(1794183600000)
static SedsRouter router_cookie;
static struct {SedsRouter*r;uint8_t created;} g_router={&router_cookie,1};
static ULONG ticks;
static ULONG tx_time_get(void){return ticks;}
/* Modeled clocks, as offsets from true time BASE+ticks (1 tick = 1 ms), and
 * their epochs. The GS remembers its last step to translate older stamps. */
static int64_t gs_offset,net_offset,last_step;static uint16_t gs_epoch,net_epoch;
static uint64_t gs_now(void){return (uint64_t)((int64_t)(BASE+ticks)+gs_offset);}
static uint64_t net_now(void){return (uint64_t)((int64_t)(BASE+ticks)+net_offset);}
static SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){assert(r==&router_cookie);*out=net_now();return SEDS_OK;}
static void telemetry_event_clock_observe(uint64_t utc,uint32_t tick){(void)utc;(void)tick;}
static bool telemetry_router_enter(void){return true;}
static bool telemetry_router_leave(void){return true;}
static volatile uint32_t g_telemetry_timesync_valid,g_telemetry_discovery_seen=1,g_telemetry_network_ready;
/* GS admission with the core age API: a report of the previous epoch is
 * translated across the step; refused more than 100 ms ahead or 2 s old. */
static ULONG last_report[7];static ULONG max_gap,gap_from;static unsigned reports,refused;
static SedsResult seds_router_log_typed_ex(SedsRouter*r,int type,const void*data,size_t count,size_t width,int kind,const uint64_t*stamp,bool queue){
 assert(r==&router_cookie&&type==SEDS_DT_UMBILICAL_STATUS&&count==2&&width==1&&kind==SEDS_EK_UNSIGNED&&stamp&&queue);
 /* The stamp is the router's own network clock (the SDK adds its epoch). */
 assert(*stamp==net_now());
 const uint8_t id=((const uint8_t*)data)[0];assert(id<7);
 if(ticks>=gap_from&&last_report[id]&&ticks-last_report[id]>max_gap)max_gap=ticks-last_report[id];
 last_report[id]=ticks;reports++;
 int64_t translated=(int64_t)*stamp;
 if(net_epoch+1U==gs_epoch)translated+=last_step;else assert(net_epoch==gs_epoch);
 const int64_t skew=translated-(int64_t)gs_now();
 if(skew>100||skew<-2000)refused++;
 return SEDS_OK;}
static uint8_t snapshot(uint8_t id,uint8_t*state){(void)id;*state=0U;return 1U;}
static uint8_t g_pilot_valve_state,g_no_servo_open_state,g_nitrous_servo_open_state,g_launch_sequence_active;
"""
    pieces = [
        scaffold,
        *(function(telemetry, n) for n in ("telemetry_update_network_health",
                                             "telemetry_report_utc_locked",
                                             "telemetry_publish_umbilical_snapshot_locked",
                                             "telemetry_submit_umbilical_snapshot")),
        retry, period, reporter,
    ]
    return "\n".join(pieces) + MAIN


MAIN = r"""
/* One case: the GS clock steps by delta_ms at 60 s into a new epoch; the
 * board follows at its next announce (every 2 s); then run 20 minutes. */
static void run(int64_t delta){
 const ULONG step_at=60000U,end=step_at+20U*60000U;
 status_report_bind_snapshot(snapshot);
 for(ticks=1;ticks<end;ticks++){
  (void)telemetry_update_network_health(&router_cookie);
  if(ticks==step_at){gs_offset+=delta;last_step=delta;gs_epoch++;gap_from=step_at;}
  if(ticks%2000U==37U&&net_epoch!=gs_epoch){net_epoch=gs_epoch;net_offset=gs_offset;}
  publish_all_umbilical_statuses();
  retry_pending_status_reports();
  if(ticks==step_at-1U)assert(reports>=4U*50U);
 }
 printf("gs %+lld ms: reports=%u max_gap=%lu ms refused=%u\n",(long long)delta,reports,max_gap,refused);
 /* Never a stall: every valve key keeps its cadence through the step. */
 assert(max_gap<=1000U+100U);
 assert(reports>=4U*(end/1000U-2U));
 /* Judged by age across the step, no report is ever refused. */
 assert(refused==0U);
 assert(net_epoch==gs_epoch&&net_offset==gs_offset);
}
int main(int argc,char**argv){assert(argc==2);run(atoll(argv[1]));return 0;}
"""


class TimeStepReports(unittest.TestCase):
    def test_reports_keep_cadence_and_admission_across_network_time_steps(self):
        with tempfile.TemporaryDirectory() as directory:
            exe = Path(directory) / "time-step"
            build = subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                                    "-Wno-unused-function", "-fsanitize=address,undefined",
                                    "-I", str(ROOT / "Core/Inc"), "-x", "c", "-", "-o", str(exe)],
                                   input=harness(), text=True, capture_output=True)
            self.assertEqual(build.returncode, 0, build.stderr)
            for delta in (-600000, 600000, -4 * 3600000, 38 * 360000):
                with self.subTest(step_ms=delta):
                    run = subprocess.run([str(exe), str(delta)], text=True,
                                         capture_output=True, timeout=120)
                    self.assertEqual(run.returncode, 0, run.stdout + run.stderr)


if __name__ == "__main__":
    unittest.main()
