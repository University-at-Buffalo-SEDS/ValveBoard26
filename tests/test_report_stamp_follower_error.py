"""Valve-state report stamps stay inside the GS confirmation budget while
the board's SEDSnet clock is off the GS clock.

The GS refuses a report stamped more than 100 ms ahead of its network clock
and confirms a valve only until 2 s after the report's stamp. A follower that
has fallen out of lock (for example while the station slews a large UTC
correction at 5 %, which a SEDSnet host model shows leaves fill-side
followers up to about 1 s behind for the first minute and unlocked with a
~1.1 s bound afterwards) used to stamp reports with its own clock whenever it
was within 2 s of the GS heartbeat. Every report then looked up to 2 s old,
so confirmations lapsed between reports or were refused as ahead.

The actual reporter, heartbeat and stamp C runs against a board clock held a
fixed offset from the GS (always valid, no restart) and a GS heartbeat every
500 ms that reaches the board 30 ms after it was stamped.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest
import test_time_step_reports as base

ROOT = Path(__file__).resolve().parents[1]

TRACK = r"""
static int64_t worst_lead=INT64_MIN,worst_age=INT64_MIN;static ULONG track_from=5000U;
static void track(int64_t skew){if(ticks<track_from)return;if(skew>worst_lead)worst_lead=skew;if(-skew>worst_age)worst_age=-skew;}
"""

MAIN = r"""
static void run(int64_t error){
 const ULONG end=120000U;ULONG last_status=0;(void)last_status;
 upstream_offset=error;net_offset=error;net_valid=true;
 for(ticks=1;ticks<end;ticks++){
  telemetry_update_network_health(&router_cookie);
  if(!net_valid){net_valid=true;net_offset=upstream_offset;}
  if(ticks%500U==0U){SedsPacketView hb={SEDS_DT_HEARTBEAT,"GS",2,gs_now()-30U};(void)Heartbeat_handler(&hb,NULL);}
  REPORTER
 }
 printf("board clock %+lld ms: reports=%u worst lead=%lld ms worst age=%lld ms refused=%u restarts=%u\n",
        (long long)error,reports,(long long)worst_lead,(long long)worst_age,refused,restarts);
 assert(reports>=100U);
 /* Never refused as ahead of the GS. */
 assert(worst_lead<=100);
 /* Age is the heartbeat transit, not the follower's error. */
 assert(worst_age<=100);
}
int main(int argc,char**argv){assert(argc==2);run(atoll(argv[1]));return 0;}
"""


def source():
    text = base.harness()
    hook = "if(ticks>=gap_from&&(skew>100||skew<-2000))refused++;"
    assert hook in text and base.MAIN in text
    text = text.replace(hook, hook + "track(skew);")
    first = text.index("static ULONG last_report[")
    text = text[:first] + TRACK + text[first:]
    return text.replace(base.MAIN, MAIN.replace("REPORTER", REPORTER))


REPORTER = "if(ticks==1U)status_report_bind_snapshot(snapshot);publish_all_umbilical_statuses();retry_pending_status_reports();"


class ReportStampFollowerError(unittest.TestCase):
    def test_reports_keep_their_confirmation_budget_when_the_board_clock_is_off(self):
        with tempfile.TemporaryDirectory() as directory:
            exe = Path(directory) / "stamp"
            build = subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                                    "-Wno-unused-function", "-fsanitize=address,undefined",
                                    "-I", str(ROOT / "Core/Inc"), "-x", "c", "-", "-o", str(exe)],
                                   input=source(), text=True, capture_output=True)
            self.assertEqual(build.returncode, 0, build.stderr)
            for error in (-1900, -1000, -400, -60, 0, 20, 150, 400, 1500):
                with self.subTest(board_minus_gs_ms=error):
                    run = subprocess.run([str(exe), str(error)], text=True,
                                         capture_output=True, timeout=120)
                    self.assertEqual(run.returncode, 0, run.stdout + run.stderr)


if __name__ == "__main__":
    unittest.main()
