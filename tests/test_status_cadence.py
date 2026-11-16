"""Actual periodic status reporters must stay within the GSE freshness window."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

# The GroundStation treats a valve as confirmed only while its newest admitted
# report is at most this old, measured from the board's source timestamp
# (backend state.rs UMBILICAL_PENDING_VALVE_TIMEOUT, gse_sequence `<= 2000`).
GS_FRESH_MS = 2000
# Age a report has already used when the GS admits it: CAN -> gateway ->
# Pico-Fi Wi-Fi -> GS transport plus any board/GS network clock disagreement
# (the GS counts a board clock that lags its own as report age).
TRANSPORT_AND_CLOCK_AGE_MS = 500


def reporter_source():
    actuator = (ROOT / "Core/Src/main_task.c").exists()
    source = (ROOT / ("Core/Src/main_task.c" if actuator else "Core/Src/main_thread.c")).read_text()
    period = re.search(r"^#define UMBILICAL_STATUS_PERIOD_TICKS .*$", source, re.M).group(0)
    if actuator:
        a = source.index("static void publish_umbilical_statuses_if_due(")
        body = source[a:source.index("static ULONG ms_to_ticks", a)]
    else:
        a = source.index("static void publish_all_umbilical_statuses(")
        body = source[a:source.index("static void publish_abort_umbilical_statuses", a)]
    return actuator, period, body


def compile_and_run(program, ticks, directory):
    executable = Path(directory) / "status-cadence"
    subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-fsanitize=address,undefined", f"-DTX_TIMER_TICKS_PER_SECOND={ticks}U",
                    "-x", "c", "-", "-o", str(executable)],
                   input=program, text=True, check=True)
    return subprocess.run([str(executable)], check=True, text=True, capture_output=True).stdout


class StatusCadenceTests(unittest.TestCase):
    def test_periodic_refresh_without_tick_rate_flooding(self):
        actuator, period, body = reporter_source()
        scaffold = r"""
#include <assert.h>
#include <stdint.h>
typedef uint32_t ULONG;
static ULONG clock_ticks;
static unsigned reports;
static ULONG tx_time_get(void) { return clock_ticks; }
""" + period + "\n#define P ((ULONG)(UMBILICAL_STATUS_PERIOD_TICKS))\n"
        if actuator:
            scaffold += "static void publish_all_umbilical_statuses(void) { reports++; }\n"
            main = r"""
int main(void) {
 assert(P >= TX_TIMER_TICKS_PER_SECOND / 10U);
 ULONG last=0;
 clock_ticks=P-1; publish_umbilical_statuses_if_due(&last); assert(reports==0);
 clock_ticks++; publish_umbilical_statuses_if_due(&last); assert(reports==1 && last==clock_ticks);
 publish_umbilical_statuses_if_due(&last); assert(reports==1);
 clock_ticks=2*P-1; publish_umbilical_statuses_if_due(&last); assert(reports==1);
 clock_ticks++; publish_umbilical_statuses_if_due(&last); assert(reports==2);
 last=UINT32_MAX-P/2;
 clock_ticks=P-1-P/2;
 publish_umbilical_statuses_if_due(&last); assert(reports==3);
}
"""
        else:
            scaffold += r"""
enum { CMD_PILOT_OPEN=1, CMD_NORMALLY_OPEN_OPEN, CMD_DUMP_OPEN, CMD_SEQUENCE };
static uint8_t g_pilot_valve_state,g_no_servo_open_state,g_nitrous_servo_open_state,g_launch_sequence_active;
static int publish_umbilical_status(unsigned command, uint8_t value) {
 assert(command>=1 && command<=4 && value==0); reports++; return 0;
}
"""
            main = r"""
int main(void) {
 assert(P >= TX_TIMER_TICKS_PER_SECOND / 10U);
 publish_all_umbilical_statuses(); assert(reports==4);
 clock_ticks=P-1; publish_all_umbilical_statuses(); assert(reports==4);
 clock_ticks++; publish_all_umbilical_statuses(); assert(reports==8);
 publish_all_umbilical_statuses(); assert(reports==8);
 clock_ticks=2*P-1; publish_all_umbilical_statuses(); assert(reports==8);
 clock_ticks++; publish_all_umbilical_statuses(); assert(reports==12);
 clock_ticks=UINT32_MAX-P/2; publish_all_umbilical_statuses(); assert(reports==16);
 clock_ticks=P-1-P/2;
 publish_all_umbilical_statuses(); assert(reports==20);
}
"""
        for ticks in (100, 1000):
            with self.subTest(ticks_per_second=ticks), tempfile.TemporaryDirectory() as directory:
                compile_and_run(scaffold + body + main, ticks, directory)

    def test_one_lost_report_does_not_expire_station_confirmation(self):
        """Field symptom: fill valves flickered "Unconfirmed" on Actions,
        Dashboard and Data. At a 1 s cadence two report periods already equal
        the GS's 2 s window, so a single lost, late or refused report always
        let the confirmation lapse until the following report arrived.

        The actual reporter runs tick by tick (the control loop sleeps one
        tick); each report cycle in turn is dropped, and the GS freshness rule
        is applied to what is left."""
        actuator, period, body = reporter_source()
        program = r"""
#include <stdint.h>
#include <stdio.h>
typedef uint32_t ULONG;
static ULONG clock_ticks;
static ULONG tx_time_get(void) { return clock_ticks; }
""" + period + "\n"
        if actuator:
            program += r"""
static void publish_all_umbilical_statuses(void) { printf("%lu\n", (unsigned long)clock_ticks); }
""" + body + r"""
int main(void) {
 ULONG last=0;
 publish_all_umbilical_statuses();
 for (clock_ticks=0; clock_ticks<30U*TX_TIMER_TICKS_PER_SECOND; clock_ticks++) publish_umbilical_statuses_if_due(&last);
 return 0;
}
"""
        else:
            program += r"""
enum { CMD_PILOT_OPEN=1, CMD_NORMALLY_OPEN_OPEN, CMD_DUMP_OPEN, CMD_SEQUENCE };
static uint8_t g_pilot_valve_state,g_no_servo_open_state,g_nitrous_servo_open_state,g_launch_sequence_active;
static int publish_umbilical_status(unsigned command, uint8_t value) {
 (void)value; if (command==CMD_PILOT_OPEN) printf("%lu\n", (unsigned long)clock_ticks); return 0;
}
""" + body + r"""
int main(void) {
 for (clock_ticks=0; clock_ticks<30U*TX_TIMER_TICKS_PER_SECOND; clock_ticks++) publish_all_umbilical_statuses();
 return 0;
}
"""
        for ticks in (100, 1000):
            with self.subTest(ticks_per_second=ticks), tempfile.TemporaryDirectory() as directory:
                out = compile_and_run(program, ticks, directory)
                sent_ms = [int(line) * 1000 // ticks for line in out.split()]
                self.assertGreater(len(sent_ms), 20)
                end_ms = sent_ms[-1]
                for lost in range(1, len(sent_ms) - 2):
                    kept = sent_ms[:lost] + sent_ms[lost + 1:]
                    # Evidence lapses between a report's expiry (source + 2 s)
                    # and the receipt of the next admitted report.
                    for source, following in zip(kept, kept[1:]):
                        if following > end_ms:
                            break
                        received = following + TRANSPORT_AND_CLOCK_AGE_MS
                        self.assertLessEqual(
                            received - source, GS_FRESH_MS,
                            f"report sent at {following} ms (after losing the one at {sent_ms[lost]} ms) "
                            f"reaches the GS {received - source} ms after the previous capture; "
                            f"confirmation lapses for {received - source - GS_FRESH_MS} ms")


if __name__ == "__main__":
    unittest.main()
