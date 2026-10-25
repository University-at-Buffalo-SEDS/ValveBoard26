"""Actual periodic status reporters must stay within the GSE freshness window."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class StatusCadenceTests(unittest.TestCase):
    def test_one_second_refresh_without_tick_rate_flooding(self):
        actuator = (ROOT / "Core/Src/main_task.c").exists()
        source = (ROOT / ("Core/Src/main_task.c" if actuator else "Core/Src/main_thread.c")).read_text()
        period = re.search(r"^#define UMBILICAL_STATUS_PERIOD_TICKS .*$", source, re.M).group(0)
        if actuator:
            a = source.index("static void publish_umbilical_statuses_if_due(")
            body = source[a:source.index("static ULONG ms_to_ticks", a)]
        else:
            a = source.index("static void publish_all_umbilical_statuses(")
            body = source[a:source.index("static void publish_abort_umbilical_statuses", a)]
        scaffold = r"""
#include <assert.h>
#include <stdint.h>
typedef uint32_t ULONG;
static ULONG clock_ticks;
static unsigned reports;
static ULONG tx_time_get(void) { return clock_ticks; }
""" + period + "\n"
        if actuator:
            scaffold += "static void publish_all_umbilical_statuses(void) { reports++; }\n"
            main = r"""
int main(void) {
 ULONG last=0;
 clock_ticks=TX_TIMER_TICKS_PER_SECOND-1; publish_umbilical_statuses_if_due(&last); assert(reports==0);
 clock_ticks++; publish_umbilical_statuses_if_due(&last); assert(reports==1 && last==clock_ticks);
 publish_umbilical_statuses_if_due(&last); assert(reports==1);
 clock_ticks=2*TX_TIMER_TICKS_PER_SECOND-1; publish_umbilical_statuses_if_due(&last); assert(reports==1);
 clock_ticks++; publish_umbilical_statuses_if_due(&last); assert(reports==2);
 last=UINT32_MAX-TX_TIMER_TICKS_PER_SECOND/2;
 clock_ticks=TX_TIMER_TICKS_PER_SECOND-1-TX_TIMER_TICKS_PER_SECOND/2;
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
 publish_all_umbilical_statuses(); assert(reports==4);
 clock_ticks=TX_TIMER_TICKS_PER_SECOND-1; publish_all_umbilical_statuses(); assert(reports==4);
 clock_ticks++; publish_all_umbilical_statuses(); assert(reports==8);
 publish_all_umbilical_statuses(); assert(reports==8);
 clock_ticks=2*TX_TIMER_TICKS_PER_SECOND-1; publish_all_umbilical_statuses(); assert(reports==8);
 clock_ticks++; publish_all_umbilical_statuses(); assert(reports==12);
 clock_ticks=UINT32_MAX-TX_TIMER_TICKS_PER_SECOND/2; publish_all_umbilical_statuses(); assert(reports==16);
 clock_ticks=TX_TIMER_TICKS_PER_SECOND-1-TX_TIMER_TICKS_PER_SECOND/2;
 publish_all_umbilical_statuses(); assert(reports==20);
}
"""
        for ticks in (100, 1000):
            with self.subTest(ticks_per_second=ticks), tempfile.TemporaryDirectory() as directory:
                executable = Path(directory) / "status-cadence"
                subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                                "-fsanitize=address,undefined", f"-DTX_TIMER_TICKS_PER_SECOND={ticks}U",
                                "-x", "c", "-", "-o", str(executable)],
                               input=scaffold+body+main, text=True, check=True)
                subprocess.run([str(executable)], check=True)
