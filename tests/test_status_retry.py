import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class StatusRetryTests(unittest.TestCase):
    def test_bounded_retry_latest_state_and_recovery(self):
        header = (ROOT / "Core/Inc/status_report_retry.h").read_text()
        header = "\n".join(line for line in header.splitlines()
                           if not line.startswith("#include"))
        code = r"""
#include <assert.h>
#include <stdint.h>
typedef uint32_t ULONG;
typedef int SedsResult;
#define SEDS_OK 0
#define TX_TIMER_TICKS_PER_SECOND 1000
static ULONG now;
static unsigned calls, last_id, last_state;
static int fail = 1;
static ULONG tx_time_get(void) { return now; }
static SedsResult telemetry_publish_umbilical_status(uint8_t id, uint8_t state) {
    calls++; last_id=id; last_state=state; return fail ? -14 : 0;
}
""" + header + r"""
int main(void) {
    assert(publish_umbilical_status(2,1)!=0 && calls==1);
    now=99; retry_pending_status_reports(); assert(calls==1);
    now=100; retry_pending_status_reports(); assert(calls==2);
    /* A close/safety transition supersedes a failed open, never replay it. */
    assert(publish_umbilical_status(2,0)!=0 && calls==3);
    now=200; fail=0; retry_pending_status_reports();
    assert(calls==4 && last_id==2 && last_state==0);
    assert(status_report_pending==0);
    now=300; retry_pending_status_reports(); assert(calls==4);
    assert(publish_umbilical_status(2,0)==0 && calls==5);
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            binary = str(pathlib.Path(tmp) / "status-retry")
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-x", "c", "-", "-o", binary],
                           input=code, text=True, check=True)
            subprocess.run([binary], check=True)
