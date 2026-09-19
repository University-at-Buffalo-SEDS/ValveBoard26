"""Exercise the production abort reporter with a rejecting transport queue."""
import pathlib
import re
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class AbortReportingTests(unittest.TestCase):
    def test_retry_local_abort_without_echo_or_flood(self):
        source = (ROOT / "Core/Src/telemetry_thread.c").read_text()
        function = re.search(
            r"static void telemetry_report_safety_abort\(void\)\s*\{.*?\n\}",
            source, re.S).group()
        code = r"""
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "safety_diagnostics.h"
typedef uint32_t ULONG;
#define TX_TIMER_TICKS_PER_SECOND 1000U
#define SEDS_DT_ABORT 9
#define SEDS_OK 0
volatile safety_abort_diagnostic_t g_safety_first_abort;
static ULONG ticks;
static unsigned calls;
static int result = -1;
static ULONG tx_time_get(void) { return ticks; }
static int log_telemetry_string_asynchronous(int type, const char *reason) {
    assert(type == SEDS_DT_ABORT);
    assert(strstr(reason, "heartbeat timeout") != 0);
    calls++;
    return result;
}
""" + function + r"""
int main(void) {
    /* A received abort has no locally recorded trigger: no echo. */
    telemetry_report_safety_abort(); assert(calls == 0);
    g_safety_first_abort.reason = SAFETY_ABORT_HEARTBEAT;
    ticks = UINT32_MAX - 500;
    telemetry_report_safety_abort(); assert(calls == 1);
    ticks += 999;
    telemetry_report_safety_abort(); assert(calls == 1);
    ticks++;
    result = SEDS_OK;
    telemetry_report_safety_abort(); assert(calls == 2);
    ticks += 10000;
    telemetry_report_safety_abort(); assert(calls == 2);
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            binary = str(pathlib.Path(tmp) / "abort-report")
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-I", str(ROOT / "Core/Inc"), "-x", "c", "-",
                            "-o", binary], input=code, text=True, check=True)
            subprocess.run([binary], check=True)
