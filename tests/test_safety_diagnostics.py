"""Compile the production safety checks with inert time, GPIO and abort mocks."""
import pathlib
import re
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class SafetyDiagnosticsTests(unittest.TestCase):
    def test_first_cause_and_heartbeat_deadline(self):
        valve = (ROOT / "Core/Src/safety_thread.c").exists()
        path = "safety_thread.c" if valve else "safety_task.c"
        source = (ROOT / "Core/Src" / path).read_text()
        names = ["safety_check_heartbeat"]
        if valve:
            names = ["safety_request_abort", "safety_check_heartbeat", "safety_check_continuity"]
        functions = "\n".join(re.search(
            r"static void " + name + r"\([^)]*\)\s*\{.*?\n\}",
            source, re.S).group() for name in names)
        code = r"""
#include <assert.h>
#include <stdint.h>
#include "safety_diagnostics.h"
volatile safety_abort_diagnostic_t g_safety_first_abort;
static uint64_t now_ms, heartbeat, safety_heartbeat_missing_since_ms;
static uint32_t aborted, allowed = 1, calls;
static uint32_t safety_heartbeat_timeout_count;
#define SAFETY_HEARTBEAT_TIMEOUT_MS 5000ULL
static uint64_t safety_now_ms(void) { return now_ms; }
static uint64_t thread_comm_get_groundstation_heartbeat_ms(void) { return heartbeat; }
static uint32_t thread_comm_get_abort(void) { return aborted; }
static uint32_t thread_comm_abort_allowed(void) { return allowed; }
static uint32_t thread_comm_get_flight_state(void) { return 3; }
static int thread_comm_set_abort(uint32_t value) { aborted=value; return 0; }
"""
        if valve:
            code += r"""
typedef uint32_t GPIO_PinState;
#define Continuity_GPIO_Port 0
#define Continuity_Pin 0
#define CONTINUITY_PRESENT_STATE 1
#define VALVE_FLIGHT_STATE_LAUNCH 7
#define SAFETY_LAUNCH_CONTINUITY_TIMEOUT_MS 300000ULL
#define CMD_ABORT 0
#define TX_NO_WAIT 0
static uint32_t continuity=1, safety_last_continuity_state;
static uint32_t safety_continuity_loss_count, safety_continuity_loss_latched_count;
static uint32_t safety_launch_continuity_timeout_count;
static uint64_t safety_launch_continuity_loss_since_ms;
static uint32_t HAL_GPIO_ReadPin(int p, int n) { (void)p; (void)n; return continuity; }
static int thread_comm_send(int cmd, int wait) { (void)cmd; (void)wait; calls++; return 0; }
static int telemetry_broadcast_abort(const char *msg) { assert(msg[0]); return 0; }
"""
        else:
            code += "static void main_task_force_outputs_safe_off(void) { calls++; }\n"
        code += functions + r"""
int main(void) {
    heartbeat=100; now_ms=5099;
    safety_check_heartbeat(); assert(!aborted && !g_safety_first_abort.reason);
    now_ms=5100; safety_check_heartbeat();
    assert(aborted && calls==1);
    assert(g_safety_first_abort.reason==SAFETY_ABORT_HEARTBEAT);
    assert(g_safety_first_abort.at_ms==5100);
    assert(g_safety_first_abort.last_heartbeat_ms==100);
    now_ms=9000; safety_check_heartbeat();
    assert(g_safety_first_abort.at_ms==5100);
    safety_record_first_abort(SAFETY_ABORT_ADC_START, 9999, 0, 0, 9, 0);
    assert(g_safety_first_abort.reason==SAFETY_ABORT_HEARTBEAT);
    g_safety_first_abort=(safety_abort_diagnostic_t){0};
    safety_record_first_abort(SAFETY_ABORT_ADC_START, 9999, 0, 0, 9, 1);
    assert(g_safety_first_abort.reason==0);
    aborted=0; heartbeat=0; now_ms=10000;
    safety_heartbeat_missing_since_ms=0;
    safety_check_heartbeat(); assert(!aborted);
    now_ms=15000; safety_check_heartbeat();
"""
        if valve:
            code += r"""
    assert(!aborted); /* Preserve Valve standalone boot policy. */
    continuity=0; safety_check_continuity();
    assert(aborted && g_safety_first_abort.reason==SAFETY_ABORT_CONTINUITY);
    assert(g_safety_first_abort.at_ms==15000 && g_safety_first_abort.detail==0);
    heartbeat=1; now_ms=20000; safety_check_heartbeat();
    assert(g_safety_first_abort.reason==SAFETY_ABORT_CONTINUITY);
"""
        else:
            code += r"""
    assert(aborted);
    assert(g_safety_first_abort.reason==SAFETY_ABORT_INITIAL_HEARTBEAT);
"""
        code += "}\n"
        with tempfile.TemporaryDirectory() as tmp:
            binary = str(pathlib.Path(tmp) / "safety-checks")
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-I", str(ROOT / "Core/Inc"), "-x", "c", "-", "-o", binary],
                           input=code, text=True, check=True)
            subprocess.run([binary], check=True)

