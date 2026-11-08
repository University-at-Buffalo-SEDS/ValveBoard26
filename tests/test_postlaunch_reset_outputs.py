"""Actual boot outputs and storage safe-off guard after a reset following Launch.

A watchdog, brownout or NRST reset keeps the TAMP launch marker. Opening Vent
and Dump then would drain the oxidiser tank of a burning motor; only an
explicit Stop or operator valve command may do that after Launch.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from test_launch_abort_policy import function

ROOT = Path(__file__).resolve().parents[1]

STUB = r"""
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#define RCC_APB1ENR1_PWREN 1U
#define RCC_APB1ENR1_RTCAPBEN 2U
#define PWR_CR1_DBP 4U
#define __DSB() ((void)0)
static struct {uint32_t APB1ENR1;} rcc;
static struct {uint32_t CR1;} pwr;
static struct {uint32_t BKP28R, BKP29R, BKP30R, BKP31R;} tamp;
#define RCC (&rcc)
#define PWR (&pwr)
#define TAMP (&tamp)
"""

HARNESS = r"""
typedef unsigned long ULONG;
static uint8_t g_pilot_valve_state, g_no_servo_open_state, g_nitrous_servo_open_state;
static uint8_t g_outputs_initialized, g_launch_sequence_active, g_launch_sequence_actuator_started;
static uint8_t g_launch_sequence_pilot_opened;
static uint32_t g_launch_sequence_pilot_open_ticks, g_sim_pilot_valve_state;
static int pilot_solenoid;
static bool storage_ready, continuity;
enum { CMD_PILOT_OPEN = 0, CMD_NORMALLY_OPEN_OPEN = 1, CMD_DUMP_OPEN = 2 };
#define NO_SERVO_TIMER 0
#define NO_SERVO_CHANNEL 0
#define DUMP_SERVO_TIMER 0
#define DUMP_SERVO_CHANNEL 0
static void HAL_TIM_PWM_Start(int t, int c) { (void)t; (void)c; }
static void solenoidInit(int *p) { (void)p; }
static void solenoidOff(int *p) { (void)p; }
static void no_servo_open(void) { g_no_servo_open_state = 1U; }
static void no_servo_close(void) { g_no_servo_open_state = 0U; }
static void dump_servo_open(void) { g_nitrous_servo_open_state = 1U; }
static void dump_servo_close(void) { g_nitrous_servo_open_state = 0U; }
static int publish_umbilical_status(int id, uint8_t s) { (void)id; (void)s; return 0; }
static bool persistent_store_controls_ready(void) { return storage_ready; }
static uint8_t thread_comm_launch_continuity(void) { return continuity; }
/* Real thread_comm replays the retained Stop at bind time. */
static int thread_comm_bind_manual_stop_apply(void (*apply)(void)) {
  if (board_abort_latch_is_set()) apply();
  return 0;
}
/*ACTUAL_HELPERS*/
static void boot(ULONG initial_input) /*ACTUAL_PROLOGUE*/
static void loop_guard(void) { /*ACTUAL_GUARD*/ }
static void reset_with(bool launched, bool aborted) {
  memset(&tamp, 0, sizeof tamp);
  if (launched) { tamp.BKP28R = UINT32_C(0x4c41554e); tamp.BKP29R = ~UINT32_C(0x4c41554e); }
  if (aborted) { tamp.BKP30R = UINT32_C(0x41425254); tamp.BKP31R = ~UINT32_C(0x41425254); }
  g_no_servo_open_state = g_nitrous_servo_open_state = 0U; g_pilot_valve_state = 1U;
  g_outputs_initialized = 0U; storage_ready = false; continuity = false;
  board_abort_latch_init();
  boot(0UL);
}
int main(void) {
  /* Pad: no launch marker, the defaults still vent and dump. */
  reset_with(false, false);
  assert(g_no_servo_open_state && g_nitrous_servo_open_state && !g_pilot_valve_state);
  loop_guard(); assert(g_no_servo_open_state && g_nitrous_servo_open_state);
  /* Explicit Stop after Launch keeps venting and dumping. */
  reset_with(true, true);
  assert(g_no_servo_open_state && g_nitrous_servo_open_state && !g_pilot_valve_state);
  /* Reset after an accepted, unaborted Launch (storage not ready yet). */
  reset_with(true, false);
  assert(!g_pilot_valve_state);
  assert(!g_no_servo_open_state && !g_nitrous_servo_open_state);
  loop_guard();
  assert(!g_no_servo_open_state && !g_nitrous_servo_open_state);
  return 0;
}
"""


class PostlaunchResetOutputTests(unittest.TestCase):
    def test_reset_after_launch_does_not_vent_or_dump(self):
        main = (ROOT / 'Core/Src/main_thread.c').read_text()
        latch = '\n'.join(line for line in (ROOT / 'Core/Src/board_abort_latch.c').read_text().splitlines()
                          if not line.startswith('#include'))
        entry = function(main, 'main_thread_entry')
        prologue = entry[entry.index('{'):entry.index('    thread_comm_msg_t msg;')] + '}'
        guard = re.search(r'^\s*if \(!persistent_store_controls_ready\(\)[^\n]*main_thread_force_outputs_safe_off\(\);',
                          entry, re.M).group(0)
        names = ['manual_stop_apply_locked', 'main_thread_force_outputs_safe_off', 'pilot_valve_off',
                 'nitrous_valve_open', 'normally_open_valve_open']
        names += [n for n in ['post_launch_reset_hold', 'nitrous_valve_close', 'normally_open_valve_close']
                  if re.search(r'^[^\n;]*\b' + n + r'\(void\)\s*\{', main, re.M)]
        helpers = '\n'.join(function(main, n) for n in names)
        source = STUB + latch + HARNESS.replace('/*ACTUAL_HELPERS*/', helpers) \
            .replace('/*ACTUAL_PROLOGUE*/', prologue).replace('/*ACTUAL_GUARD*/', guard)
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / 'test.c'
            exe = Path(tmp) / 'test'
            src.write_text(source)
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
                            '-fsanitize=address,undefined', str(src), '-o', str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
