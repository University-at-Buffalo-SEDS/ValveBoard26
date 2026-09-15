"""Run production pilot handlers against inert driver and transport mocks."""
import pathlib
import re
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class ValveConfirmationTests(unittest.TestCase):
    def test_pilot_reports_success_rejection_and_repeated_close(self):
        source = (ROOT / 'Core/Src/main_thread.c').read_text()
        handlers = '\n'.join(re.search(r'void ' + name + r'\(void\).*?\n}',
                                      source, re.S).group()
                             for name in ('pilot_valve_on', 'pilot_valve_off'))
        code = r'''
#include <stdint.h>
#include <assert.h>
#define CMD_PILOT_OPEN 1
static int pilot_solenoid, rejected, count, reported;
static int32_t g_sim_pilot_solenoid_result;
static uint8_t g_pilot_valve_state, g_sim_pilot_valve_state;
static int solenoidOn(int *p) { (void)p; return rejected ? -1 : 0; }
static void solenoidOff(int *p) { (void)p; }
static int publish_umbilical_status(int cmd, int on) {
    assert(cmd==CMD_PILOT_OPEN); count++; reported=on; return 0;
}
''' + handlers + r'''
int main(void) {
    rejected=1; pilot_valve_on(); assert(count==1 && reported==0);
    rejected=0; pilot_valve_on(); assert(count==2 && reported==1);
    rejected=1; pilot_valve_on(); assert(count==3 && reported==1);
    pilot_valve_off(); assert(count==4 && reported==0);
    pilot_valve_off(); assert(count==5 && reported==0);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary = str(pathlib.Path(tmp) / 'pilot-confirmation')
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-x', 'c', '-', '-o', binary], input=code, text=True, check=True)
            subprocess.run([binary], check=True)
