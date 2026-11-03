"""Exercise the production state publisher without any protocol ACK arriving."""
import pathlib
import re
import subprocess
import tempfile
import unittest

from checked_router_fixture import checked_hooks
from router_lifecycle_fixture import function as actual_function
ROOT = pathlib.Path(__file__).resolve().parents[1]


class ImmediateStateReportTests(unittest.TestCase):
    def test_servo_actions_report_state_immediately(self):
        source = (ROOT / "Core/Src/main_thread.c").read_text()
        from test_abort_owner_notification import function
        handle=function(source,'handle_command')
        marker='if(cmd==CMD_PILOT_OPEN||'
        start=handle.index(marker);brace=handle.index('{',start);end=brace+1;depth=1
        while depth:
            depth+=(handle[end]=='{')-(handle[end]=='}');end+=1
        handlers=function(source,'apply_automatic_valve')+'\n'+function(source,'apply_manual_valve_locked')
        publication=handle[start:end]
        code = r"""
#include <assert.h>
#include <stdint.h>
enum { CMD_PILOT_OPEN=0,CMD_DUMP_OPEN=2,CMD_NORMALLY_OPEN_OPEN=1,CMD_PILOT_CLOSE=3,CMD_DUMP_CLOSE=4,CMD_NORMALLY_OPEN_CLOSE=5 };
static uint8_t g_nitrous_servo_open_state, g_no_servo_open_state;
static int output, output_id, reports,pilot_solenoid;
static int32_t g_sim_pilot_solenoid_result;static uint8_t g_pilot_valve_state,g_sim_pilot_valve_state;
static int solenoidOn(int*p){(void)p;return 0;}static void solenoidOff(int*p){(void)p;}
static unsigned thread_comm_manual_effect_owned(uint64_t t,uint32_t e,void(*apply)(void*),void*p){assert(!t&&e==7);apply(p);return 1;}
static void dump_servo_open(void) { output=1; output_id=2; }
static void dump_servo_close(void) { output=0; output_id=2; }
static void no_servo_open(void) { output=1; output_id=1; }
static void no_servo_close(void) { output=0; output_id=1; }
static int publish_umbilical_status(int id, int state) {
    assert(output_id==id && output==state); reports++; return 0;
}
""" + handlers + "\nstatic void command(uint8_t cmd){uint32_t command_epoch=7;const uint8_t valve=cmd;" + publication + "}\n" + r"""
int main(void) {
    command(CMD_DUMP_OPEN); assert(reports==1 && output==1);
    command(CMD_DUMP_CLOSE); assert(reports==2 && output==0);
    command(CMD_NORMALLY_OPEN_OPEN); assert(reports==3 && output==1);
    command(CMD_NORMALLY_OPEN_CLOSE); assert(reports==4 && output==0);
    command(CMD_NORMALLY_OPEN_CLOSE); assert(reports==5 && output==0);
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            binary = str(pathlib.Path(tmp) / "servo-state")
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-x", "c", "-", "-o", binary],
                           input=code, text=True, check=True)
            subprocess.run([binary], check=True)

    def test_reports_both_states_into_retained_queue_without_claiming_remote_ack(self):
        from status_submit_fixture import run
        run(r"""
int main(void){
 for(int id=0;id<7;id++){
  unsigned before=sdk_calls;assert(telemetry_publish_umbilical_status(id,1)==SEDS_OK);
  assert(sdk_calls==before+1&&last_id==(unsigned)id&&last_state==1&&last_stamp==1000);
  assert(telemetry_publish_umbilical_status(id,0)==SEDS_OK);
  assert(sdk_calls==before+2&&last_id==(unsigned)id&&last_state==0);
 }
 sdk_result=SEDS_ERR;unsigned before=sdk_calls;
 assert(telemetry_publish_umbilical_status(2,0)==SEDS_ERR&&sdk_calls==before+1);
 return 0;
}
""")
