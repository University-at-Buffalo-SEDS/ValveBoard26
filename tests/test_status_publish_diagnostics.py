import pathlib
import re
import subprocess
import tempfile
import unittest

from checked_router_fixture import checked_hooks
from router_lifecycle_fixture import function as actual_function
ROOT = pathlib.Path(__file__).resolve().parents[1]

class StatusPublishDiagnostics(unittest.TestCase):
    def test_raw_and_transport_frame_classification(self):
        code = r"""
#include <assert.h>
#include "status_frame_probe.h"
int main(void) {
  uint8_t raw[] = {0, 1, 42};
  uint8_t full[] = {83,68,84,1,129,1,0,1,42};
  uint8_t compact[] = {83,68,84,2,1,42};
  uint8_t overflow[] = {0,1,255,255,255,255,127};
  assert(status_frame_type(raw,sizeof(raw)) == 42);
  assert(status_frame_type(full,sizeof(full)) == 42);
  assert(status_frame_type(compact,sizeof(compact)) == UINT32_MAX);
  for(unsigned kind=2; kind<=5; kind++) {
    compact[3]=kind;
    assert(status_frame_type(compact,sizeof(compact)) == UINT32_MAX);
  }
  assert(status_frame_type(overflow,sizeof(overflow)) == UINT32_MAX);
  assert(status_frame_type(NULL,0) == UINT32_MAX);
  for(unsigned n=0; n<sizeof(full); n++)
    assert(status_frame_type(full,n) == UINT32_MAX);
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            exe = str(pathlib.Path(tmp) / "frames")
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-I", str(ROOT / "Core/Inc"), "-x", "c", "-", "-o", exe],
                           input=code, text=True, check=True)
            subprocess.run([exe], check=True)

    def test_deferred_is_not_acceptance_and_real_refusal_remains_counted(self):
        from status_submit_fixture import run
        run(r"""
int main(void){
 SedsResult result=999;
 fixture_get_failure=1;
 assert(telemetry_submit_umbilical_status(2,1,&result)==TELEMETRY_STATUS_REFUSED&&result==SEDS_BAD_ARG&&!sdk_calls);
 fixture_get_failure=0;g_router.r=0;g_router.created=0;
 assert(telemetry_submit_umbilical_status(2,1,&result)==TELEMETRY_STATUS_DEFERRED&&result==SEDS_IO&&!sdk_calls&&!clock_reads);
 g_router.r=(void*)1;g_router.created=1;utc=0;
 assert(telemetry_submit_umbilical_status(2,1,&result)==TELEMETRY_STATUS_DEFERRED&&!sdk_calls&&!g_sim_umbilical_status_fail&&!g_valve_status_publish[2].calls);
 utc=1000;change_clock_after_read=2000;
 assert(telemetry_publish_umbilical_status(2,1)==0&&last_stamp==1000&&utc==2000);
 change_clock_after_read=0;sdk_result=-7;
 unsigned before=sdk_calls;assert(telemetry_publish_umbilical_status(2,0)==-7&&sdk_calls==before+1);
 assert(g_valve_status_publish[2].calls==2&&g_valve_status_publish[2].failed==1&&g_sim_umbilical_status_fail==1&&g_valve_status_publish[2].last_result==-7&&g_valve_status_publish[2].last_state==0);
 sdk_result=0;assert(telemetry_publish_umbilical_status(255,1)==0);
 fixture_put_failure=1;assert(telemetry_publish_umbilical_status(1,0)==SEDS_ERR);fixture_put_failure=0;
 return 0;
}
""")
