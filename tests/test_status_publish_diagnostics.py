import pathlib
import re
import subprocess
import tempfile
import unittest

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

    def test_production_publish_acceptance_and_retry_failure(self):
        source = (ROOT / "Core/Src/telemetry.c").read_text()
        function = re.search(r"SedsResult telemetry_publish_umbilical_status\([^)]*\)\s*\{.*?\n\}", source, re.S).group()
        declaration = re.search(r"typedef struct \{.*?\} ValveStatusDiagnostic;", source, re.S).group()
        code = r"""
#include <assert.h>
#include <stdint.h>
#define TELEMETRY_ENABLED
#define SEDS_OK 0
#define SEDS_ERR -1
#define SEDS_DT_UMBILICAL_STATUS 42
#define SEDS_EK_UNSIGNED 0
typedef int SedsResult;
static struct { void *r; } g_router = {(void *)1};
static unsigned tick=100, sends, sleeps;
static int failures;
static unsigned tx_time_get(void) { return tick; }
static void tx_thread_sleep(unsigned n) { sleeps+=n; tick+=n; }
static int init_telemetry_router(void) { return -1; }
static int seds_router_log_typed(void *r, int ty, const uint8_t *p,
                                 unsigned n, unsigned size, int kind) {
  (void)r; (void)kind; assert(ty==42 && n==2 && size==1 && p[1]<=1);
  sends++; if(failures>0) {failures--; return -7;} return 0;
}
""" + declaration + """
volatile ValveStatusDiagnostic g_valve_status_publish[7];
""" + function + r"""
int main(void) {
  assert(telemetry_publish_umbilical_status(2,1)==0);
  assert(g_valve_status_publish[2].accepted==1);
  assert(g_valve_status_publish[2].last_state==1);
  failures=3;
  assert(telemetry_publish_umbilical_status(2,0)==-7);
  assert(sends==4 && sleeps==3);
  assert(g_valve_status_publish[2].calls==2);
  assert(g_valve_status_publish[2].failed==1);
  assert(g_valve_status_publish[2].last_state==0);
  assert(g_valve_status_publish[2].last_result==-7);
  assert(g_valve_status_publish[2].last_tick==103);
  failures=1;
  assert(telemetry_publish_umbilical_status(1,0)==0);
  assert(g_valve_status_publish[1].accepted==1);
  assert(g_valve_status_publish[1].failed==0);
  assert(telemetry_publish_umbilical_status(255,0)==0);
  g_router.r=0;
  assert(telemetry_publish_umbilical_status(2,0)==-1);
  assert(g_valve_status_publish[2].failed==2);
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            exe = str(pathlib.Path(tmp) / "status")
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-x", "c", "-", "-o", exe], input=code, text=True, check=True)
            subprocess.run([exe], check=True)
