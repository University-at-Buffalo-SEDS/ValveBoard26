"""Exercise the production state publisher without any protocol ACK arriving."""
import pathlib
import re
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class ImmediateStateReportTests(unittest.TestCase):
    def test_servo_actions_report_state_immediately(self):
        source = (ROOT / "Core/Src/main_thread.c").read_text()
        names = ("nitrous_valve_open", "nitrous_valve_close",
                 "normally_open_valve_open", "normally_open_valve_close")
        handlers = "\n".join(re.search(
            r"static void " + name + r"\(void\).*?\n}", source, re.S).group()
            for name in names)
        code = r"""
#include <assert.h>
#include <stdint.h>
enum { CMD_DUMP_OPEN=2, CMD_NORMALLY_OPEN_OPEN=1 };
static uint8_t g_nitrous_servo_open_state, g_no_servo_open_state;
static int output, output_id, reports;
static void dump_servo_open(void) { output=1; output_id=2; }
static void dump_servo_close(void) { output=0; output_id=2; }
static void no_servo_open(void) { output=1; output_id=1; }
static void no_servo_close(void) { output=0; output_id=1; }
static int publish_umbilical_status(int id, int state) {
    assert(output_id==id && output==state); reports++; return 0;
}
""" + handlers + r"""
int main(void) {
    nitrous_valve_open(); assert(reports==1 && output==1);
    nitrous_valve_close(); assert(reports==2 && output==0);
    normally_open_valve_open(); assert(reports==3 && output==1);
    normally_open_valve_close(); assert(reports==4 && output==0);
    normally_open_valve_close(); assert(reports==5 && output==0);
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            binary = str(pathlib.Path(tmp) / "servo-state")
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-x", "c", "-", "-o", binary],
                           input=code, text=True, check=True)
            subprocess.run([binary], check=True)

    def test_reports_open_and_closed_before_return_without_protocol_ack(self):
        source = (ROOT / "Core/Src/telemetry.c").read_text()
        publisher = re.search(
            r"SedsResult telemetry_publish_umbilical_status\(.*?\n}",
            source, re.S).group()
        code = r"""
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#define TELEMETRY_ENABLED 1
typedef int SedsResult;
enum { SEDS_OK=0, SEDS_ERR=-1, SEDS_DT_UMBILICAL_STATUS=7, SEDS_EK_UNSIGNED=0 };
uint32_t g_umbilical_status_attempts, g_umbilical_status_last_payload;
uint32_t g_umbilical_status_enqueue_failures;
int32_t g_umbilical_status_last_result;
typedef struct { uint32_t calls, accepted, failed, last_state, last_tick;
                 int32_t last_result; } ValveStatusDiagnostic;
volatile ValveStatusDiagnostic g_valve_status_publish[7];
static struct { void *r; } g_router = { (void *)1 };
static int calls, state, status_id, fail;
static unsigned tx_time_get(void) { return 42; }
static void tx_thread_sleep(unsigned ticks) { (void)ticks; }
static int init_telemetry_router(void) { return SEDS_OK; }
static int seds_router_log_typed(void *router, int type, const void *data,
                               size_t count, size_t size, int kind) {
    assert(router && type==SEDS_DT_UMBILICAL_STATUS);
    assert(count==2 && size==1 && kind==SEDS_EK_UNSIGNED);
    const uint8_t *p=data;
    calls++; status_id=p[0]; state=p[1];
    return fail ? SEDS_ERR : SEDS_OK;
}
static int log_telemetry_synchronous(int type, const void *data,
                                     size_t count, size_t size) {
    return seds_router_log_typed(g_router.r,type,data,count,size,SEDS_EK_UNSIGNED);
}
/* Deliberately no async queue or protocol ACK mock: neither may be required. */
""" + publisher + r"""
int main(void) {
    for (int id=0; id<7; ++id) {
        calls=0;
        assert(telemetry_publish_umbilical_status(id,1)==SEDS_OK);
        assert(calls==1 && status_id==id && state==1);
        assert(telemetry_publish_umbilical_status(id,0)==SEDS_OK);
        assert(calls==2 && status_id==id && state==0);
    }
    fail=1; calls=0;
    assert(telemetry_publish_umbilical_status(2,0)==SEDS_ERR);
    assert(calls>=1 && calls<=3);
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            binary = str(pathlib.Path(tmp) / "immediate-state")
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-Wno-unused-function", "-x", "c", "-", "-o", binary],
                           input=code, text=True, check=True)
            subprocess.run([binary], check=True)
