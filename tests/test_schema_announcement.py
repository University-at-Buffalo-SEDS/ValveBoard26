"""Exercise the production discovery poll against a failing announcement queue."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class SchemaAnnouncementTests(unittest.TestCase):
    def test_startup_retry_and_no_repeated_schema_flood(self):
        source = (ROOT / 'Core/Src/telemetry.c').read_text()
        start = source.index('SedsResult telemetry_poll_discovery(')
        end = source.index('\n}', start) + 2
        poll = source[start:end]
        code = r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#define SEDS_DT_DISCOVERY_SCHEMA_REQUEST 12U
#define TELEMETRY_ENABLED 1
typedef int SedsResult;
enum { SEDS_OK=0, SEDS_ERR=1 };
static struct { void *r; } g_router = {(void*)1};
static uint8_t g_discovery_schema_announced;
static uint8_t g_discovery_schema_requested;
static uint64_t g_discovery_schema_retry_ms;
static uint64_t g_discovery_schema_request_retry_ms;
static uint32_t g_discovery_schema_retry_delay_ms=1000U;
static uint32_t g_valve_discovery_attempts;
static int32_t g_valve_discovery_result;
static uint64_t now;
static unsigned attempts, polls, health, requests;
static int ready, fail, request_fail=-1;
static unsigned g_telemetry_service_stage, g_telemetry_discovery_seen;
static int init_telemetry_router(void) { return ready ? SEDS_OK : SEDS_ERR; }
static uint64_t tx_raw_now_ms_locked(void) { return now; }
static uint64_t telemetry_now_ms(void) { return now; }
static int seds_router_announce_discovery(void *r) { assert(r); ++attempts; return fail; }
static int seds_router_log_bytes(void *r,unsigned ty,const uint8_t *p,size_t n) { assert(r && ty==12 && p && n==0); ++requests; return request_fail<0 ? fail : request_fail; }
static int seds_router_poll_discovery(void *r, bool *queued) { assert(r && queued); ++polls; return SEDS_OK; }
static void telemetry_update_network_health(void *r) { assert(r); ++health; }
static void sim_probe_emit_heartbeat(void *r,uint64_t t) { (void)r;(void)t; }
static int flight_state_cache_poll(void *r) { (void)r;return 0; }
static int daq_calibration_poll(void *r) { (void)r;return 0; }
static int daq_log_clock_poll(void *r) { (void)r;return 0; }
static int flight_buzzer_poll(void *r) { (void)r;return 0; }
static int av_bay_underglow_poll(void *r) { (void)r;return 0; }
static void ota_stream_poll(void) {}
static void telemetry_lock(void) {}
static void telemetry_unlock(void) {}
''' + poll + r'''
int main(void) {
 assert(telemetry_poll_discovery()==SEDS_ERR && attempts==0 && polls==0);
 ready=1; fail=SEDS_ERR;
 assert(telemetry_poll_discovery()==SEDS_OK && attempts==1 && polls==1);
 now=999; telemetry_poll_discovery(); assert(attempts==1 && polls==2);
 now=1000; telemetry_poll_discovery(); assert(attempts==2 && polls==3);
 now=2000;telemetry_poll_discovery();assert(attempts==2);
 /* Captured late-join failure: local announcement still refused, but a
  * known peer must be asked for schemas without waiting for that success. */
 g_telemetry_discovery_seen=1;telemetry_poll_discovery();assert(requests==1&&!g_discovery_schema_announced);
 fail=SEDS_OK;now=3000;telemetry_poll_discovery();assert(attempts==3&&requests==2);
 g_telemetry_discovery_seen=0;
 for(now=3001;now<100000;now+=1000) telemetry_poll_discovery();
 assert(attempts==3 && requests==2 && health==polls);
 g_telemetry_discovery_seen=1; g_discovery_schema_requested=0; fail=SEDS_ERR;
 telemetry_poll_discovery(); assert(requests==3);
 now+=999; telemetry_poll_discovery(); assert(requests==3);
 now++; fail=SEDS_OK; telemetry_poll_discovery(); assert(requests==4);
 now+=5000; telemetry_poll_discovery(); assert(requests==4);
 /* Successful router recreation resets bootstrap state. */
 g_discovery_schema_announced=0; g_discovery_schema_requested=0; g_discovery_schema_retry_ms=0;g_discovery_schema_request_retry_ms=0;g_discovery_schema_retry_delay_ms=1000U;
 telemetry_poll_discovery(); assert(attempts==4);
 /* Persistent local schema refusal still learns peers, with bounded retries. */
 g_discovery_schema_announced=0;g_discovery_schema_requested=0;
 g_discovery_schema_retry_ms=0;g_discovery_schema_request_retry_ms=0;
 g_discovery_schema_retry_delay_ms=1000U;attempts=requests=0;
 fail=SEDS_ERR;request_fail=SEDS_OK;
 for(now=0;now<60000;now++)telemetry_poll_discovery();
 assert(attempts==6&&requests==1&&g_discovery_schema_requested);
 assert(!g_discovery_schema_announced&&g_discovery_schema_retry_delay_ms==30000U);
}
'''
        with tempfile.TemporaryDirectory() as directory:
            exe=Path(directory)/'test'
            result=subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',
                '-Wno-unused-function','-Wno-unused-variable','-x','c','-','-o',str(exe)],
                input=code,text=True,capture_output=True)
            self.assertEqual(result.returncode,0,result.stderr)
            subprocess.run([str(exe)],check=True)
        self.assertIn('g_router.created = 1U;\n  g_discovery_schema_announced = 0U;\n  g_discovery_schema_requested = 0U;\n  g_discovery_schema_retry_ms = 0ULL;',source)

if __name__ == '__main__':
    unittest.main()
