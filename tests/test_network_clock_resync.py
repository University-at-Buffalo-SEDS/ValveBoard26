"""The network thread only observes the SEDSnet clock; it never restarts it.

SEDSnet 6 followers adopt a newer source epoch and step to it at once, in
either direction, so the old "restart timesync when the network clock is
more than 60 s from the GS heartbeat clock" heuristic is gone. The actual
telemetry_update_network_health() runs with modeled SDK calls across a
backward and a forward multi-hour network time step.
"""
from pathlib import Path
import re,subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class NetworkClockResync(unittest.TestCase):
 def test_network_health_observes_steps_without_restarting_timesync(self):
  source=(ROOT/'Core/Src/telemetry.c').read_text()
  self.assertNotIn('NETWORK_CLOCK_RESYNC',source)
  self.assertNotIn('network_clock_resync',source)
  body=re.search(r'static void telemetry_update_network_health\(SedsRouter \*router\)\s*\{.*?\n\}',source,re.S).group()
  code=r'''#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
typedef int SedsResult;typedef int SedsRouter;
#define SEDS_OK 0
static SedsRouter router;static uint64_t net;static bool valid;static unsigned observed;static uint64_t last_observed=1;
static uint32_t g_telemetry_timesync_valid,g_telemetry_discovery_seen=1,g_telemetry_network_ready;
typedef unsigned long ULONG;
static ULONG ticks=7;static ULONG tx_time_get(void){return ticks;}
static SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){assert(r==&router);if(!valid)return -1;*out=net;return SEDS_OK;}
static void telemetry_event_clock_observe(uint64_t utc,uint32_t tick){assert(tick==(uint32_t)ticks);observed++;last_observed=utc;}
'''+body+r'''
int main(void){
 valid=false;telemetry_update_network_health(&router);assert(last_observed==0&&!g_telemetry_timesync_valid&&!g_telemetry_network_ready);
 valid=true;net=1794183600000ULL;telemetry_update_network_health(&router);assert(last_observed==net&&g_telemetry_network_ready);
 /* GS steps back 4 h, then forward 3.8 h: each new time is observed as is. */
 ticks++;net-=4ULL*3600000ULL;telemetry_update_network_health(&router);assert(last_observed==net);
 ticks++;net+=3800ULL*3600ULL;telemetry_update_network_health(&router);assert(last_observed==net&&observed==4);
 return 0;}
'''
  with tempfile.TemporaryDirectory() as d:
   exe=Path(d)/'health';out=subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',str(exe)],input=code,text=True,capture_output=True)
   self.assertEqual(out.returncode,0,out.stderr);subprocess.run([str(exe)],check=True,timeout=10)
if __name__=='__main__':unittest.main()
