"""A wrongly anchored network clock is restarted from the network thread.

Bench evidence: the board's SEDSnet clock read about its own uptime while the
station's clock was UTC. A relayed consumer never completes a verified time
exchange, so only a timesync restart lets the next announce anchor it again.
The actual telemetry_update_network_health() runs with modeled SDK calls.
"""
from pathlib import Path
import re,subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class NetworkClockResync(unittest.TestCase):
 def test_suspect_clock_restarts_timesync_and_is_not_observed(self):
  source=(ROOT/'Core/Src/telemetry.c').read_text()
  holdoff=re.search(r'#define NETWORK_CLOCK_RESYNC_HOLDOFF_TICKS.*?static ULONG g_network_clock_resync_tick;',source,re.S).group()
  body=re.search(r'static void telemetry_update_network_health\(SedsRouter \*router\)\s*\{.*?\n\}',source,re.S).group()
  code=r'''#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
typedef int SedsResult;typedef int SedsRouter;
#define SEDS_OK 0
static SedsRouter router;static uint64_t net;static bool valid,suspect;static unsigned configures,observed;static uint64_t last_observed=1;
static uint32_t g_telemetry_timesync_valid,g_telemetry_discovery_seen=1,g_telemetry_network_ready,g_network_clock_resyncs;
typedef unsigned long ULONG;
#define TX_TIMER_TICKS_PER_SECOND 1000U
static ULONG ticks=7;static ULONG tx_time_get(void){return ticks;}
'''+holdoff+r'''
static SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){assert(r==&router);if(!valid)return -1;*out=net;return SEDS_OK;}
static bool board_recovery_network_clock_resync_needed(uint64_t n){assert(n==net);return suspect;}
static SedsResult telemetry_configure_timesync_locked(SedsRouter*r){assert(r==&router);configures++;return SEDS_OK;}
static void telemetry_event_clock_observe(uint64_t utc,uint32_t tick){(void)tick;observed++;last_observed=utc;}
'''+body+r'''
int main(void){
 valid=false;telemetry_update_network_health(&router);assert(!configures&&last_observed==0&&!g_telemetry_timesync_valid);
 valid=true;net=1794183600000ULL;telemetry_update_network_health(&router);assert(!configures&&last_observed==net&&g_telemetry_network_ready);
 net=5006972U;suspect=true;telemetry_update_network_health(&router);assert(configures==1&&g_network_clock_resyncs==1&&last_observed==0);
 /* A source that still disagrees with the GS is not torn down again on
  * every poll: the next restart waits 30 s of monotonic ticks. */
 ticks+=30U*TX_TIMER_TICKS_PER_SECOND-1U;telemetry_update_network_health(&router);assert(configures==1&&last_observed==net);
 ticks++;telemetry_update_network_health(&router);assert(configures==2&&g_network_clock_resyncs==2&&last_observed==0);
 ticks=(ULONG)-5;telemetry_update_network_health(&router);assert(configures==3);
 ticks=(ULONG)(30U*TX_TIMER_TICKS_PER_SECOND-6U);telemetry_update_network_health(&router);assert(configures==3);
 ticks++;telemetry_update_network_health(&router);assert(configures==4);
 return 0;}
'''
  with tempfile.TemporaryDirectory() as d:
   exe=Path(d)/'resync';out=subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',str(exe)],input=code,text=True,capture_output=True)
   self.assertEqual(out.returncode,0,out.stderr);subprocess.run([str(exe)],check=True,timeout=10)
if __name__=='__main__':unittest.main()
