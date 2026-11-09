"""Exercise unique control endpoint heartbeat dispatch with the production GS guard."""
from pathlib import Path
import re,subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class UniqueHeartbeatDispatchTests(unittest.TestCase):
 def test_genuine_ground_heartbeat_uses_guard_without_reaching_command_effects(self):
  source=(ROOT/'Core/Src/telemetry.c').read_text()
  def function(signature):return re.search(signature+r'\([^)]*\)\s*\{.*?\n\}',source,re.S).group()
  code=r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include "ground_heartbeat_owner.h"
typedef int SedsResult;
#define SEDS_OK 0
#define SEDS_BAD_ARG -2
#define SEDS_DT_HEARTBEAT 106U
#define SEDS_DT_VALVE_COMMAND 245583120U
#define TX_SUCCESS 0
typedef struct {uint32_t ty;const char*sender;size_t sender_len;const void*payload;size_t payload_len;uint64_t timestamp;} SedsPacketView;
typedef unsigned long ULONG;static ULONG tick=3;static ULONG tx_time_get(void){return tick;}
static uint64_t g_station_report_utc;static ULONG g_station_report_tick;
static uint64_t now,last;
static uint32_t aborted,commands,g_heartbeat_handler_count,g_heartbeat_handler_error_count;
static SedsResult sim_probe_heartbeat_handler(const SedsPacketView*p,void*u){(void)p;(void)u;return SEDS_OK;}
static struct{void*r;}g_router={(void*)1};
static unsigned station_observed;static void board_recovery_observe_station_time(void*r,const SedsPacketView*p){assert(r==(void*)1&&p);station_observed++;}
static uint32_t thread_comm_get_abort(void){return aborted;}
static uint64_t telemetry_now_ms(void){return now;}
static int thread_comm_note_groundstation_heartbeat(uint64_t t){last=t;return TX_SUCCESS;}
static SedsResult Valve_Command_handler(const SedsPacketView*p,void*u){(void)u;commands++;return p?17:SEDS_BAD_ARG;}
'''+function('SedsResult Heartbeat_handler')+'\n'+function('static SedsResult control_packet_handler')+r'''
int main(void){SedsPacketView p={.ty=SEDS_DT_HEARTBEAT,.sender="GS",.sender_len=2,.payload=NULL,.payload_len=0};
for(now=500;now<=10000;now+=500){assert(control_packet_handler(&p,NULL)==SEDS_OK);assert(last==now);assert(commands==0);}
assert(g_heartbeat_handler_count==20&&!g_heartbeat_handler_error_count);
const char*other[]={"GB","RRLS","GSX","@addr:3561204632"};const size_t sizes[]={2,4,3,16};now=11000;
for(unsigned i=0;i<4;i++){p.sender=other[i];p.sender_len=sizes[i];assert(control_packet_handler(&p,NULL)==SEDS_OK);assert(last==10000&&!commands);}
p.sender="@addr:3561204631";p.sender_len=16;assert(control_packet_handler(&p,NULL)==SEDS_OK);assert(last==11000&&!commands);
unsigned seen=station_observed;assert(seen);aborted=1;now=12000;assert(control_packet_handler(&p,NULL)==SEDS_OK);assert(last==11000&&!commands);assert(station_observed==seen+1);aborted=0;
p.ty=7;assert(control_packet_handler(&p,NULL)==SEDS_OK&&!commands);
p.ty=SEDS_DT_VALVE_COMMAND;assert(control_packet_handler(&p,NULL)==17&&commands==1);assert(control_packet_handler(NULL,NULL)==SEDS_BAD_ARG&&commands==2);
/* Valve-report reference: the latest GS heartbeat time, followed in either
 * direction and also in Abort; other senders and unset stamps never move it. */
assert(!g_station_report_utc);p.ty=SEDS_DT_HEARTBEAT;p.sender="GS";p.sender_len=2;p.timestamp=900;tick=40;
assert(control_packet_handler(&p,NULL)==SEDS_OK&&g_station_report_utc==900&&g_station_report_tick==40);
p.timestamp=100;tick=41;aborted=1;assert(control_packet_handler(&p,NULL)==SEDS_OK&&g_station_report_utc==100&&g_station_report_tick==41);aborted=0;
p.timestamp=0;tick=42;assert(control_packet_handler(&p,NULL)==SEDS_OK&&g_station_report_utc==100&&g_station_report_tick==41);
p.timestamp=UINT64_MAX;assert(control_packet_handler(&p,NULL)==SEDS_OK&&g_station_report_utc==100&&g_station_report_tick==41);
p.sender="GB";p.timestamp=5000;assert(control_packet_handler(&p,NULL)==SEDS_OK&&g_station_report_utc==100&&g_station_report_tick==41);
return 0;}
'''
  with tempfile.TemporaryDirectory()as d:
   exe=Path(d)/'dispatch';p=subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(ROOT/'Core/Inc'),'-x','c','-','-o',str(exe)],input=code,text=True,capture_output=True)
   self.assertEqual(p.returncode,0,p.stderr);subprocess.run([str(exe)],check=True)
