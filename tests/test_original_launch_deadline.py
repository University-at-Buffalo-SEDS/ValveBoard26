"""Real launch admission and epoch owner; deterministic paired clock fixtures.

No SDK/wire changes or physical outputs. Each case starts a fresh process.
"""
from pathlib import Path
from board_repo import ACTUATOR_BOARD,VALVE_BOARD,board_name
import re, unittest
from test_postlaunch_storage_continuity import prefix, compile_case
ROOT=Path(__file__).resolve().parents[1]

class OriginalLaunchDeadlineTests(unittest.TestCase):
 def test_actual_original_timeline_admission_and_owner_guards(self):
  ab=board_name(ROOT)==ACTUATOR_BOARD
  pre=re.sub(r'#ifdef VB\n.*?#endif','',prefix(),flags=re.S)
  code=pre+'\n#include "'+str((ROOT/'Core/Src/thread_comm.c').resolve())+'"\n'+r'''
static uint32_t fake_tick,clock_calls,observed_deadline,effects;
static uint32_t clock_now(void){clock_calls++;return fake_tick;}
static void begin_effect(void*context){assert(*(uint32_t*)context==observed_deadline);effects++;}
static void continuation_effect(void*context){(void)context;effects++;}
static UINT offer(uint64_t source,uint64_t utc,uint32_t tick,uint32_t(*clock)(void)){
OFFER
}
int main(int argc,char**argv){
 assert(argc==2);board_abort_latch_init();TX_BYTE_POOL pool={0};assert(thread_comm_init(&pool)==TX_SUCCESS);
 uint64_t source=10000U,utc=10000U;uint32_t pair=100U;fake_tick=100U;
 const char*which=argv[1];bool invalid=false;
 if(!strncmp(which,"delay-",6)){unsigned d=(unsigned)strtoul(which+6,0,10);utc+=d;pair+=d;fake_tick=pair;}
 else if(!strcmp(which,"future-boundary"))source+=1000U;
 else if(!strcmp(which,"future-refuse")){source+=1001U;invalid=true;}
 else if(!strcmp(which,"stale-refuse")){utc+=5001U;invalid=true;}
 else if(!strcmp(which,"zero-source")){source=0;invalid=true;}
 else if(!strcmp(which,"zero-utc")){utc=0;invalid=true;}
 else if(!strcmp(which,"max-utc")){utc=UINT64_MAX;invalid=true;}
 else if(!strcmp(which,"aged-pair-valid"))fake_tick+=1000U;
 else if(!strcmp(which,"aged-pair-stale")){utc+=5000U;fake_tick+=1U;invalid=true;}
 else if(!strcmp(which,"halfwrap")){fake_tick+=UINT32_C(0x80000000);invalid=true;}
 else if(!strcmp(which,"utc-overflow")){source=UINT64_MAX-1U;utc=UINT64_MAX-1U;fake_tick+=2U;invalid=true;}
 else if(!strcmp(which,"tick-wrap")){pair=UINT32_MAX-9U;fake_tick=pair;}
 else if(!strcmp(which,"no-clock"))invalid=true;
 if(invalid){
  assert(offer(source,utc,pair,!strcmp(which,"no-clock")?0:clock_now)!=TX_SUCCESS);
  assert(!thread_comm_launch_continuity()&&!board_launch_latch_is_set()&&!thread_comm_get_abort());
  assert(g_accepted_launch.phase==LAUNCH_NONE&&g_launch_serial==0U);
  return 0;
 }
 uint32_t expected=(!strcmp(which,"tick-wrap")?pair:100U)+(uint32_t)THREAD_COMM_LAUNCH_EFFECT_DELAY_MS;
 if(!strcmp(which,"future-boundary"))expected+=1000U;
 assert(offer(source,utc,pair,clock_now)==TX_SUCCESS);assert(clock_calls==1U);
 assert(g_accepted_launch.deadline_valid&&g_accepted_launch.deadline==expected);
 /* A delayed/clockless duplicate owns no new instant, tick or identity. */
 uint32_t calls=clock_calls;fake_tick+=30000U;
 assert(offer(source,0U,0U,0)==TX_SUCCESS&&clock_calls==calls);
 assert(g_accepted_launch.deadline==expected);
 thread_comm_launch_owner_t owner={0};
TAKE
 assert(owner.source_utc==source&&owner.epoch&&owner.serial);
 if(!strcmp(which,"stop-before-begin")){
  assert(thread_comm_request_manual_abort()==TX_SUCCESS);
  uint32_t output=123U;assert(!thread_comm_launch_begin_owned_deadline(&owner,begin_effect,&output,&output));
  assert(!effects&&output==123U&&!thread_comm_launch_continuity());return 0;
 }
 uint32_t output=123U;observed_deadline=expected;
 assert(thread_comm_launch_begin_owned_deadline(&owner,begin_effect,&output,&output));
 assert(output==expected&&effects==1U);
 /* Neither lost UTC nor backup marker changes the running owner's timeline. */
 storage_ready=false;test_tamp.BKP28R=test_tamp.BKP29R=0;drop_launch=1;
 assert(thread_comm_launch_effect_owned(&owner,continuation_effect,0)&&effects==2U);
 assert(g_accepted_launch.deadline==expected&&!thread_comm_get_abort());
 assert(!thread_comm_request_automatic_abort());
 assert(thread_comm_request_manual_abort()==TX_SUCCESS&&thread_comm_get_abort());
 assert(!thread_comm_launch_effect_owned(&owner,continuation_effect,0)&&effects==2U);
 return 0;
}
'''
  code='#include <stdlib.h>\n'+code
  code=code.replace('OFFER','return '+('thread_comm_admit_network_launch_original_at((thread_comm_msg_t){.cmd=CMD_IGNITER_SEQUENCE,.timestamp_ms=source},"GS",2U,utc,tick,clock);'if ab else'thread_comm_request_network_launch_original_at(source,"GS",2U,utc,tick,clock);'))
  code=code.replace('TAKE','thread_comm_msg_t queued;uint32_t epoch;assert(thread_comm_receive_owned(&queued,&epoch)==TX_SUCCESS);assert(thread_comm_launch_claim(queued.timestamp_ms,epoch,&owner));'if ab else'assert(thread_comm_take_accepted_launch(&owner));')
  for case in ['delay-0','delay-1000','delay-4000','delay-4250','delay-4500','delay-5000','future-boundary','future-refuse','stale-refuse','zero-source','zero-utc','max-utc','aged-pair-valid','aged-pair-stale','halfwrap','utc-overflow','tick-wrap','no-clock','stop-before-begin']:
   with self.subTest(case=case):compile_case(code,case,inline_state=True)
