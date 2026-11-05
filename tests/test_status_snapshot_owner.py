"""Actual status producer and nonwaiting owner helper causal ordering controls."""
from pathlib import Path
import unittest
from status_submit_fixture import run
from router_lifecycle_fixture import function
ROOT=Path(__file__).resolve().parents[1]
class CoherentStatusOwnerTests(unittest.TestCase):
 def test_known_utc_before_no_wait_owner_and_no_state_lock_across_sdk(self):
  helper=function((ROOT/'Core/Src/thread_comm.c').read_text(),'thread_comm_status_snapshot_try')
  run(r'''
typedef uint8_t(*thread_comm_status_read_t)(uint8_t,uint8_t*);
#define TX_NO_WAIT 0
static unsigned state_held,state_busy,readiness=1,release_ok=1;
static unsigned g_thread_comm_initialized=1,g_command_admission_fault;
static int g_thread_comm_state_mutex;
static int state_mutex_get(int*m,unsigned wait){(void)m;assert(wait==TX_NO_WAIT&&checked_depth==1);if(state_busy)return 1;assert(!state_held);state_held=1;return 0;}
static unsigned bootstrap_state_release(void){assert(state_held);state_held=0;return release_ok;}
static unsigned persistent_store_controls_ready(void){assert(!state_held);return readiness;}
#define tx_mutex_get state_mutex_get
''' +helper+r'''
#undef tx_mutex_get
static uint8_t output=1,sequence=1;static unsigned reads;static uint64_t observed_at;
static uint8_t read_locked(uint8_t id,uint8_t*s){assert(state_held&&checked_depth==1);reads++;observed_at=utc;*s=id==6?sequence:output;return 1;}
static unsigned stop_before_sdk;
static void require_state_released(void){assert(!state_held);if(stop_before_sdk){output=0;sequence=0;stop_before_sdk=0;}}
static void stop_during_router_acquisition(void){output=0;sequence=0;}
static uint8_t snapshot(uint8_t id,uint8_t*s){return thread_comm_status_snapshot_try(id,s,read_locked)==0;}
int main(void){SedsResult result;test_before_sdk=require_state_released;
 /* Stop while waiting for router ownership: only current safe state is read. */
 output=1;sequence=1;test_after_lock=stop_during_router_acquisition;utc=1000;change_clock_after_read=2000;
 assert(telemetry_submit_umbilical_snapshot(0,snapshot,&result)==TELEMETRY_STATUS_ACCEPTED_LOCAL);
 assert(result==0&&sdk_calls==1&&last_state==0&&last_stamp==1000&&observed_at==2000&&!state_held);test_after_lock=0;
 assert(telemetry_submit_umbilical_snapshot(6,snapshot,&result)==TELEMETRY_STATUS_ACCEPTED_LOCAL&&last_state==0);
 sequence=1;assert(telemetry_submit_umbilical_snapshot(6,snapshot,&result)==TELEMETRY_STATUS_ACCEPTED_LOCAL&&last_state==1);
 unsigned prior=sdk_calls,n=reads;state_busy=1;
 assert(telemetry_submit_umbilical_snapshot(0,snapshot,&result)==TELEMETRY_STATUS_DEFERRED&&result==-14&&sdk_calls==prior&&reads==n);
 state_busy=0;readiness=0;
 assert(telemetry_submit_umbilical_snapshot(0,snapshot,&result)==TELEMETRY_STATUS_ACCEPTED_LOCAL&&sdk_calls==prior+1&&!state_held);
 assert(!persistent_store_controls_ready()&&g_command_admission_fault==0);prior=sdk_calls;
 readiness=1;utc=0;change_clock_after_read=0;n=reads;
 assert(telemetry_submit_umbilical_snapshot(0,snapshot,&result)==TELEMETRY_STATUS_DEFERRED&&reads==n&&sdk_calls==prior);
 utc=3000;sdk_result=-14;
 assert(telemetry_submit_umbilical_snapshot(0,snapshot,&result)==TELEMETRY_STATUS_REFUSED&&result==-14&&sdk_calls==prior+1);
 /* Old retained observation cannot be recalled; its original timestamp remains
  * unchanged after a later Stop, while the next actual observation reports0. */
 sdk_result=0;output=1;sequence=1;utc=4000;stop_before_sdk=1;
 assert(telemetry_submit_umbilical_snapshot(0,snapshot,&result)==TELEMETRY_STATUS_ACCEPTED_LOCAL&&last_stamp==4000&&last_state==1);
 assert(output==0&&sequence==0);utc=5000;assert(last_stamp==4000&&last_state==1);
 assert(telemetry_submit_umbilical_snapshot(0,snapshot,&result)==TELEMETRY_STATUS_ACCEPTED_LOCAL&&last_stamp==5000&&last_state==0);
 return 0;}
''')

 def test_virgin_abort_and_prior_fault_observation_does_not_enable_commands(self):
  source=(ROOT/'Core/Src/thread_comm.c').read_text()
  helper=function(source,'thread_comm_status_snapshot_try')
  send=function(source,'thread_comm_send_owned_nowait')
  admit=function(source,'thread_comm_admit_network')
  run(r'''
typedef uint8_t(*thread_comm_status_read_t)(uint8_t,uint8_t*);
typedef unsigned UINT;typedef uint32_t thread_comm_msg_t;
#define TX_NO_WAIT 0U
#define TX_SUCCESS 0U
#define TX_MUTEX_ERROR 1U
#define TX_QUEUE_ERROR 2U
#define THREAD_COMM_COMMIT_UNCERTAIN 3U
#define THREAD_COMM_STALE_TIMESTAMP 4U
static unsigned state_held,state_busy,readiness,release_ok=1,reads,read_valid=1;
static unsigned g_thread_comm_initialized=1,g_command_admission_fault,g_thread_comm_abort,g_recovery_pending;
static int g_thread_comm_state_mutex,g_thread_comm_spaces_sem;
static unsigned command_commits;
static UINT state_mutex_get(int*m,unsigned wait){(void)m;assert(wait==TX_NO_WAIT||wait==TX_WAIT_FOREVER);if(state_busy)return TX_MUTEX_ERROR;assert(!state_held);state_held=1;return TX_SUCCESS;}
static unsigned bootstrap_state_release(void){assert(state_held);state_held=0;if(!release_ok){g_command_admission_fault=1;readiness=0;}return release_ok;}
static unsigned command_state_release(void){return bootstrap_state_release();}
static unsigned persistent_store_controls_ready(void){return readiness;}
static unsigned network_timestamp_allowed_locked(uint64_t source){assert(state_held&&source==1000);return 1;}
static UINT tx_semaphore_get(int*s,unsigned wait){(void)s;assert(wait==TX_NO_WAIT);return TX_SUCCESS;}
static UINT thread_comm_commit_reserved_locked(thread_comm_msg_t msg,uint8_t*committed){(void)msg;assert(state_held);*committed=1;command_commits++;return TX_SUCCESS;}
#define tx_mutex_get state_mutex_get
''' +helper+'\n'+send+'\n'+admit+r'''
#undef tx_mutex_get
static uint8_t read_locked(uint8_t id,uint8_t*out){assert(state_held&&checked_depth==1);reads++;if(!read_valid)return 0;switch(id){case 0:*out=0;break;case 1:case 2:*out=1;break;case 6:*out=0;break;default:return 0;}return 1;}
static uint8_t snapshot(uint8_t id,uint8_t*out){return thread_comm_status_snapshot_try(id,out,read_locked)==0;}
static void require_release_before_sdk(void){assert(!state_held);}
int main(void){
 SedsResult result;test_before_sdk=require_release_before_sdk;const uint8_t ids[]={0,1,2,6};const uint8_t expected[]={0,1,1,0};
 /* Virgin/held, explicit Abort, and prior admission fault all keep their
  * command barriers. Valid commanded GPIO/PWM observations remain visible. */
 for(unsigned mode=0;mode<3;mode++){
  readiness=mode!=0;g_thread_comm_abort=mode==1;g_command_admission_fault=mode==2;
  for(unsigned i=0;i<4;i++){
   unsigned prior=sdk_calls;utc=1000+mode*100+i;
   assert(telemetry_submit_umbilical_snapshot(ids[i],snapshot,&result)==TELEMETRY_STATUS_ACCEPTED_LOCAL);
   assert(result==SEDS_OK&&sdk_calls==prior+1&&last_id==ids[i]&&last_state==expected[i]&&last_stamp==utc&&!state_held);
   assert(readiness==(mode!=0)&&g_thread_comm_abort==(mode==1)&&g_command_admission_fault==(mode==2));
   UINT refused=thread_comm_admit_network(0,1000);
   assert(refused!=TX_SUCCESS&&command_commits==0&&!state_held);
  }
 }
 unsigned prior=sdk_calls,n=reads;state_busy=1;
 assert(telemetry_submit_umbilical_snapshot(0,snapshot,&result)==TELEMETRY_STATUS_DEFERRED&&sdk_calls==prior&&reads==n);
 state_busy=0;g_thread_comm_initialized=0;
 assert(telemetry_submit_umbilical_snapshot(0,snapshot,&result)==TELEMETRY_STATUS_DEFERRED&&sdk_calls==prior&&reads==n);
 g_thread_comm_initialized=1;read_valid=0;
 assert(telemetry_submit_umbilical_snapshot(0,snapshot,&result)==TELEMETRY_STATUS_DEFERRED&&sdk_calls==prior&&!state_held);
 read_valid=1;release_ok=0;
 assert(telemetry_submit_umbilical_snapshot(0,snapshot,&result)==TELEMETRY_STATUS_DEFERRED&&sdk_calls==prior&&g_command_admission_fault&&command_commits==0);
 return 0;
}
''')
