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
 assert(telemetry_submit_umbilical_snapshot(0,snapshot,&result)==TELEMETRY_STATUS_DEFERRED&&sdk_calls==prior&&!state_held);
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
