"""Dropped backup writes must not authorize a physical launch sequence.

They keep the RAM launch inhibit but never latch Abort: no automatic Abort
after Launch.

Actual latch/admission and VB owner functions; MMIO/ThreadX/output callbacks are
host fixtures. This injects a fault, not evidence of a hardware incident.
"""
from pathlib import Path
import ast,re,subprocess,tempfile,unittest
from test_recovery_durable_barrier import MAIN
from test_launch_abort_policy import function
ROOT=Path(__file__).resolve().parents[1]
HARNESS=r'''#include <stdbool.h>
#include <stdint.h>
bool persistent_store_controls_ready(void){return true;}
bool persistent_store_completed_boundary_ready(void){return false;}
uint8_t board_recovery_manual_time_retired(uint64_t source){(void)source;return 0;}
/* Legacy-owner unit boundary: no commissioning is in flight in these tests.
 * Real BRPC integration is covered separately with the actual journal. */
bool persistent_store_bootstrap_pending(void){return false;}
bool persistent_store_bootstrap_metadata_ready(void){return true;}
void persistent_store_hold_bootstrap(void){}
bool persistent_store_publish_bootstrap_complete(void){return false;}

#include <stdint.h>
bool persistent_store_recovery_ready(void){return true;}
bool persistent_store_classify_absent_floor(uint64_t expected){(void)expected;return false;}
bool persistent_store_commit_absent_floor(const uint8_t value[16]){(void)value;return false;}


#include "main.h"
#include "thread_comm.h"
#include "board_abort_latch.h"
#include "board_commands.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
struct RccFake test_rcc;struct PwrFake test_pwr;struct TampFake test_tamp;
static unsigned drop_launch;
#ifdef VB
static unsigned peer_sends,status_sends;
#endif
static TX_MUTEX*state_mutex;static void(*before_lock)(void);
void test_dsb(void){if(drop_launch){test_tamp.BKP28R=0;test_tamp.BKP29R=0;}}
uint32_t HAL_GetTick(void){return 0;}
UINT tx_mutex_create(TX_MUTEX*m,const char*n,UINT i){(void)i;m->locked=0;if(!strcmp(n,"thread_comm_state_mutex"))state_mutex=m;return 0;}
UINT tx_mutex_get(TX_MUTEX*m,ULONG w){(void)w;if(m==state_mutex&&before_lock){void(*hook)(void)=before_lock;before_lock=0;hook();}assert(!m->locked);m->locked=1;return 0;}
UINT tx_mutex_put(TX_MUTEX*m){assert(m->locked);m->locked=0;return 0;}
UINT tx_semaphore_create(TX_SEMAPHORE*s,const char*n,ULONG c){(void)n;s->count=c;return 0;}
UINT tx_semaphore_get(TX_SEMAPHORE*s,ULONG w){(void)w;if(!s->count)return TX_NO_INSTANCE;s->count--;return 0;}
UINT tx_semaphore_put(TX_SEMAPHORE*s){s->count++;return 0;}
#ifdef VB
static uint8_t g_launch_sequence_active,g_launch_sequence_actuator_started,g_launch_sequence_pilot_opened,g_launch_sequence_completed;
static ULONG g_launch_sequence_start_ticks,g_launch_sequence_pilot_open_ticks,g_launch_sequence_last_pilot_status_ticks;
static uint32_t g_launch_sequence_start_count;
#define CMD_IGNITER_SEQUENCE 14U
ULONG tx_time_get(void){return 100;}
uint64_t telemetry_unix_ms(void){return 10000;}
int telemetry_publish_flight_state_at(uint8_t state,uint64_t timestamp){assert(state==7&&timestamp);return 0;}
int telemetry_send_actuator_command_at(uint8_t cmd,uint64_t timestamp){assert(cmd==14&&timestamp);peer_sends++;return 0;}
int telemetry_send_actuator_command(uint8_t cmd){assert(cmd==14);peer_sends++;return 0;}
static int publish_umbilical_status(uint8_t cmd,uint8_t state){assert(cmd==6&&state==1);status_sends++;return 0;}
#define LAUNCH_SEQUENCE_PILOT_OPEN_DELAY_MS 10000U
ACTUAL_OWNER
#endif
static UINT launch_request(void){
#ifdef VB
 return thread_comm_request_launch_sequence(10000);
#else
 thread_comm_msg_t message={.cmd=14,.timestamp_ms=10000};return thread_comm_admit_launch(message);
#endif
}
static void manual_stop(void){assert(thread_comm_request_manual_abort()==0);}
static void save(const char*path){FILE*f=fopen(path,"wb");assert(f&&fwrite(&test_tamp,sizeof test_tamp,1,f)==1&&fclose(f)==0);}
static void load(const char*path){FILE*f=fopen(path,"rb");assert(f&&fread(&test_tamp,sizeof test_tamp,1,f)==1&&fclose(f)==0);}
int main(int argc,char**argv){
 assert(argc==3);const char*scenario=argv[1];
 if(!strcmp(scenario,"reboot_bad")||!strcmp(scenario,"reboot_good"))load(argv[2]);
 board_abort_latch_init();TX_BYTE_POOL pool={0};assert(thread_comm_init(&pool)==0);
 if(!strcmp(scenario,"reboot_bad")){assert(!thread_comm_get_abort()&&!board_abort_latch_is_set()&&!board_launch_latch_is_set());return 0;}
 if(!strcmp(scenario,"reboot_good")){assert(board_launch_latch_is_set()&&!thread_comm_get_abort());assert(thread_comm_set_flight_state(6)==0);assert(thread_comm_request_automatic_abort()==0);manual_stop();assert(thread_comm_get_abort());return 0;}
 assert(!thread_comm_get_abort());assert(thread_comm_set_flight_state(6)==0);
 if(!strcmp(scenario,"request_bad")){
  drop_launch=1;assert(launch_request()!=TX_SUCCESS);assert(!thread_comm_get_abort()&&board_launch_latch_is_set());assert(!test_tamp.BKP30R&&!test_tamp.BKP31R);
  assert(thread_comm_request_automatic_abort()==0&&!thread_comm_get_abort());
#ifdef VB
  uint64_t timestamp;assert(!thread_comm_take_launch_sequence_request(&timestamp));
#else
  thread_comm_msg_t message;assert(thread_comm_receive(&message,TX_NO_WAIT)!=TX_SUCCESS);
#endif
  save(argv[2]);return 0;
 }
 if(!strcmp(scenario,"stop_before_request")){before_lock=manual_stop;assert(launch_request()!=TX_SUCCESS&&thread_comm_get_abort());return 0;}
 if(!strcmp(scenario,"managed_bad")){drop_launch=1;assert(thread_comm_apply_managed_flight_state(7)!=TX_SUCCESS);assert(thread_comm_get_flight_state()==6&&!thread_comm_get_abort()&&board_launch_latch_is_set());
  /* The safety service retries without latching Abort, then verifies once writes work. */
  thread_comm_service_safety_publication();assert(!thread_comm_get_abort()&&!board_launch_latch_was_verified());
  assert(thread_comm_request_automatic_abort()==0&&!test_tamp.BKP30R&&!test_tamp.BKP31R);
  drop_launch=0;thread_comm_service_safety_publication();assert(board_launch_latch_was_verified()&&!thread_comm_get_abort());
  assert(thread_comm_apply_managed_flight_state(7)==TX_SUCCESS&&thread_comm_get_flight_state()==7);return 0;}
 if(!strcmp(scenario,"setter_bad")){drop_launch=1;assert(thread_comm_set_flight_state(7)!=TX_SUCCESS);assert(thread_comm_get_flight_state()==6&&!thread_comm_get_abort()&&board_launch_latch_is_set());assert(!test_tamp.BKP30R&&!test_tamp.BKP31R);return 0;}
#ifdef VB
 if(!strcmp(scenario,"owner_bad")){
  assert(launch_request()==TX_SUCCESS);test_tamp.BKP28R=test_tamp.BKP29R=0;drop_launch=1;service_launch_sequence_request();
  /* Initial verified acceptance owns continuation despite SECOND loss. */
  assert(peer_sends==1&&status_sends==1&&g_launch_sequence_active&&!thread_comm_get_abort());return 0;
 }
 if(!strcmp(scenario,"direct_owner_bad")){drop_launch=1;start_launch_sequence((thread_comm_launch_owner_t){.source_utc=10000,.epoch=thread_comm_manual_epoch(),.serial=1});assert(!peer_sends&&!status_sends&&!g_launch_sequence_active&&!thread_comm_get_abort());return 0;}
 if(!strcmp(scenario,"stop_before_owner")){assert(launch_request()==0);before_lock=manual_stop;service_launch_sequence_request();assert(!peer_sends&&!g_launch_sequence_active&&thread_comm_get_abort());return 0;}
#endif
 assert(!strcmp(scenario,"healthy"));assert(launch_request()==TX_SUCCESS);assert(board_launch_latch_is_set()&&!thread_comm_get_abort());
 assert(test_tamp.BKP28R==UINT32_C(0x4c41554e)&&test_tamp.BKP29R==~UINT32_C(0x4c41554e));
#ifdef VB
 service_launch_sequence_request();assert(peer_sends==1&&status_sends==1&&g_launch_sequence_active);
#else
 thread_comm_msg_t message;assert(thread_comm_receive(&message,TX_NO_WAIT)==TX_SUCCESS&&message.cmd==14);
#endif
 save(argv[2]);return 0;
}
'''
def owner_source(owner):
 # Preserve actual typed original-epoch context and bounded raw setup callback.
 context=re.search(r'typedef struct\{[^\n]*\} LaunchStart;',owner).group(0)
 return 'static uint32_t g_launch_manual_epoch;\nstatic thread_comm_launch_owner_t g_launch_owner;\n'+context+'\n'+'\n'.join(function(owner,name)for name in ['ms_to_ticks','start_launch_sequence_locked','start_launch_sequence','service_launch_sequence_request'])

class VerifiedLaunchTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.tmp=tempfile.TemporaryDirectory(prefix='verified-launch-');p=Path(cls.tmp.name);(p/'main.h').write_text(MAIN)
  tree=ast.parse((ROOT/'tests/test_board_recovery_owner.py').read_text());tx=next(n.args[0].value for n in ast.walk(tree)if isinstance(n,ast.Call)and isinstance(n.func,ast.Attribute)and n.func.attr=='write_text'and n.args and isinstance(n.args[0],ast.Constant)and isinstance(n.args[0].value,str)and 'typedef struct{UINT locked;}TX_MUTEX'in n.args[0].value);(p/'tx_api.h').write_text(tx)
  cls.vb=ROOT.name=='ValveBoard26';code=HARNESS
  if cls.vb:
   owner=(ROOT/'Core/Src/main_thread.c').read_text();code=code.replace('ACTUAL_OWNER',owner_source(owner))
  else:code=code.replace('ACTUAL_OWNER','')
  (p/'harness.c').write_text(code);cls.exe=p/'test';cls.image=p/'backup.bin'
  subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',*(['-DVB=1','-DTELEMETRY_ENABLED=1']if cls.vb else[]),'-I'+str(p),'-I'+str(ROOT/'Core/Inc'),str(ROOT/'Core/Src/board_abort_latch.c'),str(ROOT/'Core/Src/thread_comm.c'),str(p/'harness.c'),'-o',str(cls.exe)],check=True)
 @classmethod
 def tearDownClass(cls):cls.tmp.cleanup()
 def case(self,name):
  result=subprocess.run([str(self.exe),name,str(self.image)],capture_output=True,text=True);self.assertEqual(result.returncode,0,result.stdout+result.stderr)
 def test_dropped_launch_writes_reject_admission_and_leave_no_launch_owner(self):self.case('request_bad');self.case('reboot_bad')
 def test_managed_launch_state_never_accepts_failed_durable_latch(self):self.case('managed_bad')
 def test_direct_state_setter_propagates_failed_durable_latch(self):self.case('setter_bad')
 def test_healthy_launch_and_watchdog_retention_preserve_manual_stop(self):self.case('healthy');self.case('reboot_good')
 def test_vb_owner_requires_initial_acceptance_and_preserves_it_across_second_loss(self):
  if self.vb:self.case('owner_bad');self.case('direct_owner_bad');self.case('stop_before_owner')
  else:self.case('stop_before_request')
