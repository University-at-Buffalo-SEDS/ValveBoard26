"""Actual owner/queue/callback source; bounded host-only MMIO/ThreadX fixtures.

The selected source may be canonical for a preserved causal old RED. No SDK,
network, physical outputs or private dependency override is involved.
"""
from pathlib import Path
from board_repo import ACTUATOR_BOARD,VALVE_BOARD,board_name
import ast,os,re,subprocess,tempfile,unittest
from test_launch_abort_policy import function
BASE=Path(__file__).resolve().parents[1]
ROOT=Path(os.environ.get('BOARD_POSTLAUNCH_SOURCE',BASE))
def literal(path,name):
 tree=ast.parse(path.read_text())
 return next(n.value.value for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name)and t.id==name for t in n.targets))
def prefix():
 harness=literal(BASE/'tests/test_verified_launch_latch.py','HARNESS').split('static UINT launch_request(void)')[0]
 harness=harness.replace('bool persistent_store_controls_ready(void){return true;}','static bool storage_ready=true;bool persistent_store_controls_ready(void){return storage_ready;}')
 harness=harness.replace('bool persistent_store_recovery_ready(void){return true;}','bool persistent_store_recovery_ready(void){return storage_ready;}')
 return harness.replace('ACTUAL_OWNER','')+'\nstatic uint32_t sampled_launch_tick(void){return 0U;}\nUINT fixture_timed_launch(uint64_t original,const char*sender,uint32_t length){return thread_comm_request_network_launch_original_at(original,sender,length,10000U,0U,sampled_launch_tick);}\n'
def failure_prefix():
 pre=prefix()
 old='UINT tx_mutex_put(TX_MUTEX*m){assert(m->locked);m->locked=0;return 0;}'
 assert old in pre
 return pre.replace(old,'static bool fail_next_state_release;UINT tx_mutex_put(TX_MUTEX*m){assert(m->locked);m->locked=0;if(m==state_mutex&&fail_next_state_release){fail_next_state_release=false;return TX_MUTEX_ERROR;}return 0;}')
def compile_case(code,case,inline_state=False):
 # Calls in the fixture offer an explicit coherent pre-admission clock.
 # The production source is separately compiled/included under its real names.
 code=code.replace('thread_comm_request_network_launch_original(', 'fixture_timed_launch(')
 effective=re.sub(r'#ifdef VB\n.*?#endif','',code,flags=re.S) if board_name(ROOT)==ACTUATOR_BOARD else code
 if 'ULONG tx_time_get(void){' not in effective:code+='\nULONG tx_time_get(void){return 0;}\n'
 if 'SedsResult Valve_Command_handler(' in code:
  telemetry=(ROOT/'Core/Src/telemetry.c').read_text()
  adapter='\nstatic SedsResult seds_router_get_network_time_ms(void*r,uint64_t*out){(void)r;*out=10000U;return SEDS_OK;}\n'
  helpers=adapter+function(telemetry,'launch_monotonic_tick')+'\n'+function(telemetry,'admit_timed_launch')+'\n'
  if 'network_admission_result(' in telemetry:
   helpers+='static SedsResult board_recovery_refused(void){return SEDS_OK;}\n'+function(telemetry,'network_admission_result')+'\n'
  code=code.replace('SedsResult Valve_Command_handler(',helpers+'SedsResult Valve_Command_handler(')
  code=code.replace('static struct{void*r;}g_router;','static struct{void*r;}g_router={(void*)1};')
 with tempfile.TemporaryDirectory(prefix='postlaunch-storage-')as temp:
  p=Path(temp);(p/'main.h').write_text(literal(BASE/'tests/test_recovery_durable_barrier.py','MAIN'))
  tree=ast.parse((BASE/'tests/test_board_recovery_owner.py').read_text())
  tx=next(n.args[0].value for n in ast.walk(tree)if isinstance(n,ast.Call)and isinstance(n.func,ast.Attribute)and n.func.attr=='write_text'and n.args and isinstance(n.args[0],ast.Constant)and isinstance(n.args[0].value,str)and 'typedef struct{UINT locked;}TX_MUTEX'in n.args[0].value)
  (p/'tx_api.h').write_text(tx);(p/'case.c').write_text(code);exe=p/'case'
  subprocess.run(['cc','-DVB=1','-DTELEMETRY_ENABLED=1','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(p),'-I'+str(ROOT/'Core/Inc'),str(ROOT/'Core/Src/board_abort_latch.c'),*([]if inline_state else[str(ROOT/'Core/Src/thread_comm.c')]),str(p/'case.c'),'-o',str(exe)],check=True,timeout=60)
  result=subprocess.run([str(exe),case],capture_output=True,text=True,timeout=5)
  assert result.returncode==0,result.stdout+result.stderr
  return result

class PostlaunchStorageContinuityTests(unittest.TestCase):
 def test_actual_main_loop_preserves_pending_running_and_completed_original(self):
  main=(ROOT/'Core/Src/main_thread.c').read_text()
  entry=function(main,'main_thread_entry');loop=entry[entry.index('    for (;;)'):entry.rfind('}')]
  context=re.search(r'typedef struct\{[^\n]*\} LaunchStart;',main).group(0)
  pre=failure_prefix().replace('ULONG tx_time_get(void){return 100;}','static ULONG ticks;ULONG tx_time_get(void){return ticks;}')
  pre=pre.replace('assert(cmd==6&&state==1);status_sends++;','(void)cmd;(void)state;status_sends++;')
  code=pre+r"""
#include <setjmp.h>
#include "board_recovery.h"
#include "bootstrap_owner.h"
static jmp_buf done;
static unsigned iterations,physical_on,physical_off;
static uint8_t g_aborted,g_pilot_valve_state,g_no_servo_open_state,g_nitrous_servo_open_state,g_outputs_initialized=1U;
static uint32_t g_launch_manual_epoch,g_launch_sequence_service_count,g_launch_sequence_pilot_attempt_count,g_launch_sequence_finish_count,g_launch_sequence_abort_seen_count;
static uint32_t g_sim_pilot_valve_state;
static int32_t g_sim_pilot_solenoid_result;
static ULONG g_launch_sequence_last_elapsed_ticks;
static thread_comm_launch_owner_t g_launch_owner;
static int pilot_solenoid;
#define TX_TIMER_TICKS_PER_SECOND 1000U
#define LAUNCH_SEQUENCE_PILOT_OPEN_DELAY_MS 10000U
#define LAUNCH_SEQUENCE_PILOT_OPEN_DURATION_MS 1500U
#define LAUNCH_SEQUENCE_PILOT_STATUS_PERIOD_TICKS 100U
#define MAIN_THREAD_MAX_COMMANDS_PER_TICK 4U
#define BOARD_WATCHDOG_CONTROL 0
static void no_servo_open(void){g_no_servo_open_state=1;}
static void no_servo_close(void){g_no_servo_open_state=0;}
static void dump_servo_open(void){g_nitrous_servo_open_state=1;}
static void dump_servo_close(void){g_nitrous_servo_open_state=0;}
static int solenoidOn(int*p){(void)p;physical_on++;return 0;}
static void solenoidOff(int*p){(void)p;physical_off++;}
static void board_watchdog_progress(int tag){(void)tag;iterations++;assert(iterations<50);}
static void board_bootstrap_storage_service(void){}
static void retry_pending_status_reports(void){}
static void publish_all_umbilical_statuses(void){}
/* Pressure stays nominal here; tests/test_overpressure.py owns that service. */
static void service_overpressure(void){}
"""+context+'\n'+'\n'.join(function(main,n)for n in [*(['post_launch_reset_hold'] if 'post_launch_reset_hold(void)' in main else []),'ms_to_ticks','manual_stop_apply_locked','main_thread_force_outputs_safe_off','start_launch_sequence_locked','start_launch_sequence','service_launch_sequence_request','apply_automatic_valve','service_launch_sequence_locked','service_launch_sequence'])+r"""
static void abort_state(void){manual_stop_apply_locked();g_aborted=1;}
static void service_abort_state(void){manual_stop_apply_locked();}
static void handle_command(thread_comm_msg_t message,uint32_t epoch){(void)epoch;assert(message==CMD_ABORT);(void)thread_comm_request_manual_abort();}
static unsigned scenario;static bool duplicate_running;
void tx_thread_sleep(ULONG delay){
 (void)delay;
 if(iterations==1){assert(g_launch_sequence_active&&!thread_comm_get_abort());if(duplicate_running){fail_next_state_release=true;assert(thread_comm_request_network_launch_original(10000,"GS",2)==THREAD_COMM_COMMIT_UNCERTAIN);assert(thread_comm_launch_continuity());}storage_ready=false;test_tamp.BKP28R=test_tamp.BKP29R=0;drop_launch=1;}
 ticks+=1000;
 if(scenario==1&&iterations==11){assert(g_pilot_valve_state&&physical_on==1);assert(thread_comm_request_manual_abort()==TX_SUCCESS);assert(!g_pilot_valve_state&&!thread_comm_launch_continuity());longjmp(done,1);}
 if(iterations==14){assert(g_launch_sequence_completed&&!g_launch_sequence_active&&physical_on==1&&!thread_comm_get_abort());assert(thread_comm_launch_continuity());assert(g_launch_sequence_start_count==1&&peer_sends==1);service_launch_sequence_request();assert(g_launch_sequence_start_count==1);longjmp(done,1);}
}
static void actual_control_loop(void){thread_comm_msg_t msg;uint32_t command_epoch=0;
"""+loop+r"""
}
int main(int argc,char**argv){
 assert(argc==2);scenario=!strcmp(argv[1],"manual-stop")||!strcmp(argv[1],"duplicate-running-stop");duplicate_running=!strncmp(argv[1],"duplicate-running",17);board_abort_latch_init();TX_BYTE_POOL pool={0};assert(thread_comm_init(&pool)==0);
 assert(thread_comm_bind_manual_stop_apply(manual_stop_apply_locked)==0);
 assert(thread_comm_request_network_launch_original(10000,"GS",2)==TX_SUCCESS);
 if(!strcmp(argv[1],"duplicate-pending")){fail_next_state_release=true;assert(thread_comm_request_network_launch_original(10000,"GS",2)==THREAD_COMM_COMMIT_UNCERTAIN);assert(thread_comm_launch_continuity());}
 if(strcmp(argv[1],"running")&&!duplicate_running){storage_ready=false;test_tamp.BKP28R=test_tamp.BKP29R=0;drop_launch=1;}
 if(setjmp(done)==0)actual_control_loop();
 assert(!persistent_store_controls_ready());return 0;
}
"""
  if 'thread_comm_launch_owner_t' not in (ROOT/'Core/Inc/thread_comm.h').read_text():
   code=code.replace('static thread_comm_launch_owner_t g_launch_owner;','').replace('&&!thread_comm_launch_continuity()','').replace('assert(thread_comm_launch_continuity());','')
  compile_case(code,'pending-running')
  compile_case(code,'running')
  compile_case(code,'manual-stop')
  compile_case(code,'duplicate-pending')
  compile_case(code,'duplicate-running')
  compile_case(code,'duplicate-running-stop')

 def test_actual_receipt_initial_failure_identity_consumption_and_manual_stop(self):
  code=re.sub(r"#ifdef VB\n.*?#endif", "",prefix(),flags=re.S)+r"""
static unsigned effects;
static void effect(void*context){(void)context;effects++;}
int main(int argc,char**argv){
 assert(argc==2);board_abort_latch_init();TX_BYTE_POOL pool={0};assert(thread_comm_init(&pool)==TX_SUCCESS);
 thread_comm_launch_owner_t owner={0};
"""
  code+=r"""
 if(!strcmp(argv[1],"initial-failure")){drop_launch=1;assert(thread_comm_request_network_launch_original(10000,"GS",2)!=TX_SUCCESS);assert(!thread_comm_launch_continuity()&&!thread_comm_take_accepted_launch(&owner));assert(thread_comm_request_manual_abort()==TX_SUCCESS);return 0;}
 assert(thread_comm_request_network_launch_original(10000,"GS",2)==TX_SUCCESS);
 assert(thread_comm_request_network_launch_original(10000,"GS",2)==TX_SUCCESS);
 assert(thread_comm_request_network_launch_original(10001,"GS",2)!=TX_SUCCESS);
 assert(thread_comm_request_network_launch_original(10000,"X",1)!=TX_SUCCESS);
 assert(thread_comm_take_accepted_launch(&owner));thread_comm_launch_owner_t second={0};assert(!thread_comm_take_accepted_launch(&second));
"""
  code+=r"""
 assert(owner.source_utc==10000&&owner.serial!=0&&thread_comm_launch_continuity());
 if(!strcmp(argv[1],"stop-after-take")){assert(thread_comm_request_manual_abort()==TX_SUCCESS);assert(!thread_comm_launch_begin_owned(&owner,effect,0)&&!thread_comm_launch_effect_owned(&owner,effect,0)&&!effects&&!thread_comm_launch_continuity());return 0;}
 storage_ready=false;test_tamp.BKP28R=test_tamp.BKP29R=0;drop_launch=1;
 assert(thread_comm_launch_original_accepted(10000,"GS",2));
 thread_comm_launch_owner_t wrong=owner;wrong.serial++;assert(!thread_comm_launch_begin_owned(&wrong,effect,0));
 wrong=owner;wrong.source_utc++;assert(!thread_comm_launch_begin_owned(&wrong,effect,0));
 assert(!effects&&thread_comm_launch_begin_owned(&owner,effect,0)&&effects==1);
 assert(!thread_comm_launch_begin_owned(&owner,effect,0)&&effects==1);
 assert(thread_comm_launch_effect_owned(&owner,effect,0)&&effects==2);
 thread_comm_launch_complete_owned(&owner);assert(thread_comm_launch_continuity());
 assert(!thread_comm_launch_effect_owned(&owner,effect,0)&&effects==2);
 assert(thread_comm_launch_original_accepted(10000,"GS",2));
 assert(thread_comm_request_manual_abort()==TX_SUCCESS);assert(thread_comm_get_abort()&&!thread_comm_launch_continuity());
 assert(!thread_comm_launch_original_accepted(10000,"GS",2)&&!thread_comm_launch_begin_owned(&owner,effect,0)&&!thread_comm_launch_effect_owned(&owner,effect,0)&&effects==2);
 return 0;
}
"""
  compile_case(code,'initial-failure')
  compile_case(code,'receipt-lifecycle')
  compile_case(code,'stop-after-take')

 def test_actual_callback_trusted_manual_stop_uses_original_ram_floor_after_storage_fault(self):
  telemetry=(ROOT/'Core/Src/telemetry.c').read_text()
  code=re.sub(r'#ifdef VB\n.*?#endif','',prefix(),flags=re.S)+r'''
typedef int SedsResult;
#define SEDS_OK 0
#define SEDS_ERR -1
#define SEDS_BAD_ARG -2
#define SEDS_IO -3
#define SEDS_HANDLER_ERROR -4
typedef struct{uint32_t ty;const char*sender;size_t sender_len;uint64_t timestamp;const uint8_t*payload;size_t payload_len;}SedsPacketView;
static struct{void*r;}g_router;
static unsigned sync_calls,broadcasts,physical_off;
static bool trusted=true;
static unsigned g_sim_valve_commands_received,g_sim_valve_open_seen,g_sim_groundstation_confirmation_received;
static int board_recovery_sync_command_cutoff(void){sync_calls++;return SEDS_IO;}
static bool board_recovery_command_is_stale(const SedsPacketView*p){(void)p;return false;}
static bool board_recovery_manual_stop_source_allowed(void*r,const SedsPacketView*p){(void)r;(void)p;return trusted;}
static int telemetry_broadcast_abort(const char*reason){assert(reason);broadcasts++;return 0;}
static int32_t seds_pkt_get_u8(const SedsPacketView*p,uint8_t*out,size_t count){assert(count==1U);*out=p->payload[0];return 1;}
static void physical_stop(void){physical_off++;}
#include "board_recovery.h"
#include "bootstrap_owner.h"
static unsigned recovery_finishes;
static uint8_t read_outputs(uint32_t*mask){*mask=0;return BOARD_RECOVERY_OUTPUTS_VALID|BOARD_RECOVERY_SAFE_OUTPUTS|BOARD_RECOVERY_OWNER_STOPPED;}
static uint8_t prepare_outputs(uint8_t desired){assert(desired==0);return 1;}
static void finish_recovery(void){recovery_finishes++;}
'''+function(telemetry,'Valve_Command_handler')+r'''
int main(int argc,char**argv){
 assert(argc==2);bool preparing=!strcmp(argv[1],"prepare-race");board_abort_latch_init();TX_BYTE_POOL pool={0};assert(thread_comm_init(&pool)==TX_SUCCESS);
 assert(thread_comm_bind_manual_stop_apply(physical_stop)==TX_SUCCESS);
 assert(thread_comm_publish_command_cutoff(5000)==TX_SUCCESS);
 assert(thread_comm_request_network_launch_original(10000,"GS",2)==TX_SUCCESS);
 const uint32_t accepted_epoch=thread_comm_manual_epoch();
 if(preparing){
  board_recovery_bind_owner(read_outputs,prepare_outputs,finish_recovery);
  const struct TampFake before=test_tamp;
  assert(board_recovery_begin_prepare(0,accepted_epoch)==-10);
  assert(thread_comm_launch_continuity()&&thread_comm_manual_epoch()==accepted_epoch&&!thread_comm_get_abort());
  assert(!physical_off&&!recovery_finishes&&!memcmp(&before,&test_tamp,sizeof before));
 }
 storage_ready=false;const uint8_t stop=CMD_ABORT;
 SedsPacketView packet={.sender="GS",.sender_len=2,.timestamp=4000,.payload=&stop,.payload_len=1};
 uint32_t epoch=thread_comm_manual_epoch();assert(Valve_Command_handler(&packet,0)==SEDS_OK);
 assert(!physical_off&&!thread_comm_get_abort()&&thread_comm_launch_continuity()&&epoch==thread_comm_manual_epoch()&&!broadcasts&&!sync_calls);
 trusted=false;packet.timestamp=11000;assert(Valve_Command_handler(&packet,0)==SEDS_IO);assert(sync_calls==1&&!physical_off);
 trusted=true;assert(Valve_Command_handler(&packet,0)==SEDS_OK);
 assert(physical_off==1&&thread_comm_get_abort()&&thread_comm_manual_epoch()!=epoch&&broadcasts==1&&sync_calls==1);
 if(preparing){storage_ready=true;BoardRecoverySnapshot snapshot={0};assert(board_recovery_apply_safe(0,accepted_epoch,&snapshot)==-10);board_recovery_execute_owned();assert(recovery_finishes==0&&thread_comm_get_abort());}
 return 0;
}
'''
  compile_case(code,'trusted-stop-ram-floor')
  compile_case(code,'prepare-race')

 def test_checked_local_serial_exhaustion_refuses_without_marker_or_owner(self):
  pre=re.sub(r'#ifdef VB\n.*?#endif','',prefix(),flags=re.S)
  code=pre+'\n#include "'+str((ROOT/'Core/Src/thread_comm.c').resolve())+'"\n'+r"""
int main(int argc,char**argv){
 (void)argv;assert(argc==2);board_abort_latch_init();TX_BYTE_POOL pool={0};assert(thread_comm_init(&pool)==TX_SUCCESS);
 g_launch_serial=UINT32_MAX-1U;
"""
  code+=r"""
 assert(thread_comm_request_network_launch_original(10000,"GS",2)!=TX_SUCCESS);
 thread_comm_launch_owner_t output={0};assert(!thread_comm_take_accepted_launch(&output));
"""
  code+=r"""
 assert(!thread_comm_launch_continuity()&&!board_launch_latch_is_set()&&!thread_comm_get_abort());
 assert(g_launch_serial==UINT32_MAX-1U);fprintf(stderr,"HOST_LAYOUT receipt=%zu serial=%zu continuity=%zu token=%zu ring_entry=%zu ULONG=%zu\n",sizeof g_accepted_launch,sizeof g_launch_serial,sizeof g_launch_continuity,sizeof(thread_comm_launch_owner_t),sizeof(thread_comm_msg_t),sizeof(ULONG));return 0;
}
"""
  result=compile_case(code,'exhausted',inline_state=True)
  print(result.stderr.strip())

 def test_invalid_recovery_epoch_never_authorizes_launch_but_manual_stop_remains_safe(self):
  pre=re.sub(r'#ifdef VB\n.*?#endif','',prefix(),flags=re.S)
  code=pre+'\n#include "'+str((ROOT/'Core/Src/thread_comm.c').resolve())+'"\n'+r"""
static unsigned safe_off;
static void manual_safe_off(void){safe_off++;}
int main(int argc,char**argv){
 assert(argc==2);board_abort_latch_init();TX_BYTE_POOL pool={0};assert(thread_comm_init(&pool)==TX_SUCCESS);
 assert(thread_comm_bind_manual_stop_apply(manual_safe_off)==TX_SUCCESS);
 g_thread_comm_recovery_epoch=!strcmp(argv[1],"zero")?0U:UINT32_MAX;
"""
  code+=r"""
 assert(thread_comm_request_network_launch_original(10000,"GS",2)!=TX_SUCCESS);
 thread_comm_launch_owner_t output={0};assert(!thread_comm_take_accepted_launch(&output));
 assert(thread_comm_request_accepted_launch_manual_abort(11000)==TX_QUEUE_ERROR);
"""
  code+=r"""
 assert(!thread_comm_launch_continuity()&&!board_launch_latch_is_set()&&!thread_comm_get_abort()&&!safe_off);
 storage_ready=false;assert(thread_comm_request_manual_abort()==TX_SUCCESS);
 assert(thread_comm_get_abort()&&safe_off==1&&!thread_comm_launch_continuity());return 0;
}
"""
  compile_case(code,'zero',inline_state=True)
  compile_case(code,'max',inline_state=True)

 def test_initial_uncertain_release_never_mints_accepted_history(self):
  pre=re.sub(r'#ifdef VB\n.*?#endif','',failure_prefix(),flags=re.S)
  code=pre+'\n#include "'+str((ROOT/'Core/Src/thread_comm.c').resolve())+'"\n'+r"""
int main(int argc,char**argv){
 (void)argv;assert(argc==2);board_abort_latch_init();TX_BYTE_POOL pool={0};assert(thread_comm_init(&pool)==TX_SUCCESS);
 fail_next_state_release=true;
"""
  code+=r"""
 assert(thread_comm_request_network_launch_original(10000,"GS",2)==THREAD_COMM_COMMIT_UNCERTAIN);
 thread_comm_launch_owner_t owner={0};assert(!thread_comm_take_accepted_launch(&owner));
 storage_ready=false;assert(thread_comm_request_accepted_launch_manual_abort(11000)==TX_QUEUE_ERROR);
"""
  code+=r"""
 assert(!g_accepted_launch.initial_verified&&g_accepted_launch.phase==LAUNCH_NONE&&!thread_comm_launch_continuity());
 assert(!thread_comm_launch_original_accepted(10000,"GS",2));assert(!thread_comm_get_abort());return 0;
}
"""
  compile_case(code,'initial-uncertain',inline_state=True)

 def test_later_duplicate_or_distinct_release_error_never_revokes_prior_accepted_owner(self):
  pre=re.sub(r'#ifdef VB\n.*?#endif','',failure_prefix(),flags=re.S)
  code=pre+'\n#include "'+str((ROOT/'Core/Src/thread_comm.c').resolve())+'"\n'+r"""
static unsigned effects,safe_off;
static void effect(void*context){(void)context;effects++;}
static void stop(void){safe_off++;}
int main(int argc,char**argv){
 assert(argc==2);board_abort_latch_init();TX_BYTE_POOL pool={0};assert(thread_comm_init(&pool)==TX_SUCCESS);assert(thread_comm_bind_manual_stop_apply(stop)==TX_SUCCESS);
 thread_comm_launch_owner_t owner={0};uint64_t offered=!strcmp(argv[1],"distinct")?10001:10000;
"""
  code+=r"""
 assert(thread_comm_request_network_launch_original(10000,"GS",2)==TX_SUCCESS);
 fail_next_state_release=true;assert(thread_comm_request_network_launch_original(offered,"GS",2)==THREAD_COMM_COMMIT_UNCERTAIN);
 assert(thread_comm_launch_continuity()&&thread_comm_launch_original_accepted(10000,"GS",2));assert(thread_comm_take_accepted_launch(&owner));
"""
  code+=r"""
 assert(owner.source_utc==10000&&owner.epoch==thread_comm_manual_epoch()&&owner.serial==1&&g_accepted_launch.initial_verified);
 storage_ready=false;assert(thread_comm_launch_begin_owned(&owner,effect,0)&&effects==1);assert(thread_comm_launch_effect_owned(&owner,effect,0)&&effects==2);
 assert(thread_comm_request_manual_abort()==TX_SUCCESS&&safe_off==1&&!thread_comm_launch_continuity());assert(!thread_comm_launch_effect_owned(&owner,effect,0)&&effects==2);return 0;
}
"""
  compile_case(code,'duplicate',inline_state=True)
  compile_case(code,'distinct',inline_state=True)
