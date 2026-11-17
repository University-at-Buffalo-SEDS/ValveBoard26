"""Recovery and command admission across SEDSnet network time steps.

SEDSnet 6 stamps every packet with (time_epoch, timestamp). The board judges
request freshness only with seds_router_network_stamp_fresh() and orders its
replay floor, command cutoff and recovery sources by (epoch, timestamp). After
the GS clock stepped back hours, the old board refused every station request
(the persisted floor and the forward-only station clock were hours ahead);
these cases prove that a backward 4 h step, a forward 3.8 h step and a newer
epoch the board is still re-anchoring to are all handled by the core
mechanism, while a late copy stamped before the step, a replay below the floor
and a replay of the pre-Reset request stay refused.

The actual recovery transport, storage, boot identity, floor and thread_comm C
run over the durable-recovery flash fixture; only the SDK's network-time
answers are modeled: one remembered step translates stamps of the previous
epoch, a newer epoch is fresh only while this node has known of it for at most
the window, and there is no network time at all until the model sets it.
"""
from pathlib import Path
import os,subprocess,tempfile,unittest
from native_sdk import configured_dependency_source
ROOT=Path(__file__).resolve().parents[1]
FRESH_OLD='SedsResult seds_router_network_stamp_fresh(SedsRouter*r,uint16_t e,uint64_t t,uint64_t age,uint64_t ahead,bool*out){assert(sdk_unlocked());(void)r;*out=e==0U&&t&&now&&(now>=t?now-t<=age:t-now<=ahead);return SEDS_OK;}'
RAISE_OLD='SedsResult seds_router_raise_time_epoch(SedsRouter*r,uint16_t e,uint16_t*o){(void)r;if(o)*o=e;return SEDS_OK;}'
FRESH_NEW=r'''/* SDK network-time model. now: this node's network time in net_epoch.
 * A stamp of prev_epoch is translated by prev_shift (new = old + shift). A
 * newer epoch is fresh while newer_known_ticks is at most the window old. */
static bool net_valid=true;static uint16_t net_epoch,prev_epoch,newer_epoch;static bool prev_valid,newer_valid;
static int64_t prev_shift;static ULONG newer_known_ticks;static unsigned raise_calls;
static bool window(uint64_t at,uint64_t t,uint64_t age,uint64_t ahead){return t&&at&&(at>=t?at-t<=age:t-at<=ahead);}
SedsResult seds_router_network_stamp_fresh(SedsRouter*r,uint16_t e,uint64_t t,uint64_t age,uint64_t ahead,bool*out){assert(sdk_unlocked());(void)r;
 *out=false;if(!net_valid)return SEDS_OK;
 if(e==net_epoch)*out=window(now,t,age,ahead);
 else if(prev_valid&&e==prev_epoch)*out=window(now,(uint64_t)((int64_t)t+prev_shift),age,ahead);
 else if(newer_valid&&e==newer_epoch&&e>net_epoch)*out=(ULONG)(ticks-newer_known_ticks)<=age;
 return SEDS_OK;}
SedsResult seds_router_raise_time_epoch(SedsRouter*r,uint16_t e,uint16_t*o){(void)r;raise_calls++;if(e>net_epoch)net_epoch=e;if(o)*o=net_epoch;return SEDS_OK;}
/* The node steps to a new epoch: its clock jumps by shift, the old epoch is remembered. */
static void step(int64_t shift){prev_epoch=net_epoch;prev_shift=shift;prev_valid=true;net_epoch++;now=(uint64_t)((int64_t)now+shift);newer_valid=false;}
'''
MAIN=r'''
#define T0 UINT64_C(1794183589988)
#define HOUR UINT64_C(3600000)
#define K(epoch,ms) network_stamp_key((uint16_t)(epoch),(ms))
static void normal_apply(void*context){(void)context;}
static void prove_command(uint64_t key){
 thread_comm_msg_t msg=2,received=0;uint32_t epoch=0;
 assert(thread_comm_admit_network(msg,key)==TX_SUCCESS);
 assert(thread_comm_receive_owned(&received,&epoch)==TX_SUCCESS&&received==2);
 assert(thread_comm_manual_effect_owned(key,epoch,normal_apply,NULL));
}
static uint32_t live_epoch(void){BoardRecoverySnapshot s={0};(void)board_recovery_snapshot(&s);assert(s.abort_epoch);return s.abort_epoch;}
static uint8_t recover(SedsRouter*r,uint64_t nonce,uint64_t query_time,uint64_t time,uint8_t desired){
 BoardRecoveryRequest q={.operation=BOARD_RECOVERY_QUERY,.nonce=nonce,.issuer_session=77};
 if(query_time){send_req(r,&q,query_time);assert(response[1]==0);}
 BoardRecoveryRequest rq={.operation=BOARD_RECOVERY_RECOVER,.desired_state=desired,.nonce=nonce+1,.issuer_session=77,.expected_boot_generation=board_recovery_boot_current(),.expected_abort_epoch=live_epoch()};
 unsigned before=responses;send_req(r,&rq,time);if(responses!=before)return response[1];
 main_service();return response[1];
}
/* A valve command from the GS stamped with key: stale per the actual gate? */
static bool command_stale(uint64_t key){uint32_t endpoint=SEDS_EP_VALVE_BOARD;uint8_t cmd=2;
 SedsPacketView p={.ty=SEDS_DT_VALVE_COMMAND,.sender="GS",.sender_len=2,.endpoints=&endpoint,.num_endpoints=1,.payload=&cmd,.payload_len=1,.timestamp=network_stamp_ms(key),.time_epoch=network_stamp_epoch(key)};
 return board_recovery_command_is_stale(&p);}
static uint64_t floor_key(void){uint64_t f=0;assert(board_recovery_floor_read_cutoff(&f));return f;}
static void boot(bool load){
 if(load){FILE*f=fopen(image_path,"rb");assert(f&&fread(&image,sizeof image,1,f)==1&&fclose(f)==0);}
 else{memset(image.nvm,255,NVM_SIZE);launchcore_storage_set_driver(&launchcore_board_storage_driver);seed_control("seed_healthy");}
 launchcore_storage_set_driver(&launchcore_board_storage_driver);fixture=false;
 board_abort_latch_init();assert(persistent_store_boot_begin()==LAUNCHCORE_PERSIST_OK);assert(board_recovery_boot_generation());board_bootstrap_storage_probe_boot();
 assert(persistent_store_runtime_start()==TX_SUCCESS);thread_present=true;TX_BYTE_POOL pool={0};assert(thread_comm_init(&pool)==TX_SUCCESS);
 board_recovery_bind_owner(read_outputs,prepare_outputs,finish_outputs);assert(board_bootstrap_storage_bind_owner(stop_owner));
}
static void save_image(void){FILE*f=fopen(image_path,"wb");assert(f&&fwrite(&image,sizeof image,1,f)==1&&fclose(f)==0);}
int main(int argc,char**argv){
 assert(argc==3);scenario=argv[1];image_path=argv[2];
 int cookie=0;SedsRouter*r=(SedsRouter*)&cookie;
 if(!strcmp(scenario,"reboot_pending")){
  now=T0+60000;boot(true);assert(flight_state_cache_init(r)==SEDS_OK);board_recovery_transport_set_router(r);
  /* The interrupted NORMAL recovery from the previous boot generation. */
  assert(board_bootstrap_storage_pending()&&thread_comm_get_abort()&&!persistent_store_controls_ready());
  uint64_t gen=board_recovery_boot_current();assert(gen==43);
  assert(recover(r,31,T0+60001,T0+60002,0)==BOARD_RECOVERY_RESULT_OK);
  assert(!thread_comm_get_abort()&&!board_abort_latch_is_set()&&persistent_store_controls_ready()&&!board_bootstrap_storage_pending());
  prove_command(T0+60003);puts("interrupted NORMAL recovery completes after reboot PASS");return 0;
 }
 if(!strcmp(scenario,"reboot_raise")){
  /* Whole-stack power cycle: every node restarts at epoch 0. The persisted
   * floor's epoch is restored before the router is published. */
  net_epoch=0;now=T0-4*HOUR+90000;boot(true);assert(flight_state_cache_init(r)==SEDS_OK);
  assert(network_stamp_epoch(floor_key())==1);
  assert(board_recovery_raise_time_epoch(r)==SEDS_OK&&raise_calls==1&&net_epoch==1);
  board_recovery_transport_set_router(r);
  assert(!command_stale(K(1,now))&&command_stale(K(0,T0+5000)));
  puts("persisted floor epoch raises the network epoch after a power cycle PASS");return 0;
 }
 boot(false);assert(flight_state_cache_init(r)==SEDS_OK);board_recovery_transport_set_router(r);
 assert(!thread_comm_get_abort()&&persistent_store_controls_ready());
 /* A healthy floor of the epoch 0 board raises nothing. */
 assert(board_recovery_raise_time_epoch(r)==SEDS_OK&&net_epoch==0);
 if(!strcmp(scenario,"no_time")){
  /* No network time: nothing is fresh, nothing is written. */
  net_valid=false;now=T0;assert(thread_comm_request_manual_abort()==TX_SUCCESS&&thread_comm_get_abort());
  unsigned writes=media_writes;
  BoardRecoveryRequest early={.operation=BOARD_RECOVERY_RECOVER,.desired_state=0,.nonce=2,.issuer_session=77,.expected_boot_generation=board_recovery_boot_current(),.expected_abort_epoch=live_epoch()};
  send_req(r,&early,T0);assert(response[1]==BOARD_RECOVERY_RESULT_STALE&&media_writes==writes&&thread_comm_get_abort());
  net_valid=true;assert(recover(r,11,T0+1,T0+2,0)==BOARD_RECOVERY_RESULT_OK&&!thread_comm_get_abort());
  puts("no network time refuses recovery until SEDSnet time exists PASS");return 0;
 }
 if(!strcmp(scenario,"step_back")){
  /* A Reset at T0 installs the persisted floor (epoch 0). */
  now=T0;assert(thread_comm_request_manual_abort()==TX_SUCCESS);
  assert(recover(r,11,T0+1,T0+2,0)==BOARD_RECOVERY_RESULT_OK&&!thread_comm_get_abort());
  assert(floor_key()==K(0,T0+2)&&board_recovery_floor_active());
  BoardRecoveryRequest pre={.operation=BOARD_RECOVERY_RECOVER,.desired_state=0,.nonce=12,.issuer_session=77,.expected_boot_generation=board_recovery_boot_current(),.expected_abort_epoch=live_epoch()-1U};
  /* The epoch 0 floor record is the old 16-byte layout: a plain timestamp. */
  { uint8_t wire[16];size_t n=sizeof wire;assert(persistent_store_get(BOARD_RECOVERY_FLOOR_KEY,wire,&n)==LAUNCHCORE_PERSIST_OK&&n==16&&board_recovery_read_le64(wire+8)==T0+2); }
  now=T0+5000;prove_command(K(0,T0+4000));
  /* The GS clock steps back 4 h: the board follows into epoch 1 at once. */
  step(-(int64_t)(4*HOUR));now+=3000;
  /* A fresh request on the new clock passes the floor and the cutoff. */
  assert(!command_stale(K(1,now-500)));prove_command(K(1,now-500));
  /* A copy stamped on the old clock just before the step is judged by its
   * real age (the remembered step translates it): 3.5 s old it is fresh;
   * delivered late, 15.5 s old, it is refused. */
  assert(!command_stale(K(0,T0+4500)));now+=12000;assert(command_stale(K(0,T0+4500)));
  /* A replay below the floor is refused whatever its age. */
  assert(command_stale(K(0,T0+2))&&command_stale(K(0,T0+1)));
  assert(thread_comm_admit_network(2,K(0,T0+2))==THREAD_COMM_STALE_TIMESTAMP);
  /* Abort and a Reset stamped after the step, below the old floor's
   * timestamp but in the newer epoch, are accepted and move the floor. */
  assert(thread_comm_request_manual_abort_original_utc(K(1,now))==TX_SUCCESS&&thread_comm_get_abort());
  assert(recover(r,21,K(1,now+1),K(1,now+2),0)==BOARD_RECOVERY_RESULT_OK&&!thread_comm_get_abort());
  assert(floor_key()==K(1,now+2)&&network_stamp_ms(floor_key())<T0);
  /* The pre-Reset request (a replay stamped on the old clock) stays refused. */
  pre.expected_abort_epoch=live_epoch();send_req(r,&pre,K(0,T0+2));assert(response[1]==BOARD_RECOVERY_RESULT_STALE&&!thread_comm_get_abort());
  send_req(r,&pre,K(0,T0+5000));assert(response[1]==BOARD_RECOVERY_RESULT_STALE&&!thread_comm_get_abort());
  /* A manual Abort stamped on the old clock below the new floor is retired. */
  assert(thread_comm_request_manual_abort_original_utc(K(0,T0+5000))==TX_SUCCESS&&!thread_comm_get_abort());
  prove_command(K(1,now+3));
  save_image();puts("backward 4 h step: fresh requests and a Reset accepted, late copies and replays refused PASS");return 0;
 }
 if(!strcmp(scenario,"step_forward")){
  now=T0;assert(thread_comm_request_manual_abort()==TX_SUCCESS);
  assert(recover(r,11,T0+1,T0+2,0)==BOARD_RECOVERY_RESULT_OK);
  step((int64_t)(38*HOUR/10));now+=1000;
  assert(!command_stale(K(1,now))&&command_stale(K(1,now+5000)));
  /* A copy stamped on the old clock 1 s before the step is translated onto
   * the new clock: fresh now, refused once it is 20 s old. */
  assert(!command_stale(K(0,T0+1000)));
  now+=20000;assert(command_stale(K(0,T0+1000))&&command_stale(K(1,now-16000)));
  assert(thread_comm_request_manual_abort()==TX_SUCCESS);
  assert(recover(r,21,K(1,now+1),K(1,now+2),0)==BOARD_RECOVERY_RESULT_OK&&floor_key()==K(1,now+2));
  assert(command_stale(K(0,T0+1000)));
  puts("forward 3.8 h step: translated freshness, Reset accepted PASS");return 0;
 }
 if(!strcmp(scenario,"newer_epoch")){
  /* The GS stepped back into epoch 1 and this board has not re-anchored
   * yet: a newer-epoch request is fresh only while it re-anchors. */
  now=T0;assert(thread_comm_request_manual_abort()==TX_SUCCESS);
  assert(recover(r,11,T0+1,T0+2,0)==BOARD_RECOVERY_RESULT_OK);
  uint64_t gs=T0-4*HOUR;newer_epoch=1;newer_valid=true;newer_known_ticks=ticks;
  assert(!command_stale(K(1,gs)));prove_command(K(1,gs));
  assert(thread_comm_request_manual_abort()==TX_SUCCESS);
  assert(recover(r,21,K(1,gs+1),K(1,gs+2),0)==BOARD_RECOVERY_RESULT_OK&&floor_key()==K(1,gs+2));
  ticks+=16000;assert(command_stale(K(1,gs+3)));
  /* An epoch older than any remembered step cannot be translated: stale. */
  net_epoch=3;prev_epoch=2;newer_valid=false;assert(command_stale(K(1,gs+3)));
  puts("newer-epoch requests are fresh only while the board re-anchors PASS");return 0;
 }
 if(!strcmp(scenario,"interrupt")){
  now=T0-1;assert(thread_comm_request_manual_abort()==TX_SUCCESS);
  BoardRecoveryRequest q={.operation=BOARD_RECOVERY_QUERY,.nonce=21,.issuer_session=77};send_req(r,&q,T0-1);assert(response[1]==0);
  BoardRecoveryRequest rq={.operation=BOARD_RECOVERY_RECOVER,.desired_state=0,.nonce=22,.issuer_session=77,.expected_boot_generation=board_recovery_boot_current(),.expected_abort_epoch=live_epoch()};
  now=T0;scenario="floor_after";begin_phase(3);send_req(r,&rq,T0);main_service();
  assert(!"the fixture power cut did not happen");
 }
 if(!strcmp(scenario,"reset_stale")||!strcmp(scenario,"reset_late")){
  /* A running board told to reset: the request went stale before the main
   * service reached it (stale), or after the board had stopped (late). */
  assert(thread_comm_set_flight_state(1)==TX_SUCCESS);now=T0;
  BoardRecoveryRequest rq={.operation=BOARD_RECOVERY_RECOVER,.desired_state=0,.nonce=41,.issuer_session=77,.expected_boot_generation=board_recovery_boot_current(),.expected_abort_epoch=live_epoch()};
  unsigned before=responses,writes=media_writes;send_req(r,&rq,T0);assert(responses==before);
  if(!strcmp(scenario,"reset_stale")){
   now+=16000;ticks+=16000;main_service();
   assert(response[1]==BOARD_RECOVERY_RESULT_STALE&&media_writes==writes);
   assert(!thread_comm_get_abort()&&!board_abort_latch_is_set()&&persistent_store_controls_ready());
   prove_command(T0+16001);
   assert(recover(r,43,T0+16002,T0+16003,0)==BOARD_RECOVERY_RESULT_OK&&!thread_comm_get_abort());
   puts("stale reset of a running board latches no Abort and keeps commands PASS");return 0;
  }
  main_owner=true;board_bootstrap_storage_service();main_owner=false;assert(response[1]==BOARD_RECOVERY_RESULT_PENDING);
  now+=16000;ticks+=16000;main_service();
  assert(response[1]==BOARD_RECOVERY_RESULT_OK&&!thread_comm_get_abort()&&!board_abort_latch_is_set()&&persistent_store_controls_ready());
  puts("reset that already stopped the board finishes to its safe target PASS");return 0;
 }
 return 2;
}
'''
class NetworkTimeSteps(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.tmp=tempfile.TemporaryDirectory(prefix='network-time-steps-');p=Path(cls.tmp.name)
  sdk=Path(os.environ['SEDSNET_TEST_SOURCE']);lc=configured_dependency_source(ROOT,'sedslaunchcore','SDK_FREE_LAUNCHCORE_SOURCE')
  fixture=ROOT/'tests/fixtures/durable_recovery'
  harness=(fixture/'harness.c').read_text().split('int main(int argc,char**argv){',1)[0]
  assert FRESH_OLD in harness and RAISE_OLD in harness
  harness=harness.replace(RAISE_OLD,'').replace(FRESH_OLD,FRESH_NEW)
  (p/'steps.c').write_text(harness+MAIN)
  modules=['board_abort_latch.c','thread_comm.c','board_recovery_wire.c','board_recovery_boot.c','board_recovery_transport.c','flight_state_cache.c','persistent_store.c','board_bootstrap_storage.c','bootstrap_marker.c']
  cls.exe=p/'steps'
  subprocess.run(['cc','-DEXPECTED_CAP=32','-DTELEMETRY_ENABLED=1','-DSAFE_MASK=6','-DRECOVERY_ENDPOINT=2001','-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-attributes','-fsanitize=address,undefined','-I'+str(fixture),'-I'+str(ROOT/'Core/Inc'),'-I'+str(ROOT/'Core/Src'),'-I'+str(sdk/'C-Headers'),'-I'+str(lc/'bootloader/include'),*[str(ROOT/'Core/Src'/n)for n in modules],str(ROOT/'Bootloader/storage_dispatch.c'),str(lc/'bootloader/src/persist.c'),str(lc/'bootloader/src/crc32.c'),str(p/'steps.c'),'-o',str(cls.exe)],check=True)
 @classmethod
 def tearDownClass(cls):cls.tmp.cleanup()
 def run_case(self,name,image,code=0):
  out=subprocess.run([str(self.exe),name,str(image)],capture_output=True,text=True,timeout=60)
  self.assertEqual(out.returncode,code,out.stdout+out.stderr)
  return out
 def test_backward_4h_step_accepts_fresh_requests_and_reset_refuses_late_copies_and_replays(self):
  self.run_case('step_back',Path(self.tmp.name)/'a.bin')
 def test_persisted_floor_epoch_raises_the_network_epoch_after_power_cycle(self):
  image=Path(self.tmp.name)/'b.bin'
  self.run_case('step_back',image)
  self.run_case('reboot_raise',image)
 def test_forward_3_8h_step(self):
  self.run_case('step_forward',Path(self.tmp.name)/'c.bin')
 def test_newer_epoch_request_while_the_board_re_anchors(self):
  self.run_case('newer_epoch',Path(self.tmp.name)/'d.bin')
 def test_no_network_time_is_never_fresh(self):
  self.run_case('no_time',Path(self.tmp.name)/'e.bin')
 def test_interrupted_normal_recovery_completes_after_reboot(self):
  image=Path(self.tmp.name)/'f.bin'
  self.run_case('interrupt',image,42)
  self.run_case('reboot_pending',image)
 def test_stale_reset_of_running_board_latches_no_abort(self):
  self.run_case('reset_stale',Path(self.tmp.name)/'g.bin')
 def test_reset_that_stopped_the_board_finishes_after_window(self):
  self.run_case('reset_late',Path(self.tmp.name)/'h.bin')
if __name__=='__main__':unittest.main()
