"""Actual storage, boot identity, command queue and explicit abort recovery.

Real strict LaunchCore journal code runs over the existing bounded flash fixture;
no telemetry/schema shortcut, GPIO hardware or SDK radio is involved.
"""
from pathlib import Path
import os,subprocess,tempfile,unittest
from native_sdk import configured_dependency_source
ROOT=Path(__file__).resolve().parents[1]
MAIN=r'''
extern volatile uint32_t g_persistent_store_errors;
static unsigned normal_applied;
static void normal_apply(void*context);
static void prove_blocked(void){thread_comm_msg_t msg=2;assert(thread_comm_admit_network(msg,10000)!=TX_SUCCESS);assert(!thread_comm_manual_effect_owned(10000,thread_comm_manual_epoch(),normal_apply,NULL));}
static void normal_apply(void*context){(void)context;++normal_applied;}
static void prove_command(uint64_t timestamp){
 thread_comm_msg_t msg=2,received=0;uint32_t epoch=0;
 assert(thread_comm_admit_network(msg,timestamp)==TX_SUCCESS);
 assert(thread_comm_receive_owned(&received,&epoch)==TX_SUCCESS&&received==2);
 assert(thread_comm_manual_effect_owned(timestamp,epoch,normal_apply,NULL));
}
int main(int argc,char**argv){
 assert(argc==3);scenario=argv[1];image_path=argv[2];
 memset(image.nvm,255,NVM_SIZE);
 launchcore_storage_set_driver(&launchcore_board_storage_driver);
 if(!strcmp(scenario,"legacy"))seed_control("seed_absent");
 if(!strcmp(scenario,"deleted"))seed_control("seed_deletedgen");
 if(!strcmp(scenario,"corrupt"))seed_control("seed_corrupt");
 if(!strcmp(scenario,"deletedfloor"))seed_control("seed_deletedfloor");
 if(!strcmp(scenario,"badmarker"))seed_control("seed_badmarker");
 if(!strcmp(scenario,"healthy"))seed_control("seed_healthy");
 fixture=false;
 board_abort_latch_init();
 if(!strcmp(scenario,"abort_boot"))board_abort_latch_set();
 launchcore_persist_status_t boot=persistent_store_boot_begin();
 uint64_t generation=board_recovery_boot_generation();
 board_bootstrap_storage_probe_boot();
 bool faulty=!strcmp(scenario,"deleted")||!strcmp(scenario,"corrupt")||!strcmp(scenario,"deletedfloor")||!strcmp(scenario,"badmarker");
 if(faulty){assert(!persistent_store_controls_ready());assert(g_persistent_store_errors);return 0;}
 assert(boot==LAUNCHCORE_PERSIST_OK&&generation&&generation!=UINT64_MAX&&persistent_store_controls_ready());
 assert(board_recovery_floor_is_healthy());
 assert(persistent_store_runtime_start()==TX_SUCCESS);thread_present=true;TX_BYTE_POOL pool={0};assert(thread_comm_init(&pool)==TX_SUCCESS);
 board_recovery_bind_owner(read_outputs,prepare_outputs,finish_outputs);assert(board_bootstrap_storage_bind_owner(stop_owner));
 int cookie=0;SedsRouter*router=(SedsRouter*)&cookie;assert(flight_state_cache_init(router)==SEDS_OK);board_recovery_transport_set_router(router);
 if(!strcmp(scenario,"abort_boot")){
  assert(thread_comm_get_abort());prove_blocked();assert(!normal_applied);
 }else{
  assert(!thread_comm_get_abort());
  assert(thread_comm_set_flight_state(1)==TX_SUCCESS);prove_command(10000);assert(normal_applied==1);
  assert(thread_comm_request_manual_abort()==TX_SUCCESS);prove_blocked();assert(normal_applied==1);
 }
 /* No manual commissioning leg: one explicit fresh normal recovery clears
  * the actual Abort only after safe owner/cache/durable completion. */
 BoardRecoverySnapshot snapshot={0};(void)board_recovery_snapshot(&snapshot);
 BoardRecoveryRequest query={.operation=BOARD_RECOVERY_QUERY,.nonce=1,.issuer_session=77};
 query_req(router,&query,10000);assert(get64(response+12)==generation);
 BoardRecoveryRequest recover={.operation=BOARD_RECOVERY_RECOVER,.desired_state=1,.nonce=2,.issuer_session=77,.expected_boot_generation=generation,.expected_abort_epoch=snapshot.abort_epoch};
 send_req(router,&recover,10001);
 main_service();main_service();
 assert(response[1]==BOARD_RECOVERY_RESULT_OK&&persistent_store_controls_ready()&&!thread_comm_get_abort());
 assert(!board_abort_latch_is_set());
 prove_command(10002);assert(normal_applied==(!strcmp(scenario,"abort_boot")?1U:2U));
 return 0;
}
'''
class NormalBootAdmission(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.tmp=tempfile.TemporaryDirectory(prefix='normal-board-boot-');p=Path(cls.tmp.name)
  sdk=Path(os.environ['SEDSNET_TEST_SOURCE']);lc=configured_dependency_source(ROOT,'sedslaunchcore','SDK_FREE_LAUNCHCORE_SOURCE')
  fixture=ROOT/'tests/fixtures/durable_recovery'
  harness=(fixture/'harness.c').read_text().split('int main(int argc,char**argv){',1)[0]
  harness=harness.replace('state_mutex->locked&&desired==0','state_mutex->locked&&desired<=1')
  (p/'normal.c').write_text(harness+MAIN)
  modules=['board_abort_latch.c','thread_comm.c','board_recovery_wire.c','board_recovery_boot.c','board_recovery_transport.c','flight_state_cache.c','persistent_store.c','board_bootstrap_storage.c','bootstrap_marker.c']
  cls.exe=p/'normal';cls.image=p/'flash.bin'
  subprocess.run(['cc','-DEXPECTED_CAP=32','-DTELEMETRY_ENABLED=1','-DSAFE_MASK=6','-DRECOVERY_ENDPOINT=2001','-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-attributes','-fsanitize=address,undefined','-I'+str(fixture),'-I'+str(ROOT/'Core/Inc'),'-I'+str(ROOT/'Core/Src'),'-I'+str(sdk/'C-Headers'),'-I'+str(lc/'bootloader/include'),*[str(ROOT/'Core/Src'/n)for n in modules],str(ROOT/'Bootloader/storage_dispatch.c'),str(lc/'bootloader/src/persist.c'),str(lc/'bootloader/src/crc32.c'),str(p/'normal.c'),'-o',str(cls.exe)],check=True)
 @classmethod
 def tearDownClass(cls):cls.tmp.cleanup()
 def case(self,name):
  out=subprocess.run([str(self.exe),name,str(self.image)],capture_output=True,text=True)
  self.assertEqual(out.returncode,0,out.stdout+out.stderr)
 def test_virgin_boot_accepts_commands_then_abort_requires_explicit_recovery(self):self.case('virgin')
 def test_healthy_legacy_absent_metadata_initializes_without_commissioning(self):self.case('legacy')
 def test_healthy_existing_bookkeeping_preserves_normal_command_processing(self):self.case('healthy')
 def test_actual_abort_on_boot_is_not_cleared_by_initialization(self):self.case('abort_boot')
 def test_corrupt_deleted_keys_and_invalid_marker_remain_faulted(self):
  for case in ['deleted','corrupt','deletedfloor','badmarker']:
   with self.subTest(case=case):self.case(case)
