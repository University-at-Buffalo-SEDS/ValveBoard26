"""Actual recovery-wire/storage/retained Launch exclusion; NVM/MMIO are fixtures."""
from pathlib import Path
from native_sdk import configured_dependency_source
import os,subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class RecoveryLaunchExclusionTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.tmp=tempfile.TemporaryDirectory(prefix="recovery-launch-exclusion-");p=Path(cls.tmp.name)
  source=Path(os.environ.get("RECOVERY_LAUNCH_TEST_SOURCE",str(ROOT)))
  sdk=Path(os.environ["SEDSNET_TEST_SOURCE"]);lc=configured_dependency_source(ROOT,"sedslaunchcore","SDK_FREE_LAUNCHCORE_SOURCE")
  cls.has_claim="board_recovery_stage_storage_hold" in (source/"Core/Inc/board_recovery.h").read_text()
  src=source/"Core/Src";fixtures=ROOT/"tests/fixtures/durable_recovery";new=ROOT/"tests/fixtures/recovery_launch_exclusion/harness.c"
  prefixes={"thread_comm.c":"#define persistent_store_hold_bootstrap audit_hold_bootstrap\n", "persistent_store.c":"#define persistent_store_publish_bootstrap_complete audit_real_publish_complete\n", "board_bootstrap_storage.c":"#define persistent_store_hold_bootstrap audit_hold_bootstrap\n#define board_recovery_apply_safe audit_apply_safe\n"+("#define board_recovery_stage_storage_hold audit_stage_storage_hold\n"if cls.has_claim else "")}
  modules=["board_abort_latch.c","thread_comm.c","board_recovery_wire.c","board_recovery_boot.c","board_recovery_transport.c","flight_state_cache.c","persistent_store.c","board_bootstrap_storage.c","bootstrap_marker.c"]
  ab=(source/"Core/Src/main_task.c").is_file()
  if ab:modules +=["fill_settings.c","fill_motion_status.c"]
  paths=[]
  for name in modules:
   if name not in prefixes:paths.append(str(src/name));continue
   body=(src/name).read_text()
   if name=="board_bootstrap_storage.c":body+='\nsize_t audit_job_copy(uint8_t*out,size_t capacity){size_t n=sizeof job+sizeof query;if(capacity<n)return 0;memcpy(out,&job,sizeof job);memcpy(out+sizeof job,&query,sizeof query);return n;}\n'
   (p/name).write_text(prefixes[name]+body);paths.append(str(p/name))
  cls.exe=p/"probe";cls.image=p/"image.nvm"
  subprocess.run(["cc","-DAUDIT_HAS_CLAIM="+str(int(cls.has_claim)),"-DEXPECTED_CAP=32","-DTELEMETRY_ENABLED=1","-DSAFE_MASK="+("0"if ab else"6"),"-DRECOVERY_ENDPOINT="+("2000"if ab else"2001"),"-std=c11","-Wall","-Wextra","-Werror","-fsanitize=address,undefined","-I"+str(fixtures),"-I"+str(source/"Core/Inc"),"-I"+str(src),"-I"+str(sdk/"C-Headers"),"-I"+str(lc/"bootloader/include"),*paths,str(source/"Bootloader/storage_dispatch.c"),str(lc/"bootloader/src/persist.c"),str(lc/"bootloader/src/crc32.c"),str(new),"-o",str(cls.exe)],check=True)
 @classmethod
 def tearDownClass(cls):cls.tmp.cleanup()
 def case(self,name,phase="pending"):
  result=subprocess.run([str(self.exe),name,str(self.image),phase],capture_output=True,text=True,timeout=20)
  self.assertEqual(result.returncode,0,result.stdout+result.stderr)
 def test_accepted_launch_all_phases_reject_delayed_recovery_without_changes(self):
  for phase in ["pending","taken","running","completed"]:
   with self.subTest(phase=phase):self.case("audit_accepted_first",phase)
 def test_actual_stage_linearization_both_orders(self):
  for name in ["audit_stage_wins","audit_interleave","audit_after_claim"]:
   with self.subTest(name=name):self.case(name)
 def test_direct_preparation_storage_cleanup_and_release_fault_preserve_launch(self):
  for name in ["audit_direct_prepare","audit_direct_storage","audit_direct_cleanup","audit_release_error","audit_marker_loss","audit_terminal_cleanup"]:
   with self.subTest(name=name):self.case(name)
 def test_idle_recovery_and_manual_stop_remain_valid_stale_intents_refuse(self):
  for name in ["audit_idle","audit_manual_stop","audit_manual_recover","audit_stale_source","audit_stale_epoch"]:
   with self.subTest(name=name):self.case(name)
 def test_busy_atomic_stage_never_holds_or_installs_job(self):
  self.assertTrue(self.has_claim,"Current firmware must expose atomic stage claim")
  self.case("audit_claim_busy")
if __name__=="__main__":unittest.main()
