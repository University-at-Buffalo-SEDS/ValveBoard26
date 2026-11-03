"""Actual owner/recovery/report source under a held SDK call, using two host threads.

The SDK enqueue is a rejecting/accepting boundary fixture. This test does not
claim actual radio delivery, hardware outputs or recall of SDK-retained frames.
"""
from pathlib import Path
from native_sdk import configured_dependency_source
import os,re,subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class SafetyPublicationTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.tmp=tempfile.TemporaryDirectory(prefix='safety-publication-owner-');p=Path(cls.tmp.name)
  sdk=Path(os.environ['SEDSNET_TEST_SOURCE']);lc=configured_dependency_source(ROOT,'sedslaunchcore','SDK_FREE_LAUNCHCORE_SOURCE')
  fixtures=ROOT/'tests/fixtures/safety_report_publication';cls.exe=p/'test';cls.image=p/'nvm.bin'
  production=(ROOT/'Core/Src/telemetry_thread.c').read_text()
  reporter=re.search(r'static void telemetry_report_safety_abort\(void\)\s*\{.*?\n\}',production,re.S).group()
  (p/'harness.c').write_text((fixtures/'harness.c').read_text().replace('ACTUAL_REPORTER',reporter))
  modules=['board_abort_latch.c','thread_comm.c','board_recovery_wire.c','board_recovery_boot.c','board_recovery_transport.c','flight_state_cache.c','persistent_store.c','board_bootstrap_storage.c','bootstrap_marker.c']
  ab=ROOT.name=='ActuatorBoard26'
  if ab:modules+=['fill_settings.c','fill_motion_status.c']
  subprocess.run(['cc','-pthread','-DEXPECTED_CAP=32','-DTELEMETRY_ENABLED=1','-DSAFE_MASK='+('0'if ab else'6'),'-DRECOVERY_ENDPOINT='+('2000'if ab else'2001'),'-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(fixtures),'-I'+str(ROOT/'Core/Inc'),'-I'+str(ROOT/'Core/Src'),'-I'+str(sdk/'C-Headers'),'-I'+str(lc/'bootloader/include'),*[str(ROOT/'Core/Src'/name)for name in modules],str(ROOT/'Bootloader/storage_dispatch.c'),str(lc/'bootloader/src/persist.c'),str(lc/'bootloader/src/crc32.c'),str(p/'harness.c'),'-o',str(cls.exe)],check=True)
 @classmethod
 def tearDownClass(cls):cls.tmp.cleanup()
 def case(self,name):
  result=subprocess.run([str(self.exe),name,str(self.image)],capture_output=True,text=True,timeout=15)
  self.assertEqual(result.returncode,0,result.stdout+result.stderr)
 def test_held_publisher_blocks_recovery_completion_and_launch_acceptance(self):self.case('held_recovery_launch')
 def test_manual_stop_does_not_wait_for_sdk_and_invalidates_recovery(self):self.case('manual_stop')
 def test_new_real_cause_after_recovery_retains_first_diagnostic_and_utc_zero(self):self.case('new_cause')
 def test_old_finish_cannot_release_newer_lease_for_same_cause(self):self.case('stale_finish')
 def test_original_intent_expires_without_firing_or_rollback(self):self.case('lease_timeout')
