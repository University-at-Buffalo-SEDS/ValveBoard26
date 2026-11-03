"""Actual OTA/store/Stop owners and actual SDK P2P protocol; no hardware effects.

LaunchCore delta operation bodies are controlled success/fault boundaries.
These tests prove receiver ownership, not physical flash stalls or image safety.
"""
from pathlib import Path
import os,subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class OtaResponsiveness(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.tmp=tempfile.TemporaryDirectory(prefix='ota-actual-owner-');cls.work=Path(cls.tmp.name)
  cls.env=dict(os.environ,OTA_OWNER_SOURCE=str(ROOT),OTA_OWNER_WORK=str(cls.work),ASAN_OPTIONS='detect_leaks=1:abort_on_error=1',UBSAN_OPTIONS='halt_on_error=1')
  helper=ROOT/'tests/ota_owner'
  for script,args in [('support_fixture.py',['ValveBoard26','candidate']),('runtime_fixture.py',[]),('sdk_uploader_fixture.py',[]),('lc_abort_fixture.py',[])]:
   subprocess.run(['python3',str(helper/script),*args],check=True,env=cls.env)
 @classmethod
 def tearDownClass(cls):cls.tmp.cleanup()
 def test_actual_pending_reply_cleanup_epoch_and_reset_owners(self):
  for case in ['foreign','busy_stop','payload_reply','terminal_busy','cleanup_failure','aba','armed','restore','launch_after_finish','reset_aba','safe_reset','recovery_after_launch','bounds','terminal_request','terminal_reply','generation_exhaustion','reset_release','physical_release','init_release','cleanup_release','boot_release','boot_fault','cleanup_readback','cleanup_commit_lost','cleanup_exhausted','cleanup_nondelta']:
   with self.subTest(case=case):subprocess.run([str(self.work/'native/runtime/test'),case],check=True,env=self.env,timeout=10)
 def test_actual_launchcore_abort_commit_failure_is_not_success(self):
  for case in ['old_write','old_read','new_write','new_read','positive']:
   with self.subTest(case=case):subprocess.run([str(self.work/'native/lc-abort/test'),case],check=True,env=self.env,timeout=10)
 def test_actual_sdk_single_request_upload_with_busy_store(self):
  subprocess.run([str(self.work/'native/sdk-uploader/test')],check=True,env=self.env,timeout=15)
if __name__=='__main__':unittest.main()
