"""Managed metadata cannot create a new Abort after completed inhibition.

Actual owner/latch source, retained warm-reset marker, register loss/torn marker,
no new physical authority. Full backup-domain loss is explicitly not persistence.
"""
from pathlib import Path
from board_repo import ACTUATOR_BOARD,VALVE_BOARD,board_name
import ast,subprocess,tempfile,unittest
from test_verified_launch_latch import MAIN,HARNESS,function,owner_source
ROOT=Path(__file__).resolve().parents[1]
class ManagedLaunchMetadataTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.tmp=tempfile.TemporaryDirectory(prefix='managed-launch-metadata-');p=Path(cls.tmp.name);cls.exe=p/'test';cls.image=p/'marker.bin';(p/'main.h').write_text(MAIN)
  tree=ast.parse((ROOT/'tests/test_board_recovery_owner.py').read_text());tx=next(n.args[0].value for n in ast.walk(tree)if isinstance(n,ast.Call)and isinstance(n.func,ast.Attribute)and n.func.attr=='write_text'and n.args and isinstance(n.args[0],ast.Constant)and isinstance(n.args[0].value,str)and 'typedef struct{UINT locked;}TX_MUTEX'in n.args[0].value);(p/'tx_api.h').write_text(tx)
  code=HARNESS.replace('static unsigned drop_launch;','static unsigned drop_launch,dsb_writes;').replace('void test_dsb(void){','void test_dsb(void){dsb_writes++;')
  code=code.replace(' board_abort_latch_init();TX_BYTE_POOL',' if(!strcmp(scenario,"warm_metadata")||!strcmp(scenario,"cold_metadata")||!strcmp(scenario,"torn_metadata"))load(argv[2]);\n if(!strcmp(scenario,"cold_metadata"))memset(&test_tamp,0,sizeof test_tamp);\n if(!strcmp(scenario,"torn_metadata"))test_tamp.BKP29R=0;\n board_abort_latch_init();TX_BYTE_POOL')
  marker=' assert(!thread_comm_get_abort());assert(thread_comm_set_flight_state(6)==0);';assert marker in code;code=code.replace(marker,marker+(ROOT/'tests/fixtures/managed_launch_metadata/cases.inc').read_text())
  vb=board_name(ROOT)==VALVE_BOARD
  if vb:
   owner=(ROOT/'Core/Src/main_thread.c').read_text();code=code.replace('ACTUAL_OWNER',owner_source(owner))
  else:code=code.replace('ACTUAL_OWNER','')
  (p/'test.c').write_text(code)
  subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',*(['-DVB=1','-DTELEMETRY_ENABLED=1']if vb else[]),'-I'+str(p),'-I'+str(ROOT/'Core/Inc'),str(ROOT/'Core/Src/board_abort_latch.c'),str(ROOT/'Core/Src/thread_comm.c'),str(p/'test.c'),'-o',str(cls.exe)],check=True)
 @classmethod
 def tearDownClass(cls):cls.tmp.cleanup()
 def case(self,name):
  p=subprocess.run([str(self.exe),name,str(self.image)],capture_output=True,text=True);self.assertEqual(p.returncode,0,p.stdout+p.stderr)
 def test_later_register_corruption_cannot_create_abort_from_metadata_or_refire(self):self.case('accepted_metadata')
 def test_verified_warm_marker_preserves_metadata_inhibition_after_register_loss(self):self.case('seed_metadata');self.case('warm_metadata')
 def test_backup_domain_loss_does_not_borrow_old_completed_marker(self):self.case('seed_metadata');self.case('cold_metadata')
 def test_torn_retained_marker_does_not_borrow_verified_authority(self):self.case('seed_metadata');self.case('torn_metadata')
