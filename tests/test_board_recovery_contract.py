"""Application recovery wire and verified boot-generation contracts."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
class RecoveryContractTests(unittest.TestCase):
 def compile(self,work,source,modules):
  (work/'test.c').write_text(source)
  exe=work/'test'
  subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(work),'-I'+str(ROOT/'Core/Inc'),str(work/'test.c'),*[str(ROOT/'Core/Src'/m)for m in modules],'-o',str(exe)],check=True)
  return exe
 def test_request_shape_and_status_offsets(self):
  with tempfile.TemporaryDirectory() as directory:
   work=Path(directory)
   exe=self.compile(work,r"""
#include <assert.h>
#include <string.h>
#include "board_recovery_wire.h"
int main(void) {
 uint8_t request[33]={1,1,0,0};request[4]=17;request[12]=23;request[20]=31;request[24]=42;
 BoardRecoveryRequest decoded={0};assert(board_recovery_decode_request(request,32,&decoded));
 assert(decoded.nonce==17&&decoded.expected_boot_generation==23&&decoded.expected_abort_epoch==31&&decoded.issuer_session==42);
 for(size_t n=0;n<33;n++)if(n!=32)assert(!board_recovery_decode_request(request,n,&decoded));
 for(unsigned state=2;state<256;state++){request[2]=(uint8_t)state;assert(!board_recovery_decode_request(request,32,&decoded));}request[2]=1;
 for(unsigned field=0;field<4;field++){uint8_t old=request[field];request[field]=255;assert(!board_recovery_decode_request(request,32,&decoded));request[field]=old;}
 const unsigned offsets[]={4,12,20,24};for(unsigned i=0;i<4;i++){unsigned offset=offsets[i];uint8_t old=request[offset];request[offset]=0;assert(!board_recovery_decode_request(request,32,&decoded));request[offset]=old;}
 assert(!board_recovery_decode_request(0,32,&decoded));assert(!board_recovery_decode_request(request,32,0));
 assert(board_recovery_decode_request(request,32,&decoded));uint8_t status[42];memset(status,0xaa,sizeof status);
 board_recovery_encode_status(status+1,4,15,31,&decoded,UINT64_C(0x8877665544332211),0x44332211,0x88776655,6);
 assert(status[0]==0xaa&&status[41]==0xaa);assert(status[1]==1&&status[2]==4&&status[3]==15&&status[4]==31);
 assert(status[5]==17&&status[13]==0x11&&status[20]==0x88&&status[21]==0x11&&status[24]==0x44);
 assert(status[25]==0x55&&status[28]==0x88&&status[29]==6&&status[33]==42);
 return 0;
}
""",['board_recovery_wire.c'])
   subprocess.run([str(exe)],check=True)
 def test_verified_generation_no_reuse_and_storage_fail_closed(self):
  with tempfile.TemporaryDirectory() as directory:
   work=Path(directory)
   (work/'persistent_store.h').write_text('#include <stdint.h>\n#include <stddef.h>\ntypedef enum {LAUNCHCORE_PERSIST_OK=0,LAUNCHCORE_PERSIST_NOT_FOUND=1,LAUNCHCORE_PERSIST_INVALID_ARGUMENT=2,LAUNCHCORE_PERSIST_NO_SPACE=3,LAUNCHCORE_PERSIST_STORAGE_ERROR=4,LAUNCHCORE_PERSIST_CORRUPT=5} launchcore_persist_status_t;\nlaunchcore_persist_status_t persistent_store_init(void);\nlaunchcore_persist_status_t persistent_store_get(uint32_t,void*,size_t*);\nlaunchcore_persist_status_t persistent_store_set(uint32_t,const void*,size_t);\n')
   exe=self.compile(work,r"""
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include "persistent_store.h"
#include "board_recovery_boot.h"
static unsigned scenario,reads,writes;static uint64_t value;
launchcore_persist_status_t persistent_store_init(void){return scenario==1?LAUNCHCORE_PERSIST_STORAGE_ERROR:LAUNCHCORE_PERSIST_OK;}
launchcore_persist_status_t persistent_store_get(uint32_t key,void*out,size_t*length){assert(key==0x42524745);reads++;if(scenario==2)return LAUNCHCORE_PERSIST_CORRUPT;if(scenario==0)return LAUNCHCORE_PERSIST_NOT_FOUND;if(scenario==3)*length=7;else *length=8;uint64_t result=(scenario==6&&reads==2)?value-1:value;memcpy(out,&result,8);return LAUNCHCORE_PERSIST_OK;}
launchcore_persist_status_t persistent_store_set(uint32_t key,const void*in,size_t length){assert(key==0x42524745&&length==8);writes++;if(scenario==5)return LAUNCHCORE_PERSIST_STORAGE_ERROR;memcpy(&value,in,8);return LAUNCHCORE_PERSIST_OK;}
int main(int argc,char**argv){assert(argc==2);scenario=(unsigned)atoi(argv[1]);value=scenario==4?UINT64_MAX:scenario==7?0:41;uint64_t expected=scenario==8?42:0;assert(board_recovery_boot_current()==0&&reads==0&&writes==0);assert(board_recovery_boot_generation()==expected);unsigned savedreads=reads,savedwrites=writes;if(scenario==6){assert(value==42&&writes==1);assert(board_recovery_boot_generation()==43&&value==43&&writes==2);savedreads=reads;savedwrites=writes;assert(board_recovery_boot_generation()==43&&reads==savedreads&&writes==savedwrites);}else if(expected){assert(board_recovery_boot_generation()==expected);assert(reads==savedreads&&writes==savedwrites);}else{if(scenario==0)assert(writes==0);assert(board_recovery_boot_current()==0);assert(board_recovery_boot_generation()==0);}return 0;}
""",['board_recovery_boot.c'])
   for scenario in range(9):subprocess.run([str(exe),str(scenario)],check=True)
