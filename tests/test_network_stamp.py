"""A network stamp key orders exactly like SEDSnet's seds_stamp_cmp().

Stored ordering marks (replay floor, command cutoff, Launch owner, recovery
sources) keep one u64: the time epoch above a 47-bit millisecond timestamp.
An epoch 0 key is the plain timestamp, so records written before epochs read
as epoch 0, and 0 / UINT64_MAX keep their meanings.
"""
from pathlib import Path
import os,subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
CODE=r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include "sedsnet.h"
#include "network_stamp.h"
int main(void){
 const uint16_t epochs[]={0,1,2,7,65535};
 const uint64_t times[]={1,999,1794183589988ULL,1794183589988ULL-4ULL*3600000ULL,1794183589988ULL+13680000ULL,NETWORK_STAMP_MS_MASK};
 for(size_t a=0;a<5;a++)for(size_t b=0;b<6;b++)for(size_t c=0;c<5;c++)for(size_t d=0;d<6;d++){
  const uint64_t x=network_stamp_key(epochs[a],times[b]),y=network_stamp_key(epochs[c],times[d]);
  const int cmp=seds_stamp_cmp(epochs[a],times[b],epochs[c],times[d]);
  assert((cmp<0)==(x<y)&&(cmp==0)==(x==y)&&(cmp>0)==(x>y));
  assert(x&&x!=UINT64_MAX&&network_stamp_epoch(x)==epochs[a]&&network_stamp_ms(x)==times[b]);
 }
 assert(network_stamp_key(0,1794183589988ULL)==1794183589988ULL);
 assert(network_stamp_key(3,0)==0&&network_stamp_key(0,UINT64_MAX)==UINT64_MAX&&network_stamp_key(1,NETWORK_STAMP_MS_MASK+1U)==UINT64_MAX);
 SedsPacketView v={0};v.timestamp=1234;v.time_epoch=2;assert(network_stamp_of(&v)==network_stamp_key(2,1234));
 return 0;}
'''
class NetworkStampKey(unittest.TestCase):
 def test_key_order_matches_seds_stamp_cmp(self):
  sdk=Path(os.environ['SEDSNET_TEST_SOURCE'])
  with tempfile.TemporaryDirectory() as d:
   exe=Path(d)/'stamp'
   out=subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(ROOT/'Core/Inc'),'-I'+str(sdk/'C-Headers'),'-x','c','-','-o',str(exe)],input=CODE,text=True,capture_output=True)
   self.assertEqual(out.returncode,0,out.stderr);subprocess.run([str(exe)],check=True,timeout=10)
if __name__=='__main__':unittest.main()
