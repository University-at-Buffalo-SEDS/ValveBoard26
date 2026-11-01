"""Verified persisted command floor survives reboot and fails closed on IO."""
from pathlib import Path
import subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class RecoveryFloorTests(unittest.TestCase):
 def test_actual_floor_readback_reboot_repair_and_failed_write(self):
  with tempfile.TemporaryDirectory()as directory:
   w=Path(directory)
   (w/'persistent_store.h').write_text('#pragma once\n#include <stdint.h>\n#include <stddef.h>\ntypedef enum {LAUNCHCORE_PERSIST_OK,LAUNCHCORE_PERSIST_NOT_FOUND,LAUNCHCORE_PERSIST_STORAGE_ERROR} launchcore_persist_status_t;\nlaunchcore_persist_status_t persistent_store_init(void);\nlaunchcore_persist_status_t persistent_store_get(uint32_t,void*,size_t*);\nlaunchcore_persist_status_t persistent_store_set(uint32_t,const void*,size_t);\n')
   source=r"""
#include <assert.h>
#include <string.h>
#include "persistent_store.h"
static unsigned writes,reads,exists,io,wrong_readback;static uint8_t storage[16];static size_t stored_length=16;
launchcore_persist_status_t persistent_store_init(void){return LAUNCHCORE_PERSIST_OK;}
launchcore_persist_status_t persistent_store_get(uint32_t key,void*out,size_t*len){assert(key==0x4252464c);reads++;if(!exists)return LAUNCHCORE_PERSIST_NOT_FOUND;memcpy(out,storage,16);if(wrong_readback)((uint8_t*)out)[8]^=1;*len=stored_length;return LAUNCHCORE_PERSIST_OK;}
launchcore_persist_status_t persistent_store_set(uint32_t key,const void*in,size_t length){assert(key==0x4252464c&&length==16);writes++;if(io)return LAUNCHCORE_PERSIST_STORAGE_ERROR;memcpy(storage,in,16);exists=1;stored_length=16;return LAUNCHCORE_PERSIST_OK;}
PRODUCTION
int main(void){
 assert(board_recovery_command_time_allowed(0));assert(writes==0);
 assert(board_recovery_floor_commit(77,100));assert(writes==1);
 assert(!board_recovery_floor_commit(77,99)&&!board_recovery_floor_commit(77,100)&&writes==1);
 assert(!board_recovery_command_time_allowed(0)&&!board_recovery_command_time_allowed(99)&&!board_recovery_command_time_allowed(100)&&board_recovery_command_time_allowed(101));
 unsigned old_reads=reads;for(unsigned i=0;i<10000;i++)assert(board_recovery_command_time_allowed(101));assert(reads==old_reads&&writes==1);
 floor_state=0;floor_timestamp=0;assert(!board_recovery_command_time_allowed(100)&&board_recovery_command_time_allowed(101));assert(writes==1);
 stored_length=8;floor_state=0;assert(!board_recovery_command_time_allowed(UINT64_MAX));
 /* Authenticated fresh recovery is separately required before this repair API. */
 assert(!board_recovery_floor_commit(88,50));assert(board_recovery_floor_commit(88,150));assert(!board_recovery_command_time_allowed(150)&&board_recovery_command_time_allowed(151));
 uint8_t previous[16];memcpy(previous,storage,16);io=1;assert(!board_recovery_floor_commit(99,200));assert(!memcmp(previous,storage,16));assert(!board_recovery_command_time_allowed(UINT64_MAX));
 io=0;wrong_readback=1;assert(!board_recovery_floor_commit(99,201));assert(!board_recovery_command_time_allowed(UINT64_MAX));
 wrong_readback=0;assert(board_recovery_floor_commit(99,202));assert(!board_recovery_command_time_allowed(202)&&board_recovery_command_time_allowed(203));
 unsigned old_writes=writes;assert(!board_recovery_floor_commit(0,201)&&!board_recovery_floor_commit(99,0));assert(writes==old_writes);
 return 0;
}
""".replace('PRODUCTION',(ROOT/'Core/Src/board_recovery_floor.c').read_text())
   (w/'test.c').write_text(source);exe=w/'test'
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(w),'-I'+str(ROOT/'Core/Inc'),str(w/'test.c'),'-o',str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
