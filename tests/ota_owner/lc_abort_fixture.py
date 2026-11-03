"""Actual LaunchCore e2 commit/read fault controls for the app readback helper."""
from pathlib import Path
import os,sys,subprocess,importlib.util,json
HERE=Path(__file__).resolve().parent;ROOT=Path(os.environ['OTA_OWNER_SOURCE']);P=Path(os.environ['OTA_OWNER_WORK'])/'native/lc-abort';P.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,str(ROOT/'tests'));from native_sdk import configured_dependency_source
LC=configured_dependency_source(ROOT,'sedslaunchcore','SDK_FREE_LAUNCHCORE_SOURCE')
spec=importlib.util.spec_from_file_location('support',HERE/'support_fixture.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
s=(LC/'tests/unit/test_delta_update.c').read_text();s=s[:s.index('static uint32_t make_image(')]
s=s.replace('static bool initialized;','static bool initialized,fail_metadata_write,fail_read_after_bad,bad_written;static uint32_t delta_erased;')
s=s.replace('memset(flash + address, 0xFF, size);','if(address==SLOT_B_BASE){delta_erased+=size;}memset(flash + address, 0xFF, size);')
s=s.replace('    memcpy(flash + address, data, size);','    if(fail_metadata_write&&address<SLOT_A_BASE)return LAUNCHCORE_STORAGE_ERR_WRITE;\n    if(address<SLOT_A_BASE&&size==sizeof(launchcore_metadata_t)&&((const launchcore_metadata_t*)data)->slot_b_state==LAUNCHCORE_SLOT_BAD)bad_written=true;\n    memcpy(flash + address, data, size);')
s=s.replace('    memcpy(data, flash + address, size);','    if(fail_read_after_bad&&bad_written&&address<SLOT_A_BASE)return LAUNCHCORE_STORAGE_ERR_READ;\n    memcpy(data, flash + address, size);')
s+='\n'+m.function((ROOT/'Core/Src/ota_stream.c').read_text(),'abort_verified')+r'''
#include <stdio.h>
int main(int argc,char**argv){
 assert(argc==2);launchcore_storage_set_driver(&driver);assert(launchcore_storage_init()==LAUNCHCORE_STORAGE_OK);
 launchcore_metadata_t md;launchcore_metadata_init_default(&md);md.active_slot=md.confirmed_slot=LAUNCHCORE_SLOT_A;md.slot_a_state=LAUNCHCORE_SLOT_CONFIRMED;assert(launchcore_metadata_commit(&md)==LAUNCHCORE_METADATA_OK);
 assert(launchcore_delta_update_begin(sizeof(launchcore_delta_header_t))==LAUNCHCORE_OK);assert(delta_erased==SLOT_B_SIZE&&delta_erased>sizeof(launchcore_delta_header_t));
 assert(launchcore_metadata_read_latest(&md)==LAUNCHCORE_METADATA_OK&&md.slot_b_state==LAUNCHCORE_SLOT_WRITING);uint32_t before=md.sequence;
 bool old=!strncmp(argv[1],"old",3),writefault=strstr(argv[1],"write")!=NULL,readfault=strstr(argv[1],"read")!=NULL;fail_metadata_write=writefault;fail_read_after_bad=readfault;
 launchcore_status_t result=old?launchcore_delta_update_abort():abort_verified();
 if(old){assert((writefault||readfault)&&result==LAUNCHCORE_OK);puts("OLD RED: actual e2 abort returns OK despite metadata commit/write/read fault");}
 else if(writefault||readfault){assert(result!=LAUNCHCORE_OK);puts("NEW GREEN: actual app readback rejects false cleanup success from e2 abort");}
 else {assert(result==LAUNCHCORE_OK);puts("GREEN: actual e2 abort+exact sequence/content readback proves cleanup");}
 fail_read_after_bad=false;assert(launchcore_metadata_read_latest(&md)==LAUNCHCORE_METADATA_OK);
 if(writefault)assert(md.slot_b_state==LAUNCHCORE_SLOT_WRITING&&md.sequence==before);
 else assert(md.slot_b_state==LAUNCHCORE_SLOT_BAD&&md.update_slot==LAUNCHCORE_SLOT_NONE&&md.sequence==before+1U);
 printf("Observed BEGIN erased entire delta slot: %u bytes for %zu-byte declaration; no time bound inferred\n",delta_erased,sizeof(launchcore_delta_header_t));return 0;
}
'''
f=P/'test.c';f.write_text(s);cmd=['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-omit-frame-pointer','-ffunction-sections','-fdata-sections','-Wl,--gc-sections','-I'+str(LC/'bootloader/include'),'-I'+str(LC/'update_lib/include'),str(f),str(LC/'update_lib/src/delta_update.c'),str(LC/'bootloader/src/storage.c'),str(LC/'bootloader/src/metadata.c'),str(LC/'bootloader/src/crc32.c'),'-o',str(P/'test')];subprocess.run(cmd,check=True);(P/'BUILD.json').write_text(json.dumps({'command':cmd,'limits':'Actual e2 delta_begin/abort/metadata/CRC/store with native memory driver, injected storage faults. App helper extracted exactly. No hardware timing or flash fetch-stall model.'},indent=2)+'\n')
