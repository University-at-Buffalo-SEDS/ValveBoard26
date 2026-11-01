"""Actual recovery/owner/cache/floor/latch C: cut power at each flash boundary.

The host NVM/MMIO fixtures model persistence and injected IO, not physical flash
or valve movement. Each crash exits once; Python starts exactly one reboot.
"""
from pathlib import Path
import ast,os,subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
MAIN=r'''#pragma once
#include <stdint.h>
struct RccFake{uint32_t APB1ENR1;};struct PwrFake{uint32_t CR1;};
struct TampFake{uint32_t BKP28R,BKP29R,BKP30R,BKP31R;};
extern struct RccFake test_rcc;extern struct PwrFake test_pwr;extern struct TampFake test_tamp;
#define RCC (&test_rcc)
#define PWR (&test_pwr)
#define TAMP (&test_tamp)
#define RCC_APB1ENR1_PWREN 1U
#define RCC_APB1ENR1_RTCAPBEN 2U
#define PWR_CR1_DBP 4U
void test_dsb(void);
#define __DSB() test_dsb()
uint32_t HAL_GetTick(void);
'''
PERSIST=r'''#pragma once
#include <stdint.h>
#include <stddef.h>
typedef enum{LAUNCHCORE_PERSIST_OK,LAUNCHCORE_PERSIST_NOT_FOUND,LAUNCHCORE_PERSIST_STORAGE_ERROR}launchcore_persist_status_t;
launchcore_persist_status_t persistent_store_init(void);
launchcore_persist_status_t persistent_store_get(uint32_t,void*,size_t*);
launchcore_persist_status_t persistent_store_set(uint32_t,const void*,size_t);
'''
HARNESS=r'''
#include "main.h"
#include "persistent_store.h"
#include "thread_comm.h"
#include "board_recovery.h"
#include "board_recovery_boot.h"
#include "board_recovery_transport.h"
#include "board_abort_latch.h"
#include "flight_state_cache.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
struct RccFake test_rcc;struct PwrFake test_pwr;struct TampFake test_tamp;
volatile uint32_t g_telemetry_discovery_seen;
static TX_MUTEX*state_mutex;static const char*scenario;static const char*image_path;
static unsigned floor_writes,state_writes,barrier_calls;static uint8_t backup_fault;
static uint64_t now=10000;static uint8_t response[40];
struct Record{uint32_t key;uint32_t length;uint8_t bytes[16];};
struct Image{struct TampFake backup;struct Record records[3];};
static struct Image image={.records={{0x46535445,1,{1}},{0x42524745,8,{41}},{0x4252464c,0,{0}}}};
static SedsResult(*update_callback)(const SedsPacketView*,void*);
static struct Record*record(uint32_t key){for(unsigned i=0;i<3;i++)if(image.records[i].key==key)return &image.records[i];assert(0);return NULL;}
static void save_crash(void){image.backup=test_tamp;FILE*f=fopen(image_path,"wb");assert(f&&fwrite(&image,sizeof image,1,f)==1&&fclose(f)==0);_Exit(42);}
static void check_barrier(void){assert(state_mutex&&!state_mutex->locked);assert(thread_comm_get_abort());assert(board_abort_latch_is_set());assert(test_tamp.BKP30R==UINT32_C(0x41425254)&&test_tamp.BKP31R==~UINT32_C(0x41425254));}
void test_dsb(void){barrier_calls++;if(backup_fault){test_tamp.BKP30R=0;test_tamp.BKP31R=0;}}
uint32_t HAL_GetTick(void){return 123;}
launchcore_persist_status_t persistent_store_init(void){assert(!state_mutex||!state_mutex->locked);return LAUNCHCORE_PERSIST_OK;}
launchcore_persist_status_t persistent_store_set(uint32_t key,const void*data,size_t length){
 assert(!state_mutex||!state_mutex->locked);struct Record*r=record(key);assert(length<=16);
 if(key!=0x42524745){check_barrier();if(key==0x4252464c)floor_writes++;else state_writes++;}
 if((key==0x4252464c&&!strcmp(scenario,"floor_before"))||(key==0x46535445&&!strcmp(scenario,"state_before")))save_crash();
 memcpy(r->bytes,data,length);r->length=(uint32_t)length;
 if((key==0x4252464c&&!strcmp(scenario,"floor_after"))||(key==0x46535445&&!strcmp(scenario,"state_after")))save_crash();
 if((key==0x4252464c&&!strcmp(scenario,"abort_floor"))||(key==0x46535445&&!strcmp(scenario,"abort_state")))assert(thread_comm_request_manual_abort()==0);
 return LAUNCHCORE_PERSIST_OK;
}
launchcore_persist_status_t persistent_store_get(uint32_t key,void*data,size_t*length){
 assert(!state_mutex||!state_mutex->locked);struct Record*r=record(key);
 if((key==0x4252464c&&floor_writes&&!strcmp(scenario,"floor_read"))||(key==0x46535445&&state_writes&&!strcmp(scenario,"state_read")))save_crash();
 if(!r->length){return LAUNCHCORE_PERSIST_NOT_FOUND;}assert(*length>=r->length);*length=r->length;memcpy(data,r->bytes,r->length);return LAUNCHCORE_PERSIST_OK;
}
UINT tx_mutex_create(TX_MUTEX*m,const char*n,UINT i){(void)i;m->locked=0;if(!strcmp(n,"thread_comm_state_mutex"))state_mutex=m;return 0;}
UINT tx_mutex_get(TX_MUTEX*m,ULONG w){(void)w;assert(!m->locked);m->locked=1;return 0;}
UINT tx_mutex_put(TX_MUTEX*m){assert(m->locked);m->locked=0;return 0;}
UINT tx_semaphore_create(TX_SEMAPHORE*s,const char*n,ULONG count){(void)n;s->count=count;return 0;}
UINT tx_semaphore_get(TX_SEMAPHORE*s,ULONG w){(void)w;if(!s->count)return TX_NO_INSTANCE;s->count--;return 0;}
UINT tx_semaphore_put(TX_SEMAPHORE*s){s->count++;return 0;}
SedsResult seds_router_resolve_hostname_address(SedsRouter*r,const char*n,size_t size,uint32_t*out){(void)r;(void)n;(void)size;*out=1;return SEDS_OK;}
SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){(void)r;*out=now;return SEDS_OK;}
SedsResult seds_router_log_queue_typed(SedsRouter*r,SedsDataType ty,const void*data,size_t count,size_t size,SedsElemKind kind){(void)r;assert(ty==201&&count==40&&size==1&&kind==SEDS_EK_UNSIGNED);memcpy(response,data,40);return SEDS_OK;}
void seds_router_disable_managed_variable(SedsRouter*r,SedsDataType ty){(void)r;assert(!state_mutex->locked&&ty==SEDS_DT_FLIGHT_STATE);update_callback=NULL;}
SedsResult seds_router_enable_network_variable(SedsRouter*r,SedsDataType ty,bool read,bool write){(void)r;assert(!state_mutex->locked&&ty==SEDS_DT_FLIGHT_STATE&&read&&!write);return SEDS_OK;}
SedsResult seds_router_on_network_variable_update(SedsRouter*r,SedsDataType ty,SedsResult(*cb)(const SedsPacketView*,void*),void*user){(void)r;assert(ty==SEDS_DT_FLIGHT_STATE&&!user);update_callback=cb;return SEDS_OK;}
SedsResult seds_router_request_managed_variable(SedsRouter*r,SedsDataType ty){(void)r;(void)ty;return SEDS_OK;}
SedsResult flight_state_cache_apply_network_update(const SedsPacketView*p){return thread_comm_apply_managed_flight_state(p->payload[0])==TX_SUCCESS?SEDS_OK:SEDS_HANDLER_ERROR;}
static uint8_t read_outputs(uint32_t*out){assert(state_mutex->locked);*out=SAFE_MASK;return BOARD_RECOVERY_OUTPUTS_VALID|BOARD_RECOVERY_SAFE_OUTPUTS;}
static uint8_t prepare_outputs(uint8_t desired){assert(state_mutex->locked&&desired==0);return 1;}
static void finish_outputs(void){assert(state_mutex->locked);}
static void request(SedsRouter*r,uint8_t*body,uint64_t timestamp){uint32_t endpoint=RECOVERY_ENDPOINT;SedsPacketView p={.ty=200,.sender="GS",.sender_len=2,.endpoints=&endpoint,.num_endpoints=1,.payload=body,.payload_len=32,.timestamp=timestamp};assert(board_recovery_handle(r,&p,endpoint,123)==SEDS_OK);}
int main(int argc,char**argv){
 assert(argc==3);scenario=argv[1];image_path=argv[2];
 if(!strcmp(scenario,"reboot")){FILE*f=fopen(image_path,"rb");assert(f&&fread(&image,sizeof image,1,f)==1&&fclose(f)==0);test_tamp=image.backup;}
 board_abort_latch_init();TX_BYTE_POOL pool={0};assert(thread_comm_init(&pool)==0);
 if(!strcmp(scenario,"reboot")){assert(thread_comm_get_abort()&&board_abort_latch_is_set());assert(test_tamp.BKP30R||test_tamp.BKP31R);puts("reboot: persistent Abort barrier retained; no owner clear");return 0;}
 assert(!thread_comm_get_abort());assert(thread_comm_set_flight_state(1)==0);
 board_recovery_bind_owner(read_outputs,prepare_outputs,finish_outputs);
 int cookie=0;SedsRouter*r=(SedsRouter*)&cookie;assert(flight_state_cache_init(r)==SEDS_OK);board_recovery_transport_set_router(r);
 assert(board_recovery_boot_generation()==42);uint8_t body[32]={1,0,0,0};body[4]=1;body[24]=77;request(r,body,now);assert(response[1]==0&&board_recovery_boot_current()==42);
 body[1]=1;body[4]=2;body[12]=42;body[20]=1;
 if(!strcmp(scenario,"backup_repair"))backup_fault=1;
 request(r,body,now);
 if(!strcmp(scenario,"backup_repair")){
  assert(response[1]==7&&floor_writes==0&&state_writes==0&&thread_comm_get_abort());
  backup_fault=0;unsigned dsb=barrier_calls;request(r,body,now);assert(response[1]==7&&barrier_calls==dsb&&floor_writes==0&&state_writes==0);
  body[4]=3;now++;request(r,body,now);
 }
 if(!strcmp(scenario,"abort_floor")||!strcmp(scenario,"abort_state")){
  assert(response[1]==4&&thread_comm_get_abort());thread_comm_service_board_recovery();BoardRecoverySnapshot s;assert(board_recovery_snapshot(&s)==0&&s.abort_epoch==2&&(s.flags&BOARD_RECOVERY_ABORT_LATCHED));puts("new manual Stop during NVM wins; no clear");return 0;
 }
 assert(response[1]==1&&thread_comm_get_abort()&&floor_writes==1&&state_writes==1&&record(0x46535445)->bytes[0]==0);
 /* Before owner completion a fresh managed Aborted update cannot overwrite it. */
 uint8_t state=15;SedsPacketView update={.ty=SEDS_DT_FLIGHT_STATE,.sender="GS",.sender_len=2,.timestamp=now+1,.payload=&state,.payload_len=1};assert(update_callback&&update_callback(&update,NULL)!=SEDS_OK);
 thread_comm_service_board_recovery();request(r,body,now);assert(response[1]==0&&!thread_comm_get_abort()&&response[2]==0&&response[20]==2);
 assert(!test_tamp.BKP28R&&!test_tamp.BKP29R&&!test_tamp.BKP30R&&!test_tamp.BKP31R);
 assert(floor_writes==1&&state_writes==1);puts("verified barrier -> durable floor/state -> actual safe owner clear: PASS");return 0;
}
'''
class DurableRecoveryTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.tmp=tempfile.TemporaryDirectory(prefix='recovery-durable-');p=Path(cls.tmp.name)
  for name,text in [('main.h',MAIN),('persistent_store.h',PERSIST)]:(p/name).write_text(text)
  tree=ast.parse((ROOT/'tests/test_board_recovery_owner.py').read_text())
  tx=next(n.args[0].value for n in ast.walk(tree)if isinstance(n,ast.Call)and isinstance(n.func,ast.Attribute)and n.func.attr=='write_text'and n.args and isinstance(n.args[0],ast.Constant)and isinstance(n.args[0].value,str)and 'typedef struct{UINT locked;}TX_MUTEX'in n.args[0].value)
  (p/'tx_api.h').write_text(tx);(p/'harness.c').write_text(HARNESS);cls.exe=p/'test';cls.image=p/'power-cut.bin'
  sdk=Path(os.environ['SEDSNET_TEST_SOURCE']);ab=ROOT.name=='ActuatorBoard26'
  modules=['board_abort_latch.c','thread_comm.c','board_recovery_wire.c','board_recovery_boot.c','board_recovery_floor.c','board_recovery_transport.c','flight_state_cache.c']
  subprocess.run(['cc','-DTELEMETRY_ENABLED=1','-DSAFE_MASK='+('0'if ab else'6'),'-DRECOVERY_ENDPOINT='+('2000'if ab else'2001'),'-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(p),'-I'+str(ROOT/'Core/Inc'),'-I'+str(sdk/'C-Headers'),*[str(ROOT/'Core/Src'/m)for m in modules],str(p/'harness.c'),'-o',str(cls.exe)],check=True)
 @classmethod
 def tearDownClass(cls):cls.tmp.cleanup()
 def case(self,name,expected=0):
  result=subprocess.run([str(self.exe),name,str(self.image)],capture_output=True,text=True)
  self.assertEqual(result.returncode,expected,result.stdout+result.stderr)
 def test_successful_nonaborted_startup_uses_barrier_then_owner_clear(self):self.case('success')
 def test_backup_failure_never_writes_flash_same_nonce_no_retry_new_nonce_repairs(self):self.case('backup_repair')
 def test_manual_stop_during_floor_or_state_nvm_cancels_owner(self):
  for name in ['abort_floor','abort_state']:
   with self.subTest(name=name):self.case(name)
 def test_power_cut_before_after_and_readback_of_every_flash_record_retains_abort(self):
  for name in ['floor_before','floor_after','floor_read','state_before','state_after','state_read']:
   with self.subTest(name=name):self.case(name,42);self.case('reboot')
