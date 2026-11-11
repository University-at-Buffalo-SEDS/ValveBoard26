#include "board_recovery_floor.c"

#include "main.h"
#include "persistent_store.h"
#include "thread_comm.h"
#include "board_recovery.h"
#include "board_recovery_boot.h"
#include "board_recovery_transport.h"
#include "board_abort_latch.h"
#include "flight_state_cache.h"
#include <assert.h>
/* Deterministically refused requests are acknowledged and counted. */
static __attribute__((unused)) uint32_t refusals_expected;
#define REFUSED(call) (refusals_expected=g_reliable_refusal_count,(call)==SEDS_OK&&g_reliable_refusal_count==refusals_expected+1U)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
struct RccFake test_rcc;struct PwrFake test_pwr;struct TampFake test_tamp;
volatile uint32_t g_telemetry_discovery_seen;
static TX_MUTEX*state_mutex;static const char*scenario;static const char*image_path;
static unsigned floor_writes,state_writes,barrier_calls;static uint8_t backup_fault;
static bool unsafe_outputs,native_cache_fault,native_floor_fault;
static uint64_t now=10000;static uint8_t response[40];static unsigned responses;static unsigned media_writes,media_erases;static bool send_failure;static bool main_owner;static ULONG ticks;static unsigned phase,phase_writes,phase_reads,cut_write,cut_read,cut_bytes;static bool completed_owner,stop_injected;
ULONG tx_time_get(void){return ticks;}

#include "launchcore/storage.h"
#include "board_recovery_wire.h"
#include "persistent_store_health.h"
#include "board_bootstrap_storage.h"
#include "bootstrap_owner.h"
#include "bootstrap_marker.h"
#define NVM_SIZE 4096U
static bool fixture=true,thread_present,request_inflight,owner_stopped=true;
static TX_MUTEX *store_mutex,*jobs_lock;static bool floor_held,saw_complete,floor_busy_injected;static unsigned first_verified_unlocks,complete_verified_unlocks;
static bool fail_state_get,fail_state_put,fail_jobs_get,fail_jobs_put;
static bool fail_store_lock,fail_store_unlock,injected_identity_race;
static unsigned read_calls,fail_read,safe_readbacks,finished,identity_writes,last_safe_marker,write_barrier_marker;
static uint32_t identity_at=UINT32_MAX;
static SedsResult(*update_callback)(const SedsPacketView*,void*);
static uint32_t floor_at=UINT32_MAX,state_at=UINT32_MAX;
struct Image{struct TampFake backup;uint8_t nvm[NVM_SIZE];unsigned completed_witness;};
static struct Image image;
static uint32_t u32(const void*p){uint32_t v;memcpy(&v,p,4);return v;}
static bool sdk_unlocked(void){return (!store_mutex||!store_mutex->locked)&&(!state_mutex||!state_mutex->locked)&&(!jobs_lock||!jobs_lock->locked);}
static void save_crash(void){image.backup=(struct TampFake){0};image.completed_witness=completed_owner;FILE*f=fopen(image_path,"wb");assert(f&&fwrite(&image,sizeof image,1,f)==1&&fclose(f)==0);_Exit(42);}
static void check_barrier(void){assert(state_mutex&&!state_mutex->locked);assert(thread_comm_get_abort());assert(board_abort_latch_is_set());assert(test_tamp.BKP30R==UINT32_C(0x41425254)&&test_tamp.BKP31R==~UINT32_C(0x41425254));if(!write_barrier_marker){assert(barrier_calls&&last_safe_marker==barrier_calls);write_barrier_marker=barrier_calls;}assert(last_safe_marker>=write_barrier_marker);}
void test_dsb(void){barrier_calls++;if(backup_fault){test_tamp.BKP30R=0;test_tamp.BKP31R=0;}}
uint32_t HAL_GetTick(void){return 123;}
void *tx_thread_identify(void){return thread_present?(void*)1:NULL;}
static const launchcore_storage_layout_t geometry={.persistent_data_base=0,.persistent_data_size=NVM_SIZE,.persistent_data_erase_size=NVM_SIZE/2,.persistent_data_write_size=8};
static launchcore_storage_status_t init_storage(void){return LAUNCHCORE_STORAGE_OK;}
static const launchcore_storage_layout_t*get_layout(void){return &geometry;}
static launchcore_storage_status_t read_storage(uint32_t a,void*p,uint32_t n){
 assert(a<=NVM_SIZE&&n<=NVM_SIZE-a);assert(fixture||!thread_present||(store_mutex&&store_mutex->locked));
 if(++read_calls==fail_read)return LAUNCHCORE_STORAGE_ERR_READ;
 if(request_inflight){
  phase_reads++;if(cut_read&&phase_reads==cut_read){fprintf(stderr,"INJECT read phase=%u index=%u offset=%u size=%u\n",phase,phase_reads,a,n);if(!strcmp(scenario,"read_fault"))return LAUNCHCORE_STORAGE_ERR_READ;save_crash();}
  if(identity_writes&&!strcmp(scenario,"identity_read")&&a<=identity_at+31&&a+n>identity_at)save_crash();
  if(floor_writes&&!strcmp(scenario,"floor_read")&&a<=floor_at+39&&a+n>floor_at)save_crash();
  if(state_writes&&!strcmp(scenario,"state_read")&&a<=state_at+31&&a+n>state_at)save_crash();
 }
 memcpy(p,image.nvm+a,n);return LAUNCHCORE_STORAGE_OK;
}
static launchcore_storage_status_t erase_storage(uint32_t a,uint32_t n){media_erases++;
 assert(a<=NVM_SIZE&&n<=NVM_SIZE-a);assert(fixture||!thread_present||(store_mutex&&store_mutex->locked));
 if(request_inflight){if(phase==1)assert(main_owner);if(!completed_owner)check_barrier();if(!strcmp(scenario,"erase_before"))save_crash();}
 memset(image.nvm+a,255,n);return LAUNCHCORE_STORAGE_OK;
}
static launchcore_storage_status_t write_storage(uint32_t a,const void*p,uint32_t n){media_writes++;
 assert(a<=NVM_SIZE&&n<=NVM_SIZE-a);assert(fixture||!thread_present||(store_mutex&&store_mutex->locked));
 uint32_t key=n>=24&&u32(p)==UINT32_C(0x4c435052)?u32((const uint8_t*)p+4):0;
 if(key==BOOTSTRAP_MARKER_KEY&&n>=56&&((const uint8_t*)p)[25]==0)saw_complete=true;
 if(request_inflight){if(phase==1)assert(main_owner);if(!completed_owner)check_barrier();
  if(key==0x42524745U){identity_writes++;identity_at=a;
  if(!strcmp(scenario,"identity_before"))save_crash();
  if(!strcmp(scenario,"identity_fault"))return LAUNCHCORE_STORAGE_ERR_WRITE;
  if(!strcmp(scenario,"silent_identity"))return LAUNCHCORE_STORAGE_OK;
 }}
 if(request_inflight&&(key==UINT32_C(0x4252464c)||key==UINT32_C(0x46535445))){
  if(!completed_owner)check_barrier();
  if(key==UINT32_C(0x4252464c)){floor_writes++;floor_at=a;}else{state_writes++;state_at=a;}
  if((key==UINT32_C(0x4252464c)&&!strcmp(scenario,"floor_before"))||(key==UINT32_C(0x46535445)&&!strcmp(scenario,"state_before")))save_crash();
  if(key==UINT32_C(0x4252464c)&&(!strcmp(scenario,"floor_fault")||native_floor_fault))return LAUNCHCORE_STORAGE_ERR_WRITE;
  if(key==UINT32_C(0x4252464c)&&!strcmp(scenario,"silent_floor"))return LAUNCHCORE_STORAGE_OK;
 }
 if(request_inflight){
  phase_writes++;
  if(!strcmp(scenario,"silent_write")&&cut_write==phase_writes){fprintf(stderr,"INJECT silent phase=%u index=%u offset=%u size=%u\n",phase,phase_writes,a,n);return LAUNCHCORE_STORAGE_OK;}
  if(!strcmp(scenario,"cut")&&cut_write==phase_writes){fprintf(stderr,"INJECT write phase=%u index=%u offset=%u size=%u bytes=%u\n",phase,phase_writes,a,n,cut_bytes);
   fprintf(stderr,"INJECT write phase=%u index=%u offset=%u size=%u bytes=%u\n",phase,phase_writes,a,n,cut_bytes);unsigned bytes=cut_bytes<n?cut_bytes:n;for(unsigned i=0;i<bytes;i++)image.nvm[a+i]&=((const uint8_t*)p)[i];save_crash();
  }
  if(!stop_injected&&!strcmp(scenario,"stop")&&cut_write==phase_writes){stop_injected=true;assert(thread_comm_request_manual_abort()==0);}
 }
 for(uint32_t i=0;i<n;i++){assert((image.nvm[a+i]&((const uint8_t*)p)[i])==((const uint8_t*)p)[i]);image.nvm[a+i]=((const uint8_t*)p)[i];}
 if(request_inflight){
  if(key==0x42524745U&&!strcmp(scenario,"identity_after"))save_crash();
  if(n==4U&&a%2048U==16U&&identity_writes&&!floor_writes&&!strcmp(scenario,"identity_committed"))save_crash();
  if(key==0x42524745U&&!strcmp(scenario,"stop_identity"))assert(thread_comm_request_manual_abort()==0);
  if(key==0x4252464cU&&!strcmp(scenario,"stop_floor"))assert(thread_comm_request_manual_abort()==0);
  if((key==UINT32_C(0x4252464c)&&!strcmp(scenario,"floor_after"))||(key==UINT32_C(0x46535445)&&!strcmp(scenario,"state_after")))save_crash();
  if((key==UINT32_C(0x4252464c)&&!strcmp(scenario,"abort_floor"))||(key==UINT32_C(0x46535445)&&!strcmp(scenario,"abort_state")))assert(thread_comm_request_manual_abort()==0);
  if(key==UINT32_C(0x4252464c)&&!strcmp(scenario,"readback_fault"))fail_read=read_calls+1;
 }
 return LAUNCHCORE_STORAGE_OK;
}
const launchcore_storage_driver_t launchcore_board_storage_driver={.init=init_storage,.erase=erase_storage,.write=write_storage,.read=read_storage,.layout=get_layout};
UINT tx_mutex_create(TX_MUTEX*m,const char*n,UINT i){(void)i;m->locked=0;if(!strcmp(n,"thread_comm_state_mutex"))state_mutex=m;if(!strcmp(n,"persist"))store_mutex=m;if(!strcmp(n,"bootstrap_jobs"))jobs_lock=m;return 0;}
UINT tx_mutex_get(TX_MUTEX*m,ULONG w){(void)w;
 if(m==store_mutex&&fail_store_lock)return TX_MUTEX_ERROR;
 if((m==state_mutex&&fail_state_get)||(m==jobs_lock&&fail_jobs_get))return TX_MUTEX_ERROR;

 assert(!m->locked);m->locked=1;return 0;}
UINT tx_mutex_put(TX_MUTEX*m){assert(m->locked);m->locked=0;
 if(m==state_mutex&&!floor_busy_injected&&!floor_held&&!strcmp(scenario,"floor_busy_before")&&!identity_writes&&board_abort_latch_is_set()){floor_busy_injected=true;assert(!atomic_flag_test_and_set_explicit(&floor_owner,memory_order_acquire));floor_held=true;}
 if(m==store_mutex&&request_inflight){
  if(identity_writes&&floor_writes&&!strcmp(scenario,"floor_busy_after_first")&&++first_verified_unlocks==2){assert(!atomic_flag_test_and_set_explicit(&floor_owner,memory_order_acquire));floor_held=true;}
  if(saw_complete&&!strcmp(scenario,"floor_busy_complete")&&++complete_verified_unlocks==2){assert(!atomic_flag_test_and_set_explicit(&floor_owner,memory_order_acquire));floor_held=true;}
 }
 if((m==state_mutex&&fail_state_put)||(m==jobs_lock&&fail_jobs_put))return TX_MUTEX_ERROR;
 if(m==state_mutex&&barrier_calls>=2&&!strcmp(scenario,"barrier_unlock_fail"))return TX_MUTEX_ERROR;
 if(m==state_mutex&&barrier_calls>=2&&!injected_identity_race&&(!strcmp(scenario,"tombstone_race")||!strcmp(scenario,"generation_race"))){
  injected_identity_race=true;bool in_request=request_inflight;request_inflight=false;assert(persistent_store_enter());launchcore_persist_strict_info_t info;
  if(!strcmp(scenario,"tombstone_race"))assert(launchcore_persist_delete_strict(0x42524745U,&info)==0);
  else{uint64_t generation=7;assert(launchcore_persist_set_strict(0x42524745U,&generation,8,&info)==0);}
  assert(persistent_store_leave());request_inflight=in_request;
 }
 if(m==store_mutex&&(fail_store_unlock||(!strcmp(scenario,"commit_unlock")&&floor_writes))){fail_store_unlock=false;return TX_MUTEX_ERROR;}
 return 0;}
UINT tx_semaphore_create(TX_SEMAPHORE*s,const char*n,ULONG count){(void)n;s->count=count;return 0;}
UINT tx_semaphore_get(TX_SEMAPHORE*s,ULONG w){(void)w;if(!s->count)return TX_NO_INSTANCE;s->count--;return 0;}
UINT tx_semaphore_put(TX_SEMAPHORE*s){s->count++;return 0;}
bool seds_router_sender_matches_hostname(const SedsRouter*r,const char*s,size_t n,const char*owner,size_t owner_n){assert(sdk_unlocked());
 assert(r&&owner_n==2&&!memcmp(owner,"GS",2));if(!s)return false;
 /* Named/assigned SDK boundary only; generic compact identity is exercised
  * against the actual SDK in the identity regressions. */
 if(n==owner_n&&!memcmp(s,owner,n))return true;
 char alias[32];int length=snprintf(alias,sizeof alias,"@addr:%u",(unsigned)1U);
 return length>0&&(size_t)length==n&&!memcmp(s,alias,n);}

SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){assert(sdk_unlocked());(void)r;*out=now;return SEDS_OK;}
SedsResult seds_router_log_queue_typed(SedsRouter*r,SedsDataType ty,const void*data,size_t count,size_t size,SedsElemKind kind){assert(sdk_unlocked());(void)r;assert(ty==201&&count==40&&size==1&&kind==SEDS_EK_UNSIGNED);assert(!state_mutex->locked);if(send_failure){send_failure=false;return SEDS_IO;}responses++;memcpy(response,data,40);if(ty==201)assert((response[3]&32U)==EXPECTED_CAP);return SEDS_OK;}
void seds_router_disable_managed_variable(SedsRouter*r,SedsDataType ty){assert(sdk_unlocked());(void)r;(void)ty;assert(!"whole-service disable forbidden during selective retirement");}
SedsResult seds_router_invalidate_network_variable_cache(SedsRouter*r,SedsDataType ty,bool*out_invalidated){assert(sdk_unlocked());assert(out_invalidated==NULL);(void)r;assert(!state_mutex->locked&&ty==SEDS_DT_FLIGHT_STATE);return native_cache_fault?SEDS_IO:SEDS_OK;}

static unsigned selective_enable_calls,selective_callback_calls;
SedsResult seds_router_enable_network_variable(SedsRouter*r,SedsDataType ty,bool read,bool write){assert(sdk_unlocked());assert(++selective_enable_calls==1U);(void)r;assert(!state_mutex->locked&&ty==SEDS_DT_FLIGHT_STATE&&read&&!write);return SEDS_OK;}
SedsResult seds_router_on_network_variable_update(SedsRouter*r,SedsDataType ty,SedsResult(*cb)(const SedsPacketView*,void*),void*user){assert(sdk_unlocked());assert(++selective_callback_calls==1U);(void)r;assert(ty==SEDS_DT_FLIGHT_STATE&&!user);update_callback=cb;return SEDS_OK;}
SedsResult seds_router_request_managed_variable(SedsRouter*r,SedsDataType ty){assert(sdk_unlocked());(void)r;(void)ty;return SEDS_OK;}
SedsResult flight_state_cache_apply_network_update(const SedsPacketView*p){return thread_comm_apply_managed_flight_state(p->payload[0])==TX_SUCCESS?SEDS_OK:SEDS_HANDLER_ERROR;}
static uint8_t read_outputs(uint32_t*out){assert(state_mutex->locked);*out=SAFE_MASK;
 if(barrier_calls){safe_readbacks++;last_safe_marker=barrier_calls;if(!strcmp(scenario,"group_read_fault"))fail_read=read_calls+1;}
 return BOARD_RECOVERY_OUTPUTS_VALID|(unsafe_outputs?0:BOARD_RECOVERY_SAFE_OUTPUTS)|(owner_stopped?BOARD_RECOVERY_OWNER_STOPPED:0);}
static uint8_t last_prepare=255;
static uint8_t prepare_outputs(uint8_t desired){assert(state_mutex->locked&&desired<=BOARD_RECOVERY_MAX_STATE);last_prepare=desired;return 1;}
static void finish_outputs(void){assert(state_mutex->locked&&(!store_mutex||!store_mutex->locked));finished++;completed_owner=true;}
static void request(SedsRouter*r,uint8_t*body,uint64_t timestamp){uint32_t endpoint=RECOVERY_ENDPOINT;SedsPacketView p={.ty=200,.sender="GS",.sender_len=2,.endpoints=&endpoint,.num_endpoints=1,.payload=body,.payload_len=32,.timestamp=timestamp};assert(board_recovery_handle(r,&p,endpoint,123)==SEDS_OK);}


static void put64(uint8_t*p,uint64_t v){for(unsigned i=0;i<8;i++)p[i]=(uint8_t)(v>>(8*i));}
static void packet_body(uint8_t*p,const BoardRecoveryRequest*r){memset(p,0,32);p[0]=1;p[1]=r->operation;p[2]=r->desired_state;put64(p+4,r->nonce);put64(p+12,r->expected_boot_generation);memcpy(p+20,&r->expected_abort_epoch,4);put64(p+24,r->issuer_session);}


static uint8_t stop_owner(void){assert(main_owner&&state_mutex->locked);owner_stopped=true;return 1;}
static void launch_noop(void*context){(void)context;}
static void main_service(void){main_owner=true;for(unsigned drain=0;drain<3;drain++){thread_comm_service_board_recovery();board_bootstrap_storage_service();}main_owner=false;}
static uint64_t get64(const uint8_t*p){uint64_t value=0;for(unsigned i=0;i<8;i++)value|=(uint64_t)p[i]<<(8*i);return value;}
static void send_req(SedsRouter*r,BoardRecoveryRequest*q,uint64_t time){uint8_t body[32];packet_body(body,q);request(r,body,time);}
static void query_req(SedsRouter*r,BoardRecoveryRequest*q,uint64_t time){unsigned reads=read_calls,writes=media_writes,erases=media_erases;struct TampFake backup=test_tamp;uint8_t flash[NVM_SIZE];memcpy(flash,image.nvm,sizeof flash);send_req(r,q,time);assert(reads==read_calls&&writes==media_writes&&erases==media_erases&&!memcmp(flash,image.nvm,sizeof flash)&&!memcmp(&backup,&test_tamp,sizeof backup));}
static void blocked(void){thread_comm_msg_t command=9;assert(thread_comm_admit_network(command,10020)!=TX_SUCCESS);assert(thread_comm_send(command,TX_NO_WAIT)!=TX_SUCCESS);}
static void begin_phase(unsigned value){phase=value;phase_writes=phase_reads=0;request_inflight=true;}
static void end_phase(void){request_inflight=false;fprintf(stderr,"phase%u writes=%u reads=%u\n",phase,phase_writes,phase_reads);}
static void marker_state(int*state){if(!persistent_store_enter()){*state=-1;return;}uint8_t wire[40];size_t size=40;launchcore_persist_strict_info_t info;launchcore_persist_status_t status=launchcore_persist_get_strict(BOOTSTRAP_MARKER_KEY,wire,&size,&info);BootstrapMarker marker;*state=status==0&&bootstrap_marker_decode(wire,size,&marker)?marker.state:-1;assert(persistent_store_leave());}

static void seed_control(const char*which){
 if(strncmp(which,"seed_",5))return;
 launchcore_storage_set_driver(&launchcore_board_storage_driver);launchcore_persist_strict_info_t info;
 uint64_t gen=41,floor[2]={77,100};uint8_t cal[8]={0};
 if(!strcmp(which,"seed_corrupt")){image.nvm[0]=0;return;}
 if(!strcmp(which,"seed_max"))gen=UINT64_MAX;
 if(!strcmp(which,"seed_nearmax"))gen=UINT64_MAX-1U;
 bool identity=strcmp(which,"seed_absent")&&strcmp(which,"seed_badmarker")&&strcmp(which,"seed_deletedmarker")&&strcmp(which,"seed_deletedfloor");
 assert(launchcore_persist_provision_virgin_strict(identity?0x42524745U:0x43414c42U,identity?(void*)&gen:(void*)cal,8,&info)==0);
 if(!strcmp(which,"seed_deletedgen"))assert(launchcore_persist_delete_strict(0x42524745U,&info)==0);
 if(!strcmp(which,"seed_healthy")||!strcmp(which,"seed_max")||!strcmp(which,"seed_nearmax"))assert(launchcore_persist_set_strict(0x4252464cU,floor,16,&info)==0);
 if(!strcmp(which,"seed_badmarker")||!strcmp(which,"seed_deletedmarker")){
  uint8_t invalid[32]={0};assert(launchcore_persist_set_strict(BOOTSTRAP_MARKER_KEY,invalid,32,&info)==0);
  if(!strcmp(which,"seed_deletedmarker"))assert(launchcore_persist_delete_strict(BOOTSTRAP_MARKER_KEY,&info)==0);
 }
 if(!strcmp(which,"seed_deletedfloor")){assert(launchcore_persist_set_strict(0x4252464cU,floor,16,&info)==0);assert(launchcore_persist_delete_strict(0x4252464cU,&info)==0);}
}
int main(int argc,char**argv){
 assert(argc>=3);scenario=argv[1];image_path=argv[2];
 bool reboot=!strcmp(scenario,"reboot");
 if(argc>4)cut_write=(unsigned)strtoul(argv[4],0,10);
 if(argc>5)cut_bytes=(unsigned)strtoul(argv[5],0,10);
 if(argc>6)cut_read=(unsigned)strtoul(argv[6],0,10);
 if(reboot){FILE*f=fopen(image_path,"rb");assert(f&&fread(&image,sizeof image,1,f)==1&&fclose(f)==0);assert(!memcmp(&image.backup,&test_tamp,sizeof test_tamp));}
 else memset(image.nvm,255,NVM_SIZE);
 launchcore_storage_set_driver(&launchcore_board_storage_driver);if(!reboot)seed_control("seed_healthy");fixture=false;
 board_abort_latch_init();(void)persistent_store_boot_begin();(void)board_recovery_boot_generation();board_bootstrap_storage_probe_boot();
 assert(persistent_store_runtime_start()==0);thread_present=true;TX_BYTE_POOL pool={0};assert(thread_comm_init(&pool)==0);
 board_recovery_bind_owner(read_outputs,prepare_outputs,finish_outputs);assert(board_bootstrap_storage_bind_owner(stop_owner));
 int cookie=0;SedsRouter*r=(SedsRouter*)&cookie;assert(flight_state_cache_init(r)==SEDS_OK);board_recovery_transport_set_router(r);
 BoardRecoveryRequest q={.operation=BOARD_RECOVERY_QUERY,.nonce=1,.issuer_session=77};query_req(r,&q,10000);
 if(reboot){
  int marker;marker_state(&marker);
  fprintf(stderr,"cold reboot: marker=%d gen=%llu controls=%u abort=%u completed_witness=%u result=%u\n",marker,(unsigned long long)board_recovery_boot_current(),persistent_store_controls_ready(),thread_comm_get_abort(),image.completed_witness,response[1]);
  assert(!persistent_store_controls_ready()||(marker==0&&image.completed_witness));
  if(!persistent_store_controls_ready())blocked();
  if(marker==1){assert(thread_comm_get_abort());main_service();assert(thread_comm_get_abort()&&!persistent_store_controls_ready());
   uint64_t gen=board_recovery_boot_current();if(gen){
    /* A legitimate new primary may finish after a cold GS restart. */
    q.nonce=13;q.issuer_session=88;query_req(r,&q,10010);
    BoardRecoverySnapshot snap={0};(void)board_recovery_snapshot(&snap);
    BoardRecoveryRequest recover={.operation=BOARD_RECOVERY_RECOVER,.desired_state=0,.nonce=14,.expected_boot_generation=gen,.expected_abort_epoch=snap.abort_epoch,.issuer_session=88};
    send_req(r,&recover,10011);
    if(response[1]==1){main_service();send_req(r,&recover,10011);assert(response[1]==0&&persistent_store_controls_ready()&&!thread_comm_get_abort());}
    else assert(!persistent_store_controls_ready()&&thread_comm_get_abort());
   }
  }
  puts("cold allRAM+RTCbackup barrier PASS");return 0;
 }


 if(!strcmp(scenario,"transport")){
  BoardRecoverySnapshot snapshot={0};assert(board_recovery_snapshot(&snapshot)==0);unsigned reads=read_calls,writes=media_writes,old_responses=responses;
  uint32_t ends[2]={RECOVERY_ENDPOINT,9999};uint8_t body[32];BoardRecoveryRequest req={.operation=1,.nonce=2,.expected_boot_generation=board_recovery_boot_current(),.expected_abort_epoch=snapshot.abort_epoch,.issuer_session=77};packet_body(body,&req);
  SedsPacketView packet={.ty=200,.sender="GS",.sender_len=2,.endpoints=ends,.num_endpoints=2,.payload=body,.payload_len=32,.timestamp=10000};
  assert(REFUSED(board_recovery_handle(r,&packet,RECOVERY_ENDPOINT,123)));packet.num_endpoints=1;assert(REFUSED(board_recovery_handle(r,&packet,9999,123)));
  packet.sender="RF";assert(REFUSED(board_recovery_handle(r,&packet,RECOVERY_ENDPOINT,123)));packet.sender="@addr:4294967296";packet.sender_len=16;assert(REFUSED(board_recovery_handle(r,&packet,RECOVERY_ENDPOINT,123)));
  packet.sender="GS";packet.sender_len=2;body[3]=1;assert(REFUSED(board_recovery_handle(r,&packet,RECOVERY_ENDPOINT,123)));body[3]=0;body[2]=2;assert(REFUSED(board_recovery_handle(r,&packet,RECOVERY_ENDPOINT,123)));body[2]=0;assert(responses==old_responses&&writes==media_writes&&reads==read_calls);
  req.expected_boot_generation--;packet_body(body,&req);assert(board_recovery_handle(r,&packet,RECOVERY_ENDPOINT,123)==0&&response[1]==4);req.expected_boot_generation++;packet_body(body,&req);
  packet.timestamp=0;assert(board_recovery_handle(r,&packet,RECOVERY_ENDPOINT,123)==0&&response[1]==4);packet.timestamp=10000;now=25001;assert(board_recovery_handle(r,&packet,RECOVERY_ENDPOINT,123)==0&&response[1]==4);now=10000;
  req.operation=0;packet_body(body,&req);packet.sender="@addr:1";packet.sender_len=7;assert(board_recovery_handle(r,&packet,RECOVERY_ENDPOINT,123)==0&&response[1]==0);assert(reads==read_calls&&writes==media_writes);
  req.operation=1;req.nonce=7;now=10001;begin_phase(3);send_req(r,&req,10001);main_service();end_phase();assert(response[1]==0&&finished==1&&floor_writes==1&&state_writes==1);
  BoardRecoveryRequest fresh_query={.operation=0,.nonce=8,.issuer_session=77};reads=read_calls;writes=media_writes;now=10002;query_req(r,&fresh_query,10002);assert(response[1]==0&&response[2]==0&&!(response[3]&1U)&&get64(response+12)==req.expected_boot_generation&&response[20]==2&&reads==read_calls&&writes==media_writes);
  writes=media_writes;reads=read_calls;send_req(r,&req,10001);assert(response[1]==0&&writes==media_writes&&reads==read_calls);send_req(r,&req,10002);assert(response[1]==6&&writes==media_writes&&reads==read_calls);req.desired_state=1;send_req(r,&req,10001);assert(response[1]==6&&finished==1&&writes==media_writes);
  packet.timestamp=10001;assert(board_recovery_command_is_stale(&packet));packet.timestamp=10002;assert(!board_recovery_automatic_is_stale(&packet));packet.timestamp=12000;assert(board_recovery_command_is_stale(&packet));assert(board_recovery_automatic_is_stale(NULL));board_recovery_transport_set_router(NULL);packet.timestamp=10002;assert(board_recovery_command_is_stale(&packet));
  puts("actual transport auth/target/shape/time/epoch/original tuple/read-only Q and staged strict owner PASS");return 0;
 }
 if(!strcmp(scenario,"resync_launch")){
  /* The board is in a live Launch (not aborted). RESYNC to Idle must be bound
   * to this board's own prior QUERY: a different issuer session or the
   * query's own nonce is refused and the launch keeps running; the bound
   * request stops it and returns the board to Idle with no Abort. */
  assert(thread_comm_request_launch_sequence(10000)==TX_SUCCESS);
  thread_comm_launch_owner_t owner;assert(thread_comm_take_accepted_launch(&owner));assert(thread_comm_launch_begin_owned(&owner,launch_noop,0));
  assert(thread_comm_get_flight_state()==7&&!thread_comm_get_abort());
  BoardRecoveryRequest lq={.operation=0,.nonce=3,.issuer_session=77};now=10001;query_req(r,&lq,10001);
  assert(response[1]==0&&response[2]==7&&!(response[3]&BOARD_RECOVERY_ABORT_LATCHED));
  uint32_t epoch=(uint32_t)response[20]|(uint32_t)response[21]<<8|(uint32_t)response[22]<<16|(uint32_t)response[23]<<24;
  uint64_t gen=board_recovery_boot_current();unsigned writes=media_writes;
  BoardRecoveryRequest rs={.operation=BOARD_RECOVERY_RESYNC,.desired_state=1,.nonce=4,.expected_boot_generation=gen,.expected_abort_epoch=epoch,.issuer_session=78};
  now=10002;send_req(r,&rs,10002);main_service();
  assert(response[1]==4&&writes==media_writes&&!finished&&thread_comm_get_flight_state()==7&&!thread_comm_get_abort()&&thread_comm_launch_continuity());
  rs.issuer_session=77;rs.nonce=3;send_req(r,&rs,10002);main_service();
  assert(response[1]==4&&writes==media_writes&&!finished&&thread_comm_get_flight_state()==7&&thread_comm_launch_continuity());
  rs.nonce=5;now=10003;send_req(r,&rs,10003);main_service();send_req(r,&rs,10003);
  assert(response[1]==0&&response[2]==1&&!(response[3]&BOARD_RECOVERY_ABORT_LATCHED)&&!thread_comm_get_abort()&&!thread_comm_launch_continuity());
  puts("RESYNC from a live Launch: wrong session and replayed query nonce refused; bound request returns to Idle with no Abort PASS");return 0;
 }
 if(!strcmp(scenario,"resync")||!strcmp(scenario,"resync_unsafe")){
  /* The station is filling; this board rebooted or desynced and latched Abort.
   * RESYNC carries the station phase and must never move an output. */
  assert(response[3]&BOARD_RECOVERY_RESYNC_IMPLEMENTATION);
  assert(thread_comm_request_manual_abort()==TX_SUCCESS&&thread_comm_get_abort());
  q.nonce=3;now=10001;query_req(r,&q,10001);assert(response[1]==0&&(response[3]&BOARD_RECOVERY_ABORT_LATCHED)&&(response[3]&BOARD_RECOVERY_RESYNC_IMPLEMENTATION));
  uint32_t epoch=(uint32_t)response[20]|(uint32_t)response[21]<<8|(uint32_t)response[22]<<16|(uint32_t)response[23]<<24;
  uint64_t gen=board_recovery_boot_current();assert(gen&&gen!=UINT64_MAX&&response[2]==0);
  BoardRecoveryRequest rs={.operation=BOARD_RECOVERY_RESYNC,.desired_state=BOARD_RECOVERY_MAX_STATE+1U,.nonce=4,.expected_boot_generation=gen,.expected_abort_epoch=epoch,.issuer_session=77};
  unsigned reads=read_calls,writes=media_writes,old_responses=responses;uint32_t endpoint=RECOVERY_ENDPOINT;uint8_t body[32];
  SedsPacketView packet={.ty=200,.sender="GS",.sender_len=2,.endpoints=&endpoint,.num_endpoints=1,.payload=body,.payload_len=32,.timestamp=10002};now=10002;
  /* Launch onward and the commissioning sentinel never decode (Startup and Idle
   * do: a board left in Launch is resynced back without an Abort): no
   * reply, storage access or state change. RECOVER keeps Startup/Idle only. */
  static const uint8_t refused[]={7U,8U,13U,14U,15U,16U,255U};
  for(unsigned i=0;i<sizeof refused;i++){rs.desired_state=refused[i];packet_body(body,&rs);assert(REFUSED(board_recovery_handle(r,&packet,RECOVERY_ENDPOINT,123)));}
  rs.desired_state=2;rs.expected_boot_generation=UINT64_MAX;packet_body(body,&rs);assert(REFUSED(board_recovery_handle(r,&packet,RECOVERY_ENDPOINT,123)));
  rs.expected_boot_generation=gen;rs.operation=BOARD_RECOVERY_RECOVER;packet_body(body,&rs);assert(REFUSED(board_recovery_handle(r,&packet,RECOVERY_ENDPOINT,123)));
  rs.operation=BOARD_RECOVERY_RESYNC;packet_body(body,&rs);body[1]=3;assert(REFUSED(board_recovery_handle(r,&packet,RECOVERY_ENDPOINT,123)));
  assert(responses==old_responses&&reads==read_calls&&writes==media_writes&&thread_comm_get_abort()&&!finished&&last_prepare==255);
  /* Stale context: the epoch from the readback before this Abort. */
  rs.expected_abort_epoch=epoch-1U;send_req(r,&rs,10002);assert(response[1]==4&&thread_comm_get_abort()&&writes==media_writes&&!finished);
  rs.expected_abort_epoch=epoch;
  /* RESYNC to Startup/Idle keeps every binding: a wrong boot generation, a
   * stale source time or a stale epoch is refused with nothing written. */
  {BoardRecoveryRequest bad=rs;bad.desired_state=1;bad.nonce=40;bad.expected_boot_generation=gen+1U;send_req(r,&bad,10002);
   assert(response[1]==4&&writes==media_writes&&!finished&&thread_comm_get_abort());
   bad.expected_boot_generation=gen;bad.nonce=41;bad.expected_abort_epoch=epoch+1U;send_req(r,&bad,10002);
   assert(response[1]==4&&writes==media_writes&&!finished);
   bad.expected_abort_epoch=epoch;bad.nonce=42;uint64_t saved_now=now;now=40000;send_req(r,&bad,10002);now=saved_now;
   assert(response[1]==4&&writes==media_writes&&!finished);}
  if(!strcmp(scenario,"resync_unsafe")){
   unsafe_outputs=true;rs.nonce=5;now=10003;send_req(r,&rs,10003);main_service();send_req(r,&rs,10003);
   BoardRecoverySnapshot after={0};(void)board_recovery_snapshot(&after);
   assert(response[1]!=0&&response[1]!=1&&thread_comm_get_abort()&&!finished&&after.actual_state==0);
   puts("RESYNC with unsafe outputs refused; Abort kept, nothing driven PASS");return 0;
  }
  rs.nonce=5;now=10003;send_req(r,&rs,10003);main_service();send_req(r,&rs,10003);
  uint32_t outputs=(uint32_t)response[28]|(uint32_t)response[29]<<8|(uint32_t)response[30]<<16|(uint32_t)response[31]<<24;
  uint32_t applied=(uint32_t)response[20]|(uint32_t)response[21]<<8|(uint32_t)response[22]<<16|(uint32_t)response[23]<<24;
  assert(response[1]==0&&response[2]==2&&applied==epoch+1U&&!(response[3]&BOARD_RECOVERY_ABORT_LATCHED));
  assert(outputs==SAFE_MASK&&(response[3]&BOARD_RECOVERY_SAFE_OUTPUTS)&&last_prepare==2&&finished==1&&!thread_comm_get_abort()&&thread_comm_get_flight_state()==2);
  uint8_t fstate=255;size_t fsize=1;assert(persistent_store_get(0x46535445U,&fstate,&fsize)==LAUNCHCORE_PERSIST_OK&&fsize==1&&fstate==2);
  /* The completed nonce cannot be replayed with a new time, and an exact
   * duplicate only reports the finished result. */
  writes=media_writes;send_req(r,&rs,10003);assert(response[1]==0&&finished==1&&writes==media_writes);
  now=10004;send_req(r,&rs,10004);assert(response[1]==6&&finished==1&&writes==media_writes);
  /* A RESYNC built from the pre-resync readback is stale now. */
  rs.nonce=6;now=10005;send_req(r,&rs,10005);assert(response[1]==4&&finished==1&&writes==media_writes&&thread_comm_get_flight_state()==2);
  puts("RESYNC to NitrousFill: proof/epoch checked, Abort cleared, outputs untouched, Launch onward refused PASS");return 0;
 }
 if(!strncmp(scenario,"seed_",5)&&strcmp(scenario,"seed_absent")){
  if(!strcmp(scenario,"seed_healthy")){assert(response[1]==0&&persistent_store_controls_ready());}
  else{assert(get64(response+12)!=UINT64_MAX&&!persistent_store_controls_ready());blocked();}
  puts("strict classification/read-only QUERY PASS");return 0;
 }

 assert(persistent_store_controls_ready()&&response[1]==0);
 BoardRecoverySnapshot normal_snapshot={0};(void)board_recovery_snapshot(&normal_snapshot);
 BoardRecoveryRequest normal={.operation=BOARD_RECOVERY_RECOVER,.desired_state=0,.nonce=2,.expected_boot_generation=board_recovery_boot_current(),.expected_abort_epoch=normal_snapshot.abort_epoch,.issuer_session=77};
 unsigned initial_writes=media_writes,initial_reads=read_calls,initial_responses=responses;
 if(!strcmp(scenario,"backup_repair"))backup_fault=1;
 bool retry_fault=!strcmp(scenario,"backup_repair")||!strcmp(scenario,"cache_retry")||!strcmp(scenario,"floor_retry");native_cache_fault=!strcmp(scenario,"cache_retry");native_floor_fault=!strcmp(scenario,"floor_retry");
 begin_phase(3);send_req(r,&normal,10001);assert(responses==initial_responses&&!thread_comm_get_abort()&&!persistent_store_controls_ready());
 thread_comm_service_board_recovery();assert(media_writes==initial_writes&&read_calls==initial_reads&&!finished);
 main_owner=true;board_bootstrap_storage_service();main_owner=false;
 if(!strcmp(scenario,"success")){assert(response[1]==1&&thread_comm_get_abort()&&floor_writes==1&&state_writes==1);uint8_t state=15;SedsPacketView update={.ty=SEDS_DT_FLIGHT_STATE,.sender="GS",.sender_len=2,.timestamp=10002,.payload=&state,.payload_len=1};assert(update_callback&&update_callback(&update,NULL)!=SEDS_OK);assert(floor_writes==1&&state_writes==1);}
 main_service();
 if(retry_fault){
  assert(response[1]==7&&thread_comm_get_abort()&&!finished&&!persistent_store_controls_ready());
  if(!strcmp(scenario,"backup_repair"))assert(media_writes==initial_writes&&floor_writes==0&&state_writes==0);
  unsigned failed_writes=media_writes;backup_fault=0;native_cache_fault=false;native_floor_fault=false;send_req(r,&normal,10001);main_service();assert(response[1]==7&&media_writes==failed_writes&&!finished);
  q.nonce=3;now=10002;query_req(r,&q,10002);(void)board_recovery_snapshot(&normal_snapshot);normal.nonce=4;normal.expected_abort_epoch=normal_snapshot.abort_epoch;now=10003;send_req(r,&normal,10003);main_service();
 }
 end_phase();
 if(!strcmp(scenario,"abort_floor")||!strcmp(scenario,"abort_state")){
  assert(response[1]!=0&&thread_comm_get_abort()&&!persistent_store_controls_ready()&&!finished);blocked();puts("actual NORMAL Stop during NVM cancels owner PASS");return 0;
 }
 assert(response[1]==0&&persistent_store_controls_ready()&&!thread_comm_get_abort()&&finished==1&&completed_owner&&floor_writes>=1&&state_writes>=1);
 if(!retry_fault||!strcmp(scenario,"backup_repair"))assert(floor_writes==1&&state_writes==1);
 uint8_t fstate=255;size_t fsize=1;assert(persistent_store_get(0x46535445U,&fstate,&fsize)==LAUNCHCORE_PERSIST_OK&&fsize==1&&fstate==0);
 unsigned done_writes=media_writes;send_req(r,&normal,retry_fault?10003:10001);assert(response[1]==0&&media_writes==done_writes&&finished==1);
 puts("actual NORMAL durable PENDING->floor/FSTATE->owner->COMPLETE idempotent PASS");return 0;
}
