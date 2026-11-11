from pathlib import Path
import importlib.util,json,subprocess,os
HERE=Path(__file__).resolve().parent;P=Path(os.environ['OTA_OWNER_WORK']).resolve()/'native';spec=importlib.util.spec_from_file_location('base',HERE/'support_fixture.py');base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
old=P/'ValveBoard26/candidate';out=P/'runtime';out.mkdir(exist_ok=True)
for n in ['tx_api.h','main.h','stm32g4xx_hal.h']:(out/n).write_bytes((old/n).read_bytes())
f=out/'main.h';s=f.read_text().replace('return 0;}','extern unsigned test_isr;return test_isr;}');f.write_text(s)
s=(old/'test.c').read_text();prefix=base.function(s,'main');prefix=prefix[prefix.index(' assert(argc==2);'):prefix.index(' setup_outputs();')]
s=s.replace('int main(int argc,char**argv)','int prior_critical_main(int argc,char**argv)')
s=s.replace('static _Thread_local unsigned state_depth;','static _Thread_local unsigned state_depth,physical_depth;\ntypedef int TX_THREAD;TX_THREAD telemetry_thread;static _Thread_local bool foreign;static bool fail_state_release,fail_physical_release;unsigned test_isr;')
s=s.replace('return atomic_load(&runtime)?(void*)1:NULL;','return atomic_load(&runtime)?(foreign?(void*)2:(void*)&telemetry_thread):NULL;')
s=s.replace('if(m->state)state_depth++;return TX_SUCCESS;','if(m->state)state_depth++;if(!m->state&&!m->ring)physical_depth++;return TX_SUCCESS;')
s=s.replace('UINT tx_mutex_put(TX_MUTEX*m){if(m->state)', 'UINT tx_mutex_put(TX_MUTEX*m){if(!m->state&&!m->ring){assert(physical_depth);if(fail_physical_release){fail_physical_release=false;return TX_MUTEX_ERROR;}physical_depth--;}if(m->state)')
s=s.replace('assert(!pthread_mutex_unlock(&m->raw));return TX_SUCCESS;}', 'assert(!pthread_mutex_unlock(&m->raw));if(m->state&&fail_state_release){fail_state_release=false;return TX_MUTEX_ERROR;}return TX_SUCCESS;}')
extra=r'''
#include "launchcore/update.h"
#include "launchcore/metadata.h"
#define SEDS_ERR -1
#define OTA_STREAM_PORT 4510U
#define OTA_STREAM_MAX_CHUNK 120U
#define BOARD_DELTA_SIZE 0x00006000U
#define SEDS_P2P_STREAM_ACCEPTED 1
#define SEDS_P2P_STREAM_DATA 3
#define SEDS_P2P_STREAM_CLOSED 4
#define SEDS_P2P_STREAM_RESET 5
typedef struct {uint8_t kind;uint32_t stream_id,peer_stream_id,sequence;const char*peer_hostname;size_t peer_hostname_len;uint32_t peer_address;uint16_t local_port,peer_port;const uint8_t*payload;size_t payload_len;}SedsP2pStreamEventView;
typedef enum {OTA_STREAM_OK=0,OTA_STREAM_BAD_MESSAGE,OTA_STREAM_BAD_STATE,OTA_STREAM_BAD_OFFSET,OTA_STREAM_NO_SPACE,OTA_STREAM_STORAGE_ERROR,OTA_STREAM_BAD_IMAGE,OTA_STREAM_INTERNAL_ERROR}ota_stream_status_t;
static SedsResult(*bound)(const SedsP2pStreamEventView*,void*);
static uint64_t ota_clock=1000;static unsigned begins,writes_ota,finishes,cleanups,resets,send_calls,stream_resets;static bool fail_send,fail_cleanup,fail_confirm,fail_verify_read,fail_abort_commit;
static launchcore_metadata_t ota_md;static uint8_t last_reply[13],original_reply[13];
static uint64_t telemetry_now_ms(void){return ota_clock;}
static void NVIC_SystemReset(void){assert(state_depth&&!physical_depth);resets++;}
static SedsResult seds_router_bind_p2p_stream_port(SedsRouter*r,uint16_t p,SedsResult(*cb)(const SedsP2pStreamEventView*,void*),void*u){assert(r&&p==4510&&cb&&!u&&!physical_depth&&!state_depth);bound=cb;return SEDS_OK;}
static SedsResult seds_router_reset_p2p_stream(SedsRouter*r,uint32_t id){assert(r&&id&&!physical_depth&&!state_depth);stream_resets++;return SEDS_OK;}
static SedsResult seds_router_send_p2p_stream(SedsRouter*r,uint32_t id,const uint8_t*p,size_t n){assert(r&&id&&n==13&&!physical_depth&&!state_depth);memcpy(last_reply,p,13);send_calls++;return fail_send?SEDS_ERR:SEDS_OK;}
uint32_t launchcore_metadata_crc(const launchcore_metadata_t*m){const uint8_t*p=(const uint8_t*)m;uint32_t sum=0;for(size_t i=0;i<offsetof(launchcore_metadata_t,metadata_crc32);i++)sum=sum*33U+p[i];return sum;}
launchcore_metadata_status_t launchcore_metadata_read_latest(launchcore_metadata_t*m){assert(physical_depth&&!state_depth);if(fail_verify_read&&cleanups)return LAUNCHCORE_METADATA_ERR_STORAGE;*m=ota_md;return LAUNCHCORE_METADATA_OK;}
launchcore_status_t launchcore_delta_update_begin(uint32_t n){assert(physical_depth&&!state_depth&&n==8);memset(&ota_md,0,sizeof ota_md);ota_md.magic=LAUNCHCORE_METADATA_MAGIC;ota_md.version=LAUNCHCORE_METADATA_VERSION;ota_md.size=sizeof ota_md;ota_md.sequence=10;ota_md.confirmed_slot=LAUNCHCORE_SLOT_A;ota_md.pending_slot=LAUNCHCORE_SLOT_NONE;ota_md.update_slot=LAUNCHCORE_SLOT_NONE;ota_md.slot_b_state=LAUNCHCORE_SLOT_WRITING;ota_md.metadata_crc32=launchcore_metadata_crc(&ota_md);begins++;return LAUNCHCORE_OK;}
launchcore_status_t launchcore_delta_update_write(const void*p,size_t n){assert(physical_depth&&!state_depth&&n==8);const uint8_t*bytes=p;for(unsigned i=0;i<8;i++)assert(bytes[i]==i);writes_ota++;return LAUNCHCORE_OK;}
launchcore_status_t launchcore_delta_update_finish(void){assert(physical_depth&&!state_depth);finishes++;return LAUNCHCORE_OK;}
launchcore_status_t launchcore_delta_update_abort(void){assert(physical_depth&&!state_depth);cleanups++;if(fail_cleanup)return LAUNCHCORE_ERR_FLASH;if(!fail_abort_commit){ota_md.sequence++;ota_md.slot_b_state=LAUNCHCORE_SLOT_BAD;ota_md.update_slot=LAUNCHCORE_SLOT_NONE;ota_md.metadata_crc32=launchcore_metadata_crc(&ota_md);}return LAUNCHCORE_OK;}
launchcore_status_t launchcore_confirm_boot(void){assert(physical_depth&&!state_depth);return fail_confirm?LAUNCHCORE_ERR_FLASH:LAUNCHCORE_OK;}
launchcore_status_t launchcore_request_recovery(void){assert(physical_depth&&!state_depth);return LAUNCHCORE_OK;}
uint32_t ota_stream_max_patch_size(void);
ACTUAL_OTA
static uint32_t req_sequence;
static SedsResult event(unsigned kind,const uint8_t*p,size_t n){SedsP2pStreamEventView e={.kind=kind,.stream_id=12,.sequence=++req_sequence,.payload=p,.payload_len=n};return bound(&e,NULL);}
static void request(const uint8_t*p,size_t n){assert(event(SEDS_P2P_STREAM_DATA,p,n)==SEDS_OK);}
static void step_reply(void){ota_stream_poll();assert(work.phase==WORK_REPLY);ota_stream_poll();assert(work.phase==WORK_EMPTY);}
static void begin(void){uint8_t q[]={1,8,0,0,0};request(q,sizeof q);memset(q,255,sizeof q);step_reply();assert(begins==1&&state.active&&last_reply[0]==0x81&&last_reply[1]==0);}
static void finish(void){uint8_t q=3;request(&q,1);step_reply();}
static void recover_same_state(void){BoardRecoverySnapshot snapshot;assert(board_recovery_snapshot(&snapshot)==0);assert(board_recovery_begin_prepare(1,snapshot.abort_epoch)==0);assert(board_recovery_apply_safe(1,snapshot.abort_epoch,&snapshot)==-14);board_recovery_execute_owned();assert(board_recovery_apply_safe(1,snapshot.abort_epoch,&snapshot)==0&&!thread_comm_get_abort());}
int main(int argc,char**argv){
PREFIX
 geometry.slot_b_is_delta=true;setup_outputs();assert(thread_comm_bind_manual_stop_apply(manual_stop_apply_locked)==TX_SUCCESS);board_recovery_bind_owner(queue_recovery_read,queue_recovery_prepare,queue_recovery_finish);check_gpio_owner=true;
 SedsRouter router={0};
 if(!strcmp(argv[1],"init_release")){fail_physical_release=true;assert(ota_stream_init(&router)==SEDS_ERR&&physical_depth==1&&terminal.phase==CLEAN_FAILED&&(terminal.error&0x80U)&&!bound);assert(ota_stream_register_owner());SedsP2pStreamEventView e={.kind=1,.stream_id=12};assert(ota_stream_event(&e,NULL)==SEDS_ERR&&!stream_resets);ota_stream_poll();assert(!send_calls);assert(thread_comm_request_manual_abort()==TX_SUCCESS&&output_mask()==STOP_MASK);return 0;}
 assert(ota_stream_init(&router)==SEDS_OK&&bound);foreign=true;assert(!ota_stream_register_owner());foreign=false;test_isr=1;assert(!ota_stream_register_owner());test_isr=0;assert(ota_stream_register_owner());
 if(!strcmp(argv[1],"foreign")){foreign=true;assert(event(SEDS_P2P_STREAM_ACCEPTED,NULL,0)==SEDS_ERR);ota_stream_poll();foreign=false;test_isr=1;assert(event(SEDS_P2P_STREAM_ACCEPTED,NULL,0)==SEDS_ERR);ota_stream_poll();test_isr=0;assert(!state.connected&&!begins&&!cleanups&&!send_calls);return 0;}
 assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_IDLE)==TX_SUCCESS);assert(event(SEDS_P2P_STREAM_ACCEPTED,NULL,0)==SEDS_OK);
 if(!strcmp(argv[1],"busy_stop")){
  pthread_t worker;assert(!pthread_mutex_lock(&gate));io_block=true;assert(!pthread_mutex_unlock(&gate));assert(!pthread_create(&worker,NULL,writer,NULL));assert(!pthread_mutex_lock(&gate));deadline_wait(&io_entered);assert(!pthread_mutex_unlock(&gate));
  uint8_t q[]={1,8,0,0,0};request(q,sizeof q);ota_stream_poll();assert(work.phase==WORK_REQUEST&&!begins&&!g_ota_service_errors);
  const uint8_t bytes[]="SEDS_MANUAL_ABORT_V1:K";SedsPacketView packet={.ty=SEDS_DT_ABORT,.payload=bytes,.payload_len=sizeof(bytes)-1,.sender=(const uint8_t*)"GS",.sender_len=2,.timestamp=501};assert(Abort_handler(&packet,NULL)==SEDS_OK&&output_mask()==STOP_MASK);ota_stream_poll();assert(terminal.phase==CLEAN_PENDING&&!begins);
  assert(!pthread_mutex_lock(&gate));io_release=true;assert(!pthread_cond_broadcast(&condition));assert(!pthread_mutex_unlock(&gate));assert(!pthread_join(worker,NULL));ota_stream_poll();assert(terminal.phase==CLEAN_NONE&&!begins);puts("NEW GREEN: actual callback+poll never wait on NVM; following validated StopGPIO precedes release");return 0;
 }
 if(!strcmp(argv[1],"recovery_after_launch")){assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_LAUNCH)==TX_SUCCESS);uint8_t q=6;request(&q,1);step_reply();assert(last_reply[1]==OTA_STREAM_BAD_STATE&&!resets&&!state.reset_requested&&!thread_comm_get_abort());return 0;}
 if(!strcmp(argv[1],"boot_release")||!strcmp(argv[1],"boot_fault")){bool uncertain=!strcmp(argv[1],"boot_release");fail_physical_release=uncertain;fail_confirm=!uncertain;ota_clock+=5001;ota_stream_poll();assert(terminal.phase==CLEAN_FAILED&&!state.boot_confirmed&&g_ota_service_errors==1&&!thread_comm_get_abort());assert(physical_depth==(unsigned)uncertain);assert(event(SEDS_P2P_STREAM_ACCEPTED,NULL,0)==SEDS_ERR&&stream_resets==(unsigned)!uncertain);ota_stream_poll();assert(!send_calls);assert(thread_comm_request_manual_abort()==TX_SUCCESS&&output_mask()==STOP_MASK);return 0;}
 begin();
 if(!strcmp(argv[1],"cleanup_readback")||!strcmp(argv[1],"cleanup_commit_lost")||!strcmp(argv[1],"cleanup_exhausted")||!strcmp(argv[1],"cleanup_nondelta")){
  fail_verify_read=!strcmp(argv[1],"cleanup_readback");fail_abort_commit=!strcmp(argv[1],"cleanup_commit_lost");bool exhausted=!strcmp(argv[1],"cleanup_exhausted"),nondelta=!strcmp(argv[1],"cleanup_nondelta");if(exhausted){ota_md.sequence=UINT32_MAX;ota_md.metadata_crc32=launchcore_metadata_crc(&ota_md);}if(nondelta)geometry.slot_b_is_delta=false;
  assert(event(SEDS_P2P_STREAM_RESET,NULL,0)==SEDS_OK);ota_stream_poll();assert(terminal.phase==CLEAN_FAILED&&state.active&&g_ota_cleanup_result==5&&g_ota_service_errors==1&&cleanups==(unsigned)!(exhausted||nondelta));assert(!thread_comm_get_abort());return 0;
 }
 if(!strcmp(argv[1],"cleanup_release")){assert(event(SEDS_P2P_STREAM_RESET,NULL,0)==SEDS_OK);fail_physical_release=true;ota_stream_poll();assert(physical_depth==1&&cleanups==1&&terminal.phase==CLEAN_FAILED&&(terminal.error&0x80U)&&g_ota_service_errors==1);assert(event(SEDS_P2P_STREAM_ACCEPTED,NULL,0)==SEDS_ERR&&!stream_resets);ota_stream_poll();assert(send_calls==1);assert(thread_comm_request_manual_abort()==TX_SUCCESS&&output_mask()==STOP_MASK);return 0;}
 if(!strcmp(argv[1],"physical_release")){uint8_t q[]={2,0,0,0,0,0,1,2,3,4,5,6,7};request(q,sizeof q);fail_physical_release=true;ota_stream_poll();assert(physical_depth==1&&terminal.phase==CLEAN_FAILED&&(terminal.error&0x80U)&&!work.phase&&g_ota_service_errors==1&&send_calls==1);assert(event(SEDS_P2P_STREAM_ACCEPTED,NULL,0)==SEDS_ERR&&!stream_resets);ota_stream_poll();assert(send_calls==1&&writes_ota==1);assert(thread_comm_request_manual_abort()==TX_SUCCESS&&output_mask()==STOP_MASK);return 0;}
 if(!strcmp(argv[1],"reset_release")){uint32_t epoch=thread_comm_manual_epoch();fail_state_release=true;assert(thread_comm_ota_reset_original(epoch,NULL,NULL)==0&&!resets&&!thread_comm_get_abort());assert(thread_comm_ota_reset_original(epoch,NULL,NULL)==1);return 0;}
 if(!strcmp(argv[1],"bounds")){uint8_t q[126]={0};assert(event(3,q,sizeof q)==SEDS_BAD_ARG);assert(event(3,NULL,1)==SEDS_BAD_ARG);assert(!work.phase&&begins==1&&!writes_ota&&send_calls==1);return 0;}
 if(!strcmp(argv[1],"terminal_request")||!strcmp(argv[1],"terminal_reply")){
  uint8_t q[]={2,0,0,0,0,0,1,2,3,4,5,6,7};request(q,sizeof q);
  bool replied=!strcmp(argv[1],"terminal_reply");if(replied){ota_stream_poll();assert(work.phase==WORK_REPLY&&writes_ota==1);}
  assert(event(SEDS_P2P_STREAM_CLOSED,NULL,0)==SEDS_OK&&!work.phase&&terminal.phase==CLEAN_PENDING);ota_stream_poll();assert(terminal.phase==CLEAN_NONE&&cleanups==1&&send_calls==1&&writes_ota==(unsigned)replied);
  assert(event(3,q,sizeof q)==SEDS_ERR);assert(event(SEDS_P2P_STREAM_ACCEPTED,NULL,0)==SEDS_OK);ota_stream_poll();assert(!work.phase&&writes_ota==(unsigned)replied);return 0;
 }
 if(!strcmp(argv[1],"generation_exhaustion")){stream_generation=UINT32_MAX;assert(event(SEDS_P2P_STREAM_RESET,NULL,0)==SEDS_OK);ota_stream_poll();assert(terminal.phase==CLEAN_NONE&&stream_generation==UINT32_MAX);assert(event(SEDS_P2P_STREAM_ACCEPTED,NULL,0)==SEDS_ERR&&!thread_comm_get_abort());return 0;}
 if(!strcmp(argv[1],"payload_reply")){
  uint8_t q[]={2,0,0,0,0,0,1,2,3,4,5,6,7};request(q,sizeof q);SedsP2pStreamEventView dup={.kind=3,.stream_id=12,.sequence=req_sequence,.payload=q,.payload_len=sizeof q};assert(bound(&dup,NULL)==SEDS_OK);dup.sequence++;assert(bound(&dup,NULL)==SEDS_ERR);memset(q,255,sizeof q);ota_stream_poll();assert(writes_ota==1&&work.phase==WORK_REPLY);memcpy(original_reply,work.reply,13);fail_send=true;ota_stream_poll();ota_stream_poll();assert(writes_ota==1&&work.phase==WORK_REPLY&&!memcmp(original_reply,last_reply,13));fail_send=false;ota_stream_poll();assert(writes_ota==1&&work.phase==WORK_EMPTY&&!memcmp(original_reply,last_reply,13));return 0;
 }
 if(!strcmp(argv[1],"terminal_busy")||!strcmp(argv[1],"cleanup_failure")){
  pthread_t worker;assert(!pthread_mutex_lock(&gate));io_block=true;assert(!pthread_mutex_unlock(&gate));assert(!pthread_create(&worker,NULL,writer,NULL));assert(!pthread_mutex_lock(&gate));deadline_wait(&io_entered);assert(!pthread_mutex_unlock(&gate));
  assert(event(SEDS_P2P_STREAM_RESET,NULL,0)==SEDS_OK);ota_stream_poll();assert(terminal.phase==CLEAN_PENDING&&!cleanups);assert(event(SEDS_P2P_STREAM_ACCEPTED,NULL,0)==SEDS_ERR&&stream_resets==1);
  assert(!pthread_mutex_lock(&gate));io_release=true;assert(!pthread_cond_broadcast(&condition));assert(!pthread_mutex_unlock(&gate));assert(!pthread_join(worker,NULL));fail_cleanup=!strcmp(argv[1],"cleanup_failure");ota_stream_poll();
  if(fail_cleanup){assert(terminal.phase==CLEAN_FAILED&&g_ota_cleanup_result==5&&g_ota_service_errors==1&&state.active);ota_stream_poll();assert(cleanups==1);assert(event(SEDS_P2P_STREAM_ACCEPTED,NULL,0)==SEDS_ERR);}
  else {assert(terminal.phase==CLEAN_NONE&&!state.active&&cleanups==1);assert(event(SEDS_P2P_STREAM_ACCEPTED,NULL,0)==SEDS_OK);}
  return 0;
 }
 if(!strcmp(argv[1],"aba")){uint8_t q=5;request(&q,1);uint32_t original=work.original_epoch;assert(thread_comm_request_manual_abort()==TX_SUCCESS);recover_same_state();assert(thread_comm_manual_epoch()!=original);ota_stream_poll();assert(!work.phase&&terminal.phase==CLEAN_NONE&&!send_calls+0);return 0;}
 if(!strcmp(argv[1],"armed")){assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_ARMED)==TX_SUCCESS);finish();assert(!finishes&&!state.reset_requested&&last_reply[1]==OTA_STREAM_BAD_STATE);return 0;}
 if(!strcmp(argv[1],"restore")){assert(board_launch_latch_set_verified());finish();assert(!finishes&&!state.reset_requested&&last_reply[1]==OTA_STREAM_BAD_STATE);return 0;}
 finish();assert(finishes==1&&state.reset_requested&&!resets);
 if(!strcmp(argv[1],"launch_after_finish")){assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_LAUNCH)==TX_SUCCESS);assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_IN_FLIGHT)==TX_SUCCESS);ota_clock+=300;ota_stream_poll();assert(!resets&&!state.reset_requested&&board_launch_latch_is_set());return 0;}
 if(!strcmp(argv[1],"reset_aba")){assert(thread_comm_request_manual_abort()==TX_SUCCESS);recover_same_state();ota_clock+=300;ota_stream_poll();assert(!resets&&!state.reset_requested);return 0;}
 ota_clock+=300;ota_stream_poll();assert(resets==1);puts("GREEN: original safe epoch reset after actual prepare+reply; distinct from update prepared only");return 0;
}
'''
actual=(Path(os.environ['OTA_OWNER_SOURCE'])/'Core/Src/ota_stream.c').read_text();actual='\n'.join(l for l in actual.splitlines()if not l.startswith('#include'))
extra=extra.replace('ACTUAL_OTA',actual).replace('PREFIX',prefix)
# Begin already emitted exactly one response before testing ABA retirement.
extra=extra.replace('&&!send_calls+0','&&send_calls==1')
(out/'test.c').write_text(s+'\n'+extra)
cmd=json.loads((old/'BUILD.json').read_text())['command'];cmd=[str(out/'test.c')if x==str(old/'test.c')else'-I'+str(out)if x=='-I'+str(old)else str(out/'test')if x==str(old/'test')else x for x in cmd];cmd.insert(1,'-I'+str(base.LC/'update_lib/include'))
subprocess.run(cmd,check=True)
(out/'BUILD.json').write_text(json.dumps({'command':cmd,'limits':'Actual ota_stream module, thread/state/epoch/reset guard, real physical store and validated StopGPIO; LaunchCore OTA IO operation bodies + SDK retention are fault-injected boundaries, not full firmware image validation.'},indent=2)+'\n')
