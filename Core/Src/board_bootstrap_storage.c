#include "metadata_atomic.h"
#include "board_bootstrap_storage.h"
#include "bootstrap_marker.h"
#include "bootstrap_owner.h"
#include "persistent_store.h"
#include "persistent_store_health.h"
#include "board_recovery.h"
#include "board_recovery_boot.h"
#include "board_recovery_floor.h"
#include "board_abort_latch.h"
#include "thread_comm.h"
#include "flight_state_cache.h"
#include "tx_api.h"
#include <stdatomic.h>
#include <string.h>
enum {JOB_FIRST=1,JOB_SECOND=2,JOB_NORMAL=3,JOB_RECONCILE=4};
static _Atomic uint8_t eligible;
static _Atomic bool normal_eligible,uncertain_first;
static _Atomic bool pending,known_identity,repairable_floor,healthy_floor,runtime_ready,history_valid,floor_publication_pending;
/* Written before publishing healthy/identity or before tasks exist. Immutable
 * while a nonzero identity can arm recovery; main completion changes no history. */
static BootstrapMarker history;
static TX_MUTEX jobs_mutex;
static bool reply_waiting;static uint32_t reply_started;
static struct {BoardRecoveryRequest request;uint64_t source;void*context;BootstrapReply reply;BootstrapNow now;uint8_t result,kind,phase;bool valid,waiting,dirty;}job;
/* These query fields have ONE serialized SDK transport owner; main never writes. */
typedef struct {uint64_t nonce,issuer,source,generation;uint32_t epoch;bool valid;BoardRecoveryCompletedLaunch completed;} BootstrapQuery;
static BootstrapQuery query;
/* These are native object representations, never wire images. All tuple
 * ranges lie within their whole objects and consist of two unsigned u64s. */
#define PAIR_FIELD(type,field,offset,width) _Static_assert(offsetof(type,field)==offset&&sizeof(((type*)0)->field)==width,"native tuple field layout")
PAIR_FIELD(BootstrapQuery,nonce,0U,8U);PAIR_FIELD(BootstrapQuery,issuer,8U,8U);PAIR_FIELD(BootstrapQuery,source,16U,8U);PAIR_FIELD(BootstrapQuery,generation,24U,8U);PAIR_FIELD(BootstrapQuery,epoch,32U,4U);PAIR_FIELD(BootstrapQuery,valid,36U,1U);PAIR_FIELD(BootstrapQuery,completed,40U,16U);
PAIR_FIELD(BoardRecoveryCompletedLaunch,source_utc,0U,8U);PAIR_FIELD(BoardRecoveryCompletedLaunch,serial,8U,4U);
PAIR_FIELD(BootstrapMarker,state,0U,1U);PAIR_FIELD(BootstrapMarker,desired,1U,1U);PAIR_FIELD(BootstrapMarker,version,2U,1U);PAIR_FIELD(BootstrapMarker,kind,3U,1U);PAIR_FIELD(BootstrapMarker,first_epoch,4U,4U);PAIR_FIELD(BootstrapMarker,first_time,8U,8U);PAIR_FIELD(BootstrapMarker,first_nonce,16U,8U);PAIR_FIELD(BootstrapMarker,first_issuer,24U,8U);PAIR_FIELD(BootstrapMarker,first_expected_generation,32U,8U);
_Static_assert(offsetof(BoardRecoveryRequest,nonce)==0U&&offsetof(BoardRecoveryRequest,issuer_session)==8U&&16U<=sizeof(BoardRecoveryRequest),"request tuple within whole object");
_Static_assert(sizeof(BootstrapQuery)==56U&&16U<=sizeof(BootstrapQuery),"query tuple within whole object");
_Static_assert(sizeof(BootstrapMarker)==40U&&offsetof(BootstrapMarker,first_nonce)+16U<=sizeof(BootstrapMarker),"history tuple within whole object");
#undef PAIR_FIELD
static __attribute__((noinline,noclone)) bool same_tuple(const void*a,const void*b){return memcmp(a,b,16U)==0;}
static __attribute__((noinline,noclone,cold)) bool recovery_shape(const BoardRecoveryRequest*r){return r->operation==BOARD_RECOVERY_RECOVER&&r->desired_state<=BOARD_RECOVERY_MAX_STATE&&r->nonce&&r->issuer_session;}
static bool same_request(const BoardRecoveryRequest*a,const BoardRecoveryRequest*b){return memcmp(a,b,BOARD_RECOVERY_REQUEST_NATIVE_PREFIX_SIZE)==0;}
static bool marker_request(const BoardRecoveryRequest*r,uint64_t source){return r->desired_state==history.desired&&r->expected_abort_epoch==history.first_epoch&&r->expected_boot_generation==history.first_expected_generation&&source==history.first_time;}
static bool jobs_enter(void){if(!metadata_load_bool(&runtime_ready))return false;UINT status=tx_mutex_get(&jobs_mutex,TX_WAIT_FOREVER);if(status==TX_SUCCESS)return true;if(status!=TX_NOT_AVAILABLE&&status!=TX_WAIT_ABORTED)metadata_store_bool(&runtime_ready, (false));persistent_store_note_failure();return false;}
static bool jobs_leave(void){if(tx_mutex_put(&jobs_mutex)==TX_SUCCESS)return true;metadata_store_bool(&runtime_ready, (false));persistent_store_note_failure();return false;}
/* Caller holds physical owner. Return-1 for invalid/tombstone;0 absent;1/2
 * complete/pending. Virgin and healthy ABSENT remain distinct in strict info. */
static int read_marker(BootstrapMarker*out){
 uint8_t wire[40];size_t size=sizeof wire;launchcore_persist_strict_info_t info;
 launchcore_persist_status_t status=launchcore_persist_get_strict(BOOTSTRAP_MARKER_KEY,wire,&size,&info);
 if(status==LAUNCHCORE_PERSIST_NOT_FOUND&&info.record==LAUNCHCORE_PERSIST_RECORD_ABSENT&&(info.health==LAUNCHCORE_PERSIST_HEALTH_HEALTHY||info.health==LAUNCHCORE_PERSIST_HEALTH_VIRGIN))return 0;
 if(status!=LAUNCHCORE_PERSIST_OK||!bootstrap_marker_decode(wire,size,out))return -1;
 return out->state?2:1;
}
static bool read_floor(uint64_t*issuer,uint64_t*time,bool*absent){
 uint8_t wire[16];size_t size=sizeof wire;launchcore_persist_strict_info_t info;
 launchcore_persist_status_t status=launchcore_persist_get_strict(UINT32_C(0x4252464c),wire,&size,&info);
 *absent=status==LAUNCHCORE_PERSIST_NOT_FOUND&&info.record==LAUNCHCORE_PERSIST_RECORD_ABSENT&&(info.health==LAUNCHCORE_PERSIST_HEALTH_HEALTHY||info.health==LAUNCHCORE_PERSIST_HEALTH_VIRGIN);
 if(*absent){*issuer=0;*time=0;return true;}
 if(status!=LAUNCHCORE_PERSIST_OK||size!=16U)return false;
 uint64_t decoded_issuer=bootstrap_read64(wire),decoded_time=bootstrap_read64(wire+8);
 if(!decoded_issuer||!decoded_time||decoded_time==UINT64_MAX)return false;
 *issuer=decoded_issuer;*time=decoded_time;return true;
}
/* Caller owns media. A missing critical key is never inferred from an error. */
typedef struct {BootstrapMarker marker;int state;uint64_t generation,issuer,time;bool identity,absent,virgin;} MetadataView;
static bool read_view_locked(MetadataView*v){
 *v=(MetadataView){0};v->state=read_marker(&v->marker);if(v->state<0)return false;
 uint8_t bytes[8];size_t size=8;launchcore_persist_strict_info_t info;
 launchcore_persist_status_t s=launchcore_persist_get_strict(UINT32_C(0x42524745),bytes,&size,&info);
 bool missing=s==LAUNCHCORE_PERSIST_NOT_FOUND&&info.record==LAUNCHCORE_PERSIST_RECORD_ABSENT&&(info.health==LAUNCHCORE_PERSIST_HEALTH_HEALTHY||info.health==LAUNCHCORE_PERSIST_HEALTH_VIRGIN);
 v->virgin=missing&&info.health==LAUNCHCORE_PERSIST_HEALTH_VIRGIN;
 if(!missing){if(s!=LAUNCHCORE_PERSIST_OK||size!=8)return false;uint64_t decoded_generation=bootstrap_read64(bytes);if(!decoded_generation||decoded_generation==UINT64_MAX)return false;v->generation=decoded_generation;v->identity=true;}
 return read_floor(&v->issuer,&v->time,&v->absent);
}
/* Pure RAM bookkeeping only. Caller is exclusive boot or holds jobs_mutex.
 * Return candidate eligibility; boot publishes it only after its separate
 * Stop/cache checks, runtime publishes it at its existing guarded point. */
static uint8_t cache_view_health(const MetadataView*v,uint64_t current){
 bool identity_ok=v->identity&&current&&current!=UINT64_MAX&&v->generation==current;
 bool floor_ok=identity_ok&&!v->absent;
 metadata_store_bool(&known_identity, (identity_ok));
 if(identity_ok)metadata_store_bool(&uncertain_first, (false));
 metadata_store_bool(&healthy_floor, (floor_ok));metadata_store_bool(&repairable_floor, (identity_ok&&v->absent&&v->state==2));metadata_store_bool(&normal_eligible, (identity_ok&&v->state!=2));
 if(!current&&!v->identity&&(v->state==0||(v->state==2&&v->marker.kind==0&&v->marker.first_expected_generation==UINT64_MAX)))return v->virgin?BOOTSTRAP_VIRGIN:BOOTSTRAP_MISSING_GENERATION;
 return identity_ok&&v->absent&&v->state==0?BOOTSTRAP_MISSING_FLOOR:BOOTSTRAP_NONE;
}
void board_bootstrap_storage_probe_boot(void){
 (void)board_recovery_floor_prime_boot();
 metadata_store_bool(&uncertain_first, (false));metadata_store_bool(&normal_eligible, (false));eligible=BOOTSTRAP_NONE;metadata_store_bool(&pending, (false));metadata_store_bool(&known_identity, (false));metadata_store_bool(&repairable_floor, (false));metadata_store_bool(&healthy_floor, (false));metadata_store_bool(&history_valid, (false));metadata_store_bool(&floor_publication_pending, (false));
 if(!persistent_store_enter())return;
 MetadataView view;bool valid=read_view_locked(&view);bool released=persistent_store_leave();
 if(view.state>0){history=view.marker;metadata_store_bool(&history_valid, (true));}
 /* A verified PENDING prefix is an earlier explicit Stop even if a later
  * critical read fails. Restore its inhibit, never repair an unproven key. */
 if(view.state==2){metadata_store_bool(&pending, (true));persistent_store_hold_bootstrap();if(!board_abort_latch_prepare_recovery()){persistent_store_note_failure();return;}}
 if(!valid||!released){persistent_store_note_failure();return;}
 uint8_t candidate=cache_view_health(&view,board_recovery_boot_current());
 if(!view.absent&&view.time&&!board_recovery_floor_accept_bootstrap(view.time)){persistent_store_note_failure();return;}
 if(view.state==2&&!board_recovery_floor_note_bootstrap_attempt(view.marker.first_time)){persistent_store_note_failure();return;}
 if(!persistent_store_bootstrap_faulted())eligible=candidate;
}
bool board_bootstrap_storage_uncertain_first(void){return metadata_load_bool(&uncertain_first);}
uint64_t board_bootstrap_storage_query_generation(void){
 if(board_bootstrap_storage_uncertain_first())return 0U;
 uint64_t generation=board_recovery_boot_current();
 /* Published MAX is exhaustion, never bootstrap eligibility. */
 if(generation==UINT64_MAX)return 0U;
 if(generation)return generation;
 return metadata_load_bool(&runtime_ready)&&eligible!=BOOTSTRAP_NONE&&!persistent_store_bootstrap_faulted()?UINT64_MAX:0U;
}
bool board_bootstrap_storage_pending(void){return metadata_load_bool(&pending);}
bool board_bootstrap_storage_query_healthy(void){return metadata_load_bool(&runtime_ready)&&metadata_load_bool(&pending)&&metadata_load_bool(&known_identity)&&metadata_load_bool(&healthy_floor)&&!metadata_load_bool(&floor_publication_pending)&&persistent_store_bootstrap_metadata_ready()&&!persistent_store_bootstrap_faulted();}
static bool note_query_locked(const BoardRecoveryRequest*r,uint64_t source,uint64_t generation,uint32_t epoch){
 if(!metadata_load_bool(&pending)) {
  BoardRecoveryCompletedLaunch completed={0};
  if(!metadata_load_bool(&runtime_ready)||!metadata_load_bool(&normal_eligible)||job.waiting||
     !r||r->operation!=BOARD_RECOVERY_QUERY||!r->nonce||!r->issuer_session||
     !generation||generation==UINT64_MAX||generation!=board_recovery_boot_current()||
     persistent_store_bootstrap_faulted()||
     !board_recovery_completed_query(epoch,source,&completed))return false;
  query.nonce=r->nonce;query.issuer=r->issuer_session;query.source=source;
  query.generation=generation;query.epoch=epoch;query.completed=completed;query.valid=true;return true;
 }
 uint64_t cutoff=0U;if(!board_recovery_floor_cached_cutoff(&cutoff)||source<=cutoff)return false;
 bool uncertain=board_bootstrap_storage_uncertain_first();
 if(!metadata_load_bool(&runtime_ready)||!r||r->operation!=BOARD_RECOVERY_QUERY||!r->nonce||!r->issuer_session||source<=history.first_time||same_tuple(r,(const uint8_t*)&history+offsetof(BootstrapMarker,first_nonce))||(!uncertain&&(!metadata_load_bool(&known_identity)||(!metadata_load_bool(&healthy_floor)&&!metadata_load_bool(&repairable_floor))||!generation||generation==UINT64_MAX||generation!=board_recovery_boot_current()))||persistent_store_bootstrap_faulted())return false;
 BoardRecoverySnapshot live={0};(void)board_recovery_snapshot(&live);
 uint8_t required=BOARD_RECOVERY_SUPPORTED|BOARD_RECOVERY_OUTPUTS_VALID|BOARD_RECOVERY_SAFE_OUTPUTS|BOARD_RECOVERY_ABORT_LATCHED;
 if(!epoch||epoch==UINT32_MAX||epoch!=live.abort_epoch||(live.flags&required)!=required||!board_recovery_boot_retry_is_safe())return false;
 query.nonce=r->nonce;query.issuer=r->issuer_session;query.source=source;query.generation=generation;query.epoch=epoch;query.completed=(BoardRecoveryCompletedLaunch){0};query.valid=true;return true;
}
bool board_bootstrap_storage_note_query(const BoardRecoveryRequest*r,uint64_t source,uint64_t generation,uint32_t epoch){
 if(!jobs_enter())return false;
 bool valid=note_query_locked(r,source,generation,epoch);bool released=jobs_leave();return valid&&released;
}
/* Share exact strict readback checks, never skip bank/CRC/ancestry proof. */
static bool record_matches_locked(uint32_t key,const uint8_t*expected,size_t length){
 uint8_t observed[40];size_t size=sizeof observed;launchcore_persist_strict_info_t info;
 return length<=sizeof observed&&launchcore_persist_get_strict(key,observed,&size,&info)==LAUNCHCORE_PERSIST_OK&&size==length&&!memcmp(observed,expected,length);
}
static bool owner_complete(uint64_t generation,uint32_t old_epoch,uint32_t new_epoch,uint8_t desired,uint64_t issuer,uint64_t source){
 if(!jobs_enter())return false;
 BootstrapMarker saved=history;bool normal=saved.kind==1U&&job.valid&&job.waiting&&job.kind==JOB_NORMAL&&job.source==source&&job.request.expected_abort_epoch==old_epoch&&job.request.nonce==saved.first_nonce&&job.request.issuer_session==saved.first_issuer&&job.request.expected_boot_generation==saved.first_expected_generation;
 bool time_ok=normal?source==saved.first_time:source>saved.first_time;bool unlocked=jobs_leave();
 if(!unlocked||!metadata_load_bool(&pending)||!metadata_load_bool(&runtime_ready)||!metadata_load_bool(&known_identity)||!time_ok||generation!=board_recovery_boot_current()||!board_recovery_bootstrap_verify_applied(old_epoch,new_epoch,desired))return false;
 if(!persistent_store_enter())return false;
 BootstrapMarker original=saved,actual;int state=read_marker(&actual);uint8_t marker[40],floor[16],wire[40];size_t marker_size=bootstrap_marker_size(&original);BootstrapMarker complete=original;complete.state=0U;bootstrap_marker_encode(marker,&complete);bootstrap_write64(floor,issuer);bootstrap_write64(floor+8,source);
 bool valid=state==1||state==2;
 if(valid){actual.state=1U;valid=bootstrap_marker_same(&actual,&original);}
 size_t size=8U;launchcore_persist_strict_info_t info;
 if(valid)valid=launchcore_persist_get_strict(UINT32_C(0x42524745),wire,&size,&info)==LAUNCHCORE_PERSIST_OK&&size==8U&&bootstrap_read64(wire)==generation;
 size=16U;if(valid)valid=launchcore_persist_get_strict(UINT32_C(0x4252464c),wire,&size,&info)==LAUNCHCORE_PERSIST_OK&&size==16U&&!memcmp(wire,floor,16U);
 if(valid&&state==2)valid=launchcore_persist_set_strict(BOOTSTRAP_MARKER_KEY,marker,marker_size,&info)==LAUNCHCORE_PERSIST_OK;
 if(valid)valid=record_matches_locked(BOOTSTRAP_MARKER_KEY,marker,marker_size);
 bool released=persistent_store_leave();if(!valid||!released){persistent_store_note_failure();return false;}
 if(!persistent_store_verify_bootstrap(generation,floor,marker,true))return false;
 uint64_t cutoff=0U;
 if(!board_recovery_floor_cached_cutoff(&cutoff)||cutoff!=source){metadata_store_bool(&floor_publication_pending, (true));return false;}
 if(!board_recovery_bootstrap_publish_complete(old_epoch,new_epoch,desired))return false;
 metadata_store_bool(&floor_publication_pending, (false));metadata_store_bool(&pending, (false));metadata_store_bool(&normal_eligible, (true));eligible=BOOTSTRAP_NONE;return true;
}
bool board_bootstrap_storage_bind_owner(uint8_t(*stop)(void)){
 if(tx_mutex_create(&jobs_mutex,"bootstrap_jobs",TX_INHERIT)!=TX_SUCCESS){persistent_store_note_failure();return false;}
 if(!board_recovery_bind_bootstrap_owner(owner_complete,stop)){persistent_store_note_failure();return false;}
 metadata_store_bool(&runtime_ready, (true));return true;
}
/* No SDK, GPIO or state mutex here. Physical owner is already released. */
static bool publish_view(const MetadataView*v){
 uint64_t current=board_recovery_boot_current();
 if(v->identity&&!current&&v->state==2){if(!board_recovery_boot_publish_bootstrap(v->generation))return false;current=v->generation;}
 if(!jobs_enter())return false;
 if(v->state){history=v->marker;metadata_store_bool(&history_valid, (true));}
 eligible=cache_view_health(v,current);
 if(v->state==2){metadata_store_bool(&pending, (true));persistent_store_hold_bootstrap();}
 return jobs_leave();
}
/* One physical transaction skeleton; eligibility remains kind-specific.
 * The actual main Stop/readback precedes every PENDING/key effect. */
static BoardBootstrapResult write_transaction(const BoardRecoveryRequest*r,uint64_t source,uint8_t kind){
 int32_t prepared=board_recovery_prepare_storage_only(r->expected_abort_epoch);if(prepared)return prepared==-10?BOOTSTRAP_STALE:BOOTSTRAP_IO;
 persistent_store_hold_bootstrap();if(!board_recovery_floor_note_bootstrap_attempt(source))return BOOTSTRAP_IO;
 bool max_first=(kind==JOB_FIRST||kind==JOB_RECONCILE)&&r->expected_boot_generation==UINT64_MAX&&!board_recovery_boot_current();
 BootstrapMarker retained={0};bool retained_valid=false;
 if(max_first||kind==JOB_SECOND){
  if(!jobs_enter())return BOOTSTRAP_IO;
  if(max_first&&!metadata_load_bool(&history_valid)){bootstrap_marker_request(&history,r,source,1U,0U);metadata_store_bool(&history_valid, (true));}
  retained=history;retained_valid=metadata_load_bool(&history_valid);if(!jobs_leave())return BOOTSTRAP_IO;
 }
 metadata_store_bool(&floor_publication_pending, (true));if(!persistent_store_enter())return BOOTSTRAP_IO;
 if(max_first){eligible=BOOTSTRAP_NONE;metadata_store_bool(&pending, (true));metadata_store_bool(&uncertain_first, (true));}
 MetadataView view;bool valid=read_view_locked(&view);uint64_t current=board_recovery_boot_current(),generation=view.identity?view.generation:1U;
 bool identity_case=false,floor_case=false,readonly_identity=false;
 if(kind==JOB_RECONCILE){
  valid=valid&&max_first&&retained_valid&&retained.kind==0&&retained.first_expected_generation==UINT64_MAX&&r->desired_state==retained.desired&&source>retained.first_time&&source>view.time&&(view.state==0||(view.state==2&&bootstrap_marker_same(&view.marker,&retained)));
  valid=valid&&(view.identity?view.state==2:true);readonly_identity=valid&&view.identity;
 }else if(kind==JOB_FIRST){
  identity_case=valid&&!view.identity&&!current&&view.state!=1&&r->expected_boot_generation==UINT64_MAX;
  floor_case=valid&&view.identity&&view.generation==current&&view.absent&&view.state==0&&r->expected_boot_generation==current;
  valid=(identity_case||floor_case)&&source>view.time&&(view.state!=2||source>view.marker.first_time);
 }else{
  valid=valid&&view.identity&&view.generation==r->expected_boot_generation&&view.generation==current&&source>view.time;
  if(valid)valid=kind==JOB_NORMAL?(view.state!=2||view.marker.kind==1U):retained_valid&&retained.kind==0U&&(view.state==1||view.state==2)&&view.marker.kind==0U&&source>view.marker.first_time;
  if(valid&&kind==JOB_SECOND){
   /* The fresh intent authorizes new effects. Original tuple/version/kind and
    * desired remain immutable history; normalize lifecycle ONLY for equality. */
   retained.state=1U;view.marker.state=1U;
   valid=bootstrap_marker_same(&view.marker,&retained);
  }
 }
 BootstrapMarker next;
 if(kind==JOB_RECONCILE)next=retained;
 else if(kind==JOB_SECOND||(kind==JOB_FIRST&&valid&&view.state==2))next=view.marker;
 else bootstrap_marker_request(&next,r,source,kind==JOB_FIRST&&!floor_case?1U:2U,kind==JOB_NORMAL?1U:0U);
 uint8_t marker[40],identity[8],floor[16];size_t marker_size=bootstrap_marker_size(&next);bootstrap_marker_encode(marker,&next);bootstrap_write64(identity,generation);bootstrap_write64(floor,readonly_identity?view.issuer:r->issuer_session);bootstrap_write64(floor+8,readonly_identity?view.time:source);launchcore_persist_strict_info_t info;
 if(valid&&max_first&&view.state==2)valid=retained_valid&&retained.kind==0&&retained.first_expected_generation==UINT64_MAX&&bootstrap_marker_same(&view.marker,&retained);
 if(valid&&(kind==JOB_NORMAL||(kind==JOB_SECOND&&view.state==1)||((kind==JOB_FIRST||kind==JOB_RECONCILE)&&view.state==0)))valid=((kind==JOB_FIRST||kind==JOB_RECONCILE)&&view.virgin?launchcore_persist_provision_virgin_strict(BOOTSTRAP_MARKER_KEY,marker,marker_size,&info):launchcore_persist_set_strict(BOOTSTRAP_MARKER_KEY,marker,marker_size,&info))==LAUNCHCORE_PERSIST_OK;
 if(valid)valid=record_matches_locked(BOOTSTRAP_MARKER_KEY,marker,marker_size);
 if(valid&&(kind==JOB_FIRST||kind==JOB_RECONCILE)&&!view.identity)valid=launchcore_persist_set_strict(UINT32_C(0x42524745),identity,8,&info)==LAUNCHCORE_PERSIST_OK;
 if(valid&&!readonly_identity)valid=launchcore_persist_set_strict(UINT32_C(0x4252464c),floor,16,&info)==LAUNCHCORE_PERSIST_OK;
 MetadataView actual;bool observed_valid=read_view_locked(&actual);bool released=persistent_store_leave();
 if(!released)return BOOTSTRAP_IO;
 if(observed_valid&&metadata_load_bool(&uncertain_first)&&actual.state&&(!retained_valid||!bootstrap_marker_same(&actual.marker,&retained)))observed_valid=false;
 if(observed_valid&&!publish_view(&actual))valid=false;
 if(!valid||!observed_valid||((!readonly_identity||!view.absent)&&!persistent_store_verify_bootstrap(generation,floor,marker,false))){persistent_store_note_failure();return BOOTSTRAP_IO;}
 uint64_t accepted=readonly_identity?view.time:source;
 if((!readonly_identity||!view.absent)&&(!board_recovery_floor_accept_bootstrap(accepted)||thread_comm_publish_command_cutoff(accepted)!=TX_SUCCESS))return BOOTSTRAP_IO;
 metadata_store_bool(&floor_publication_pending, (false));metadata_store_bool(&uncertain_first, (false));return BOOTSTRAP_STORED_INHIBITED;
}
BoardBootstrapResult board_bootstrap_storage_first_leg(const BoardRecoveryRequest*r,uint64_t source,uint64_t now,BoardBootstrapReply*out){
 if(!r||!out||!recovery_shape(r)||!r->expected_boot_generation||!r->expected_abort_epoch||r->expected_abort_epoch==UINT32_MAX)return BOOTSTRAP_INVALID;
 *out=(BoardBootstrapReply){0};if(!bootstrap_request_fresh(source,now))return BOOTSTRAP_STALE;
 BoardBootstrapResult result=write_transaction(r,source,JOB_FIRST);if(result!=BOOTSTRAP_STORED_INHIBITED)return result;
 BoardRecoverySnapshot live={0};(void)board_recovery_snapshot(&live);out->durable_generation=board_recovery_boot_current();out->actual_abort_epoch=live.abort_epoch;
 return live.abort_epoch==r->expected_abort_epoch&&(live.flags&BOARD_RECOVERY_ABORT_LATCHED)?BOOTSTRAP_STORED_INHIBITED:BOOTSTRAP_IO;
}
static bool prepare_transaction(const BoardRecoveryRequest*r,uint64_t source,bool normal){
 if(write_transaction(r,source,normal?JOB_NORMAL:JOB_SECOND)!=BOOTSTRAP_STORED_INHIBITED)return false;
 if(board_recovery_begin_prepare(r->desired_state,r->expected_abort_epoch)!=0)return false;
 if(flight_state_cache_prepare_recovery(r->desired_state)!=0||!board_recovery_bind_bootstrap_request(r->expected_boot_generation,r->expected_abort_epoch,r->desired_state,r->issuer_session,source)){board_recovery_cancel_prepare(r->desired_state,r->expected_abort_epoch);return false;}
 BoardRecoverySnapshot snapshot={0};return board_recovery_apply_safe(r->desired_state,r->expected_abort_epoch,&snapshot)==-14;
}
int32_t board_bootstrap_storage_stage_first_leg(const BoardRecoveryRequest*r,uint64_t source,uint64_t network_now,uint32_t live_epoch,void*context,BootstrapReply reply,BootstrapNow now){
 if(!r||!reply||!now||!recovery_shape(r))return BOARD_RECOVERY_RESULT_INVALID;
 if(!jobs_enter())return BOARD_RECOVERY_RESULT_IO;
 bool duplicate=job.valid&&same_tuple(r,&job.request);int32_t result=BOARD_RECOVERY_RESULT_IO;
 if(duplicate){
  if(!same_request(r,&job.request)||source!=job.source)result=BOARD_RECOVERY_RESULT_INVALID;
  else if(!bootstrap_request_fresh(source,network_now))result=BOARD_RECOVERY_RESULT_STALE;
  else if(job.waiting)result=job.phase?BOARD_RECOVERY_RESULT_PENDING:-1;
  else result=job.result==BOARD_RECOVERY_RESULT_OK&&live_epoch!=r->expected_abort_epoch+1U?BOARD_RECOVERY_RESULT_STALE:job.result;
 }else if(job.waiting)result=BOARD_RECOVERY_RESULT_IO;
 else if(metadata_load_bool(&history_valid)&&same_tuple(r,(const uint8_t*)&history+offsetof(BootstrapMarker,first_nonce)))result=!marker_request(r,source)?BOARD_RECOVERY_RESULT_INVALID:history.kind==0U||history.state?BOARD_RECOVERY_RESULT_IO:BOARD_RECOVERY_RESULT_STALE;
 else if(!live_epoch||live_epoch==UINT32_MAX||persistent_store_bootstrap_faulted())result=BOARD_RECOVERY_RESULT_IO;
 else if(!bootstrap_request_fresh(source,network_now)||r->expected_abort_epoch!=live_epoch||(job.valid&&source<=job.source)||!board_recovery_floor_allows_recovery_time(r->issuer_session,source))result=BOARD_RECOVERY_RESULT_STALE;
 else{
  uint64_t generation=board_recovery_boot_current();uint8_t kind=0;
  /* The QUERY tuple has one SDK owner; jobs_mutex and !job.waiting exclude
   * runtime publication here. Share this exact common binding, not the
   * independent FIRST/NORMAL/SECOND generation or history conditions. */
  bool uncertain=board_bootstrap_storage_uncertain_first();
  bool armed=query.valid&&bootstrap_request_fresh(query.source,network_now)&&r->issuer_session==query.issuer&&r->expected_abort_epoch==query.epoch&&!same_tuple(r,&query);
  if(uncertain&&metadata_load_bool(&history_valid)&&history.kind==0&&history.first_expected_generation==UINT64_MAX&&r->desired_state==history.desired&&r->expected_boot_generation==UINT64_MAX&&armed&&!query.generation&&source>history.first_time)kind=JOB_RECONCILE;
  else if(!uncertain&&(eligible==BOOTSTRAP_VIRGIN||eligible==BOOTSTRAP_MISSING_GENERATION)&&!generation&&r->expected_boot_generation==UINT64_MAX)kind=JOB_FIRST;
  else if(metadata_load_bool(&normal_eligible)&&!metadata_load_bool(&pending)&&generation&&generation!=UINT64_MAX&&r->expected_boot_generation==generation)kind=JOB_NORMAL;
  else if(metadata_load_bool(&pending)&&metadata_load_bool(&known_identity)&&(metadata_load_bool(&healthy_floor)||metadata_load_bool(&repairable_floor))&&armed&&query.generation==generation&&r->expected_boot_generation==query.generation&&source>history.first_time)kind=history.kind==1U?JOB_NORMAL:JOB_SECOND;
  if(kind){
   /* Claim the RAM hold under the same state lock as Launch BEFORE changing
    * this query/job. An already accepted Launch leaves all of them untouched. */
   int32_t held=board_recovery_stage_storage_hold(r->expected_abort_epoch);
   /* Only NORMAL and only a QUERY observed after this exact completed owner.
    * Existing bootstrap legs and every active/prelaunch recovery retain the
    * old exclusion. A new issuer, nonce replay, stale query or boot cannot use
    * another review's completion proof. No source timestamp is renewed. */
   if(held==-10&&kind==JOB_NORMAL&&!metadata_load_bool(&pending)&&
      metadata_load_bool(&normal_eligible)&&armed&&query.completed.serial&&
      query.generation==generation&&r->expected_boot_generation==query.generation&&
      source>query.source&&query.source>query.completed.source_utc)
    held=board_recovery_stage_completed_hold(r->expected_abort_epoch,source,query.source,&query.completed);
   if(held!=0)result=held==-10?BOARD_RECOVERY_RESULT_STALE:BOARD_RECOVERY_RESULT_IO;
   else {
    query.valid=false;job.request=*r;job.source=source;job.context=context;job.reply=reply;job.now=now;job.result=BOARD_RECOVERY_RESULT_IO;job.kind=kind;job.phase=0;job.valid=true;job.waiting=true;job.dirty=false;
    result=-1;
   }
  }else result=BOARD_RECOVERY_RESULT_STALE;
 }
 return jobs_leave()?result:BOARD_RECOVERY_RESULT_IO;
}
void board_bootstrap_storage_service(void){
 if(!metadata_load_bool(&runtime_ready))return;
 BoardRecoveryRequest request={0};uint64_t source=0;void*context=0;BootstrapReply reply=0;BootstrapNow now=0;bool waiting=false,dirty=false;uint8_t result=BOARD_RECOVERY_RESULT_IO,kind=0,phase=0;
 if(!jobs_enter())return;
 if(job.valid&&(job.waiting||job.dirty)){request=job.request;source=job.source;context=job.context;reply=job.reply;now=job.now;waiting=job.waiting;dirty=job.dirty;result=job.result;kind=job.kind;phase=job.phase;}
 if(!jobs_leave())return;
 bool terminal=false,progress=false,effect=phase!=0U;
 if(waiting){
  BoardRecoverySnapshot snapshot={0};(void)board_recovery_snapshot(&snapshot);
  /* Freshness gates the first effect. A stopped board whose owner apply is
   * not blocked finishes to its safe target; a blocked one still expires. */
  if(!phase&&!bootstrap_request_fresh(source,now(context))){result=BOARD_RECOVERY_RESULT_STALE;terminal=true;}
  else if(!phase&&snapshot.abort_epoch!=request.expected_abort_epoch){result=BOARD_RECOVERY_RESULT_STALE;terminal=true;}
  else if(!phase){
   if(kind==JOB_FIRST||kind==JOB_RECONCILE){BoardBootstrapReply observed;BoardBootstrapResult status=kind==JOB_FIRST?board_bootstrap_storage_first_leg(&request,source,now(context),&observed):write_transaction(&request,source,JOB_RECONCILE);result=status==BOOTSTRAP_STALE?BOARD_RECOVERY_RESULT_STALE:status==BOOTSTRAP_INVALID?BOARD_RECOVERY_RESULT_INVALID:BOARD_RECOVERY_RESULT_IO;terminal=true;}
   else if(effect=true,!prepare_transaction(&request,source,kind==JOB_NORMAL)){result=BOARD_RECOVERY_RESULT_IO;terminal=true;}
   else{phase=1;result=BOARD_RECOVERY_RESULT_PENDING;progress=true;}
  }else{
   board_recovery_execute_owned();
   int32_t applied=board_recovery_apply_safe(request.desired_state,request.expected_abort_epoch,&snapshot);
   if(applied!=-14){result=applied==0?BOARD_RECOVERY_RESULT_OK:applied==-10?BOARD_RECOVERY_RESULT_STALE:BOARD_RECOVERY_RESULT_IO;terminal=true;}
   else if(!bootstrap_request_fresh(source,now(context))){result=BOARD_RECOVERY_RESULT_STALE;terminal=true;}
  }
  /* A refusal before any Stop/storage effect latches nothing: a reset of a
   * running board must not leave it aborted, and its RAM hold is released
   * unless a pending marker still requires it. */
  if(terminal&&kind!=JOB_FIRST&&kind!=JOB_RECONCILE&&result!=BOARD_RECOVERY_RESULT_OK){
   if(effect)board_recovery_bootstrap_retain_stop();
   else if(!metadata_load_bool(&pending))(void)persistent_store_release_unused_hold();
  }
  if(!jobs_enter())return;
  job.phase=phase;job.result=result;if(terminal){job.waiting=false;job.dirty=true;dirty=true;reply_waiting=false;}
  if(!jobs_leave())return;
 }
 /* Progress is application Pending, never an ACK-as-physical-confirmation. */
 if(progress&&reply)(void)reply(context,&request,BOARD_RECOVERY_RESULT_PENDING);
 if(dirty&&reply){
  uint32_t tick=(uint32_t)tx_time_get();if(reply_waiting&&(uint32_t)(tick-reply_started)<(TX_TIMER_TICKS_PER_SECOND+9U)/10U)return;
  BoardRecoverySnapshot snapshot={0};(void)board_recovery_snapshot(&snapshot);
  if(result==BOARD_RECOVERY_RESULT_OK&&(!persistent_store_controls_ready()||snapshot.abort_epoch!=request.expected_abort_epoch+1U||(snapshot.flags&BOARD_RECOVERY_ABORT_LATCHED)))result=BOARD_RECOVERY_RESULT_STALE;
  if(!reply(context,&request,result)){reply_waiting=true;reply_started=tick;return;}
  reply_waiting=false;if(!jobs_enter())return;
  if(job.valid&&same_request(&job.request,&request)&&job.source==source)job.dirty=false;
  (void)jobs_leave();
 }
}
