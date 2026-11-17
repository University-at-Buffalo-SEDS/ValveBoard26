#include "persistent_store_health.h"
#include "board_recovery_transport.h"
#include "board_recovery_wire.h"
#include "board_recovery_boot.h"
#include "board_recovery_floor.h"
#include "board_recovery.h"
#include "flight_state_cache.h"
#include "thread_comm.h"
#include "board_bootstrap_storage.h"
#include "bootstrap_owner.h"
#include "bootstrap_marker.h"
#include "network_stamp.h"
#include <stdatomic.h>
#include <string.h>
static SedsRouter *recovery_router;
void board_recovery_transport_set_router(SedsRouter *router) { recovery_router=router; }
static atomic_flag dispatching=ATOMIC_FLAG_INIT;
/* A delivered reliable packet that is deliberately not applied (stale,
 * unauthorized, malformed, refused in this state) is still acknowledged: an
 * SDK handler error withholds the receipt and the station retransmits the
 * same refused packet forever. Count it here instead. */
volatile uint32_t g_reliable_refusal_count __attribute__((used));
SedsResult board_recovery_refused(void){++g_reliable_refusal_count;return SEDS_OK;}
static uint8_t authorized_primary(SedsRouter *router,const SedsPacketView *packet)
{
 return packet!=NULL && seds_router_sender_matches_hostname(router,
     packet->sender,packet->sender_len,"GS",2U);
}
uint8_t board_recovery_manual_stop_source_allowed(SedsRouter*router,const SedsPacketView*packet){
 return router&&packet&&(authorized_primary(router,packet)||seds_router_sender_matches_hostname(router,packet->sender,packet->sender_len,"RRLS",4U));
}
SedsResult board_recovery_sync_command_cutoff(void)
{
 uint64_t cutoff=0U;
 if (!board_recovery_floor_read_cutoff(&cutoff)) return SEDS_IO;
 return cutoff==0U || thread_comm_publish_command_cutoff(cutoff)==TX_SUCCESS ? SEDS_OK : SEDS_IO;
}
/* THE request-validity window (15 s old to 1 s ahead), judged by SEDSnet
 * against this node's network time and translated across the time steps it
 * remembers. A stamp from a newer epoch is fresh only while this node is
 * still re-anchoring to it; an untranslatable older epoch or no network time
 * is never fresh. Never compared with a local clock. */
bool board_recovery_request_fresh(SedsRouter*router,uint64_t source){
 bool fresh=false;
 return router&&source&&source!=UINT64_MAX&&
  seds_router_network_stamp_fresh(router,network_stamp_epoch(source),network_stamp_ms(source),
   BOARD_RECOVERY_FRESH_AGE_MS,BOARD_RECOVERY_FRESH_AHEAD_MS,&fresh)==SEDS_OK&&fresh;
}
/* Restore the network epoch of the persisted replay floor (and any attempted
 * recovery stamp) so the network's epoch never falls below it, even after a
 * whole-stack power cycle restarted every node at epoch 0. */
SedsResult board_recovery_raise_time_epoch(SedsRouter*router){
 uint64_t mark=0U;
 if(!router)return SEDS_BAD_ARG;
 if(!board_recovery_floor_cached_cutoff(&mark)&&!board_recovery_floor_read_cutoff(&mark))return SEDS_IO;
 return mark&&mark!=UINT64_MAX&&network_stamp_epoch(mark)?seds_router_raise_time_epoch(router,network_stamp_epoch(mark),NULL):SEDS_OK;
}
uint8_t board_recovery_command_is_stale(const SedsPacketView *packet)
{
 if (packet==0) return 1U;
 const uint64_t source=network_stamp_of(packet);
 if (!board_recovery_command_time_allowed(source)) return 1U;
 if (board_recovery_floor_active() && !board_recovery_request_fresh(recovery_router,source)) return 1U;
 return 0U;
}
uint8_t board_recovery_managed_update_allowed(const SedsPacketView *packet)
{
 if (!board_recovery_floor_is_healthy()) return 0U;
 if (!board_recovery_floor_active()) return packet!=NULL && packet->ty==SEDS_DT_FLIGHT_STATE ? recovery_router!=NULL && authorized_primary(recovery_router,packet) : 1U;
 return recovery_router!=0 && authorized_primary(recovery_router,packet) &&
        !board_recovery_command_is_stale(packet);
}
uint8_t board_recovery_automatic_is_stale(const SedsPacketView *packet)
{
 return board_recovery_command_is_stale(packet);
}

/* Issuer of the last valid station QUERY, echoed by the Abort push. */
static uint64_t last_query_issuer;
static SedsResult send_status(SedsRouter*router,uint8_t result,const BoardRecoveryRequest*request,
                            uint64_t generation,const BoardRecoverySnapshot*snapshot,uint32_t uptime_ms) {
 uint8_t status[BOARD_RECOVERY_STATUS_SIZE];
 board_recovery_encode_status(status,result,snapshot->actual_state,(uint8_t)(snapshot->flags|BOARD_RECOVERY_VERIFIED_IMPLEMENTATION|BOARD_RECOVERY_MANUAL_UTC_IMPLEMENTATION|BOARD_RECOVERY_RESYNC_IMPLEMENTATION),request,
  generation,snapshot->abort_epoch,uptime_ms,snapshot->actual_outputs_mask);
 return seds_router_log_queue_typed(router,SEDS_DT_BOARD_STATE_STATUS,status,sizeof status,sizeof(uint8_t),SEDS_EK_UNSIGNED);
}
static bool bootstrap_fresh(void*context,uint64_t source) {
 return board_recovery_request_fresh((SedsRouter*)context,source);
}
static bool bootstrap_reply(void*context,const BoardRecoveryRequest*request,uint8_t result) {
 BoardRecoverySnapshot snapshot={.actual_state=255U};(void)board_recovery_snapshot(&snapshot);
 return send_status((SedsRouter*)context,result,request,board_recovery_boot_current(),&snapshot,
  (uint32_t)(((uint64_t)tx_time_get()*1000U)/TX_TIMER_TICKS_PER_SECOND))==SEDS_OK;
}
static SedsResult handle_serialized(SedsRouter*router,const SedsPacketView*packet,uint32_t local_endpoint,uint32_t uptime_ms) {
 if(!router||!packet)return SEDS_BAD_ARG;
 BoardRecoveryRequest request;
 if(packet->ty!=SEDS_DT_BOARD_RECOVERY_REQUEST||packet->num_endpoints!=1U||!packet->endpoints||packet->endpoints[0]!=local_endpoint||
    !authorized_primary(router,packet)||!board_recovery_decode_request(packet->payload,packet->payload_len,&request))return board_recovery_refused();
 BoardRecoverySnapshot snapshot={.actual_state=255U};int32_t observed=board_recovery_snapshot(&snapshot);
 uint64_t generation=board_recovery_boot_current();uint8_t result=observed==0?BOARD_RECOVERY_RESULT_OK:BOARD_RECOVERY_RESULT_IO;
 if(request.operation==BOARD_RECOVERY_QUERY) {
  last_query_issuer=request.issuer_session;
  generation=board_bootstrap_storage_query_generation();
  if((!generation&&!board_bootstrap_storage_uncertain_first())||(snapshot.flags&BOARD_RECOVERY_SUPPORTED)==0U){snapshot.flags&=(uint8_t)~BOARD_RECOVERY_SUPPORTED;result=BOARD_RECOVERY_RESULT_UNSUPPORTED;}
  else if(generation==UINT64_MAX)result=BOARD_RECOVERY_RESULT_IO;
  else if(board_bootstrap_storage_pending()) {
   uint8_t required=BOARD_RECOVERY_ABORT_LATCHED|BOARD_RECOVERY_SAFE_OUTPUTS|BOARD_RECOVERY_OUTPUTS_VALID|BOARD_RECOVERY_SUPPORTED;
   result=board_bootstrap_storage_query_healthy()&&(snapshot.flags&required)==required?BOARD_RECOVERY_RESULT_OK:BOARD_RECOVERY_RESULT_IO;
  }
  /* RAM only. Normal completed recovery needs a fresh QUERY too; active
   * Launch cannot arm it, and an old queued query never gains a renewed UTC. */
  const uint64_t source=network_stamp_of(packet);
  if((result==BOARD_RECOVERY_RESULT_OK||board_bootstrap_storage_pending())&&
     board_recovery_request_fresh(router,source))
   (void)board_bootstrap_storage_note_query(&request,source,generation,snapshot.abort_epoch);
  return send_status(router,result,&request,generation,&snapshot,uptime_ms);
 }
 int32_t staged=board_bootstrap_storage_stage_first_leg(&request,network_stamp_of(packet),snapshot.abort_epoch,router,bootstrap_reply,bootstrap_fresh);
 if(staged<0)return SEDS_OK;
 result=(uint8_t)staged;
 (void)board_recovery_snapshot(&snapshot);

 return send_status(router,result,&request,generation,&snapshot,uptime_ms);
}
/* One unsolicited status for a fresh Abort latch, so the station sees it even
 * while its ordered queries wait: nonce 0, the issuer of the last valid QUERY,
 * the QUERY reply's generation, the live latch flags and abort epoch. */
SedsResult board_recovery_push_abort_status(SedsRouter*router,uint32_t uptime_ms) {
 BoardRecoverySnapshot snapshot={.actual_state=255U};(void)board_recovery_snapshot(&snapshot);
 snapshot.flags|=BOARD_RECOVERY_ABORT_LATCHED;
 BoardRecoveryRequest request={0};request.issuer_session=last_query_issuer;
 return send_status(router,BOARD_RECOVERY_RESULT_OK,&request,board_bootstrap_storage_query_generation(),&snapshot,uptime_ms);
}
SedsResult board_recovery_handle(SedsRouter*router,const SedsPacketView*packet,uint32_t local_endpoint,uint32_t uptime_ms) {
 /* Nonblocking serialization: no transport waits/SDK lock cycle, and manual
  * Stop has its independent endpoint/owner path. */
 if(atomic_flag_test_and_set_explicit(&dispatching,memory_order_acquire))return SEDS_IO;
 SedsResult result=handle_serialized(router,packet,local_endpoint,uptime_ms);
 atomic_flag_clear_explicit(&dispatching,memory_order_release);return result;
}
