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
/* Station clock: the newest GS-issued timestamp, advanced by the board tick
 * since its receipt. It is a lower bound of GS time (transit only delays it),
 * so it never makes an old request look fresher than the GS clock would. It
 * only moves forward (a network clock resync restarts it from the next GS
 * packet) and is used beside, never instead of, a valid SEDSnet
 * network clock: an unsynced or wrongly anchored network clock cannot make a
 * fresh station request stale, and an absent one no longer refuses everything.
 * One writer (the network thread); readers use the sequence count. */
static volatile uint64_t station_utc;static volatile ULONG station_tick;static _Atomic uint32_t station_seq;
static uint64_t station_estimate(void){
 for(unsigned i=0U;i<4U;i++){
  uint32_t seq=atomic_load_explicit(&station_seq,memory_order_acquire);
  uint64_t utc=station_utc;ULONG tick=station_tick;
  atomic_thread_fence(memory_order_acquire);
  if(!(seq&1U)&&seq==atomic_load_explicit(&station_seq,memory_order_relaxed))
#if TX_TIMER_TICKS_PER_SECOND == 1000
   return utc?utc+(ULONG)(tx_time_get()-tick):0U;
#else
   return utc?utc+((uint64_t)(ULONG)(tx_time_get()-tick)*1000U)/TX_TIMER_TICKS_PER_SECOND:0U;
#endif
 }
 return 0U;
}
static void station_store(uint64_t utc){
 uint32_t seq=atomic_load_explicit(&station_seq,memory_order_relaxed);
 atomic_store_explicit(&station_seq,seq+1U,memory_order_relaxed);atomic_thread_fence(memory_order_release);
 station_utc=utc;station_tick=tx_time_get();
 atomic_store_explicit(&station_seq,seq+2U,memory_order_release);
}
void board_recovery_observe_station_time(SedsRouter*router,const SedsPacketView*packet){
 if(authorized_primary(router,packet)&&packet->timestamp!=UINT64_MAX&&packet->timestamp>station_estimate())station_store(packet->timestamp);
}
/* A relayed consumer cannot complete a verified SEDSnet time exchange, so a
 * first anchor from a source without real time (an RF board without a GPS fix
 * announces its uptime) is only slewed at 5% and never corrected. A valid
 * network clock more than 60 s from the station clock needs a timesync restart
 * so the next announce anchors it afresh. The station clock also restarts from
 * the next GS packet, so a GS clock step cannot make this repeat every poll. */
bool board_recovery_network_clock_resync_needed(uint64_t network_now){
 uint64_t station=station_estimate();
 if(!station||network_now-station+60000U<=120000U)return false;
 station_store(0U);return true;
}
uint64_t board_recovery_station_now(SedsRouter*router){
 uint64_t now=0U;if(!router)router=recovery_router;
 if(!router||seds_router_get_network_time_ms(router,&now)!=SEDS_OK)now=0U;
 uint64_t station=station_estimate();return station>now?station:now;
}
uint8_t board_recovery_command_is_stale(const SedsPacketView *packet)
{
 if (packet==0 || !board_recovery_command_time_allowed(packet->timestamp)) return 1U;
 if (board_recovery_floor_active()) {
  uint64_t now=recovery_router?board_recovery_station_now(recovery_router):0U;
  if (!bootstrap_request_fresh(packet->timestamp,now)) return 1U;
 }
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
static uint64_t bootstrap_now(void*context) {
 return board_recovery_station_now((SedsRouter*)context);
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
  /* RAM clock only. Normal completed recovery needs a fresh QUERY too; active
   * Launch cannot arm it, and an old queued query never gains a renewed UTC.
   * The station's own QUERY time may advance the station clock: the station
   * acts only on a reply to its nonce within its query deadline. */
  board_recovery_observe_station_time(router,packet);
  uint64_t now=bootstrap_now(router);
  if((result==BOARD_RECOVERY_RESULT_OK||board_bootstrap_storage_pending())&&
     bootstrap_request_fresh(packet->timestamp,now))
   (void)board_bootstrap_storage_note_query(&request,packet->timestamp,generation,snapshot.abort_epoch);
  return send_status(router,result,&request,generation,&snapshot,uptime_ms);
 }
 uint64_t now=bootstrap_now(router);
 int32_t staged=board_bootstrap_storage_stage_first_leg(&request,packet->timestamp,now,snapshot.abort_epoch,router,bootstrap_reply,bootstrap_now);
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
