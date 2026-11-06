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
uint8_t board_recovery_command_is_stale(const SedsPacketView *packet)
{
 if (packet==0 || !board_recovery_command_time_allowed(packet->timestamp)) return 1U;
 if (board_recovery_floor_active()) {
  uint64_t now=0U;
  if (recovery_router==0 || seds_router_get_network_time_ms(recovery_router,&now)!=SEDS_OK ||
      packet->timestamp==0U || (now>=packet->timestamp ? now-packet->timestamp>5000U : packet->timestamp-now>1000U)) return 1U;
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

static SedsResult send_status(SedsRouter*router,uint8_t result,const BoardRecoveryRequest*request,
                            uint64_t generation,const BoardRecoverySnapshot*snapshot,uint32_t uptime_ms) {
 uint8_t status[BOARD_RECOVERY_STATUS_SIZE];
 board_recovery_encode_status(status,result,snapshot->actual_state,(uint8_t)(snapshot->flags|BOARD_RECOVERY_VERIFIED_IMPLEMENTATION|BOARD_RECOVERY_MANUAL_UTC_IMPLEMENTATION),request,
  generation,snapshot->abort_epoch,uptime_ms,snapshot->actual_outputs_mask);
 return seds_router_log_queue_typed(router,SEDS_DT_BOARD_STATE_STATUS,status,sizeof status,sizeof(uint8_t),SEDS_EK_UNSIGNED);
}
static uint64_t bootstrap_now(void*context) {
 uint64_t now=0U;if(seds_router_get_network_time_ms((SedsRouter*)context,&now)!=SEDS_OK)return 0U;return now;
}
static bool bootstrap_reply(void*context,const BoardRecoveryRequest*request,uint8_t result) {
 BoardRecoverySnapshot snapshot={.actual_state=255U};(void)board_recovery_snapshot(&snapshot);
 return send_status((SedsRouter*)context,result,request,board_recovery_boot_current(),&snapshot,
  (uint32_t)(((uint64_t)tx_time_get()*1000U)/TX_TIMER_TICKS_PER_SECOND))==SEDS_OK;
}
static SedsResult handle_serialized(SedsRouter*router,const SedsPacketView*packet,uint32_t local_endpoint,uint32_t uptime_ms) {
 if(!router||!packet||packet->ty!=SEDS_DT_BOARD_RECOVERY_REQUEST||packet->num_endpoints!=1U||!packet->endpoints||packet->endpoints[0]!=local_endpoint)return SEDS_BAD_ARG;
 if(!authorized_primary(router,packet))return SEDS_PERMISSION_DENIED;
 BoardRecoveryRequest request;
 if(!board_recovery_decode_request(packet->payload,packet->payload_len,&request))return SEDS_BAD_ARG;
 BoardRecoverySnapshot snapshot={.actual_state=255U};int32_t observed=board_recovery_snapshot(&snapshot);
 uint64_t generation=board_recovery_boot_current();uint8_t result=observed==0?BOARD_RECOVERY_RESULT_OK:BOARD_RECOVERY_RESULT_IO;
 if(request.operation==BOARD_RECOVERY_QUERY) {
  generation=board_bootstrap_storage_query_generation();
  if((!generation&&!board_bootstrap_storage_uncertain_first())||(snapshot.flags&BOARD_RECOVERY_SUPPORTED)==0U){snapshot.flags&=(uint8_t)~BOARD_RECOVERY_SUPPORTED;result=BOARD_RECOVERY_RESULT_UNSUPPORTED;}
  else if(generation==UINT64_MAX)result=BOARD_RECOVERY_RESULT_IO;
  else if(board_bootstrap_storage_pending()) {
   uint8_t required=BOARD_RECOVERY_ABORT_LATCHED|BOARD_RECOVERY_SAFE_OUTPUTS|BOARD_RECOVERY_OUTPUTS_VALID|BOARD_RECOVERY_SUPPORTED;
   result=board_bootstrap_storage_query_healthy()&&(snapshot.flags&required)==required?BOARD_RECOVERY_RESULT_OK:BOARD_RECOVERY_RESULT_IO;
  }
  /* RAM clock only. Normal completed recovery needs a fresh QUERY too; active
   * Launch cannot arm it, and an old queued query never gains a renewed UTC. */
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
SedsResult board_recovery_handle(SedsRouter*router,const SedsPacketView*packet,uint32_t local_endpoint,uint32_t uptime_ms) {
 /* Nonblocking serialization: no transport waits/SDK lock cycle, and manual
  * Stop has its independent endpoint/owner path. */
 if(atomic_flag_test_and_set_explicit(&dispatching,memory_order_acquire))return SEDS_IO;
 SedsResult result=handle_serialized(router,packet,local_endpoint,uptime_ms);
 atomic_flag_clear_explicit(&dispatching,memory_order_release);return result;
}
