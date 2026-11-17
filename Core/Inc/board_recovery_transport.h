#ifndef BOARD_RECOVERY_TRANSPORT_H
#define BOARD_RECOVERY_TRANSPORT_H
#include "sedsnet.h"
#include "sedsnet_config.h"
void board_recovery_transport_set_router(SedsRouter *router);
/* Caller holds the router: one nonce-0 Abort-latched BOARD_STATE_STATUS. */
SedsResult board_recovery_push_abort_status(SedsRouter *router, uint32_t uptime_ms);
uint8_t board_recovery_manual_stop_source_allowed(SedsRouter*router,const SedsPacketView*packet);
SedsResult board_recovery_handle(SedsRouter *router,const SedsPacketView *packet,
                                uint32_t local_endpoint,uint32_t uptime_ms);
/* Application replay floor; SDK stays generic. No current-time/RNG claim. */
uint8_t board_recovery_command_is_stale(const SedsPacketView *packet);
uint8_t board_recovery_managed_update_allowed(const SedsPacketView *packet);
uint8_t board_recovery_automatic_is_stale(const SedsPacketView *packet);
/* Acknowledge a delivered reliable packet that is deliberately not applied:
 * counts it in g_reliable_refusal_count and returns SEDS_OK. */
extern volatile uint32_t g_reliable_refusal_count;
SedsResult board_recovery_refused(void);
/* Request-validity window of a GS-stamped request: at most this old and at
 * most this far ahead of this node's network time. */
#define BOARD_RECOVERY_FRESH_AGE_MS UINT64_C(15000)
#define BOARD_RECOVERY_FRESH_AHEAD_MS UINT64_C(1000)
/* A network stamp key (network_stamp.h) inside that window, judged with
 * seds_router_network_stamp_fresh across network time steps. */
bool board_recovery_request_fresh(SedsRouter*router,uint64_t source);
/* Once the router exists at boot: raise its time epoch to the epoch of the
 * persisted replay floor. */
SedsResult board_recovery_raise_time_epoch(SedsRouter*router);
#endif

/* Validated floor publication OUTSIDE owner locks. IO/mutex failure is not stale ACK. */
SedsResult board_recovery_sync_command_cutoff(void);
