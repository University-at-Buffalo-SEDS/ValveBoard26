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
/* Advance the station clock from an authorized GS packet's timestamp. */
void board_recovery_observe_station_time(SedsRouter*router,const SedsPacketView*packet);
/* Freshness clock: max(valid network time, station clock); 0 when neither. */
uint64_t board_recovery_station_now(SedsRouter*router);
/* Network thread only: a valid network clock more than 60 s from the station
 * clock (which then restarts from the next GS packet). */
bool board_recovery_network_clock_resync_needed(uint64_t network_now);
#endif

/* Validated floor publication OUTSIDE owner locks. IO/mutex failure is not stale ACK. */
SedsResult board_recovery_sync_command_cutoff(void);
