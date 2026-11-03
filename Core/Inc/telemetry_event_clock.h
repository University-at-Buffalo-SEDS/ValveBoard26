#pragma once
#include <stdint.h>
/* Actual SDK sample observed before a local event, not an extrapolated event
 * time. One-second local freshness; zero denotes unknown/expired/rollback. */
void telemetry_event_clock_observe(uint64_t network_utc, uint32_t local_tick);
uint64_t telemetry_event_clock_snapshot(void);
