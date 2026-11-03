#ifndef STATUS_REPORT_RETRY_H
#define STATUS_REPORT_RETRY_H
#include <stdint.h>
typedef uint8_t (*StatusReportSnapshot)(uint8_t id, uint8_t *state);
#ifdef TELEMETRY_ENABLED
#include "telemetry.h"
#include "tx_api.h"

/* Main/output owner only. A pending bit requests a NEW current readback, not
 * replay of a cached open/close observation. SDK acceptance is not delivery. */
static volatile uint32_t status_report_pending;
static volatile uint32_t status_report_failures;
static volatile uint32_t status_report_deferred;
static volatile uint32_t status_report_recovered;
static ULONG status_report_last_retry;
static StatusReportSnapshot status_report_snapshot;
static inline void status_report_bind_snapshot(StatusReportSnapshot snapshot)
{ status_report_snapshot = snapshot; }

static inline SedsResult publish_umbilical_status(uint8_t id, uint8_t state)
{
    if (id < 16U) status_report_pending |= (uint16_t)(1U << id);
    (void)state; /* Cached action state is never an observation. */
    SedsResult result;
    const TelemetryStatusSubmit submitted = telemetry_submit_umbilical_snapshot(id, status_report_snapshot, &result);
    if (submitted == TELEMETRY_STATUS_ACCEPTED_LOCAL && id < 16U)
        status_report_pending &= (uint16_t)~(1U << id);
    else if (submitted == TELEMETRY_STATUS_DEFERRED) status_report_deferred++;
    else if (submitted == TELEMETRY_STATUS_REFUSED) status_report_failures++;
    return result;
}

static inline void retry_pending_status_reports(void)
{
    const ULONG now = tx_time_get();
    const ULONG period = (TX_TIMER_TICKS_PER_SECOND + 9U) / 10U;
    if ((ULONG)(now - status_report_last_retry) < period) return;
    status_report_last_retry = now;
    for (uint8_t id = 0U; id < 16U; ++id) {
        if ((status_report_pending & (uint16_t)(1U << id)) != 0U) {
            if (publish_umbilical_status(id, 0U) == SEDS_OK)
                status_report_recovered++;
        }
    }
}
#else
static inline void status_report_bind_snapshot(StatusReportSnapshot snapshot)
{ (void)snapshot; }
static inline void publish_umbilical_status(uint8_t id, uint8_t state)
{ (void)id; (void)state; }
static inline void retry_pending_status_reports(void) {}
#endif
#endif
