#ifndef STATUS_REPORT_RETRY_H
#define STATUS_REPORT_RETRY_H
#include "telemetry.h"
#include "tx_api.h"

/* Main/output thread only. Keep the latest actual state, not a backlog of
 * obsolete open/close events. Protocol ACKs remain entirely in SEDSNet. */
/* Volatile diagnostic probes are observed by the firmware simulator. */
static volatile uint32_t status_report_pending;
static volatile uint32_t status_report_failures;
static volatile uint32_t status_report_recovered;
static uint8_t status_report_latest[16];
static ULONG status_report_last_retry;

static inline SedsResult publish_umbilical_status(uint8_t id, uint8_t state)
{
    if (id < 16U) {
        status_report_latest[id] = state;
        status_report_pending |= (uint16_t)(1U << id);
    }
    SedsResult result = telemetry_publish_umbilical_status(id, state);
    if (result == SEDS_OK && id < 16U)
        status_report_pending &= (uint16_t)~(1U << id);
    if (result != SEDS_OK) status_report_failures++;
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
            if (publish_umbilical_status(id, status_report_latest[id]) == SEDS_OK)
                status_report_recovered++;
        }
    }
}
#endif
