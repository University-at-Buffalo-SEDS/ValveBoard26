#ifndef PRESSURE_SD_ADMISSION_H
#define PRESSURE_SD_ADMISSION_H
#include "sedsnet.h"
#include <stdint.h>

/* Archive traffic cannot consume the reliable credits needed by live output
 * status. This application policy never changes or evicts accepted parents. */
#ifndef PRESSURE_SD_CONTROL_RESERVE_PERCENT
#define PRESSURE_SD_CONTROL_RESERVE_PERCENT 25U
#endif
#ifndef PRESSURE_SD_CONTROL_RESERVE_MIN
#define PRESSURE_SD_CONTROL_RESERVE_MIN 4U
#endif
#if PRESSURE_SD_CONTROL_RESERVE_MIN < 1
#error "Pressure archive must reserve a live status credit"
#endif
#if PRESSURE_SD_CONTROL_RESERVE_PERCENT < 1 || PRESSURE_SD_CONTROL_RESERVE_PERCENT > 99
#error "Pressure archive control reservation must be from 1 to 99 percent"
#endif
static inline SedsResult pressure_sd_archive_admission(SedsRouter *router)
{
    if (router == NULL) return SEDS_IO;
    SedsRuntimeTuningConfig config;
    SedsResult result = seds_get_runtime_tuning_config(&config);
    if (result != SEDS_OK) return result;
    size_t limit = config.reliable_max_end_to_end_pending;
    if (limit == 0U) limit = 1U; /* SDK normalizes this limit in the same way. */
    /* Divide first to avoid overflow at any size_t width. Round the reserved
     * share upward; even a small window retains at least one control credit. */
    size_t reserved = (limit / 100U) * PRESSURE_SD_CONTROL_RESERVE_PERCENT +
        ((limit % 100U) * PRESSURE_SD_CONTROL_RESERVE_PERCENT + 99U) / 100U;
    /* Periodic output synchronization submits four states as one burst. */
    if (reserved < PRESSURE_SD_CONTROL_RESERVE_MIN)
        reserved = PRESSURE_SD_CONTROL_RESERVE_MIN;
    if (reserved > limit) reserved = limit;
    uint32_t counters[16];
    /* Published SDK4.3.2 returns 16 for every valid aligned snapshot. Its
     * blocking state lock has no transient refusal path; a negative result
     * means invalid arguments, so expose failure rather than retrying it. */
    if (seds_router_copy_queue_counters(router, counters, 16U) != 16)
        return SEDS_ERR;
    /* Include already accepted queued originals, not just dispatched parents,
     * so a stalled dispatch worker cannot bypass the reservation. Counting
     * other queued work conservatively preserves the original FIFO owners. */
    const uint64_t committed = (uint64_t)counters[2] + counters[11];
    return committed < limit - reserved ? SEDS_OK : SEDS_IO;
}
#endif
