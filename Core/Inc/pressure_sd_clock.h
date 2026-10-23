#ifndef PRESSURE_SD_CLOCK_H
#define PRESSURE_SD_CLOCK_H
#include <stdint.h>
#include "board_uptime.h"
#include "pressure_sd_wire.h"
/* The ADC sample clock is advanced by acquired sample sequence. HAL capture
 * time is retained separately for UTC projection: IRQ tick drift must neither
 * discard real samples nor project sample-grid ticks through the HAL clock. */
typedef struct { uint64_t ticks; uint32_t sequence; uint8_t initialized; } pressure_sd_clock_t;
static inline uint64_t pressure_sd_clock_next(pressure_sd_clock_t *clock,
    uint32_t sequence, uint64_t observed_uptime_ms)
{
    if(!clock->initialized){clock->ticks=observed_uptime_ms;clock->initialized=1U;}
    else clock->ticks+=(uint64_t)(uint32_t)(sequence-clock->sequence)*PRESSURE_SD_PERIOD_MS;
    clock->sequence=sequence;
    return clock->ticks;
}
static inline void pressure_sd_stamp_clock(pressure_sd_clock_t *clock,
    pressure_sd_batch_t *batch, uint32_t sequence, uint32_t captured_hal_ms,
    uint64_t now_uptime_ms, uint64_t now_unix_ms)
{
    const uint64_t observed=board_uptime_project(now_uptime_ms,captured_hal_ms);
    batch->uptime_ms=pressure_sd_clock_next(clock,sequence,observed);
    batch->first_sequence=sequence;
    const uint64_t age=now_uptime_ms-observed;
    batch->unix_ms=now_unix_ms>age?now_unix_ms-age:0U;
}
#endif
