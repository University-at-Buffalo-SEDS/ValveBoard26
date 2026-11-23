#ifndef CONTINUITY_DEBOUNCE_H
#define CONTINUITY_DEBOUNCE_H
#include <stdint.h>

/* Launch continuity wire debounce. Pure logic: no RTOS, HAL or SDK calls.
 *
 * The safety thread samples PB15 every 10 ms. A single sample that is not
 * the "present" level used to abort the board in Fill/Armed and, after
 * Launch, start the 150 s auto-vent countdown. A connector bounce or an EMI
 * spike while the servos move is enough to do that. The wire's reported
 * state now changes only after CONTINUITY_DEBOUNCE_SAMPLES consecutive
 * samples of the other level: 4 samples at 10 ms span 30 ms, so a real break
 * is confirmed 30-40 ms after it happens, while any shorter glitch in either
 * direction is ignored.
 *
 * The pin has a pull-down (main.c), so an open or cut wire reads low, which
 * is the "broken" level (fail-safe). broken_since_ms is the time of the first
 * sample of the confirmed break, so the post-Launch countdown still starts
 * at the real break, not at the confirmation. */
#define CONTINUITY_DEBOUNCE_SAMPLES 4U

typedef struct {
    uint8_t broken;          /* confirmed state: 1 broken, 0 intact */
    uint8_t run;             /* consecutive samples disagreeing with it */
    uint64_t run_started_ms; /* first sample of that run */
    uint64_t broken_since_ms;/* first sample of the confirmed break, 0 intact */
} continuity_debounce_t;

/* Boot state: intact. A wire already open at boot is confirmed after the
 * debounce window, like any other break. */
static inline void continuity_debounce_init(continuity_debounce_t *d)
{
    d->broken = 0U;
    d->run = 0U;
    d->run_started_ms = 0ULL;
    d->broken_since_ms = 0ULL;
}

/* One sample. present: the pin reads the continuity-present level.
 * Returns the confirmed state: 1 broken, 0 intact. */
static inline uint8_t continuity_debounce_step(continuity_debounce_t *d, uint8_t present,
                                               uint64_t now_ms)
{
    const uint8_t sample_broken = present ? 0U : 1U;
    if (sample_broken == d->broken) {
        d->run = 0U;
        return d->broken;
    }
    if (d->run == 0U) {
        d->run_started_ms = now_ms;
    }
    if (++d->run < CONTINUITY_DEBOUNCE_SAMPLES) {
        return d->broken;
    }
    d->broken = sample_broken;
    d->run = 0U;
    d->broken_since_ms = sample_broken ? (d->run_started_ms ? d->run_started_ms : 1ULL) : 0ULL;
    return d->broken;
}

#endif
