#ifndef SAFETY_DIAGNOSTICS_H
#define SAFETY_DIAGNOSTICS_H
#include <stdint.h>

enum {
    SAFETY_ABORT_NONE = 0,
    SAFETY_ABORT_HEARTBEAT = 1,
    SAFETY_ABORT_INITIAL_HEARTBEAT = 2,
    SAFETY_ABORT_CONTINUITY = 3,
    SAFETY_ABORT_LAUNCH_CONTINUITY = 4,
    SAFETY_ABORT_ADC_START = 5
};

/* Local monotonic milliseconds, not network time. First local safety trigger
 * per boot only. An already-latched external abort must not be misattributed. */
typedef struct {
    uint64_t at_ms;
    uint64_t last_heartbeat_ms;
    uint32_t flight_state;
    uint32_t detail;
    uint32_t reason; /* Written last; zero means no recorded local trigger. */
} safety_abort_diagnostic_t;

extern volatile safety_abort_diagnostic_t g_safety_first_abort;

static inline void safety_record_first_abort(uint32_t reason, uint64_t now_ms,
                                             uint64_t heartbeat_ms,
                                             uint32_t flight_state,
                                             uint32_t detail,
                                             uint32_t already_aborted)
{
    /* Called only by the safety thread. Snapshot remains immutable afterward. */
    if (already_aborted || g_safety_first_abort.reason != SAFETY_ABORT_NONE)
        return;
    g_safety_first_abort.at_ms = now_ms;
    g_safety_first_abort.last_heartbeat_ms = heartbeat_ms;
    g_safety_first_abort.flight_state = flight_state;
    g_safety_first_abort.detail = detail;
    g_safety_first_abort.reason = reason;
}
#endif

