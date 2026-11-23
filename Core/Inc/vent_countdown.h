#ifndef VENT_COUNTDOWN_H
#define VENT_COUNTDOWN_H
#include <stdint.h>

/* Post-Launch auto-vent. Pure decision logic: no RTOS, HAL or SDK calls. The
 * main control thread owns one instance and applies the vent through the
 * shared state owner.
 *
 * After an accepted, unaborted Launch the board no longer aborts on link loss
 * and holds Dump and Vent closed, even across a reset. With nobody in control
 * that leaves a pressurized tank for the recovery crew. The countdown runs
 * while the GroundStation link is lost OR the launch continuity wire is
 * broken, and on expiry drives the Abort safe state (Vent open, Dump open,
 * Pilot off) and latches it.
 *
 * Timing, all on the board's own monotonic clock:
 *  - Link lost: no GroundStation heartbeat for VENT_LINK_LOSS_DETECT_MS (the
 *    same 5 s as the pre-Launch heartbeat abort). The countdown is measured
 *    from the LAST heartbeat, so the vent comes 150 s after the last
 *    heartbeat (detection itself happens 5 s in).
 *  - Continuity broken: from the first sample of a break the safety thread
 *    has confirmed (4 consecutive samples, continuity_debounce.h).
 *  - Expiry = max(start + VENT_AFTER_LINK_LOSS_MS, Launch + VENT_BURN_GUARD_MS).
 *    The guard covers the 10 s Launch delay, the 1.5 s pilot window and the
 *    burn with a wide margin, so the vent can never fire during the burn even
 *    if both triggers are present at Launch.
 *  - Cancelled only when the heartbeat is present AND continuity is intact,
 *    or by an operator valve command. The deadline of a running countdown is
 *    fixed: a trigger clearing while the other persists keeps counting.
 *  - An operator valve command counts as both a heartbeat and a fresh
 *    continuity detection: if a trigger persists a new full countdown starts
 *    from the command.
 *
 * Reset after Launch: the launch marker survives in TAMP backup registers and
 * so does a small vent marker (Launch elapsed seconds, saturating at the
 * guard, and the vented latch). A rebooted board treats boot as its last
 * heartbeat, so with no link it vents 150 s after boot; its burn guard
 * continues from the persisted elapsed time (a lower bound of the real one).
 * A vented board stays vented across a reset. */
#define VENT_LINK_LOSS_DETECT_MS 5000U
#define VENT_AFTER_LINK_LOSS_MS 150000U
#define VENT_BURN_GUARD_MS 60000U
#define VENT_GUARD_S (VENT_BURN_GUARD_MS / 1000U)
/* Report cadence while counting or vented; changes report at once. */
#define VENT_REPORT_PERIOD_MS 5000U
#define VENT_REPORT_MAX 80U

enum { VENT_CD_IDLE = 0, VENT_CD_COUNTING = 1, VENT_CD_VENTED = 2 };
enum { VENT_CD_LINK = 1U, VENT_CD_CONTINUITY = 2U };

typedef struct {
    uint8_t state;
    uint8_t triggers;      /* Triggers active at the last step. */
    uint8_t launched;      /* Launch latch seen (and not aborted). */
    uint8_t reported;      /* state|triggers<<2 last reported, 0xFF none. */
    uint32_t elapsed_base_s; /* Launch elapsed seconds at base_ms. */
    uint64_t boot_ms;      /* Stands in for the first heartbeat. */
    uint64_t base_ms;
    uint64_t guard_ms;     /* No vent before this. */
    uint64_t operator_ms;  /* Last operator valve command, 0 none. */
    uint64_t deadline_ms;
    uint64_t report_ms;    /* Last successful report. */
    uint64_t attempt_ms;   /* Last attempt. */
} vent_countdown_t;

/* Boot: launched is the retained launch latch without Abort; elapsed_s and
 * vented come from the vent marker (0 when absent or invalid). */
void vent_countdown_boot(vent_countdown_t *v, uint64_t now_ms, uint8_t launched,
                         uint32_t elapsed_s, uint8_t vented);
/* heartbeat_ms: last GroundStation heartbeat, 0 none since boot.
 * continuity_since_ms: first sample of the current break, 0 intact.
 * Returns 1 when the vent is due; the caller applies it and then calls
 * vent_countdown_vented (a refused effect is retried on the next step). */
uint8_t vent_countdown_step(vent_countdown_t *v, uint64_t now_ms, uint8_t launched,
                             uint64_t heartbeat_ms, uint64_t continuity_since_ms);
/* The vent effect was applied: latch VENTED. */
void vent_countdown_vented(vent_countdown_t *v, uint64_t now_ms);
/* An applied operator valve command: the operator has control. */
void vent_countdown_operator(vent_countdown_t *v, uint64_t now_ms);
/* A report is owed: a state or trigger change, or the periodic re-announce
 * while counting or vented. report_attempt throttles retries; report_sent
 * records the published state. */
uint8_t vent_countdown_report_due(const vent_countdown_t *v, uint64_t now_ms);
void vent_countdown_report_attempt(vent_countdown_t *v, uint64_t now_ms);
void vent_countdown_report_sent(vent_countdown_t *v, uint64_t now_ms);
uint32_t vent_countdown_remaining_ms(const vent_countdown_t *v, uint64_t now_ms);
/* Launch elapsed seconds to persist, saturated at VENT_GUARD_S. */
uint32_t vent_countdown_elapsed_s(const vent_countdown_t *v, uint64_t now_ms);
/* "Valve Board auto-vent in 145 s: link lost [L145]"; codes L link, K
 * continuity, B both (value: remaining s), V vented (value: trigger mask),
 * X cancelled. */
uint32_t vent_countdown_format_report(const vent_countdown_t *v, uint64_t now_ms,
                                      char *out, uint32_t size);
#endif
