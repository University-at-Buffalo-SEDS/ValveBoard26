#include "vent_countdown.h"

/* Throttle for a refused report; the main loop runs every tick. */
#define VENT_REPORT_RETRY_MS 100U

static uint8_t report_key(const vent_countdown_t *v)
{
    return (uint8_t)(v->state | (v->triggers << 2));
}

void vent_countdown_boot(vent_countdown_t *v, uint64_t now_ms, uint8_t launched,
                         uint32_t elapsed_s, uint8_t vented)
{
    *v = (vent_countdown_t){0};
    v->boot_ms = now_ms;
    if (launched) {
        if (elapsed_s > VENT_GUARD_S) elapsed_s = VENT_GUARD_S;
        v->launched = 1U;
        v->elapsed_base_s = elapsed_s;
        v->base_ms = now_ms;
        v->guard_ms = now_ms + (uint64_t)(VENT_GUARD_S - elapsed_s) * 1000U;
        if (vented) v->state = VENT_CD_VENTED;
    }
    /* A retained vent is announced at once; a quiet boot owes nothing. */
    v->reported = v->state == VENT_CD_VENTED ? 0xFFU : 0U;
}

static uint64_t later(uint64_t a, uint64_t b) { return a > b ? a : b; }

uint8_t vent_countdown_step(vent_countdown_t *v, uint64_t now_ms, uint8_t launched,
                            uint64_t heartbeat_ms, uint64_t continuity_since_ms)
{
    if (!launched) {
        /* Abort or recovery: the Abort safe state or the operator owns it. */
        v->launched = 0U;
        v->state = VENT_CD_IDLE;
        v->triggers = 0U;
        return 0U;
    }
    if (!v->launched) {
        /* Launch accepted in this boot. */
        v->launched = 1U;
        v->elapsed_base_s = 0U;
        v->base_ms = now_ms;
        v->guard_ms = now_ms + VENT_BURN_GUARD_MS;
    }
    if (v->state == VENT_CD_VENTED) return 0U;

    const uint64_t link_ref = later(heartbeat_ms ? heartbeat_ms : v->boot_ms, v->operator_ms);
    const uint8_t link = now_ms >= link_ref && now_ms - link_ref >= VENT_LINK_LOSS_DETECT_MS;
    const uint8_t continuity = continuity_since_ms != 0U;
    v->triggers = (uint8_t)((link ? VENT_CD_LINK : 0U) | (continuity ? VENT_CD_CONTINUITY : 0U));

    if (v->triggers == 0U) {
        v->state = VENT_CD_IDLE;
        return 0U;
    }
    if (v->state == VENT_CD_IDLE) {
        uint64_t start = UINT64_MAX;
        if (link) start = link_ref;
        if (continuity) {
            const uint64_t c = later(continuity_since_ms, v->operator_ms);
            if (c < start) start = c;
        }
        v->deadline_ms = later(start + VENT_AFTER_LINK_LOSS_MS, v->guard_ms);
        v->state = VENT_CD_COUNTING;
    }
    return now_ms >= v->deadline_ms;
}

void vent_countdown_vented(vent_countdown_t *v, uint64_t now_ms)
{
    (void)now_ms;
    if (v->launched) v->state = VENT_CD_VENTED;
}

void vent_countdown_operator(vent_countdown_t *v, uint64_t now_ms)
{
    if (!v->launched) return;
    v->state = VENT_CD_IDLE;
    v->triggers = 0U;
    v->operator_ms = now_ms ? now_ms : 1U;
}

uint32_t vent_countdown_remaining_ms(const vent_countdown_t *v, uint64_t now_ms)
{
    if (v->state != VENT_CD_COUNTING || now_ms >= v->deadline_ms) return 0U;
    const uint64_t left = v->deadline_ms - now_ms;
    return left > UINT32_MAX ? UINT32_MAX : (uint32_t)left;
}

uint32_t vent_countdown_elapsed_s(const vent_countdown_t *v, uint64_t now_ms)
{
    if (!v->launched) return 0U;
    const uint64_t s = v->elapsed_base_s + (now_ms >= v->base_ms ? (now_ms - v->base_ms) / 1000U : 0U);
    return s > VENT_GUARD_S ? VENT_GUARD_S : (uint32_t)s;
}

uint8_t vent_countdown_report_due(const vent_countdown_t *v, uint64_t now_ms)
{
    if (now_ms - v->attempt_ms < VENT_REPORT_RETRY_MS && v->attempt_ms != 0U) return 0U;
    if (report_key(v) != v->reported) return 1U;
    return v->state != VENT_CD_IDLE && now_ms - v->report_ms >= VENT_REPORT_PERIOD_MS;
}

void vent_countdown_report_attempt(vent_countdown_t *v, uint64_t now_ms)
{
    v->attempt_ms = now_ms ? now_ms : 1U;
}

void vent_countdown_report_sent(vent_countdown_t *v, uint64_t now_ms)
{
    v->reported = report_key(v);
    v->report_ms = now_ms;
    v->attempt_ms = 0U;
}

static uint32_t put_text(char *out, uint32_t at, uint32_t size, const char *text)
{
    while (*text && at + 1U < size) out[at++] = *text++;
    return at;
}

static uint32_t put_u32(char *out, uint32_t at, uint32_t size, uint32_t value)
{
    char digits[10];
    uint32_t n = 0U;
    do { digits[n++] = (char)('0' + value % 10U); value /= 10U; } while (value && n < sizeof digits);
    while (n && at + 1U < size) out[at++] = digits[--n];
    return at;
}

uint32_t vent_countdown_format_report(const vent_countdown_t *v, uint64_t now_ms,
                                      char *out, uint32_t size)
{
    if (!out || size == 0U) return 0U;
    static const char *const cause[4] = {"", "link lost", "continuity broken",
                                         "link lost, continuity broken"};
    static const char code[4] = {'X', 'L', 'K', 'B'};
    const uint8_t t = v->triggers & 3U;
    uint32_t at = put_text(out, 0U, size, "Valve Board auto-vent");
    char c = 'X';
    uint32_t value = 0U;
    if (v->state == VENT_CD_COUNTING) {
        value = (vent_countdown_remaining_ms(v, now_ms) + 999U) / 1000U;
        c = code[t];
        at = put_text(out, at, size, " in ");
        at = put_u32(out, at, size, value);
        at = put_text(out, at, size, " s: ");
        at = put_text(out, at, size, cause[t]);
    } else if (v->state == VENT_CD_VENTED) {
        c = 'V';
        value = t;
        at = put_text(out, at, size, "ed: Vent and Dump open, Pilot closed");
    } else {
        at = put_text(out, at, size, " cancelled");
    }
    at = put_text(out, at, size, " [");
    if (at + 1U < size) out[at++] = c;
    at = put_u32(out, at, size, value);
    at = put_text(out, at, size, "]");
    out[at] = '\0';
    return at;
}
