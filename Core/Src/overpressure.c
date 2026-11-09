#include "overpressure.h"
#include "board_commands.h"

_Static_assert(OVERPRESSURE_RELIEF_PSI < OVERPRESSURE_ABORT_PSI, "relief must precede abort");
_Static_assert(OVERPRESSURE_PILOT_MS > 0U && OVERPRESSURE_PILOT_MS < 60000U, "bounded main-valve window");
_Static_assert(OVERPRESSURE_SUSTAIN_MS >= 100U && OVERPRESSURE_SUSTAIN_MS < OVERPRESSURE_FRESH_MS,
               "a brief glitch never trips");
_Static_assert(OVERPRESSURE_STEP_MS > 0U && OVERPRESSURE_MAX_STEP_PSI > 0.0f, "bounded plausible rate");

uint8_t overpressure_sample_valid(uint32_t raw_adc, float psi, float min_adc,
                                  float max_adc, float max_psi)
{
    /* Calibration that cannot represent the abort pressure, or whose span is
     * inverted/off the ADC range, makes every reading unusable. */
    if (!(max_adc > min_adc) || !(min_adc >= 0.0f) ||
        !(max_adc <= (float)OVERPRESSURE_ADC_MAX_CODE) ||
        !(max_psi > OVERPRESSURE_ABORT_PSI)) return 0U;
    /* NaN fails every comparison below. Full scale is a fault, not psi. */
    return (raw_adc < OVERPRESSURE_ADC_MAX_CODE && (float)raw_adc < max_adc &&
            psi >= 0.0f && psi < max_psi) ? 1U : 0U;
}

/* True once the reading has stayed over the threshold for SUSTAIN_MS. */
static uint8_t sustained(uint8_t *over, uint32_t *since, uint8_t now_over, uint32_t at_ms,
                         uint8_t restart)
{
    if (!now_over) return *over = 0U;
    if (!*over || restart) {
        *over = 1U;
        *since = at_ms;
    }
    return (uint32_t)(at_ms - *since) >= OVERPRESSURE_SUSTAIN_MS;
}

static uint32_t enter_pilot(overpressure_t *op, uint32_t now_ms)
{
    op->stage = OVERPRESSURE_PILOT;
    op->pilot_open_ms = now_ms;
    return OVERPRESSURE_ACT_PILOT_OPEN;
}

uint32_t overpressure_step(overpressure_t *op, const overpressure_sample_t *sample,
                           uint32_t now_ms, uint8_t burn)
{
    uint32_t act = 0U;
    uint8_t level = 0U;
    float psi = 0.0f;

    if (!op->started) {
        op->started = 1U;
        op->started_ms = now_ms;
    }
    if (sample && sample->seq != 0U && sample->seq != op->last_seq) {
        op->last_seq = sample->seq;
        /* A future or older-than-fresh timestamp neither counts nor clears. */
        if ((uint32_t)(now_ms - sample->at_ms) <= OVERPRESSURE_FRESH_MS) {
            if (sample->settling) {
                /* Not a reading: it never counts toward or continues a trip.
                 * Before the first reading it proves acquisition is alive, so
                 * the stale clock starts after the settle window, not before. */
                op->relief_streak = op->abort_streak = 0U;
                if (!op->have_valid) op->started_ms = sample->at_ms;
            } else if (!sample->valid) {
                op->relief_streak = op->abort_streak = 0U;
                if (op->fault != OVERPRESSURE_FAULT_INVALID) {
                    op->fault = OVERPRESSURE_FAULT_INVALID;
                    op->fault_raw = sample->raw_adc;
                    act |= OVERPRESSURE_ACT_FAULT;
                }
            } else {
                psi = sample->psi;
                const uint32_t gap = sample->at_ms - op->last_valid_ms;
                const float step = psi - op->last_valid_psi;
                const float limit = (float)(gap < OVERPRESSURE_STEP_MS ? OVERPRESSURE_STEP_MS : gap) *
                                    (OVERPRESSURE_MAX_STEP_PSI / (float)OVERPRESSURE_STEP_MS);
                const uint8_t restart = !op->have_valid || gap > OVERPRESSURE_SUSTAIN_MS;
                if (op->have_valid && (step > limit || -step > limit)) {
                    op->settling = 1U;
                    op->settle_from_ms = sample->at_ms;
                    if (op->fault != OVERPRESSURE_FAULT_IMPLAUSIBLE) {
                        op->fault = OVERPRESSURE_FAULT_IMPLAUSIBLE;
                        op->fault_raw = (uint32_t)(psi + 0.5f);
                        act |= OVERPRESSURE_ACT_FAULT;
                    }
                } else if (op->settling &&
                           (uint32_t)(sample->at_ms - op->settle_from_ms) >= OVERPRESSURE_SETTLE_MS) {
                    op->settling = 0U;
                }
                op->have_valid = 1U;
                op->last_valid_ms = sample->at_ms;
                op->last_valid_psi = psi;
                if (op->settling) {
                    op->relief_streak = op->abort_streak = 0U;
                } else {
                    op->fault = OVERPRESSURE_FAULT_NONE;
                    const uint8_t at_relief = sustained(&op->relief_streak, &op->relief_since_ms,
                                                     psi >= OVERPRESSURE_RELIEF_PSI, sample->at_ms, restart);
                    const uint8_t at_abort = sustained(&op->abort_streak, &op->abort_since_ms,
                                                    psi >= OVERPRESSURE_ABORT_PSI, sample->at_ms, restart);
                    if (op->stage != OVERPRESSURE_IDLE && psi > op->peak_psi) op->peak_psi = psi;
                    if (at_abort) level = 2U;
                    else if (at_relief) level = 1U;
                }
            }
        }
    }
    if ((op->fault == OVERPRESSURE_FAULT_NONE || op->fault == OVERPRESSURE_FAULT_IMPLAUSIBLE) &&
        (uint32_t)(now_ms - (op->have_valid ? op->last_valid_ms : op->started_ms)) >
            OVERPRESSURE_FRESH_MS) {
        op->fault = OVERPRESSURE_FAULT_STALE;
        op->fault_raw = 0U;
        act |= OVERPRESSURE_ACT_FAULT;
    }

    if (level != 0U) {
        if (op->stage == OVERPRESSURE_IDLE) {
            op->trips++;
            op->peak_psi = psi;
            act |= OVERPRESSURE_ACT_REPORT;
            if (burn) {
                op->stage = OVERPRESSURE_ALARM;
                op->alarm_abort = level == 2U;
            } else {
                op->stage = OVERPRESSURE_RELIEF;
                act |= OVERPRESSURE_ACT_RELIEF;
                if (level == 2U) act |= enter_pilot(op, now_ms);
            }
        } else if (level == 2U && op->stage == OVERPRESSURE_RELIEF && !burn) {
            act |= OVERPRESSURE_ACT_RELIEF | OVERPRESSURE_ACT_REPORT | enter_pilot(op, now_ms);
        } else if (level == 2U && op->stage == OVERPRESSURE_ALARM && !op->alarm_abort) {
            op->alarm_abort = 1U;
            act |= OVERPRESSURE_ACT_REPORT;
        }
    }
    if (op->stage == OVERPRESSURE_PILOT &&
        (uint32_t)(now_ms - op->pilot_open_ms) >= OVERPRESSURE_PILOT_MS) {
        /* Main valve closed first, then Abort, in that order. */
        op->stage = OVERPRESSURE_ABORTED;
        act |= OVERPRESSURE_ACT_PILOT_CLOSE | OVERPRESSURE_ACT_ABORT | OVERPRESSURE_ACT_REPORT;
    }
    return act;
}

void overpressure_demote(overpressure_t *op)
{
    if (op->stage == OVERPRESSURE_RELIEF || op->stage == OVERPRESSURE_PILOT) {
        op->alarm_abort = op->stage == OVERPRESSURE_PILOT;
        op->stage = OVERPRESSURE_ALARM;
    }
}

uint8_t overpressure_abort_observed(overpressure_t *op)
{
    if (op->stage != OVERPRESSURE_PILOT) return 0U;
    op->stage = OVERPRESSURE_ABORTED;
    return 1U;
}

uint8_t overpressure_acknowledge(overpressure_t *op, uint32_t now_ms)
{
    if (op->stage == OVERPRESSURE_IDLE) return OVERPRESSURE_ACK_NOT_LATCHED;
    if (op->stage == OVERPRESSURE_PILOT) return OVERPRESSURE_ACK_PILOT_RUNNING;
    if (!op->have_valid || op->fault != OVERPRESSURE_FAULT_NONE ||
        (uint32_t)(now_ms - op->last_valid_ms) > OVERPRESSURE_FRESH_MS)
        return OVERPRESSURE_ACK_NO_FRESH_SAMPLE;
    if (!(op->last_valid_psi < OVERPRESSURE_RELIEF_PSI)) return OVERPRESSURE_ACK_STILL_HIGH;
    /* Valves stay as they are: nothing is re-closed automatically. */
    op->stage = OVERPRESSURE_IDLE;
    op->relief_streak = op->abort_streak = 0U;
    op->alarm_abort = 0U;
    return OVERPRESSURE_ACK_CLEARED;
}

uint8_t overpressure_owns_valves(const overpressure_t *op)
{
    return op->stage == OVERPRESSURE_RELIEF || op->stage == OVERPRESSURE_PILOT ||
           op->stage == OVERPRESSURE_ABORTED;
}

uint8_t overpressure_refuses_command(const overpressure_t *op, uint32_t command)
{
    if (!overpressure_owns_valves(op)) return 0U;
    switch (command & 0x7FU) {
    case CMD_DUMP_CLOSE:
    case CMD_NORMALLY_OPEN_CLOSE:
    case CMD_PILOT_OPEN:
    case CMD_SEQUENCE:
        return 1U;
    case CMD_PILOT_CLOSE:
        return op->stage == OVERPRESSURE_PILOT;
    default:
        return 0U;
    }
}

char overpressure_report_code(const overpressure_t *op)
{
    static const char codes[] = "CRPAL";
    return op->stage <= OVERPRESSURE_ALARM ? codes[op->stage] : '?';
}

static uint32_t put_u32(char *out, uint32_t at, uint32_t size, uint32_t value)
{
    char digits[10];
    uint32_t n = 0U;
    do { digits[n++] = (char)('0' + value % 10U); value /= 10U; } while (value != 0U);
    while (n != 0U && at + 1U < size) out[at++] = digits[--n];
    return at;
}

static uint32_t put_text(char *out, uint32_t at, uint32_t size, const char *text)
{
    while (*text != '\0' && at + 1U < size) out[at++] = *text++;
    return at;
}

uint32_t overpressure_format_report(char *out, uint32_t size, char code,
                                    uint32_t value, uint32_t trip)
{
    const char *word;
    switch (code) {
    case 'R': word = "relief: "; break;
    case 'P': word = "relief, main valve open: "; break;
    case 'A': word = "relief, main valve closed, Abort: "; break;
    case 'L': word = "during launch, no valve action: "; break;
    case 'C': word = "acknowledged: "; break;
    case 'F': word = "sensor invalid, raw ADC "; break;
    case 'S': word = "sensor stale"; break;
    case 'I': word = "sensor implausible or disconnected: "; break;
    default: word = "unknown: "; break;
    }
    if (!out || size == 0U) return 0U;
    uint32_t at = put_text(out, 0U, size, "Valve Board overpressure ");
    at = put_text(out, at, size, word);
    if (code != 'S') at = put_u32(out, at, size, value);
    if (code != 'S' && code != 'F') at = put_text(out, at, size, " psi");
    at = put_text(out, at, size, " [");
    if (at + 1U < size) out[at++] = code;
    at = put_u32(out, at, size, trip);
    at = put_text(out, at, size, "]");
    out[at] = '\0';
    return at;
}
