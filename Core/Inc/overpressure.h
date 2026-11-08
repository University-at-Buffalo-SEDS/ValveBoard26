#ifndef OVERPRESSURE_H
#define OVERPRESSURE_H
#include <stdint.h>

/* Board-local tank overpressure protection. Pure decision logic: no RTOS,
 * HAL or SDK calls. The main control thread owns one instance and applies its
 * actions through the shared state owner.
 *
 * The values mirror the GroundStation defaults (overpressure_relief_psi,
 * overpressure_abort_psi, overpressure_pilot_ms, DEBOUNCE_SAMPLES, FRESH_MS).
 * They are fixed at compile time; GroundStation configuration only changes
 * the GroundStation layer, which can still act earlier but never disables
 * this one. */
#define OVERPRESSURE_RELIEF_PSI 850.0f
#define OVERPRESSURE_ABORT_PSI 950.0f
#define OVERPRESSURE_PILOT_MS 3000U
#define OVERPRESSURE_DEBOUNCE_SAMPLES 2U
#define OVERPRESSURE_FRESH_MS 2000U
/* 12-bit ADC rail. A sample at or above the calibrated full-scale code is a
 * clamped conversion, never a measurement. */
#define OVERPRESSURE_ADC_MAX_CODE 4095U

typedef struct {
    uint32_t seq;      /* Increments per acquisition; 0 means none yet. */
    uint32_t at_ms;    /* Monotonic acquisition time (wrapping). */
    uint32_t raw_adc;
    float psi;         /* Same calibrated value published as FUEL_TANK_PRESSURE. */
    uint8_t valid;
} overpressure_sample_t;

enum {
    OVERPRESSURE_IDLE = 0,
    OVERPRESSURE_RELIEF = 1,  /* Dump and vent open, supplies commanded closed. */
    OVERPRESSURE_PILOT = 2,   /* Also main valve open for OVERPRESSURE_PILOT_MS. */
    OVERPRESSURE_ABORTED = 3, /* Main valve closed and Abort requested. */
    OVERPRESSURE_ALARM = 4    /* Tripped after Launch admission: report only. */
};

enum {
    OVERPRESSURE_ACT_RELIEF = 1U << 0,
    OVERPRESSURE_ACT_PILOT_OPEN = 1U << 1,
    OVERPRESSURE_ACT_PILOT_CLOSE = 1U << 2,
    OVERPRESSURE_ACT_ABORT = 1U << 3,
    OVERPRESSURE_ACT_REPORT = 1U << 4,
    OVERPRESSURE_ACT_FAULT = 1U << 5
};

enum {
    OVERPRESSURE_FAULT_NONE = 0,
    OVERPRESSURE_FAULT_INVALID = 1,
    OVERPRESSURE_FAULT_STALE = 2
};

enum {
    OVERPRESSURE_ACK_CLEARED = 0,
    OVERPRESSURE_ACK_NOT_LATCHED = 1,
    OVERPRESSURE_ACK_PILOT_RUNNING = 2,
    OVERPRESSURE_ACK_NO_FRESH_SAMPLE = 3,
    OVERPRESSURE_ACK_STILL_HIGH = 4
};

typedef struct {
    uint32_t last_seq;
    uint32_t started_ms;
    uint32_t last_valid_ms;
    uint32_t pilot_open_ms;
    uint32_t fault_raw;
    float last_valid_psi;
    float peak_psi;
    uint16_t trips;
    uint8_t relief_streak;
    uint8_t abort_streak;
    uint8_t stage;
    uint8_t started;
    uint8_t have_valid;
    uint8_t fault;
    uint8_t alarm_abort;
} overpressure_t;

/* A reading is usable only when the calibration is sane and the conversion
 * did not clamp: raw strictly below the calibrated full-scale code and the
 * ADC rail, and a finite psi below full-scale pressure. */
uint8_t overpressure_sample_valid(uint32_t raw_adc, float psi, float min_adc,
                                  float max_adc, float max_psi);
/* Feed the latest acquisition (repeats are ignored) and the current time.
 * burn: Launch admitted/running or flight state Launch or later. */
uint32_t overpressure_step(overpressure_t *op, const overpressure_sample_t *sample,
                           uint32_t now_ms, uint8_t burn);
/* The state owner refused the relief/pilot effect for Launch: report only. */
void overpressure_demote(overpressure_t *op);
/* The board Abort already closed the main valve and opened dump and vent. */
uint8_t overpressure_abort_observed(overpressure_t *op);
uint8_t overpressure_acknowledge(overpressure_t *op, uint32_t now_ms);
uint8_t overpressure_owns_valves(const overpressure_t *op);
/* Operator/automatic commands refused while relief owns the valves. */
uint8_t overpressure_refuses_command(const overpressure_t *op, uint32_t command);
char overpressure_report_code(const overpressure_t *op);
/* Operator-readable WARNING text with a machine tag, for example
 * "Valve Board overpressure relief: 912 psi [R3]". The bracket holds the
 * stage code (R relief, P main valve open, A Abort, L launch alarm,
 * C acknowledged, F invalid sensor, S stale sensor) and the trip number.
 * GroundStation parses only the tag and the value. Returns the length. */
uint32_t overpressure_format_report(char *out, uint32_t size, char code,
                                    uint32_t value, uint32_t trip);
#define OVERPRESSURE_REPORT_MAX 96U

#endif
