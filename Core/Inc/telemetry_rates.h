#ifndef VALVE_TELEMETRY_RATES_H
#define VALVE_TELEMETRY_RATES_H

/* Override per build; ADC3/TIM3 currently samples pressure at 100 Hz. */
#ifndef VALVE_PRESSURE_REPORT_HZ
#define VALVE_PRESSURE_REPORT_HZ 30U
#endif
#if VALVE_PRESSURE_REPORT_HZ < 1 || VALVE_PRESSURE_REPORT_HZ > 100
#error "VALVE_PRESSURE_REPORT_HZ must be between 1 and the 100 Hz ADC sample rate"
#endif
#if TX_TIMER_TICKS_PER_SECOND < VALVE_PRESSURE_REPORT_HZ
#error "ThreadX tick rate cannot support VALVE_PRESSURE_REPORT_HZ"
#endif

/* Nearest whole tick: 33 ticks at 1 kHz is approximately 30.3 Hz. */
#define VALVE_PRESSURE_REPORT_TICKS \
    ((TX_TIMER_TICKS_PER_SECOND + VALVE_PRESSURE_REPORT_HZ / 2U) / VALVE_PRESSURE_REPORT_HZ)

/* Account for acquisition work, but yield rather than burst after a stall. */
static inline unsigned long valve_pressure_sleep_ticks(unsigned long elapsed)
{
    return elapsed < VALVE_PRESSURE_REPORT_TICKS
        ? VALVE_PRESSURE_REPORT_TICKS - elapsed : 1UL;
}

#endif
