# Pressure telemetry cadence

`Core/Inc/telemetry_rates.h` sets `VALVE_PRESSURE_REPORT_HZ` to **50** by default.
With the board's 1000 Hz ThreadX clock, this uses a 20-tick period.
Override the macro in compiler definitions to tune it; values outside 1–500 Hz
are rejected. ADC3/TIM3 also samples at 500 Hz: the 170 MHz timer uses prescaler
1699 and period 199 in both generated C and `Valve_Board26.ioc`.

Pressure reporting reads the latest ADC DMA sample and queues the existing
`FUEL_TANK_PRESSURE` telemetry type. Calibration is unchanged; the ADC trigger
rate remains 500 Hz. Power telemetry remains 1 Hz. Reporting every ADC sample
previously added 500 messages/s to the shared gateway and allowed battery updates
to disappear for 10–15 seconds while pressure continued arriving. The 50 Hz
report cadence removes 450 messages/s without changing sensor acquisition. The acquisition loop accounts for its
processing time and yields after an overrun rather than bursting stale reports.
Blocking peripheral accesses or network congestion can still reduce the observed
arrival rate; this is a producer target, not an end-to-end delivery guarantee.

`./build.py test` includes compiled host-C tests for the default cadence,
overrides, invalid rates and overrun behavior. Build with `./build.py build
--release`. Hardware/network validation should check GroundStation's received
pressure timestamps under normal fill-system traffic.
