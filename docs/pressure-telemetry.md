# Pressure telemetry cadence

`Core/Inc/telemetry_rates.h` sets `VALVE_PRESSURE_REPORT_HZ` to **30** by default.
With the board's 1000 Hz ThreadX clock, this uses a 33-tick period (about
30.3 reports/second). Override the macro in compiler definitions to tune it;
values outside 1–100 Hz are rejected because ADC3/TIM3 samples at 100 Hz.

Pressure reporting reads the latest ADC DMA sample and queues the existing
`FUEL_TANK_PRESSURE` telemetry type. Calibration and ADC configuration are
unchanged. Power telemetry remains 1 Hz. The acquisition loop accounts for its
processing time and yields after an overrun rather than bursting stale reports.
Blocking peripheral accesses or network congestion can still reduce the observed
arrival rate; this is a producer target, not an end-to-end delivery guarantee.

`./build.py test` includes compiled host-C tests for the default cadence,
overrides, invalid rates and overrun behavior. Build with `./build.py build
--release`. Hardware/network validation should check GroundStation's received
pressure timestamps under normal fill-system traffic.
