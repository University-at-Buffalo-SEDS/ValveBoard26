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

Pressure SD archive packets use a separate admission policy from live pressure
telemetry. Before enqueueing a new archive packet, the board reserves reliable
credits for live output-state reports. The reservation is the larger of four
credits (the periodic four-state synchronization burst) and 25% of the actual
configured end-to-end reliable window, capped at that window. The policy also
counts queued originals so a stalled dispatch worker cannot overcommit credits.
`PRESSURE_SD_CONTROL_RESERVE_MIN` and `PRESSURE_SD_CONTROL_RESERVE_PERCENT`
configure this application policy; windows of four or fewer leave no archive
capacity and require a larger window to share archive and status traffic.

This prevents an unavailable DAQ archive destination from monopolizing the live
status window. It changes neither capture cadence nor healthy packet cadence,
and does not discard already accepted reliable packets. A refused archive batch
retains its original bytes and clocks in the existing FIFO and retries admission
for at most 200 ms. Existing admission retry, capture-drop and publish-drop
counters continue to expose prolonged outages; archive retention is bounded by
the FIFO and the configured SEDSnet retry horizon.
