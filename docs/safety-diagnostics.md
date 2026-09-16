# Safety-abort diagnostics

These diagnostics do not alter fail-safe behavior. Inspect the ELF symbols with
ST-Link/GDB after reproducing the problem.

- `g_safety_first_abort`: first local safety trigger per boot. Reason 0 means
  none recorded; 1 established-heartbeat timeout; 2 initial-heartbeat timeout
  (Actuator); 3 continuity loss (Valve); 4 launch continuity timeout (Valve);
  5 ADC startup failure (Actuator).
- `at_ms` and `last_heartbeat_ms` are local monotonic milliseconds, not network
  timestamps. `flight_state` captures the phase at the trigger.
- `detail` contains the continuity GPIO level on Valve or ADC startup step on
  Actuator. Later checks cannot replace the first snapshot.
- `g_sim_last_valve_command` and `g_sim_last_valve_command_ms` identify the last
  command dispatched after the abort guard; they do not prove physical movement.

An externally latched abort is not recorded as a local safety trigger. Reason 0
does not rule out an external abort. These diagnostics reset at reboot.

Valve fail-safe opens Dump and the normally-open valve and closes Pilot.
Actuator fail-safe turns outputs off. Therefore a command can execute correctly
and then be superseded by fail-safe action. Compare the command timestamp, first
abort timestamp, and reported output state before diagnosing an ACK mapping bug.

Both boards supervise heartbeat loss at five seconds in applicable flight
phases. Valve waits for its first heartbeat before supervising link loss;
Actuator retains its existing initial-heartbeat timeout. No safety checks were
disabled. Valve sends a reason-specific abort message on its existing transport;
the RAM snapshot remains available if that message cannot reach GroundStation.

Run `./build.py test` for compiled, mocked production safety-check regressions.
Hardware reproduction is still needed to establish the Actuator actual cause.

