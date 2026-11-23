# Safety-abort diagnostics

These diagnostics do not alter fail-safe behavior. Inspect the ELF symbols with
ST-Link/GDB after reproducing the problem.

- `g_safety_first_abort`: first local safety trigger per boot. Reason 0 means
  none recorded; 1 established-heartbeat timeout; 2 initial-heartbeat timeout
  (Actuator); 3 continuity loss (Valve); 4 launch continuity timeout (unused: after
  Launch a continuity break feeds the auto-vent countdown below);
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

## Post-Launch auto-vent (Valve)

After an accepted, unaborted Launch the Valve Board no longer aborts on link
or continuity loss and holds Dump and Vent closed, also across a reset. So
that nobody approaches a pressurized tank with nobody in control, it counts
down while the GroundStation link is lost (no GS heartbeat for 5 s) OR the
launch continuity wire is broken (`Core/Inc/vent_countdown.h`):

- Link loss is measured from the last GS heartbeat, continuity from the first
  sample of the break. A break counts only once 4 consecutive 10 ms samples
  agree (PB15 has a pull-down, so an open wire reads broken); the same
  debounce gates the pre-Launch continuity abort. The vent comes at
  max(start + 150 s, Launch + 60 s): never during the burn.
- It is cancelled only when the heartbeat is present AND continuity is intact,
  or by an applied operator valve command (which also restarts a full countdown
  if a trigger persists). A trigger clearing while the other persists keeps
  the original deadline.
- On expiry it drives the Abort safe state (Vent open, Dump open, Pilot off,
  same order and functions), under the shared state owner, without an Abort
  latch, and latches "vented".
- TAMP backup registers 26/27 keep the Launch elapsed time (saturating at the
  60 s guard) and the vented latch. A reset after the vent stays vented; a
  reset before it holds closed and, with no heartbeat, vents 150 s after boot.
- It reports as a `WARNING`: `Valve Board auto-vent in N s: link lost [LN]`
  (K continuity broken, B both), `Valve Board auto-vented: ... [Vm]` and
  `Valve Board auto-vent cancelled [X0]`, on change and every 5 s while
  counting or vented. `g_sim_vent_state` holds state | triggers << 2 |
  remaining seconds << 8.

Run `./build.py test` for compiled, mocked production safety-check regressions.
Hardware reproduction is still needed to establish the Actuator actual cause.

