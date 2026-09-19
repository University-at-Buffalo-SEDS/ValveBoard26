# Safety timeouts and abort reporting

Heartbeat freshness uses local ThreadX uptime, not network time or UTC.
The heartbeat is sampled before the current time; a newer receive timestamp
cannot underflow into an apparent timeout. The existing five-second timeout,
flight-state gating, and latched safe-output behavior remain unchanged.

The first local safety cause is retained in `g_safety_first_abort`.
The telemetry thread submits that cause as the existing reliable ABORT data
type. Failed local submissions retry once per second; successful submissions
use SEDSNet's reliability handling. This is not proof of remote receipt during
a disconnected link. GroundStation's ABORT handler latches its abort indicator
and emits the reason as an error. Received aborts do not originate new broadcasts.

Run `./build.py test` for the production-function regression tests. They cover
newer heartbeat timestamps, the real timeout boundary, ten minutes of modeled
heartbeat checks, rejected notification submissions, retry tick wrap, and
suppression of duplicate/echo notifications. These are host tests, not a
hardware or full-network endurance qualification. A real abort stays latched;
restoring traffic does not automatically re-arm outputs.
