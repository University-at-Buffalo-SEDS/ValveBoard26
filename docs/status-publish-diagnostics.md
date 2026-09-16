# Status publish diagnostics

g_valve_status_publish indexes status ID (0 Pilot, 1 Vent, 2 Dump, 6 Sequence).
Each entry records completed publish calls, accepted calls, failed calls,
last requested state, local ThreadX tick, and final SEDSNet result.
The existing three-attempt retry behavior is unchanged.

Acceptance is not proof of CAN transmission or remote acknowledgement.
g_valve_status_can_attempts/ok/fail record identifiable status frames at the CAN
driver boundary. CAN success means driver acceptance, not GroundStation receipt.
g_valve_status_can_last_tick timestamps the latest identifiable attempt.
g_valve_can_unclassified counts compact/chunk/unknown frames that this stateless
observer cannot identify. Do not infer loss solely from zero status counts if
that counter advances.

g_valve_queue_service_errors and g_valve_queue_last_error preserve errors returned
by the normal combined queue service. These diagnostics are passive and static;
they do not add endpoints, change routing, or disable safety.
