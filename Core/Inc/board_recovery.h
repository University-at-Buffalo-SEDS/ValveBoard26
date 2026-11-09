#ifndef BOARD_RECOVERY_H
#define BOARD_RECOVERY_H
#include <stdint.h>
/* Highest recovery target: Armed. RECOVER uses 0..1, RESYNC 2..this. */
#define BOARD_RECOVERY_MAX_STATE 6U
/* Application protocol only. Wire encoding is explicit LE, never memcpy of this struct. */
enum {
 BOARD_RECOVERY_ABORT_LATCHED=1u<<0,
 BOARD_RECOVERY_SAFE_OUTPUTS=1u<<1,
 BOARD_RECOVERY_SUPPORTED=1u<<2,
 BOARD_RECOVERY_STATE_VALID=1u<<3,
 BOARD_RECOVERY_OUTPUTS_VALID=1u<<4,
 /* Hardened application transport; independent of readiness or output proof. */
 BOARD_RECOVERY_VERIFIED_IMPLEMENTATION=1u<<5,
 /* Exact known-UTC manual Stop retirement, independent of Ready/Applied. */
 BOARD_RECOVERY_MANUAL_UTC_IMPLEMENTATION=1u<<6,
 /* Status-only: accepts RESYNC to the station's phase without moving outputs.
  * Never set in a snapshot, so it cannot alias the internal callback bit. */
 BOARD_RECOVERY_RESYNC_IMPLEMENTATION=1u<<7
};
typedef struct {
 uint8_t actual_state;
 uint8_t flags;
 uint16_t reserved;
 uint32_t abort_epoch;
 uint32_t actual_outputs_mask;
} BoardRecoverySnapshot;
/* SEDS_OK (0) or negative SedsResult. Snapshot must be a serialized actual readback. */
int32_t board_recovery_snapshot(BoardRecoverySnapshot *out);
/* Manual operator request only. Allowed desired_state: Startup(0), Idle(1), or
 * through RESYNC the station phase PreFill(2)..Armed(6). A RESYNC target never
 * writes an output; outputs must already read safe. A latched overpressure
 * relief refuses targets beyond Idle until the operator acknowledges it.
 * Under the board's state lock, validate expected_abort_epoch, check safe outputs,
 * purge queued commands, verify explicit persistent latch clear, update state and
 * advance epoch. A concurrent abort must invalidate this request. Never replay
 * actuator/valve/igniter/launch/pyro commands or claim unsupported hardware safe.
 * Return a fresh readback even on rejection where possible. */
int32_t board_recovery_apply_safe(uint8_t desired_state,
                                uint32_t expected_abort_epoch,
                                BoardRecoverySnapshot *out);

/* Telemetry preparation barrier; no storage or SDK while this state lock is held. */
/* RAM-only job admission, serialized against accepted Launch before any
 * query/job mutation, storage hold or NVM work. 0=owned, negative=refused. */
int32_t board_recovery_stage_storage_hold(uint32_t expected_epoch);
/* Set only around staging/preparing a RESYNC to Startup/Idle: lets that one
 * recovery revoke a live (not aborted) launch owner itself instead of being
 * refused. The internal stop drives outputs safe without latching Abort. */
extern volatile uint8_t board_recovery_stop_launch;
/* RAM-only proof captured by an authorized fresh QUERY after completion.
 * The transport binds issuer/nonce/boot/epoch; this owner binds actual Launch. */
typedef struct { uint64_t source_utc; uint32_t serial; } BoardRecoveryCompletedLaunch;
uint8_t board_recovery_completed_query(uint32_t expected_epoch, uint64_t query_source,
                                      BoardRecoveryCompletedLaunch *out);
int32_t board_recovery_stage_completed_hold(uint32_t expected_epoch, uint64_t source,
                                          uint64_t query_source,
                                          const BoardRecoveryCompletedLaunch *proof);
int32_t board_recovery_begin_prepare(uint8_t desired_state, uint32_t expected_epoch);
void board_recovery_cancel_prepare(uint8_t desired_state, uint32_t expected_epoch);

/* Internal callback-only bit: never serialized into status flags. */
#define BOARD_RECOVERY_OWNER_STOPPED UINT8_C(0x80)

/* Control-owner binding. Functions run under the state mutex and must perform
 * bounded register/RAM access only: no SDK, storage, status sends or waits. */
void board_recovery_bind_owner(uint8_t (*read_outputs)(uint32_t *),
                              uint8_t (*prepare)(uint8_t), void (*finish)(void));
void thread_comm_service_board_recovery(void);
#endif
