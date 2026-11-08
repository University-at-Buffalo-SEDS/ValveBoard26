#ifndef BOARD_RECOVERY_WIRE_H
#define BOARD_RECOVERY_WIRE_H
#include <stddef.h>
#include <stdint.h>
/* Application codec helpers: explicit little-endian bytes, no alignment
 * requirement and no state/storage/volatile access. No protocol fields. */
#if defined(__GNUC__)
__attribute__((pure))
#endif
uint64_t board_recovery_read_le64(const uint8_t *p);
void board_recovery_write_le64(uint8_t *p,uint64_t value);

#define BOARD_RECOVERY_REQUEST_SIZE 32U
#define BOARD_RECOVERY_STATUS_SIZE 40U
#define BOARD_RECOVERY_VERSION 1U
/* RESYNC carries the station's current phase, PreFill(2)..Armed(6), and decodes
 * as RECOVER to that phase: same proof, safe-output, latch and epoch checks,
 * and no output is written. Startup/Idle stay with RECOVER, whose paths drive
 * or re-assert outputs; Launch onward is never recovered or aborted. */
enum { BOARD_RECOVERY_QUERY=0U, BOARD_RECOVERY_RECOVER=1U, BOARD_RECOVERY_RESYNC=2U };
enum { BOARD_RECOVERY_RESULT_OK=0U, BOARD_RECOVERY_RESULT_PENDING=1U,
 BOARD_RECOVERY_RESULT_UNSUPPORTED=2U, BOARD_RECOVERY_RESULT_UNAUTHORIZED=3U,
 BOARD_RECOVERY_RESULT_STALE=4U, BOARD_RECOVERY_RESULT_UNSAFE=5U,
 BOARD_RECOVERY_RESULT_INVALID=6U, BOARD_RECOVERY_RESULT_IO=7U };
/* PRIVATE native request, never a wire image. The explicit codec below
 * retains all32 wire offsets; this layout only shares native field equality. */
typedef struct {
 uint64_t nonce, issuer_session, expected_boot_generation;
 uint32_t expected_abort_epoch;
 uint8_t operation, desired_state;
} BoardRecoveryRequest;
#define BOARD_RECOVERY_REQUEST_NATIVE_PREFIX_SIZE 30U
#if defined(__cplusplus)
#define BOARD_REQUEST_LAYOUT_ASSERT static_assert
#else
#define BOARD_REQUEST_LAYOUT_ASSERT _Static_assert
#endif
BOARD_REQUEST_LAYOUT_ASSERT(sizeof(BoardRecoveryRequest)==32U&&
 offsetof(BoardRecoveryRequest,nonce)==0U&&sizeof(((BoardRecoveryRequest*)0)->nonce)==8U&&
 offsetof(BoardRecoveryRequest,expected_boot_generation)==16U&&sizeof(((BoardRecoveryRequest*)0)->expected_boot_generation)==8U&&
 offsetof(BoardRecoveryRequest,issuer_session)==8U&&sizeof(((BoardRecoveryRequest*)0)->issuer_session)==8U&&
 offsetof(BoardRecoveryRequest,expected_abort_epoch)==24U&&sizeof(((BoardRecoveryRequest*)0)->expected_abort_epoch)==4U&&
 offsetof(BoardRecoveryRequest,operation)==28U&&sizeof(((BoardRecoveryRequest*)0)->operation)==1U&&
 offsetof(BoardRecoveryRequest,desired_state)==29U&&sizeof(((BoardRecoveryRequest*)0)->desired_state)==1U&&
 offsetof(BoardRecoveryRequest,desired_state)+sizeof(((BoardRecoveryRequest*)0)->desired_state)==BOARD_RECOVERY_REQUEST_NATIVE_PREFIX_SIZE&&
 sizeof(BoardRecoveryRequest)-BOARD_RECOVERY_REQUEST_NATIVE_PREFIX_SIZE==2U,
 "native request equality needs contiguous unsigned fields30B; excludes2B tail");
#undef BOARD_REQUEST_LAYOUT_ASSERT
/* Strict shape validation only; caller must authorize source and validate the
 * live boot/abort epoch and canonical source-time before any state change. */
uint8_t board_recovery_decode_request(const uint8_t *wire, size_t length,
                                     BoardRecoveryRequest *request);
void board_recovery_encode_status(uint8_t wire[BOARD_RECOVERY_STATUS_SIZE],
 uint8_t result, uint8_t actual_state, uint8_t flags,
 const BoardRecoveryRequest *request, uint64_t boot_generation,
 uint32_t abort_epoch, uint32_t uptime_ms, uint32_t commanded_outputs);
#endif
