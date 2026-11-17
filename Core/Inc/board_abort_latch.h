#ifndef BOARD_ABORT_LATCH_H
#define BOARD_ABORT_LATCH_H
#include <stdint.h>
/* Board-owned safety latch, independent of ThreadX and SEDSnet.
 * Explicit control-owner recovery can clear it after safe output verification. */
void board_abort_latch_init(void);
void board_abort_latch_set(void);
/* Verified persistent Abort barrier before manual recovery writes any flash. */
uint8_t board_abort_latch_prepare_recovery(void);
uint8_t board_abort_latch_is_set(void);
/* Set by a 0->1 latch (never by recovery preparation or a retained boot);
 * the telemetry thread pushes one BOARD_STATE_STATUS, then clears it. */
uint8_t board_abort_latch_push_pending(void);
void board_abort_latch_push_sent(void);

void board_launch_latch_set(void);
/* Return true only after current backup marker write and exact readback. */
uint8_t board_launch_latch_set_verified(void);
uint8_t board_launch_latch_is_set(void);
/* Monotonic RAM evidence of a full verified marker this lifecycle/retained boot.
 * Metadata only: this never replaces fresh physical admission verification. */
uint8_t board_launch_latch_was_verified(void);

uint8_t board_abort_latch_clear_for_recovery(void);

/* Post-Launch auto-vent marker (vent_countdown.h). Cleared by a new Launch
 * marker and by recovery. A torn or absent marker loads as 0/not vented. */
uint8_t board_vent_marker_load(uint32_t *elapsed_s, uint8_t *vented);
void board_vent_marker_store(uint32_t elapsed_s, uint8_t vented);
uint8_t board_vent_marker_vented(void);

#endif
