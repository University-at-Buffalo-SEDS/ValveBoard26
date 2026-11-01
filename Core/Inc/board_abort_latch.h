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

void board_launch_latch_set(void);
/* Return true only after current backup marker write and exact readback. */
uint8_t board_launch_latch_set_verified(void);
uint8_t board_launch_latch_is_set(void);

uint8_t board_abort_latch_clear_for_recovery(void);

#endif
