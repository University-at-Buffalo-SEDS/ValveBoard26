#ifndef BOARD_ABORT_LATCH_H
#define BOARD_ABORT_LATCH_H
#include <stdint.h>
/* Board-owned safety latch, independent of ThreadX and SEDSnet.
 * Only removing all board power clears it. No runtime clear operation exists. */
void board_abort_latch_init(void);
void board_abort_latch_set(void);
uint8_t board_abort_latch_is_set(void);
#endif
