#ifndef BOARD_RECOVERY_BOOT_H
#define BOARD_RECOVERY_BOOT_H
#include <stdint.h>
/* A verified monotonic persisted boot generation; zero disables recovery.
 * It is not an authentication secret or a physical board identity. */
uint64_t board_recovery_boot_generation(void);
/* QUERY reads only the already-published RAM identity. No NVM retry. */
uint64_t board_recovery_boot_query_generation(void);
uint8_t board_recovery_boot_retry_is_safe(void);
/* Ordinary RECOVER requires prior verified identity. Explicit MAX bootstrap
 * is separately staged in the main owner and retains Stop. */
uint64_t board_recovery_boot_current(void);
uint8_t board_recovery_boot_publish_bootstrap(uint64_t generation);
#endif
