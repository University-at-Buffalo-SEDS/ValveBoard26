#ifndef BOOTSTRAP_OWNER_H
#define BOOTSTRAP_OWNER_H
#include <stdbool.h>
#include <stdint.h>
/* Main-control owner callbacks only. Completion runs AFTER releasing state;
 * the stop callback runs under state and may access bounded registers/RAM. */
typedef bool (*BootstrapOwnerComplete)(uint64_t generation,uint32_t old_epoch,uint32_t new_epoch,uint8_t desired,uint64_t issuer,uint64_t source);
bool board_recovery_bind_bootstrap_owner(BootstrapOwnerComplete complete,uint8_t(*stop)(void));
int32_t board_recovery_prepare_storage_only(uint32_t expected_epoch);
bool board_recovery_bind_bootstrap_request(uint64_t generation,uint32_t epoch,uint8_t desired,uint64_t issuer,uint64_t source);
bool board_recovery_bootstrap_verify_applied(uint32_t old_epoch,uint32_t new_epoch,uint8_t desired);
bool board_recovery_bootstrap_publish_complete(uint32_t old_epoch,uint32_t new_epoch,uint8_t desired);
void board_recovery_bootstrap_retain_stop(void);
/* Main owner only, immediately after original-source freshness validation. */
void board_recovery_execute_owned(void);
#endif
