#ifndef PERSISTENT_STORE_HEALTH_H
#define PERSISTENT_STORE_HEALTH_H
#include <stdbool.h>
#include <stdint.h>
/* Atomic, nonblocking controls gate; never takes the storage mutex. */
bool persistent_store_controls_ready(void);
bool persistent_store_completed_boundary_ready(void);
/* Only fresh explicit RECOVER may classify/commit a healthy truly absent floor.
 * These calls never provision virgin media or revive a deleted record. */
bool persistent_store_recovery_ready(void);
void persistent_store_note_failure(void);
void persistent_store_hold_bootstrap(void);
bool persistent_store_bootstrap_pending(void);
bool persistent_store_release_unused_hold(void);
bool persistent_store_bootstrap_faulted(void);
bool persistent_store_bootstrap_metadata_ready(void);
bool persistent_store_verify_bootstrap(uint64_t generation,const uint8_t floor[16],const uint8_t *marker,bool complete);
bool persistent_store_publish_bootstrap_complete(void);
#endif
