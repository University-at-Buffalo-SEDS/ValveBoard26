#ifndef PERSISTENT_STORE_H
#define PERSISTENT_STORE_H
#include "launchcore/persist.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
/* Caller owns the same physical-flash mutex through each OTA operation. */
bool persistent_store_enter(void);
bool persistent_store_leave(void);
bool persistent_store_controls_ready(void);
launchcore_persist_status_t persistent_store_boot_begin(void);
unsigned int persistent_store_runtime_start(void);
launchcore_persist_status_t persistent_store_init(void);
launchcore_persist_status_t persistent_store_get(uint32_t key, void *value, size_t *value_size);
launchcore_persist_status_t persistent_store_set(uint32_t key, const void *value, size_t value_size);
/* Atomic COMPLETE lineage + current readiness, never takes storage owner. */
bool persistent_store_completed_boundary_ready(void);
#endif

/* Receiver contention is explicit BUSY, never IO success or readiness fault. */
bool persistent_store_try_enter(bool*busy);
launchcore_persist_status_t persistent_store_try_set(uint32_t key,const void*value,size_t size,bool*busy);
