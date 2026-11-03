#ifndef BOARD_BOOTSTRAP_STORAGE_H
#define BOARD_BOOTSTRAP_STORAGE_H
#include <stdint.h>
#include <stdbool.h>
#include "board_recovery_wire.h"
typedef enum {BOOTSTRAP_NONE,BOOTSTRAP_VIRGIN,BOOTSTRAP_MISSING_GENERATION,BOOTSTRAP_MISSING_FLOOR} BoardBootstrapKind;
typedef enum {BOOTSTRAP_NOT_ELIGIBLE,BOOTSTRAP_STORED_INHIBITED,BOOTSTRAP_IO,BOOTSTRAP_STALE,BOOTSTRAP_INVALID} BoardBootstrapResult;
typedef struct{uint64_t durable_generation;uint32_t actual_abort_epoch;}BoardBootstrapReply;
typedef bool(*BootstrapReply)(void*,const BoardRecoveryRequest*,uint8_t);
typedef uint64_t(*BootstrapNow)(void*);
/* Storage-only probe in explicit boot exclusivity before first restore. */
void board_bootstrap_storage_probe_boot(void);
/* Main control task only; binds checked owner callbacks and one job slot. */
bool board_bootstrap_storage_bind_owner(uint8_t(*stop)(void));
void board_bootstrap_storage_service(void);
/* All QUERY helpers below are RAM/register-only; no storage or GPIO writes. */
uint64_t board_bootstrap_storage_query_generation(void);
bool board_bootstrap_storage_pending(void);
bool board_bootstrap_storage_uncertain_first(void);
bool board_bootstrap_storage_query_healthy(void);
bool board_bootstrap_storage_note_query(const BoardRecoveryRequest*,uint64_t,uint64_t,uint32_t);
/* Primary/freshness/shape checks precede staging. -1 means main owner owes
 * the application reply; an SDK ACK is never provisioning confirmation. */
int32_t board_bootstrap_storage_stage_first_leg(const BoardRecoveryRequest*,uint64_t,uint64_t,uint32_t,void*,BootstrapReply,BootstrapNow);
/* Main job implementation and owner-completed finalizer; no wire registration. */
BoardBootstrapResult board_bootstrap_storage_first_leg(const BoardRecoveryRequest*,uint64_t,uint64_t,BoardBootstrapReply*);
#endif
