#include "board_recovery_boot.h"
#include "persistent_store.h"
#include <stddef.h>
#include <stdatomic.h>
/* Application-owned persistent key "BRGE"; no SDK-reserved metadata. */
#define BOARD_RECOVERY_BOOT_KEY UINT32_C(0x42524745)
/* boot_generation is immutable after exclusive boot. Runtime firstleg
 * publishes only the fixed generation1 through a native-width atomic. */
static uint64_t boot_generation;
static _Atomic uint32_t bootstrap_generation_published;
static uint64_t decode_generation(const uint8_t value[8]) {
 uint64_t result=0U; for (unsigned i=0U;i<8U;i++) result|=(uint64_t)value[i]<<(8U*i); return result;
}
uint64_t board_recovery_boot_generation(void) {
 if (boot_generation!=0U) return boot_generation;
 /* Boot prime or explicitly safe unpublished retry. Re-read + increment: an
  * uncertain prior write burns an ID rather than reusing a prior boot ID. */
 if (persistent_store_init()!=LAUNCHCORE_PERSIST_OK) return 0U;
 uint8_t stored[8]={0U}; size_t length=sizeof stored;
 launchcore_persist_status_t result=persistent_store_get(BOARD_RECOVERY_BOOT_KEY,stored,&length);
 uint64_t previous=0U;
 if (result==LAUNCHCORE_PERSIST_OK) {
  if (length!=sizeof stored) return 0U;
  previous=decode_generation(stored);
  if (previous==0U || previous==UINT64_MAX) return 0U;
 } else return 0U; /* Exclusive boot initializes ABSENT identity; runtime never repairs it. */
 uint64_t next=previous+1U;
 for (unsigned i=0U;i<8U;i++) stored[i]=(uint8_t)(next>>(8U*i));
 if (persistent_store_set(BOARD_RECOVERY_BOOT_KEY,stored,sizeof stored)!=LAUNCHCORE_PERSIST_OK) return 0U;
 uint8_t verified[8]={0U}; length=sizeof verified;
 if (persistent_store_get(BOARD_RECOVERY_BOOT_KEY,verified,&length)!=LAUNCHCORE_PERSIST_OK ||
     length!=sizeof verified || decode_generation(verified)!=next) return 0U;
 boot_generation=next; return boot_generation;
}

uint64_t board_recovery_boot_current(void) { return boot_generation ? boot_generation : (atomic_load_explicit(&bootstrap_generation_published,memory_order_acquire) ? 1U : 0U); }

/* Default data/router/FC policy: never initiate storage work from QUERY. */
__attribute__((weak)) uint8_t board_recovery_boot_retry_is_safe(void) { return 0U; }
uint64_t board_recovery_boot_query_generation(void) {
 if (boot_generation!=0U) return boot_generation;
 return 0U; /* QUERY never performs storage work. */
}

uint8_t board_recovery_boot_publish_bootstrap(uint64_t generation) {
 if(boot_generation||generation!=1U)return 0U;
 uint32_t unpublished=0U;
 return atomic_compare_exchange_strong_explicit(&bootstrap_generation_published,&unpublished,1U,memory_order_release,memory_order_relaxed) ? 1U : 0U;
}
