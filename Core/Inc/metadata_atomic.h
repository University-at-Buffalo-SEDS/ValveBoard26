#ifndef METADATA_ATOMIC_H
#define METADATA_ATOMIC_H
#include <stdatomic.h>
#include <stdbool.h>
/* One exact seq_cst read per call; no caching, grouping or pure annotation. */
static __attribute__((noinline,noclone)) bool metadata_load_bool(const _Atomic bool *value)
{
 return atomic_load_explicit(value,memory_order_seq_cst);
}
/* Exactly one seq_cst store per original bool assignment. */
static __attribute__((noinline,noclone)) void metadata_store_bool(_Atomic bool *value,bool desired)
{
 atomic_store_explicit(value,desired,memory_order_seq_cst);
}
#endif
