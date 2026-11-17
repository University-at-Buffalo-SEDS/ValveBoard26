#ifndef NETWORK_STAMP_H
#define NETWORK_STAMP_H
#include <stdint.h>
/* A SEDSnet network stamp (time_epoch, timestamp_ms) as one ordered key:
 * the epoch in bits 47..62 above a 47-bit millisecond timestamp (UTC ms fit
 * until the year 6429). Unsigned order of keys is exactly seds_stamp_cmp()
 * order, so every stored ordering mark (replay floor, command cutoff,
 * accepted Launch owner, recovery request sources) keeps its width and its
 * plain comparisons while ordering correctly across network time steps in
 * either direction. An epoch 0 key equals the plain timestamp, so a record
 * written before epochs existed reads as epoch 0. 0 stays "no stamp" and
 * UINT64_MAX stays invalid. Freshness is never judged from a key's value:
 * use seds_router_network_stamp_fresh() with its epoch and milliseconds. */
#define NETWORK_STAMP_MS_BITS 47U
#define NETWORK_STAMP_MS_MASK ((UINT64_C(1) << NETWORK_STAMP_MS_BITS) - 1U)
static inline uint64_t network_stamp_key(uint16_t epoch, uint64_t ms)
{
    if (ms == 0U) return 0U;
    if (ms > NETWORK_STAMP_MS_MASK) return UINT64_MAX;
    return ((uint64_t)epoch << NETWORK_STAMP_MS_BITS) | ms;
}
static inline uint16_t network_stamp_epoch(uint64_t key) { return (uint16_t)(key >> NETWORK_STAMP_MS_BITS); }
static inline uint64_t network_stamp_ms(uint64_t key) { return key & NETWORK_STAMP_MS_MASK; }
/* The key of a received packet view. */
#define network_stamp_of(view) network_stamp_key((view)->time_epoch, (view)->timestamp)
#endif
