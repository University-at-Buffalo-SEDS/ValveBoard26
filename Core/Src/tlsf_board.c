/* The vendored TLSF allocator, built with a fail-stop invariant check that
 * does not link newlib's assert/fprintf path (~700 B of flash). A broken heap
 * invariant still stops the board: abort() ends in _exit()'s hang and the
 * watchdog resets it, exactly as the failed assert did. */
#define tlsf_assert(expr) ((expr) ? (void)0 : __builtin_abort())
#include "tlsf.c"
