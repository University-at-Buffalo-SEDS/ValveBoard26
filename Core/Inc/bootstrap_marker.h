#ifndef BOOTSTRAP_MARKER_H
#define BOOTSTRAP_MARKER_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "board_recovery_wire.h"
/* v1/32B means COMMISSION with immutable expectedMAX. v2/40B additionally
 * binds original actualgeneration for floor-only upgrade. No wire changes. */
#define BOOTSTRAP_MARKER_KEY UINT32_C(0x42525043)
#define BOOTSTRAP_MARKER_SIZE 40U
typedef struct{uint8_t state,desired,version,kind;uint32_t first_epoch;uint64_t first_time,first_nonce,first_issuer,first_expected_generation;}BootstrapMarker;
bool bootstrap_request_fresh(uint64_t source,uint64_t now);
/* Compatibility names share the wire module's explicit byte codec. */
#define bootstrap_read64 board_recovery_read_le64
#define bootstrap_write64 board_recovery_write_le64
static inline uint32_t bootstrap_read32(const uint8_t*p){return(uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
static inline void bootstrap_write32(uint8_t*p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
size_t bootstrap_marker_size(const BootstrapMarker*m);
bool bootstrap_marker_decode(const uint8_t*p,size_t n,BootstrapMarker*out);
void bootstrap_marker_encode(uint8_t*p,const BootstrapMarker*m);
bool bootstrap_marker_same(const BootstrapMarker*a,const BootstrapMarker*b);
void bootstrap_marker_request(BootstrapMarker*out,const BoardRecoveryRequest*r,uint64_t source,uint8_t version,uint8_t kind);
#endif
