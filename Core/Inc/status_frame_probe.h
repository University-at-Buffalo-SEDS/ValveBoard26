#ifndef STATUS_FRAME_PROBE_H
#define STATUS_FRAME_PROBE_H
#include <stddef.h>
#include <stdint.h>
/* Passive, stateless classification; compact and chunk frames are unknown. */
static inline uint32_t status_frame_type(const uint8_t *data, size_t len) {
  size_t offset = 0;
  if (!data || len < 3) return UINT32_MAX;
  if (data[0] == 83 && data[1] == 68 && data[2] == 84) {
    if (len < 5 || data[3] != 1) return UINT32_MAX;
    offset = 4;
    /* Full-frame template ID is a u32 LEB128. */
    for (unsigned n = 0; ; n++) {
      if (offset >= len || n >= 5) return UINT32_MAX;
      uint8_t b = data[offset++];
      if (n == 4 && (b & 0xf0)) return UINT32_MAX;
      if (!(b & 0x80)) break;
    }
  }
  if (len - offset < 3) return UINT32_MAX;
  offset += 2;
  uint32_t ty = 0;
  for (unsigned n = 0; n < 5 && offset < len; n++) {
    uint8_t b = data[offset++];
    if (n == 4 && (b & 0xf0)) return UINT32_MAX;
    ty |= (uint32_t)(b & 127) << (7 * n);
    if (!(b & 128)) return ty;
  }
  return UINT32_MAX;
}
#endif
