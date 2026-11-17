"""An Aborted FLIGHT_STATE is accepted while this board's Abort is latched.

The station sends ABORT and the Aborted FLIGHT_STATE in one burst. When the
ABORT latches first, refusing every FLIGHT_STATE write in Abort also refused
the matching Aborted value, so the board's flight-state cache never recorded
Aborted (seen in the full-network sim, about one cycle in three). The value
agrees with the latch: it is accepted without being applied again. Every other
value stays refused in Abort, and nothing changes outside Abort."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def function(source, name):
    match = re.search(r'^[^\n;]*\b' + re.escape(name) + r'\([^;]*?\)\s*\{', source, re.M)
    assert match, name
    start = match.start(); brace = source.index('{', start); end, depth = brace + 1, 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}'); end += 1
    return source[start:end]


HARNESS = r'''
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
typedef unsigned int UINT;
#define TX_SUCCESS 0U
#define THREAD_COMM_STALE_TIMESTAMP ((UINT)0xF001U)
typedef int SedsResult;
#define SEDS_OK 0
#define SEDS_BAD_ARG -1
#define SEDS_HANDLER_ERROR -3
#define SEDS_PERMISSION_DENIED -4
#define SEDS_IO -9
#define SEDS_DT_FLIGHT_STATE 256715704U
typedef struct {const uint8_t*payload;size_t payload_len;uint64_t timestamp;const char*sender;size_t sender_len;uint32_t ty;uint16_t time_epoch;} SedsPacketView;
/* network_stamp.h key of a view (epoch above a 47-bit timestamp). */
#define network_stamp_of(v) ((uint64_t)(v)->time_epoch<<47|(v)->timestamp)
#define FLIGHT_STATE_NET_ABORTED 15U
#define FLIGHT_STATE_NET_INVALID 0xFFU
static uint8_t flight_state_from_wire(uint8_t raw, uint8_t *retired){(void)retired;return raw>16U?FLIGHT_STATE_NET_INVALID:raw;}
#define FSTATE_RX() ((void)0)
#define FSTATE_OUT(pkt, result, reason) (result)
static unsigned g_flight_state_handler_error_count,g_flight_state_handler_count,g_last_flight_state_packet;
static int abort_latched; static unsigned applies;
static UINT thread_comm_get_abort(void){return (UINT)abort_latched;}
static UINT thread_comm_apply_network_flight_state(uint8_t s,uint64_t t){(void)t;applies++;if(s==FLIGHT_STATE_NET_ABORTED)abort_latched=1;return TX_SUCCESS;}
static SedsResult board_recovery_sync_command_cutoff(void){return SEDS_OK;}
static uint8_t board_recovery_command_is_stale(const SedsPacketView*p){(void)p;return 0;}
static uint8_t board_recovery_managed_update_allowed(const SedsPacketView*p){(void)p;return 1;}
static SedsResult board_recovery_refused(void){return SEDS_PERMISSION_DENIED;}
static int32_t seds_pkt_get_u8(const SedsPacketView*p,uint8_t*v,size_t n){(void)n;*v=p->payload[0];return 1;}
'''

TEST = r'''
int main(void){
  uint8_t v=FLIGHT_STATE_NET_ABORTED;
  SedsPacketView p={.ty=SEDS_DT_FLIGHT_STATE,.payload=&v,.payload_len=1,.timestamp=1767225835790ULL};
  /* Not in Abort: the Aborted value is applied (and latches). */
  assert(ADMIT(&p)==SEDS_OK&&applies==1&&abort_latched);
  /* ABORT latched first: the matching Aborted value is accepted (the cache
   * records it) without being applied a second time. */
  applies=0;
  assert(ADMIT(&p)==SEDS_OK);
  assert(applies==0);
  /* Any other value stays refused in Abort. */
  for(uint8_t s=0;s<=16;s++){
    if(s==FLIGHT_STATE_NET_ABORTED)continue;
    v=s;assert(ADMIT(&p)==SEDS_PERMISSION_DENIED);
  }
  assert(applies==0);
  /* Out of Abort a non-Aborted value is applied as before. */
  abort_latched=0;v=1;assert(ADMIT(&p)==SEDS_OK&&applies==1&&!abort_latched);
  return 0;
}
'''


class AbortedFlightStateInAbortTests(unittest.TestCase):
    def test_aborted_value_is_accepted_while_abort_is_latched(self):
        source = (ROOT / 'Core/Src/telemetry.c').read_text()
        if 'static SedsResult flight_state_admit(' in source:
            code = function(source, 'flight_state_admit') + '\n#define ADMIT(p) flight_state_admit(p)\n'
        else:
            code = function(source, 'Flight_State_handler') + '\n#define ADMIT(p) Flight_State_handler((p), 0)\n'
        with tempfile.TemporaryDirectory() as d:
            exe = str(Path(d) / 't')
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
                            '-fsanitize=address,undefined', '-x', 'c', '-', '-o', exe],
                           input=HARNESS + code + TEST, text=True, check=True)
            subprocess.run([exe], check=True)


if __name__ == '__main__':
    unittest.main()
