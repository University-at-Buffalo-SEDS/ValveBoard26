"""A fresh Abort latch pushes one BOARD_STATE_STATUS to the station.

The station's ordered recovery queries can wait behind a radio query, so a
latch is otherwise seen only at the next answered QUERY. The 0->1 latch now
owes exactly one unsolicited status: nonce 0, ABORT_LATCHED, the live abort
epoch, the QUERY reply's generation and the last valid QUERY's issuer. A
repeat latch, recovery preparation and a retained latch at boot owe none.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def _function(source: str, signature: str) -> str:
    start = source.index(signature)
    return source[start:source.index('\n}\n', start) + 3]


def _build_and_run(code: str) -> None:
    with tempfile.TemporaryDirectory() as tmp:
        src = Path(tmp) / 'test.c'
        exe = Path(tmp) / 'test'
        src.write_text(code)
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
                        '-I', str(ROOT / 'Core/Inc'), str(src), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)


LATCH_STUB = r"""
#include <stdint.h>
#include <assert.h>
#define RCC_APB1ENR1_PWREN 1U
#define RCC_APB1ENR1_RTCAPBEN 2U
#define PWR_CR1_DBP 4U
#define __DSB() ((void)0)
static struct {uint32_t CSR, APB1ENR1;} rcc;
static struct {uint32_t CR1;} pwr;
static struct {uint32_t BKP28R, BKP29R, BKP30R, BKP31R;} tamp;
#define RCC (&rcc)
#define PWR (&pwr)
#define TAMP (&tamp)
"""

LATCH_MAIN = r"""
int main(void) {
    board_abort_latch_init();
    assert(!board_abort_latch_is_set() && !board_abort_latch_push_pending());
    /* The 0->1 latch owes exactly one push. */
    board_abort_latch_set();
    assert(board_abort_latch_push_pending());
    board_abort_latch_push_sent();
    assert(!board_abort_latch_push_pending());
    /* Repeat latches while latched owe none. */
    board_abort_latch_set(); board_abort_latch_set();
    assert(!board_abort_latch_push_pending());
    /* A retained latch restored at boot owes none. */
    latched = 0; backup_ready = 0; board_abort_latch_init();
    assert(board_abort_latch_is_set() && !board_abort_latch_push_pending());
    /* Recovery preparation (which calls the setter) owes none. */
    assert(board_abort_latch_clear_for_recovery() && !board_abort_latch_is_set());
    assert(board_abort_latch_prepare_recovery() && board_abort_latch_is_set());
    assert(!board_abort_latch_push_pending());
    /* After a completed recovery a new latch owes one again. */
    assert(board_abort_latch_clear_for_recovery());
    board_abort_latch_set();
    assert(board_abort_latch_push_pending());
    return 0;
}
"""

PUSH_HARNESS = r"""
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "board_recovery.h"
#include "board_recovery_wire.h"
typedef int SedsResult; typedef int SedsRouter; typedef int SedsDataType;
#define SEDS_OK 0
#define SEDS_DT_BOARD_STATE_STATUS 77
#define SEDS_EK_UNSIGNED 1
static uint8_t sent[64]; static size_t sent_len; static unsigned sends;
static SedsResult seds_router_log_queue_typed(SedsRouter*r,SedsDataType t,const void*d,size_t n,size_t e,int k){
 assert(r&&t==SEDS_DT_BOARD_STATE_STATUS&&e==1&&k==SEDS_EK_UNSIGNED);
 memcpy(sent,d,n);sent_len=n;++sends;return SEDS_OK;}
int32_t board_recovery_snapshot(BoardRecoverySnapshot*o){
 *o=(BoardRecoverySnapshot){.actual_state=15U,.flags=BOARD_RECOVERY_SUPPORTED|BOARD_RECOVERY_STATE_VALID|BOARD_RECOVERY_OUTPUTS_VALID|BOARD_RECOVERY_SAFE_OUTPUTS,
  .abort_epoch=9U,.actual_outputs_mask=0x5AU};return 0;}
static uint64_t board_bootstrap_storage_query_generation(void){return 0x1122334455667788ULL;}
static uint64_t last_query_issuer=0xA1A2A3A4A5A6A7A8ULL;
@WIRE@
@SEND@
@PUSH@
static uint64_t le64(const uint8_t*p){uint64_t v=0;for(int i=7;i>=0;--i)v=v<<8|p[i];return v;}
static uint32_t le32(const uint8_t*p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
int main(void){
 SedsRouter router=1;
 assert(board_recovery_push_abort_status(&router,123456U)==SEDS_OK&&sends==1&&sent_len==40U);
 assert(sent[0]==BOARD_RECOVERY_VERSION&&sent[1]==BOARD_RECOVERY_RESULT_OK&&sent[2]==15U);
 assert(sent[3]&BOARD_RECOVERY_ABORT_LATCHED);
 assert(le64(sent+4)==0U);                              /* nonce 0: unsolicited */
 assert(le64(sent+12)==0x1122334455667788ULL);          /* the QUERY reply's generation */
 assert(le32(sent+20)==9U&&le32(sent+24)==123456U&&le32(sent+28)==0x5AU);
 assert(le64(sent+32)==0xA1A2A3A4A5A6A7A8ULL);          /* last valid QUERY issuer */
 return 0;
}
"""

TELEMETRY_HARNESS = r"""
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
typedef int SedsResult; typedef int SedsRouter; typedef unsigned long ULONG;
#define SEDS_OK 0
#define SEDS_ERR -1
#define TELEMETRY_ENABLED 1
#define TX_TIMER_TICKS_PER_SECOND 1000U
static ULONG tx_time_get(void){return 5000U;}
static SedsRouter router=1; static struct{SedsRouter*r;}g_router={&router};
static int entered,pending,pushes,refuse;
static int telemetry_router_enter(void){++entered;return 1;}
static int telemetry_router_leave(void){--entered;return 1;}
static uint8_t board_abort_latch_push_pending(void){return (uint8_t)pending;}
static void board_abort_latch_push_sent(void){pending=0;}
static SedsResult board_recovery_push_abort_status(SedsRouter*r,uint32_t up){assert(r&&entered==1&&up==5000U);++pushes;return refuse?SEDS_ERR:SEDS_OK;}
@PUSH@
int main(void){
 telemetry_push_abort_status();assert(pushes==0);        /* nothing owed */
 pending=1;refuse=1;telemetry_push_abort_status();
 assert(pushes==1&&pending==1&&entered==0);            /* refused: retried next pass */
 refuse=0;telemetry_push_abort_status();assert(pushes==2&&pending==0);
 telemetry_push_abort_status();assert(pushes==2);        /* exactly one accepted push */
 return 0;
}
"""


class AbortStatusPushTests(unittest.TestCase):
    def test_only_a_fresh_latch_owes_one_push(self):
        source = (ROOT / 'Core/Src/board_abort_latch.c').read_text()
        source = '\n'.join(l for l in source.splitlines() if not l.startswith('#include'))
        _build_and_run(LATCH_STUB + source + LATCH_MAIN)

    def test_push_is_a_nonce_zero_abort_latched_status(self):
        transport = (ROOT / 'Core/Src/board_recovery_transport.c').read_text()
        wire = (ROOT / 'Core/Src/board_recovery_wire.c').read_text()
        wire = '\n'.join(l for l in wire.splitlines() if not l.startswith('#include'))
        code = (PUSH_HARNESS.replace('@WIRE@', wire)
                .replace('@SEND@', _function(transport, 'static SedsResult send_status('))
                .replace('@PUSH@', _function(transport, 'SedsResult board_recovery_push_abort_status(')))
        _build_and_run(code)

    def test_issuer_is_cached_from_each_valid_query(self):
        transport = (ROOT / 'Core/Src/board_recovery_transport.c').read_text()
        handler = _function(transport, 'static SedsResult handle_serialized(')
        query = handler.index('if(request.operation==BOARD_RECOVERY_QUERY) {')
        self.assertLess(handler.index('authorized_primary(router,packet)'), query)
        self.assertLess(handler.index('board_recovery_decode_request('), query)
        self.assertIn('last_query_issuer=request.issuer_session;', handler[query:query + 120])

    def test_telemetry_thread_sends_it_and_retries_only_a_refusal(self):
        telemetry = (ROOT / 'Core/Src/telemetry.c').read_text()
        _build_and_run(TELEMETRY_HARNESS.replace(
            '@PUSH@', _function(telemetry, 'void telemetry_push_abort_status(void)')))
        loop = (ROOT / 'Core/Src/telemetry_thread.c').read_text()
        self.assertIn('telemetry_report_safety_abort();\n        telemetry_push_abort_status();', loop)


if __name__ == '__main__':
    unittest.main()
