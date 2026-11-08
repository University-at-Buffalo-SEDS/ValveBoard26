"""RESYNC on the Valve Board: the actual owner and output callback.

The transport/journal path (stale context, replayed nonce, Launch onward and
the applied phase) runs in test_board_recovery_transport. These cases cover the
Valve Board's own guarantees: a latched overpressure relief refuses a phase
target, and the real recovery output callback writes nothing for one.
"""
from pathlib import Path
import ast
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def owner_fixture():
    constants = [node.value for node in ast.walk(ast.parse(
        (ROOT / "tests/test_board_recovery_owner.py").read_text()
    )) if isinstance(node, ast.Constant) and isinstance(node.value, str)]
    tx = next(text for text in constants if "typedef unsigned UINT;" in text)
    code = next(text for text in constants if "PRODUCTION" in text and "int main(void)" in text)
    code = code.split("static unsigned prepared")[0]
    return tx, code.replace("PRODUCTION", (ROOT / "Core/Src/thread_comm.c").read_text())


def function(source, name):
    match = re.search(r"\b" + re.escape(name) + r"\([^;{}]*\)\s*\{", source)
    assert match, name
    start = source.rfind("\n", 0, match.start()) + 1
    end = match.end()
    depth = 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


class ValveResyncTests(unittest.TestCase):
    def build_and_run(self, tx, code, extra=()):
        with tempfile.TemporaryDirectory(prefix="vb-resync-") as directory:
            work = Path(directory)
            (work / "tx_api.h").write_text(tx)
            (work / "test.c").write_text(code)
            exe = work / "test"
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-fsanitize=address,undefined", "-I" + str(work),
                            "-I" + str(ROOT / "Core/Inc"), *extra, str(work / "test.c"),
                            "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)

    def test_latched_overpressure_relief_refuses_a_phase_target(self):
        tx, code = owner_fixture()
        code += r"""
static unsigned prepared,finished;static uint8_t last_desired=255;
static uint8_t read_outputs(uint32_t*mask){assert(g_thread_comm_state_mutex.locked);*mask=6U;return BOARD_RECOVERY_OUTPUTS_VALID|BOARD_RECOVERY_SAFE_OUTPUTS;}
static uint8_t prepare(uint8_t desired){assert(g_thread_comm_state_mutex.locked&&desired<=BOARD_RECOVERY_MAX_STATE);prepared++;last_desired=desired;return 1U;}
static void finish(void){assert(g_thread_comm_state_mutex.locked);finished++;}
static void relief(void*context){(void)context;}
int main(void){
 TX_BYTE_POOL pool={0};BoardRecoverySnapshot status;
 assert(thread_comm_init(&pool)==0);board_recovery_bind_owner(read_outputs,prepare,finish);
 assert(thread_comm_request_manual_abort()==TX_SUCCESS);
 assert(board_recovery_snapshot(&status)==0&&(status.flags&BOARD_RECOVERY_ABORT_LATCHED));uint32_t epoch=status.abort_epoch;
 /* The relief effect takes ownership exactly as the overpressure owner does. */
 assert(thread_comm_overpressure_effect(THREAD_COMM_OVERPRESSURE_OWN_RELIEF|THREAD_COMM_OVERPRESSURE_ANY_STATE,relief,0));
 for(uint8_t phase=2;phase<=BOARD_RECOVERY_MAX_STATE;phase++)assert(board_recovery_begin_prepare(phase,epoch)==-10);
 board_recovery_execute_owned();assert(!prepared&&!finished&&thread_comm_get_abort());
 assert(board_recovery_snapshot(&status)==0&&status.abort_epoch==epoch&&status.actual_state==0);
 /* Launch onward is never a recovery target, latched or not. */
 assert(board_recovery_begin_prepare(BOARD_RECOVERY_MAX_STATE+1U,epoch)==-10);
 /* The acknowledgement releases it; the phase target then applies. */
 thread_comm_overpressure_release();
 assert(board_recovery_begin_prepare(5,epoch)==0);assert(board_recovery_apply_safe(5,epoch,&status)==-14);
 board_recovery_execute_owned();assert(prepared==1&&finished==1&&last_desired==5&&!thread_comm_get_abort());
 assert(board_recovery_snapshot(&status)==0&&status.abort_epoch==epoch+1U&&status.actual_state==5&&status.actual_outputs_mask==6U);
 return 0;
}
"""
        self.build_and_run(tx, code)

    def test_actual_output_callback_writes_nothing_for_a_phase_target(self):
        source = (ROOT / "Core/Src/main_thread.c").read_text()
        body = function(source, "recovery_prepare_outputs")
        code = r"""
#include <assert.h>
#include <stdint.h>
#include "board_recovery.h"
static unsigned writes;static uint8_t g_outputs_initialized=1;
typedef struct{int unused;}Solenoid;static Solenoid pilot_solenoid;
static void no_servo_open(void){writes++;}
static void dump_servo_open(void){writes++;}
static void solenoidOff(Solenoid*s){(void)s;writes++;}
static uint8_t recovery_read_outputs(uint32_t*mask){*mask=6U;return BOARD_RECOVERY_OUTPUTS_VALID|BOARD_RECOVERY_SAFE_OUTPUTS;}
""" + body + r"""
int main(void){
 for(uint8_t phase=2;phase<=BOARD_RECOVERY_MAX_STATE;phase++){assert(recovery_prepare_outputs(phase)&&writes==0);}
 /* Recover to Startup is the path that drives the safe state. */
 assert(recovery_prepare_outputs(0)&&writes==3);
 return 0;
}
"""
        self.build_and_run("", code)


if __name__ == "__main__":
    unittest.main()
