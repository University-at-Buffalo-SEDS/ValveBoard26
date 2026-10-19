from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class AbortLatchTests(unittest.TestCase):
    def test_actual_retained_latch_across_reset_classes_and_torn_writes(self):
        source = (ROOT / 'Core/Src/board_abort_latch.c').read_text()
        source = '\n'.join(line for line in source.splitlines() if not line.startswith('#include'))
        stub = r"""
#include <stdint.h>
#include <assert.h>
#define RCC_APB1ENR1_PWREN 1U
#define RCC_APB1ENR1_RTCAPBEN 2U
#define PWR_CR1_DBP 4U
#define RCC_CSR_BORRSTF (1U << 27)
#define __DSB() ((void)0)
static struct {uint32_t CSR, APB1ENR1;} rcc;
static struct {uint32_t CR1;} pwr;
static struct {uint32_t BKP30R, BKP31R;} tamp;
#define RCC (&rcc)
#define PWR (&pwr)
#define TAMP (&tamp)
"""
        main = r"""
int main(void) {
    /* Actual power loss resets backup registers, not just the core CPU. */
    tamp.BKP30R=0; tamp.BKP31R=0; rcc.CSR=RCC_CSR_BORRSTF;
    board_abort_latch_init(); assert(!board_abort_latch_is_set());
    assert(!tamp.BKP30R && !tamp.BKP31R);
    board_abort_latch_set(); assert(board_abort_latch_is_set());
    for (unsigned reset=0; reset<5; ++reset) {
        /* Brownout, software, watchdog and external NRST resets retain power. */
        rcc.CSR=1U << (27U+reset); latched=0; backup_ready=0;
        board_abort_latch_init(); assert(board_abort_latch_is_set());
        board_abort_latch_set(); assert(board_abort_latch_is_set());
    }
    tamp.BKP31R=0; rcc.CSR=0; latched=0;
    board_abort_latch_init(); assert(board_abort_latch_is_set());
    /* Even a brownout flag is not permission to unlock a retained latch. */
    rcc.CSR=RCC_CSR_BORRSTF; board_abort_latch_init();
    assert(board_abort_latch_is_set());
    /* Removing VDD and VBAT resets the backup domain in hardware. */
    tamp.BKP30R=0; tamp.BKP31R=0; latched=0; backup_ready=0;
    board_abort_latch_init();
    assert(!board_abort_latch_is_set());
    rcc.CSR=1U << 28; board_abort_latch_init();
    assert(!board_abort_latch_is_set()); /* Normal reboot stays clear. */
    return 0;
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            src=Path(tmp)/'test.c'; exe=Path(tmp)/'test'
            src.write_text(stub+'\n'+source+'\n'+main)
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',str(src),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

    def test_launch_cannot_clear_or_bypass_abort(self):
        source=(ROOT/'Core/Src/main_thread.c').read_text()
        fn=source.split('static void service_launch_sequence_request(void)',1)[1].split('static void service_launch_sequence(void)',1)[0]
        fn='static void service_launch_sequence_request(void)'+fn
        stub=r"""
#include <stdint.h>
#include <assert.h>
#define VALVE_FLIGHT_STATE_LAUNCH 6
static unsigned aborted=1, starts, takes;
static unsigned thread_comm_get_abort(void){return aborted;}
static unsigned thread_comm_take_launch_sequence_request(uint64_t *stamp){takes++;*stamp=123;return 1;}
static void thread_comm_set_flight_state(unsigned state){(void)state;}
static void thread_comm_set_launch_command_timestamp_ms(uint64_t stamp){(void)stamp;}
static void start_launch_sequence(void){starts++;}
"""
        main=r"""
int main(void){
 service_launch_sequence_request();assert(!starts && !takes && aborted);
 aborted=0;service_launch_sequence_request();assert(starts==1 && takes==1);
 return 0;
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            src=Path(tmp)/'test.c';exe=Path(tmp)/'test'
            src.write_text(stub+fn+main)
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',str(src),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

    def test_init_precedes_reset_flag_clear_and_no_clear_api_exists(self):
        source=(ROOT/'Core/Src/main.c').read_text()
        self.assertLess(source.index('board_abort_latch_init();'),source.index('board_watchdog_start();'))
        header=(ROOT/'Core/Inc/board_abort_latch.h').read_text()
        self.assertNotIn('void board_abort_latch_clear',header)

if __name__ == '__main__':
    unittest.main()
