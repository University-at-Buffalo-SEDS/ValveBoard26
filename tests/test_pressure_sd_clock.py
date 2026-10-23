"""Execute the production clock stamp with independent ADC and IRQ clock drift."""
from pathlib import Path
import subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class PressureClockTests(unittest.TestCase):
 def test_sequence_clock_and_observed_utc_age_remain_independent(self):
  code=r'''#include "pressure_sd_clock.h"
#include <assert.h>
int main(void){
 pressure_sd_clock_t clock={0};pressure_sd_batch_t batch={0};
 const uint64_t unix_base=1800000000000ULL;
 for(uint32_t n=0;n<4096;n++){
   const uint32_t observed=1000U+n*16U-n/8U;
   pressure_sd_stamp_clock(&clock,&batch,n*4U,observed,observed+14U,unix_base+observed+14U);
   assert(batch.uptime_ms==1000ULL+n*16ULL);
   assert(batch.first_sequence==n*4U);
   assert(batch.unix_ms==unix_base+observed);
 }
 /* A genuinely missing batch stays a gap in sequence/sample clock. */
 pressure_sd_stamp_clock(&clock,&batch,4097U*4U,70000U,70014U,unix_base+70014U);
 assert(batch.uptime_ms==1000ULL+4097ULL*16ULL);
 assert(batch.unix_ms==unix_base+70000ULL);
 /* Capture precedes a HAL wrap; sample sequence independently wraps too. */
 pressure_sd_clock_t wrap={.ticks=0x1fffffffeULL,.sequence=UINT32_MAX-3U,.initialized=1U};
 pressure_sd_stamp_clock(&wrap,&batch,0U,UINT32_MAX-3U,0x10000000aULL,unix_base);
 assert(batch.uptime_ms==0x20000000eULL);
 assert(batch.unix_ms==unix_base-14ULL);
 return 0;
}'''
  with tempfile.TemporaryDirectory() as d:
   exe=Path(d)/'clock'
   subprocess.run(['cc','-I',str(ROOT/'Core/Inc'),'-std=c11','-O2','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',str(exe)],input=code,text=True,check=True)
   subprocess.run([str(exe)],check=True)
