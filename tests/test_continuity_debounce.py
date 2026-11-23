"""Launch continuity wire: fail-safe pull and debounce.

The pure debounce module (continuity_debounce.h) runs against random glitch
patterns and real breaks; the GPIO configuration is checked in main.c and the
CubeMX .ioc. Host logic evidence only: no MCU timing or hardware.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

HARNESS = r'''
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "continuity_debounce.h"
#define PERIOD_MS 10U
int main(void){
 continuity_debounce_t d;continuity_debounce_init(&d);
 uint64_t now=1000;
 /* Window: the confirmed state changes on the Nth sample of a run. */
 const uint64_t window=(CONTINUITY_DEBOUNCE_SAMPLES-1U)*PERIOD_MS;
 assert(window>=20U&&window<=50U);
 /* Random glitches of up to N-1 samples in either direction never change
  * the confirmed state. */
 srand(1);
 for(int phase=0;phase<2;phase++){
  const uint8_t base=phase?0U:1U; /* intact, then a real break */
  for(unsigned i=0;i<CONTINUITY_DEBOUNCE_SAMPLES;i++,now+=PERIOD_MS)continuity_debounce_step(&d,base,now);
  assert(d.broken==!base);
  const uint64_t since=d.broken_since_ms;
  for(int i=0;i<200000;i++){
   unsigned glitch=(unsigned)rand()%CONTINUITY_DEBOUNCE_SAMPLES;
   for(unsigned g=0;g<glitch;g++,now+=PERIOD_MS)assert(continuity_debounce_step(&d,!base,now)==!base);
   unsigned steady=1U+(unsigned)rand()%3U;
   for(unsigned g=0;g<steady;g++,now+=PERIOD_MS)assert(continuity_debounce_step(&d,base,now)==!base);
  }
  assert(d.broken_since_ms==since);
 }
 /* A real break trips within the window and reports its first sample. */
 continuity_debounce_init(&d);
 for(unsigned i=0;i<CONTINUITY_DEBOUNCE_SAMPLES;i++,now+=PERIOD_MS)continuity_debounce_step(&d,1U,now);
 const uint64_t broke=now;uint64_t confirmed=0;
 for(unsigned i=0;i<CONTINUITY_DEBOUNCE_SAMPLES;i++,now+=PERIOD_MS)
  if(continuity_debounce_step(&d,0U,now)&&!confirmed)confirmed=now;
 assert(confirmed==broke+window&&d.broken_since_ms==broke);
 /* An open wire at boot is confirmed after the window too. */
 continuity_debounce_init(&d);now=0;
 for(unsigned i=0;i<CONTINUITY_DEBOUNCE_SAMPLES-1U;i++,now+=PERIOD_MS)assert(!continuity_debounce_step(&d,0U,now));
 assert(continuity_debounce_step(&d,0U,now)&&d.broken_since_ms==1U);
 /* Reconnection needs N intact samples and clears the break time. */
 for(unsigned i=0;i<CONTINUITY_DEBOUNCE_SAMPLES-1U;i++)assert(continuity_debounce_step(&d,1U,now+=PERIOD_MS));
 assert(!continuity_debounce_step(&d,1U,now+=PERIOD_MS)&&d.broken_since_ms==0U);
 printf("window=%llu ms\n",(unsigned long long)window);
 return 0;
}
'''


class ContinuityDebounce(unittest.TestCase):
    def test_glitches_never_trip_and_breaks_trip_within_window(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / 'debounce'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-I', str(ROOT / 'Core/Inc'),
                            '-x', 'c', '-', '-o', str(binary)], input=HARNESS, text=True, check=True)
            subprocess.run([str(binary)], check=True, timeout=60)

    def test_open_wire_reads_the_broken_level(self):
        """An intact loop drives PB15 high (CONTINUITY_PRESENT_STATE SET); the
        internal pull-down makes an open or cut wire read low, i.e. broken."""
        main = (ROOT / 'Core/Src/main.c').read_text()
        block = main[main.index('GPIO_InitStruct.Pin = Continuity_Pin;'):]
        block = block[:block.index('HAL_GPIO_Init(Continuity_GPIO_Port')]
        self.assertIn('GPIO_InitStruct.Mode = GPIO_MODE_INPUT;', block)
        self.assertIn('GPIO_InitStruct.Pull = GPIO_PULLDOWN;', block)
        ioc = next(ROOT.glob('*.ioc')).read_text()
        self.assertIn('PB15.GPIO_PuPd=GPIO_PULLDOWN', ioc)
        self.assertRegex(ioc, r'PB15.GPIOParameters=[^\n]*GPIO_PuPd')
        safety = (ROOT / 'Core/Src/safety_thread.c').read_text()
        self.assertRegex(safety, r'#define CONTINUITY_PRESENT_STATE GPIO_PIN_SET')
        self.assertIn('continuity_debounce_step(&safety_continuity', safety)
        self.assertEqual(len(re.findall(r'HAL_GPIO_ReadPin\(Continuity_GPIO_Port', safety)), 2)


if __name__ == '__main__':
    unittest.main()
