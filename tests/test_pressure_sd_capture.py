"""Execute the actual ADC-half ISR capture; no synthetic 250 Hz polling."""
from pathlib import Path
import subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class CaptureTests(unittest.TestCase):
    def test_alternate_adc_samples_source_ticks_and_bounded_queue_drops(self):
        source=(ROOT/'Core/Src/pressure_sd.c').read_text()
        functions=source[source.index('static void capture('):source.index('/* Only a refused enqueue')]
        harness=r'''#include <stdint.h>
#include <assert.h>
#include <string.h>
#include "pressure_sd_clock.h"
typedef struct{unsigned remaining;}dma_t;
typedef struct{dma_t *DMA_Handle;}ADC_HandleTypeDef;
#define __HAL_DMA_GET_COUNTER(d) ((d)->remaining)
typedef struct{uint16_t raw[4];uint32_t first_ms,first_sequence;}capture_t;
typedef struct{ADC_HandleTypeDef *hadc;uint16_t dma_samples[16];}pressure_transducer_t;
static pressure_transducer_t device,*source=&device;
static uint8_t ready=1;static uint32_t sequence,queue,tick,last_first_ms,last_sequence,last_observed_first_ms;
static unsigned last_offset;
static uint8_t captured_once;
static unsigned g_pressure_sd_captured_samples,g_pressure_sd_capture_drops,g_pressure_sd_late_half_drops;
static pressure_sd_clock_t sample_clock;static uint64_t received_ticks;
static capture_t received;static unsigned sends;static int full;
#define TX_NO_WAIT 0
#define TX_SUCCESS 0
unsigned HAL_GetTick(void){return tick;}
int tx_queue_send(void *q,void *message,int wait){(void)q;assert(!wait);++sends;if(full)return 1;memcpy(&received,message,sizeof(received));received_ticks=pressure_sd_clock_next(&sample_clock,received.first_sequence,received.first_ms);return 0;}
''' + functions + r'''
int main(void){
 (void)last_first_ms;
 dma_t dma={8};ADC_HandleTypeDef adc={&dma},other={&dma};source->hadc=&adc;
 for(unsigned i=0;i<16;++i)source->dma_samples[i]=100+i;
 tick=16;HAL_ADC_ConvHalfCpltCallback(&other);assert(!sends);
 HAL_ADC_ConvHalfCpltCallback(&adc);
 assert(sends==1 && received_ticks==2 && received.first_sequence==0);
 for(unsigned i=0;i<4;++i)assert(received.raw[i]==100+2*i);
 dma.remaining=16;tick=32;HAL_ADC_ConvCpltCallback(&adc);
 assert(sends==2 && received_ticks==18 && received.first_sequence==4);
 for(unsigned i=0;i<4;++i)assert(received.raw[i]==108+2*i);
 dma.remaining=8;full=1;tick=48;HAL_ADC_ConvHalfCpltCallback(&adc);
 assert(g_pressure_sd_capture_drops==4 && g_pressure_sd_captured_samples==12);
 dma.remaining=16;full=0;tick=64;HAL_ADC_ConvCpltCallback(&adc);
 assert(received.first_sequence==12 && received_ticks==50);
 /* Late half interrupt: DMA is already overwriting that half. */
 dma.remaining=14;tick=68;HAL_ADC_ConvHalfCpltCallback(&adc);
 assert(sends==4 && g_pressure_sd_late_half_drops==4);
 /* Lost whole DMA periods appear as source-sequence gaps, not invented data. */
 dma.remaining=8;tick=112;HAL_ADC_ConvHalfCpltCallback(&adc);
 assert(sends==5 && received_ticks==98 && received.first_sequence==24);
 HAL_ADC_ConvHalfCpltCallback(&adc);assert(sends==5);
 ready=0;HAL_ADC_ConvHalfCpltCallback(&adc);assert(sends==5);
 ready=1;
 /* Sub-half IRQ/tick jitter must not move samples off their four-sample
  * acquisition grid; the exact prior overlap (3431 twice) must fail. Real DMA data remains complete in both halves. */
 for(unsigned n=0;n<128;++n){
   const uint32_t expected=114U+16U*n;
   tick=expected+14U+(n%3U==0U?7U:n%3U==1U?-7U:0U);
   dma.remaining=n%2U==0U?16U:8U;
   const unsigned before=sends;
   if(n%2U==0U)HAL_ADC_ConvCpltCallback(&adc);
   else HAL_ADC_ConvHalfCpltCallback(&adc);
   assert(sends==before+1U);
   assert(received_ticks==expected && received.first_sequence==28U+4U*n);
   if(n%2U==0U)HAL_ADC_ConvCpltCallback(&adc);
   else HAL_ADC_ConvHalfCpltCallback(&adc);
   assert(sends==before+1U);
 }
 /* Accumulated HAL interrupt-time drift must not discard alternating
  * DMA completions. Each observed interval remains within one tick of16ms. */
 const uint32_t grid=(uint32_t)received_ticks+16U;
 const uint32_t next_sequence=received.first_sequence+4U;
 const uint32_t observed=last_observed_first_ms+16U;
 for(unsigned n=0;n<512;++n){
   tick=observed+16U*n-n/8U+14U;
   dma.remaining=n%2U==0U?16U:8U;
   const unsigned before=sends;
   if(n%2U==0U)HAL_ADC_ConvCpltCallback(&adc);
   else HAL_ADC_ConvHalfCpltCallback(&adc);
   assert(sends==before+1U);
   assert(received_ticks==grid+16U*n);
   assert((uint32_t)(tick-received.first_ms)==14U); /* UTC uses observed HAL age. */
   assert(received.first_sequence==next_sequence+4U*n);
 }
 /* Preserve wraparound, including the source sequence wrapping to zero. */
 last_observed_first_ms=UINT32_MAX-13U;last_sequence=UINT32_MAX-3U;last_offset=8U;
 sample_clock.ticks=UINT32_MAX-13U;sample_clock.sequence=UINT32_MAX-3U;sample_clock.initialized=1U;
 dma.remaining=8;tick=16;
 HAL_ADC_ConvHalfCpltCallback(&adc);
 assert(received_ticks==(uint64_t)UINT32_MAX+3U && received.first_sequence==0U);

 return 0;
}'''
        with tempfile.TemporaryDirectory() as d:
            exe=Path(d)/'capture'
            subprocess.run(['cc','-I',str(ROOT/'Core/Inc'),'-std=c11','-O2','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',str(exe)],input=harness,text=True,check=True)
            subprocess.run([str(exe)],check=True)
