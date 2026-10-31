/* Actual producer/worker under a finite scheduler/API fault model.
 * Real SDK admission is independently qualified by the external integration;
 * this persistent component test does not pretend its refusal stub is SDK. */
#include <stdlib.h>
#include "pressure_sd.h"
#include "telemetry.h"
#include "board_uptime.h"
#include "driver-body.h"
#include "pressure_sd.c"
/* ACTUAL_CONVERSION_FUNCTION */
static uint64_t release_ms;
static unsigned terminal_fault,terminal_sent,calibration_change,changed;
static uint8_t seen[128],accepted[128],wire_copy[128][44];
int witness_sdk_attempt(const unsigned char*wire,size_t n){
 assert(n==44);pressure_sd_batch_t decoded;assert(pressure_sd_decode(&decoded,wire,n)==0);
 const unsigned index=decoded.first_sequence/4;assert(index<128&&!accepted[index]);
 if(seen[index])assert(!memcmp(wire_copy[index],wire,44));else{memcpy(wire_copy[index],wire,44);seen[index]=1;}
 if(clock64<release_ms)return SEDS_IO;
 if(terminal_fault&&!terminal_sent){terminal_sent=1;return SEDS_ERR;}
 accepted[index]=1;return SEDS_OK;
}
void witness_sdk_queue(uint32_t*out){memset(out,0,16*sizeof(*out));}
void witness_sdk_tick(uint64_t now){(void)now;}
void witness_sdk_release(uint64_t at){release_ms=at;}
static void advance(uint64_t until){
 while(clock64<until){++clock64;
  if(clock64==next_capture&&clock64<=capture_end){
   for(unsigned i=0;i<16;++i)sensor.dma_samples[i]=(uint16_t)(100U+(capture_index%100U)+i);
   dma.remaining=capture_index%2U?16U:8U;
   if(capture_index%2U)HAL_ADC_ConvCpltCallback(&adc);else HAL_ADC_ConvHalfCpltCallback(&adc);
   ++capture_index;next_capture+=16U;
  }
  if(g_pressure_sd_retained_samples&&g_pressure_sd_deferral_windows){
   pressure_transducer_t other=sensor;
   assert(pressure_sd_init(&other)==TX_QUEUE_ERROR&&source==&sensor);
   if(calibration_change&&!changed){sensor.max_pressure=1000;changed=1;}
  }
  if(clock64>run_end)longjmp(stopped,2);
 }
}
int main(int argc,char**argv){
 assert(argc==7);unsigned prefill=(unsigned)strtoul(argv[1],0,10),phase=(unsigned)strtoul(argv[2],0,10),blackout=(unsigned)strtoul(argv[3],0,10);clock64=strtoul(argv[4],0,10)?UINT32_MAX-32U:1000U;terminal_fault=(unsigned)strtoul(argv[5],0,10);calibration_change=(unsigned)strtoul(argv[6],0,10);
 const uint64_t start=clock64;next_capture=start+16+phase;capture_end=start+1200;run_end=capture_end+3000;
 assert(pressure_sd_init(0)==TX_PTR_ERROR);assert(pressure_sd_init(&sensor)==TX_SUCCESS);assert(queue.capacity==32);
 if(prefill){advance(next_capture+16U*(prefill-1U));}
 release_ms=clock64+blackout;
 const int ended=setjmp(stopped);if(!ended)run(0);
 assert(g_pressure_sd_captured_samples==g_pressure_sd_published_samples+g_pressure_sd_capture_drops+g_pressure_sd_publish_drops+queue.tx_queue_enqueued*4+g_pressure_sd_retained_samples);
 assert(!g_pressure_sd_late_half_drops);
 unsigned count=0;for(unsigned i=0;i<128;++i)count+=accepted[i];assert(count*4==g_pressure_sd_published_samples);
 if(blackout<=402){assert(!g_pressure_sd_capture_drops);assert(g_pressure_sd_publish_drops==(terminal_fault?4U:0U));assert(g_pressure_sd_captured_samples==g_pressure_sd_published_samples+g_pressure_sd_publish_drops);assert(!g_pressure_sd_retained_samples&&!queue.tx_queue_enqueued);assert(g_pressure_sd_deferred_batches==1&&g_pressure_sd_deferral_windows>=1);}
 else{assert(g_pressure_sd_capture_drops>0);if(blackout>4200)assert(ended==2&&g_pressure_sd_retained_samples==4);}
 if(calibration_change){assert(changed);pressure_sd_batch_t first;assert(!pressure_sd_decode(&first,wire_copy[0],44));assert(first.psi[0]>60);}
 printf("captured=%u published=%u captureDrop=%u publishDrop=%u deferred=%u windows=%u high=%u pending=%u\n",g_pressure_sd_captured_samples,g_pressure_sd_published_samples,g_pressure_sd_capture_drops,g_pressure_sd_publish_drops,g_pressure_sd_deferred_batches,g_pressure_sd_deferral_windows,highwater,g_pressure_sd_retained_samples);
 return 0;
}
