#include <assert.h>
#include <string.h>
#include <setjmp.h>
#include <stdio.h>
static uint64_t clock64,next_capture,capture_end,run_end;
static jmp_buf stopped;
static DMA_HandleTypeDef dma;
static ADC_HandleTypeDef adc={&dma};
static pressure_transducer_t sensor={.hadc=&adc,.min_adc=0,.max_adc=3103,.min_pressure=0,.max_pressure=2000};
static unsigned capture_index,highwater,sdk_calls;
extern int witness_sdk_attempt(const unsigned char*,size_t);
extern void witness_sdk_queue(uint32_t*);
extern void witness_sdk_tick(uint64_t);
extern void witness_sdk_release(uint64_t);
static void advance(uint64_t until);
uint32_t HAL_GetTick(void){return(uint32_t)clock64;}
uint64_t board_uptime_ticks(void){return clock64;}
static uint64_t telemetry_unix_ms(void){return UINT64_C(1767225600000)+clock64;}
SedsResult seds_get_runtime_tuning_config(SedsRuntimeTuningConfig*out){out->reliable_max_end_to_end_pending=8;return SEDS_OK;}
int seds_router_copy_queue_counters(SedsRouter*r,uint32_t*out,size_t n){assert(r&&n==16);witness_sdk_queue(out);return 16;}
static int init_telemetry_router(void){return SEDS_OK;}
static int log_telemetry_asynchronous(unsigned ty,const void*wire,size_t n,size_t width){assert(ty==157797238&&n==44&&width==1);++sdk_calls;return witness_sdk_attempt(wire,n);}
static UINT tx_queue_create(TX_QUEUE*q,const char*name,unsigned words,void*buffer,size_t size){(void)name;assert(words==4);q->buffer=buffer;q->capacity=size/16;return TX_SUCCESS;}
static UINT tx_queue_send(TX_QUEUE*q,void*item,unsigned wait){assert(wait==TX_NO_WAIT);if(q->tx_queue_enqueued==q->capacity)return TX_QUEUE_ERROR;memcpy((uint8_t*)q->buffer+q->tail*16,item,16);q->tail=(q->tail+1)%q->capacity;++q->tx_queue_enqueued;if(q->tx_queue_enqueued>highwater)highwater=q->tx_queue_enqueued;return TX_SUCCESS;}
static UINT tx_queue_receive(TX_QUEUE*q,void*out,unsigned wait){assert(wait==TX_WAIT_FOREVER);while(!q->tx_queue_enqueued){if(clock64>=capture_end)longjmp(stopped,1);advance(clock64+1);}memcpy(out,(uint8_t*)q->buffer+q->head*16,16);q->head=(q->head+1)%q->capacity;--q->tx_queue_enqueued;return TX_SUCCESS;}
static UINT tx_queue_delete(TX_QUEUE*q){(void)q;return TX_SUCCESS;}
static UINT tx_thread_create(TX_THREAD*t,const char*n,void(*entry)(ULONG),ULONG arg,void*stack,size_t size,unsigned p,unsigned threshold,unsigned slice,unsigned start){(void)n;(void)entry;(void)arg;assert(p==7&&threshold==7&&slice==0&&start==1);t->tx_thread_stack_start=stack;t->tx_thread_stack_end=(uint8_t*)stack+size;t->tx_thread_stack_highest_ptr=stack;return TX_SUCCESS;}
static void tx_thread_sleep(unsigned ticks){assert(ticks==1);advance(clock64+ticks);}
