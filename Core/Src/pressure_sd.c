#include "pressure_sd.h"
#include "pressure_sd_wire.h"
#include "pressure_sd_clock.h"
#include "pressure_sd_admission.h"
#include "board_uptime.h"
#include "telemetry.h"

/* The ADC runs at 500 Hz. Capture alternate conversions, four per half-ring,
 * with no formatting, floats, network calls, allocation or waits in the ISR. */
typedef struct { uint16_t raw[4]; uint32_t first_ms, first_sequence; } capture_t;
_Static_assert(sizeof(capture_t)==4*sizeof(ULONG), "pressure capture queue uses four words");
static TX_QUEUE queue;
static TX_THREAD worker;
/* Fair queue admission can reclaim routing metadata during a publication.
 * The three-radio profile exceeds the former 4.5 KiB producer stack. */
#define PRESSURE_SD_THREAD_STACK_SIZE ((6U * 1024U) + 512U)
/* Three prefilled batches plus two observed-size201ms service gaps require
 * 3+ceil(402/16)=29 slots.32 is a finite deficit envelope, not an unlimited
 * congestion guarantee. ISR refusal remains a real counted loss. */
#define PRESSURE_SD_CAPTURE_BATCHES 32U
_Static_assert(PRESSURE_SD_CAPTURE_BATCHES == 32U, "Capture retention bound");
static ULONG storage[PRESSURE_SD_CAPTURE_BATCHES*4U], stack[PRESSURE_SD_THREAD_STACK_SIZE/sizeof(ULONG)];
static pressure_transducer_t *source;
static uint32_t sequence, last_sequence, last_observed_first_ms;
static pressure_sd_clock_t source_sample_clock;
static unsigned last_offset;
static uint8_t captured_once;
static uint8_t ready;
volatile uint32_t g_pressure_sd_captured_samples;
volatile uint32_t g_pressure_sd_capture_drops;
volatile uint32_t g_pressure_sd_late_half_drops;
volatile uint32_t g_pressure_sd_published_samples;
volatile uint32_t g_pressure_sd_publish_drops;
volatile uint32_t g_pressure_sd_admission_retries;
/* Captured owner remains local until one successful SDK enqueue. A failed
 * attempt window is not a discard while this fixed owner still owns its data. */
volatile uint32_t g_pressure_sd_retained_samples;
volatile uint32_t g_pressure_sd_deferred_batches;
volatile uint32_t g_pressure_sd_deferral_windows;
volatile uint32_t g_pressure_sd_max_retained_age_ms;

#ifdef SEDS_FIRMWARE_SIM_TEST
/* Passive simulator-only observations; no queue, wait or publication policy. */
volatile uint32_t g_pressure_sd_capture_queue_high_water;
volatile uint32_t g_pressure_sd_max_dequeue_gap_ms;
volatile uint32_t g_pressure_sd_encode_failures;
volatile int32_t g_pressure_sd_last_publish_status;
volatile int32_t g_pressure_sd_last_non_ok_publish_status;
volatile uint32_t g_pressure_sd_non_ok_publications;
volatile uint32_t g_pressure_sd_retry_deadline_exits;
volatile uint32_t g_pressure_sd_max_sdk_call_ms;
volatile uint32_t g_pressure_sd_sdk_call_stage;
volatile uint32_t g_pressure_sd_max_sdk_call_stage;
volatile uint32_t g_pressure_sd_last_non_ok_stage;
volatile uint32_t g_pressure_sd_archive_refusals;
volatile uint32_t g_pressure_sd_enqueue_refusals;
static uint32_t observed_dequeue_ms;
static uint8_t observed_dequeue_once;
static void observe_sdk_call(uint32_t started, uint32_t stage, SedsResult result)
{
    const uint32_t elapsed=HAL_GetTick()-started;
    if(elapsed>g_pressure_sd_max_sdk_call_ms){
        g_pressure_sd_max_sdk_call_ms=elapsed;
        g_pressure_sd_max_sdk_call_stage=stage;
    }
    if(result!=SEDS_OK)g_pressure_sd_last_non_ok_stage=stage;
    if(result==SEDS_IO && stage==2U)++g_pressure_sd_archive_refusals;
    if(result==SEDS_IO && stage==3U)++g_pressure_sd_enqueue_refusals;
    g_pressure_sd_sdk_call_stage=0U;
}
#endif

static void capture(ADC_HandleTypeDef *adc, unsigned offset)
{
    if (!ready || !source || adc != source->hadc) return;
    if (!pressureTransducerSettled(HAL_GetTick())) return; /* Power-up settling. */
    const uint32_t remaining=__HAL_DMA_GET_COUNTER(adc->DMA_Handle);
    const uint32_t phase=(16U-(remaining%16U))%16U;
    /* A delayed IRQ may arrive after DMA began overwriting that half. Do not
     * publish mixed old/new conversions. The other completed half is intact. */
    if ((offset==0U && phase<8U) || (offset==8U && phase>=8U)) {
        g_pressure_sd_late_half_drops+=4U;
        if(!captured_once)sequence+=4U;
        return;
    }
    capture_t item;
    const uint32_t age_samples=offset==0U?phase-8U:phase;
    item.first_ms=HAL_GetTick()-14U-age_samples*2U;
    const uint32_t observed_first_ms=item.first_ms;
    if(captured_once){
        const uint32_t elapsed=item.first_ms-last_observed_first_ms;
        if(elapsed>=UINT32_C(0x80000000))return;
        const uint32_t alternate=offset!=last_offset;
        if(!alternate && elapsed<24U)return; /* One half cannot complete twice in16ms. */
        /* Compare observed clock intervals, never a timestamp already projected
         * onto the ADC grid. Accumulated HAL tick drift must not discard a real
         * DMA completion. Alternating identities imply an odd number of halves;
         * the same identity implies an even number. Preserve genuine gaps. */
        const uint32_t halves=alternate?(elapsed/32U)*2U+1U:
            ((elapsed+16U)/32U)*2U;
        const uint32_t samples=halves*4U;
        item.first_sequence=last_sequence+samples;
    }else item.first_sequence=sequence;
    for (unsigned i=0;i<4;++i) item.raw[i]=source->dma_samples[offset+2*i];
    last_observed_first_ms=observed_first_ms;last_offset=offset;
    last_sequence=item.first_sequence;
    captured_once=1;sequence=item.first_sequence+4U;
    g_pressure_sd_captured_samples+=4U;
    if(tx_queue_send(&queue,&item,TX_NO_WAIT)!=TX_SUCCESS)g_pressure_sd_capture_drops+=4U;
#ifdef SEDS_FIRMWARE_SIM_TEST
    /* ISR owns execution here; a newly awakened worker runs after ISR exit. */
    if(queue.tx_queue_enqueued>g_pressure_sd_capture_queue_high_water)
        g_pressure_sd_capture_queue_high_water=queue.tx_queue_enqueued;
#endif
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *adc){capture(adc,0);}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc){capture(adc,8);}
/* Only a refused enqueue may be retried here. Once accepted, SEDSnet owns
 * delivery/receipt retries and preserves the packet identity. Keep the original
 * capture timestamp and bytes across admission attempts, with a bounded wait.
 * This call retains its original200ms attempt budget. Its caller retains the
 * same prepared owner after IO budget exhaustion; no capture clock is renewed.
 * The fixed capture FIFO can still overflow during prolonged congestion. */
static SedsResult publish_batch(const uint8_t *wire)
{
    const uint32_t started=HAL_GetTick();
    for(;;){
        SedsResult result=SEDS_OK;
        if (g_router.r == NULL) {
#ifdef SEDS_FIRMWARE_SIM_TEST
            const uint32_t before=HAL_GetTick();g_pressure_sd_sdk_call_stage=1U;
#endif
            result=init_telemetry_router();
#ifdef SEDS_FIRMWARE_SIM_TEST
            observe_sdk_call(before,1U,result);
#endif
        }
        if (result==SEDS_OK) {
#ifdef SEDS_FIRMWARE_SIM_TEST
            const uint32_t before=HAL_GetTick();g_pressure_sd_sdk_call_stage=2U;
#endif
            result=pressure_sd_archive_admission(g_router.r);
#ifdef SEDS_FIRMWARE_SIM_TEST
            observe_sdk_call(before,2U,result);
#endif
        }
        if (result==SEDS_OK) {
#ifdef SEDS_FIRMWARE_SIM_TEST
            const uint32_t before=HAL_GetTick();g_pressure_sd_sdk_call_stage=3U;
#endif
            result=log_telemetry_asynchronous(
                SEDS_DT_VALVE_PRESSURE_SD_V1,wire,PRESSURE_SD_BYTES,1U);
#ifdef SEDS_FIRMWARE_SIM_TEST
            observe_sdk_call(before,3U,result);
#endif
        }
        if(result!=SEDS_IO || (uint32_t)(HAL_GetTick()-started)>=200U){
#ifdef SEDS_FIRMWARE_SIM_TEST
            if(result==SEDS_IO)++g_pressure_sd_retry_deadline_exits;
            g_pressure_sd_last_publish_status=result;
            if(result!=SEDS_OK){
                g_pressure_sd_last_non_ok_publish_status=result;
                ++g_pressure_sd_non_ok_publications;
            }
#endif
            return result;
        }
        ++g_pressure_sd_admission_retries;
        tx_thread_sleep((TX_TIMER_TICKS_PER_SECOND+999U)/1000U);
    }
}
static void run(ULONG unused)
{
    (void)unused;
    for(;;){
        capture_t item;
        if(tx_queue_receive(&queue,&item,TX_WAIT_FOREVER)!=TX_SUCCESS)continue;
#ifdef SEDS_FIRMWARE_SIM_TEST
        const uint32_t dequeued_ms=HAL_GetTick();
        if(observed_dequeue_once){
            const uint32_t gap=dequeued_ms-observed_dequeue_ms;
            if(gap>g_pressure_sd_max_dequeue_gap_ms)g_pressure_sd_max_dequeue_gap_ms=gap;
        }
        observed_dequeue_ms=dequeued_ms;observed_dequeue_once=1U;
#endif
        pressure_sd_batch_t batch={0};uint8_t wire[PRESSURE_SD_BYTES];
        /* Preserve both clocks without adding capture/wire bytes. Sequence
         * advances the ADC grid; the observed HAL capture time projects UTC.
         * Projecting a drifting sample grid through HAL can otherwise yield
         * zero uptime or an unsigned age underflow after a long acquisition. */
        const uint64_t now=board_uptime_ticks();
        const uint64_t utc=telemetry_unix_ms();
        pressure_sd_stamp_clock(&source_sample_clock,&batch,item.first_sequence,
            item.first_ms,now,utc);
        for(unsigned i=0;i<4;++i){batch.raw[i]=item.raw[i];batch.psi[i]=pressureTransducerConvertRaw(source,item.raw[i]);}
        const int encoded=pressure_sd_encode(wire,sizeof(wire),&batch);
#ifdef SEDS_FIRMWARE_SIM_TEST
        if(encoded<0)++g_pressure_sd_encode_failures;
#endif
        if(encoded<0){g_pressure_sd_publish_drops+=4U;continue;}
        const uint32_t prepared_ms=HAL_GetTick();
        uint8_t deferred=0U;
        g_pressure_sd_retained_samples=PRESSURE_SD_SAMPLES;
        for(;;){
            const SedsResult result=publish_batch(wire);
            const uint32_t age=HAL_GetTick()-prepared_ms;
            if(age>g_pressure_sd_max_retained_age_ms)g_pressure_sd_max_retained_age_ms=age;
            if(result==SEDS_OK){g_pressure_sd_published_samples+=4U;break;}
            if(result!=SEDS_IO){g_pressure_sd_publish_drops+=4U;break;}
            ++g_pressure_sd_deferral_windows;
            if(!deferred){++g_pressure_sd_deferred_batches;deferred=1U;}
            /* No accepted parent is retried here. Every IO attempt refused
             * local ownership; original44B clocks/values/sequence stay fixed. */
            tx_thread_sleep((TX_TIMER_TICKS_PER_SECOND+999U)/1000U);
        }
        g_pressure_sd_retained_samples=0U;
    }
}
uint32_t pressure_sd_stack_remaining(void)
{
    const uintptr_t start = (uintptr_t)worker.tx_thread_stack_start;
    const uintptr_t end = (uintptr_t)worker.tx_thread_stack_end;
    const uintptr_t highest = (uintptr_t)worker.tx_thread_stack_highest_ptr;
    if (start == 0U || highest < start || highest > end) return 0U;
    return (uint32_t)(highest - start);
}

UINT pressure_sd_init(pressure_transducer_t *sensor)
{
    if(!sensor || !sensor->hadc || !sensor->hadc->DMA_Handle || PRESSURE_TRANSDUCER_DMA_BUFFER_LEN!=16U)return TX_PTR_ERROR;
    /* Runtime reinitialization must not replace a live retained owner/source. */
    if(ready)return TX_QUEUE_ERROR;
    source=sensor;
    UINT status=tx_queue_create(&queue,"PT SD capture",4,storage,sizeof(storage));
    if(status!=TX_SUCCESS)return status;
    status=tx_thread_create(&worker,"PT SD transmit",run,0,stack,sizeof(stack),7,7,TX_NO_TIME_SLICE,TX_AUTO_START);
    if(status!=TX_SUCCESS){tx_queue_delete(&queue);return status;}
    ready=1;return TX_SUCCESS;
}
