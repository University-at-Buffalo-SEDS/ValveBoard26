#include "pressure_sd.h"
#include "pressure_sd_wire.h"
#include "pressure_sd_clock.h"
#include "board_uptime.h"
#include "telemetry.h"

/* The ADC runs at 500 Hz. Capture alternate conversions, four per half-ring,
 * with no formatting, floats, network calls, allocation or waits in the ISR. */
typedef struct { uint16_t raw[4]; uint32_t first_ms, first_sequence; } capture_t;
_Static_assert(sizeof(capture_t)==4*sizeof(ULONG), "pressure capture queue uses four words");
static TX_QUEUE queue;
static TX_THREAD worker;
static ULONG storage[16*4], stack[4608/sizeof(ULONG)];
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
static void capture(ADC_HandleTypeDef *adc, unsigned offset)
{
    if (!ready || !source || adc != source->hadc) return;
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
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *adc){capture(adc,0);}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc){capture(adc,8);}
/* Only a refused enqueue may be retried here. Once accepted, SEDSnet owns
 * delivery/receipt retries and preserves the packet identity. Keep the original
 * capture timestamp and bytes across admission attempts, with a bounded wait.
 * Sixteen capture batches cover 256 ms; stop retrying before that queue fills. */
static SedsResult publish_batch(const uint8_t *wire)
{
    const uint32_t started=HAL_GetTick();
    for(;;){
        const SedsResult result=log_telemetry_asynchronous(
            SEDS_DT_VALVE_PRESSURE_SD_V1,wire,PRESSURE_SD_BYTES,1U);
        if(result!=SEDS_IO || (uint32_t)(HAL_GetTick()-started)>=200U)return result;
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
        if(pressure_sd_encode(wire,sizeof(wire),&batch)<0 ||
           publish_batch(wire)!=SEDS_OK)
            g_pressure_sd_publish_drops+=4U;
        else g_pressure_sd_published_samples+=4U;
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
    source=sensor;
    UINT status=tx_queue_create(&queue,"PT SD capture",4,storage,sizeof(storage));
    if(status!=TX_SUCCESS)return status;
    status=tx_thread_create(&worker,"PT SD transmit",run,0,stack,sizeof(stack),7,7,TX_NO_TIME_SLICE,TX_AUTO_START);
    if(status!=TX_SUCCESS){tx_queue_delete(&queue);return status;}
    ready=1;return TX_SUCCESS;
}
