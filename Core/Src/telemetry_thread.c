// telemetry_thread.c
#include "VB-Threads.h"
#include "tx_api.h"
#include "telemetry.h"
#include "safety_diagnostics.h"
#include "ota_stream.h"
#include "can_bus.h"
#include "main.h"
#include <stdint.h>

TX_THREAD telemetry_thread;
extern TX_THREAD main_thread;
extern TX_THREAD data_acq_thread;
extern TX_THREAD safety_thread;
#define TELEMETRY_THREAD_STACK_SIZE (13U * 1024U)
#define TELEMETRY_QUEUE_SERVICE_BUDGET_MS 1U
extern FDCAN_HandleTypeDef hfdcan2;
static ULONG telemetry_thread_stack[TELEMETRY_THREAD_STACK_SIZE / sizeof(ULONG)];

volatile uint32_t g_sim_main_stack_remaining = 0U;
volatile uint32_t g_sim_telemetry_stack_remaining = 0U;
volatile uint32_t g_sim_data_acq_stack_remaining = 0U;
volatile uint32_t g_sim_safety_stack_remaining = 0U;

static uint32_t stack_remaining(const TX_THREAD *thread)
{
    if (thread == TX_NULL || thread->tx_thread_stack_start == TX_NULL ||
        thread->tx_thread_stack_end == TX_NULL ||
        thread->tx_thread_stack_highest_ptr == TX_NULL)
    {
        return 0U;
    }
    const uintptr_t start = (uintptr_t)thread->tx_thread_stack_start;
    const uintptr_t end = (uintptr_t)thread->tx_thread_stack_end;
    const uintptr_t high_water = (uintptr_t)thread->tx_thread_stack_highest_ptr;
    if (high_water < start || high_water > end)
    {
        return 0U;
    }
    return (uint32_t)(high_water - start);
}

/* Network-thread reporting keeps allocation and transport out of safety checks.
 * Retry queue rejection once per second; never rebroadcast a received abort. */
static void telemetry_report_safety_abort(void)
{
    static uint8_t sent, attempted;
    static ULONG last_attempt;
    if (sent || g_safety_first_abort.reason == SAFETY_ABORT_NONE) return;
    const ULONG now = tx_time_get();
    if (attempted && (ULONG)(now - last_attempt) < TX_TIMER_TICKS_PER_SECOND) return;
    attempted = 1U;
    last_attempt = now;
    const char *reason;
    switch (g_safety_first_abort.reason) {
    case SAFETY_ABORT_HEARTBEAT: reason = "Valve safety abort: heartbeat timeout"; break;
    case SAFETY_ABORT_INITIAL_HEARTBEAT: reason = "Valve safety abort: initial heartbeat missing"; break;
    case SAFETY_ABORT_CONTINUITY: reason = "Valve safety abort: continuity lost"; break;
    case SAFETY_ABORT_LAUNCH_CONTINUITY: reason = "Valve safety abort: launch continuity timeout"; break;
    case SAFETY_ABORT_ADC_START: reason = "Valve safety abort: ADC start failed"; break;
    default: reason = "Valve safety abort"; break;
    }
    if (log_telemetry_string_asynchronous(SEDS_DT_ABORT, reason) == SEDS_OK) sent = 1U;
}

void telemetry_thread_entry(ULONG initial_input)
{
    (void)initial_input;

    g_sim_main_stack_remaining = stack_remaining(&main_thread);
    g_sim_telemetry_stack_remaining = stack_remaining(&telemetry_thread);
    g_sim_data_acq_stack_remaining = stack_remaining(&data_acq_thread);
    g_sim_safety_stack_remaining = stack_remaining(&safety_thread);

    (void)can_bus_init(&hfdcan2);
    // Ensure router exists early (so we can send requests immediately)
    (void)init_telemetry_router();

    for (;;) {
        can_bus_process_rx();
        (void)telemetry_poll_discovery();
        (void)telemetry_poll_timesync();
        ota_stream_poll();
        (void)process_all_queues_timeout(TELEMETRY_QUEUE_SERVICE_BUDGET_MS);
        telemetry_report_safety_abort();

        g_sim_main_stack_remaining = stack_remaining(&main_thread);
        g_sim_telemetry_stack_remaining = stack_remaining(&telemetry_thread);
        g_sim_data_acq_stack_remaining = stack_remaining(&data_acq_thread);
        g_sim_safety_stack_remaining = stack_remaining(&safety_thread);

        tx_thread_sleep(1);
    }
}

UINT create_telemetry_thread(TX_BYTE_POOL *byte_pool)
{
    (void)byte_pool;

    UINT status = tx_thread_create(&telemetry_thread,
                                   "Telemetry Thread",
                                   telemetry_thread_entry,
                                   0,
                                   telemetry_thread_stack,
                                   TELEMETRY_THREAD_STACK_SIZE,
                                   5,
                                   5,
                                   TX_NO_TIME_SLICE,
                                   TX_AUTO_START);

    return status;
}
