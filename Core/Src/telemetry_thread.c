#include "board_watchdog.h"
// telemetry_thread.c
#include "VB-Threads.h"
#include "tx_api.h"
#include "telemetry.h"
#include "thread_comm.h"
#include "pressure_sd.h"
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
volatile uint32_t g_sim_pressure_sd_stack_remaining = 0U;

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
 * Retry only the current owner cause once per second; received Abort never echoes.
 * A lease spans SDK retain with no state lock and is drained before recovery. */
static void telemetry_report_safety_abort(void)
{
    static uint8_t attempted;
    static uint32_t attempted_epoch;
    static ULONG last_attempt;

    const ULONG now = tx_time_get();
    thread_comm_safety_report_t report_owner;
    if (!thread_comm_safety_report_claim(&report_owner)) return;
    if (attempted && attempted_epoch == report_owner.epoch &&
        (ULONG)(now - last_attempt) < TX_TIMER_TICKS_PER_SECOND) {
        thread_comm_safety_report_finish(report_owner.lease, 0U);return;
    }
    attempted = 1U;attempted_epoch = report_owner.epoch;last_attempt = now;
    const char *reason;
    switch (report_owner.reason) {
    case SAFETY_ABORT_HEARTBEAT: reason = "heartbeat timeout"; break;
    case SAFETY_ABORT_INITIAL_HEARTBEAT: reason = "initial heartbeat missing"; break;
    case SAFETY_ABORT_CONTINUITY: reason = "continuity lost"; break;
    case SAFETY_ABORT_LAUNCH_CONTINUITY: reason = "launch continuity timeout"; break;
    case SAFETY_ABORT_ADC_START: reason = "ADC start failed"; break;
    case SAFETY_ABORT_OVERPRESSURE: reason = "tank overpressure"; break;
    default: reason = "local"; break;
    }
    static const char prefix[] = "SEDS_AUTO_ABORT_V1:";
    char report[sizeof(prefix) + sizeof("initial heartbeat missing") - 1U];
    _Static_assert(sizeof(prefix) + sizeof("heartbeat timeout") - 1U <= sizeof(report), "automatic report buffer must hold every exact reason");
    _Static_assert(sizeof(prefix) + sizeof("initial heartbeat missing") - 1U <= sizeof(report), "automatic report buffer must hold every exact reason");
    _Static_assert(sizeof(prefix) + sizeof("continuity lost") - 1U <= sizeof(report), "automatic report buffer must hold every exact reason");
    _Static_assert(sizeof(prefix) + sizeof("launch continuity timeout") - 1U <= sizeof(report), "automatic report buffer must hold every exact reason");
    _Static_assert(sizeof(prefix) + sizeof("ADC start failed") - 1U <= sizeof(report), "automatic report buffer must hold every exact reason");
    _Static_assert(sizeof(prefix) + sizeof("tank overpressure") - 1U <= sizeof(report), "automatic report buffer must hold every exact reason");
    _Static_assert(sizeof(prefix) + sizeof("local") - 1U <= sizeof(report), "automatic report buffer must hold every exact reason");
    unsigned used = 0U;
    for (; used < sizeof(prefix) - 1U; ++used) report[used] = prefix[used];
    do { report[used++] = *reason; } while (*reason++ != '\0');
    const uint8_t retained = log_telemetry_string_asynchronous_at(SEDS_DT_ABORT, report,
                                              report_owner.original_utc) == SEDS_OK;
    thread_comm_safety_report_finish(report_owner.lease, retained);
}

void telemetry_thread_entry(ULONG initial_input)
{
    (void)ota_stream_register_owner();
    (void)initial_input;

    g_sim_main_stack_remaining = stack_remaining(&main_thread);
    g_sim_telemetry_stack_remaining = stack_remaining(&telemetry_thread);
    g_sim_data_acq_stack_remaining = stack_remaining(&data_acq_thread);
    g_sim_safety_stack_remaining = stack_remaining(&safety_thread);
    g_sim_pressure_sd_stack_remaining = pressure_sd_stack_remaining();

    (void)can_bus_init(&hfdcan2);
    // Ensure router exists early (so we can send requests immediately)
    (void)init_telemetry_router();

    for (;;) {
        board_watchdog_progress(BOARD_WATCHDOG_NETWORK);
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
    g_sim_pressure_sd_stack_remaining = pressure_sd_stack_remaining();

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
