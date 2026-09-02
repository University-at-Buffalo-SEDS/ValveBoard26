// telemetry_thread.c
#include "VB-Threads.h"
#include "tx_api.h"
#include "telemetry.h"
#include "ota_stream.h"
#include "can_bus.h"
#include "main.h"
#include <stdint.h>

TX_THREAD telemetry_thread;
extern TX_THREAD main_thread;
extern TX_THREAD data_acq_thread;
extern TX_THREAD safety_thread;
#define TELEMETRY_THREAD_STACK_SIZE (10U * 1024U)
extern FDCAN_HandleTypeDef hfdcan2;
static ULONG telemetry_thread_stack[TELEMETRY_THREAD_STACK_SIZE / sizeof(ULONG)];

volatile uint32_t g_sim_main_stack_remaining = 0U;
volatile uint32_t g_sim_telemetry_stack_remaining = 0U;
volatile uint32_t g_sim_data_acq_stack_remaining = 0U;
volatile uint32_t g_sim_safety_stack_remaining = 0U;

static uint32_t stack_remaining(const TX_THREAD *thread)
{
    if (thread == TX_NULL || thread->tx_thread_stack_start == TX_NULL ||
        thread->tx_thread_stack_highest_ptr == TX_NULL)
    {
        return 0U;
    }
    return (uint32_t)((uintptr_t)thread->tx_thread_stack_highest_ptr -
                      (uintptr_t)thread->tx_thread_stack_start);
}

void telemetry_thread_entry(ULONG initial_input)
{
    (void)initial_input;

    (void)can_bus_init(&hfdcan2);
    // Ensure router exists early (so we can send requests immediately)
    (void)init_telemetry_router();

    for (;;) {
        can_bus_process_rx();
        (void)process_rx_queue_timeout(0);
        (void)telemetry_poll_discovery();
        (void)telemetry_poll_timesync();
        ota_stream_poll();
        (void)dispatch_tx_queue_timeout(50);

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
