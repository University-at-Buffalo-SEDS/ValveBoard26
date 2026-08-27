// telemetry_thread.c
#include "VB-Threads.h"
#include "tx_api.h"
#include "telemetry.h"
#include "ota_stream.h"
#include "can_bus.h"
#include "main.h"

TX_THREAD telemetry_thread;
#define TELEMETRY_THREAD_STACK_SIZE (12U * 1024U)
extern FDCAN_HandleTypeDef hfdcan2;
static ULONG telemetry_thread_stack[TELEMETRY_THREAD_STACK_SIZE / sizeof(ULONG)];

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
        (void)dispatch_tx_queue_timeout(0);

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
