// safety_thread.c
#include "VB-Threads.h"
#include "tx_api.h"
#include "telemetry.h"
#include "can_bus.h"
#include "main.h"

TX_THREAD safety_thread;
#define SAFETY_THREAD_STACK_SIZE (16U *1024U)

void safety_thread_entry(ULONG initial_input)
{
    (void)initial_input;

    // Ensure router exists early (so we can send requests immediately)
    (void)init_telemetry_router();

    for (;;) {
        can_bus_process_rx();
        (void)telemetry_poll_discovery();
        (void)process_all_queues_timeout(50);
        (void)telemetry_poll_timesync();

        tx_thread_sleep(1);
    }
}


UINT create_safety_thread(TX_BYTE_POOL *byte_pool)
{

        CHAR *pointer;

  /* Allocate the stack for test  */
  if (tx_byte_allocate(byte_pool, (VOID**) &pointer,
                       SAFETY_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

    UINT status = tx_thread_create(&safety_thread,
                                   "Safety Thread",
                                   safety_thread_entry,
                                   0,
                                   pointer,
                                   SAFETY_THREAD_STACK_SIZE,
                                   5,
                                   5,
                                   TX_NO_TIME_SLICE,
                                   TX_AUTO_START);

    return status;
}
