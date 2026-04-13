// main_thread.c
#include "VB-Threads.h"
#include "tx_api.h"
#include "telemetry.h"
#include "can_bus.h"
#include "main.h"
#include "thread_comm.h"
#include "ltc2990.h"
#include "solenoid_driver.h"


extern TX_QUEUE tx_queue; 
extern TX_EVENT_FLAGS_GROUP event_flags;
extern LTC2990_Handle_t ltc2990_dev;

typedef struct {
    float voltages[4];
    float differential;
    uint32_t pressure;
    uint32_t timestamp;
    uint8_t abort_triggered;
} valveBoardPayload_t;

TX_THREAD main_thread;
#define MAIN_THREAD_STACK_SIZE (16U *1024U)

static void read_sensors(valveBoardPayload_t *payload);
static void execute_commands(void);
static void publish_board_data(valveBoardPayload_t *payload);
static void emit_umbilical_status(void);
void main_thread_entry(ULONG initial_input)
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

UINT create_main_thread(TX_BYTE_POOL *byte_pool)
{

        CHAR *pointer;

  /* Allocate the stack for test  */
  if (tx_byte_allocate(byte_pool, (VOID**) &pointer,
                       MAIN_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

    UINT status = tx_thread_create(&main_thread,
                                   "Main Thread",
                                   main_thread_entry,
                                   0,
                                   pointer,
                                   MAIN_THREAD_STACK_SIZE,
                                   6,
                                   6,
                                   TX_NO_TIME_SLICE,
                                   TX_AUTO_START);

    return status;
}
