#include "VB-Threads.h"
#include "tx_api.h"
#include "thread_comm.h"
#include "main.h"
#include "telemetry.h"

#include <stdint.h>

TX_THREAD main_task_thread;
#define MAIN_TASK_STACK_SIZE (4U * 1024U)
#define UMBILICAL_STATUS_PERIOD_TICKS TX_TIMER_TICKS_PER_SECOND

static uint8_t g_aborted = 0U;
static uint8_t g_pilot_open = 0U;

static void publish_all_umbilical_statuses(void)
{
    (void)telemetry_publish_umbilical_status(CMD_PILOT_OPEN, g_pilot_open);
}

static void handle_command(thread_comm_msg_t cmd)
{
    HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);

    switch (cmd)
    {
    case CMD_PILOT_OPEN:
        HAL_GPIO_WritePin(Solenoid_GPIO_Port, Solenoid_Pin, GPIO_PIN_SET);
        g_pilot_open = 1U;

        (void)telemetry_publish_umbilical_status(CMD_PILOT_OPEN, g_pilot_open);
        break;

    case CMD_PILOT_CLOSE:
        HAL_GPIO_WritePin(Solenoid_GPIO_Port, Solenoid_Pin, GPIO_PIN_RESET);
        g_pilot_open = 0U;
        (void)telemetry_publish_umbilical_status(CMD_PILOT_OPEN, g_pilot_open);
        break;

    default:
        break;
    }
}

void main_task_entry(ULONG initial_input)
{
    (void)initial_input;
    HAL_GPIO_WritePin(Solenoid_GPIO_Port, Solenoid_Pin, GPIO_PIN_RESET);

    thread_comm_msg_t msg;
    ULONG last_umbilical_status_ticks = tx_time_get();

    publish_all_umbilical_statuses();

    for (;;)
    {
        if (thread_comm_get_abort() != 0U)
        {
            if (g_aborted == 0U)
            {
                HAL_GPIO_WritePin(Solenoid_GPIO_Port, Solenoid_Pin, GPIO_PIN_RESET);
                g_pilot_open = 0U;
                (void)telemetry_publish_umbilical_status(CMD_PILOT_OPEN, g_pilot_open);
                g_aborted = 1U;
            }

            while (thread_comm_receive(&msg, TX_NO_WAIT) == TX_SUCCESS)
            {
                // Dump any pending messages bc we ABORTING!!!!!!!
            }

            tx_thread_sleep(10);
            continue;
        }

        while (thread_comm_receive(&msg, TX_NO_WAIT) == TX_SUCCESS)
        {
            handle_command(msg);
        }

        if ((ULONG)(tx_time_get() - last_umbilical_status_ticks) >= UMBILICAL_STATUS_PERIOD_TICKS)
        {
            publish_all_umbilical_statuses();
            last_umbilical_status_ticks = tx_time_get();
        }

        tx_thread_sleep(1);
    }
}

UINT create_main_task(TX_BYTE_POOL *byte_pool)
{
    CHAR *pointer;

    if (tx_byte_allocate(byte_pool, (VOID **)&pointer,
                         MAIN_TASK_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
    {
        return TX_POOL_ERROR;
    }

    return tx_thread_create(&main_task_thread,
                            "Main Task",
                            main_task_entry,
                            0,
                            pointer,
                            MAIN_TASK_STACK_SIZE,
                            6,
                            6,
                            TX_NO_TIME_SLICE,
                            TX_AUTO_START);
}
