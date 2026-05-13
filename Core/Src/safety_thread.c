// safety_thread.c
#include "VB-Threads.h"

#include "main.h"
#include "telemetry.h"
#include "thread_comm.h"
#include "tx_api.h"

TX_THREAD safety_thread;

#define SAFETY_THREAD_STACK_SIZE (4U * 1024U)
#define SAFETY_HEARTBEAT_TIMEOUT_MS (5000ULL)
#define SAFETY_CHECK_PERIOD_TICKS ((TX_TIMER_TICKS_PER_SECOND + 19U) / 20U)
#define SAFETY_CONTINUITY_LOSS_LIMIT (10U)
#ifndef CONTINUITY_PRESENT_STATE
#define CONTINUITY_PRESENT_STATE GPIO_PIN_SET
#endif

static uint64_t safety_heartbeat_missing_since_ms;
static volatile uint32_t safety_heartbeat_timeout_count;
static volatile uint32_t safety_continuity_loss_count;
static volatile uint32_t safety_continuity_loss_latched_count;
static volatile GPIO_PinState safety_last_continuity_state;

static uint64_t safety_now_ms(void)
{
    return ((uint64_t)tx_time_get() * 1000ULL) / (uint64_t)TX_TIMER_TICKS_PER_SECOND;
}

static void safety_request_abort(void)
{
    (void)thread_comm_set_abort(1U);
    (void)thread_comm_send(CMD_ABORT, TX_NO_WAIT);
    (void)telemetry_broadcast_abort("Valve board safety abort");
}

static void safety_check_heartbeat(void)
{
    const uint64_t now_ms = safety_now_ms();
    const uint64_t last_heartbeat_ms = thread_comm_get_groundstation_heartbeat_ms();

    if (thread_comm_abort_allowed() == 0U)
    {
        safety_heartbeat_missing_since_ms = 0ULL;
        return;
    }

    if (last_heartbeat_ms != 0ULL)
    {
        safety_heartbeat_missing_since_ms = 0ULL;
        if ((now_ms - last_heartbeat_ms) >= SAFETY_HEARTBEAT_TIMEOUT_MS)
        {
            safety_heartbeat_timeout_count++;
            safety_request_abort();
        }
        return;
    }

    if (safety_heartbeat_missing_since_ms == 0ULL)
    {
        safety_heartbeat_missing_since_ms = now_ms;
        return;
    }

    if ((now_ms - safety_heartbeat_missing_since_ms) >= SAFETY_HEARTBEAT_TIMEOUT_MS)
    {
        safety_heartbeat_timeout_count++;
        safety_request_abort();
    }
}

static void safety_check_continuity(void)
{
    const uint64_t last_heartbeat_ms = thread_comm_get_groundstation_heartbeat_ms();
    const GPIO_PinState continuity_state = HAL_GPIO_ReadPin(Continuity_GPIO_Port, Continuity_Pin);
    safety_last_continuity_state = continuity_state;

    if (thread_comm_abort_allowed() == 0U)
    {
        safety_continuity_loss_count = 0U;
        return;
    }

    if (last_heartbeat_ms == 0ULL)
    {
        safety_continuity_loss_count = 0U;
        return;
    }

    if (continuity_state == CONTINUITY_PRESENT_STATE)
    {
        safety_continuity_loss_count = 0U;
        return;
    }

    safety_continuity_loss_count++;
    if (safety_continuity_loss_count >= SAFETY_CONTINUITY_LOSS_LIMIT)
    {
        safety_continuity_loss_latched_count++;
        safety_request_abort();
    }
}

void safety_thread_entry(ULONG initial_input)
{
    (void)initial_input;

    for (;;)
    {
        safety_check_heartbeat();
        safety_check_continuity();
        tx_thread_sleep(SAFETY_CHECK_PERIOD_TICKS);
    }
}

UINT create_safety_thread(TX_BYTE_POOL *byte_pool)
{
    CHAR *pointer;

    if (tx_byte_allocate(byte_pool, (VOID **)&pointer,
                         SAFETY_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
    {
        return TX_POOL_ERROR;
    }

    return tx_thread_create(&safety_thread,
                            "Safety Thread",
                            safety_thread_entry,
                            0,
                            pointer,
                            SAFETY_THREAD_STACK_SIZE,
                            5,
                            5,
                            TX_NO_TIME_SLICE,
                            TX_AUTO_START);
}
