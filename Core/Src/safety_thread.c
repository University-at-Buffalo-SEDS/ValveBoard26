// safety_thread.c
#include "VB-Threads.h"

#include "main.h"
#include "telemetry.h"
#include "thread_comm.h"
#include "tx_api.h"

TX_THREAD safety_thread;

#define SAFETY_THREAD_STACK_SIZE (3U * 1024U)
#define SAFETY_HEARTBEAT_TIMEOUT_MS (5000ULL)
#define SAFETY_LAUNCH_CONTINUITY_TIMEOUT_MS (5ULL * 60ULL * 1000ULL)
#define SAFETY_CHECK_PERIOD_TICKS ((TX_TIMER_TICKS_PER_SECOND + 99U) / 100U)
#ifndef CONTINUITY_PRESENT_STATE
#define CONTINUITY_PRESENT_STATE GPIO_PIN_SET
#endif

static uint64_t safety_heartbeat_missing_since_ms;
static volatile uint32_t safety_heartbeat_timeout_count;
static volatile uint32_t safety_continuity_loss_count;
static volatile uint32_t safety_continuity_loss_latched_count;
static volatile uint32_t safety_launch_continuity_timeout_count;
static volatile GPIO_PinState safety_last_continuity_state;
static uint64_t safety_launch_continuity_loss_since_ms;
static ULONG safety_thread_stack[SAFETY_THREAD_STACK_SIZE / sizeof(ULONG)];

static uint64_t safety_now_ms(void)
{
    return ((uint64_t)tx_time_get() * 1000ULL) / (uint64_t)TX_TIMER_TICKS_PER_SECOND;
}

static void safety_request_abort(void)
{
    if (thread_comm_get_abort() != 0U)
    {
        return;
    }

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
    const GPIO_PinState continuity_state = HAL_GPIO_ReadPin(Continuity_GPIO_Port, Continuity_Pin);
    const uint8_t flight_state = thread_comm_get_flight_state();
    safety_last_continuity_state = continuity_state;

    if (continuity_state == CONTINUITY_PRESENT_STATE)
    {
        safety_continuity_loss_count = 0U;
        safety_launch_continuity_loss_since_ms = 0ULL;
        return;
    }

    if (thread_comm_abort_allowed() != 0U)
    {
        safety_continuity_loss_count++;
        safety_continuity_loss_latched_count++;
        safety_request_abort();
        return;
    }

    if (flight_state >= VALVE_FLIGHT_STATE_LAUNCH)
    {
        const uint64_t now_ms = safety_now_ms();

        if (safety_launch_continuity_loss_since_ms == 0ULL)
        {
            safety_launch_continuity_loss_since_ms = now_ms;
            return;
        }

        if ((now_ms - safety_launch_continuity_loss_since_ms) >=
            SAFETY_LAUNCH_CONTINUITY_TIMEOUT_MS)
        {
            safety_launch_continuity_timeout_count++;
            safety_request_abort();
        }
        return;
    }

    safety_continuity_loss_count = 0U;
    safety_launch_continuity_loss_since_ms = 0ULL;
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
    (void)byte_pool;

    return tx_thread_create(&safety_thread,
                            "Safety Thread",
                            safety_thread_entry,
                            0,
                            safety_thread_stack,
                            SAFETY_THREAD_STACK_SIZE,
                            5,
                            5,
                            TX_NO_TIME_SLICE,
                            TX_AUTO_START);
}
