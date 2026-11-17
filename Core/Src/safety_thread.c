#include "telemetry_event_clock.h"
#include "board_watchdog.h"
// safety_thread.c
#include "VB-Threads.h"

#include "main.h"
#include "telemetry.h"
#include "thread_comm.h"
#include "tx_api.h"
#include "safety_diagnostics.h"
#include "board_abort_latch.h"



TX_THREAD safety_thread;

#define SAFETY_THREAD_STACK_SIZE (4U * 1024U)
#define SAFETY_HEARTBEAT_TIMEOUT_MS (5000ULL)
#define SAFETY_CHECK_PERIOD_TICKS ((TX_TIMER_TICKS_PER_SECOND + 99U) / 100U)
#ifndef CONTINUITY_PRESENT_STATE
#define CONTINUITY_PRESENT_STATE GPIO_PIN_SET
#endif

static uint64_t safety_heartbeat_missing_since_ms;
static volatile uint32_t safety_heartbeat_timeout_count;
static volatile uint32_t safety_continuity_loss_count;
static volatile uint32_t safety_continuity_loss_latched_count;
static volatile GPIO_PinState safety_last_continuity_state;
/* First sample of the current post-Launch continuity break, 0 intact. It
 * feeds the post-Launch auto-vent countdown (vent_countdown.h). */
static volatile uint64_t safety_launch_continuity_loss_since_ms;
static ULONG safety_thread_stack[SAFETY_THREAD_STACK_SIZE / sizeof(ULONG)];

static uint64_t safety_now_ms(void)
{
    return ((uint64_t)tx_time_get() * 1000ULL) / (uint64_t)TX_TIMER_TICKS_PER_SECOND;
}

static void safety_request_abort(uint32_t reason, const char *message)
{
    (void)message; /* Network thread reports the current owner-tagged local cause. */
    const uint64_t failure_ms = safety_now_ms();
    const uint64_t heartbeat_ms = thread_comm_get_groundstation_heartbeat_ms();
    const uint8_t flight_state = thread_comm_get_flight_state();
    const uint32_t continuity = HAL_GPIO_ReadPin(Continuity_GPIO_Port, Continuity_Pin);
    const uint64_t origin_utc = telemetry_event_clock_snapshot();
    if (!thread_comm_request_local_abort(reason, failure_ms, heartbeat_ms, flight_state,
        continuity, origin_utc, reason == SAFETY_ABORT_HEARTBEAT)) return;
    /* The retained Abort flag is consumed by every control-owner loop.
     * A redundant queued Abort could outlive verified recovery and relatch. */
}

static void safety_check_heartbeat(void)
{
    const uint64_t last_heartbeat_ms = thread_comm_get_groundstation_heartbeat_ms();
    /* Read reception before local uptime to avoid a concurrent-update underflow. */
    const uint64_t now_ms = safety_now_ms();

    if (thread_comm_heartbeat_abort_allowed() == 0U)
    {
        safety_heartbeat_missing_since_ms = 0ULL;
        return;
    }

    /* A board that boots before the GroundStation must remain safely idle.
     * Arm link-loss supervision only after the first GroundStation heartbeat;
     * from then on, loss of that established link still enters fail-safe. */
    if (last_heartbeat_ms == 0ULL)
    {
        safety_heartbeat_missing_since_ms = 0ULL;
        return;
    }

    safety_heartbeat_missing_since_ms = 0ULL;
    if ((now_ms >= last_heartbeat_ms) &&
        ((now_ms - last_heartbeat_ms) >= SAFETY_HEARTBEAT_TIMEOUT_MS))
    {
        safety_heartbeat_timeout_count++;
        safety_request_abort(SAFETY_ABORT_HEARTBEAT, "Valve safety abort: heartbeat timeout");
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
        safety_request_abort(SAFETY_ABORT_CONTINUITY, "Valve safety abort: continuity lost");
        return;
    }

    /* After Launch a break never aborts; it starts the auto-vent countdown.
     * The launch latch also covers a board reset after Launch, which boots
     * in Startup. */
    if (flight_state >= VALVE_FLIGHT_STATE_LAUNCH || board_launch_latch_is_set())
    {
        if (safety_launch_continuity_loss_since_ms == 0ULL)
        {
            const uint64_t now_ms = safety_now_ms();
            safety_launch_continuity_loss_since_ms = now_ms ? now_ms : 1ULL;
        }
        return;
    }

    safety_continuity_loss_count = 0U;
    safety_launch_continuity_loss_since_ms = 0ULL;
}

uint64_t safety_launch_continuity_broken_since_ms(void)
{
    uint64_t a, b;
    do {
        a = safety_launch_continuity_loss_since_ms;
        b = safety_launch_continuity_loss_since_ms;
    } while (a != b);
    return a;
}

void safety_thread_entry(ULONG initial_input)
{
    (void)initial_input;

    for (;;)
    {
        board_watchdog_progress(BOARD_WATCHDOG_SAFETY);
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
