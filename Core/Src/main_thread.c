// main_thread.c
#include "VB-Threads.h"
#include "tx_api.h"
#include "telemetry.h"
#include "can_bus.h"
#include "main.h"
#include "thread_comm.h"
#include "solenoid_driver.h"
#include "servo_driver.h"
#include <stdbool.h>

TX_THREAD main_thread;

#define MAIN_THREAD_STACK_SIZE (12U * 1024U)
#define UMBILICAL_STATUS_PERIOD_TICKS TX_TIMER_TICKS_PER_SECOND
#define LAUNCH_SEQUENCE_PILOT_OPEN_DELAY_MS 10000U
#define LAUNCH_SEQUENCE_PILOT_OPEN_DURATION_MS 1500U
#define LAUNCH_SEQUENCE_PILOT_STATUS_PERIOD_TICKS \
    ((TX_TIMER_TICKS_PER_SECOND >= 10U) ? (TX_TIMER_TICKS_PER_SECOND / 10U) : 1U)
#define MAIN_THREAD_MAX_COMMANDS_PER_TICK 4U
#define ABORT_STATUS_PERIOD_TICKS TX_TIMER_TICKS_PER_SECOND
#define ABORT_LED_PERIOD_TICKS ((TX_TIMER_TICKS_PER_SECOND >= 2U) ? (TX_TIMER_TICKS_PER_SECOND / 2U) : 1U)


static uint8_t g_aborted = 0U; 
static uint8_t g_pilot_valve_state = 0U;
static uint8_t g_no_servo_open_state = 0U;
static uint8_t g_nitrous_servo_open_state = 0U;
static uint8_t g_launch_sequence_active = 0U;
static uint8_t g_launch_sequence_actuator_started = 0U;
static uint8_t g_launch_sequence_pilot_opened = 0U;
static uint8_t g_launch_sequence_completed = 0U;
static volatile uint8_t g_outputs_initialized = 0U;
static volatile uint32_t g_launch_sequence_start_count = 0U;
static volatile uint32_t g_launch_sequence_service_count = 0U;
static volatile uint32_t g_launch_sequence_pilot_attempt_count = 0U;
static volatile uint32_t g_launch_sequence_finish_count = 0U;
static volatile uint32_t g_launch_sequence_abort_seen_count = 0U;
static volatile ULONG g_launch_sequence_last_elapsed_ticks = 0U;
static ULONG g_launch_sequence_start_ticks = 0U;
static ULONG g_launch_sequence_pilot_open_ticks = 0U;
static ULONG g_launch_sequence_last_pilot_status_ticks = 0U;
static ULONG g_abort_last_status_ticks = 0U;
static ULONG g_abort_last_led_ticks = 0U;
static ULONG main_thread_stack[MAIN_THREAD_STACK_SIZE / sizeof(ULONG)];

// Umbilical GPIO inputs 
solenoid_t pilot_solenoid = {Solenoid_GPIO_Port, Solenoid_Pin, Solenoid_GPIO_Port, Solenoid_Pin, NULL, 0, 5};


static uint8_t publish_umbilical_status(uint8_t status_id, uint8_t state)
{
    return (telemetry_publish_umbilical_status(status_id,
                                               (state != 0U) ? 1U : 0U) == SEDS_OK)
               ? 1U
               : 0U;
}

static ULONG ms_to_ticks(uint32_t ms)
{
    ULONG ticks = (ULONG)(((uint64_t)ms * (uint64_t)TX_TIMER_TICKS_PER_SECOND + 999ULL) / 1000ULL);
    return (ticks == 0U) ? 1U : ticks;
}

static void publish_all_umbilical_statuses(void){
    static uint8_t published_once = 0U;
    static ULONG last_publish_ticks = 0U;
    ULONG now = tx_time_get();

    if ((published_once != 0U) &&
        ((ULONG)(now - last_publish_ticks) < UMBILICAL_STATUS_PERIOD_TICKS))
    {
        return;
    }

    (void)publish_umbilical_status(CMD_PILOT_OPEN, g_pilot_valve_state);
    (void)publish_umbilical_status(CMD_NORMALLY_OPEN_OPEN, g_no_servo_open_state);
    (void)publish_umbilical_status(CMD_DUMP_OPEN, g_nitrous_servo_open_state);
    (void)publish_umbilical_status(CMD_SEQUENCE, g_launch_sequence_active);

    published_once = 1U;
    last_publish_ticks = now;
}

static void publish_abort_umbilical_statuses(void)
{
    (void)publish_umbilical_status(CMD_PILOT_OPEN, g_pilot_valve_state);
    (void)publish_umbilical_status(CMD_NORMALLY_OPEN_OPEN, g_no_servo_open_state);
    (void)publish_umbilical_status(CMD_DUMP_OPEN, g_nitrous_servo_open_state);
    (void)publish_umbilical_status(CMD_SEQUENCE, g_launch_sequence_active);
    g_abort_last_status_ticks = tx_time_get();
}

void pilot_valve_on(void){
    if (solenoidOn(&pilot_solenoid) == 0) {
        g_pilot_valve_state = 1U;
        (void)publish_umbilical_status(CMD_PILOT_OPEN, g_pilot_valve_state);
    }
}

void pilot_valve_off(void)
{
    solenoidOff(&pilot_solenoid);
    g_pilot_valve_state = 0U;
    (void)publish_umbilical_status(CMD_PILOT_OPEN, g_pilot_valve_state);
}

static void nitrous_valve_open(void)
{
    dump_servo_open();
    g_nitrous_servo_open_state = 1U;
    (void)publish_umbilical_status(CMD_DUMP_OPEN, g_nitrous_servo_open_state);
}

static void nitrous_valve_close(void)
{
    dump_servo_close();
    g_nitrous_servo_open_state = 0U;
    (void)publish_umbilical_status(CMD_DUMP_OPEN, g_nitrous_servo_open_state);
}

static void normally_open_valve_open(void)
{
    no_servo_open();
    g_no_servo_open_state = 1U;
    (void)publish_umbilical_status(CMD_NORMALLY_OPEN_OPEN, g_no_servo_open_state);
}

static void normally_open_valve_close(void)
{
    no_servo_close();
    g_no_servo_open_state = 0U;
    (void)publish_umbilical_status(CMD_NORMALLY_OPEN_OPEN, g_no_servo_open_state);
}

static void main_thread_force_outputs_safe_off(void)
{
    g_launch_sequence_active = 0U;
    g_launch_sequence_actuator_started = 0U;
    g_launch_sequence_pilot_opened = 0U;
    g_launch_sequence_pilot_open_ticks = 0U;

    if (g_outputs_initialized == 0U)
    {
        return;
    }

    no_servo_open();
    dump_servo_open();
    solenoidOff(&pilot_solenoid);

    g_no_servo_open_state = 1U;
    g_nitrous_servo_open_state = 1U;
    g_pilot_valve_state = 0U;
}

static void start_launch_sequence(void)
{
    if (g_launch_sequence_active != 0U)
    {
        return;
    }

    g_launch_sequence_active = 1U;
    g_launch_sequence_actuator_started = 1U;
    g_launch_sequence_pilot_opened = 0U;
    g_launch_sequence_completed = 0U;
    g_launch_sequence_start_ticks = tx_time_get();
    g_launch_sequence_pilot_open_ticks = 0U;
    g_launch_sequence_last_pilot_status_ticks = 0U;
    g_launch_sequence_start_count++;
    uint64_t launch_timestamp_ms = thread_comm_get_launch_command_timestamp_ms();
    if (launch_timestamp_ms == 0ULL)
    {
        launch_timestamp_ms = telemetry_unix_ms();
    }
    if (thread_comm_get_flight_state() != VALVE_FLIGHT_STATE_LAUNCH)
    {
        (void)thread_comm_set_flight_state(VALVE_FLIGHT_STATE_LAUNCH);
    }
    if (launch_timestamp_ms != 0ULL)
    {
        (void)telemetry_publish_flight_state_at(VALVE_FLIGHT_STATE_LAUNCH,
                                                launch_timestamp_ms);
    }
    if (launch_timestamp_ms != 0ULL)
    {
        (void)telemetry_send_actuator_command_at(CMD_IGNITER_SEQUENCE,
                                                 launch_timestamp_ms);
    }
    else
    {
        (void)telemetry_send_actuator_command(CMD_IGNITER_SEQUENCE);
    }
    (void)publish_umbilical_status(CMD_SEQUENCE, 1U);
}

static void service_launch_sequence_request(void)
{
    uint64_t launch_timestamp_ms = 0ULL;

    if (thread_comm_take_launch_sequence_request(&launch_timestamp_ms) == 0U)
    {
        return;
    }

    (void)thread_comm_set_abort(0U);
    (void)thread_comm_set_flight_state(VALVE_FLIGHT_STATE_LAUNCH);
    g_aborted = 0U;
    (void)thread_comm_set_launch_command_timestamp_ms(launch_timestamp_ms);
    start_launch_sequence();
}

static void service_launch_sequence(void)
{
    if (g_launch_sequence_active == 0U)
    {
        return;
    }

    const ULONG now = tx_time_get();
    const ULONG elapsed = now - g_launch_sequence_start_ticks;
    g_launch_sequence_service_count++;
    g_launch_sequence_last_elapsed_ticks = elapsed;
    const ULONG pilot_open_ticks = ms_to_ticks(LAUNCH_SEQUENCE_PILOT_OPEN_DELAY_MS);
    const ULONG pilot_open_duration_ticks =
        ms_to_ticks(LAUNCH_SEQUENCE_PILOT_OPEN_DURATION_MS);

    if ((g_launch_sequence_pilot_opened == 0U) && (elapsed >= pilot_open_ticks))
    {
        g_launch_sequence_pilot_attempt_count++;
        pilot_valve_on();
        g_launch_sequence_pilot_opened = 1U;
        g_launch_sequence_pilot_open_ticks = now;
        g_launch_sequence_last_pilot_status_ticks = now;
    }

    if ((g_launch_sequence_pilot_opened != 0U) &&
        ((ULONG)(now - g_launch_sequence_last_pilot_status_ticks) >=
         LAUNCH_SEQUENCE_PILOT_STATUS_PERIOD_TICKS))
    {
        (void)publish_umbilical_status(CMD_PILOT_OPEN, 1U);
        g_launch_sequence_last_pilot_status_ticks = now;
    }

    if ((g_launch_sequence_pilot_opened != 0U) &&
        ((ULONG)(now - g_launch_sequence_pilot_open_ticks) >=
         pilot_open_duration_ticks))
    {
        pilot_valve_off();
        g_launch_sequence_active = 0U;
        g_launch_sequence_pilot_opened = 0U;
        g_launch_sequence_pilot_open_ticks = 0U;
        g_launch_sequence_completed = 1U;
        g_launch_sequence_finish_count++;
        (void)publish_umbilical_status(CMD_SEQUENCE, 0U);
    }
}

static void handle_command(thread_comm_msg_t cmd){
    if (g_aborted)
    {
        return;
    }
    switch (cmd) {
    case CMD_PILOT_OPEN:
        pilot_valve_on();
        break;
    case CMD_PILOT_CLOSE:
        pilot_valve_off();
        break;
    case CMD_DUMP_CLOSE:
        nitrous_valve_close();
        break;
    case CMD_DUMP_OPEN:
        nitrous_valve_open();
        break;
    case CMD_NORMALLY_OPEN_CLOSE:
        normally_open_valve_close();
        break;
    case CMD_NORMALLY_OPEN_OPEN:
        normally_open_valve_open();
        break;
    case CMD_SEQUENCE:
        start_launch_sequence();
        break;
    case CMD_ABORT:
        (void)thread_comm_set_abort(1U);
        break;
    default:
        break;
    }
}

static void abort_state(void){
    main_thread_force_outputs_safe_off();
    g_aborted = 1U;
    publish_abort_umbilical_statuses();
    g_abort_last_led_ticks = tx_time_get();
    HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
    HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
}

static void service_abort_state(void)
{
    const ULONG now = tx_time_get();

    if ((ULONG)(now - g_abort_last_status_ticks) >= ABORT_STATUS_PERIOD_TICKS)
    {
        publish_abort_umbilical_statuses();
    }

    if ((ULONG)(now - g_abort_last_led_ticks) >= ABORT_LED_PERIOD_TICKS)
    {
        HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
        HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
        g_abort_last_led_ticks = now;
    }
}

void main_thread_entry(ULONG initial_input)
{
    (void)initial_input;

    // Initialize pilot valve solenoid
    solenoidInit(&pilot_solenoid);
    
    // Initialize the PWM for servos
    HAL_TIM_PWM_Start(NO_SERVO_TIMER, NO_SERVO_CHANNEL);
    HAL_TIM_PWM_Start(DUMP_SERVO_TIMER, DUMP_SERVO_CHANNEL);
    g_outputs_initialized = 1U;

    // Set initial valve positions
    pilot_valve_off();
    normally_open_valve_open();
    nitrous_valve_open();
    
    thread_comm_msg_t msg;
    
    publish_all_umbilical_statuses();
    for (;;) {
        service_launch_sequence_request();

        if (thread_comm_get_abort() != 0U)
        {
            if (g_launch_sequence_active != 0U)
            {
                g_launch_sequence_abort_seen_count++;
            }
            if (g_aborted == 0U)
            {
                abort_state();
            }
            else
            {
                service_abort_state();
            }
            while (thread_comm_receive(&msg, TX_NO_WAIT) == TX_SUCCESS){}
            tx_thread_sleep(1);
            continue;
        }

        service_launch_sequence();
        service_launch_sequence_request();

        uint32_t commands_drained = 0U;
        while ((g_aborted != 1U) &&
               (thread_comm_get_abort() == 0U) &&
               (commands_drained < MAIN_THREAD_MAX_COMMANDS_PER_TICK) &&
               (thread_comm_receive(&msg, TX_NO_WAIT) == TX_SUCCESS))
        {
            handle_command(msg);
            commands_drained++;
        }

        if (thread_comm_get_abort() != 0U)
        {
            continue;
        }

        service_launch_sequence();
        service_launch_sequence_request();
        publish_all_umbilical_statuses();
        tx_thread_sleep(1);
    }
}    

UINT create_main_thread(TX_BYTE_POOL *byte_pool)
{
    (void)byte_pool;

    UINT status = tx_thread_create(&main_thread,
                                   "Main Thread",
                                   main_thread_entry,
                                   0,
                                   main_thread_stack,
                                   MAIN_THREAD_STACK_SIZE,
                                   6,
                                   6,
                                   TX_NO_TIME_SLICE,
                                   TX_AUTO_START);

    return status;
}
