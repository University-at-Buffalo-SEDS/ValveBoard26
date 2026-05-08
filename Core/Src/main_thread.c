// main_thread.c
#include "VB-Threads.h"
#include "tx_api.h"
#include "telemetry.h"
#include "can_bus.h"
#include "main.h"
#include "thread_comm.h"
#include "ltc2990.h"
#include "pressure_transducer_driver.h"
#include "solenoid_driver.h"
#include "servo_driver.h"
#include <stdbool.h>

LTC2990_Handle_t ltc2990_dev;
TX_THREAD main_thread;
extern I2C_HandleTypeDef hi2c2;

#define MAIN_THREAD_STACK_SIZE (16U *1024U)
#define UMBILICAL_STATUS_PERIOD_TICKS TX_TIMER_TICKS_PER_SECOND
#define PRESSURE_ACQ_PERIOD_TICKS \
    ((TX_TIMER_TICKS_PER_SECOND >= 4U) ? (TX_TIMER_TICKS_PER_SECOND / 4U) : 1U)
#define LAUNCH_SEQUENCE_ACTUATOR_START_DELAY_MS 5000U
#define LAUNCH_SEQUENCE_PILOT_OPEN_DELAY_MS 10000U
#define LAUNCH_SEQUENCE_PILOT_OPEN_DURATION_MS 1500U


static uint8_t g_aborted = 0U; 
static uint8_t g_pilot_valve_state = 0U;
static uint8_t g_no_servo_open_state = 0U;
static uint8_t g_nitrous_servo_open_state = 0U;
static uint8_t g_launch_sequence_active = 0U;
static uint8_t g_launch_sequence_actuator_started = 0U;
static uint8_t g_launch_sequence_pilot_opened = 0U;
static uint8_t g_launch_sequence_completed = 0U;
static uint8_t g_launch_sequence_actuator_command_sent = 0U;
static ULONG g_launch_sequence_start_ticks = 0U;

// Umbilical GPIO inputs 
solenoid_t pilot_solenoid = {Solenoid_GPIO_Port, Solenoid_Pin, Solenoid_GPIO_Port, Solenoid_Pin, NULL, 0, 5};
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim3;

static pressure_transducer_t fuel_tank_pressure = {
    .hadc = &hadc3,
    .trigger_timer = &htim3,
    .channel = ADC_CHANNEL_5,
    .vref = PRESSURE_TRANSDUCER_ADC_VREF,
    .min_voltage = PRESSURE_TRANSDUCER_MIN_VOLTAGE,
    .max_voltage = PRESSURE_TRANSDUCER_MAX_VOLTAGE,
    .min_pressure = PRESSURE_TRANSDUCER_MIN_PRESSURE,
    .max_pressure = PRESSURE_TRANSDUCER_MAX_PRESSURE,
    .timeout_ms = PRESSURE_TRANSDUCER_ADC_TIMEOUT_MS,
};


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

    published_once = 1U;
    last_publish_ticks = now;
}

static void publish_pressure_transducer(void)
{
    static uint8_t published_once = 0U;
    static ULONG last_publish_ticks = 0U;
    pressure_transducer_sample_t sample;
    ULONG now = tx_time_get();

    if ((published_once != 0U) &&
        ((ULONG)(now - last_publish_ticks) < PRESSURE_ACQ_PERIOD_TICKS))
    {
        return;
    }

    if (pressureTransducerRead(&fuel_tank_pressure, &sample) == HAL_OK)
    {
        (void)log_telemetry_asynchronous(SEDS_DT_FUEL_TANK_PRESSURE, &sample.pressure, 1U,
                                         sizeof(sample.pressure));
        published_once = 1U;
        last_publish_ticks = now;
    }
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

static void start_launch_sequence(void)
{
    if ((g_launch_sequence_active != 0U) || (g_launch_sequence_completed != 0U))
    {
        return;
    }

    g_launch_sequence_active = 1U;
    g_launch_sequence_actuator_started = 0U;
    g_launch_sequence_pilot_opened = 0U;
    g_launch_sequence_start_ticks = tx_time_get();
    (void)publish_umbilical_status(CMD_SEQUENCE, 1U);
}

static void service_launch_sequence(void)
{
    if (g_launch_sequence_active == 0U)
    {
        return;
    }

    const ULONG now = tx_time_get();
    const ULONG elapsed = now - g_launch_sequence_start_ticks;
    const ULONG actuator_start_ticks = ms_to_ticks(LAUNCH_SEQUENCE_ACTUATOR_START_DELAY_MS);
    const ULONG pilot_open_ticks = ms_to_ticks(LAUNCH_SEQUENCE_PILOT_OPEN_DELAY_MS);
    const ULONG pilot_close_ticks =
        pilot_open_ticks + ms_to_ticks(LAUNCH_SEQUENCE_PILOT_OPEN_DURATION_MS);

    if ((g_launch_sequence_actuator_started == 0U) && (elapsed >= actuator_start_ticks))
    {
        if (g_launch_sequence_actuator_command_sent == 0U)
        {
            (void)telemetry_send_actuator_command(CMD_IGNITER_SEQUENCE);
            g_launch_sequence_actuator_command_sent = 1U;
        }
        g_launch_sequence_actuator_started = 1U;
    }

    if ((g_launch_sequence_pilot_opened == 0U) && (elapsed >= pilot_open_ticks))
    {
        pilot_valve_on();
        g_launch_sequence_pilot_opened = 1U;
    }

    if ((g_launch_sequence_pilot_opened != 0U) && (elapsed >= pilot_close_ticks))
    {
        pilot_valve_off();
        g_launch_sequence_active = 0U;
        g_launch_sequence_pilot_opened = 0U;
        g_launch_sequence_completed = 1U;
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
    default:
        break;
    }
}

static void abort_state(void){
    g_launch_sequence_active = 0U;
    g_launch_sequence_actuator_started = 0U;
    g_launch_sequence_pilot_opened = 0U;
    normally_open_valve_open();
    nitrous_valve_open();
    pilot_valve_off();
    g_aborted = 1U;
    // Blink LEDs to indicate abort
    for(int i = 0; i < 10; i++) {
        HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
        HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
    }
}

void main_thread_entry(ULONG initial_input)
{
    (void)initial_input;

    // Initialize pilot valve solenoid
    solenoidInit(&pilot_solenoid);
    (void)pressureTransducerInit(&fuel_tank_pressure);
    
    // Initialize the PWM for servos
    HAL_TIM_PWM_Start(NO_SERVO_TIMER, NO_SERVO_CHANNEL);
    HAL_TIM_PWM_Start(DUMP_SERVO_TIMER, DUMP_SERVO_CHANNEL);


    // Set initial valve positions
    pilot_valve_off();
    normally_open_valve_open();
    nitrous_valve_open();
    
    // Initialize LTC2990 
    LTC2990_Init(&ltc2990_dev, &hi2c2, LTC2990_I2C_ADDRESS);
    LTC2990_SetMode(&ltc2990_dev, V1_V2_V3_V4, CLEAR_ALL);
    
    thread_comm_msg_t msg;
    
    publish_all_umbilical_statuses();
    for (;;) {
        if (thread_comm_get_abort() != 0U)
        {
            if (g_aborted == 0U)
            {
                abort_state();
            }
            while (thread_comm_receive(&msg, TX_NO_WAIT) == TX_SUCCESS){}
            tx_thread_sleep(10);
            continue;
        }
        while (g_aborted != 1U && thread_comm_receive(&msg, TX_NO_WAIT) == TX_SUCCESS)
        {
            handle_command(msg);
        }

        publish_all_umbilical_statuses();
        service_launch_sequence();
        publish_pressure_transducer();
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
