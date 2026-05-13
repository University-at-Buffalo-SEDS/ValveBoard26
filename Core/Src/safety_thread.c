#include "VB-Threads.h"
#include "tx_api.h"
#include "telemetry.h"
#include "main.h"
#include "thread_comm.h"
#include "ltc2990.h"
#include "pressure_transducer_driver.h"
#include "solenoid_driver.h"
#include "servo_driver.h"
#include <stdbool.h>

#define PRESSURE_THRESHOLD      900.0f   // 900 psi 
#define CURRENT_THRESHOLD       V1DV2_THRESHOLD  // 3000 mA
#define STARTUP_TIMEOUT         5000U // 5000 ms
#define SAFETY_SLEEP            5U // 5 ticks 

TX_THREAD safety_thread;
#define SAFETY_THREAD_STACK_SIZE (16U * 1024U)

extern LTC2990_Handle_t       ltc2990_dev;
extern ADC_HandleTypeDef      hadc3;
extern TIM_HandleTypeDef      htim3;
extern TX_EVENT_FLAGS_GROUP   event_flags;

// same as in main_thread 
static pressure_transducer_t  safety_pressure = {
    .hadc          = &hadc3,
    .trigger_timer = &htim3,
    .channel       = ADC_CHANNEL_5,
    .vref          = PRESSURE_TRANSDUCER_ADC_VREF,
    .min_voltage   = PRESSURE_TRANSDUCER_MIN_VOLTAGE,
    .max_voltage   = PRESSURE_TRANSDUCER_MAX_VOLTAGE,
    .min_pressure  = PRESSURE_TRANSDUCER_MIN_PRESSURE,
    .max_pressure  = PRESSURE_TRANSDUCER_MAX_PRESSURE,
    .timeout_ms    = PRESSURE_TRANSDUCER_ADC_TIMEOUT_MS,
};

typedef enum {
    STATE_STARTUP,
    STATE_IDLE,
    STATE_LAUNCH,
    STATE_ABORT,
} flight_state_t;

static flight_state_t g_flight_state = STATE_STARTUP;

// ---------------------------------------------------------

static void trigger_abort(void)
{
    (void)tx_event_flags_set(&event_flags, ABORT_FLAG, TX_OR);
    g_flight_state = STATE_ABORT;
}
static void trigger_launch(void)
{
    g_flight_state = STATE_LAUNCH;
}

static bool voltages_normal(void)
{
    float v1 = 0.0f;
    float v2 = 0.0f;
    float v3 = 0.0f;
    float v4 = 0.0f;
    LTC2990_GetSingleEndedVoltage(&ltc2990_dev, 0U, &v1);
    LTC2990_GetSingleEndedVoltage(&ltc2990_dev, 1U, &v2);
    LTC2990_GetSingleEndedVoltage(&ltc2990_dev, 2U, &v3);
    LTC2990_GetSingleEndedVoltage(&ltc2990_dev, 3U, &v4);
    if (v1 < V1_THRESHOLD) { 
        return false; 
    } if (v2 < V2_THRESHOLD) { 
        return false; 
    } if (v3 < V3_THRESHOLD) { 
        return false; 
    } if (v4 < V4_THRESHOLD) { 
        return false; 
    }
    return true;
}

static bool current_normal(void)
{
    // pretty sure it's already in milliamps, if not simply convert...
    // float current = ltc2990_dev.current * 1000.0f; 
    float current = LTC2990_getCurrent(&ltc2990_dev);
    return (current < CURRENT_ABORT); 
}

static bool pressure_normal(void)
{
    pressure_transducer_sample_t sample;
    // unsure if there's a certain pressure threshold it must hit, but I know what threshold it can't 
    return (sample.pressure < PRESSURE_ABORT);
}

static bool continuity_ok(void)
{
    return (HAL_GPIO_ReadPin(CONTINUITY_GPIO_Port, CONTINUITY_Pin) == GPIO_PIN_SET); // pin always set high, if low it needs to trigger abort  
}

static bool solenoid_ok(void)
{
    // kinda unsure how to go about this one 
    // don't know how to go about the check if the solenoid skyrockets due to a short 
    // just make sure pilot pin is set to high and there's no fault?
    return (HAL_GPIO_ReadPin(PILOT_GPIO_Port, PILOT_Pin) == GPIO_PIN_SET) && !solenoidFault(&pilot_solenoid);
}

static bool servos_normal(void)
{
    bool no_signal_high   = (HAL_GPIO_ReadPin(NO_SIGNAL_GPIO_Port,   NO_SIGNAL_Pin)   == GPIO_PIN_SET);
    bool dump_signal_high = (HAL_GPIO_ReadPin(DUMP_SIGNAL_GPIO_Port, DUMP_SIGNAL_Pin) == GPIO_PIN_SET);
    return (no_signal_high && dump_signal_high);
}

//-----------------------------------------------------------------------------

static void handle_startup(void)
{
    const ULONG start = tx_time_get();
    const ULONG timeout_ticks = (ULONG)(((uint64_t)STARTUP_TIMEOUT_MS * (uint64_t)TX_TIMER_TICKS_PER_SECOND + 999ULL) / 1000ULL);
    for (;;) {
        // Check if main_thread already set abort
        ULONG flags = 0U;
        if (tx_event_flags_get(&event_flags, ABORT_FLAG, TX_OR, &flags, TX_NO_WAIT) == TX_SUCCESS) {
            g_flight_state = STATE_ABORT;
            return;
        }

        bool ltc_ready      = voltages_normal() && current_normal();
        bool pressure_ready = pressure_normal();
        bool servos_ready   = servos_normal();

        if (ltc_ready && pressure_ready && servos_ready) {
            g_flight_state = STATE_IDLE;
            return;
        }

        // Timeout, i think this works??
        if ((ULONG)(tx_time_get() - start) >= timeout_ticks) {
            trigger_abort();
            return;
        }

        tx_thread_sleep(SAFETY_SLEEP_TICKS);
    }
}

static void handle_idle(void)
{
    for (;;) {
        // Check if main_thread already set abort
        ULONG flags = 0U;
        if (tx_event_flags_get(&event_flags, ABORT_FLAG, TX_OR, &flags, TX_NO_WAIT) == TX_SUCCESS) {
            g_flight_state = STATE_ABORT;
            return;
        } 
        
        ULONG launch_flag = 0U;
        if (tx_event_flags_get(&event_flags, LAUNCH_FLAG, TX_OR, &launch_flag, TX_NO_WAIT) == TX_SUCCESS)
        {
            trigger_launch();
            return;
        } 
        
        if (!voltages_normal()){
            (void)log_error_asynchronous("SAFETY: voltage below threshold — aborting\r\n");
            trigger_abort();
            return;
        } if (!current_normal()) {
            (void)log_error_asynchronous("SAFETY: current above threshold — aborting\r\n");
            trigger_abort();
            return;
        } if (!pressure_normal()){
            (void)log_error_asynchronous("SAFETY: pressure above threshold — aborting\r\n");
            trigger_abort();
            return;
        } if (!continuity_ok()) {
            (void)log_error_asynchronous("SAFETY: umbilical continuity lost — aborting\r\n");
            trigger_abort();
            return;
        } if (!servos_normal()){
            (void)log_error_asynchronous("SAFETY: servo pin is low — aborting\r\n");
            trigger_abort();
            return;
        } if (!solenoid_ok()) {
            (void)log_error_asynchronous("SAFETY: solenoid fault — aborting\r\n");
            trigger_abort();
            return;
        }

        tx_thread_sleep(SAFETY_SLEEP_TICKS);
    }
}

// In LAUNCH we only watch for an externally-triggered abort; the rocket is
// on its own once the sequence fires.
static void handle_launch(void)
{
    for (;;) {
        ULONG flags = 0U;
        if (tx_event_flags_get(&event_flags, ABORT_FLAG, TX_OR, &flags, TX_NO_WAIT) == TX_SUCCESS)
        {
            g_flight_state = STATE_ABORT;
            return;
        }

        tx_thread_sleep(SAFETY_SLEEP_TICKS);
    }
}

static void handle_abort(void)
{
    // does nothing, main_thread actually handles the abort sequence 
    for (;;)
    {
        tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
    }
}

// -----------------------------------------------------------------------------

void safety_thread_entry(ULONG initial_input)
{
    (void)initial_input;

    // a little pause to let main_thread initialize everything 
    tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 2U);

    g_flight_state = STATE_STARTUP;

    for (;;)
    {
        switch (g_flight_state) {
        case STATE_STARTUP:
            handle_startup();
            break;
        case STATE_IDLE:
            handle_idle();
            break;
        case STATE_LAUNCH:
            handle_launch();
            break;
        case STATE_ABORT:
            handle_abort();
            break;
        default:
            trigger_abort();
            break;
        }
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