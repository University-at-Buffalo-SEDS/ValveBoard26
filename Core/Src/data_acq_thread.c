#include "VB-Threads.h"

#include "ltc2990.h"
#include "pressure_transducer_driver.h"
#include "telemetry.h"
#include "tx_api.h"

extern I2C_HandleTypeDef hi2c2;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim3;

TX_THREAD data_acq_thread;

#define DATA_ACQ_THREAD_STACK_SIZE (8U * 1024U)
#define DATA_ACQ_REPORT_PERIOD_TICKS ((ULONG)TX_TIMER_TICKS_PER_SECOND)
#define DATA_ACQ_STARTUP_DELAY_TICKS (1U * TX_TIMER_TICKS_PER_SECOND)
#define DATA_ACQ_PRESSURE_PERIOD_TICKS \
    ((TX_TIMER_TICKS_PER_SECOND >= 4U) ? (TX_TIMER_TICKS_PER_SECOND / 4U) : 1U)
#define DATA_ACQ_LOOP_PERIOD_TICKS DATA_ACQ_PRESSURE_PERIOD_TICKS

static LTC2990_Handle_t ltc2990_voltage_handle;
static LTC2990_Handle_t ltc2990_current_handle;
static uint8_t ltc2990_voltage_ready;
static uint8_t ltc2990_current_ready;
static float ltc2990_latest_voltages[4];
static volatile uint32_t data_acq_voltage_init_fail_count;
static volatile uint32_t data_acq_current_init_fail_count;
static volatile uint32_t data_acq_pressure_init_fail_count;
static volatile uint32_t data_acq_pressure_read_fail_count;
static volatile uint32_t data_acq_cycle_count;
static ULONG data_acq_last_power_ticks;
static ULONG data_acq_last_pressure_ticks;
static ULONG data_acq_thread_stack[DATA_ACQ_THREAD_STACK_SIZE / sizeof(ULONG)];

static pressure_transducer_t fuel_tank_pressure = {
    .hadc = &hadc3,
    .trigger_timer = &htim3,
    .channel = ADC_CHANNEL_5,
    .min_adc = PRESSURE_TRANSDUCER_MIN_ADC,
    .max_adc = PRESSURE_TRANSDUCER_MAX_ADC,
    .min_pressure = PRESSURE_TRANSDUCER_MIN_PRESSURE,
    .max_pressure = PRESSURE_TRANSDUCER_MAX_PRESSURE,
    .timeout_ms = PRESSURE_TRANSDUCER_ADC_TIMEOUT_MS,
};

void data_acq_get_latest_voltages(float voltages[4])
{
    if (voltages == TX_NULL) {
        return;
    }

    for (uint8_t i = 0U; i < 4U; i++) {
        voltages[i] = ltc2990_latest_voltages[i];
    }
}

static void data_acq_ltc2990_init(void)
{
    ltc2990_voltage_ready =
        (LTC2990_Init(&ltc2990_voltage_handle,
                      &hi2c2,
                      LTC2990_I2C_ADDRESS_VOLTAGE,
                      LTC2990_ROLE_VOLTAGE) == 0)
            ? 1U
            : 0U;
    if (ltc2990_voltage_ready == 0U) {
        data_acq_voltage_init_fail_count++;
    }

    ltc2990_current_ready =
        (LTC2990_Init(&ltc2990_current_handle,
                      &hi2c2,
                      LTC2990_I2C_ADDRESS_CURRENT,
                      LTC2990_ROLE_CURRENT) == 0)
            ? 1U
            : 0U;
    if (ltc2990_current_ready == 0U) {
        data_acq_current_init_fail_count++;
    }
}

static void data_acq_pressure_init(void)
{
    if (pressureTransducerInit(&fuel_tank_pressure) != HAL_OK) {
        data_acq_pressure_init_fail_count++;
    }
}

static void data_acq_report_pressure(void)
{
    pressure_transducer_sample_t sample;
    const ULONG now = tx_time_get();

    if ((ULONG)(now - data_acq_last_pressure_ticks) < DATA_ACQ_PRESSURE_PERIOD_TICKS) {
        return;
    }

    if (pressureTransducerRead(&fuel_tank_pressure, &sample) == HAL_OK) {
        (void)log_telemetry_asynchronous(SEDS_DT_FUEL_TANK_PRESSURE, &sample.pressure, 1U,
                                         sizeof(sample.pressure));
        data_acq_last_pressure_ticks = now;
    } else {
        data_acq_pressure_read_fail_count++;
    }
}

static void data_acq_report_power(void)
{
    const ULONG now = tx_time_get();

    if ((ULONG)(now - data_acq_last_power_ticks) < DATA_ACQ_REPORT_PERIOD_TICKS) {
        return;
    }

    data_acq_cycle_count++;

    if (ltc2990_voltage_ready != 0U) {
        telemetry_ltc2990_update_voltage(&ltc2990_voltage_handle);
        LTC2990_Get_Values(&ltc2990_voltage_handle, ltc2990_latest_voltages);
    }

    if (ltc2990_current_ready != 0U) {
        telemetry_ltc2990_update_current(&ltc2990_current_handle);
    }

    data_acq_last_power_ticks = now;
}

void data_acq_thread_entry(ULONG initial_input)
{
    (void)initial_input;

    tx_thread_sleep(DATA_ACQ_STARTUP_DELAY_TICKS);
    data_acq_ltc2990_init();
    data_acq_pressure_init();

    for (;;) {
        if ((ltc2990_voltage_ready != 0U) || (ltc2990_current_ready != 0U)) {
            data_acq_report_power();
        }
        data_acq_report_pressure();
        tx_thread_sleep(DATA_ACQ_LOOP_PERIOD_TICKS);
    }
}

UINT create_data_acq_thread(TX_BYTE_POOL *byte_pool)
{
    (void)byte_pool;

    return tx_thread_create(&data_acq_thread,
                            "Data Acquisition Thread",
                            data_acq_thread_entry,
                            0,
                            data_acq_thread_stack,
                            DATA_ACQ_THREAD_STACK_SIZE,
                            6,
                            6,
                            TX_NO_TIME_SLICE,
                            TX_AUTO_START);
}
