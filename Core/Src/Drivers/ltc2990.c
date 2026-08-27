#include "ltc2990.h"

#include "main.h"
#include "sedsnet_config.h"
#include "telemetry.h"

#include <math.h>
#include <stdint.h>

static inline void ltc2990_sleep_ms(uint32_t ms)
{
    ULONG ticks = (ULONG)(((uint64_t)ms * TX_TIMER_TICKS_PER_SECOND + 999ULL) / 1000ULL);
    if (ticks == 0U) {
        ticks = 1U;
    }

    if (tx_thread_identify() != TX_NULL) {
        tx_thread_sleep(ticks);
    } else {
        HAL_Delay(ms);
    }
}

static inline void i2c_lock(LTC2990_Handle_t *handle)
{
    if ((handle->i2c_mutex != TX_NULL) && (tx_thread_identify() != TX_NULL)) {
        (void)tx_mutex_get(handle->i2c_mutex, TX_WAIT_FOREVER);
    }
}

static inline void i2c_unlock(LTC2990_Handle_t *handle)
{
    if ((handle->i2c_mutex != TX_NULL) && (tx_thread_identify() != TX_NULL)) {
        (void)tx_mutex_put(handle->i2c_mutex);
    }
}

static uint8_t status_bit_from_msb(uint8_t msb_reg)
{
    switch (msb_reg) {
    case V1_MSB_REG:
        return 2U;
    case V2_MSB_REG:
        return 3U;
    case V3_MSB_REG:
        return 4U;
    case V4_MSB_REG:
        return 5U;
    default:
        return 0xFFU;
    }
}

int LTC2990_Init(LTC2990_Handle_t *handle,
                 I2C_HandleTypeDef *hi2c,
                 uint8_t address,
                 LTC2990_Role_t role)
{
    if ((handle == TX_NULL) || (hi2c == TX_NULL)) {
        return 1;
    }

    handle->hi2c = hi2c;
    handle->i2c_address = address;
    handle->role = role;
    handle->i2c_mutex = TX_NULL;

    for (uint8_t i = 0U; i < 4U; i++) {
        handle->last_values[i] = NAN;
    }

    if (HAL_I2C_IsDeviceReady(hi2c,
                              (uint16_t)(address << 1),
                              LTC2990_I2C_READY_TRIALS,
                              LTC2990_I2C_READY_TIMEOUT_MS) != HAL_OK) {
        return 1;
    }

    const uint8_t control = (role == LTC2990_ROLE_VOLTAGE)
                                ? (uint8_t)(CTRL_ALL | V1_V2_V3_V4)
                                : (uint8_t)(CTRL_ALL | MODE_DUAL_DIFF);

    if (LTC2990_Set_Mode(handle, control, (uint8_t)(TEMP_MEAS_MODE_MASK | VOLTAGE_MODE_MASK)) != 0) {
        return 1;
    }

    if ((role == LTC2990_ROLE_VOLTAGE) && (LTC2990_Enable_All_Voltages(handle) != 0)) {
        return 1;
    }

    return 0;
}

void LTC2990_Step(LTC2990_Handle_t *handle)
{
    if (handle == TX_NULL) {
        return;
    }

    if (LTC2990_Trigger_Conversion(handle) != 0) {
        return;
    }
    ltc2990_sleep_ms(10U);

    if (handle->role == LTC2990_ROLE_VOLTAGE) {
        const uint8_t regs[4] = {V1_MSB_REG, V2_MSB_REG, V3_MSB_REG, V4_MSB_REG};

        for (uint8_t i = 0U; i < 4U; i++) {
            uint16_t raw15 = 0U;
            int8_t valid = 0;
            if ((LTC2990_ADC_Read_New_Data(handle, regs[i], &raw15, &valid) == 0U) && (valid != 0)) {
                handle->last_values[i] =
                    LTC2990_Code_To_Single_Ended_Voltage(handle, (uint16_t)(raw15 & 0x3FFFU), i);
            } else {
                handle->last_values[i] = NAN;
            }
        }
    } else {
        uint16_t raw15_v1 = 0U;
        uint16_t raw15_v3 = 0U;
        int8_t valid_v1 = 0;
        int8_t valid_v3 = 0;

        handle->last_values[0] = NAN;
        handle->last_values[1] = NAN;
        handle->last_values[2] = NAN;
        handle->last_values[3] = NAN;

        if ((LTC2990_ADC_Read_New_Data(handle, V1_MSB_REG, &raw15_v1, &valid_v1) == 0U) &&
            (valid_v1 != 0)) {
            handle->last_values[0] = LTC2990_Code15_To_CurrentA(raw15_v1);
        }

        if ((LTC2990_ADC_Read_New_Data(handle, V3_MSB_REG, &raw15_v3, &valid_v3) == 0U) &&
            (valid_v3 != 0)) {
            handle->last_values[1] = LTC2990_Code15_To_CurrentA(raw15_v3);
        }
    }
}

void LTC2990_Get_Values(LTC2990_Handle_t *handle, float *values)
{
    if ((handle == TX_NULL) || (values == TX_NULL)) {
        return;
    }

    for (uint8_t i = 0U; i < 4U; i++) {
        values[i] = handle->last_values[i];
    }
}

int8_t LTC2990_Enable_All_Voltages(LTC2990_Handle_t *handle)
{
    return LTC2990_Set_Mode(handle, ENABLE_ALL, TEMP_MEAS_MODE_MASK);
}

int8_t LTC2990_Set_Mode(LTC2990_Handle_t *handle, uint8_t bits_to_set, uint8_t bits_to_clear)
{
    uint8_t reg = 0U;

    if ((handle == TX_NULL) || (LTC2990_Read_Register(handle, CONTROL_REG, &reg) != 0)) {
        return 1;
    }

    reg &= (uint8_t)~bits_to_clear;
    reg |= bits_to_set;

    return LTC2990_Write_Register(handle, CONTROL_REG, reg);
}

int8_t LTC2990_Trigger_Conversion(LTC2990_Handle_t *handle)
{
    return LTC2990_Write_Register(handle, TRIGGER_REG, 0x00U);
}

uint8_t LTC2990_ADC_Read_New_Data(LTC2990_Handle_t *handle,
                                  uint8_t msb_register_address,
                                  uint16_t *raw15,
                                  int8_t *data_valid)
{
    uint32_t timeout = LTC2990_TIMEOUT_MS;
    uint8_t status = 0U;
    uint8_t bit = status_bit_from_msb(msb_register_address);
    uint8_t ready = 0U;

    if ((handle == TX_NULL) || (raw15 == TX_NULL) || (data_valid == TX_NULL) || (bit == 0xFFU)) {
        return 1U;
    }

    while (timeout-- != 0U) {
        if (LTC2990_Read_Register(handle, STATUS_REG, &status) != 0) {
            return 1U;
        }
        if (((status >> bit) & 0x01U) != 0U) {
            ready = 1U;
            break;
        }
        ltc2990_sleep_ms(1U);
    }

    if (ready == 0U) {
        return 1U;
    }

    uint8_t msb = 0U;
    uint8_t lsb = 0U;
    if (LTC2990_Read_Register(handle, msb_register_address, &msb) != 0) {
        return 1U;
    }
    if (LTC2990_Read_Register(handle, (uint8_t)(msb_register_address + 1U), &lsb) != 0) {
        return 1U;
    }

    const uint16_t code = (uint16_t)(((uint16_t)msb << 8) | lsb);
    *data_valid = (int8_t)((code >> 15) & 0x01U);
    *raw15 = (uint16_t)(code & 0x7FFFU);

    return (*data_valid != 0) ? 0U : 1U;
}

float LTC2990_Code_To_Single_Ended_Voltage(LTC2990_Handle_t *handle,
                                           uint16_t code14,
                                           uint8_t channel)
{
    (void)handle;

    float voltage = (float)(code14 & 0x3FFFU) * SINGLE_ENDED_LSB;

    switch (channel) {
    case 0U:
        return voltage * VOLTAGE_DIVIDER_RATIO_12V;
    case 1U:
        return voltage * VOLTAGE_DIVIDER_RATIO_VBATT;
    case 2U:
        return voltage * VOLTAGE_DIVIDER_RATIO_7V;
    case 3U:
        return voltage * VOLTAGE_DIVIDER_RATIO_7V_DUMP;
    default:
        return NAN;
    }
}

float LTC2990_Code15_To_CurrentA(uint16_t raw15)
{
    const float a_per_count = 19.42e-6f / (RSENSE_OHM * CURRENT_DIVIDER_RATIO);
    const uint16_t magnitude = (uint16_t)(raw15 & 0x3FFFU);

    if ((raw15 & 0x4000U) != 0U) {
        return -((float)magnitude + 1.0f) * a_per_count;
    }

    return (float)magnitude * a_per_count;
}

int8_t LTC2990_Read_Register(LTC2990_Handle_t *handle, uint8_t reg_address, uint8_t *data)
{
    if ((handle == TX_NULL) || (handle->hi2c == TX_NULL) || (data == TX_NULL)) {
        return 1;
    }

    i2c_lock(handle);
    const HAL_StatusTypeDef status = HAL_I2C_Mem_Read(handle->hi2c,
                                                       (uint16_t)(handle->i2c_address << 1),
                                                       reg_address,
                                                       I2C_MEMADD_SIZE_8BIT,
                                                       data,
                                                       1U,
                                                       LTC2990_TIMEOUT_MS);
    i2c_unlock(handle);

    return (status == HAL_OK) ? 0 : 1;
}

int8_t LTC2990_Write_Register(LTC2990_Handle_t *handle, uint8_t reg_address, uint8_t data)
{
    if ((handle == TX_NULL) || (handle->hi2c == TX_NULL)) {
        return 1;
    }

    i2c_lock(handle);
    const HAL_StatusTypeDef status = HAL_I2C_Mem_Write(handle->hi2c,
                                                        (uint16_t)(handle->i2c_address << 1),
                                                        reg_address,
                                                        I2C_MEMADD_SIZE_8BIT,
                                                        &data,
                                                        1U,
                                                        LTC2990_TIMEOUT_MS);
    i2c_unlock(handle);

    return (status == HAL_OK) ? 0 : 1;
}

void telemetry_ltc2990_update_voltage(LTC2990_Handle_t *ltc2990_handle)
{
    float voltages[4] = {NAN, NAN, NAN, NAN};

    LTC2990_Step(ltc2990_handle);
    LTC2990_Get_Values(ltc2990_handle, voltages);

    if (!isnan(voltages[1])) {
        (void)log_telemetry_asynchronous(SEDS_DT_BATTERY_VOLTAGE, &voltages[1], 1U, sizeof(float));
    }
}

void telemetry_ltc2990_update_current(LTC2990_Handle_t *ltc2990_handle)
{
    float currents[4] = {NAN, NAN, NAN, NAN};

    LTC2990_Step(ltc2990_handle);
    LTC2990_Get_Values(ltc2990_handle, currents);

    float current = (currents[CURRENT_TELEMETRY_CHANNEL_INDEX] * CURRENT_DRAW_GAIN) +
                    CURRENT_DRAW_OFFSET_A;
    if (!isnan(current)) {
        (void)log_telemetry_asynchronous(SEDS_DT_BATTERY_CURRENT, &current, 1U, sizeof(float));
    }
}
