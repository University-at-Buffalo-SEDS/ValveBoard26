#ifndef LTC2990_H
#define LTC2990_H

#include "stm32g4xx_hal.h"
#include "tx_api.h"

#include <stdint.h>

#define STATUS_REG          (0x00)
#define CONTROL_REG         (0x01)
#define TRIGGER_REG         (0x02)

#define V1_MSB_REG          (0x06)
#define V2_MSB_REG          (0x08)
#define V3_MSB_REG          (0x0A)
#define V4_MSB_REG          (0x0C)

#define MODE_DUAL_DIFF      (0x06)
#define VOLTAGE_MODE_MASK   (0x07)
#define TEMP_MEAS_MODE_MASK (0x18)
#define V1_V2_V3_V4         (0x07)
#define ENABLE_ALL          (0x18)
#define CTRL_ALL            (3U << 3)

#define SINGLE_ENDED_LSB    (5.0f / 16384.0f)

#define LTC2990_I2C_ADDRESS_CURRENT (0x4C)
#define LTC2990_I2C_ADDRESS_VOLTAGE (0x4D)

#define VOLTAGE_DIVIDER_RATIO_12V       (4.0f)
#define VOLTAGE_DIVIDER_RATIO_VBATT     (5.7f)
#define VOLTAGE_DIVIDER_RATIO_7V        (2.5f)
#define VOLTAGE_DIVIDER_RATIO_7V_DUMP   (2.5f)

#define RSENSE_OHM                      (0.02f)
#define CURRENT_DIVIDER_TOP_OHM         (71500.0f)
#define CURRENT_DIVIDER_BOTTOM_OHM      (10000.0f)
#define CURRENT_DIVIDER_RATIO           (CURRENT_DIVIDER_BOTTOM_OHM / \
                                         (CURRENT_DIVIDER_TOP_OHM + CURRENT_DIVIDER_BOTTOM_OHM))
#define CURRENT_TELEMETRY_CHANNEL_INDEX (0U)
#define CURRENT_DRAW_POLARITY           (1.0f)
#define CURRENT_DRAW_GAIN               (0.58047616f)
#define CURRENT_DRAW_OFFSET_A           (-1.4267476f)

#define LTC2990_I2C_READY_TRIALS        (2U)
#define LTC2990_I2C_READY_TIMEOUT_MS    (10U)
#define LTC2990_TIMEOUT_MS              (25U)

typedef enum {
    LTC2990_ROLE_VOLTAGE,
    LTC2990_ROLE_CURRENT
} LTC2990_Role_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t i2c_address;
    LTC2990_Role_t role;
    float last_values[4];
    TX_MUTEX *i2c_mutex;
} LTC2990_Handle_t;

int LTC2990_Init(LTC2990_Handle_t *handle,
                 I2C_HandleTypeDef *hi2c,
                 uint8_t address,
                 LTC2990_Role_t role);
void LTC2990_Step(LTC2990_Handle_t *handle);
void LTC2990_Get_Values(LTC2990_Handle_t *handle, float *values);

int8_t LTC2990_Enable_All_Voltages(LTC2990_Handle_t *handle);
int8_t LTC2990_Set_Mode(LTC2990_Handle_t *handle, uint8_t bits_to_set, uint8_t bits_to_clear);
int8_t LTC2990_Trigger_Conversion(LTC2990_Handle_t *handle);
uint8_t LTC2990_ADC_Read_New_Data(LTC2990_Handle_t *handle,
                                  uint8_t msb_register_address,
                                  uint16_t *raw15,
                                  int8_t *data_valid);

float LTC2990_Code_To_Single_Ended_Voltage(LTC2990_Handle_t *handle,
                                           uint16_t code14,
                                           uint8_t channel);
float LTC2990_Code15_To_CurrentA(uint16_t raw15);

int8_t LTC2990_Read_Register(LTC2990_Handle_t *handle, uint8_t reg_address, uint8_t *data);
int8_t LTC2990_Write_Register(LTC2990_Handle_t *handle, uint8_t reg_address, uint8_t data);

void telemetry_ltc2990_update_voltage(LTC2990_Handle_t *ltc2990_handle);
void telemetry_ltc2990_update_current(LTC2990_Handle_t *ltc2990_handle);

#endif
