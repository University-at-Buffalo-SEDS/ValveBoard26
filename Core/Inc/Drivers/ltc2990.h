#ifndef LTC2990_H
#define LTC2990_H

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_def.h"
#include <math.h>
#include <stdint.h>

#include "main.h"

#define LTC2990_I2C_ADDRESS     0x4C

// register addresses p.15
#define STATUS_REG    0X00
#define CONTROL_REG   0X01
#define TRIGGER_REG   0X02

#define V1_MSB_REG    0X06 // SE 12V, 
#define V1_LSB_REG    0X07
#define V2_MSB_REG    0X08 // SE VBatt, 
#define V2_LSB_REG    0X09
#define V3_MSB_REG    0X0A // SE 7V N.O.
#define V3_LSB_REG    0X0B
#define V4_MSB_REG    0X0C // SE 7V Dump
#define V4_LSB_REG    0X0D
#define VCC_MSB_REG   0X0E // Supply voltage, CCVS
#define VCC_LSB_REG   0X0F

// control modes 
#define V1_V2_V3_V4			(0x1F) //00011111 - Repeat acquisition, all singled ended voltages enabled
#define V1DV2       		(0x5E) //01011001 - Single acquisition, V1-V2 diff voltages enabled 
                                   // V3-V4 diff voltages enabled, but not implemented 
#define CLEAR_ALL			(0xFF) // used for changing modes 

// conversion constants 
#define VEND_LSB_VALUE         0.00030518 // 305.18µV per LSB for single-ended voltages
#define CEND_LSB_VALUE         0.00030518 // 305.18µV per LSB for differential voltage


// voltage divider ratios [(R1 + R2) / R2]
#define VOLTAGE_DIVIDER_RATIO_V1     4.0  // (30k + 10k) / 10k = 4.0
#define VOLTAGE_DIVIDER_RATIO_V2     5.7 // (47k + 10k) / 10k = 5.7
#define VOLTAGE_DIVIDER_RATIO_V3     2.5 // (15k + 10k) / 10k = 2.5
#define VOLTAGE_DIVIDER_RATIO_V4     2.5 // (15k + 10k) / 10k = 2.5

// voltage thresholds for warnings (within a 10% tolerance)
#define V1_THRESHOLD          10.8   // 12V supply warning below 10.8V
#define V2_THRESHOLD          6.3    // 7V supply warning below 6.3V
#define V3_THRESHOLD          6.3    // 7V N.O. warning below 6.3V
#define V4_THRESHOLD          6.3    // 7V Dump warning below 6.3V
#define V1DV2_THRESHOLD       3000.0 // mA high current value 

// reference voltage 
#define REFERENCE_VOLTAGE         3.3 // 3.3V

// resistance for current calcultaion
#define RSENSE            0.02    // Ohm 

#define TIMEOUT           100     // ms 

typedef struct {
    I2C_HandleTypeDef *hi2c;       // I2C handle (using PA8/PA9)
    uint8_t i2c_address;           // LTC2990 I2C address 
    float voltages[4];             // single ended voltage measurements V1-V4
    float differential;            // differential voltage measurement (V1-V2)
    float current;                 // calculated current from differential voltage
    uint32_t time_last_read;       // timestamp of last reading (ms)
} LTC2990_Handle_t;

typedef enum {
    LTC2990_MODE_SINGLE_ENDED = 0,
    LTC2990_MODE_DIFFERENTIAL = 1, 
} LTC2990_Mode_t;

HAL_StatusTypeDef LTC2990_Init(LTC2990_Handle_t *handle, I2C_HandleTypeDef *hi2c, uint8_t address);
HAL_StatusTypeDef LTC2990_SetMode(LTC2990_Handle_t *handle, uint8_t set_bits, uint8_t clear_bits);
HAL_StatusTypeDef LTC2990_TriggerConversion(LTC2990_Handle_t *handle); 
void LTC2990_Step(LTC2990_Handle_t *handle);
HAL_StatusTypeDef LTC2990_ReadADCData(LTC2990_Handle_t *handle, uint8_t msb_reg, uint16_t *adc_code, uint8_t *data_valid);
// single ended
void LTC2990_GetSingleEndedVoltage(LTC2990_Handle_t *dev, uint8_t channel, float *voltage);
float LTC2990_SingleEndedCodeToData(LTC2990_Handle_t *dev, uint16_t adc_code, uint8_t channel); 
float LTC2990_CodeToVoltage(uint16_t adc_code, float lsb_value);

// differential
float LTC2990_GetCurrent(LTC2990_Handle_t *dev);

// i2c communication helpers
HAL_StatusTypeDef LTC2990_WriteRegister(LTC2990_Handle_t *dev, uint8_t reg, uint8_t value);
HAL_StatusTypeDef LTC2990_ReadRegister(LTC2990_Handle_t *dev, uint8_t reg, uint8_t *data);
 
#endif 
