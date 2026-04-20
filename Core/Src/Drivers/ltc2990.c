#include <Drivers/ltc2990.h>
#include <math.h>

/**
  * @brief  Init LTC2990 
  * @param  dev Pointer to LTC2990 handle
  * @param  hi2c Pointer to I2C handle
  * @param  address I2C address
  * @retval HAL status
  */
HAL_StatusTypeDef LTC2990_Init(LTC2990_Handle_t *dev, I2C_HandleTypeDef *hi2c, uint8_t address) {
    HAL_StatusTypeDef status;
    uint8_t reg_value;
    
    // device connected? 
    if (dev == NULL || hi2c == NULL) {
        return HAL_ERROR;
    }
    
    dev->hi2c = hi2c;
    dev->i2c_address = address;
    
    // initialize voltage array
    for (int i = 0; i < 4; i++) {
        dev->voltages[i] = NAN;
    }
    
    // no differential voltage yet
    dev->differential = NAN;
    dev->time_last_read = 0;
    
    // single-ended mode initially
    status = LTC2990_SetMode(dev, V1_V2_V3_V4, CLEAR_ALL);
    if (status != HAL_OK) {
        return status;
    }

    HAL_Delay(100); // time for mode change
    
    // Verify config
    status = LTC2990_ReadRegister(dev, CONTROL_REG, &reg_value);
    if (status != HAL_OK) {
        return status;
    }
    
    // Check if correctly configured (0x1F for single-ended mode)
    if (reg_value != V1_V2_V3_V4) {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

//--------------------------------------------------------------------------------------------

/**
  * @brief  Change mode by setting and clearing bits in control register
  * @param  dev Pointer to LTC2990 handle
  * @param  set_bits Mode bits to set
  * @param  clear_bits Mode bits to clear
  * @retval HAL status
  */
HAL_StatusTypeDef LTC2990_SetMode(LTC2990_Handle_t *dev, uint8_t set_bits, uint8_t clear_bits) {
    HAL_StatusTypeDef status;
    uint8_t reg_value;
    
    // device? 
    if (dev == NULL) {
        return HAL_ERROR;
    }
    
    status = LTC2990_ReadRegister(dev, CONTROL_REG, &reg_value);
    if (status != HAL_OK) {
        return status;
    }
    
    reg_value &= ~clear_bits; // clear previous mode (will always use CLEAR_ALL)
    reg_value |= set_bits; // set to new mode (use either V1_V2_V3_V4 or V1DV2)
    
    // Write back
    status = LTC2990_WriteRegister(dev, CONTROL_REG, reg_value);
    
    return status;
}

//--------------------------------------------------------------------------------------------

/**
  * @brief  Trigger a new ADC conversion
  * @param  dev Pointer to LTC2990 handle
  * @retval HAL status
  */
HAL_StatusTypeDef LTC2990_TriggerConversion(LTC2990_Handle_t *dev) {
    if (dev == NULL) {
        return HAL_ERROR;
    }

    return LTC2990_WriteRegister(dev, TRIGGER_REG, 0x01);
}

//--------------------------------------------------------------------------------------------

/**
  * @brief  Refresh voltage readings by reading all channels; Should be called periodically in main loop
  * @param  dev Pointer to LTC2990 handle
  */
void LTC2990_Step(LTC2990_Handle_t *dev) {
    HAL_StatusTypeDef status;
    uint16_t adc_code;
    uint8_t data_valid;
    
    if (dev == NULL) {
        return;
    }
    
    // Switch to single-ended mode (V1, V2, V3, V4 enabled)
    status = LTC2990_SetMode(dev, V1_V2_V3_V4, CLEAR_ALL);
    if (status != HAL_OK) {
        return;
    }
    
    HAL_Delay(100); // mode switch
    
    // Trigger conversion
    status = LTC2990_TriggerConversion(dev);
    if (status != HAL_OK) {
        return;
    }
    
    HAL_Delay(100); // conversion
    
    // Read V1
    status = LTC2990_ReadADCData(dev, V1_MSB_REG, &adc_code, &data_valid);
    if (status == HAL_OK && data_valid) {
        dev->voltages[0] = LTC2990_SingleEndedCodeToData(dev, adc_code, 0);
    } else {
        dev->voltages[0] = NAN;
    }
    
    // Read V2
    status = LTC2990_ReadADCData(dev, V2_MSB_REG, &adc_code, &data_valid);
    if (status == HAL_OK && data_valid) {
        dev->voltages[1] = LTC2990_SingleEndedCodeToData(dev, adc_code, 1);
    } else {
        dev->voltages[1] = NAN;
    }
    
    // Read V3
    status = LTC2990_ReadADCData(dev, V3_MSB_REG, &adc_code, &data_valid);
    if (status == HAL_OK && data_valid) {
        dev->voltages[2] = LTC2990_SingleEndedCodeToData(dev, adc_code, 2);
    } else {
        dev->voltages[2] = NAN;
    }
    
    // Read V4
    status = LTC2990_ReadADCData(dev, V4_MSB_REG, &adc_code, &data_valid);
    if (status == HAL_OK && data_valid) {
        dev->voltages[3] = LTC2990_SingleEndedCodeToData(dev, adc_code, 3);
    } else {
        dev->voltages[3] = NAN;
    }
    
    // Switch to differential mode (V1-V2 enabled)
    status = LTC2990_SetMode(dev, V1DV2, CLEAR_ALL);
    if (status != HAL_OK) {
        return;
    }
    
    HAL_Delay(10); // Allow mode to switch
    
    // Trigger conversion
    status = LTC2990_TriggerConversion(dev);
    if (status != HAL_OK) {
        return;
    }
    
    HAL_Delay(100); // Wait for conversion
    
    // Read differential voltage and calculate current
    status = LTC2990_ReadADCData(dev, V1_MSB_REG, &adc_code, &data_valid);
    if (status == HAL_OK && data_valid) {
        dev->differential = LTC2990_CodeToVoltage(adc_code, CEND_LSB_VALUE);
        dev->current = dev->differential / RSENSE;
    } else {
        dev->differential = NAN;
        dev->current = NAN;
    }
    
    // Update timestamp
    dev->time_last_read = HAL_GetTick();
}

//--------------------------------------------------------------------------------------------

/**
  * @brief  Get single-ended voltage for a specific channel
  * @param  dev Pointer to LTC2990 handle
  * @param  channel Channel number (0-3 for V1-V4)
  * @param  voltage Pointer to store the retrieved voltage  
  */
void LTC2990_GetSingleEndedVoltage(LTC2990_Handle_t *dev, uint8_t channel, float *voltage){
    if (dev == NULL || channel > 3 || voltage == NULL) {
        return;
    }
    *voltage = dev->voltages[channel];
}

//--------------------------------------------------------------------------------------------

/**
  * @brief  Convert ADC code to single-ended voltage
  * @param  dev Pointer to LTC2990 handle
  * @param  channel Channel number
  * @retval Voltage value in volts
  */
float LTC2990_SingleEndedCodeToData(LTC2990_Handle_t *dev, uint16_t adc_code, uint8_t channel) {
     float voltage;
    
    if (dev == NULL || channel > 3) {
        return 0.0f;
    }
    
    // Convert ADC code to voltage using LSB value
    voltage = adc_code * VEND_LSB_VALUE;
    
    // voltage divider ratio based on channel
    switch(channel) {
        case 0: // V1 - 12V supply
            voltage *= VOLTAGE_DIVIDER_RATIO_V1;
            break;
        case 1: // V2 - VBatt
            voltage *= VOLTAGE_DIVIDER_RATIO_V2;
            break;
        case 2: // V3 - 7V N.O.
            voltage *= VOLTAGE_DIVIDER_RATIO_V3;
            break;
        case 3: // V4 - 7V Dump
            voltage *= VOLTAGE_DIVIDER_RATIO_V4;
            break;
        default:
            break;
    }
    
    return voltage;
}

//--------------------------------------------------------------------------------------------
/**
  * @brief  Convert ADC code to voltage
  * @param  adc_code ADC code
  * @param  lsb_value Least significant bit value
  * @retval Voltage value in volts
  */
float LTC2990_CodeToVoltage(uint16_t adc_code, float lsb_value) {
    return adc_code * lsb_value;
}

//--------------------------------------------------------------------------------------------

/**
  * @brief  Get current from differential measurement
  * @param  dev Pointer to LTC2990 handle
  * @retval Current in microAmperes
  */
float LTC2990_GetCurrent(LTC2990_Handle_t *dev) {
    HAL_StatusTypeDef status;
    uint16_t adc_code;
    uint8_t data_valid;
    float voltage;
    
    if (dev == NULL) {
        return 0.0f;
    }
    
    // Switch to differential mode
    status = LTC2990_SetMode(dev, V1DV2, CLEAR_ALL);
    if (status != HAL_OK) return 0.0f;
    
    // Trigger conversion
    status = LTC2990_TriggerConversion(dev);
    if (status != HAL_OK) return 0.0f;
    
    HAL_Delay(100);
    
    // Read differential voltage
    status = LTC2990_ReadADCData(dev, V1_MSB_REG, &adc_code, &data_valid);
    if (status != HAL_OK || !data_valid) return 0.0f;
    
    // Convert to current 
    voltage = LTC2990_CodeToVoltage(adc_code, CEND_LSB_VALUE);
    return voltage / RSENSE;
}

//--------------------------------------------------------------------------------------------

/**
  * @brief  Write a value to an LTC2990 register
  * @param  dev Pointer to LTC2990 handle
  * @param  reg Register address
  * @param  value Value to write
  * @retval HAL status
  */
HAL_StatusTypeDef LTC2990_WriteRegister(LTC2990_Handle_t *dev, uint8_t reg, uint8_t value) {
    // device does not exist or 
    if (dev == NULL || dev->hi2c == NULL) {
        return HAL_ERROR;
    }
    
    return HAL_I2C_Mem_Write(dev->hi2c, dev->i2c_address << 1, reg, 
                             I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

/**
  * @brief  Read a value from an LTC2990 register
  * @param  dev Pointer to LTC2990 handle
  * @param  reg Register address
  * @param  data Pointer to store read value
  * @retval HAL status
  */
HAL_StatusTypeDef LTC2990_ReadRegister(LTC2990_Handle_t *dev, uint8_t reg, uint8_t *data) {
    if (dev == NULL || dev->hi2c == NULL || data == NULL) {
        return HAL_ERROR;
    }
    
    return HAL_I2C_Mem_Read(dev->hi2c, dev->i2c_address << 1, reg, 
                            I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}

/**
  * @brief  Read ADC data
  * @param  dev Pointer to LTC2990 handle
  * @param  msb_reg MSB register address
  * @param  adc_code Pointer to store ADC code
  * @retval HAL status
  */
HAL_StatusTypeDef LTC2990_ReadADCData(LTC2990_Handle_t *dev, uint8_t msb_reg, uint16_t *adc_code, uint8_t *data_valid) {
    HAL_StatusTypeDef status;
    uint8_t msb, lsb;
    uint16_t code;
    
    if (dev == NULL || adc_code == NULL || data_valid == NULL) {
        return HAL_ERROR;
    }
    
    // Read MSB and LSB
    status = LTC2990_ReadRegister(dev, msb_reg, &msb);
    if (status != HAL_OK) return status;
    
    status = LTC2990_ReadRegister(dev, msb_reg + 1, &lsb);
    if (status != HAL_OK) return status;
    
    // Check data valid bit (MSB bit 7)
    *data_valid = (msb & 0x80) ? 1 : 0;
    
    // Combine and mask out data valid bit
    code = ((uint16_t)msb << 8) | lsb;
    *adc_code = code & 0x7FFF;
    
    return HAL_OK;
}
