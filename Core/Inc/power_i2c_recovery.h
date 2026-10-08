#pragma once
#include "main.h"
#include <stdbool.h>

/* I2C2 is owned exclusively by the power-sensor acquisition thread. */
bool power_i2c_needs_recovery(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef power_i2c_recover(I2C_HandleTypeDef *hi2c);
