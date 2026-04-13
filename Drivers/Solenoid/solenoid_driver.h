#pragma once

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>


typedef struct {
    GPIO_TypeDef *en_port;
    uint16_t      en_pin;
    GPIO_TypeDef *sig_port;
    uint16_t      sig_pin;
    GPIO_TypeDef *fault_port;
    uint16_t      fault_pin;
    uint8_t       adc_channel;
} solenoid_t;

void solenoidInit(solenoid_t *hw);
int solenoidOn(solenoid_t *hw);
void solenoidOff(solenoid_t *hw);
bool solenoidFault(solenoid_t *hw);