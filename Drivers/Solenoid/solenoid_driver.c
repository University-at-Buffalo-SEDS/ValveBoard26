#include "solenoid_driver.h"

void solenoidInit(solenoid_t *hw){
    HAL_GPIO_WritePin(hw->sig_port, hw->sig_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hw->en_port,  hw->en_pin,  GPIO_PIN_RESET);
}



int solenoidOn(solenoid_t *hw) {
    if (solenoidFault(hw)) {
        return -1;
    }

    HAL_GPIO_WritePin(hw->en_port,  hw->en_pin,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(hw->sig_port, hw->sig_pin, GPIO_PIN_SET);

    return 0;
}

void solenoidOff(solenoid_t *hw) {
    HAL_GPIO_WritePin(hw->sig_port, hw->sig_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hw->en_port,  hw->en_pin,  GPIO_PIN_RESET);
}

bool solenoidFault(solenoid_t *hw) {
    return HAL_GPIO_ReadPin(hw->fault_port, hw->fault_pin) == GPIO_PIN_RESET;
}