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

// Pilot valve control
void pilot_valve_on(void){
    if (solenoidOn(&pilot_solenoid) == 0) {
        g_pilot_valve_state = 1U;
        HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_SET);
        (void)telemetry_publish_umbilical_status(CMD_PILOT_VALVE_OPEN, g_pilot_valve_state);
    }
}

void pilot_valve_off(void)
{
    solenoidOff(&pilot_solenoid);
    g_pilot_valve_state = 0U;
    HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
    (void)telemetry_publish_umbilical_status(CMD_PILOT_VALVE_OPEN, g_pilot_valve_state);
}
