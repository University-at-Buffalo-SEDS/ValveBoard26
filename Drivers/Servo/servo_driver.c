#include "servo_driver.h"

// Servo control using TIM2 PWM, 
// pulse range is between 500us - 2500us
// Netral position, 1500us, so open is 1000us and closed is 2000us 
void servo_set_position(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse_width_us){ 
    uint32_t period_us = 20000; 
    uint32_t duty_cycle = (pulse_width_us * htim->Init.Period) / period_us;
    __HAL_TIM_SET_COMPARE(htim, channel, duty_cycle);
}

void no_servo_open(void){
    servo_set_position(NO_SERVO_TIMER, NO_SERVO_CHANNEL, 1000);
    g_no_servo_state = 0U;
    HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);  // servo open
}

void no_servo_close(void){
    servo_set_position(NO_SERVO_TIMER, NO_SERVO_CHANNEL, 2000);
    g_no_servo_state = 1U;
    HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET); // servo closed
}

void dump_servo_open(void){
    servo_set_position(DUMP_SERVO_TIMER, DUMP_SERVO_CHANNEL, 1000);
    g_dump_servo_state = 0U;
}

void dump_servo_close(void){
    servo_set_position(DUMP_SERVO_TIMER, DUMP_SERVO_CHANNEL, 2000);
    g_dump_servo_state = 1U;
}