#include "servo_driver.h"

static uint8_t g_no_servo_state = 0U;
static uint8_t g_dump_servo_state = 0U;

#define SERVO_MIN_DEGREES      0U
#define SERVO_TRAVEL_DEGREES   270U
#define SERVO_MIN_PULSE_US     500U
#define SERVO_MAX_PULSE_US     2500U

// Servo control using TIM2 PWM, 
// pulse range is between 500us - 2500us
// Netral position, 1500us, so open is 1000us and closed is 2000us 
void servo_set_position(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse_width_us){ 
    uint32_t period_us = 20000; 
    uint32_t timer_counts = __HAL_TIM_GET_AUTORELOAD(htim) + 1U;
    uint32_t duty_cycle = ((uint32_t)pulse_width_us * timer_counts) / period_us;
    __HAL_TIM_SET_COMPARE(htim, channel, duty_cycle);
}

static uint16_t servo_clamp_degrees(int32_t degrees){
    if (degrees < (int32_t)SERVO_MIN_DEGREES) {
        return SERVO_MIN_DEGREES;
    }

    if (degrees > (int32_t)SERVO_TRAVEL_DEGREES) {
        return SERVO_TRAVEL_DEGREES;
    }

    return (uint16_t)degrees;
}

uint16_t servo_degrees_to_us(uint16_t degrees){
    if (degrees > SERVO_TRAVEL_DEGREES) {
        degrees = SERVO_TRAVEL_DEGREES;
    }

    return (uint16_t)(SERVO_MIN_PULSE_US +
                     (((uint32_t)(degrees - SERVO_MIN_DEGREES) *
                       (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) /
                      (SERVO_TRAVEL_DEGREES - SERVO_MIN_DEGREES)));
}

uint16_t servo_relative_degrees_to_us(int16_t degrees, int16_t zero_degrees){
    int32_t calibrated_degrees;

    calibrated_degrees = (int32_t)zero_degrees + (int32_t)degrees;

    return servo_degrees_to_us(servo_clamp_degrees(calibrated_degrees));
}

void no_servo_open(void){
    servo_set_position(NO_SERVO_TIMER,
                       NO_SERVO_CHANNEL,
                       servo_relative_degrees_to_us(NO_SERVO_OPEN_DEGREES,
                                                     NO_SERVO_ZERO_DEGREES));
    g_no_servo_state = 0U;
    // HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);  // servo open
}

void no_servo_close(void){
    servo_set_position(NO_SERVO_TIMER,
                       NO_SERVO_CHANNEL,
                       servo_relative_degrees_to_us(NO_SERVO_CLOSE_DEGREES,
                                                     NO_SERVO_ZERO_DEGREES));
    g_no_servo_state = 1U;
    // HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET); // servo closed
}

void dump_servo_open(void){
    servo_set_position(DUMP_SERVO_TIMER,
                       DUMP_SERVO_CHANNEL,
                       servo_relative_degrees_to_us(DUMP_SERVO_OPEN_DEGREES,
                                                     DUMP_SERVO_ZERO_DEGREES));
    g_dump_servo_state = 0U;
}

void dump_servo_close(void){
    servo_set_position(DUMP_SERVO_TIMER,
                       DUMP_SERVO_CHANNEL,
                       servo_relative_degrees_to_us(DUMP_SERVO_CLOSE_DEGREES,
                                                     DUMP_SERVO_ZERO_DEGREES));
    g_dump_servo_state = 1U;
}
