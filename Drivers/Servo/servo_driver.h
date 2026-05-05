#pragma once

#include "main.h"

extern TIM_HandleTypeDef htim2;

// PWM servo timer and channels
#define NO_SERVO_TIMER         &htim2
#define NO_SERVO_CHANNEL       TIM_CHANNEL_2
#define NO_SERVO_ZERO_DEGREES  (4)
// Angles are relative to zero: positive is clockwise, negative is counterclockwise.
#define NO_SERVO_OPEN_DEGREES  0
#define NO_SERVO_CLOSE_DEGREES 90

#define DUMP_SERVO_TIMER       &htim2
#define DUMP_SERVO_CHANNEL     TIM_CHANNEL_1
#define DUMP_SERVO_ZERO_DEGREES (2)
// Angles are relative to zero: positive is clockwise, negative is counterclockwise.
#define DUMP_SERVO_OPEN_DEGREES 0
#define DUMP_SERVO_CLOSE_DEGREES (90)

void servo_set_position(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse_width_us); 
uint16_t servo_degrees_to_us(uint16_t degrees);
uint16_t servo_relative_degrees_to_us(int16_t degrees, int16_t zero_degrees);
void no_servo_open(void); 
void no_servo_close(void); 
void dump_servo_open(void); 
void dump_servo_close(void); 
