// PWM servo timer and channels
#define NO_SERVO_TIMER         &htim2
#define NO_SERVO_CHANNEL       TIM_CHANNEL_1
#define DUMP_SERVO_TIMER       &htim2
#define DUMP_SERVO_CHANNEL     TIM_CHANNEL_2

void servo_set_position(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse_width_us); 
void no_servo_open(void); 
void no_servo_close(void); 
void dump_servo_open(void); 
void dump_servo_close(void); 