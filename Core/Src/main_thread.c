// main_thread.c
#include "VB-Threads.h"
#include "tx_api.h"
#include "telemetry.h"
#include "can_bus.h"
#include "main.h"
#include "thread_comm.h"
#include "ltc2990.h"
#include "solenoid_driver.h"
#include "servo_driver.h"
#include <stdbool.h>

LTC2990_Handle_t ltc2990_dev;
TX_THREAD main_thread;
#define MAIN_THREAD_STACK_SIZE (16U *1024U)


static uint8_t g_aborted = 0U; 
static uint8_t g_pilot_valve_state = 0U;
static uint8_t g_no_servo_state = 0U;      // 0U = open, 1U = closed
static uint8_t g_dump_servo_state = 0U;    // ^^^
static uint8_t g_continuity_state = 1U;    // 0U = disconnected, 1U = connected 
static uint8_t g_no_signal_state = 0U;     // umbilical
static uint8_t g_dump_signal_state = 0U;   // ^^^
static uint8_t g_pilot_signal_state = 0U;  // ^^^

// Umbilical GPIO inputs 
solenoid_t pilot_solenoid = {Solenoid_GPIO_Port, Solenoid_Pin, Solenoid_GPIO_Port, Solenoid_Pin, NULL, 0, 5};


static void publish_all_umbilical_statuses(void){
    (void)telemetry_publish_umbilical_status(CMD_PILOT_VALVE_OPEN, g_pilot_valve_state);
    (void)telemetry_publish_umbilical_status(CMD_NORMALLY_OPEN_OPEN, g_no_servo_state);
    (void)telemetry_publish_umbilical_status(CMD_DUMP_SERVO_CLOSE, g_dump_servo_state);
    (void)telemetry_publish_umbilical_status(0x08, g_continuity_state);
}

static void handle_command(thread_comm_msg_t cmd){
    switch (cmd) {
    case CMD_PILOT_VALVE_OPEN:
        if (!g_aborted){pilot_valve_on();}
        break;
    case CMD_PILOT_VALVE_CLOSE:
        if (!g_aborted){pilot_valve_off();}
        break;
    case CMD_DUMP_SERVO_CLOSE:
        if (!g_aborted){dump_servo_close();}
        break;
    case CMD_DUMP_SERVO_OPEN:
        if (!g_aborted){dump_servo_open();}
        break;
    case CMD_NORMALLY_OPEN_CLOSE:
        if (!g_aborted){no_servo_close();}
        break;
    case CMD_NORMALLY_OPEN_OPEN:
        if (!g_aborted){no_servo_open();}
        break;
    default:
        break;
    }
}

static void abort_state(void){
    no_servo_open();
    dump_servo_open();
    pilot_valve_off();
    g_aborted = 1U;
    // Blink LEDs to indicate abort
    for(int i = 0; i < 10; i++) {
        HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
        HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
    }
}

void main_thread_entry(ULONG initial_input)
{
    (void)initial_input;

    // Initialize pilot valve solenoid
    solenoidInit(&pilot_solenoid);
    
    // Initialize the PWM for servos
    HAL_TIM_PWM_Start(NO_SERVO_TIMER, NO_SERVO_CHANNEL);
    HAL_TIM_PWM_Start(DUMP_SERVO_TIMER, DUMP_SERVO_CHANNEL);
    
    // Set initial servo positions
    no_servo_open();
    dump_servo_open();
    
    // Initialize LTC2990 
    LTC2990_Init(&ltc2990_dev, &hi2c2, LTC2990_I2C_ADDRESS);
    LTC2990_SetMode(&ltc2990_dev, V1_V2_V3_V4, CLEAR_ALL);
    
    // Initialize router
    (void)init_telemetry_router();

    ULONG last_umbilical_status_ticks = tx_time_get();
    thread_comm_msg_t cmd;
    
    publish_all_umbilical_statuses();
    for (;;) {
        if (thread_comm_get_abort() != 0U)
        {
            if (g_aborted == 0U)
            {
                abort_state();
            }
            while (thread_comm_receive(&msg, TX_NO_WAIT) == TX_SUCCESS){}
            tx_thread_sleep(10);
            continue;
        }
        while (g_aborted != 1U && thread_comm_receive(&msg, TX_NO_WAIT) == TX_SUCCESS)
        {
            handle_command(msg);
        }
        
        can_bus_process_rx();
        (void)telemetry_poll_discovery();
        (void)process_all_queues_timeout(50);
        (void)telemetry_poll_timesync();
    }
}    

UINT create_main_thread(TX_BYTE_POOL *byte_pool)
{

        CHAR *pointer;

  /* Allocate the stack for test  */
  if (tx_byte_allocate(byte_pool, (VOID**) &pointer,
                       MAIN_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

    UINT status = tx_thread_create(&main_thread,
                                   "Main Thread",
                                   main_thread_entry,
                                   0,
                                   pointer,
                                   MAIN_THREAD_STACK_SIZE,
                                   6,
                                   6,
                                   TX_NO_TIME_SLICE,
                                   TX_AUTO_START);

    return status;
}

