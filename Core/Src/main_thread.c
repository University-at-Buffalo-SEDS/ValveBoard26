#include "persistent_store_health.h"
#include "board_watchdog.h"
// main_thread.c
#include "VB-Threads.h"
#include "tx_api.h"
#include "telemetry.h"
#include "status_report_retry.h"
#include "can_bus.h"
#include "main.h"
#include "thread_comm.h"
#include "board_recovery.h"
#include "board_bootstrap_storage.h"
#include "solenoid_driver.h"
#include "servo_driver.h"
#include <stdbool.h>

TX_THREAD main_thread;

/* Rebalance one KiB toward the publishing tasks; measured control headroom
 * exceeded six KiB in the linked flight and network profiles. */
#define MAIN_THREAD_STACK_SIZE (11U * 1024U)
/* GSE requires actual valve-state reports fresher than two seconds. */
#define UMBILICAL_STATUS_PERIOD_TICKS TX_TIMER_TICKS_PER_SECOND
#define LAUNCH_SEQUENCE_PILOT_OPEN_DELAY_MS 10000U
_Static_assert(THREAD_COMM_LAUNCH_EFFECT_DELAY_MS == LAUNCH_SEQUENCE_PILOT_OPEN_DELAY_MS,"Launch timeline delay drift");
#define LAUNCH_SEQUENCE_PILOT_OPEN_DURATION_MS 1500U
#define LAUNCH_SEQUENCE_PILOT_STATUS_PERIOD_TICKS \
    ((TX_TIMER_TICKS_PER_SECOND >= 10U) ? (TX_TIMER_TICKS_PER_SECOND / 10U) : 1U)
#define MAIN_THREAD_MAX_COMMANDS_PER_TICK 4U
#define ABORT_STATUS_PERIOD_TICKS TX_TIMER_TICKS_PER_SECOND
#define ABORT_LED_PERIOD_TICKS ((TX_TIMER_TICKS_PER_SECOND >= 2U) ? (TX_TIMER_TICKS_PER_SECOND / 2U) : 1U)


static uint8_t g_aborted = 0U; 
static uint8_t g_pilot_valve_state = 0U;
static uint8_t g_no_servo_open_state = 0U;
static uint8_t g_nitrous_servo_open_state = 0U;
static uint8_t g_launch_sequence_active = 0U;
static uint32_t g_launch_manual_epoch;
static thread_comm_launch_owner_t g_launch_owner;
static uint8_t g_launch_sequence_actuator_started = 0U;
static uint8_t g_launch_sequence_pilot_opened = 0U;
static uint8_t g_launch_sequence_completed = 0U;
static volatile uint8_t g_outputs_initialized = 0U;
static volatile uint32_t g_launch_sequence_start_count = 0U;
static volatile uint32_t g_launch_sequence_service_count = 0U;
static volatile uint32_t g_launch_sequence_pilot_attempt_count = 0U;
static volatile uint32_t g_launch_sequence_finish_count = 0U;
static volatile uint32_t g_launch_sequence_abort_seen_count = 0U;
volatile uint32_t g_sim_valve_commands_executed = 0U;
volatile uint32_t g_sim_last_valve_command = UINT32_MAX;
volatile uint64_t g_sim_last_valve_command_ms = 0;
volatile uint32_t g_sim_pilot_valve_state = 0U;
volatile uint32_t g_sim_valve_aborted = 0U;
volatile int32_t g_sim_pilot_solenoid_result = 0;
static volatile ULONG g_launch_sequence_last_elapsed_ticks = 0U;
static ULONG g_launch_sequence_start_ticks = 0U;
static ULONG g_launch_sequence_pilot_open_ticks = 0U;
static ULONG g_launch_sequence_last_pilot_status_ticks = 0U;
static ULONG g_abort_last_status_ticks = 0U;
static ULONG g_abort_last_led_ticks = 0U;
static ULONG main_thread_stack[MAIN_THREAD_STACK_SIZE / sizeof(ULONG)];

// Umbilical GPIO inputs 
solenoid_t pilot_solenoid = {Solenoid_GPIO_Port, Solenoid_Pin, Solenoid_GPIO_Port, Solenoid_Pin, NULL, 0, 5};



static ULONG ms_to_ticks(uint32_t ms)
{
    ULONG ticks = (ULONG)(((uint64_t)ms * (uint64_t)TX_TIMER_TICKS_PER_SECOND + 999ULL) / 1000ULL);
    return (ticks == 0U) ? 1U : ticks;
}

static void publish_all_umbilical_statuses(void){
    static uint8_t published_once = 0U;
    static ULONG last_publish_ticks = 0U;
    ULONG now = tx_time_get();

    if ((published_once != 0U) &&
        ((ULONG)(now - last_publish_ticks) < UMBILICAL_STATUS_PERIOD_TICKS))
    {
        return;
    }

    (void)publish_umbilical_status(CMD_PILOT_OPEN, g_pilot_valve_state);
    (void)publish_umbilical_status(CMD_NORMALLY_OPEN_OPEN, g_no_servo_open_state);
    (void)publish_umbilical_status(CMD_DUMP_OPEN, g_nitrous_servo_open_state);
    (void)publish_umbilical_status(CMD_SEQUENCE, g_launch_sequence_active);

    published_once = 1U;
    last_publish_ticks = now;
}

static void publish_abort_umbilical_statuses(void)
{
    (void)publish_umbilical_status(CMD_PILOT_OPEN, g_pilot_valve_state);
    (void)publish_umbilical_status(CMD_NORMALLY_OPEN_OPEN, g_no_servo_open_state);
    (void)publish_umbilical_status(CMD_DUMP_OPEN, g_nitrous_servo_open_state);
    (void)publish_umbilical_status(CMD_SEQUENCE, g_launch_sequence_active);
    g_abort_last_status_ticks = tx_time_get();
}

void pilot_valve_on(void){
    const int result = solenoidOn(&pilot_solenoid);
    g_sim_pilot_solenoid_result = (int32_t)result;
    if (result == 0) {
        g_pilot_valve_state = 1U;
        g_sim_pilot_valve_state = 1U;
    }
    /* A rejected open still needs an immediate actual-state confirmation. */
    (void)publish_umbilical_status(CMD_PILOT_OPEN, g_pilot_valve_state);
}

void pilot_valve_off(void)
{
    solenoidOff(&pilot_solenoid);
    g_pilot_valve_state = 0U;
    g_sim_pilot_valve_state = 0U;
    (void)publish_umbilical_status(CMD_PILOT_OPEN, g_pilot_valve_state);
}

static void nitrous_valve_open(void)
{
    dump_servo_open();
    g_nitrous_servo_open_state = 1U;
    (void)publish_umbilical_status(CMD_DUMP_OPEN, g_nitrous_servo_open_state);
}



static void normally_open_valve_open(void)
{
    no_servo_open();
    g_no_servo_open_state = 1U;
    (void)publish_umbilical_status(CMD_NORMALLY_OPEN_OPEN, g_no_servo_open_state);
}



/* Called under actual shared state owner. Register-only/driver bounded writes. */
static void manual_stop_apply_locked(void)
{
    g_launch_sequence_active=0U;g_launch_sequence_actuator_started=0U;
    g_launch_sequence_pilot_opened=0U;g_launch_sequence_pilot_open_ticks=0U;
    no_servo_open();dump_servo_open();solenoidOff(&pilot_solenoid);
    g_no_servo_open_state=g_nitrous_servo_open_state=1U;g_pilot_valve_state=0U;
    g_sim_pilot_valve_state=0U;
}

static void main_thread_force_outputs_safe_off(void)
{
    g_launch_sequence_active = 0U;
    g_launch_sequence_actuator_started = 0U;
    g_launch_sequence_pilot_opened = 0U;
    g_launch_sequence_pilot_open_ticks = 0U;

    if (g_outputs_initialized == 0U)
    {
        return;
    }

    no_servo_open();
    dump_servo_open();
    solenoidOff(&pilot_solenoid);

    g_no_servo_open_state = 1U;
    g_nitrous_servo_open_state = 1U;
    g_pilot_valve_state = 0U;
}

typedef struct{uint32_t epoch;uint32_t now;uint8_t started;} LaunchStart;
static void start_launch_sequence_locked(void*context){
 LaunchStart*q=context;if(g_launch_sequence_active)return;
 g_launch_manual_epoch=q->epoch;
    g_launch_sequence_active = 1U;
    g_launch_sequence_actuator_started = 1U;
    g_launch_sequence_pilot_opened = 0U;
    g_launch_sequence_completed = 0U;
    g_launch_sequence_start_ticks=q->now;
    g_launch_sequence_pilot_open_ticks = 0U;
    g_launch_sequence_last_pilot_status_ticks = 0U;
    g_launch_sequence_start_count++;
 q->started=1U;
}
static void start_launch_sequence(thread_comm_launch_owner_t owner)
{
    if(g_launch_sequence_active)return;
    /* INITIAL verification and mailbox acceptance already established this
     * exact owner. A SECOND storage failure cannot invent a new Abort. */
    LaunchStart request={owner.epoch,(uint32_t)tx_time_get()+(uint32_t)ms_to_ticks(THREAD_COMM_LAUNCH_EFFECT_DELAY_MS),0U};
    g_launch_owner=owner;
    if(!thread_comm_launch_begin_owned_deadline(&owner,start_launch_sequence_locked,&request,&request.now)||!request.started)return;
#ifdef TELEMETRY_ENABLED
    const uint64_t launch_timestamp_ms=owner.source_utc;
    if(launch_timestamp_ms!=0ULL){
        (void)telemetry_publish_flight_state_at(VALVE_FLIGHT_STATE_LAUNCH,launch_timestamp_ms);
        (void)telemetry_send_actuator_command_at(CMD_IGNITER_SEQUENCE,launch_timestamp_ms);
    }
    /* Source-zero is a local fixture boundary, never a renewed network UTC. */
#endif
    (void)publish_umbilical_status(CMD_SEQUENCE,1U);
}
static void service_launch_sequence_request(void)
{
    thread_comm_launch_owner_t owner={0};
    if(thread_comm_get_abort()!=0U||!thread_comm_take_accepted_launch(&owner))return;
    start_launch_sequence(owner);
}

static void apply_automatic_valve(uint8_t command);
static void service_launch_sequence_locked(void*context)
{
    uint8_t*events=context;
    if (g_launch_sequence_active == 0U)
    {
        return;
    }

    const ULONG now = tx_time_get();
    /* start_ticks is the immutable pilot deadline, not receipt time. */
    const ULONG elapsed = (uint32_t)now - ((uint32_t)g_launch_sequence_start_ticks-(uint32_t)ms_to_ticks(LAUNCH_SEQUENCE_PILOT_OPEN_DELAY_MS));
    g_launch_sequence_service_count++;
    g_launch_sequence_last_elapsed_ticks = elapsed;
    const ULONG pilot_open_duration_ticks =
        ms_to_ticks(LAUNCH_SEQUENCE_PILOT_OPEN_DURATION_MS);

    if ((g_launch_sequence_pilot_opened == 0U) && ((uint32_t)((uint32_t)now-(uint32_t)g_launch_sequence_start_ticks)<UINT32_C(0x80000000)))
    {
        g_launch_sequence_pilot_attempt_count++;
        apply_automatic_valve(CMD_PILOT_OPEN);*events|=1U;
        g_launch_sequence_pilot_opened = 1U;
        g_launch_sequence_pilot_open_ticks = now;
        g_launch_sequence_last_pilot_status_ticks = now;
    }

    if ((g_launch_sequence_pilot_opened != 0U) &&
        ((ULONG)(now - g_launch_sequence_last_pilot_status_ticks) >=
         LAUNCH_SEQUENCE_PILOT_STATUS_PERIOD_TICKS))
    {
        *events|=1U;
        g_launch_sequence_last_pilot_status_ticks = now;
    }

    if ((g_launch_sequence_pilot_opened != 0U) &&
        ((ULONG)(now - g_launch_sequence_pilot_open_ticks) >=
         pilot_open_duration_ticks))
    {
        apply_automatic_valve(CMD_PILOT_CLOSE);*events|=1U;
        g_launch_sequence_active = 0U;
        g_launch_sequence_pilot_opened = 0U;
        g_launch_sequence_pilot_open_ticks = 0U;
        g_launch_sequence_completed = 1U;
        g_launch_sequence_finish_count++;
        *events|=2U;
    }
}
static void service_launch_sequence(void){
 uint8_t events=0U;
 (void)thread_comm_launch_effect_owned(&g_launch_owner,service_launch_sequence_locked,&events);
 if(!g_launch_sequence_active)thread_comm_launch_complete_owned(&g_launch_owner);
 if(events&1U)(void)publish_umbilical_status(CMD_PILOT_OPEN,g_pilot_valve_state);
 if(events&2U)(void)publish_umbilical_status(CMD_SEQUENCE,g_launch_sequence_active);
}

/* No SDK/status publication while the shared launch-state mutex is held. */
static void apply_automatic_valve(uint8_t command)
{
    switch (command) {
    case CMD_PILOT_OPEN:
        g_sim_pilot_solenoid_result = (int32_t)solenoidOn(&pilot_solenoid);
        if (g_sim_pilot_solenoid_result == 0) { g_pilot_valve_state = 1U; g_sim_pilot_valve_state = 1U; }
        break;
    case CMD_PILOT_CLOSE:
        solenoidOff(&pilot_solenoid); g_pilot_valve_state = 0U; g_sim_pilot_valve_state = 0U; break;
    case CMD_DUMP_CLOSE:
        dump_servo_close(); g_nitrous_servo_open_state = 0U; break;
    case CMD_DUMP_OPEN:
        dump_servo_open(); g_nitrous_servo_open_state = 1U; break;
    case CMD_NORMALLY_OPEN_CLOSE:
        no_servo_close(); g_no_servo_open_state = 0U; break;
    case CMD_NORMALLY_OPEN_OPEN:
        no_servo_open(); g_no_servo_open_state = 1U; break;
    default: break;
    }
}

static void apply_manual_valve_locked(void*context){apply_automatic_valve(*(const uint8_t*)context);}
static void handle_command(thread_comm_msg_t cmd,uint32_t command_epoch){
    if(cmd==CMD_ABORT){(void)thread_comm_request_manual_abort();return;}
    if (!persistent_store_controls_ready()) {
        if(!thread_comm_launch_continuity())main_thread_force_outputs_safe_off();
        return;
    }

    if (g_aborted)
    {
        return;
    }
    if ((cmd & THREAD_COMM_AUTOMATIC_TAG) != 0U) {
        if (thread_comm_apply_automatic_owned(cmd,command_epoch,apply_automatic_valve) != 0U) {
            const uint8_t command = (uint8_t)(cmd & ~THREAD_COMM_AUTOMATIC_TAG);
            g_sim_valve_commands_executed++;
            g_sim_last_valve_command = command;
            g_sim_last_valve_command_ms = ((uint64_t)tx_time_get() * 1000ULL) / TX_TIMER_TICKS_PER_SECOND;
            if (command == CMD_PILOT_OPEN || command == CMD_PILOT_CLOSE)
                (void)publish_umbilical_status(CMD_PILOT_OPEN, g_pilot_valve_state);
            else if (command == CMD_DUMP_OPEN || command == CMD_DUMP_CLOSE)
                (void)publish_umbilical_status(CMD_DUMP_OPEN, g_nitrous_servo_open_state);
            else
                (void)publish_umbilical_status(CMD_NORMALLY_OPEN_OPEN, g_no_servo_open_state);
        }
        return;
    }
    g_sim_valve_commands_executed++;
    g_sim_last_valve_command = (uint32_t)cmd;
    g_sim_last_valve_command_ms = ((uint64_t)tx_time_get() * 1000ULL) /
                                  TX_TIMER_TICKS_PER_SECOND;
    const uint8_t valve=(uint8_t)cmd;
    if(cmd==CMD_PILOT_OPEN||cmd==CMD_PILOT_CLOSE||cmd==CMD_DUMP_OPEN||cmd==CMD_DUMP_CLOSE||cmd==CMD_NORMALLY_OPEN_OPEN||cmd==CMD_NORMALLY_OPEN_CLOSE){
      if(thread_comm_manual_effect_owned(0U,command_epoch,apply_manual_valve_locked,(void*)&valve)){
        if(cmd==CMD_PILOT_OPEN||cmd==CMD_PILOT_CLOSE)(void)publish_umbilical_status(CMD_PILOT_OPEN,g_pilot_valve_state);
        else if(cmd==CMD_DUMP_OPEN||cmd==CMD_DUMP_CLOSE)(void)publish_umbilical_status(CMD_DUMP_OPEN,g_nitrous_servo_open_state);
        else (void)publish_umbilical_status(CMD_NORMALLY_OPEN_OPEN,g_no_servo_open_state);
      }
      return;
    }
    switch(cmd){
    case CMD_SEQUENCE:service_launch_sequence_request();break;
    case CMD_ABORT:(void)thread_comm_request_manual_abort();break;
    default:break;
    }
}

static void abort_state(void){
    main_thread_force_outputs_safe_off();
    g_aborted = 1U;
    g_sim_valve_aborted = 1U;
    publish_abort_umbilical_statuses();
    g_abort_last_led_ticks = tx_time_get();
    HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
    HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
}

static void service_abort_state(void)
{
    const ULONG now = tx_time_get();

    if ((ULONG)(now - g_abort_last_status_ticks) >= ABORT_STATUS_PERIOD_TICKS)
    {
        publish_abort_umbilical_statuses();
    }

    if ((ULONG)(now - g_abort_last_led_ticks) >= ABORT_LED_PERIOD_TICKS)
    {
        HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
        HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
        g_abort_last_led_ticks = now;
    }
}


/* Servo compare/GPIO readback is commanded state, not physical position. */
static uint8_t recovery_read_outputs(uint32_t *mask)
{
    if (g_outputs_initialized == 0U) return 0U;
    uint32_t counts = __HAL_TIM_GET_AUTORELOAD(NO_SERVO_TIMER) + 1U;
    uint32_t no_compare = __HAL_TIM_GET_COMPARE(NO_SERVO_TIMER, NO_SERVO_CHANNEL);
    uint32_t dump_compare = __HAL_TIM_GET_COMPARE(DUMP_SERVO_TIMER, DUMP_SERVO_CHANNEL);
    uint32_t no_open = ((uint32_t)servo_relative_degrees_to_us(NO_SERVO_OPEN_DEGREES, NO_SERVO_ZERO_DEGREES) * counts) / 20000U;
    uint32_t no_closed = ((uint32_t)servo_relative_degrees_to_us(NO_SERVO_CLOSE_DEGREES, NO_SERVO_ZERO_DEGREES) * counts) / 20000U;
    uint32_t dump_open = ((uint32_t)servo_relative_degrees_to_us(DUMP_SERVO_OPEN_DEGREES, DUMP_SERVO_ZERO_DEGREES) * counts) / 20000U;
    uint32_t dump_closed = ((uint32_t)servo_relative_degrees_to_us(DUMP_SERVO_CLOSE_DEGREES, DUMP_SERVO_ZERO_DEGREES) * counts) / 20000U;
    if (counts == 0U || (no_compare != no_open && no_compare != no_closed) ||
        (dump_compare != dump_open && dump_compare != dump_closed)) return 0U;
    uint32_t value = (Solenoid_GPIO_Port->ODR & Solenoid_Pin) != 0U ? 1U : 0U;
    if (no_compare == no_open) value |= 2U;
    if (dump_compare == dump_open) value |= 4U;
    *mask = value;
    return BOARD_RECOVERY_OUTPUTS_VALID | (value == 6U ? BOARD_RECOVERY_SAFE_OUTPUTS : 0U) |
        (g_aborted != 0U && g_launch_sequence_active == 0U ? BOARD_RECOVERY_OWNER_STOPPED : 0U);
}

/* Called by main status publication under the nonwaiting shared state owner.
 * Only read registers/RAM; the telemetry owner releases state before SDK work. */
static uint8_t current_status_read_locked(uint8_t id, uint8_t *state)
{
    uint32_t outputs = 0U;
    if (!state || (recovery_read_outputs(&outputs) & BOARD_RECOVERY_OUTPUTS_VALID) == 0U)
        return 0U;
    switch (id) {
    case CMD_PILOT_OPEN: *state = (outputs & 1U) != 0U; break;
    case CMD_NORMALLY_OPEN_OPEN: *state = (outputs & 2U) != 0U; break;
    case CMD_DUMP_OPEN: *state = (outputs & 4U) != 0U; break;
    case CMD_SEQUENCE: *state = g_launch_sequence_active != 0U; break;
    default: return 0U;
    }
    return 1U;
}
static uint8_t current_status_snapshot(uint8_t id, uint8_t *state)
{
    return thread_comm_status_snapshot_try(id, state, current_status_read_locked) == 0;
}
static uint8_t recovery_prepare_outputs(uint8_t desired)
{
    if (g_outputs_initialized == 0U) return 0U;
    if (desired == 0U) {
        no_servo_open();
        dump_servo_open();
        solenoidOff(&pilot_solenoid);
    }
    uint32_t mask;
    return (recovery_read_outputs(&mask) & BOARD_RECOVERY_SAFE_OUTPUTS) != 0U;
}
static void recovery_finish_owner(void)
{
    g_aborted = 0U;
    g_sim_valve_aborted = 0U;
    g_launch_sequence_active = 0U;
    g_launch_sequence_actuator_started = 0U;
    g_launch_sequence_pilot_opened = 0U;
    g_launch_sequence_completed = 0U;
    g_launch_sequence_pilot_open_ticks = 0U;
    g_pilot_valve_state = 0U;
    g_sim_pilot_valve_state = 0U;
    g_no_servo_open_state = g_nitrous_servo_open_state = 1U;
}

/* Executed only by the main control owner for an explicit staged first leg. */
static uint8_t bootstrap_stop_owner(void) {
 if(!recovery_prepare_outputs(0U))return 0U;
 recovery_finish_owner();g_aborted=1U;return 1U;
}
void main_thread_entry(ULONG initial_input)
{
    (void)initial_input;

    (void)thread_comm_bind_manual_stop_apply(manual_stop_apply_locked);
    // Initialize pilot valve solenoid
    solenoidInit(&pilot_solenoid);
    
    // Initialize the PWM for servos
    HAL_TIM_PWM_Start(NO_SERVO_TIMER, NO_SERVO_CHANNEL);
    HAL_TIM_PWM_Start(DUMP_SERVO_TIMER, DUMP_SERVO_CHANNEL);
    g_outputs_initialized = 1U;

    // Set initial valve positions
    pilot_valve_off();
    normally_open_valve_open();
    nitrous_valve_open();
    
    thread_comm_msg_t msg;uint32_t command_epoch=0U;
    
    board_recovery_bind_owner(recovery_read_outputs, recovery_prepare_outputs, recovery_finish_owner);
    (void)board_bootstrap_storage_bind_owner(bootstrap_stop_owner);
    status_report_bind_snapshot(current_status_snapshot);
    publish_all_umbilical_statuses();
    for (;;) {
        board_watchdog_progress(BOARD_WATCHDOG_CONTROL);
        thread_comm_service_board_recovery();
        board_bootstrap_storage_service();
        if (!persistent_store_controls_ready()&&!thread_comm_launch_continuity()) main_thread_force_outputs_safe_off();
        service_launch_sequence_request();

        if (thread_comm_get_abort() != 0U)
        {
            if (g_launch_sequence_active != 0U)
            {
                g_launch_sequence_abort_seen_count++;
            }
            if (g_aborted == 0U)
            {
                abort_state();
            }
            else
            {
                service_abort_state();
            }
            retry_pending_status_reports();
            while (thread_comm_receive(&msg, TX_NO_WAIT) == TX_SUCCESS){}
            tx_thread_sleep(1);
            continue;
        }

        service_launch_sequence();
        service_launch_sequence_request();

        uint32_t commands_drained = 0U;
        while ((g_aborted != 1U) &&
               (thread_comm_get_abort() == 0U) &&
               (commands_drained < MAIN_THREAD_MAX_COMMANDS_PER_TICK) &&
               (thread_comm_receive_owned(&msg,&command_epoch) == TX_SUCCESS))
        {
            handle_command(msg,command_epoch);
            commands_drained++;
        }

        if (thread_comm_get_abort() != 0U)
        {
            continue;
        }

        service_launch_sequence();
        service_launch_sequence_request();
        publish_all_umbilical_statuses();
        retry_pending_status_reports();
        tx_thread_sleep(1);
    }
}    

UINT create_main_thread(TX_BYTE_POOL *byte_pool)
{
    (void)byte_pool;

    UINT status = tx_thread_create(&main_thread,
                                   "Main Thread",
                                   main_thread_entry,
                                   0,
                                   main_thread_stack,
                                   MAIN_THREAD_STACK_SIZE,
                                   6,
                                   6,
                                   TX_NO_TIME_SLICE,
                                   TX_AUTO_START);

    return status;
}
