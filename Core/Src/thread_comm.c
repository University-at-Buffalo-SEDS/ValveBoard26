#include "persistent_store_health.h"
#include "thread_comm.h"
#include "board_abort_latch.h"
#include "board_recovery.h"
#include "board_recovery_floor.h"
#include "bootstrap_owner.h"
#include <stdatomic.h>
#include "board_commands.h"
#include "safety_diagnostics.h"

static TX_MUTEX g_thread_comm_state_mutex;
static TX_MUTEX g_thread_comm_ring_mutex;
static TX_SEMAPHORE g_thread_comm_items_sem;
static TX_SEMAPHORE g_thread_comm_spaces_sem;
static thread_comm_msg_t g_thread_comm_ring[THREAD_COMM_QUEUE_DEPTH];
static uint32_t g_thread_comm_head = 0U;
static uint32_t g_thread_comm_tail = 0U;
static uint8_t g_thread_comm_initialized = 0U;
static void (*g_manual_stop_apply)(void);
/* Published after validated storage access OUTSIDE this mutex. No flash IO in
 * queue/physical-owner admission. Never lowered by ordinary producers. */
static uint64_t g_command_cutoff_ms;
static _Atomic uint8_t g_command_admission_fault;
static uint8_t g_thread_comm_abort = 0U;
static uint32_t g_thread_comm_recovery_epoch = 1U;
static uint8_t g_recovery_pending;
/* State-owner publication lease. First diagnostic is never the retry queue. */
volatile safety_abort_diagnostic_t g_safety_first_abort;
static thread_comm_safety_report_t g_safety_report;
static uint32_t g_safety_report_inflight_lease;
static uint32_t g_safety_report_last_lease;
static uint32_t g_safety_report_inflight_cause;
/* 0=open, 1=Launch closing, 2=recovery closing, 3=verified inhibition. */
static uint8_t g_safety_report_closing;

static uint8_t g_recovery_desired;
static uint32_t g_recovery_expected_epoch;
static uint32_t g_recovery_completed_epoch;
static uint8_t (*g_recovery_read_outputs)(uint32_t *);
static uint8_t (*g_recovery_prepare)(uint8_t);
static void (*g_recovery_finish)(void);

typedef struct {uint64_t generation,issuer,source;uint32_t epoch;uint8_t desired,valid;} BootstrapBinding;
static BootstrapBinding g_bootstrap_binding;
static BootstrapOwnerComplete g_bootstrap_complete;
static uint8_t (*g_bootstrap_stop)(void);
static uint8_t g_bootstrap_finalizing;
static bool bootstrap_state_release(void);
static uint8_t safety_launch_inhibit_locked(void);
static void thread_comm_latch_abort_locked(uint8_t new_manual_intent)
{
    const uint8_t new_owner = new_manual_intent != 0U || g_thread_comm_abort == 0U || g_recovery_pending != 0U;
    if (new_owner && g_thread_comm_recovery_epoch != 0U) {
        g_thread_comm_recovery_epoch++;
    }
    g_recovery_pending = 0U;
    if (new_owner) {
        g_safety_report.reason = 0U;
        g_safety_report_closing = 0U;
    }
    board_abort_latch_set();
    g_thread_comm_abort = 1U;
}

static uint8_t g_thread_comm_flight_state = VALVE_FLIGHT_STATE_STARTUP;
static uint64_t g_thread_comm_groundstation_heartbeat_ms = 0ULL;
static uint64_t g_thread_comm_launch_command_timestamp_ms = 0ULL;
static uint8_t g_thread_comm_launch_sequence_pending = 0U;
static int32_t g_thread_comm_shared_value = 0;


static uint8_t safety_launch_inhibit_locked(void)
{
    g_safety_report_closing = 1U;
    g_safety_report.reason = 0U;
    if (g_safety_report_inflight_lease != 0U) return 0U;
    if (!board_launch_latch_set_verified()) {
        /* Actual backup-write failure retains the existing local safe Stop.
         * A draining publisher above never takes this failure path. */
        thread_comm_latch_abort_locked(0U);return 0U;
    }
    g_safety_report_closing = 3U;
    return 1U;
}
void thread_comm_service_safety_publication(void)
{
    if (!g_thread_comm_initialized ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_NO_WAIT) != TX_SUCCESS) return;
    /* Finish inhibition only: this service never queues Launch or actuates. */
    if (g_safety_report_closing == 1U && !g_safety_report_inflight_lease &&
        !g_thread_comm_abort && !g_recovery_pending && persistent_store_controls_ready()) {
        if (!board_launch_latch_set_verified()) {
            thread_comm_latch_abort_locked(0U);g_command_admission_fault = 1U;
        }
        else g_safety_report_closing = 3U;
    }
    if (tx_mutex_put(&g_thread_comm_state_mutex) != TX_SUCCESS)
        g_command_admission_fault = 1U;
}
uint8_t thread_comm_request_local_abort(uint32_t reason, uint64_t at_ms,
    uint64_t heartbeat_ms, uint32_t observed_state, uint32_t detail,
    uint64_t original_utc, uint8_t heartbeat_only)
{
    if (!reason || !g_thread_comm_initialized ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER) != TX_SUCCESS) return 0U;
    uint8_t claimed = 0U;
    if (!g_safety_report_closing && !board_launch_latch_is_set() && !g_thread_comm_abort &&
        g_thread_comm_flight_state > (heartbeat_only ? VALVE_FLIGHT_STATE_IDLE : VALVE_FLIGHT_STATE_STARTUP) &&
        g_thread_comm_flight_state < VALVE_FLIGHT_STATE_LAUNCH) {
        thread_comm_latch_abort_locked(0U);
        safety_record_first_abort(reason, at_ms, heartbeat_ms, observed_state, detail, 0U, original_utc);
        g_safety_report = (thread_comm_safety_report_t){.original_utc=original_utc, .reason=reason, .epoch=g_thread_comm_recovery_epoch};
        claimed = 1U;
    }
    if (tx_mutex_put(&g_thread_comm_state_mutex) != TX_SUCCESS) {
        g_command_admission_fault = 1U;return 0U;
    }
    return claimed;
}
uint8_t thread_comm_safety_report_claim(thread_comm_safety_report_t *out)
{
    if (!out || !g_thread_comm_initialized ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_NO_WAIT) != TX_SUCCESS) return 0U;
    uint8_t claimed = 0U;
    if (!g_safety_report_closing && !g_recovery_pending && !board_launch_latch_is_set() &&
        g_thread_comm_abort && g_safety_report.reason && g_safety_report.epoch &&
        g_safety_report.epoch == g_thread_comm_recovery_epoch && !g_safety_report_inflight_lease &&
        g_safety_report_last_lease != UINT32_MAX) {
        *out = g_safety_report;
        out->lease = ++g_safety_report_last_lease;
        g_safety_report_inflight_lease = out->lease;
        g_safety_report_inflight_cause = out->epoch;
        claimed = 1U;
    }
    if (tx_mutex_put(&g_thread_comm_state_mutex) != TX_SUCCESS) {
        g_command_admission_fault = 1U;return 0U;
    }
    return claimed;
}
void thread_comm_safety_report_finish(uint32_t lease, uint8_t retained)
{
    if (!lease || !g_thread_comm_initialized ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER) != TX_SUCCESS) return;
    if (g_safety_report_inflight_lease == lease) {
        g_safety_report_inflight_lease = 0U;
        if (retained && g_safety_report.epoch == g_safety_report_inflight_cause) g_safety_report.reason = 0U;
    }
    if (tx_mutex_put(&g_thread_comm_state_mutex) != TX_SUCCESS) g_command_admission_fault = 1U;
}

static uint32_t thread_comm_ring_next(uint32_t index)
{
    index++;
    if (index >= THREAD_COMM_QUEUE_DEPTH)
    {
        index = 0U;
    }
    return index;
}

UINT thread_comm_init(TX_BYTE_POOL *byte_pool)
{
    UINT status;

    (void)byte_pool;

    if (g_thread_comm_initialized != 0U)
    {
        return TX_SUCCESS;
    }

    status = tx_mutex_create(&g_thread_comm_state_mutex,
                             "thread_comm_state_mutex",
                             TX_INHERIT);
    if (status != TX_SUCCESS)
    {
        return status;
    }

    status = tx_mutex_create(&g_thread_comm_ring_mutex,
                             "thread_comm_ring_mutex",
                             TX_INHERIT);
    if (status != TX_SUCCESS)
    {
        return status;
    }

    status = tx_semaphore_create(&g_thread_comm_items_sem,
                                 "thread_comm_items_sem",
                                 0U);
    if (status != TX_SUCCESS)
    {
        return status;
    }

    status = tx_semaphore_create(&g_thread_comm_spaces_sem,
                                 "thread_comm_spaces_sem",
                                 THREAD_COMM_QUEUE_DEPTH);
    if (status != TX_SUCCESS)
    {
        return status;
    }

    g_thread_comm_head = 0U;
    g_thread_comm_tail = 0U;
    g_thread_comm_abort = board_abort_latch_is_set();
    g_thread_comm_flight_state = VALVE_FLIGHT_STATE_STARTUP;
    g_thread_comm_groundstation_heartbeat_ms = 0ULL;
    g_thread_comm_launch_command_timestamp_ms = 0ULL;
    g_thread_comm_launch_sequence_pending = 0U;
    g_thread_comm_shared_value = 0;
    g_safety_report = (thread_comm_safety_report_t){0};
    g_safety_report_inflight_lease = g_safety_report_inflight_cause = g_safety_report_last_lease = 0U;
    g_safety_report_closing = 0U;
    g_thread_comm_initialized = 1U;

    return TX_SUCCESS;
}



/* Caller already owns state and one spaces permit. No nested state lock. */
static UINT thread_comm_commit_reserved_locked(thread_comm_msg_t msg,uint8_t*committed)
{
 *committed=0U;UINT status;
 if(tx_mutex_get(&g_thread_comm_ring_mutex,TX_NO_WAIT)!=TX_SUCCESS)status=TX_MUTEX_ERROR;
 else{
  g_thread_comm_ring[g_thread_comm_head]=msg;g_thread_comm_head=thread_comm_ring_next(g_thread_comm_head);*committed=1U;
  status=tx_mutex_put(&g_thread_comm_ring_mutex);
  if(status==TX_SUCCESS)status=tx_semaphore_put(&g_thread_comm_items_sem);
 }
 if(!*committed)(void)tx_semaphore_put(&g_thread_comm_spaces_sem);
 if(status!=TX_SUCCESS&&*committed){g_command_admission_fault=1U;status=THREAD_COMM_COMMIT_UNCERTAIN;}
 return status;
}
static UINT thread_comm_send_owned_nowait(thread_comm_msg_t msg)
{
 if(!persistent_store_controls_ready())return TX_QUEUE_ERROR;
 UINT status=tx_semaphore_get(&g_thread_comm_spaces_sem,TX_NO_WAIT);
 if(status!=TX_SUCCESS)return status;
 uint8_t committed;return thread_comm_commit_reserved_locked(msg,&committed);
}
UINT thread_comm_send(thread_comm_msg_t msg,ULONG wait_option)
{
 if(!g_thread_comm_initialized||!persistent_store_controls_ready())return TX_QUEUE_ERROR;
 if(tx_mutex_get(&g_thread_comm_state_mutex,TX_NO_WAIT)!=TX_SUCCESS)return TX_MUTEX_ERROR;
 const uint32_t original_epoch=g_thread_comm_recovery_epoch;
 if(tx_mutex_put(&g_thread_comm_state_mutex)!=TX_SUCCESS)return TX_MUTEX_ERROR;
 /* A potentially waiting local producer owns no state/ring mutex here. */
 UINT status=tx_semaphore_get(&g_thread_comm_spaces_sem,wait_option);
 if(status!=TX_SUCCESS)return status;
 if(tx_mutex_get(&g_thread_comm_state_mutex,TX_NO_WAIT)!=TX_SUCCESS){(void)tx_semaphore_put(&g_thread_comm_spaces_sem);return TX_MUTEX_ERROR;}
 uint8_t committed=0U;
 if(!original_epoch||original_epoch==UINT32_MAX||original_epoch!=g_thread_comm_recovery_epoch||!persistent_store_controls_ready()){
  (void)tx_semaphore_put(&g_thread_comm_spaces_sem);status=THREAD_COMM_STALE_TIMESTAMP;
 }else status=thread_comm_commit_reserved_locked(msg,&committed);
 if(tx_mutex_put(&g_thread_comm_state_mutex)!=TX_SUCCESS){g_command_admission_fault=1U;return committed?THREAD_COMM_COMMIT_UNCERTAIN:TX_MUTEX_ERROR;}
 return status;
}
static uint8_t network_timestamp_allowed_locked(uint64_t timestamp_ms)
{
    return persistent_store_controls_ready() &&
        (g_command_cutoff_ms == 0U || timestamp_ms > g_command_cutoff_ms);
}
UINT thread_comm_publish_command_cutoff(uint64_t cutoff_ms)
{
    if (cutoff_ms == 0U || g_thread_comm_initialized == 0U ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_NO_WAIT) != TX_SUCCESS)
        return TX_MUTEX_ERROR;
    UINT result = TX_SUCCESS;
    if (cutoff_ms < g_command_cutoff_ms) result = THREAD_COMM_STALE_TIMESTAMP;
    else g_command_cutoff_ms = cutoff_ms;
    (void)tx_mutex_put(&g_thread_comm_state_mutex);
    return result;
}
UINT thread_comm_admit_network(thread_comm_msg_t msg, uint64_t timestamp_ms)
{
    if (g_thread_comm_initialized == 0U ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_NO_WAIT) != TX_SUCCESS)
        return TX_MUTEX_ERROR;
    UINT result;
    if (g_command_admission_fault) result = THREAD_COMM_COMMIT_UNCERTAIN;
    else if (g_thread_comm_abort || g_recovery_pending || !network_timestamp_allowed_locked(timestamp_ms))
        result = THREAD_COMM_STALE_TIMESTAMP;
    else result = thread_comm_send_owned_nowait(msg);
    (void)tx_mutex_put(&g_thread_comm_state_mutex);
    return result;
}

static UINT thread_comm_send_urgent_locked(thread_comm_msg_t msg)
{
    UINT status;

    if (g_thread_comm_initialized == 0U)
    {
        return TX_QUEUE_ERROR;
    }

    status = tx_semaphore_get(&g_thread_comm_spaces_sem, TX_NO_WAIT);
    if (status != TX_SUCCESS)
    {
        status = tx_semaphore_get(&g_thread_comm_items_sem, TX_NO_WAIT);
        if (status != TX_SUCCESS)
        {
            return status;
        }

        status = tx_mutex_get(&g_thread_comm_ring_mutex, TX_NO_WAIT);
        if (status != TX_SUCCESS)
        {
            (void)tx_semaphore_put(&g_thread_comm_items_sem);
            return status;
        }

        g_thread_comm_tail = thread_comm_ring_next(g_thread_comm_tail);

        status = tx_mutex_put(&g_thread_comm_ring_mutex);
        if (status != TX_SUCCESS)
        {
            return status;
        }

        (void)tx_semaphore_put(&g_thread_comm_spaces_sem);

        status = tx_semaphore_get(&g_thread_comm_spaces_sem, TX_NO_WAIT);
        if (status != TX_SUCCESS)
        {
            return status;
        }
    }

    status = tx_mutex_get(&g_thread_comm_ring_mutex, TX_NO_WAIT);
    if (status != TX_SUCCESS)
    {
        (void)tx_semaphore_put(&g_thread_comm_spaces_sem);
        return status;
    }

    g_thread_comm_ring[g_thread_comm_head] = msg;
    g_thread_comm_head = thread_comm_ring_next(g_thread_comm_head);

    status = tx_mutex_put(&g_thread_comm_ring_mutex);
    if (status != TX_SUCCESS)
    {
        return status;
    }

    return tx_semaphore_put(&g_thread_comm_items_sem);
}
UINT thread_comm_send_urgent(thread_comm_msg_t msg){
 if(!g_thread_comm_initialized||tx_mutex_get(&g_thread_comm_state_mutex,TX_NO_WAIT)!=TX_SUCCESS)return TX_MUTEX_ERROR;
 UINT status=thread_comm_send_urgent_locked(msg);
 if(tx_mutex_put(&g_thread_comm_state_mutex)!=TX_SUCCESS){g_command_admission_fault=1U;return THREAD_COMM_COMMIT_UNCERTAIN;}
 return status;
}

UINT thread_comm_receive(thread_comm_msg_t *msg, ULONG wait_option)
{
    UINT status;

    if ((g_thread_comm_initialized == 0U) || (msg == TX_NULL))
    {
        return TX_QUEUE_ERROR;
    }

    status = tx_semaphore_get(&g_thread_comm_items_sem, wait_option);
    if (status != TX_SUCCESS)
    {
        return status;
    }

    status = tx_mutex_get(&g_thread_comm_ring_mutex, wait_option);
    if (status != TX_SUCCESS)
    {
        (void)tx_semaphore_put(&g_thread_comm_items_sem);
        return status;
    }

    *msg = g_thread_comm_ring[g_thread_comm_tail];
    g_thread_comm_tail = thread_comm_ring_next(g_thread_comm_tail);

    status = tx_mutex_put(&g_thread_comm_ring_mutex);
    if (status != TX_SUCCESS)
    {
        return status;
    }

    return tx_semaphore_put(&g_thread_comm_spaces_sem);
}

/* Only automatic producers use this atomic admission decision. Manual Abort
 * remains unconditional, and ordinary state rollback cannot clear launch. */
uint8_t thread_comm_request_automatic_abort(void)
{
    uint8_t claimed = 0U;
    if (g_thread_comm_initialized == 0U ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER) != TX_SUCCESS) {
        return 0U;
    }
    if (g_safety_report_closing == 0U && board_launch_latch_is_set() == 0U && g_thread_comm_abort == 0U &&
        g_thread_comm_flight_state > VALVE_FLIGHT_STATE_STARTUP &&
        g_thread_comm_flight_state < VALVE_FLIGHT_STATE_LAUNCH) {
        thread_comm_latch_abort_locked(0U);
        claimed = 1U;
    }
    (void)tx_mutex_put(&g_thread_comm_state_mutex);
    return claimed;
}

/* Heartbeat loss alone is inert in Idle. Recheck current state and the
 * retained launch latch atomically with the existing Abort owner. */
uint8_t thread_comm_request_heartbeat_abort(void)
{
    uint8_t claimed = 0U;
    if (g_thread_comm_initialized == 0U ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER) != TX_SUCCESS) {
        return 0U;
    }
    if (g_safety_report_closing == 0U && board_launch_latch_is_set() == 0U && g_thread_comm_abort == 0U &&
        g_thread_comm_flight_state > VALVE_FLIGHT_STATE_IDLE &&
        g_thread_comm_flight_state < VALVE_FLIGHT_STATE_LAUNCH) {
        thread_comm_latch_abort_locked(0U);
        claimed = 1U;
    }
    (void)tx_mutex_put(&g_thread_comm_state_mutex);
    return claimed;
}

/* Every newly delivered manual intent invalidates an older query/ticket, even
 * if abort is already latched. Periodic setters and automatic repeats do not. */
UINT thread_comm_request_manual_abort_original_utc(uint64_t source)
{
    if (g_thread_comm_initialized == 0U) return TX_MUTEX_ERROR;
    UINT status = tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER);
    if (status != TX_SUCCESS) return status;
    if (!board_recovery_manual_time_retired(source)) {
        /* Existing epoch/latch retirement precedes every physical effect. */
        thread_comm_latch_abort_locked(1U);
        if (g_manual_stop_apply) g_manual_stop_apply();
    }
    return tx_mutex_put(&g_thread_comm_state_mutex);
}

UINT thread_comm_set_abort(uint8_t abort_requested)
{
    UINT status;

    if (g_thread_comm_initialized == 0U)
    {
        return TX_MUTEX_ERROR;
    }

    status = tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER);
    if (status != TX_SUCCESS)
    {
        return status;
    }

    if (abort_requested != 0U)
    {
        thread_comm_latch_abort_locked(0U);
    }

    status = tx_mutex_put(&g_thread_comm_state_mutex);
    return status;
}

uint8_t thread_comm_get_abort(void)
{
    UINT status;
    uint8_t abort_requested = 0U;

    if (g_thread_comm_initialized == 0U)
    {
        return 0U;
    }

    status = tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER);
    if (status == TX_SUCCESS)
    {
        abort_requested = g_thread_comm_abort;
        (void)tx_mutex_put(&g_thread_comm_state_mutex);
    }

    return abort_requested;
}

UINT thread_comm_set_shared_value(int32_t value)
{
    UINT status;

    if (g_thread_comm_initialized == 0U)
    {
        return TX_MUTEX_ERROR;
    }

    status = tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER);
    if (status != TX_SUCCESS)
    {
        return status;
    }

    g_thread_comm_shared_value = value;

    status = tx_mutex_put(&g_thread_comm_state_mutex);
    return status;
}

int32_t thread_comm_get_shared_value(void)
{
    UINT status;
    int32_t value = 0;

    if (g_thread_comm_initialized == 0U)
    {
        return 0;
    }

    status = tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER);
    if (status == TX_SUCCESS)
    {
        value = g_thread_comm_shared_value;
        (void)tx_mutex_put(&g_thread_comm_state_mutex);
    }

    return value;
}

UINT thread_comm_set_flight_state(uint8_t flight_state)
{
    UINT status;

    if (g_thread_comm_initialized == 0U)
    {
        return TX_MUTEX_ERROR;
    }

    status = tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER);
    if (status != TX_SUCCESS)
    {
        return status;
    }

    if (!persistent_store_controls_ready()) {
        (void)tx_mutex_put(&g_thread_comm_state_mutex);
        return TX_QUEUE_ERROR;
    }
    if (flight_state >= VALVE_FLIGHT_STATE_LAUNCH && flight_state <= VALVE_FLIGHT_STATE_RECOVERY) {
        if (g_thread_comm_abort != 0U || safety_launch_inhibit_locked() == 0U) {
            (void)tx_mutex_put(&g_thread_comm_state_mutex);
            return TX_QUEUE_ERROR;
        }
    }
    g_thread_comm_flight_state = flight_state;

    return tx_mutex_put(&g_thread_comm_state_mutex);
}

static UINT thread_comm_apply_managed_flight_state_impl(uint8_t flight_state, uint64_t timestamp_ms, uint8_t network)
{
    if (g_thread_comm_initialized == 0U) return TX_MUTEX_ERROR;
    UINT status = tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER);
    if (status != TX_SUCCESS) return status;
    if (!persistent_store_controls_ready()) {
        (void)tx_mutex_put(&g_thread_comm_state_mutex);
        return TX_QUEUE_ERROR;
    }
    if (network && (g_command_admission_fault || !network_timestamp_allowed_locked(timestamp_ms))) {
        const UINT result = g_command_admission_fault ? THREAD_COMM_COMMIT_UNCERTAIN : THREAD_COMM_STALE_TIMESTAMP;
        (void)tx_mutex_put(&g_thread_comm_state_mutex);return result;
    }
    /* Launch and this decision share the same mutex. Delayed managed Aborted
     * has no manual-emergency provenance and cannot overwrite actual flight. */
    if (g_recovery_pending != 0U || g_thread_comm_abort != 0U ||
        (board_launch_latch_is_set() != 0U &&
         (flight_state == VALVE_FLIGHT_STATE_ABORTED ||
          flight_state < g_thread_comm_flight_state))) {
        (void)tx_mutex_put(&g_thread_comm_state_mutex);
        return TX_QUEUE_ERROR;
    }
    /* Managed progression is metadata, not another physical Launch admission.
     * Once inhibition completed, later backup loss cannot create a NEW Stop.
     * Initial/physical admissions still use the checked write independently. */
    if (flight_state >= VALVE_FLIGHT_STATE_LAUNCH &&
        flight_state <= VALVE_FLIGHT_STATE_RECOVERY &&
        g_safety_report_closing != 3U && !board_launch_latch_was_verified() &&
        safety_launch_inhibit_locked() == 0U) {
        (void)tx_mutex_put(&g_thread_comm_state_mutex);
        return TX_QUEUE_ERROR;
    }
    g_thread_comm_flight_state = flight_state;
    if (flight_state == VALVE_FLIGHT_STATE_ABORTED) thread_comm_latch_abort_locked(0U);
    return tx_mutex_put(&g_thread_comm_state_mutex);
}

uint8_t thread_comm_get_flight_state(void)
{
    UINT status;
    uint8_t flight_state = VALVE_FLIGHT_STATE_STARTUP;

    if (g_thread_comm_initialized == 0U)
    {
        return VALVE_FLIGHT_STATE_STARTUP;
    }

    status = tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER);
    if (status == TX_SUCCESS)
    {
        flight_state = g_thread_comm_flight_state;
        (void)tx_mutex_put(&g_thread_comm_state_mutex);
    }

    return flight_state;
}

uint8_t thread_comm_abort_allowed(void)
{
    const uint8_t flight_state = thread_comm_get_flight_state();
    return ((board_launch_latch_is_set() == 0U) &&
            (flight_state > VALVE_FLIGHT_STATE_STARTUP) &&
            (flight_state < VALVE_FLIGHT_STATE_LAUNCH))
               ? 1U
               : 0U;
}

/* Advisory only; the heartbeat claim rechecks under the state owner. */
uint8_t thread_comm_heartbeat_abort_allowed(void)
{
    const uint8_t flight_state = thread_comm_get_flight_state();
    return ((board_launch_latch_is_set() == 0U) &&
            (flight_state > VALVE_FLIGHT_STATE_IDLE) &&
            (flight_state < VALVE_FLIGHT_STATE_LAUNCH))
               ? 1U
               : 0U;
}

UINT thread_comm_apply_managed_flight_state(uint8_t state) { return thread_comm_apply_managed_flight_state_impl(state,0U,0U); }
UINT thread_comm_apply_network_flight_state(uint8_t state,uint64_t timestamp_ms) { return thread_comm_apply_managed_flight_state_impl(state,timestamp_ms,1U); }

UINT thread_comm_set_launch_command_timestamp_ms(uint64_t timestamp_ms)
{
    UINT status;

    if (g_thread_comm_initialized == 0U)
    {
        return TX_MUTEX_ERROR;
    }

    status = tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER);
    if (status != TX_SUCCESS)
    {
        return status;
    }

    g_thread_comm_launch_command_timestamp_ms = timestamp_ms;

    return tx_mutex_put(&g_thread_comm_state_mutex);
}

uint64_t thread_comm_get_launch_command_timestamp_ms(void)
{
    UINT status;
    uint64_t timestamp_ms = 0ULL;

    if (g_thread_comm_initialized == 0U)
    {
        return 0ULL;
    }

    status = tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER);
    if (status == TX_SUCCESS)
    {
        timestamp_ms = g_thread_comm_launch_command_timestamp_ms;
        (void)tx_mutex_put(&g_thread_comm_state_mutex);
    }

    return timestamp_ms;
}

static UINT thread_comm_request_launch_sequence_impl(uint64_t timestamp_ms, uint8_t network)
{
    UINT status;

    if (g_thread_comm_initialized == 0U)
    {
        return TX_MUTEX_ERROR;
    }

    status = tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER);
    if (status != TX_SUCCESS)
    {
        return status;
    }

    if (network && (g_command_admission_fault || g_recovery_pending ||
                    !network_timestamp_allowed_locked(timestamp_ms))) {
        const UINT result = g_command_admission_fault ? THREAD_COMM_COMMIT_UNCERTAIN : THREAD_COMM_STALE_TIMESTAMP;
        (void)tx_mutex_put(&g_thread_comm_state_mutex);return result;
    }
    if (g_thread_comm_abort != 0U || !persistent_store_controls_ready())
    {
        (void)tx_mutex_put(&g_thread_comm_state_mutex);
        return TX_QUEUE_ERROR;
    }

    if (safety_launch_inhibit_locked() == 0U) {
        g_thread_comm_launch_sequence_pending = 0U;
        (void)tx_mutex_put(&g_thread_comm_state_mutex);
        return TX_QUEUE_ERROR;
    }
    g_thread_comm_launch_command_timestamp_ms = timestamp_ms;
    g_thread_comm_launch_sequence_pending = 1U;

    return tx_mutex_put(&g_thread_comm_state_mutex);
}

UINT thread_comm_request_launch_sequence(uint64_t timestamp_ms) {return thread_comm_request_launch_sequence_impl(timestamp_ms,0U);}
UINT thread_comm_request_network_launch_sequence(uint64_t timestamp_ms) {return thread_comm_request_launch_sequence_impl(timestamp_ms,1U);}

uint8_t thread_comm_take_launch_sequence_request(uint64_t *timestamp_ms)
{
    UINT status;
    uint8_t pending = 0U;

    if (timestamp_ms != TX_NULL)
    {
        *timestamp_ms = 0ULL;
    }

    if (g_thread_comm_initialized == 0U)
    {
        return 0U;
    }

    status = tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER);
    if (status == TX_SUCCESS)
    {
        pending = persistent_store_controls_ready() ? g_thread_comm_launch_sequence_pending : 0U;
        if (pending != 0U)
        {
            if (timestamp_ms != TX_NULL)
            {
                *timestamp_ms = g_thread_comm_launch_command_timestamp_ms;
            }
            g_thread_comm_launch_sequence_pending = 0U;
        }
        (void)tx_mutex_put(&g_thread_comm_state_mutex);
    }

    return pending;
}

UINT thread_comm_note_groundstation_heartbeat(uint64_t timestamp_ms)
{
    UINT status;

    if (g_thread_comm_initialized == 0U)
    {
        return TX_MUTEX_ERROR;
    }

    status = tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER);
    if (status != TX_SUCCESS)
    {
        return status;
    }

    g_thread_comm_groundstation_heartbeat_ms = timestamp_ms;

    return tx_mutex_put(&g_thread_comm_state_mutex);
}

uint64_t thread_comm_get_groundstation_heartbeat_ms(void)
{
    UINT status;
    uint64_t timestamp_ms = 0ULL;

    if (g_thread_comm_initialized == 0U)
    {
        return 0ULL;
    }

    status = tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER);
    if (status == TX_SUCCESS)
    {
        timestamp_ms = g_thread_comm_groundstation_heartbeat_ms;
        (void)tx_mutex_put(&g_thread_comm_state_mutex);
    }

    return timestamp_ms;
}

/* Atomic against launch admission/state updates. A suppressed stale automatic
 * request is acknowledged without queueing or changing outputs. Genuine queue
 * refusal remains visible so reliable delivery may retry before launch. */
static UINT thread_comm_admit_automatic_impl(thread_comm_msg_t msg, uint64_t timestamp_ms, uint8_t network)
{
    if (!(msg <= CMD_DUMP_CLOSE)) return TX_QUEUE_ERROR;
    if (g_thread_comm_initialized == 0U ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER) != TX_SUCCESS) {
        return TX_MUTEX_ERROR;
    }
    UINT status = TX_SUCCESS;
    if (network && (g_command_admission_fault || g_recovery_pending ||
                    !network_timestamp_allowed_locked(timestamp_ms))) {
        status = g_command_admission_fault ? THREAD_COMM_COMMIT_UNCERTAIN : THREAD_COMM_STALE_TIMESTAMP;
        (void)tx_mutex_put(&g_thread_comm_state_mutex);return status;
    }
    if (persistent_store_controls_ready() && board_launch_latch_is_set() == 0U && g_thread_comm_abort == 0U) {
        msg |= THREAD_COMM_AUTOMATIC_TAG;
        status = thread_comm_send_owned_nowait(msg);
    }
    (void)tx_mutex_put(&g_thread_comm_state_mutex);
    return status;
}

UINT thread_comm_admit_automatic(thread_comm_msg_t msg) {
    return thread_comm_admit_automatic_impl(msg, 0U, 0U);
}
UINT thread_comm_admit_network_automatic(thread_comm_msg_t msg, uint64_t timestamp_ms) {
    return thread_comm_admit_automatic_impl(msg, timestamp_ms, 1U);
}

uint8_t thread_comm_apply_automatic(thread_comm_msg_t msg, void (*apply)(uint8_t))
{
    const uint8_t command = (uint8_t)(msg & ~THREAD_COMM_AUTOMATIC_TAG);
    if (msg > UINT8_MAX || (msg & THREAD_COMM_AUTOMATIC_TAG) == 0U || apply == 0 ||
        !(command <= CMD_DUMP_CLOSE) || g_thread_comm_initialized == 0U ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER) != TX_SUCCESS) return 0U;
    uint8_t applied = 0U;
    if (persistent_store_controls_ready() && board_launch_latch_is_set() == 0U && g_thread_comm_abort == 0U) {
        apply(command);
        applied = 1U;
    }
    (void)tx_mutex_put(&g_thread_comm_state_mutex);
    return applied;
}

/* These hooks deliberately do not register a network command. The authorized
 * wire handler must install its stale-command floor before submitting recovery. */
static void board_recovery_snapshot_locked(BoardRecoverySnapshot *out)
{
    out->actual_state = g_thread_comm_flight_state;
    out->flags = BOARD_RECOVERY_STATE_VALID;
    out->reserved = 0U;
    out->abort_epoch = g_thread_comm_recovery_epoch;
    out->actual_outputs_mask = 0U;
    if (g_thread_comm_abort != 0U) out->flags |= BOARD_RECOVERY_ABORT_LATCHED;
    if (g_recovery_read_outputs != 0 && g_recovery_prepare != 0 &&
        g_recovery_finish != 0 && g_thread_comm_recovery_epoch != 0U &&
        g_thread_comm_recovery_epoch != UINT32_MAX) {
        out->flags |= BOARD_RECOVERY_SUPPORTED;
        out->flags |= g_recovery_read_outputs(&out->actual_outputs_mask) &
            (BOARD_RECOVERY_OUTPUTS_VALID | BOARD_RECOVERY_SAFE_OUTPUTS);
    }
}
void board_recovery_bind_owner(uint8_t (*read_outputs)(uint32_t *),
                              uint8_t (*prepare)(uint8_t), void (*finish)(void))
{
    if (g_thread_comm_initialized == 0U ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER) != TX_SUCCESS) return;
    g_recovery_read_outputs = read_outputs;
    g_recovery_prepare = prepare;
    g_recovery_finish = finish;
    (void)tx_mutex_put(&g_thread_comm_state_mutex);
}
int32_t thread_comm_status_snapshot_try(uint8_t id, uint8_t *state,
                                      thread_comm_status_read_t read)
{
    if (!state || !read) return -10;
    if (!g_thread_comm_initialized ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_NO_WAIT) != TX_SUCCESS) return -14;
    const uint8_t observed = read(id, state);
    if (!bootstrap_state_release()) return -9;
    if (g_command_admission_fault || !persistent_store_controls_ready()) return -9;
    return observed ? 0 : -10;
}
int32_t board_recovery_snapshot(BoardRecoverySnapshot *out)
{
    if (out == 0) return -10;
    if (g_thread_comm_initialized == 0U ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER) != TX_SUCCESS) return -14;
    board_recovery_snapshot_locked(out);
    if(!bootstrap_state_release())return -9;
    return (g_command_admission_fault || !persistent_store_controls_ready()) ? -9 : 0;
}
/* PREPARING is not executable by the control owner. It closes managed-state
 * application and lets a fresh abort invalidate the expected epoch during NVM. */
int32_t board_recovery_begin_prepare(uint8_t desired_state, uint32_t expected_epoch)
{
    if (desired_state > 1U || expected_epoch == 0U || expected_epoch == UINT32_MAX)
        return -10;
    if (g_thread_comm_initialized == 0U ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER) != TX_SUCCESS) return -14;
    if (g_command_admission_fault || !persistent_store_recovery_ready()) {if(!bootstrap_state_release())return -9;return -9;}
    int32_t result = -10;
    if (g_thread_comm_recovery_epoch == expected_epoch && g_recovery_pending == 0U && !g_bootstrap_finalizing &&
        g_recovery_prepare != 0 && g_recovery_finish != 0 && g_recovery_read_outputs != 0) {
        /* Internal recovery barrier is not a NEW manual Stop: do not advance
         * epoch or broadcast ABORT. Failed backup verification keeps RAM safe
         * and returns before any floor/state flash can be attempted. */
        g_safety_report_closing = 2U;
        g_safety_report.reason = 0U;
        g_thread_comm_abort = 1U;
        if (board_abort_latch_prepare_recovery() != 0U) {
            uint8_t safe = 1U;
            if (!persistent_store_controls_ready()) {
                uint32_t outputs = 0U;
                const uint8_t required = BOARD_RECOVERY_OUTPUTS_VALID |
                    BOARD_RECOVERY_SAFE_OUTPUTS | BOARD_RECOVERY_OWNER_STOPPED;
                safe = (g_recovery_read_outputs(&outputs) & required) == required;
            }
            if (safe) {
                g_bootstrap_binding.valid=0U;
                g_recovery_desired = desired_state;
                g_recovery_expected_epoch = expected_epoch;
                g_recovery_pending = 2U;
                result = 0;
            } else result = -9;
        } else result = -9;
    }
    if(!bootstrap_state_release())return -9;
    return result;
}
void board_recovery_cancel_prepare(uint8_t desired_state, uint32_t expected_epoch)
{
    if (g_thread_comm_initialized == 0U ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER) != TX_SUCCESS) return;
    if (g_recovery_pending == 2U && g_recovery_expected_epoch == expected_epoch &&
        g_recovery_desired == desired_state) g_recovery_pending = 0U;
    (void)bootstrap_state_release();
}
int32_t board_recovery_apply_safe(uint8_t desired_state, uint32_t expected_abort_epoch,
                                BoardRecoverySnapshot *out)
{
    if (out == 0 || desired_state > 1U || expected_abort_epoch == 0U ||
        expected_abort_epoch == UINT32_MAX) return -10;
    if (g_thread_comm_initialized == 0U ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER) != TX_SUCCESS) return -14;
    if (!persistent_store_recovery_ready()) {
        board_recovery_snapshot_locked(out);
        if(!bootstrap_state_release())return -9;
        return -9;
    }
    int32_t result = -14;
    if (g_recovery_completed_epoch == expected_abort_epoch &&
        g_thread_comm_recovery_epoch == expected_abort_epoch + 1U &&
        g_thread_comm_abort == 0U && g_thread_comm_flight_state == desired_state) {
        result = persistent_store_controls_ready() ? 0 : -14;
    } else if (g_thread_comm_recovery_epoch == expected_abort_epoch &&
               g_recovery_prepare != 0 && g_recovery_finish != 0 &&
               g_recovery_read_outputs != 0 &&
               (g_recovery_pending != 0U &&
                g_recovery_expected_epoch == expected_abort_epoch &&
                g_recovery_desired == desired_state)) {
        g_recovery_desired = desired_state;
        g_recovery_expected_epoch = expected_abort_epoch;
        g_recovery_pending = 1U;
    } else {
        result = -10;
    }
    board_recovery_snapshot_locked(out);
    if(!bootstrap_state_release())return -9;
    return result;
}
static void service_board_recovery(bool explicit_fresh_owner)
{
    BootstrapBinding completed={0};BootstrapOwnerComplete finish_callback=0;
    if (g_thread_comm_initialized == 0U ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER) != TX_SUCCESS) return;
    if (explicit_fresh_owner && g_safety_report_inflight_lease == 0U && g_recovery_pending == 1U && !g_command_admission_fault && persistent_store_recovery_ready() &&
        (!persistent_store_bootstrap_pending() || (g_bootstrap_binding.valid && g_bootstrap_complete))) {
        g_recovery_pending = 0U;
        if (g_recovery_expected_epoch == g_thread_comm_recovery_epoch &&
            g_thread_comm_recovery_epoch != UINT32_MAX &&
            g_recovery_prepare != 0 && g_recovery_prepare(g_recovery_desired) != 0U) {
            thread_comm_msg_t discarded;uint8_t drained=0U;
            for(uint32_t n=0U;n<=THREAD_COMM_QUEUE_DEPTH;n++){
                UINT received=thread_comm_receive(&discarded,TX_NO_WAIT);
                if(received==TX_NO_INSTANCE){drained=1U;break;}
                if(received!=TX_SUCCESS)break; /* Busy/error is NOT empty. */
            }
            if(!drained){
                g_recovery_pending=1U;
                (void)bootstrap_state_release();return;
            }
            uint32_t outputs = 0U;
            uint8_t output_flags = g_recovery_read_outputs(&outputs);
            if ((output_flags & (BOARD_RECOVERY_OUTPUTS_VALID | BOARD_RECOVERY_SAFE_OUTPUTS)) ==
                (BOARD_RECOVERY_OUTPUTS_VALID | BOARD_RECOVERY_SAFE_OUTPUTS) &&
                board_abort_latch_clear_for_recovery() != 0U) {
                g_thread_comm_launch_sequence_pending = 0U;
                g_thread_comm_launch_command_timestamp_ms = 0U;
                g_recovery_finish();
                g_safety_report.reason = 0U;
                g_safety_report_closing = 0U;
                g_thread_comm_abort = 0U;
                g_thread_comm_flight_state = g_recovery_desired;
                if(persistent_store_bootstrap_pending()) {
                    completed=g_bootstrap_binding;finish_callback=g_bootstrap_complete;
                    g_bootstrap_finalizing=1U;g_bootstrap_binding.valid=0U;
                }
                g_recovery_completed_epoch = g_thread_comm_recovery_epoch;
                g_thread_comm_recovery_epoch++;
            }
        }
    }
    if(tx_mutex_put(&g_thread_comm_state_mutex)!=TX_SUCCESS) {
        g_command_admission_fault=1U;persistent_store_hold_bootstrap();return;
    }
    if(finish_callback && !finish_callback(completed.generation,completed.epoch,
            completed.epoch+1U,completed.desired,completed.issuer,completed.source))
        board_recovery_bootstrap_retain_stop();
}

/* Only the main storage service may cross the fresh original-time gate and
 * invoke owner apply. Ordinary task-loop service cannot apply queued tickets. */
void thread_comm_service_board_recovery(void){thread_comm_service_safety_publication();service_board_recovery(false);}
void board_recovery_execute_owned(void){service_board_recovery(true);}

/* Only register/RAM checks while locked; storage retry occurs after unlock. */
uint8_t board_recovery_boot_retry_is_safe(void)
{
    if (g_thread_comm_initialized == 0U ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_NO_WAIT) != TX_SUCCESS) return 0U;
    uint8_t safe = 0U;
    if (g_thread_comm_abort != 0U && g_recovery_pending == 0U && g_recovery_read_outputs != 0) {
        uint32_t outputs = 0U;
        const uint8_t required = BOARD_RECOVERY_OWNER_STOPPED | BOARD_RECOVERY_OUTPUTS_VALID | BOARD_RECOVERY_SAFE_OUTPUTS;
        safe = (g_recovery_read_outputs(&outputs) & required) == required;
    }
    (void)tx_mutex_put(&g_thread_comm_state_mutex);
    return safe;
}

static bool bootstrap_state_release(void) {
 if(tx_mutex_put(&g_thread_comm_state_mutex)==TX_SUCCESS)return true;
 g_command_admission_fault=1U;persistent_store_hold_bootstrap();return false;
}
bool board_recovery_bind_bootstrap_owner(BootstrapOwnerComplete complete,uint8_t(*stop)(void)) {
 if(!complete||!stop||!g_thread_comm_initialized||tx_mutex_get(&g_thread_comm_state_mutex,TX_WAIT_FOREVER)!=TX_SUCCESS)return false;
 g_bootstrap_complete=complete;g_bootstrap_stop=stop;return bootstrap_state_release();
}
int32_t board_recovery_prepare_storage_only(uint32_t epoch) {
 if(!epoch||epoch==UINT32_MAX)return -10;
 if(!g_thread_comm_initialized||tx_mutex_get(&g_thread_comm_state_mutex,TX_WAIT_FOREVER)!=TX_SUCCESS)return -14;
 int32_t result=-10;
 if(!g_command_admission_fault&&!g_bootstrap_finalizing&&!g_recovery_pending&&epoch==g_thread_comm_recovery_epoch&&g_bootstrap_stop&&g_recovery_read_outputs) {
  g_thread_comm_abort=1U;
  if(board_abort_latch_prepare_recovery()&&g_bootstrap_stop()) {
   thread_comm_msg_t discarded;
   for(uint32_t n=0;n<THREAD_COMM_QUEUE_DEPTH;n++)if(thread_comm_receive(&discarded,TX_NO_WAIT)!=TX_SUCCESS)break;
   uint32_t outputs=0;uint8_t required=BOARD_RECOVERY_OUTPUTS_VALID|BOARD_RECOVERY_SAFE_OUTPUTS|BOARD_RECOVERY_OWNER_STOPPED;
   result=(g_recovery_read_outputs(&outputs)&required)==required?0:-9;
  } else result=-9;
 }
 bool released=bootstrap_state_release();return released?result:-14;
}
bool board_recovery_bind_bootstrap_request(uint64_t generation,uint32_t epoch,uint8_t desired,uint64_t issuer,uint64_t source) {
 if(!generation||generation==UINT64_MAX||!issuer||!source||source==UINT64_MAX||desired>1U||!epoch||epoch==UINT32_MAX)return false;
 if(!g_thread_comm_initialized||tx_mutex_get(&g_thread_comm_state_mutex,TX_WAIT_FOREVER)!=TX_SUCCESS)return false;
 bool valid=persistent_store_bootstrap_pending()&&persistent_store_recovery_ready()&&g_bootstrap_complete&&
  !g_command_admission_fault&&!g_bootstrap_finalizing&&g_recovery_pending==2U&&g_recovery_expected_epoch==epoch&&g_recovery_desired==desired&&g_thread_comm_recovery_epoch==epoch;
 if(valid)g_bootstrap_binding=(BootstrapBinding){generation,issuer,source,epoch,desired,1U};
 bool released=bootstrap_state_release();return valid&&released;
}
static bool bootstrap_applied_locked(uint32_t old_epoch,uint32_t new_epoch,uint8_t desired) {
 uint32_t outputs=0;uint8_t flags=g_recovery_read_outputs?g_recovery_read_outputs(&outputs):0U;
 return g_bootstrap_finalizing&&!g_command_admission_fault&&!g_recovery_pending&&g_recovery_completed_epoch==old_epoch&&
  new_epoch==old_epoch+1U&&g_thread_comm_recovery_epoch==new_epoch&&!g_thread_comm_abort&&g_thread_comm_flight_state==desired&&
  (flags&(BOARD_RECOVERY_OUTPUTS_VALID|BOARD_RECOVERY_SAFE_OUTPUTS))==(BOARD_RECOVERY_OUTPUTS_VALID|BOARD_RECOVERY_SAFE_OUTPUTS);
}
bool board_recovery_bootstrap_verify_applied(uint32_t old_epoch,uint32_t new_epoch,uint8_t desired) {
 if(!g_thread_comm_initialized||tx_mutex_get(&g_thread_comm_state_mutex,TX_WAIT_FOREVER)!=TX_SUCCESS)return false;
 bool valid=bootstrap_applied_locked(old_epoch,new_epoch,desired);bool released=bootstrap_state_release();return valid&&released;
}
bool board_recovery_bootstrap_publish_complete(uint32_t old_epoch,uint32_t new_epoch,uint8_t desired) {
 if(!g_thread_comm_initialized||tx_mutex_get(&g_thread_comm_state_mutex,TX_WAIT_FOREVER)!=TX_SUCCESS)return false;
 bool valid=bootstrap_applied_locked(old_epoch,new_epoch,desired);
 g_bootstrap_finalizing=0U;bool released=bootstrap_state_release();
 /* Metadata publication clears no Stop; a Stop after unlock still wins through
  * its independent state-owned latch and a fresh response epoch check. */
 return valid&&released&&persistent_store_publish_bootstrap_complete();
}
void board_recovery_bootstrap_retain_stop(void) {
 persistent_store_hold_bootstrap();
 if(!g_thread_comm_initialized||tx_mutex_get(&g_thread_comm_state_mutex,TX_WAIT_FOREVER)!=TX_SUCCESS)return;
 g_bootstrap_finalizing=0U;g_bootstrap_binding.valid=0U;thread_comm_latch_abort_locked(0U);
 (void)board_abort_latch_prepare_recovery();(void)bootstrap_state_release();
}

UINT thread_comm_request_manual_abort(void){return thread_comm_request_manual_abort_original_utc(0U);}

/* Registration replays only an already latched Stop, never grants authority. */
UINT thread_comm_bind_manual_stop_apply(void (*apply)(void))
{
    if (!apply || !g_thread_comm_initialized) return TX_MUTEX_ERROR;
    UINT status=tx_mutex_get(&g_thread_comm_state_mutex,TX_WAIT_FOREVER);
    if(status!=TX_SUCCESS)return status;
    g_manual_stop_apply=apply;
    if(g_thread_comm_abort)apply();
    return tx_mutex_put(&g_thread_comm_state_mutex);
}
uint8_t thread_comm_manual_effect(uint64_t source,void(*apply)(void*),void*context)
{
    if(!apply||!g_thread_comm_initialized||tx_mutex_get(&g_thread_comm_state_mutex,TX_WAIT_FOREVER)!=TX_SUCCESS)return 0U;
    /* No new readiness/metadata guard may abort an already accepted Launch. */
    uint8_t allowed=!g_thread_comm_abort&&(source==0U||network_timestamp_allowed_locked(source));
    if(allowed)apply(context);
    (void)tx_mutex_put(&g_thread_comm_state_mutex);return allowed;
}

UINT thread_comm_receive_owned(thread_comm_msg_t*message,uint32_t*epoch)
{
 if(!epoch||!message||!g_thread_comm_initialized||tx_mutex_get(&g_thread_comm_state_mutex,TX_NO_WAIT)!=TX_SUCCESS)return TX_MUTEX_ERROR;
 UINT status=thread_comm_receive(message,TX_NO_WAIT);
 if(status==TX_SUCCESS)*epoch=g_thread_comm_recovery_epoch;
 (void)tx_mutex_put(&g_thread_comm_state_mutex);return status;
}
uint32_t thread_comm_manual_epoch(void)
{
 if(!g_thread_comm_initialized||tx_mutex_get(&g_thread_comm_state_mutex,TX_WAIT_FOREVER)!=TX_SUCCESS)return 0U;
 uint32_t epoch=g_thread_comm_recovery_epoch;
 (void)tx_mutex_put(&g_thread_comm_state_mutex);return epoch;
}
uint8_t thread_comm_manual_effect_owned(uint64_t source,uint32_t epoch,void(*apply)(void*),void*context)
{
 if(!apply||!epoch||epoch==UINT32_MAX||!g_thread_comm_initialized||tx_mutex_get(&g_thread_comm_state_mutex,TX_WAIT_FOREVER)!=TX_SUCCESS)return 0U;
 uint8_t allowed=epoch==g_thread_comm_recovery_epoch&&!g_thread_comm_abort&&(source==0U||network_timestamp_allowed_locked(source));
 if(allowed)apply(context);
 (void)tx_mutex_put(&g_thread_comm_state_mutex);return allowed;
}

uint8_t thread_comm_apply_automatic_owned(thread_comm_msg_t msg,uint32_t epoch,void (*apply)(uint8_t))
{
    if(!epoch||epoch==UINT32_MAX)return 0U;
    const uint8_t command = (uint8_t)(msg & ~THREAD_COMM_AUTOMATIC_TAG);
    if (msg > UINT8_MAX || (msg & THREAD_COMM_AUTOMATIC_TAG) == 0U || apply == 0 ||
        !(command <= CMD_DUMP_CLOSE) || g_thread_comm_initialized == 0U ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER) != TX_SUCCESS) return 0U;
    uint8_t applied = 0U;
    if (epoch==g_thread_comm_recovery_epoch && persistent_store_controls_ready() && board_launch_latch_is_set() == 0U && g_thread_comm_abort == 0U) {
        apply(command);
        applied = 1U;
    }
    (void)tx_mutex_put(&g_thread_comm_state_mutex);
    return applied;
}

uint8_t thread_comm_take_launch_sequence_request_owned(uint64_t *timestamp_ms,uint32_t*epoch)
{
    if(!epoch)return 0U;
    *epoch=0U;
    UINT status;
    uint8_t pending = 0U;

    if (timestamp_ms != TX_NULL)
    {
        *timestamp_ms = 0ULL;
    }

    if (g_thread_comm_initialized == 0U)
    {
        return 0U;
    }

    status = tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER);
    if (status == TX_SUCCESS)
    {
        pending = persistent_store_controls_ready() ? g_thread_comm_launch_sequence_pending : 0U;
        if (pending != 0U)
        {
            if (timestamp_ms != TX_NULL)
            {
                *timestamp_ms = g_thread_comm_launch_command_timestamp_ms;
            }
            *epoch=g_thread_comm_recovery_epoch;
            g_thread_comm_launch_sequence_pending = 0U;
        }
        (void)tx_mutex_put(&g_thread_comm_state_mutex);
    }

    return pending;
}

int thread_comm_ota_reset_original(uint32_t epoch,void(*apply)(void*),void*context) {
 if(!g_thread_comm_initialized||tx_mutex_get(&g_thread_comm_state_mutex,TX_NO_WAIT)!=TX_SUCCESS)return -1;
 int allowed=epoch&&epoch!=UINT32_MAX&&epoch==g_thread_comm_recovery_epoch&&!g_thread_comm_abort&&!board_launch_latch_is_set()&&
  (g_thread_comm_flight_state==VALVE_FLIGHT_STATE_STARTUP||g_thread_comm_flight_state==VALVE_FLIGHT_STATE_IDLE||g_thread_comm_flight_state==VALVE_FLIGHT_STATE_RECOVERY);
 if(allowed&&apply)apply(context);
 return tx_mutex_put(&g_thread_comm_state_mutex)==TX_SUCCESS?allowed:0;
}
