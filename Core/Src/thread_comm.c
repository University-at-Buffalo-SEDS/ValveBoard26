#include "thread_comm.h"
#include "board_abort_latch.h"
#include "board_recovery.h"
#include "board_commands.h"

static TX_MUTEX g_thread_comm_state_mutex;
static TX_MUTEX g_thread_comm_ring_mutex;
static TX_SEMAPHORE g_thread_comm_items_sem;
static TX_SEMAPHORE g_thread_comm_spaces_sem;
static thread_comm_msg_t g_thread_comm_ring[THREAD_COMM_QUEUE_DEPTH];
static uint32_t g_thread_comm_head = 0U;
static uint32_t g_thread_comm_tail = 0U;
static uint8_t g_thread_comm_initialized = 0U;
static uint8_t g_thread_comm_abort = 0U;
static uint32_t g_thread_comm_recovery_epoch = 1U;
static uint8_t g_recovery_pending;
static uint8_t g_recovery_desired;
static uint32_t g_recovery_expected_epoch;
static uint32_t g_recovery_completed_epoch;
static uint8_t (*g_recovery_read_outputs)(uint32_t *);
static uint8_t (*g_recovery_prepare)(uint8_t);
static void (*g_recovery_finish)(void);

static void thread_comm_latch_abort_locked(uint8_t new_manual_intent)
{
    if ((new_manual_intent != 0U || g_thread_comm_abort == 0U || g_recovery_pending != 0U) &&
        g_thread_comm_recovery_epoch != 0U) {
        g_thread_comm_recovery_epoch++;
    }
    g_recovery_pending = 0U;
    board_abort_latch_set();
    g_thread_comm_abort = 1U;
}

static uint8_t g_thread_comm_flight_state = VALVE_FLIGHT_STATE_STARTUP;
static uint64_t g_thread_comm_groundstation_heartbeat_ms = 0ULL;
static uint64_t g_thread_comm_launch_command_timestamp_ms = 0ULL;
static uint8_t g_thread_comm_launch_sequence_pending = 0U;
static int32_t g_thread_comm_shared_value = 0;

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
    g_thread_comm_initialized = 1U;

    return TX_SUCCESS;
}

UINT thread_comm_send(thread_comm_msg_t msg, ULONG wait_option)
{
    UINT status;

    if (g_thread_comm_initialized == 0U)
    {
        return TX_QUEUE_ERROR;
    }

    status = tx_semaphore_get(&g_thread_comm_spaces_sem, wait_option);
    if (status != TX_SUCCESS)
    {
        return status;
    }

    status = tx_mutex_get(&g_thread_comm_ring_mutex, wait_option);
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

UINT thread_comm_send_urgent(thread_comm_msg_t msg)
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
    if (board_launch_latch_is_set() == 0U && g_thread_comm_abort == 0U &&
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
    if (board_launch_latch_is_set() == 0U && g_thread_comm_abort == 0U &&
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
UINT thread_comm_request_manual_abort(void)
{
    if (g_thread_comm_initialized == 0U) return TX_MUTEX_ERROR;
    UINT status = tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER);
    if (status != TX_SUCCESS) return status;
    thread_comm_latch_abort_locked(1U);
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

    if (flight_state >= VALVE_FLIGHT_STATE_LAUNCH && flight_state <= VALVE_FLIGHT_STATE_RECOVERY) {
        if (g_thread_comm_abort != 0U || board_launch_latch_set_verified() == 0U) {
            thread_comm_latch_abort_locked(0U);
            (void)tx_mutex_put(&g_thread_comm_state_mutex);
            return TX_QUEUE_ERROR;
        }
    }
    g_thread_comm_flight_state = flight_state;

    return tx_mutex_put(&g_thread_comm_state_mutex);
}

UINT thread_comm_apply_managed_flight_state(uint8_t flight_state)
{
    if (g_thread_comm_initialized == 0U) return TX_MUTEX_ERROR;
    UINT status = tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER);
    if (status != TX_SUCCESS) return status;
    /* Launch and this decision share the same mutex. Delayed managed Aborted
     * has no manual-emergency provenance and cannot overwrite actual flight. */
    if (g_recovery_pending != 0U || g_thread_comm_abort != 0U ||
        (board_launch_latch_is_set() != 0U &&
         (flight_state == VALVE_FLIGHT_STATE_ABORTED ||
          flight_state < g_thread_comm_flight_state))) {
        (void)tx_mutex_put(&g_thread_comm_state_mutex);
        return TX_QUEUE_ERROR;
    }
    if (flight_state >= VALVE_FLIGHT_STATE_LAUNCH &&
        flight_state <= VALVE_FLIGHT_STATE_RECOVERY && board_launch_latch_set_verified() == 0U) {
        thread_comm_latch_abort_locked(0U);
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

UINT thread_comm_request_launch_sequence(uint64_t timestamp_ms)
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

    if (g_thread_comm_abort != 0U)
    {
        (void)tx_mutex_put(&g_thread_comm_state_mutex);
        return TX_QUEUE_ERROR;
    }

    if (board_launch_latch_set_verified() == 0U) {
        thread_comm_latch_abort_locked(0U);
        g_thread_comm_launch_sequence_pending = 0U;
        (void)tx_mutex_put(&g_thread_comm_state_mutex);
        return TX_QUEUE_ERROR;
    }
    g_thread_comm_launch_command_timestamp_ms = timestamp_ms;
    g_thread_comm_launch_sequence_pending = 1U;

    return tx_mutex_put(&g_thread_comm_state_mutex);
}

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
        pending = g_thread_comm_launch_sequence_pending;
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
UINT thread_comm_admit_automatic(thread_comm_msg_t msg)
{
    if (!(msg <= CMD_DUMP_CLOSE)) return TX_QUEUE_ERROR;
    if (g_thread_comm_initialized == 0U ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER) != TX_SUCCESS) {
        return TX_MUTEX_ERROR;
    }
    UINT status = TX_SUCCESS;
    if (board_launch_latch_is_set() == 0U && g_thread_comm_abort == 0U) {
        msg |= THREAD_COMM_AUTOMATIC_TAG;
        status = thread_comm_send(msg, TX_NO_WAIT);
    }
    (void)tx_mutex_put(&g_thread_comm_state_mutex);
    return status;
}

uint8_t thread_comm_apply_automatic(thread_comm_msg_t msg, void (*apply)(uint8_t))
{
    const uint8_t command = (uint8_t)(msg & ~THREAD_COMM_AUTOMATIC_TAG);
    if (msg > UINT8_MAX || (msg & THREAD_COMM_AUTOMATIC_TAG) == 0U || apply == 0 ||
        !(command <= CMD_DUMP_CLOSE) || g_thread_comm_initialized == 0U ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER) != TX_SUCCESS) return 0U;
    uint8_t applied = 0U;
    if (board_launch_latch_is_set() == 0U && g_thread_comm_abort == 0U) {
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
int32_t board_recovery_snapshot(BoardRecoverySnapshot *out)
{
    if (out == 0) return -10;
    if (g_thread_comm_initialized == 0U ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER) != TX_SUCCESS) return -14;
    board_recovery_snapshot_locked(out);
    (void)tx_mutex_put(&g_thread_comm_state_mutex);
    return 0;
}
/* PREPARING is not executable by the control owner. It closes managed-state
 * application and lets a fresh abort invalidate the expected epoch during NVM. */
int32_t board_recovery_begin_prepare(uint8_t desired_state, uint32_t expected_epoch)
{
    if (desired_state > 1U || expected_epoch == 0U || expected_epoch == UINT32_MAX)
        return -10;
    if (g_thread_comm_initialized == 0U ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER) != TX_SUCCESS) return -14;
    int32_t result = -10;
    if (g_thread_comm_recovery_epoch == expected_epoch && g_recovery_pending == 0U &&
        g_recovery_prepare != 0 && g_recovery_finish != 0 && g_recovery_read_outputs != 0) {
        /* Internal recovery barrier is not a NEW manual Stop: do not advance
         * epoch or broadcast ABORT. Failed backup verification keeps RAM safe
         * and returns before any floor/state flash can be attempted. */
        g_thread_comm_abort = 1U;
        if (board_abort_latch_prepare_recovery() != 0U) {
            g_recovery_desired = desired_state;
            g_recovery_expected_epoch = expected_epoch;
            g_recovery_pending = 2U;
            result = 0;
        } else result = -9;
    }
    (void)tx_mutex_put(&g_thread_comm_state_mutex);
    return result;
}
void board_recovery_cancel_prepare(uint8_t desired_state, uint32_t expected_epoch)
{
    if (g_thread_comm_initialized == 0U ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER) != TX_SUCCESS) return;
    if (g_recovery_pending == 2U && g_recovery_expected_epoch == expected_epoch &&
        g_recovery_desired == desired_state) g_recovery_pending = 0U;
    (void)tx_mutex_put(&g_thread_comm_state_mutex);
}
int32_t board_recovery_apply_safe(uint8_t desired_state, uint32_t expected_abort_epoch,
                                BoardRecoverySnapshot *out)
{
    if (out == 0 || desired_state > 1U || expected_abort_epoch == 0U ||
        expected_abort_epoch == UINT32_MAX) return -10;
    if (g_thread_comm_initialized == 0U ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER) != TX_SUCCESS) return -14;
    int32_t result = -14;
    if (g_recovery_completed_epoch == expected_abort_epoch &&
        g_thread_comm_recovery_epoch == expected_abort_epoch + 1U &&
        g_thread_comm_abort == 0U && g_thread_comm_flight_state == desired_state) {
        result = 0;
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
    (void)tx_mutex_put(&g_thread_comm_state_mutex);
    return result;
}
void thread_comm_service_board_recovery(void)
{
    if (g_thread_comm_initialized == 0U ||
        tx_mutex_get(&g_thread_comm_state_mutex, TX_WAIT_FOREVER) != TX_SUCCESS) return;
    if (g_recovery_pending == 1U) {
        g_recovery_pending = 0U;
        if (g_recovery_expected_epoch == g_thread_comm_recovery_epoch &&
            g_thread_comm_recovery_epoch != UINT32_MAX &&
            g_recovery_prepare != 0 && g_recovery_prepare(g_recovery_desired) != 0U) {
            thread_comm_msg_t discarded;
            for (uint32_t n = 0U; n < THREAD_COMM_QUEUE_DEPTH; n++) {
                if (thread_comm_receive(&discarded, TX_NO_WAIT) != TX_SUCCESS) break;
            }
            uint32_t outputs = 0U;
            uint8_t output_flags = g_recovery_read_outputs(&outputs);
            if ((output_flags & (BOARD_RECOVERY_OUTPUTS_VALID | BOARD_RECOVERY_SAFE_OUTPUTS)) ==
                (BOARD_RECOVERY_OUTPUTS_VALID | BOARD_RECOVERY_SAFE_OUTPUTS) &&
                board_abort_latch_clear_for_recovery() != 0U) {
                g_thread_comm_launch_sequence_pending = 0U;
                g_thread_comm_launch_command_timestamp_ms = 0U;
                g_recovery_finish();
                g_thread_comm_abort = 0U;
                g_thread_comm_flight_state = g_recovery_desired;
                g_recovery_completed_epoch = g_thread_comm_recovery_epoch;
                g_thread_comm_recovery_epoch++;
            }
        }
    }
    (void)tx_mutex_put(&g_thread_comm_state_mutex);
}

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
