#include "thread_comm.h"

static TX_MUTEX g_thread_comm_state_mutex;
static TX_MUTEX g_thread_comm_ring_mutex;
static TX_SEMAPHORE g_thread_comm_items_sem;
static TX_SEMAPHORE g_thread_comm_spaces_sem;
static thread_comm_msg_t g_thread_comm_ring[THREAD_COMM_QUEUE_DEPTH];
static uint32_t g_thread_comm_head = 0U;
static uint32_t g_thread_comm_tail = 0U;
static uint8_t g_thread_comm_initialized = 0U;
static uint8_t g_thread_comm_abort = 0U;
static uint8_t g_thread_comm_flight_state = VALVE_FLIGHT_STATE_STARTUP;
static uint64_t g_thread_comm_groundstation_heartbeat_ms = 0ULL;
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
    g_thread_comm_abort = 0U;
    g_thread_comm_flight_state = VALVE_FLIGHT_STATE_STARTUP;
    g_thread_comm_groundstation_heartbeat_ms = 0ULL;
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

    g_thread_comm_abort = (abort_requested != 0U) ? 1U : 0U;

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

    g_thread_comm_flight_state = flight_state;

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
    return ((flight_state > VALVE_FLIGHT_STATE_STARTUP) &&
            (flight_state < VALVE_FLIGHT_STATE_LAUNCH))
               ? 1U
               : 0U;
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
