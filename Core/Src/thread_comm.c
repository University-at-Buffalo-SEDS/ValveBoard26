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
