#include "thread_comm.h"

static TX_QUEUE g_thread_comm_queue;
static TX_MUTEX g_thread_comm_state_mutex;
static thread_comm_msg_t *g_thread_comm_queue_storage = TX_NULL;
static uint8_t g_thread_comm_initialized = 0U;
static uint8_t g_thread_comm_abort = 0U;
static int32_t g_thread_comm_shared_value = 0;

UINT thread_comm_init(TX_BYTE_POOL *byte_pool)
{
    UINT status;
    ULONG queue_storage_size;

    if (g_thread_comm_initialized != 0U)
    {
        return TX_SUCCESS;
    }

    if (byte_pool == TX_NULL)
    {
        return TX_POOL_ERROR;
    }

    queue_storage_size = (ULONG)(sizeof(thread_comm_msg_t) * THREAD_COMM_QUEUE_DEPTH);

    status = tx_byte_allocate(byte_pool, (VOID **)&g_thread_comm_queue_storage,
                              queue_storage_size, TX_NO_WAIT);
    if (status != TX_SUCCESS)
    {
        return status;
    }

    status = tx_queue_create(&g_thread_comm_queue,
                             "thread_comm_queue",
                             TX_1_ULONG,
                             g_thread_comm_queue_storage,
                             queue_storage_size);
    if (status != TX_SUCCESS)
    {
        return status;
    }

    status = tx_mutex_create(&g_thread_comm_state_mutex,
                             "thread_comm_state_mutex",
                             TX_INHERIT);
    if (status != TX_SUCCESS)
    {
        return status;
    }

    g_thread_comm_abort = 0U;
    g_thread_comm_shared_value = 0;
    g_thread_comm_initialized = 1U;

    return TX_SUCCESS;
}

UINT thread_comm_send(thread_comm_msg_t msg, ULONG wait_option)
{
    if (g_thread_comm_initialized == 0U)
    {
        return TX_QUEUE_ERROR;
    }

    return tx_queue_send(&g_thread_comm_queue, &msg, wait_option);
}

UINT thread_comm_receive(thread_comm_msg_t *msg, ULONG wait_option)
{
    if ((g_thread_comm_initialized == 0U) || (msg == TX_NULL))
    {
        return TX_QUEUE_ERROR;
    }

    return tx_queue_receive(&g_thread_comm_queue, msg, wait_option);
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
