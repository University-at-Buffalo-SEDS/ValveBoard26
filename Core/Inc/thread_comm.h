#pragma once

#include "tx_api.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Number of message slots allocated for the inter-thread queue.
 */
#define THREAD_COMM_QUEUE_DEPTH 16U

/**
 * @brief Queue message type used by the inter-thread communication layer.
 *
 * The current queue is configured for one ULONG per message. If a richer
 * payload is needed later, this type and the queue configuration can be
 * expanded together.
 */
typedef ULONG thread_comm_msg_t;

typedef enum {
    VALVE_FLIGHT_STATE_STARTUP = 0,
    VALVE_FLIGHT_STATE_IDLE = 1,
    VALVE_FLIGHT_STATE_PREFILL = 2,
    VALVE_FLIGHT_STATE_FILL_TEST = 3,
    VALVE_FLIGHT_STATE_NITROGEN_FILL = 4,
    VALVE_FLIGHT_STATE_NITROUS_FILL = 5,
    VALVE_FLIGHT_STATE_POSTINIT = 6,
    VALVE_FLIGHT_STATE_ARMED = 7,
    VALVE_FLIGHT_STATE_LAUNCH = 8,
    VALVE_FLIGHT_STATE_ASCENT = 9,
    VALVE_FLIGHT_STATE_COAST = 10,
    VALVE_FLIGHT_STATE_APOGEE = 11,
    VALVE_FLIGHT_STATE_DESCENT = 12,
    VALVE_FLIGHT_STATE_REEFING = 13,
    VALVE_FLIGHT_STATE_LANDED = 14,
    VALVE_FLIGHT_STATE_RECOVERY = 15,
} valve_flight_state_t;

/**
 * @brief Create the shared queue and state protection mutex.
 *
 * This should be called once during system startup before any thread uses the
 * communication helpers.
 *
 * @param byte_pool ThreadX byte pool used to allocate queue storage.
 * @retval TX_SUCCESS Initialization completed successfully.
 * @retval ThreadX error code Queue or mutex creation/allocation failed.
 */
UINT thread_comm_init(TX_BYTE_POOL *byte_pool);

/**
 * @brief Push one message into the shared queue.
 *
 * Safe to call from a producer thread while another thread drains the queue.
 *
 * @param msg Message value to enqueue.
 * @param wait_option ThreadX wait option passed to tx_queue_send().
 * @retval TX_SUCCESS Message was queued.
 * @retval ThreadX error code Queue was not ready, full, or send failed.
 */
UINT thread_comm_send(thread_comm_msg_t msg, ULONG wait_option);

/**
 * @brief Pop one message from the shared queue.
 *
 * Intended for the consumer thread that drains work produced by another
 * thread.
 *
 * @param msg Output pointer that receives the dequeued message.
 * @param wait_option ThreadX wait option passed to tx_queue_receive().
 * @retval TX_SUCCESS Message was received.
 * @retval ThreadX error code Queue was not ready, empty, or receive failed.
 */
UINT thread_comm_receive(thread_comm_msg_t *msg, ULONG wait_option);

/**
 * @brief Update the shared abort flag.
 *
 * The flag is protected by a mutex so one thread can request an abort while
 * other threads read a consistent value.
 *
 * @param abort_requested Non-zero sets the flag, zero clears it.
 * @retval TX_SUCCESS Flag updated successfully.
 * @retval ThreadX error code Mutex access failed.
 */
UINT thread_comm_set_abort(uint8_t abort_requested);

/**
 * @brief Read the shared abort flag.
 *
 * @return `1` if abort has been requested, otherwise `0`.
 */
uint8_t thread_comm_get_abort(void);

/**
 * @brief Store the shared integer value.
 *
 * @param value New value to publish for other threads.
 * @retval TX_SUCCESS Value updated successfully.
 * @retval ThreadX error code Mutex access failed.
 */
UINT thread_comm_set_shared_value(int32_t value);

/**
 * @brief Read the latest shared integer value.
 *
 * @return Most recently stored shared value, or `0` if the module is not yet
 * initialized.
 */
int32_t thread_comm_get_shared_value(void);

UINT thread_comm_set_flight_state(uint8_t flight_state);
uint8_t thread_comm_get_flight_state(void);
uint8_t thread_comm_abort_allowed(void);

UINT thread_comm_note_groundstation_heartbeat(uint64_t timestamp_ms);
uint64_t thread_comm_get_groundstation_heartbeat_ms(void);

#ifdef __cplusplus
}
#endif
