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

/* Application-local receipt, never a SEDSnet wire packet ID. */
#define THREAD_COMM_LAUNCH_SENDER_MAX 32U
typedef struct { uint64_t source_utc; uint32_t epoch, serial; } thread_comm_launch_owner_t;
uint8_t thread_comm_launch_continuity(void);
UINT thread_comm_request_accepted_launch_manual_abort(uint64_t source_utc);
uint8_t thread_comm_launch_original_accepted(uint64_t source,const char*sender,uint32_t sender_len);
uint8_t thread_comm_launch_claim(uint64_t source,uint32_t epoch,thread_comm_launch_owner_t*out);
uint8_t thread_comm_launch_begin_owned(const thread_comm_launch_owner_t*owner,void(*apply)(void*),void*context);
uint8_t thread_comm_launch_effect_owned(const thread_comm_launch_owner_t*owner,void(*apply)(void*),void*context);
void thread_comm_launch_complete_owned(const thread_comm_launch_owner_t*owner);
UINT thread_comm_request_network_launch_original(uint64_t source,const char*sender,uint32_t sender_len);
uint8_t thread_comm_take_accepted_launch(thread_comm_launch_owner_t*out);
typedef enum {
    VALVE_FLIGHT_STATE_STARTUP = 0,
    VALVE_FLIGHT_STATE_IDLE = 1,
    VALVE_FLIGHT_STATE_PREFILL = 2,
    VALVE_FLIGHT_STATE_FILL_TEST = 3,
    VALVE_FLIGHT_STATE_NITROGEN_FILL = 4,
    VALVE_FLIGHT_STATE_NITROUS_FILL = 5,
    VALVE_FLIGHT_STATE_ARMED = 6,
    VALVE_FLIGHT_STATE_LAUNCH = 7,
    VALVE_FLIGHT_STATE_ASCENT = 8,
    VALVE_FLIGHT_STATE_COAST = 9,
    VALVE_FLIGHT_STATE_APOGEE = 10,
    VALVE_FLIGHT_STATE_PARACHUTE_DEPLOY = 11,
    VALVE_FLIGHT_STATE_DESCENT = 12,
    VALVE_FLIGHT_STATE_LANDED = 13,
    VALVE_FLIGHT_STATE_RECOVERY = 14,
    VALVE_FLIGHT_STATE_ABORTED = 15,
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
/* Application-internal outcomes, never SDK wire IDs or fake completion. */
#define THREAD_COMM_STALE_TIMESTAMP ((UINT)0xF001U)
#define THREAD_COMM_COMMIT_UNCERTAIN ((UINT)0xF002U)
UINT thread_comm_publish_command_cutoff(uint64_t cutoff_ms);
UINT thread_comm_admit_network(thread_comm_msg_t msg, uint64_t timestamp_ms);
UINT thread_comm_apply_network_flight_state(uint8_t state,uint64_t timestamp_ms);
UINT thread_comm_admit_network_automatic(thread_comm_msg_t msg, uint64_t timestamp_ms);
UINT thread_comm_request_network_launch_sequence(uint64_t timestamp_ms);
UINT thread_comm_send_urgent(thread_comm_msg_t msg);

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
/* One newly delivered operator Stop; advances recovery epoch even if latched. */
UINT thread_comm_request_manual_abort(void);
UINT thread_comm_request_manual_abort_original_utc(uint64_t source);
/* Task-context bounded GPIO/RAM only callbacks. No SDK/storage/delay/reentry. */
UINT thread_comm_bind_manual_stop_apply(void (*apply)(void));
uint8_t thread_comm_manual_effect(uint64_t original_source,void (*apply)(void*),void*context);


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
/* Managed state is not manual Abort provenance; admission is atomic with launch. */
UINT thread_comm_apply_managed_flight_state(uint8_t flight_state);
uint8_t thread_comm_get_flight_state(void);
uint8_t thread_comm_abort_allowed(void);
uint8_t thread_comm_request_automatic_abort(void);
/* Owner-tagged finite local report. A lease is immutable through SDK return.
 * No state mutex or IRQ mask is held by the caller during SDK enqueue. */
typedef struct {
    uint64_t original_utc;
    uint32_t reason;
    uint32_t epoch;
    uint32_t lease;
} thread_comm_safety_report_t;
uint8_t thread_comm_request_local_abort(uint32_t reason, uint64_t at_ms,
    uint64_t heartbeat_ms, uint32_t observed_state, uint32_t detail,
    uint64_t original_utc, uint8_t heartbeat_only);
uint8_t thread_comm_safety_report_claim(thread_comm_safety_report_t *out);
void thread_comm_safety_report_finish(uint32_t lease, uint8_t retained);
void thread_comm_service_safety_publication(void);

/* Link loss excludes Idle; other automatic safety rules remain unchanged. */
uint8_t thread_comm_request_heartbeat_abort(void);
uint8_t thread_comm_heartbeat_abort_allowed(void);
/* Exact typed automatic ordinary-valve commands only; no launch/abort/pyro. */
#define THREAD_COMM_AUTOMATIC_TAG 0x80U
UINT thread_comm_admit_automatic(thread_comm_msg_t msg);
/* Callback must contain only bounded GPIO/timer/RAM operations, no SDK/IO wait.
 * Launch and auto-application serialize through this same state mutex. */
uint8_t thread_comm_apply_automatic(thread_comm_msg_t msg, void (*apply)(uint8_t));
UINT thread_comm_set_launch_command_timestamp_ms(uint64_t timestamp_ms);
uint64_t thread_comm_get_launch_command_timestamp_ms(void);
UINT thread_comm_request_launch_sequence(uint64_t timestamp_ms);
uint8_t thread_comm_take_launch_sequence_request(uint64_t *timestamp_ms);

UINT thread_comm_note_groundstation_heartbeat(uint64_t timestamp_ms);
uint64_t thread_comm_get_groundstation_heartbeat_ms(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C" {
#endif
/* Captures cancellation epoch with actual nonwaiting dequeue, no wire change. */
UINT thread_comm_receive_owned(thread_comm_msg_t*message,uint32_t*epoch);
uint8_t thread_comm_manual_effect_owned(uint64_t original_source,uint32_t epoch,void(*apply)(void*),void*context);
uint32_t thread_comm_manual_epoch(void);
uint8_t thread_comm_take_launch_sequence_request_owned(uint64_t*timestamp_ms,uint32_t*epoch);

uint8_t thread_comm_apply_automatic_owned(thread_comm_msg_t msg,uint32_t epoch,void(*apply)(uint8_t));
/* Bounded current output observation under the shared state owner. Callback
 * must only read registers/RAM; never SDK, storage, waits, or owner reentry. */
typedef uint8_t (*thread_comm_status_read_t)(uint8_t id, uint8_t *state);
int32_t thread_comm_status_snapshot_try(uint8_t id, uint8_t *state,
                                      thread_comm_status_read_t read);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C" {
#endif
/* OTA reset effect only: -1 busy,0 retired/unsafe,1 current (or executed).
 * No storage/SDK/work is allowed in apply. NULL performs same owned check. */
int thread_comm_ota_reset_original(uint32_t epoch,void(*apply)(void*),void*context);
#ifdef __cplusplus
}
#endif
