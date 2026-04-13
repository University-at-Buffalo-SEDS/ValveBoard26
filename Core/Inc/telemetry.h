#pragma once
#include "sedsprintf.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  SedsRouter *r;
  uint8_t created;
  uint64_t start_time;
} RouterState;

enum telemetry_init_error_code {
  TELEMETRY_INIT_OK = 0,
  TELEMETRY_INIT_SUBSCRIBE_RX_FAILED = -1001,
  TELEMETRY_INIT_ROUTER_NEW_FAILED = -1002,
  TELEMETRY_INIT_ADD_CAN_SIDE_FAILED = -1003,
  TELEMETRY_INIT_CONFIGURE_TIMESYNC_FAILED = -1004,
  TELEMETRY_INIT_ANNOUNCE_DISCOVERY_FAILED = -1005,
};

extern RouterState g_router;

SedsResult tx_send(const uint8_t *bytes, size_t len, void *user);

SedsResult on_radio_packet(const SedsPacketView *pkt, void *user);
SedsResult on_time_sync_packet(const SedsPacketView *pkt, void *user);

SedsResult init_telemetry_router(void);

SedsResult log_telemetry_synchronous(SedsDataType data_type, const void *data,
                                     size_t element_count, size_t element_size);

SedsResult log_telemetry_asynchronous(SedsDataType data_type, const void *data,
                                      size_t element_count, size_t element_size);

SedsResult log_telemetry_string_asynchronous(SedsDataType data_type, const char *str);

SedsResult dispatch_tx_queue(void);

void rx_asynchronous(const uint8_t *bytes, size_t len);

SedsResult process_rx_queue(void);

SedsResult dispatch_tx_queue_timeout(uint32_t timeout_ms);
SedsResult process_rx_queue_timeout(uint32_t timeout_ms);
SedsResult process_all_queues_timeout(uint32_t timeout_ms);

SedsResult print_telemetry_error(int32_t error_code);
SedsResult log_error_asynchronous(const char *fmt, ...);
SedsResult log_error_synchronous(const char *fmt, ...);
SedsResult log_error_asyncronous(const char *fmt, ...);
SedsResult log_error_syncronous(const char *fmt, ...);

SedsResult telemetry_poll_timesync(void);
SedsResult telemetry_announce_discovery(void);
SedsResult telemetry_poll_discovery(void);
SedsResult telemetry_publish_umbilical_status(uint8_t cmd_id, uint8_t on);
void telemetry_note_can_rx(void);
void telemetry_toggle_led_on_can_rx(void);
int32_t telemetry_get_init_error_code(void);

uint64_t telemetry_now_ms(void);
uint64_t telemetry_unix_ms(void);
uint64_t telemetry_unix_s(void);
uint8_t telemetry_unix_is_valid(void);
enum valve_commands {
    CMD_PILOT_OPEN = 0,
    CMD_NORMALLY_OPEN_OPEN = 1,
    CMD_DUMP_OPEN = 2,
    CMD_PILOT_CLOSE = 3,
    CMD_NORMALLY_OPEN_CLOSE = 4,
    CMD_DUMP_CLOSE = 5,
    CMD_SEQUENCE = 6,
};
void telemetry_set_unix_time_ms(uint64_t unix_ms);

void die(const char *fmt, ...);

#ifdef __cplusplus
}
#endif
