// telemetry.c
#include "telemetry.h"
#include "flight_state_cache.h"
#include "sim_network_probe.h"
#include "ota_stream.h"

#include "app_threadx.h"
#include "can_bus.h"
#include "main.h"
#include "sedsnet_config.h"
#include "stm32g4xx_hal.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "thread_comm.h"

#ifndef TELEMETRY_ENABLED
static void print_data_no_telem(void *data, size_t len)
{
  (void)data;
  (void)len;
}
#endif

#if defined(__GNUC__) || defined(__clang__)
#define UNUSED_FUNCTION __attribute__((unused))
#else
#define UNUSED_FUNCTION
#endif

#ifndef TELEMETRY_TIMESYNC_MASTER_PRIO
#define TELEMETRY_TIMESYNC_MASTER_PRIO (-1)
#endif

#ifndef TELEMETRY_TIMESYNC_SOURCE_TIMEOUT_MS
#define TELEMETRY_TIMESYNC_SOURCE_TIMEOUT_MS 5000U
#endif

#ifndef TELEMETRY_TIMESYNC_ANNOUNCE_INTERVAL_MS
#define TELEMETRY_TIMESYNC_ANNOUNCE_INTERVAL_MS 2000U
#endif

#ifndef TELEMETRY_TIMESYNC_REQUEST_INTERVAL_MS
#define TELEMETRY_TIMESYNC_REQUEST_INTERVAL_MS 2000U
#endif

#ifndef TELEMETRY_LINK_GRACE_MS
#define TELEMETRY_LINK_GRACE_MS 10000ULL
#endif

#ifndef TELEMETRY_LINK_TIMEOUT_MS
#define TELEMETRY_LINK_TIMEOUT_MS 5000ULL
#endif

#ifndef TX_TIMER_TICKS_PER_SECOND
#error "TX_TIMER_TICKS_PER_SECOND must be defined by ThreadX."
#endif

#define TELEMETRY_TIMESYNC_ROLE_CONSUMER 0U
#define TELEMETRY_TIMESYNC_ROLE_SOURCE 1U

static UNUSED_FUNCTION uint8_t g_can_rx_subscribed = 0U;
static UNUSED_FUNCTION int32_t g_can_side_id = -1;
static uint8_t g_local_unix_valid = 0U;
static uint64_t g_local_unix_ms = 0ULL;
static int32_t g_telemetry_init_error_code = TELEMETRY_INIT_OK;
static volatile uint32_t g_flight_state_handler_count = 0U;
static volatile uint32_t g_flight_state_handler_error_count = 0U;
static volatile uint8_t g_last_flight_state_packet = 0U;
static volatile uint32_t g_heartbeat_handler_count = 0U;
static volatile uint32_t g_heartbeat_handler_error_count = 0U;
static volatile uint8_t g_abort_broadcast_sent = 0U;
static volatile uint32_t g_last_can_rx_ms = 0U;

RouterState g_router = {.r = NULL, .created = 0U, .start_time = 0ULL};

/* Exported simulator/HIL health signals. A linked-bay test requires both a
 * remote discovery topology change and a valid SEDSNet network clock. */
volatile uint32_t g_telemetry_discovery_seen = 0U;
volatile uint32_t g_telemetry_timesync_valid = 0U;
volatile uint32_t g_telemetry_network_ready = 0U;
volatile uint32_t g_telemetry_peer_mask = 0U;
volatile uint32_t g_sim_heartbeat_attempts = 0U;
volatile uint32_t g_sim_heartbeat_ok = 0U;
volatile uint32_t g_sim_heartbeat_fail = 0U;
volatile uint32_t g_sim_heartbeat_wire_tx = 0U;
volatile uint32_t g_sim_valve_commands_received = 0U;
volatile uint32_t g_sim_groundstation_confirmation_received = 0U;
volatile uint32_t g_sim_umbilical_status_ok = 0U;
volatile uint32_t g_sim_umbilical_status_fail = 0U;
volatile uint32_t g_sim_pilot_open_status_wire_tx = 0U;
static uint8_t g_sim_valve_open_seen = 0U;

int32_t telemetry_get_init_error_code(void) { return g_telemetry_init_error_code; }

static uint64_t tx_raw_now_ms_locked(void)
{
  const uint32_t ticks32 = (uint32_t)tx_time_get();
  return ((uint64_t)ticks32 * 1000ULL) / (uint64_t)TX_TIMER_TICKS_PER_SECOND;
}

static UNUSED_FUNCTION uint64_t tx_raw_now_ms(void *user)
{
  (void)user;
  return tx_raw_now_ms_locked();
}

static uint8_t telemetry_timesync_is_source(void)
{
  return (TELEMETRY_TIMESYNC_MASTER_PRIO >= 0) ? 1U : 0U;
}

static uint64_t telemetry_timesync_priority(void)
{
  return telemetry_timesync_is_source() ? (uint64_t)TELEMETRY_TIMESYNC_MASTER_PRIO : 0ULL;
}

static uint32_t telemetry_timesync_role(void)
{
  return telemetry_timesync_is_source() ? TELEMETRY_TIMESYNC_ROLE_SOURCE
                                        : TELEMETRY_TIMESYNC_ROLE_CONSUMER;
}

static bool telemetry_unix_ms_to_utc(uint64_t unix_ms, int32_t *year, uint8_t *month,
                                     uint8_t *day, uint8_t *hour, uint8_t *minute,
                                     uint8_t *second, uint16_t *millisecond)
{
  static const uint16_t days_before_month[12] = {0U, 31U, 59U, 90U, 120U, 151U,
                                                 181U, 212U, 243U, 273U, 304U, 334U};
  uint64_t whole_seconds = unix_ms / 1000ULL;
  const uint64_t days_since_epoch = whole_seconds / 86400ULL;
  uint32_t seconds_of_day = (uint32_t)(whole_seconds % 86400ULL);
  int32_t y = 1970;
  uint64_t days = days_since_epoch;

  if (!year || !month || !day || !hour || !minute || !second || !millisecond)
  {
    return false;
  }

  while (1)
  {
    const uint32_t y_u32 = (uint32_t)y;
    const uint8_t leap =
        ((y_u32 % 4U) == 0U && ((y_u32 % 100U) != 0U || (y_u32 % 400U) == 0U)) ? 1U : 0U;
    const uint32_t days_in_year = leap ? 366U : 365U;
    if (days < days_in_year)
    {
      uint8_t m = 1U;
      uint32_t day_of_year = (uint32_t)days;
      for (; m <= 12U; ++m)
      {
        uint32_t month_start = days_before_month[m - 1U];
        uint32_t month_end =
            (m < 12U) ? days_before_month[m] : (uint32_t)(leap ? 366U : 365U);
        if (leap && m > 2U)
        {
          month_start += 1U;
          month_end += 1U;
        }
        if (day_of_year < month_end)
        {
          *year = y;
          *month = m;
          *day = (uint8_t)(day_of_year - month_start + 1U);
          *hour = (uint8_t)(seconds_of_day / 3600U);
          *minute = (uint8_t)((seconds_of_day % 3600U) / 60U);
          *second = (uint8_t)(seconds_of_day % 60U);
          *millisecond = (uint16_t)(unix_ms % 1000ULL);
          return true;
        }
      }
      return false;
    }
    days -= days_in_year;
    ++y;
  }
}

SedsResult Valve_Command_handler(const SedsPacketView *pkt, void *user)
{
  (void)user;

  if (pkt == NULL)
  {
    return SEDS_BAD_ARG;
  }
  if (pkt->payload == NULL || pkt->payload_len == 0U)
  {
    return SEDS_BAD_ARG;
  }
  // get the current command and push into the tx queue for the main task to handle
  uint8_t cmd_u8;
  int32_t got = seds_pkt_get_u8(pkt, &cmd_u8, 1U);
  if (got != 1)
  {
    return (got < 0) ? (SedsResult)got : SEDS_BAD_ARG;
  }

  g_sim_valve_commands_received++;
  if (cmd_u8 == CMD_PILOT_OPEN)
  {
    g_sim_valve_open_seen = 1U;
  }
  else if (cmd_u8 == CMD_PILOT_CLOSE && g_sim_valve_open_seen != 0U)
  {
    g_sim_groundstation_confirmation_received++;
  }

  if (cmd_u8 == CMD_ABORT)
  {
    (void)thread_comm_set_abort(true);
    (void)thread_comm_send(CMD_ABORT, TX_NO_WAIT);
    (void)telemetry_broadcast_abort("Valve board abort command");
    return SEDS_OK;
  }

  if (cmd_u8 == CMD_SEQUENCE)
  {
    (void)thread_comm_request_launch_sequence(pkt->timestamp);
    (void)thread_comm_set_flight_state(VALVE_FLIGHT_STATE_LAUNCH);
    return SEDS_OK;
  }

  (void)thread_comm_send(cmd_u8, TX_NO_WAIT);
  return SEDS_OK;
}

SedsResult Abort_handler(const SedsPacketView *pkt, void *user)
{
  (void)pkt;
  (void)user;
  (void)thread_comm_set_abort(true);
  (void)thread_comm_send(CMD_ABORT, TX_NO_WAIT);
  (void)telemetry_broadcast_abort("Valve board abort command");
  return SEDS_OK;
}

SedsResult Flight_State_handler(const SedsPacketView *pkt, void *user)
{
  (void)user;

  if (thread_comm_get_abort() != 0U)
  {
    return SEDS_OK;
  }

  if (pkt == NULL || pkt->payload == NULL || pkt->payload_len == 0U)
  {
    g_flight_state_handler_error_count++;
    return SEDS_BAD_ARG;
  }

  uint8_t flight_state = 0U;
  int32_t got = seds_pkt_get_u8(pkt, &flight_state, 1U);
  if (got != 1)
  {
    g_flight_state_handler_error_count++;
    return (got < 0) ? (SedsResult)got : SEDS_BAD_ARG;
  }

  g_flight_state_handler_count++;
  g_last_flight_state_packet = flight_state;

  const uint8_t current_flight_state = thread_comm_get_flight_state();
  if ((current_flight_state >= VALVE_FLIGHT_STATE_LAUNCH) &&
      (flight_state < current_flight_state))
  {
    return SEDS_OK;
  }

  if (thread_comm_set_flight_state(flight_state) != TX_SUCCESS)
  {
    g_flight_state_handler_error_count++;
  }

  return SEDS_OK;
}

SedsResult flight_state_cache_apply_network_update(const SedsPacketView *packet)
{
  return Flight_State_handler(packet, NULL);
}

SedsResult Heartbeat_handler(const SedsPacketView *pkt, void *user)
{
  (void)sim_probe_heartbeat_handler(pkt, user);
  (void)user;

  if (thread_comm_get_abort() != 0U)
  {
    return SEDS_OK;
  }

  if (pkt == NULL || pkt->ty != SEDS_DT_HEARTBEAT)
  {
    g_heartbeat_handler_error_count++;
    return SEDS_BAD_ARG;
  }

  g_heartbeat_handler_count++;
  if (thread_comm_note_groundstation_heartbeat(telemetry_now_ms()) != TX_SUCCESS)
  {
    g_heartbeat_handler_error_count++;
  }

  return SEDS_OK;
}

static SedsResult telemetry_apply_local_unix_time_locked(SedsRouter *router)
{
  int32_t year = 0;
  uint8_t month = 0;
  uint8_t day = 0;
  uint8_t hour = 0;
  uint8_t minute = 0;
  uint8_t second = 0;
  uint16_t millisecond = 0;

  if (!router || !telemetry_timesync_is_source() || !g_local_unix_valid)
  {
    return SEDS_OK;
  }

  if (!telemetry_unix_ms_to_utc(g_local_unix_ms, &year, &month, &day, &hour, &minute,
                                &second, &millisecond))
  {
    return SEDS_BAD_ARG;
  }

  return seds_router_set_local_network_datetime_millis(router, year, month, day, hour, minute,
                                                       second, millisecond);
}

static UNUSED_FUNCTION SedsResult telemetry_configure_timesync_locked(SedsRouter *router)
{
  SedsResult result;

  if (!router)
  {
    return SEDS_BAD_ARG;
  }

  result = seds_router_configure_timesync(
      router, true, telemetry_timesync_role(), telemetry_timesync_priority(),
      (uint64_t)TELEMETRY_TIMESYNC_SOURCE_TIMEOUT_MS,
      (uint64_t)TELEMETRY_TIMESYNC_ANNOUNCE_INTERVAL_MS,
      (uint64_t)TELEMETRY_TIMESYNC_REQUEST_INTERVAL_MS);
  if (result != SEDS_OK)
  {
    return result;
  }

  return telemetry_apply_local_unix_time_locked(router);
}

uint64_t telemetry_now_ms(void) { return tx_raw_now_ms_locked(); }

void telemetry_note_can_rx(void)
{
  g_last_can_rx_ms = (uint32_t)tx_time_get();
}

uint8_t telemetry_link_recently_active(void)
{
  const uint64_t now_ms = tx_raw_now_ms_locked();
  const uint32_t now_ticks = (uint32_t)tx_time_get();
  const uint32_t last_rx_ticks = g_last_can_rx_ms;
  const uint32_t timeout_ticks =
      (uint32_t)((TELEMETRY_LINK_TIMEOUT_MS * (uint64_t)TX_TIMER_TICKS_PER_SECOND) / 1000ULL);

  if ((g_router.created != 0U) && ((now_ms - g_router.start_time) < TELEMETRY_LINK_GRACE_MS))
  {
    return 1U;
  }

  if (last_rx_ticks == 0U)
  {
    return 0U;
  }

  return ((uint32_t)(now_ticks - last_rx_ticks) < timeout_ticks) ? 1U : 0U;
}

uint64_t telemetry_unix_ms(void)
{
#ifndef TELEMETRY_ENABLED
  return g_local_unix_valid ? g_local_unix_ms : 0ULL;
#else
  uint64_t unix_ms = 0ULL;

  if (g_router.r && seds_router_get_network_time_ms(g_router.r, &unix_ms) == SEDS_OK)
  {
    return unix_ms;
  }

  if (telemetry_timesync_is_source() && g_local_unix_valid)
  {
    return g_local_unix_ms;
  }

  return 0ULL;
#endif
}

uint64_t telemetry_unix_s(void) { return telemetry_unix_ms() / 1000ULL; }

uint8_t telemetry_unix_is_valid(void) { return telemetry_unix_ms() != 0ULL ? 1U : 0U; }

void telemetry_set_unix_time_ms(uint64_t unix_ms)
{
  g_local_unix_ms = unix_ms;
  g_local_unix_valid = (unix_ms != 0ULL) ? 1U : 0U;

#ifdef TELEMETRY_ENABLED
  if (g_router.r != NULL)
  {
    (void)telemetry_apply_local_unix_time_locked(g_router.r);
  }
#endif
}

static UNUSED_FUNCTION uint64_t node_now_since_ms(void *user)
{
  (void)user;
  const RouterState s = g_router;
  const uint64_t now = tx_raw_now_ms_locked();
  return s.r ? (now - s.start_time) : 0ULL;
}

SedsResult tx_send(const uint8_t *bytes, size_t len, void *user)
{
  (void)user;

  if (!bytes || len == 0U)
  {
    return SEDS_BAD_ARG;
  }
#ifdef SEDS_FIRMWARE_SIM_TEST
  if (sim_probe_packed_data_type(bytes, len) == (uint32_t)SEDS_DT_UMBILICAL_STATUS)
  {
    SedsOwnedPacket *owned = seds_pkt_unpack_owned(bytes, len);
    SedsPacketView view;
    if (owned != NULL && seds_owned_pkt_view(owned, &view) == SEDS_OK &&
        view.payload_len == 2U && view.payload[0] == 0U && view.payload[1] != 0U)
    {
      g_sim_pilot_open_status_wire_tx++;
    }
    if (owned != NULL)
    {
      seds_owned_pkt_free(owned);
    }
  }
#endif
  HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
  const uint32_t can_id =
      sim_probe_packed_data_type(bytes, len) == (uint32_t)SEDS_DT_HEARTBEAT
          ? 0x006U
          : 0x106U;
  if (can_bus_send_large(bytes, len, can_id) == HAL_OK)
  {
    sim_probe_observe_can_tx(bytes, len);
    return SEDS_OK;
  }
  return SEDS_IO;
}

static UNUSED_FUNCTION void telemetry_can_rx(const uint8_t *data, size_t len, void *user)
{
  (void)user;
  sim_probe_observe_packed(data, len);
  telemetry_note_can_rx();
  rx_asynchronous(data, len);
}

void rx_asynchronous(const uint8_t *bytes, size_t len)
{
#ifndef TELEMETRY_ENABLED
  (void)bytes;
  (void)len;
  return;
#else
  SedsResult result = SEDS_OK;

  if (!bytes || len == 0U)
  {
    return;
  }

  if (!g_router.r && init_telemetry_router() != SEDS_OK)
  {
    return;
  }

  if (g_can_side_id >= 0)
  {
    result = seds_router_receive_packed_from_side(
        g_router.r, (uint32_t)g_can_side_id, bytes, len);
  }
  else
  {
    result = seds_router_receive_packed(g_router.r, bytes, len);
  }

  (void)result;
  g_telemetry_discovery_seen = 1U;
#endif
}

static UNUSED_FUNCTION void rx_synchronous(const uint8_t *bytes, size_t len)
{
#ifndef TELEMETRY_ENABLED
  (void)bytes;
  (void)len;
  return;
#else
  if (!bytes || len == 0U)
  {
    return;
  }

  if (!g_router.r && init_telemetry_router() != SEDS_OK)
  {
    return;
  }

  if (g_can_side_id >= 0)
  {
    (void)seds_router_receive_packed_from_side(g_router.r, (uint32_t)g_can_side_id, bytes,
                                                   len);
  }
  else
  {
    (void)seds_router_receive_packed(g_router.r, bytes, len);
  }
#endif
}

static void telemetry_update_network_health(SedsRouter *router)
{
  uint64_t network_time_ms = 0ULL;
  if (seds_router_get_network_time_ms(router, &network_time_ms) == SEDS_OK)
  {
    g_telemetry_timesync_valid = 1U;
  }
  if (g_telemetry_discovery_seen != 0U &&
      g_telemetry_timesync_valid != 0U)
  {
    g_telemetry_network_ready = 1U;
  }
}

SedsResult telemetry_poll_timesync(void)
{
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (init_telemetry_router() != SEDS_OK)
  {
    return SEDS_ERR;
  }

  const SedsResult result = seds_router_poll_timesync(g_router.r, NULL);
  telemetry_update_network_health(g_router.r);
  return result;
#endif
}

SedsResult telemetry_announce_discovery(void)
{
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (init_telemetry_router() != SEDS_OK)
  {
    return SEDS_ERR;
  }

  return seds_router_announce_discovery(g_router.r);
#endif
}

SedsResult telemetry_poll_discovery(void)
{
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (init_telemetry_router() != SEDS_OK)
  {
    return SEDS_ERR;
  }

  bool did_queue = false;
  (void)flight_state_cache_poll(g_router.r);
  const SedsResult result = seds_router_poll_discovery(g_router.r, &did_queue);
  if (result == SEDS_OK) {
    sim_probe_emit_heartbeat(g_router.r, telemetry_now_ms());
  }
  telemetry_update_network_health(g_router.r);
  return result;
#endif
}

SedsResult telemetry_publish_umbilical_status(uint8_t cmd_id, uint8_t on)
{
  const uint8_t payload[2] = {cmd_id, (uint8_t)((on != 0U) ? 1U : 0U)};

#ifndef TELEMETRY_ENABLED
  (void)payload;
  return SEDS_OK;
#else
  SedsResult result;
  if (on != 0U)
  {
    if (!g_router.r && init_telemetry_router() != SEDS_OK)
    {
      result = SEDS_ERR;
    }
    else
    {
      /* Command acknowledgements must reach the wire immediately.  Do not
       * place an asserted valve state behind the periodic telemetry queue.
       * A full CAN mailbox or a momentarily fragmented allocator can clear on
       * the following scheduler tick, so retry this safety-relevant ACK for a
       * small bounded interval instead of silently losing it. */
      result = SEDS_ERR;
      for (uint32_t attempt = 0U; attempt < 3U; ++attempt)
      {
        result = seds_router_log_typed(g_router.r, SEDS_DT_UMBILICAL_STATUS,
                                       payload, 2U, sizeof(payload[0]),
                                       SEDS_EK_UNSIGNED);
        if (result == SEDS_OK)
        {
          break;
        }
        tx_thread_sleep(1U);
      }
    }
  }
  else
  {
    result = log_telemetry_asynchronous(SEDS_DT_UMBILICAL_STATUS, payload, 2U,
                                        sizeof(payload[0]));
  }
#ifdef SEDS_FIRMWARE_SIM_TEST
  if (result == SEDS_OK) {
    g_sim_umbilical_status_ok++;
  } else {
    g_sim_umbilical_status_fail++;
  }
#endif
  return result;
#endif
}

SedsResult telemetry_send_actuator_command(uint8_t cmd_id)
{
#ifndef TELEMETRY_ENABLED
  (void)cmd_id;
  return SEDS_OK;
#else
  if (!g_router.r && init_telemetry_router() != SEDS_OK)
  {
    return SEDS_ERR;
  }

  return seds_router_log_typed_ex(g_router.r, SEDS_DT_ACTUATOR_COMMAND, &cmd_id, 1U,
                                  sizeof(cmd_id), SEDS_EK_UNSIGNED, NULL, 1);
#endif
}

SedsResult telemetry_send_actuator_command_at(uint8_t cmd_id, uint64_t timestamp_ms)
{
#ifndef TELEMETRY_ENABLED
  (void)cmd_id;
  (void)timestamp_ms;
  return SEDS_OK;
#else
  if (!g_router.r && init_telemetry_router() != SEDS_OK)
  {
    return SEDS_ERR;
  }

  return seds_router_log_typed_ex(g_router.r, SEDS_DT_ACTUATOR_COMMAND, &cmd_id, 1U,
                                  sizeof(cmd_id), SEDS_EK_UNSIGNED, &timestamp_ms, 1);
#endif
}

SedsResult telemetry_broadcast_abort(const char *reason)
{
  static const char default_reason[] = "Valve board abort";

  if (g_abort_broadcast_sent != 0U)
  {
    return SEDS_OK;
  }

  g_abort_broadcast_sent = 1U;
  return log_telemetry_string_asynchronous(SEDS_DT_ABORT,
                                           (reason != NULL) ? reason : default_reason);
}

SedsResult telemetry_publish_flight_state_at(uint8_t flight_state, uint64_t timestamp_ms)
{
#ifndef TELEMETRY_ENABLED
  (void)flight_state;
  (void)timestamp_ms;
  return SEDS_OK;
#else
  if (!g_router.r && init_telemetry_router() != SEDS_OK)
  {
    return SEDS_ERR;
  }

  return seds_router_log_typed_ex(g_router.r, SEDS_DT_FLIGHT_STATE, &flight_state, 1U,
                                  sizeof(flight_state), SEDS_EK_UNSIGNED, &timestamp_ms, 1);
#endif
}

SedsResult telemetry_publish_flight_state(uint8_t flight_state)
{
  const uint64_t timestamp_ms = telemetry_unix_ms();

  if (timestamp_ms == 0ULL)
  {
    return SEDS_ERR;
  }

  return telemetry_publish_flight_state_at(flight_state, timestamp_ms);
}

SedsResult init_telemetry_router(void)
{
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  SedsRouter *r = NULL;
  SedsResult result = SEDS_OK;

  if (g_router.created && g_router.r)
  {
    g_telemetry_init_error_code = TELEMETRY_INIT_OK;
    return SEDS_OK;
  }

  if (!g_can_rx_subscribed)
  {
    if (can_bus_subscribe_rx(telemetry_can_rx, NULL) == HAL_OK)
    {
      g_can_rx_subscribed = 1U;
    }
    else
    {
      g_telemetry_init_error_code = TELEMETRY_INIT_SUBSCRIBE_RX_FAILED;
      printf("Error %ld: can_bus_subscribe_rx failed\r\n",
             (long)g_telemetry_init_error_code);
    }
  }
  static const SedsLocalEndpointDesc locals[] = {
      {
          .endpoint = SEDS_EP_VALVE_BOARD,
          .packet_handler = Valve_Command_handler,
          .packed_handler = NULL,
          .user = NULL,
      },
      {
          .endpoint = SEDS_EP_ABORT,
          .packet_handler = Abort_handler,
          .packed_handler = NULL,
          .user = NULL,
      },
      {
          .endpoint = SEDS_EP_FLIGHT_STATE,
          .packet_handler = Flight_State_handler,
          .packed_handler = NULL,
          .user = NULL,
      },
      {
          .endpoint = SEDS_EP_HEART_BEAT,
          .packet_handler = Heartbeat_handler,
          .packed_handler = NULL,
          .user = NULL,
      }};

  r = seds_router_new(Seds_RM_Relay, node_now_since_ms, NULL, locals,
                      sizeof(locals) / sizeof(locals[0]));
  if (!r)
  {
    g_telemetry_init_error_code = TELEMETRY_INIT_ROUTER_NEW_FAILED;
    printf("Error %ld: failed to create router\r\n",
           (long)g_telemetry_init_error_code);
    g_router.r = NULL;
    g_router.created = 0U;
    g_can_side_id = -1;
    return SEDS_ERR;
  }

  g_can_side_id = seds_router_add_side_packed(r, "can", 3U, tx_send, NULL, false);
  if (g_can_side_id < 0)
  {
    g_telemetry_init_error_code = TELEMETRY_INIT_ADD_CAN_SIDE_FAILED;
    printf("Error %ld: failed to add CAN side: %ld\r\n",
           (long)g_telemetry_init_error_code, (long)g_can_side_id);
    g_can_side_id = -1;
  }

  /* Keep the compact address summary used by discovery routing, but avoid
   * constructing the diagnostic full-network graph on this MCU. */
  if (g_can_side_id >= 0 &&
      seds_router_set_typed_route(r, -1, SEDS_DT_DISCOVERY_TOPOLOGY,
                                  g_can_side_id, false) != SEDS_OK)
  {
    seds_router_free(r);
    g_router.r = NULL;
    g_router.created = 0U;
    g_can_side_id = -1;
    return SEDS_ERR;
  }

  result = telemetry_configure_timesync_locked(r);
  if (result != SEDS_OK)
  {
    g_telemetry_init_error_code = TELEMETRY_INIT_CONFIGURE_TIMESYNC_FAILED;
    printf("Error %ld: failed to configure telemetry timesync: %d\r\n",
           (long)g_telemetry_init_error_code, (int)result);
    seds_router_free(r);
    g_router.r = NULL;
    g_router.created = 0U;
    g_can_side_id = -1;
    return result;
  }

  result = ota_stream_init(r);
  if (result != SEDS_OK) {
    printf("Error: failed to bind OTA stream: %d\r\n", (int)result);
    seds_router_free(r);
    g_router.r = NULL;
    g_router.created = 0U;
    return result;
  }

  /* Discovery begins from the normal poll loop after CAN startup. */

  g_telemetry_init_error_code = TELEMETRY_INIT_OK;
  g_router.r = r;
  (void)flight_state_cache_init(r);
  g_router.created = 1U;
  g_router.start_time = tx_raw_now_ms_locked();
  return SEDS_OK;
#endif
}

static inline SedsElemKind guess_kind_from_elem_size(size_t elem_size)
{
  if (elem_size == 4U || elem_size == 8U)
  {
    return SEDS_EK_FLOAT;
  }
  return SEDS_EK_UNSIGNED;
}

SedsResult log_telemetry_synchronous(SedsDataType data_type, const void *data,
                                     size_t element_count, size_t element_size)
{
#ifdef TELEMETRY_ENABLED
  if (!data || element_count == 0U || element_size == 0U)
  {
    return SEDS_BAD_ARG;
  }

  if (telemetry_link_recently_active() == 0U)
  {
    return SEDS_IO;
  }

  if (!g_router.r && init_telemetry_router() != SEDS_OK)
  {
    return SEDS_ERR;
  }

  return seds_router_log_typed(g_router.r, data_type, data, element_count, element_size,
                               guess_kind_from_elem_size(element_size));
#else
  (void)data_type;
  print_data_no_telem((void *)data, element_count * element_size);
  return SEDS_OK;
#endif
}

SedsResult log_telemetry_asynchronous(SedsDataType data_type, const void *data,
                                      size_t element_count, size_t element_size)
{
#ifdef TELEMETRY_ENABLED
  if (!data || element_count == 0U || element_size == 0U)
  {
    return SEDS_BAD_ARG;
  }

  if (!g_router.r && init_telemetry_router() != SEDS_OK)
  {
    return SEDS_ERR;
  }

  return seds_router_log_queue_typed(g_router.r, data_type, data, element_count, element_size,
                                     guess_kind_from_elem_size(element_size));
#else
  (void)data_type;
  print_data_no_telem((void *)data, element_count * element_size);
  return SEDS_OK;
#endif
}

SedsResult log_telemetry_string_asynchronous(SedsDataType data_type, const char *str)
{
#ifdef TELEMETRY_ENABLED
  if (!str)
  {
    return SEDS_BAD_ARG;
  }

  if (telemetry_link_recently_active() == 0U)
  {
    return SEDS_IO;
  }

  if (!g_router.r && init_telemetry_router() != SEDS_OK)
  {
    return SEDS_ERR;
  }

  return seds_router_log_string_ex(g_router.r, data_type, str, strlen(str), NULL, 1);
#else
  (void)data_type;
  (void)str;
  return SEDS_OK;
#endif
}

SedsResult dispatch_tx_queue(void)
{
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r && init_telemetry_router() != SEDS_OK)
  {
    return SEDS_ERR;
  }

  return seds_router_process_tx_queue(g_router.r);
#endif
}

SedsResult process_rx_queue(void)
{
#ifndef TELEMETRY_ENABLED
  return SEDS_OK;
#else
  if (!g_router.r && init_telemetry_router() != SEDS_OK)
  {
    return SEDS_ERR;
  }

  return seds_router_process_rx_queue(g_router.r);
#endif
}

SedsResult dispatch_tx_queue_timeout(uint32_t timeout_ms)
{
#ifndef TELEMETRY_ENABLED
  (void)timeout_ms;
  return SEDS_OK;
#else
  if (!g_router.r && init_telemetry_router() != SEDS_OK)
  {
    return SEDS_ERR;
  }

  return seds_router_process_tx_queue_with_timeout(g_router.r, timeout_ms);
#endif
}

SedsResult process_rx_queue_timeout(uint32_t timeout_ms)
{
#ifndef TELEMETRY_ENABLED
  (void)timeout_ms;
  return SEDS_OK;
#else
  if (!g_router.r && init_telemetry_router() != SEDS_OK)
  {
    return SEDS_ERR;
  }

  return seds_router_process_rx_queue_with_timeout(g_router.r, timeout_ms);
#endif
}

SedsResult process_all_queues_timeout(uint32_t timeout_ms)
{
#ifndef TELEMETRY_ENABLED
  (void)timeout_ms;
  return SEDS_OK;
#else
  if (!g_router.r && init_telemetry_router() != SEDS_OK)
  {
    return SEDS_ERR;
  }

  return seds_router_process_all_queues_with_timeout(g_router.r, timeout_ms);
#endif
}

static UNUSED_FUNCTION SedsResult log_error_impl(uint8_t queue, const char *fmt, va_list args)
{
  va_list args_copy;
  int len = 0;
  int written = 0;

  if (!fmt)
  {
    return SEDS_BAD_ARG;
  }

  if (!g_router.r && init_telemetry_router() != SEDS_OK)
  {
    return SEDS_ERR;
  }

  va_copy(args_copy, args);
  len = vsnprintf(NULL, 0U, fmt, args_copy);
  va_end(args_copy);

  if (len < 0)
  {
    const char *empty = "";
    return seds_router_log_string_ex(g_router.r, SEDS_DT_TELEMETRY_ERROR, empty, 0U, NULL, queue);
  }

  if (len > 512)
  {
    len = 512;
  }

  char buf[(size_t)len + 1U];
  written = vsnprintf(buf, (size_t)len + 1U, fmt, args);
  if (written < 0)
  {
    const char *empty = "";
    return seds_router_log_string_ex(g_router.r, SEDS_DT_TELEMETRY_ERROR, empty, 0U, NULL, queue);
  }

  return seds_router_log_string_ex(g_router.r, SEDS_DT_TELEMETRY_ERROR, buf, (size_t)written,
                                   NULL, queue);
}

SedsResult log_error_asynchronous(const char *fmt, ...)
{
#ifndef TELEMETRY_ENABLED
  (void)fmt;
  return SEDS_OK;
#else
  va_list args;
  SedsResult result;

  va_start(args, fmt);
  result = log_error_impl(1U, fmt, args);
  va_end(args);
  return result;
#endif
}

SedsResult log_error_synchronous(const char *fmt, ...)
{
#ifndef TELEMETRY_ENABLED
  (void)fmt;
  return SEDS_OK;
#else
  va_list args;
  SedsResult result;

  va_start(args, fmt);
  result = log_error_impl(0U, fmt, args);
  va_end(args);
  return result;
#endif
}

SedsResult log_error_asyncronous(const char *fmt, ...)
{
#ifndef TELEMETRY_ENABLED
  (void)fmt;
  return SEDS_OK;
#else
  va_list args;
  SedsResult result;

  va_start(args, fmt);
  result = log_error_impl(1U, fmt, args);
  va_end(args);
  return result;
#endif
}

SedsResult log_error_syncronous(const char *fmt, ...)
{
#ifndef TELEMETRY_ENABLED
  (void)fmt;
  return SEDS_OK;
#else
  va_list args;
  SedsResult result;

  va_start(args, fmt);
  result = log_error_impl(0U, fmt, args);
  va_end(args);
  return result;
#endif
}

SedsResult print_telemetry_error(const int32_t error_code)
{
#ifndef TELEMETRY_ENABLED
  (void)error_code;
  return SEDS_OK;
#else
  const int32_t need = seds_error_to_string_len(error_code);
  if (need <= 0)
  {
    return (SedsResult)need;
  }

  char buf[(size_t)need];
  SedsResult res = seds_error_to_string(error_code, buf, sizeof(buf));
  if (res == SEDS_OK)
  {
    printf("Error: %s\r\n", buf);
  }
  else
  {
    (void)log_error_asynchronous("Error: seds_error_to_string failed: %d\r\n", (int)res);
  }

  return res;
#endif
}

void die(const char *fmt, ...)
{
  char buf[128];
  va_list args;

  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  while (1)
  {
    printf("FATAL: %s\r\n", buf);
    HAL_Delay(1000);
  }
}
