#include "persistent_store.h"
#include "ota_stream.h"

#include "board_config.h"
#include "launchcore/storage.h"
#include "launchcore/update.h"
#include "launchcore/metadata.h"
#include "main.h"
#include "telemetry.h"
#include "thread_comm.h"
#include "tx_api.h"
#include <string.h>
#include <stdbool.h>
#include <stddef.h>

#define OTA_OP_BEGIN_DELTA 0x01U
#define OTA_OP_CHUNK 0x02U
#define OTA_OP_FINISH 0x03U
#define OTA_OP_ABORT 0x04U
#define OTA_OP_STATUS 0x05U
#define OTA_OP_ENTER_RECOVERY 0x06U
#define OTA_RESPONSE_FLAG 0x80U
#define OTA_CONFIRM_DELAY_MS 5000ULL
#define OTA_RECOVERY_RESET_DELAY_MS 250ULL

extern const launchcore_storage_driver_t launchcore_board_storage_driver;

typedef struct {
  SedsRouter *router;
  uint32_t stream_id;
  uint32_t expected_offset;
  uint32_t declared_size;
  uint64_t initialized_ms;
  uint64_t reset_requested_ms;
  uint32_t reset_epoch;
  bool active;
  bool connected;
  bool boot_confirmed;
  bool reset_requested;
} ota_stream_state_t;

static ota_stream_state_t state;
/* One actual telemetry task owns this state; callbacks never retain SDK views. */
extern TX_THREAD telemetry_thread;
static bool owner_registered;
static uint32_t stream_generation;
enum { WORK_EMPTY, WORK_REQUEST, WORK_REPLY };
enum { CLEAN_NONE, CLEAN_PENDING, CLEAN_FAILED };
typedef struct {
  uint32_t generation,stream_id,sequence,original_epoch;
  uint16_t length;
  uint8_t phase,opcode;
  uint8_t payload[5U+OTA_STREAM_MAX_CHUNK],reply[13],result,reply_pending;
} OtaWork;
typedef struct {uint32_t generation,stream_id;uint8_t phase,error;uint16_t reserved;} OtaTerminal;
static OtaWork work;
static OtaTerminal terminal;
volatile uint32_t g_ota_service_errors;
volatile uint32_t g_ota_cleanup_result;
static bool owner_ready(void) {
  return __get_IPSR()==0U && tx_thread_identify()==&telemetry_thread && owner_registered;
}
bool ota_stream_register_owner(void) {
  if(__get_IPSR()!=0U||tx_thread_identify()!=&telemetry_thread)return false;
  owner_registered=true;return true;
}
static void retire_stream(void) {
  if(stream_generation!=UINT32_MAX)++stream_generation;
  work.phase=WORK_EMPTY;state.reset_requested=false;state.connected=false;
  if(terminal.phase==CLEAN_NONE){terminal=(OtaTerminal){stream_generation,state.stream_id,CLEAN_PENDING,0,0};g_ota_cleanup_result=1U;}
}


/* A failed owner release is not permission to enter SDK or pretend cleanup.
 * Quarantine only this service; the independent raw ManualStop path remains. */
static void cleanup_failed(bool uncertain_owner) {
  work.phase=WORK_EMPTY;state.connected=false;state.reset_requested=false;
  terminal.phase=CLEAN_FAILED;
  terminal.error=(uint8_t)(OTA_STREAM_STORAGE_ERROR|(uncertain_owner?0x80U:0U));
  g_ota_cleanup_result=OTA_STREAM_STORAGE_ERROR;++g_ota_service_errors;
}

static uint32_t read_u32_le(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static void write_u32_le(uint8_t *p, uint32_t value)
{
  p[0] = (uint8_t)value;
  p[1] = (uint8_t)(value >> 8);
  p[2] = (uint8_t)(value >> 16);
  p[3] = (uint8_t)(value >> 24);
}

static ota_stream_status_t map_status(launchcore_status_t status)
{
  switch (status) {
  case LAUNCHCORE_OK: return OTA_STREAM_OK;
  case LAUNCHCORE_ERR_NO_SPACE: return OTA_STREAM_NO_SPACE;
  case LAUNCHCORE_ERR_BAD_STATE: return OTA_STREAM_BAD_STATE;
  case LAUNCHCORE_ERR_BAD_IMAGE:
  case LAUNCHCORE_ERR_VERIFY: return OTA_STREAM_BAD_IMAGE;
  case LAUNCHCORE_ERR_FLASH:
  case LAUNCHCORE_ERR_METADATA: return OTA_STREAM_STORAGE_ERROR;
  default: return OTA_STREAM_INTERNAL_ERROR;
  }
}

static void build_response(uint8_t opcode,ota_stream_status_t status) {
  work.reply[0]=(uint8_t)(opcode|OTA_RESPONSE_FLAG);
  write_u32_le(&work.reply[1],(uint32_t)status);
  write_u32_le(&work.reply[5],state.expected_offset);
  write_u32_le(&work.reply[9],ota_stream_max_patch_size());
  work.result=(uint8_t)status;work.phase=WORK_REPLY;
}
/* e2 delta_update.c targets only delta slotB. Its abort ignores commit
 * failure, so API success alone cannot prove persistent cleanup. */
static launchcore_status_t abort_verified(void) {
  const launchcore_storage_layout_t*layout=launchcore_storage_driver()->layout();
  launchcore_metadata_t expected,observed;
  if(!layout||!layout->slot_b_is_delta||launchcore_metadata_read_latest(&expected)!=LAUNCHCORE_METADATA_OK||
     expected.sequence==UINT32_MAX||expected.slot_b_state!=LAUNCHCORE_SLOT_WRITING)return LAUNCHCORE_ERR_METADATA;
  launchcore_status_t result=launchcore_delta_update_abort();
  if(result!=LAUNCHCORE_OK)return result;
  if(launchcore_metadata_read_latest(&observed)!=LAUNCHCORE_METADATA_OK)return LAUNCHCORE_ERR_METADATA;
  ++expected.sequence;expected.slot_b_state=LAUNCHCORE_SLOT_BAD;expected.update_slot=LAUNCHCORE_SLOT_NONE;
  expected.metadata_crc32=launchcore_metadata_crc(&expected);
  return memcmp(&expected,&observed,sizeof expected)==0?LAUNCHCORE_OK:LAUNCHCORE_ERR_METADATA;
}

static ota_stream_status_t handle_data(const uint8_t *payload, size_t len)
{
  if (payload == NULL || len == 0U) return OTA_STREAM_BAD_MESSAGE;
  const uint8_t opcode = payload[0];

  if (opcode == OTA_OP_BEGIN_DELTA) {
    if (len != 5U || state.active) return OTA_STREAM_BAD_STATE;
    const uint32_t size = read_u32_le(&payload[1]);
    launchcore_status_t result = launchcore_delta_update_begin(size);
    if (result == LAUNCHCORE_OK) {
      state.active = true;
      state.declared_size = size;
      state.expected_offset = 0U;
    }
    return map_status(result);
  }

  if (opcode == OTA_OP_CHUNK) {
    if (!state.active || len < 6U || len > 5U + OTA_STREAM_MAX_CHUNK)
      return OTA_STREAM_BAD_STATE;
    const uint32_t offset = read_u32_le(&payload[1]);
    const size_t chunk_len = len - 5U;
    if (offset != state.expected_offset) return OTA_STREAM_BAD_OFFSET;
    if (offset > state.declared_size || chunk_len > state.declared_size - offset)
      return OTA_STREAM_NO_SPACE;
    if ((offset & 7U) != 0U ||
        (offset + chunk_len < state.declared_size && (chunk_len & 7U) != 0U))
      return OTA_STREAM_BAD_MESSAGE;
    launchcore_status_t result = launchcore_delta_update_write(&payload[5], chunk_len);
    if (result == LAUNCHCORE_OK) state.expected_offset += (uint32_t)chunk_len;
    return map_status(result);
  }

  if (opcode == OTA_OP_FINISH) {
    if (len != 1U || !state.active) return OTA_STREAM_BAD_STATE;
    launchcore_status_t result = launchcore_delta_update_finish();
    if (result == LAUNCHCORE_OK) {
      state.active = false;
      state.reset_requested = true;
      state.reset_requested_ms = telemetry_now_ms();
    }
    return map_status(result);
  }

  if (opcode == OTA_OP_ABORT) {
    if (len != 1U || !state.active) return OTA_STREAM_BAD_STATE;
    launchcore_status_t result = abort_verified();
    if (result == LAUNCHCORE_OK) state.active = false;
    return map_status(result);
  }

  if (opcode == OTA_OP_STATUS && len == 1U) return OTA_STREAM_OK;

  if (opcode == OTA_OP_ENTER_RECOVERY && len == 1U && !state.active) {
    launchcore_status_t result = launchcore_request_recovery();
    if (result == LAUNCHCORE_OK) {
      state.reset_requested = true;
      state.reset_requested_ms = telemetry_now_ms();
    }
    return map_status(result);
  }

  return OTA_STREAM_BAD_MESSAGE;
}

static SedsResult ota_stream_event(const SedsP2pStreamEventView *event,void *user) {
  (void)user;
  if(!event)return SEDS_BAD_ARG;
  if(!owner_ready())return SEDS_ERR;
  if(event->kind==SEDS_P2P_STREAM_ACCEPTED){
    if(terminal.phase==CLEAN_FAILED&&(terminal.error&0x80U))return SEDS_ERR;
    if(state.connected||state.active||work.phase!=WORK_EMPTY||terminal.phase!=CLEAN_NONE||state.reset_requested||stream_generation==UINT32_MAX){
      /* SDK inserted this session before callback. Retire only the refused
       * extra session; an error return alone would leak its admission. */
      (void)seds_router_reset_p2p_stream(state.router,event->stream_id);
      return SEDS_ERR;
    }
    ++stream_generation;state.stream_id=event->stream_id;state.connected=true;return SEDS_OK;
  }
  if(event->stream_id!=state.stream_id)return SEDS_ERR;
  if(event->kind==SEDS_P2P_STREAM_CLOSED||event->kind==SEDS_P2P_STREAM_RESET){
    if(terminal.phase==CLEAN_NONE)retire_stream();
    return SEDS_OK; /* Own cleanup before returning; SDK session already gone. */
  }
  if(event->kind!=SEDS_P2P_STREAM_DATA||!state.connected||terminal.phase!=CLEAN_NONE||state.reset_requested)return SEDS_ERR;
  if(!event->payload||!event->payload_len||event->payload_len>sizeof(work.payload))return SEDS_BAD_ARG;
  if(work.phase!=WORK_EMPTY){
    return work.generation==stream_generation&&work.stream_id==event->stream_id&&work.sequence==event->sequence&&work.length==event->payload_len&&!memcmp(work.payload,event->payload,work.length)?SEDS_OK:SEDS_ERR;
  }
  uint32_t epoch=thread_comm_manual_epoch();
  if(!epoch||epoch==UINT32_MAX||thread_comm_get_abort())return SEDS_ERR;
  work.generation=stream_generation;work.stream_id=event->stream_id;work.sequence=event->sequence;work.original_epoch=epoch;
  work.length=(uint16_t)event->payload_len;work.opcode=event->payload[0];memcpy(work.payload,event->payload,event->payload_len);
  work.phase=WORK_REQUEST;return SEDS_OK; /* Request retained, no application OK yet. */
}

uint32_t ota_stream_max_patch_size(void) { return BOARD_DELTA_SIZE; }

SedsResult ota_stream_init(SedsRouter *router)
{
  if (router == NULL) return SEDS_BAD_ARG;
  if(state.connected||state.active||work.phase!=WORK_EMPTY||terminal.phase!=CLEAN_NONE||state.reset_requested)return SEDS_ERR;
  if (!persistent_store_enter()) return SEDS_ERR;
  launchcore_storage_set_driver(&launchcore_board_storage_driver);
  const launchcore_storage_status_t status = launchcore_storage_init();
  const bool released = persistent_store_leave();
  if(!released||status!=LAUNCHCORE_STORAGE_OK){cleanup_failed(!released);return SEDS_ERR;}
  state = (ota_stream_state_t){.router = router, .initialized_ms = telemetry_now_ms()};
  return seds_router_bind_p2p_stream_port(router, OTA_STREAM_PORT, ota_stream_event, NULL);
}

static void reset_now(void*unused){(void)unused;state.reset_requested=false;NVIC_SystemReset();}
void ota_stream_poll(void) {
  if(!owner_ready()||!state.router)return;
  const uint64_t now=telemetry_now_ms();
  if((work.phase!=WORK_EMPTY||state.reset_requested)&&
     (thread_comm_get_abort()||thread_comm_manual_epoch()!=(work.phase!=WORK_EMPTY?work.original_epoch:state.reset_epoch)))retire_stream();
  if(terminal.phase==CLEAN_PENDING){
    bool busy=false;if(!persistent_store_try_enter(&busy)){if(!busy){terminal.phase=CLEAN_FAILED;terminal.error=OTA_STREAM_STORAGE_ERROR;g_ota_cleanup_result=OTA_STREAM_STORAGE_ERROR;++g_ota_service_errors;}return;}
    launchcore_status_t result=state.active?abort_verified():LAUNCHCORE_OK;
    bool released=persistent_store_leave();
    if(result!=LAUNCHCORE_OK||!released){cleanup_failed(!released);return;}
    state.active=false;state.stream_id=0U;terminal.phase=CLEAN_NONE;g_ota_cleanup_result=0U;return;
  }
  if(terminal.phase==CLEAN_FAILED)return; /* Visible fault denies OTA only. */
  if(work.phase==WORK_REQUEST){
    if(work.generation!=stream_generation||work.stream_id!=state.stream_id){retire_stream();return;}
    if(work.opcode==OTA_OP_FINISH||work.opcode==OTA_OP_ENTER_RECOVERY){
      int reset_allowed=thread_comm_ota_reset_original(work.original_epoch,NULL,NULL);
      if(reset_allowed<0)return;
      if(!reset_allowed){build_response(work.opcode,OTA_STREAM_BAD_STATE);return;}
    }
    bool busy=false;if(!persistent_store_try_enter(&busy)){if(!busy){++g_ota_service_errors;build_response(work.opcode,OTA_STREAM_STORAGE_ERROR);}return;}
    ota_stream_status_t result=handle_data(work.payload,work.length);
    bool released=persistent_store_leave();
    if(!released){retire_stream();cleanup_failed(true);return;}
    if(thread_comm_get_abort()||thread_comm_manual_epoch()!=work.original_epoch){retire_stream();return;}
    if(state.reset_requested)state.reset_epoch=work.original_epoch;
    build_response(work.opcode,result);return;
  }
  if(work.phase==WORK_REPLY){
    if(work.generation!=stream_generation||work.stream_id!=state.stream_id){retire_stream();return;}
    if(seds_router_send_p2p_stream(state.router,work.stream_id,work.reply,sizeof work.reply)==SEDS_OK)work.phase=WORK_EMPTY;
    return; /* Never re-run handle_data when SDK retention refused. */
  }
  if(state.reset_requested&&now-state.reset_requested_ms>=OTA_RECOVERY_RESET_DELAY_MS){
    /* Final accepted-Launch/manual epoch owner check and reset are one bounded
     * state-owned effect. Launch admission is never gated by OTA. */
    int result=thread_comm_ota_reset_original(state.reset_epoch,reset_now,NULL);
    if(result==0)state.reset_requested=false; /* Retired, never auto-resume. */
    return;
  }
  if(!state.boot_confirmed&&now-state.initialized_ms>=OTA_CONFIRM_DELAY_MS){
    bool busy=false;if(!persistent_store_try_enter(&busy))return;
    state.boot_confirmed=launchcore_confirm_boot()==LAUNCHCORE_OK;
    if(!persistent_store_leave()){state.boot_confirmed=false;cleanup_failed(true);}
    else if(!state.boot_confirmed)cleanup_failed(false);
  }
}
