/* Actual VB handlers inserted by the test, linked to the real host SDK.
 * A reliable packet that VB deliberately refuses (stale, aborted, malformed,
 * unauthorized, refused for safety) must still earn its SDK end-to-end
 * receipt, or the station retransmits it forever. Only transient failures
 * withhold the receipt. Owners/storage/ThreadX are inert fixtures. */
#include "sedsnet.h"
#include "sedsnet_config.h"
#include "board_commands.h"
#include "board_recovery_wire.h"
#include "board_recovery.h"
#include "board_bootstrap_storage.h"
#include "flight_state_cache.h"
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef unsigned UINT;typedef unsigned long ULONG;typedef ULONG thread_comm_msg_t;
#define TX_SUCCESS 0U
#define TX_QUEUE_ERROR 0x09U
#define TX_QUEUE_FULL 0x0BU
#define TX_MUTEX_ERROR 0x1CU
#define TX_TIMER_TICKS_PER_SECOND 1000U
#define THREAD_COMM_STALE_TIMESTAMP ((UINT)0xF001U)
#define THREAD_COMM_AUTOMATIC_TAG 0x80U
#define THREAD_COMM_LAUNCH_SENDER_MAX 32U
#define FLIGHT_STATE_PERSIST_KEY 0x46535445u
#define FLIGHT_STATE_MAX_VALUE 15U
#define LAUNCHCORE_PERSIST_OK 0
volatile uint32_t g_reliable_refusal_count;
static struct{SedsRouter*r;}g_router;
static SedsRouter*g_cache_router;
/* Fixture state */
static unsigned stale,aborted,allowed=1,applies,admits,aborts,persists,acks;
static UINT apply_result=TX_SUCCESS,admit_result=TX_SUCCESS;
static volatile uint32_t g_flight_state_handler_count,g_flight_state_handler_error_count;
static volatile uint8_t g_last_flight_state_packet;
static volatile uint32_t g_sim_valve_commands_received,g_sim_groundstation_confirmation_received;
static uint8_t g_sim_valve_open_seen;
static volatile uint32_t g_flight_state_cache_errors,g_flight_state_cache_value,g_flight_state_cache_updates,g_flight_state_cache_writes;
static bool g_persist_ready=true,g_has_value,g_network_value_seen;static uint8_t g_persisted_state=UINT8_MAX;
static SedsResult board_recovery_sync_command_cutoff(void){return SEDS_OK;}
static uint8_t board_recovery_command_is_stale(const SedsPacketView*p){(void)p;return (uint8_t)stale;}
static uint8_t board_recovery_automatic_is_stale(const SedsPacketView*p){(void)p;return (uint8_t)stale;}
static uint8_t board_recovery_managed_update_allowed(const SedsPacketView*p){(void)p;return (uint8_t)allowed;}
static uint8_t board_recovery_manual_stop_source_allowed(SedsRouter*r,const SedsPacketView*p){(void)r;(void)p;return 0;}
static uint8_t thread_comm_get_abort(void){return (uint8_t)aborted;}
static uint8_t applied_state=UINT8_MAX;
static UINT thread_comm_apply_network_flight_state(uint8_t s,uint64_t t){(void)t;if(apply_result==TX_SUCCESS){++applies;applied_state=s;}return apply_result;}
static volatile uint32_t g_flight_state_contract_violations,g_flight_state_contract_logged,g_flight_state_contract_first_raw;
static UINT thread_comm_admit_network(thread_comm_msg_t m,uint64_t t){(void)m;(void)t;if(admit_result==TX_SUCCESS)++admits;return admit_result;}
static UINT thread_comm_admit_network_automatic(thread_comm_msg_t m,uint64_t t){return thread_comm_admit_network(m,t);}
static UINT thread_comm_request_accepted_launch_manual_abort(uint64_t t){(void)t;return TX_QUEUE_ERROR;}
static UINT thread_comm_launch_original_accepted(uint64_t t,const char*s,uint32_t n){(void)t;(void)s;(void)n;return 0;}
static UINT admit_timed_launch(const SedsPacketView*p){(void)p;return thread_comm_admit_network(CMD_SEQUENCE,0);}
static void thread_comm_note_overpressure_ack(void){}
static UINT thread_comm_request_manual_abort(void){++aborts;aborted=1;return TX_SUCCESS;}
static UINT thread_comm_request_manual_abort_original_utc(uint64_t t){(void)t;return thread_comm_request_manual_abort();}
static UINT thread_comm_request_automatic_abort(void){return thread_comm_request_manual_abort();}
static SedsResult telemetry_broadcast_abort(const char*s){(void)s;return SEDS_OK;}
static int persistent_store_try_set(uint32_t k,const void*v,size_t n,bool*busy){(void)k;(void)v;(void)n;*busy=false;++persists;return LAUNCHCORE_PERSIST_OK;}
static ULONG tx_time_get(void){return 1;}
/* Recovery collaborators: the refused paths never reach them. */
static uint8_t authorized_primary(SedsRouter*router,const SedsPacketView*packet);
static void board_recovery_observe_station_time(SedsRouter*r,const SedsPacketView*p){(void)r;(void)p;}
static bool bootstrap_request_fresh(uint64_t s,uint64_t n){(void)s;(void)n;abort();}
static uint64_t board_recovery_station_now(SedsRouter*r){(void)r;abort();}
uint8_t board_recovery_decode_request(const uint8_t*b,size_t n,BoardRecoveryRequest*o){(void)b;(void)n;(void)o;abort();}
void board_recovery_encode_status(uint8_t o[BOARD_RECOVERY_STATUS_SIZE],uint8_t r,uint8_t s,uint8_t f,const BoardRecoveryRequest*q,uint64_t g,uint32_t e,uint32_t u,uint32_t m){(void)o;(void)r;(void)s;(void)f;(void)q;(void)g;(void)e;(void)u;(void)m;abort();}
int32_t board_recovery_snapshot(BoardRecoverySnapshot*o){(void)o;abort();}
uint64_t board_recovery_boot_current(void){abort();}
uint64_t board_bootstrap_storage_query_generation(void){abort();}
bool board_bootstrap_storage_pending(void){abort();}
bool board_bootstrap_storage_uncertain_first(void){abort();}
bool board_bootstrap_storage_query_healthy(void){abort();}
bool board_bootstrap_storage_note_query(const BoardRecoveryRequest*r,uint64_t a,uint64_t b,uint32_t c){(void)r;(void)a;(void)b;(void)c;abort();}
int32_t board_bootstrap_storage_stage_first_leg(const BoardRecoveryRequest*r,uint64_t a,uint64_t b,uint32_t c,void*d,BootstrapReply e,BootstrapNow f){(void)r;(void)a;(void)b;(void)c;(void)d;(void)e;(void)f;abort();}
/* SOURCE_FUNCTION:authorized_primary */
/* SOURCE_FUNCTION:board_recovery_refused */
static uint64_t last_query_issuer;
/* SOURCE_FUNCTION:send_status */
/* SOURCE_FUNCTION:bootstrap_now */
/* SOURCE_FUNCTION:bootstrap_reply */
/* SOURCE_FUNCTION:handle_serialized */
static bool g_persist_pending;static uint32_t g_last_refresh_ms;
static uint32_t HAL_GetTick(void){return 0U;}
/* SOURCE_FUNCTION:clear_managed_cache */
/* SOURCE_FUNCTION:flight_state_cache_from_wire */
/* SOURCE_FUNCTION:persist_pending */
/* SOURCE_FUNCTION:flight_state_admit */
/* SOURCE_FUNCTION:Flight_State_handler */
SedsResult flight_state_cache_apply_network_update(const SedsPacketView *packet);
/* SOURCE_FUNCTION:flight_state_cache_apply_network_update */
/* SOURCE_FUNCTION:persist_update */
/* SOURCE_FUNCTION:network_admission_result */
/* SOURCE_FUNCTION:Valve_Command_handler */
/* SOURCE_FUNCTION:Abort_handler */
static SedsResult Heartbeat_handler(const SedsPacketView*p,void*u){(void)p;(void)u;return SEDS_OK;}
/* SOURCE_FUNCTION:control_packet_handler */
static SedsResult Board_Recovery_handler(const SedsPacketView*p,void*u){(void)u;return handle_serialized(g_router.r,p,SEDS_EP_BOARD_RECOVERY,1U);}

static uint64_t now(void*u){(void)u;return 1767225601000ULL;}
static SedsResult side(const SedsPacketView*p,void*u){(void)u;if(p->sender_len>=7&&!memcmp(p->sender,"E2EACK:",7))acks++;return SEDS_OK;}
static SedsRouter*r;static int32_t sid;static uint64_t stamp=1767225600000ULL;
static unsigned deliver(uint32_t ty,const uint32_t*eps,size_t neps,const char*sender,const uint8_t*payload,size_t len){
 SedsPacketView p={.ty=ty,.data_size=len,.sender=sender,.sender_len=strlen(sender),.endpoints=eps,.num_endpoints=neps,.timestamp=++stamp,.payload=payload,.payload_len=len};
 unsigned before=acks;SedsResult rx=seds_router_receive_from_side(r,(uint32_t)sid,&p);(void)seds_router_process_all_queues(r);
 printf("ty=%u sender=%s rx=%d acks=%u\n",ty,sender,rx,acks-before);return acks-before;
}
static const uint32_t fs_eps[]={SEDS_EP_FLIGHT_STATE,SEDS_EP_SD_CARD,SEDS_EP_GROUND_STATION};
static const uint32_t vc_eps[]={SEDS_EP_GROUND_STATION,SEDS_EP_VALVE_BOARD};
static const uint32_t abort_eps[]={SEDS_EP_GROUND_STATION,SEDS_EP_FLIGHT_CONTROLLER,SEDS_EP_VALVE_BOARD,SEDS_EP_ACTUATOR_BOARD,SEDS_EP_ABORT,SEDS_EP_FLIGHT_STATE,SEDS_EP_SD_CARD};
static const uint32_t rec_eps[]={SEDS_EP_BOARD_RECOVERY};
int main(int argc,char**argv){
 setbuf(stdout,NULL);assert(argc==3);assert(seds_schema_register_json_file(argv[1],strlen(argv[1]))==SEDS_OK);
 SedsLocalEndpointDesc locals[]={{.endpoint=SEDS_EP_BOARD_RECOVERY,.packet_handler=Board_Recovery_handler},{.endpoint=SEDS_EP_VALVE_BOARD,.packet_handler=control_packet_handler},{.endpoint=SEDS_EP_ABORT,.packet_handler=Abort_handler},{.endpoint=SEDS_EP_FLIGHT_STATE,.packet_handler=Flight_State_handler},{.endpoint=SEDS_EP_HEART_BEAT,.packet_handler=Heartbeat_handler}};
 r=seds_router_new(now,NULL,locals,sizeof locals/sizeof locals[0]);assert(r);assert(seds_router_set_sender_id(r,"VB",2)==SEDS_OK);g_router.r=g_cache_router=r;
 assert(seds_router_enable_network_variable(r,SEDS_DT_FLIGHT_STATE,true,false)==SEDS_OK);
 assert(seds_router_on_network_variable_update(r,SEDS_DT_FLIGHT_STATE,persist_update,NULL)==SEDS_OK);
 sid=seds_router_add_side_packet(r,"CAN",3,side,NULL,false);assert(sid>=0);
 const char*mode=argv[2];uint8_t fs=2;
 if(!strcmp(mode,"flight_state")){
  /* Accepted control: applied and acknowledged. */
  assert(deliver(SEDS_DT_FLIGHT_STATE,fs_eps,3,"GS",&fs,1)==1&&applies>=1);
  unsigned base=applies;
  allowed=0;assert(deliver(SEDS_DT_FLIGHT_STATE,fs_eps,3,"GS",&fs,1)==1);allowed=1;          /* managed gate refusal */
  stale=1;assert(deliver(SEDS_DT_FLIGHT_STATE,fs_eps,3,"GS",&fs,1)==1);stale=0;              /* stale */
  aborted=1;assert(deliver(SEDS_DT_FLIGHT_STATE,fs_eps,3,"GS",&fs,1)==1);aborted=0;          /* aborted */
  apply_result=THREAD_COMM_STALE_TIMESTAMP;assert(deliver(SEDS_DT_FLIGHT_STATE,fs_eps,3,"GS",&fs,1)==1); /* state refusal */
  assert(applies==base&&persists<=1);
  apply_result=TX_MUTEX_ERROR;assert(deliver(SEDS_DT_FLIGHT_STATE,fs_eps,3,"GS",&fs,1)==0);   /* transient: retried */
  assert(g_reliable_refusal_count>=4);
  puts("PASS refused FLIGHT_STATE acknowledged, not applied or persisted; transient failure withholds receipt");
 }else if(!strcmp(mode,"in_flight")){
  /* InFlight (16) is in contract set (a): applied, persisted, acknowledged. */
  uint8_t s=16;assert(deliver(SEDS_DT_FLIGHT_STATE,fs_eps,3,"GS",&s,1)==1&&applies>=1&&applied_state==16U);
  assert(g_flight_state_cache_value==16U&&persists==1&&g_flight_state_contract_violations==0U);
  /* Above InFlight is not a FLIGHT_STATE: acknowledged, never applied. */
  const unsigned before=applies;
  s=17;assert(deliver(SEDS_DT_FLIGHT_STATE,fs_eps,3,"GS",&s,1)==1&&applies==before&&g_flight_state_cache_value==16U);
  puts("PASS InFlight FLIGHT_STATE applied and persisted; 17 acknowledged, not applied");
 }else if(!strcmp(mode,"retired")){
  /* Older firmware: a retired value is mapped, logged once and acknowledged,
   * never refused into a retransmit loop. */
  const uint8_t raw[]={4U,9U,14U,3U},mapped[]={2U,16U,13U,2U};
  for(unsigned i=0;i<sizeof raw;i++){
   assert(deliver(SEDS_DT_FLIGHT_STATE,fs_eps,3,"GS",&raw[i],1)==1&&applied_state==mapped[i]&&g_flight_state_cache_value==mapped[i]);
   assert(g_flight_state_contract_violations==i+1U&&g_flight_state_contract_logged==1U&&g_flight_state_contract_first_raw==4U);
  }
  puts("PASS retired FLIGHT_STATE mapped, logged once and acknowledged");
 }else if(!strcmp(mode,"valve_command")){
  uint8_t cmd=CMD_PILOT_OPEN;assert(deliver(SEDS_DT_VALVE_COMMAND,vc_eps,2,"GS",&cmd,1)==1&&admits==1);
  uint8_t bad=THREAD_COMM_AUTOMATIC_TAG|0x7FU;assert(deliver(SEDS_DT_VALVE_COMMAND,vc_eps,2,"GS",&bad,1)==1);  /* malformed */
  uint32_t before=g_reliable_refusal_count;
  admit_result=THREAD_COMM_STALE_TIMESTAMP;assert(deliver(SEDS_DT_VALVE_COMMAND,vc_eps,2,"GS",&cmd,1)==1);  /* refused for safety */
  assert(g_reliable_refusal_count==before+1);
  stale=1;admit_result=TX_SUCCESS;assert(deliver(SEDS_DT_VALVE_COMMAND,vc_eps,2,"GS",&cmd,1)==1);stale=0;
  assert(admits==1);
  admit_result=TX_QUEUE_FULL;assert(deliver(SEDS_DT_VALVE_COMMAND,vc_eps,2,"GS",&cmd,1)==0);  /* transient: retried */
  puts("PASS refused VALVE_COMMAND acknowledged and counted, never admitted; full queue withholds receipt");
 }else if(!strcmp(mode,"recovery")){
  uint8_t req[32]={0};assert(deliver(SEDS_DT_BOARD_RECOVERY_REQUEST,rec_eps,1,"FC",req,sizeof req)==1);
  assert(g_reliable_refusal_count==1);
  puts("PASS unauthorized BOARD_RECOVERY_REQUEST acknowledged without effect");
 }else if(!strcmp(mode,"abort")){
  static const char msg[]="Manual Abort Command Issued";
  assert(deliver(SEDS_DT_ABORT,abort_eps,7,"GS",(const uint8_t*)msg,sizeof msg-1U)==1&&aborts==1&&aborted);
  puts("PASS station ABORT fanned out to VB endpoints is applied and acknowledged");
 }else return 2;
 seds_router_free(r);return 0;
}
