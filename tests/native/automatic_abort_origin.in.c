/* Actual source is inserted by the test; SDK queue follows public NULL=>now
 * metadata contract. Recovery boundary is a fixture, not a hardware claim. */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
typedef unsigned UINT;typedef unsigned long ULONG;typedef int SedsResult;typedef uint32_t SedsDataType;
#define SEDS_OK 0
#define SEDS_BAD_ARG -10
#define SEDS_ERR -1
#define SEDS_IO -14
#define SEDS_DT_ABORT 135612662U
#define TX_WAIT_FOREVER 0xffffffffU
#define TX_SUCCESS 0
#define TX_MUTEX_ERROR 1
#define TX_TIMER_TICKS_PER_SECOND 1000U
#define TELEMETRY_ENABLED 1
static uint8_t g_thread_comm_initialized=1,g_thread_comm_abort,g_safety_report_closing;
static uint32_t g_thread_comm_flight_state;static int g_thread_comm_state_mutex;
static unsigned stops,attempts,failures=1;static uint64_t now_utc=9000,cutoff=0,captured_stamp;static ULONG now=1000;
static UINT tx_mutex_get(int*a,ULONG b){(void)a;(void)b;return 0;}
static UINT tx_mutex_put(int*a){(void)a;return 0;}
static uint8_t board_launch_latch_is_set(void){return 0;}

static void thread_comm_latch_abort_locked(uint8_t m){(void)m;++stops;g_thread_comm_abort=1;}
static unsigned primask;
static uint32_t __get_PRIMASK(void){return primask;}
static void __disable_irq(void){primask=1;}
static void __set_PRIMASK(uint32_t value){primask=value;}
static ULONG tx_time_get(void){return now;}
static char captured[256];static struct{void*r;}g_router={(void*)1};static void*recovery_router=(void*)1;
static int init_telemetry_router(void){return 0;}
static inline uint8_t telemetry_link_recently_active(void){return 1;}
static SedsResult seds_router_log_string_ex(void*r,SedsDataType t,const char*s,size_t n,const uint64_t*stamp,int q){assert(r&&t==SEDS_DT_ABORT&&q);++attempts;if(failures){--failures;return SEDS_IO;}assert(n<sizeof captured);memcpy(captured,s,n);captured[n]=0;captured_stamp=stamp?*stamp:now_utc;return 0;}
typedef struct{const uint8_t*payload;size_t payload_len;uint64_t timestamp;uint32_t ty;}SedsPacketView;
static uint8_t board_recovery_command_time_allowed(uint64_t t){return cutoff==0||t>cutoff;}
static uint8_t board_recovery_floor_active(void){return cutoff!=0;}
static int seds_router_get_network_time_ms(void*r,uint64_t*out){assert(r);*out=now_utc;return 0;}
/* Station-clock fixture: the freshness clock equals the network clock here. */
static uint64_t board_recovery_station_now(void*r){uint64_t t=0;return seds_router_get_network_time_ms(r,&t)?0U:t;}
static @@FRESH@@
static uint8_t board_recovery_manual_stop_source_allowed(void*r,const SedsPacketView*p){(void)r;(void)p;return 0;}
static UINT thread_comm_request_manual_abort_original_utc(uint64_t t){(void)t;thread_comm_latch_abort_locked(1);return 0;}
static UINT thread_comm_request_manual_abort(void){thread_comm_latch_abort_locked(1);return 0;}
static inline int telemetry_router_enter(void){return 1;}
static inline int telemetry_router_leave(void){return 1;}
@@ENUM@@
@@DIAGNOSTIC_HEADER@@
volatile safety_abort_diagnostic_t g_safety_first_abort;
typedef struct {uint64_t original_utc;uint32_t reason,epoch,lease;} thread_comm_safety_report_t;
static thread_comm_safety_report_t pending={.epoch=1};static uint32_t serial,active;
static uint8_t thread_comm_safety_report_claim(thread_comm_safety_report_t*out){if(!pending.reason||active)return 0;*out=pending;out->lease=++serial;active=out->lease;return 1;}
static void thread_comm_safety_report_finish(uint32_t lease,uint8_t retained){assert(active==lease);active=0;if(retained)pending.reason=0;}

@@EVENT_CLOCK@@
@@FUNC0@@
@@FUNC1@@
@@FUNC2@@
@@FUNC3@@
@@FUNC4@@
@@FUNC5@@
@@FUNC6@@
@@FUNC7@@

int main(int argc,char**argv){assert(argc==2);const char*test=argv[1];
 /* Snapshot contains a prior actual observation; no extrapolated UTC. */
 if(!strcmp(test,"clock")){
  assert(telemetry_event_clock_snapshot()==0);
  telemetry_event_clock_observe(9000,(uint32_t)now);assert(telemetry_event_clock_snapshot()==9000);
  now+=1000;assert(telemetry_event_clock_snapshot()==9000);++now;assert(telemetry_event_clock_snapshot()==0);
  telemetry_event_clock_observe(8999,(uint32_t)now);assert(telemetry_event_clock_snapshot()==0);
  telemetry_event_clock_observe(9000,(uint32_t)now);assert(telemetry_event_clock_snapshot()==9000);
  telemetry_event_clock_observe(0,(uint32_t)now);assert(telemetry_event_clock_snapshot()==0);
  telemetry_event_clock_observe(UINT64_MAX,(uint32_t)now);assert(telemetry_event_clock_snapshot()==0);
  now=UINT32_MAX-10U;telemetry_event_clock_observe(9100,(uint32_t)now);now=5;assert(telemetry_event_clock_snapshot()==9100);
  primask=1;assert(telemetry_event_clock_snapshot()==9100&&primask==1);
  telemetry_event_clock_observe(9200,(uint32_t)now);assert(primask==1&&telemetry_event_clock_snapshot()==9200);primask=0;
  puts("{\"clock_controls\":12,\"passed\":true}");return 0;
 }
 uint64_t origin=!strcmp(test,"fresh")||!strcmp(test,"launch")?11000:!strcmp(test,"equal")?10000:9000;
 telemetry_event_clock_observe(origin,(uint32_t)now);
 if(!strcmp(test,"unknown"))telemetry_event_clock_observe(0,(uint32_t)now);
 if(!strcmp(test,"expired"))now+=1001;
 if(!strcmp(test,"rollback"))telemetry_event_clock_observe(origin-1,(uint32_t)now);
 uint64_t captured_origin=telemetry_event_clock_snapshot();
 g_thread_comm_flight_state=2;assert(thread_comm_request_automatic_abort()==1&&g_thread_comm_abort);
 safety_record_first_abort(SAFETY_ABORT_HEARTBEAT,900,0,2,0,0,captured_origin);
 assert(g_safety_first_abort.original_network_utc==captured_origin);
 /* First-cause storage remains immutable, including its origin. */
 safety_record_first_abort(SAFETY_ABORT_ADC_START,999,0,2,0,0,UINT64_MAX);
 assert(g_safety_first_abort.original_network_utc==captured_origin);
 stops=0;g_thread_comm_abort=0;
 pending.reason=SAFETY_ABORT_HEARTBEAT;pending.original_utc=captured_origin;
 int retry=!strcmp(test,"retry");failures=retry?1:0;
 if(retry){now_utc=9000;telemetry_report_safety_abort();assert(attempts==1&&!captured[0]);now+=1000;}
 /* Retain this original packet before the modeled recovery boundary.
  * The real owner/SDK held-call boundary has separate two-thread tests. */
 telemetry_report_safety_abort();assert(captured[0]&&captured_stamp==captured_origin);
 unsigned before_recovery=attempts;pending.reason=0;
 cutoff=10000;now_utc=11000;
 telemetry_event_clock_observe(11000,(uint32_t)now);
 if(!strcmp(test,"launch"))g_thread_comm_flight_state=7;
 telemetry_report_safety_abort();assert(attempts==before_recovery);
 /* Already-retained packets are not recalled; receiver retirement still wins. */
 SedsPacketView p={.payload=(const uint8_t*)captured,.payload_len=strlen(captured),.timestamp=captured_stamp,.ty=SEDS_DT_ABORT};
 /* Other endpoint-fanout types cannot mutate the Abort owner or consume data. */
 const uint32_t unrelated[]={0U,7U,17U,256715704U,UINT32_MAX};
 for(unsigned i=0;i<sizeof unrelated/sizeof unrelated[0];++i){
  p.ty=unrelated[i];assert(Abort_handler(&p,0)==SEDS_OK);assert(stops==0&&!g_thread_comm_abort);
 }
 p.ty=SEDS_DT_ABORT;
 assert(Abort_handler(&p,0)==0);unsigned expected=!strcmp(test,"fresh");assert(stops==expected);
 p.payload=(const uint8_t*)"SEDS_MANUAL_ABORT_V1:";p.payload_len=21;assert(Abort_handler(&p,0)==0&&stops==expected+1);
 printf("{\"case\":\"%s\",\"frozen_origin\":%llu,\"new_operation_stops\":%u,\"manual_Stop_preserved\":true}\n",test,(unsigned long long)captured_stamp,expected);
 return 0;
}
