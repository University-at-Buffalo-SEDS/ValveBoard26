from pathlib import Path
import hashlib,json,subprocess,sys
import os
ROOT=Path(os.environ['OTA_OWNER_SOURCE']).resolve()
TASK=Path(os.environ['OTA_OWNER_WORK']).resolve()
sys.path.insert(0,str(ROOT/'tests'))
from native_sdk import configured_dependency_source
LC=configured_dependency_source(ROOT,'sedslaunchcore','SDK_FREE_LAUNCHCORE_SOURCE')
FIXTURE=Path(__file__).resolve().with_name('strict_write_fixture.c')

def function(text,name):
    import re
    m=re.search(r'^[^\n;]*\b'+re.escape(name)+r'\([^;]*?\)\s*\{',text,re.M)
    assert m,name
    start=m.start();brace=text.index('{',m.start());depth=1;i=brace+1
    while depth:
        depth+=(text[i]=='{')-(text[i]=='}');i+=1
    return text[start:i]

TX=r'''#pragma once
#include <stdint.h>
#include <pthread.h>
typedef unsigned UINT;typedef unsigned long ULONG;typedef void VOID;
typedef struct{pthread_mutex_t raw;int state,ring;}TX_MUTEX;
typedef struct{pthread_mutex_t raw;pthread_cond_t condition;ULONG count;int spaces;}TX_SEMAPHORE;
typedef struct{int unused;}TX_BYTE_POOL;
#define TX_NULL ((void*)0)
#define TX_SUCCESS 0U
#define TX_NO_INSTANCE 1U
#define TX_MUTEX_ERROR 2U
#define TX_QUEUE_ERROR 3U
#define TX_POOL_ERROR 4U
#define TX_NOT_AVAILABLE 5U
#define TX_WAIT_ABORTED 6U
#define TX_CALLER_ERROR 7U
#define TX_NO_WAIT 0U
#define TX_WAIT_FOREVER 0xffffffffUL
#define TX_INHERIT 1U
#define TX_TIMER_TICKS_PER_SECOND 1000U
unsigned long tx_time_get(void);void*tx_thread_identify(void);
UINT tx_mutex_create(TX_MUTEX*,const char*,UINT);UINT tx_mutex_get(TX_MUTEX*,ULONG);UINT tx_mutex_put(TX_MUTEX*);
UINT tx_semaphore_create(TX_SEMAPHORE*,const char*,ULONG);UINT tx_semaphore_get(TX_SEMAPHORE*,ULONG);UINT tx_semaphore_put(TX_SEMAPHORE*);
'''
HAL=r'''#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
typedef struct{volatile uint32_t ODR;}GPIO_TypeDef;
typedef struct{volatile uint32_t CR1,DIER,CCER,BDTR,SR,ARR,CCR[4];}TIM_TypeDef;
typedef struct{TIM_TypeDef*Instance;}TIM_HandleTypeDef;
typedef struct{uint32_t Pin,Mode,Pull,Speed,Alternate;}GPIO_InitTypeDef;
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define GPIO_MODE_OUTPUT_PP 0
#define GPIO_NOPULL 0
#define GPIO_SPEED_FREQ_HIGH 0
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 1U
#define TIM_CHANNEL_3 2U
#define TIM_DIER_UIE 1U
#define TIM_CR1_CEN 1U
#define TIM_CCER_CC3NE 1024U
#define TIM_BDTR_MOE 32768U
#define TIM_FLAG_UPDATE 1U
#define TIM_IT_UPDATE 1U
extern GPIO_TypeDef io;extern TIM_TypeDef timer1,timer2;extern TIM_HandleTypeDef htim1,htim2;
#define TIM1 (&timer1)
#define CLEAR_BIT(r,b) ((r)&=~(b))
#define __DMB() atomic_thread_fence(memory_order_seq_cst)
#define __HAL_TIM_CLEAR_FLAG(h,b) ((h)->Instance->SR&=~(b))
#define __HAL_TIM_DISABLE(h) ((h)->Instance->CR1&=~TIM_CR1_CEN)
#define __HAL_TIM_DISABLE_IT(h,b) ((h)->Instance->DIER&=~(b))
#define __HAL_TIM_GET_AUTORELOAD(h) ((h)->Instance->ARR)
#define __HAL_TIM_GET_COMPARE(h,c) ((h)->Instance->CCR[c])
void native_compare(TIM_HandleTypeDef*,uint32_t,uint32_t);
#define __HAL_TIM_SET_COMPARE(h,c,v) native_compare(h,c,v)
static inline uint32_t __get_IPSR(void){return 0;}
void HAL_GPIO_WritePin(GPIO_TypeDef*,uint16_t,int);int HAL_GPIO_ReadPin(GPIO_TypeDef*,uint16_t);void HAL_GPIO_Init(GPIO_TypeDef*,GPIO_InitTypeDef*);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef*,uint32_t);void HAL_Delay(uint32_t);
#define IGNITER_PIN_GPIO_Port (&io)
#define NITROGEN_PIN_GPIO_Port (&io)
#define N2_SIG_GPIO_Port (&io)
#define NITROUS_PIN_GPIO_Port (&io)
#define N20_SIG_GPIO_Port (&io)
#define BACKUP_VALVE_EN_GPIO_Port (&io)
#define N_FWD_EN_GPIO_Port (&io)
#define N_BCKWD_EN_GPIO_Port (&io)
#define P_FWD_EN_GPIO_Port (&io)
#define P_BCKWD_EN_GPIO_Port (&io)
#define Solenoid_GPIO_Port (&io)
#define IGNITER_PIN_Pin 1U
#define NITROGEN_PIN_Pin 2U
#define N2_SIG_Pin 4U
#define NITROUS_PIN_Pin 8U
#define N20_SIG_Pin 16U
#define BACKUP_VALVE_EN_Pin 32U
#define N_FWD_EN_Pin 64U
#define N_BCKWD_EN_Pin 128U
#define P_FWD_EN_Pin 256U
#define P_BCKWD_EN_Pin 512U
#define Solenoid_Pin 1U
'''
COMMON=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <pthread.h>
#include <errno.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "tx_api.h"
#include "main.h"
#include "thread_comm.h"
#include "board_commands.h"
#include "persistent_store.h"
#include "persistent_store_health.h"
#include "board_recovery_floor.h"
#include "bootstrap_marker.h"
#include "board_recovery.h"
#include "bootstrap_owner.h"
static _Atomic bool runtime;
static _Thread_local unsigned state_depth;
static bool check_gpio_owner;
static bool force_ring_busy;
#ifdef AB
static bool inject_timer_irq,native_in_irq;
void stepperTimerPeriodElapsedCallback(TIM_HandleTypeDef*);
#endif
GPIO_TypeDef io;TIM_TypeDef timer1,timer2;TIM_HandleTypeDef htim1={&timer1},htim2={&timer2};
unsigned long tx_time_get(void){return 1000;}
void*tx_thread_identify(void){return atomic_load(&runtime)?(void*)1:NULL;}
static pthread_mutex_t gate=PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t condition=PTHREAD_COND_INITIALIZER;
static bool io_block,io_entered,io_release,receiver_done;
static bool producer_waiting;
UINT tx_mutex_create(TX_MUTEX*m,const char*n,UINT i){(void)i;pthread_mutexattr_t a;assert(!pthread_mutexattr_init(&a));assert(!pthread_mutexattr_settype(&a,PTHREAD_MUTEX_RECURSIVE));assert(!pthread_mutex_init(&m->raw,&a));assert(!pthread_mutexattr_destroy(&a));m->state=!strcmp(n,"thread_comm_state_mutex");m->ring=!strcmp(n,"thread_comm_ring_mutex");return TX_SUCCESS;}
UINT tx_mutex_get(TX_MUTEX*m,ULONG wait){
#ifdef AB
assert(!native_in_irq);
#endif
if(m->ring&&force_ring_busy){force_ring_busy=false;return TX_NOT_AVAILABLE;}
int r=wait==TX_NO_WAIT?pthread_mutex_trylock(&m->raw):pthread_mutex_lock(&m->raw);if(r==EBUSY)return TX_NOT_AVAILABLE;assert(!r);if(m->state)state_depth++;return TX_SUCCESS;}
UINT tx_mutex_put(TX_MUTEX*m){if(m->state){assert(state_depth);state_depth--;}assert(!pthread_mutex_unlock(&m->raw));return TX_SUCCESS;}
UINT tx_semaphore_create(TX_SEMAPHORE*s,const char*n,ULONG c){assert(!pthread_mutex_init(&s->raw,NULL));assert(!pthread_cond_init(&s->condition,NULL));s->count=c;s->spaces=!strcmp(n,"thread_comm_spaces_sem");return 0;}
UINT tx_semaphore_get(TX_SEMAPHORE*s,ULONG w){assert(!pthread_mutex_lock(&s->raw));while(!s->count&&w!=TX_NO_WAIT){if(s->spaces){assert(!state_depth);assert(!pthread_mutex_lock(&gate));producer_waiting=true;assert(!pthread_cond_broadcast(&condition));assert(!pthread_mutex_unlock(&gate));}assert(!pthread_cond_wait(&s->condition,&s->raw));}UINT r=s->count?TX_SUCCESS:TX_NO_INSTANCE;if(s->count)s->count--;assert(!pthread_mutex_unlock(&s->raw));return r;}
UINT tx_semaphore_put(TX_SEMAPHORE*s){assert(!pthread_mutex_lock(&s->raw));s->count++;assert(!pthread_cond_broadcast(&s->condition));assert(!pthread_mutex_unlock(&s->raw));return 0;}
void HAL_GPIO_WritePin(GPIO_TypeDef*p,uint16_t pin,int value){if(check_gpio_owner)assert(state_depth);if(value)p->ODR|=pin;else p->ODR&=~pin;
#ifdef AB
if(inject_timer_irq&&pin==2048&&value==GPIO_PIN_RESET){inject_timer_irq=false;native_in_irq=true;stepperTimerPeriodElapsedCallback(&htim1);native_in_irq=false;}
#endif
}
int HAL_GPIO_ReadPin(GPIO_TypeDef*p,uint16_t pin){return(p->ODR&pin)?GPIO_PIN_SET:GPIO_PIN_RESET;}
void HAL_GPIO_Init(GPIO_TypeDef*p,GPIO_InitTypeDef*i){(void)p;(void)i;if(check_gpio_owner)assert(state_depth);}
void native_compare(TIM_HandleTypeDef*h,uint32_t c,uint32_t v){if(check_gpio_owner)assert(state_depth);h->Instance->CCR[c]=v;}
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef*h,uint32_t c){(void)c;h->Instance->CCER&=~TIM_CCER_CC3NE;return HAL_OK;}
void HAL_Delay(uint32_t t){(void)t;assert(!state_depth);}
static uint8_t abort_latch,launch_latch;
void board_abort_latch_init(void){}
void board_abort_latch_set(void){assert(state_depth);abort_latch=1;}
uint8_t board_abort_latch_prepare_recovery(void){abort_latch=1;return 1;}
uint8_t board_abort_latch_is_set(void){return abort_latch;}
void board_launch_latch_set(void){launch_latch=1;}
uint8_t board_launch_latch_set_verified(void){launch_latch=1;return 1;}
uint8_t board_launch_latch_is_set(void){return launch_latch;}
uint8_t board_launch_latch_was_verified(void){return launch_latch;}
uint8_t board_abort_latch_clear_for_recovery(void){abort_latch=launch_latch=0;return 1;}
typedef struct{int unused;}SedsRouter;
typedef struct{const uint8_t*payload;size_t payload_len;const uint8_t*sender;size_t sender_len;uint64_t timestamp;uint32_t ty;}SedsPacketView;
typedef int SedsResult;
#define SEDS_OK 0
#define SEDS_IO -1
#define SEDS_BAD_ARG -2
#define SEDS_HANDLER_ERROR -3
#define SEDS_PERMISSION_DENIED -6
static volatile uint32_t g_reliable_refusal_count;
#define SEDS_DT_FLIGHT_STATE 256715704U
#define SEDS_DT_ABORT 135612662U
#define FLIGHT_STATE_PERSIST_KEY 0x46535445U
#define FLIGHT_STATE_MAX_VALUE 15U
static SedsRouter router;static struct{SedsRouter*r;}g_router={&router};
static bool seds_router_sender_matches_hostname(SedsRouter*r,const uint8_t*s,size_t n,const char*h,size_t z){return r&&n==z&&!memcmp(s,h,z);}
static uint8_t board_recovery_automatic_is_stale(const SedsPacketView*p){(void)p;return 1;}
static uint32_t g_flight_state_handler_count,g_flight_state_handler_error_count,g_last_flight_state_packet;
static uint32_t g_flight_state_cache_value,g_flight_state_cache_updates,g_flight_state_cache_writes,g_flight_state_cache_errors,cache_invalidated;
static bool g_persist_ready=true,g_has_value,g_network_value_seen,g_persist_pending;
static uint8_t g_persisted_state=UINT8_MAX;
static uint32_t g_last_refresh_ms;
static uint32_t fixture_tick(void){return 0U;}
#define HAL_GetTick fixture_tick
static SedsRouter*g_cache_router=&router;
static SedsRouter*recovery_router=&router;
static SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){assert(r);*out=1000;return SEDS_OK;}
static __attribute__((unused)) uint64_t board_recovery_station_now(SedsRouter*r){uint64_t t=0;return seds_router_get_network_time_ms(r,&t)?0U:t;}
static SedsResult seds_router_invalidate_network_variable_cache(SedsRouter*r,uint32_t ty,const void*unused){(void)unused;assert(r&&ty==SEDS_DT_FLIGHT_STATE);cache_invalidated++;return SEDS_OK;}
static int32_t seds_pkt_get_u8(const SedsPacketView*p,uint8_t*out,size_t n){assert(n==1&&p->payload_len==1);*out=p->payload[0];return 1;}
AUTH_AND_HANDLER
FSTATE_HANDLERS
STRICT_FIXTURE

static launchcore_storage_status_t blocked_program(uint32_t a,const void*p,uint32_t n){
 assert(!state_depth);assert(!pthread_mutex_lock(&gate));
 if(io_block){io_entered=true;assert(!pthread_cond_broadcast(&condition));while(!io_release)assert(!pthread_cond_wait(&condition,&gate));}
 assert(!pthread_mutex_unlock(&gate));return program(a,p,n);
}
const launchcore_storage_driver_t launchcore_board_storage_driver={.init=init,.read=read_data,.write=blocked_program,.erase=erase_data,.layout=layout};
static void deadline_wait(bool*ready){struct timespec until;assert(!clock_gettime(CLOCK_REALTIME,&until));until.tv_sec+=3;while(!*ready){int r=pthread_cond_timedwait(&condition,&gate,&until);assert(r==0);}}
static void*writer(void*unused){(void)unused;uint64_t value=777;assert(persistent_store_set(0x4d414e55,&value,8)==LAUNCHCORE_PERSIST_OK);return NULL;}
static uint64_t stop_source=501;
#ifdef CANDIDATE
static UINT producer_result;
static void*old_local_producer(void*unused){(void)unused;thread_comm_msg_t command={0};producer_result=thread_comm_send(command,TX_WAIT_FOREVER);return NULL;}
#endif
static bool fstate_busy_case;
static void*receiver(void*unused){(void)unused;
 if(fstate_busy_case){uint8_t value=1;SedsPacketView f={.ty=SEDS_DT_FLIGHT_STATE,.payload=&value,.payload_len=1,.sender=(const uint8_t*)"GS",.sender_len=2,.timestamp=501};
  SedsResult status=persist_update(&f,NULL);
#ifdef CANDIDATE
  /* Applied and acknowledged while flash is busy; persistence stays pending. */
  assert(status==SEDS_OK&&cache_invalidated==0&&g_persisted_state==UINT8_MAX&&g_persist_pending&&g_has_value&&g_network_value_seen&&g_flight_state_cache_errors==0&&persistent_store_controls_ready());
#else
  assert(status==SEDS_OK);
#endif
 }
const uint8_t bytes[]="SEDS_MANUAL_ABORT_V1:K";SedsPacketView packet={.ty=SEDS_DT_ABORT,.payload=bytes,.payload_len=sizeof(bytes)-1,.sender=(const uint8_t*)"GS",.sender_len=2,.timestamp=stop_source};assert(Abort_handler(&packet,NULL)==SEDS_OK);assert(!pthread_mutex_lock(&gate));receiver_done=true;assert(!pthread_cond_broadcast(&condition));assert(!pthread_mutex_unlock(&gate));return NULL;}
BOARD_EFFECTS
#ifdef CANDIDATE
static uint8_t queue_recovery_read(uint32_t*mask){assert(state_depth);*mask=output_mask();return BOARD_RECOVERY_OUTPUTS_VALID|(*mask==STOP_MASK?BOARD_RECOVERY_SAFE_OUTPUTS:0U);}
static uint8_t queue_recovery_prepare(uint8_t desired){assert(state_depth&&desired<=1);manual_stop_apply_locked();return output_mask()==STOP_MASK;}
static void queue_recovery_finish(void){assert(state_depth);}
#endif
static void forbidden_reopen(void*unused){(void)unused;HAL_GPIO_WritePin(&io,1,GPIO_PIN_SET);}
int main(int argc,char**argv){
 assert(argc==2);assert(launchcore_storage_init()==LAUNCHCORE_STORAGE_OK);backing=launchcore_storage_driver();strict_seed(8);
 uint8_t floor[16];bootstrap_write64(floor,7);bootstrap_write64(floor+8,500);launchcore_persist_strict_info_t info;
 assert(launchcore_persist_set_strict(0x4252464c,floor,16,&info)==LAUNCHCORE_PERSIST_OK);
 BootstrapMarker marker={.state=0,.desired=0,.version=1,.kind=0,.first_epoch=1,.first_time=1,.first_nonce=9,.first_issuer=7,.first_expected_generation=UINT64_MAX};uint8_t wire[40];bootstrap_marker_encode(wire,&marker);assert(launchcore_persist_set_strict(BOOTSTRAP_MARKER_KEY,wire,32,&info)==LAUNCHCORE_PERSIST_OK);
 assert(persistent_store_boot_begin()==LAUNCHCORE_PERSIST_OK);assert(persistent_store_completed_boundary_ready());assert(board_recovery_floor_prime_boot());
 TX_BYTE_POOL pool={0};assert(thread_comm_init(&pool)==TX_SUCCESS);assert(thread_comm_publish_command_cutoff(500)==TX_SUCCESS);
 assert(persistent_store_runtime_start()==TX_SUCCESS);atomic_store(&runtime,true);
 setup_outputs();
#ifdef CANDIDATE
 assert(thread_comm_bind_manual_stop_apply(manual_stop_apply_locked)==TX_SUCCESS);
 board_recovery_bind_owner(queue_recovery_read,queue_recovery_prepare,queue_recovery_finish);
#endif
 check_gpio_owner=true;
#ifdef CANDIDATE
 if(!strcmp(argv[1],"fstate_retry")){
  pthread_t worker;assert(!pthread_mutex_lock(&gate));io_block=true;assert(!pthread_mutex_unlock(&gate));assert(!pthread_create(&worker,NULL,writer,NULL));assert(!pthread_mutex_lock(&gate));deadline_wait(&io_entered);assert(!pthread_mutex_unlock(&gate));
  uint8_t value=1;SedsPacketView f={.ty=SEDS_DT_FLIGHT_STATE,.payload=&value,.payload_len=1,.sender=(const uint8_t*)"GS",.sender_len=2,.timestamp=501};
  assert(persist_update(&f,NULL)==SEDS_OK&&g_persist_pending&&cache_invalidated==0&&g_persisted_state==UINT8_MAX&&g_flight_state_cache_errors==0&&persistent_store_controls_ready());
  assert(!pthread_mutex_lock(&gate));io_release=true;assert(!pthread_cond_broadcast(&condition));assert(!pthread_mutex_unlock(&gate));assert(!pthread_join(worker,NULL));
  persist_pending(); /* the poll's once-per-second durability retry */
  assert(!g_persist_pending&&g_persisted_state==1&&cache_invalidated==0&&g_flight_state_cache_errors==0&&persistent_store_controls_ready());
  uint8_t saved=255;size_t length=1;assert(persistent_store_get(0x46535445,&saved,&length)==LAUNCHCORE_PERSIST_OK&&saved==1);
  assert(thread_comm_request_manual_abort_original_utc(501)==TX_SUCCESS&&output_mask()==STOP_MASK);
  puts("GREEN: FSTATE acknowledged during Busy, the poll retry persists the exact value without storage fault; later Stop applies GPIO");return 0;
 }
 uint32_t dequeued_epoch=0;pthread_t old_producer;bool waiting_producer=false;
 if(!strncmp(argv[1],"queue",5)){
  thread_comm_msg_t local={0};
  if(!strcmp(argv[1],"queue_wait")){
   dequeued_epoch=thread_comm_manual_epoch();for(unsigned n=0;n<THREAD_COMM_QUEUE_DEPTH;n++)assert(thread_comm_send(local,TX_NO_WAIT)==TX_SUCCESS);
   waiting_producer=true;assert(!pthread_create(&old_producer,NULL,old_local_producer,NULL));assert(!pthread_mutex_lock(&gate));deadline_wait(&producer_waiting);assert(!pthread_mutex_unlock(&gate));
  }else{
   for(unsigned n=0;n<3;n++)assert(thread_comm_send(local,TX_NO_WAIT)==TX_SUCCESS);
   assert(thread_comm_receive_owned(&local,&dequeued_epoch)==TX_SUCCESS&&dequeued_epoch==1);
  }
 }
#endif
 if(!strcmp(argv[1],"old_stop"))stop_source=499;
 if(!strcmp(argv[1],"fstate_busy"))fstate_busy_case=true;
 if(!strcmp(argv[1],"launch")){assert(thread_comm_set_flight_state(7)==TX_SUCCESS);assert(board_launch_latch_was_verified());}
#ifdef AB
 if(!strcmp(argv[1],"irq"))inject_timer_irq=true;
#endif
 uint32_t before=output_mask();
 pthread_t worker,rx;assert(!pthread_mutex_lock(&gate));io_block=true;assert(!pthread_mutex_unlock(&gate));assert(!pthread_create(&worker,NULL,writer,NULL));assert(!pthread_mutex_lock(&gate));deadline_wait(&io_entered);assert(!pthread_mutex_unlock(&gate));
 assert(!pthread_create(&rx,NULL,receiver,NULL));assert(!pthread_mutex_lock(&gate));
#ifndef CANDIDATE
 if(fstate_busy_case){struct timespec until;assert(!clock_gettime(CLOCK_REALTIME,&until));until.tv_nsec+=100000000;if(until.tv_nsec>=1000000000){until.tv_sec++;until.tv_nsec-=1000000000;}int r=0;while(!receiver_done&&r==0)r=pthread_cond_timedwait(&condition,&gate,&until);assert(r==ETIMEDOUT&&!receiver_done&&!thread_comm_get_abort());puts("OLD RED: FSTATE persistence blocks receiver before following ManualStop parse");io_release=true;assert(!pthread_cond_broadcast(&condition));}
#endif
 deadline_wait(&receiver_done);assert(!pthread_mutex_unlock(&gate));assert(!pthread_join(rx,NULL));
 if(!strcmp(argv[1],"old_stop")){assert(output_mask()==before&&!thread_comm_get_abort());puts("GREEN: actual completed F retires older Stop without GPIO effects");}
 else{
#ifdef CANDIDATE
 assert(output_mask()==STOP_MASK);assert(thread_comm_get_abort());assert(!thread_comm_manual_effect(501,forbidden_reopen,NULL));assert(!thread_comm_manual_effect(0,forbidden_reopen,NULL));
 uint8_t command=MANUAL_OPEN_COMMAND;uint32_t old_epoch=thread_comm_manual_epoch()-1U;
 assert(!thread_comm_manual_effect_owned(0,old_epoch,apply_manual_valve_locked,&command));
 assert(!thread_comm_manual_effect_owned(0,old_epoch,service_launch_sequence_locked,&command));
 assert(output_mask()==STOP_MASK);puts("NEW GREEN: actual raw GPIO Stop while physical NVM still blocked; later ordinary/local effects denied");
#else
 assert(output_mask()==before&&output_mask()!=STOP_MASK);assert(thread_comm_get_abort());puts("OLD RED: validated ManualStop latched but GPIO not applied while main NVM blocked");
#endif
 }
 assert(!pthread_mutex_lock(&gate));io_release=true;assert(!pthread_cond_broadcast(&condition));assert(!pthread_mutex_unlock(&gate));assert(!pthread_join(worker,NULL));
#ifdef CANDIDATE
 if(!strncmp(argv[1],"queue",5)){
  BoardRecoverySnapshot snapshot;assert(board_recovery_snapshot(&snapshot)==0);uint32_t stopped_epoch=snapshot.abort_epoch;
  assert(board_recovery_begin_prepare(1,stopped_epoch)==0);
  assert(board_recovery_apply_safe(1,stopped_epoch,&snapshot)==-14);
  if(!strcmp(argv[1],"queue_busy"))force_ring_busy=true;
  board_recovery_execute_owned();
  if(!strcmp(argv[1],"queue_busy")){assert(thread_comm_get_abort());assert(board_recovery_snapshot(&snapshot)==0&&snapshot.abort_epoch==stopped_epoch);board_recovery_execute_owned();}
  assert(board_recovery_apply_safe(1,stopped_epoch,&snapshot)==0&&!thread_comm_get_abort());
  if(waiting_producer){assert(!pthread_join(old_producer,NULL));assert(producer_result==THREAD_COMM_STALE_TIMESTAMP||producer_result==TX_MUTEX_ERROR);}
  thread_comm_msg_t local={0};uint32_t new_epoch=0;
  assert(thread_comm_receive_owned(&local,&new_epoch)==TX_NO_INSTANCE);
  assert(!thread_comm_manual_effect_owned(0,dequeued_epoch,forbidden_reopen,NULL));uint8_t old_command=MANUAL_OPEN_COMMAND;assert(!thread_comm_manual_effect_owned(0,dequeued_epoch,apply_manual_valve_locked,&old_command));
  assert(output_mask()==STOP_MASK);
  assert(thread_comm_send(local,TX_NO_WAIT)==TX_SUCCESS);
  assert(thread_comm_receive_owned(&local,&new_epoch)==TX_SUCCESS&&new_epoch==snapshot.abort_epoch);
  uint8_t new_command=MANUAL_OPEN_COMMAND;assert(thread_comm_manual_effect_owned(0,new_epoch,apply_manual_valve_locked,&new_command));
  puts("GREEN: actual recovery drains old queued local0, rejects old dequeued epoch, accepts new epoch; Busy is not Empty");
 }
#endif
 return 0;
}
'''

def build(board,candidate):
    assert candidate and board=="ValveBoard26"
    source=ROOT
    # Candidate raw helper is the new source under test; baseline does NOT bind it.
    main=(ROOT/'Core/Src'/('main_task.c' if board=='ActuatorBoard26' else 'main_thread.c')).read_text()
    out=TASK/'native'/board/('candidate' if candidate else 'baseline');out.mkdir(parents=True,exist_ok=True)
    (out/'tx_api.h').write_text(TX);(out/'main.h').write_text(HAL);(out/'stm32g4xx_hal.h').write_text('#include "main.h"\n')
    telemetry=(source/'Core/Src/telemetry.c').read_text();transport=(source/'Core/Src/board_recovery_transport.c').read_text()
    snippets=function(transport,'authorized_primary')+'\n'+function(transport,'board_recovery_manual_stop_source_allowed')+'\n'+function(telemetry,'Abort_handler')
    cache=(source/'Core/Src/flight_state_cache.c').read_text()
    refusal=['board_recovery_refused'] if 'board_recovery_refused(void)' in transport else []
    admit=['flight_state_admit'] if 'flight_state_admit(' in telemetry else []
    fstate='\n'.join(function(transport,n)for n in ['board_recovery_sync_command_cutoff','board_recovery_command_is_stale','board_recovery_managed_update_allowed',*refusal])+'\n'+'\n'.join(function(telemetry,n)for n in [*admit,'Flight_State_handler','flight_state_cache_apply_network_update'])+'\n'+function(cache,'clear_managed_cache')+'\n'+function(cache,'persist_pending')+'\n'+function(cache,'persist_update')
    if 'flight_state_cache_from_wire(' in cache:
        # Contract set (a) mapping: the header block, its counters and the
        # counting receiver, ahead of the handlers that call them.
        header=(source/'Core/Inc/flight_state_cache.h').read_text()
        contract=header[header.index('#define FLIGHT_STATE_NET_STARTUP'):header.index('static inline uint8_t flight_state_from_wire')]+function(header,'flight_state_from_wire')
        counters='static uint32_t g_flight_state_contract_violations,g_flight_state_contract_logged,g_flight_state_contract_first_raw;\n'
        fstate=contract+'\n'+counters+'static '+function(cache,'flight_state_cache_from_wire')+'\n'+fstate
    effects=''
    extra=[]
    if board=='ValveBoard26':
        effects='''#include "servo_driver.h"
#include "solenoid_driver.h"
static uint8_t g_launch_sequence_active,g_launch_sequence_actuator_started,g_launch_sequence_pilot_opened,g_pilot_valve_state,g_no_servo_open_state,g_nitrous_servo_open_state;
static ULONG g_launch_sequence_pilot_open_ticks;static uint32_t g_sim_pilot_valve_state;
solenoid_t pilot_solenoid={&io,1,&io,1,NULL,0,5};
'''+function(main,'manual_stop_apply_locked')+'''
static void setup_outputs(void){timer2.ARR=19999;no_servo_close();dump_servo_close();io.ODR=1;}
static uint32_t output_mask(void){uint32_t mask=io.ODR&1;if(timer2.CCR[NO_SERVO_CHANNEL]==servo_relative_degrees_to_us(NO_SERVO_OPEN_DEGREES,NO_SERVO_ZERO_DEGREES))mask|=2;if(timer2.CCR[DUMP_SERVO_CHANNEL]==servo_relative_degrees_to_us(DUMP_SERVO_OPEN_DEGREES,DUMP_SERVO_ZERO_DEGREES))mask|=4;return mask;}
#define STOP_MASK 6U
'''
        if candidate:
            macros='\n'.join(line for line in main.replace('\\\n',' ').splitlines()if line.startswith('#define LAUNCH_SEQUENCE_'))
            effects+='\n'+macros+'''
#define MANUAL_OPEN_COMMAND CMD_PILOT_OPEN
static uint32_t g_launch_sequence_service_count,g_launch_sequence_pilot_attempt_count,g_launch_sequence_finish_count;
static ULONG g_launch_sequence_start_ticks,g_launch_sequence_last_elapsed_ticks,g_launch_sequence_last_pilot_status_ticks;
static uint8_t g_launch_sequence_completed;
static int32_t g_sim_pilot_solenoid_result;
'''
            effects+='\n'.join(function(main,n)for n in ['ms_to_ticks','apply_automatic_valve','apply_manual_valve_locked','service_launch_sequence_locked'])+'\n'
        extra=[source/'Drivers/Servo/servo_driver.c',source/'Drivers/Solenoid/solenoid_driver.c']
        inc=[source/'Drivers/Servo',source/'Drivers/Solenoid']
    else:
        driver=(ROOT/'Drivers/stepper_stuff/stepper_driver.c').read_text()
        effects='''#include "stepper_driver.h"
#include "fill_motion.h"
#include "fill_motion_status.h"
#define STEPPER_PWM_CHANNEL TIM_CHANNEL_3
stepper_t stepper={.step_port=&io,.step_pin=1024,.sleep_port=&io,.sleep_pin=2048,.dir_port=&io,.dir_pin=4096,.reset_port=&io,.reset_pin=8192};
static volatile stepper_t*timer_stepper;
static volatile bool timer_move_active;
static volatile uint32_t timer_steps_remaining,timer_period_us;
'''+function(driver,'stepperTimerPeriodElapsedCallback')+'''
static void setup_outputs(void){io.ODR=4095;timer1.CR1=TIM_CR1_CEN;timer1.DIER=TIM_DIER_UIE;timer1.CCER=TIM_CCER_CC3NE;timer1.BDTR=TIM_BDTR_MOE;timer_stepper=&stepper;timer_move_active=true;timer_steps_remaining=1;timer_period_us=30;}
static uint32_t output_mask(void){return (io.ODR&4095)|timer1.CR1|timer1.DIER|timer1.CCER|timer1.BDTR;}
#define STOP_MASK 0U
'''
        if candidate:
            effects+='''static uint8_t g_launch_sequence_active,g_launch_sequence_igniter_started,g_launch_sequence_holdoff_active,g_plumbing_retract_active,g_igniter_on,g_nitrogen_open,g_nitrous_open;
static uint32_t g_plumbing_retract_steps_remaining;
static FillMotion g_fill_motion;static FillMotionProvenance g_fill_provenance;static bool g_fill_drive_fault;
'''
            effects+='\n'.join(function(driver,n)for n in ['configureStepPinAsGpio','stepperStopAsleepImmediate'])+'\n'
            effects+='\n'.join(function(main,n)for n in ['all_fill_outputs_off','fill_outputs_are_off','fill_stop_locked','manual_stop_apply_locked'])+'\n'
        if candidate:
            macros='\n'.join(line for line in main.replace('\\\n',' ').splitlines()if line.startswith('#define LAUNCH_SEQUENCE_'))
            effects+='\n'+macros+'''
#include "solenoid_driver.h"
#include "igniter_driver.h"
#define MANUAL_OPEN_COMMAND CMD_NITROGEN_OPEN
GPIO_TypeDef fault_io={.ODR=UINT32_MAX};
solenoid_t n2_solenoid={.en_port=&io,.en_pin=2,.sig_port=&io,.sig_pin=4,.fault_port=&fault_io,.fault_pin=1};
solenoid_t n20_solenoid={.en_port=&io,.en_pin=8,.sig_port=&io,.sig_pin=16,.fault_port=&fault_io,.fault_pin=1};
igniter_t igniter={.en_port=&io,.en_pin=1,.fault_port=&fault_io,.fault_pin=1};
static ULONG g_launch_sequence_igniter_start_ticks,g_launch_sequence_igniter_stop_ticks,g_launch_sequence_holdoff_start_ticks;
'''
            effects+='\n'.join(function(main,n)for n in ['ms_to_ticks','igniter_on','igniter_off','apply_automatic_valve','apply_manual_valve_locked','service_launch_sequence_locked'])+'\n'
        extra=[source/'Core/Src/fill_motion.c',source/'Core/Src/fill_motion_status.c']
        if candidate:extra += [source/'Drivers/sol_stuff/solenoid_driver.c',source/'Drivers/spark_stuff/igniter_driver.c']
        inc=[ROOT/'Drivers/stepper_stuff',source/'Drivers/sol_stuff',source/'Drivers/spark_stuff']
    if not candidate:
        effects=effects.replace(function(main,'manual_stop_apply_locked'),'')
        import re
        effects=re.sub(r'^static (?:uint8_t|ULONG) g_.*\n','',effects,flags=re.M)
    c=COMMON.replace('AUTH_AND_HANDLER',snippets).replace('STRICT_FIXTURE','#include "'+str(FIXTURE)+'"').replace('BOARD_EFFECTS',effects).replace('FSTATE_HANDLERS',fstate)
    if not candidate:c=c.replace('static void forbidden_reopen(void*unused){(void)unused;HAL_GPIO_WritePin(&io,1,GPIO_PIN_SET);}','')
    (out/'test.c').write_text(c)
    command=['cc','-std=c11','-D_POSIX_C_SOURCE=200809L','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-g','-O1','-pthread',*(['-DCANDIDATE']if candidate else[]),*(['-DAB']if board=='ActuatorBoard26'else[]),'-I'+str(out),'-I'+str(source/'Core/Inc'),'-I'+str(LC/'bootloader/include'),'-I'+str(LC/'tests/unit'),*['-I'+str(i) for i in inc],str(out/'test.c'),str(source/'Core/Src/thread_comm.c'),str(source/'Core/Src/persistent_store.c'),str(source/'Core/Src/board_recovery_floor.c'),str(source/'Core/Src/bootstrap_marker.c'),str(source/'Core/Src/board_recovery_wire.c'),*[str(i)for i in extra],str(LC/'bootloader/src/persist.c'),str(LC/'bootloader/src/storage.c'),str(LC/'bootloader/src/crc32.c'),'-o',str(out/'test')]
    subprocess.run(command,check=True)
    (out/'BUILD.json').write_text(json.dumps({'command':command,'actual_source_hashes':{str(i):hashlib.sha256(i.read_bytes()).hexdigest()for i in [source/'Core/Src/thread_comm.c',source/'Core/Src/persistent_store.c',source/'Core/Src/board_recovery_floor.c',source/'Core/Src/telemetry.c',ROOT/'Core/Src'/('main_task.c'if board=='ActuatorBoard26'else'main_thread.c')]}},indent=2)+'\n')

if __name__=='__main__':
    build(sys.argv[1],sys.argv[2]=='candidate')
