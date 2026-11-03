"""Actual thread state/queue recovery runs only through its control owner."""
from pathlib import Path
import subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class RecoveryOwnerTests(unittest.TestCase):
 def test_actual_owner_completion_abort_race_and_clear_failure(self):
  with tempfile.TemporaryDirectory() as directory:
   work=Path(directory)
   (work/'tx_api.h').write_text(r"""
#pragma once
#include <stdint.h>
typedef unsigned UINT;typedef unsigned long ULONG;typedef void VOID;
typedef struct{UINT locked;}TX_MUTEX;typedef struct{ULONG count;}TX_SEMAPHORE;typedef struct{int unused;}TX_BYTE_POOL;
#define TX_NULL ((void*)0)
#define TX_SUCCESS 0U
#define TX_NO_INSTANCE 1U
#define TX_MUTEX_ERROR 2U
#define TX_QUEUE_ERROR 3U
#define TX_POOL_ERROR 4U
#define TX_NO_WAIT 0U
#define TX_WAIT_FOREVER 0xffffffffUL
#define TX_INHERIT 1U
#define TX_TIMER_TICKS_PER_SECOND 1000U
unsigned long tx_time_get(void);
UINT tx_mutex_create(TX_MUTEX*,const char*,UINT);UINT tx_mutex_get(TX_MUTEX*,ULONG);UINT tx_mutex_put(TX_MUTEX*);
UINT tx_semaphore_create(TX_SEMAPHORE*,const char*,ULONG);UINT tx_semaphore_get(TX_SEMAPHORE*,ULONG);UINT tx_semaphore_put(TX_SEMAPHORE*);
""")
   code=r"""
#include <stdbool.h>
#include <stdint.h>
bool persistent_store_controls_ready(void){return true;}
bool persistent_store_completed_boundary_ready(void){return false;}
uint8_t board_recovery_manual_time_retired(uint64_t source){(void)source;return 0;}
/* Legacy-owner unit boundary: no commissioning is in flight in these tests.
 * Real BRPC integration is covered separately with the actual journal. */
bool persistent_store_bootstrap_pending(void){return false;}
bool persistent_store_bootstrap_metadata_ready(void){return true;}
void persistent_store_hold_bootstrap(void){}
bool persistent_store_publish_bootstrap_complete(void){return false;}

#include <stdint.h>
bool persistent_store_recovery_ready(void){return true;}
bool persistent_store_classify_absent_floor(uint64_t expected){(void)expected;return false;}
bool persistent_store_commit_absent_floor(const uint8_t value[16]){(void)value;return false;}

#include <assert.h>
#include <stdint.h>
#include "thread_comm.h"
#include "board_recovery.h"
#include "board_recovery_boot.h"
static uint8_t abort_latch,launch_latch,clear_fails,prepare_fails;
void board_abort_latch_init(void){}
void board_abort_latch_set(void){abort_latch=1;}
uint8_t board_abort_latch_prepare_recovery(void){abort_latch=1;return !prepare_fails;}
uint8_t board_abort_latch_is_set(void){return abort_latch;}
void board_launch_latch_set(void){launch_latch=1;}
uint8_t board_launch_latch_set_verified(void){board_launch_latch_set();return 1;}
uint8_t board_launch_latch_is_set(void){return launch_latch;}
uint8_t board_launch_latch_was_verified(void){return launch_latch;}

uint8_t board_abort_latch_clear_for_recovery(void){if(clear_fails)return 0;abort_latch=launch_latch=0;return 1;}
PRODUCTION
UINT tx_mutex_create(TX_MUTEX*m,const char*n,UINT i){(void)n;(void)i;m->locked=0;return 0;}
UINT tx_mutex_get(TX_MUTEX*m,ULONG w){(void)w;if(m->locked)return TX_MUTEX_ERROR;m->locked=1;return 0;}
UINT tx_mutex_put(TX_MUTEX*m){assert(m->locked);m->locked=0;return 0;}
UINT tx_semaphore_create(TX_SEMAPHORE*s,const char*n,ULONG c){(void)n;s->count=c;return 0;}
UINT tx_semaphore_get(TX_SEMAPHORE*s,ULONG w){(void)w;if(!s->count)return TX_NO_INSTANCE;s->count--;return 0;}
UINT tx_semaphore_put(TX_SEMAPHORE*s){s->count++;return 0;}
static unsigned prepared,finished;static uint8_t safe=1,stopped;
static uint8_t read_outputs(uint32_t*mask){assert(g_thread_comm_state_mutex.locked);*mask=safe?0:1;return BOARD_RECOVERY_OUTPUTS_VALID|(safe?BOARD_RECOVERY_SAFE_OUTPUTS:0)|(stopped?0xe0:0);}
static uint8_t prepare(uint8_t desired){assert(g_thread_comm_state_mutex.locked&&desired<=1);prepared++;return safe;}
static void finish(void){assert(g_thread_comm_state_mutex.locked);finished++;}
int main(void){
 TX_BYTE_POOL pool={0};BoardRecoverySnapshot status;
 assert(thread_comm_init(&pool)==0);board_recovery_bind_owner(read_outputs,prepare,finish);
 assert(board_recovery_snapshot(&status)==0&&status.abort_epoch==1);
 assert(board_recovery_begin_prepare(0,1)==0);thread_comm_service_board_recovery();assert(prepared==0&&finished==0);
 assert(thread_comm_set_abort(1)==0);assert(thread_comm_set_abort(1)==0);
 assert(board_recovery_snapshot(&status)==0&&status.abort_epoch==2&&(status.flags&BOARD_RECOVERY_ABORT_LATCHED));
 thread_comm_msg_t message={0};assert(thread_comm_send(message,TX_NO_WAIT)==0);
 /* Abort alone does not establish owner quiescence. Unknown callback bits
  * must never escape in wire flags, and every required condition is real. */
 assert(!board_recovery_boot_retry_is_safe());stopped=1;
 assert(board_recovery_boot_retry_is_safe());
 assert(board_recovery_snapshot(&status)==0&&status.flags<=0x1f);
 safe=0;assert(!board_recovery_boot_retry_is_safe());safe=1;
 g_thread_comm_state_mutex.locked=1;assert(!board_recovery_boot_retry_is_safe());g_thread_comm_state_mutex.locked=0;

 assert(board_recovery_begin_prepare(0,2)==0);assert(!board_recovery_boot_retry_is_safe());thread_comm_service_board_recovery();assert(prepared==0&&finished==0&&abort_latch);
 assert(thread_comm_apply_managed_flight_state(VALVE_FLIGHT_STATE_ABORTED)==TX_QUEUE_ERROR);
 assert(board_recovery_apply_safe(0,2,&status)==-14&&abort_latch&&prepared==0&&finished==0);
 assert(board_recovery_apply_safe(1,2,&status)==-10);
 board_recovery_execute_owned();assert(prepared==1&&finished==1&&!abort_latch);
 assert(board_recovery_apply_safe(0,2,&status)==0&&status.abort_epoch==3);
 assert(thread_comm_receive(&message,TX_NO_WAIT)!=TX_SUCCESS);
 assert(board_recovery_apply_safe(0,2,&status)==0&&finished==1);
 assert(thread_comm_set_abort(1)==0);assert(board_recovery_apply_safe(0,2,&status)==-10);
 assert(board_recovery_begin_prepare(0,4)==0);assert(board_recovery_apply_safe(0,4,&status)==-14);
 /* A new abort while the original latch is still set invalidates pending reset. */
 assert(thread_comm_set_abort(1)==0);board_recovery_execute_owned();
 assert(prepared==1&&finished==1&&abort_latch);assert(board_recovery_snapshot(&status)==0&&status.abort_epoch==5);
 clear_fails=1;assert(board_recovery_begin_prepare(0,5)==0);assert(board_recovery_apply_safe(0,5,&status)==-14);board_recovery_execute_owned();
 assert(prepared==2&&finished==1&&abort_latch);assert(board_recovery_snapshot(&status)==0&&status.abort_epoch==5);
 clear_fails=0;safe=0;assert(board_recovery_begin_prepare(1,5)==0);assert(board_recovery_apply_safe(1,5,&status)==-14);board_recovery_execute_owned();assert(finished==1&&abort_latch);
 safe=1;assert(board_recovery_begin_prepare(0,5)==0);assert(board_recovery_apply_safe(0,5,&status)==-14);board_recovery_execute_owned();assert(finished==2&&!abort_latch);
 assert(board_recovery_snapshot(&status)==0&&status.abort_epoch==6&&status.actual_state==0);
 assert(board_recovery_apply_safe(7,6,&status)==-10);assert(board_recovery_apply_safe(0,UINT32_MAX,&status)==-10);
 /* A newer manual Stop before PREPARING invalidates the old prepared ticket. */
 assert(thread_comm_request_manual_abort()==TX_SUCCESS);assert(board_recovery_snapshot(&status)==0&&status.abort_epoch==7);
 assert(thread_comm_request_manual_abort()==TX_SUCCESS);assert(board_recovery_snapshot(&status)==0&&status.abort_epoch==8);
 assert(board_recovery_begin_prepare(0,7)==-10);
 /* A manual Stop wins both preparation phase2 and executable owner phase1. */
 assert(board_recovery_begin_prepare(0,8)==0);assert(thread_comm_request_manual_abort()==TX_SUCCESS);
 board_recovery_execute_owned();assert(board_recovery_apply_safe(0,8,&status)==-10&&status.abort_epoch==9);
 assert(board_recovery_begin_prepare(0,9)==0);assert(board_recovery_apply_safe(0,9,&status)==-14);
 assert(thread_comm_request_manual_abort()==TX_SUCCESS);board_recovery_execute_owned();
 assert(board_recovery_snapshot(&status)==0&&status.abort_epoch==10&&(status.flags&BOARD_RECOVERY_ABORT_LATCHED));
 assert(thread_comm_set_abort(1)==TX_SUCCESS);assert(thread_comm_set_abort(1)==TX_SUCCESS);
 assert(thread_comm_request_automatic_abort()==0);assert(board_recovery_snapshot(&status)==0&&status.abort_epoch==10);
 g_thread_comm_recovery_epoch=UINT32_MAX;assert(thread_comm_request_manual_abort()==0);assert(board_recovery_snapshot(&status)==0&&status.abort_epoch==0&&!(status.flags&BOARD_RECOVERY_SUPPORTED));
 return 0;
}
""".replace('PRODUCTION',(ROOT/'Core/Src/thread_comm.c').read_text())
   (work/'test.c').write_text(code);exe=work/'test'
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(work),'-I'+str(ROOT/'Core/Inc'),str(work/'test.c'),'-o',str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
