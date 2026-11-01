"""Late producer return cannot enqueue an obsolete Abort after owner recovery.

Actual thread state/queue, receiver, safety producer and command consumer run
with an interleaving at the producer's already-completed Abort claim.
"""
from pathlib import Path
import ast,os,subprocess,tempfile,unittest
from test_launch_abort_policy import function
ROOT=Path(os.environ.get('VB_ABORT_OWNER_SOURCE',Path(__file__).resolve().parents[1]))
BASE=Path(__file__).resolve().parents[1]

class AbortOwnerNotificationTests(unittest.TestCase):
 def test_late_automatic_manual_and_typed_notifications(self):
  strings=[n.value for n in ast.walk(ast.parse((BASE/'tests/test_board_recovery_owner.py').read_text())) if isinstance(n,ast.Constant)and isinstance(n.value,str)]
  tx=next(s for s in strings if 'typedef unsigned UINT;'in s)
  code=next(s for s in strings if 'PRODUCTION'in s and 'int main(void)'in s)
  code=code[:code.index('int main(void)')].replace('PRODUCTION',(ROOT/'Core/Src/thread_comm.c').read_text())
  code+=r'''
#include <stddef.h>
#include <string.h>
#include "board_commands.h"
static uint8_t g_aborted,g_pilot_valve_state,g_nitrous_servo_open_state,g_no_servo_open_state;
static uint32_t g_sim_valve_commands_executed,g_sim_last_valve_command,g_sim_valve_commands_received,g_sim_valve_open_seen,g_sim_groundstation_confirmation_received;
static uint64_t g_sim_last_valve_command_ms;
#define TX_TIMER_TICKS_PER_SECOND 1000U
ULONG tx_time_get(void){return 100;}
static void apply_automatic_valve(uint8_t cmd){(void)cmd;}
static void pilot_valve_on(void){} static void pilot_valve_off(void){}
static void nitrous_valve_close(void){} static void nitrous_valve_open(void){}
static void normally_open_valve_close(void){} static void normally_open_valve_open(void){}
static void start_launch_sequence(void){}
static int publish_umbilical_status(unsigned cmd,unsigned value){(void)cmd;(void)value;return 0;}
ACTUAL_CONSUMER
static unsigned recoveries;
static void recover_then_launch(void){
 BoardRecoverySnapshot snapshot;assert(board_recovery_snapshot(&snapshot)==0);
 assert(snapshot.flags&BOARD_RECOVERY_ABORT_LATCHED);
 assert(board_recovery_begin_prepare(0,snapshot.abort_epoch)==0);
 assert(board_recovery_apply_safe(0,snapshot.abort_epoch,&snapshot)==-14);
 thread_comm_service_board_recovery();assert(!thread_comm_get_abort());
 assert(thread_comm_request_launch_sequence(10000)==TX_SUCCESS);
 assert(board_launch_latch_is_set());recoveries++;
}
static UINT paused_manual_abort(void){UINT result=thread_comm_request_manual_abort();assert(result==TX_SUCCESS);recover_then_launch();return result;}
typedef int SedsResult;
#define SEDS_OK 0
#define SEDS_BAD_ARG -1
#define SEDS_HANDLER_ERROR -3
typedef struct {const uint8_t*payload;size_t payload_len;uint64_t timestamp;} SedsPacketView;
static uint8_t board_recovery_command_is_stale(const SedsPacketView*p){(void)p;return 0;}
static uint8_t board_recovery_automatic_is_stale(const SedsPacketView*p){(void)p;return 0;}
static int32_t seds_pkt_get_u8(const SedsPacketView*p,uint8_t*out,size_t n){assert(n==1);*out=p->payload[0];return 1;}
static int telemetry_broadcast_abort(const char*reason){assert(!strcmp(reason,"Valve board abort command"));return 0;}
#define thread_comm_request_manual_abort paused_manual_abort
ACTUAL_MANUAL
ACTUAL_TYPED
#undef thread_comm_request_manual_abort
#define Continuity_GPIO_Port 0
#define Continuity_Pin 0
static unsigned HAL_GPIO_ReadPin(unsigned port,unsigned pin){(void)port;(void)pin;return 1;}
static uint64_t safety_now_ms(void){return 100;}
static void safety_record_first_abort(uint32_t reason,uint64_t failure,uint64_t heartbeat,uint8_t flight,uint32_t continuity,uint32_t before){(void)reason;(void)failure;(void)heartbeat;(void)flight;(void)continuity;(void)before;recover_then_launch();}
ACTUAL_SAFETY
int main(int argc,char**argv){
 assert(argc==2);TX_BYTE_POOL pool={0};assert(thread_comm_init(&pool)==0);
 board_recovery_bind_owner(read_outputs,prepare,finish);assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_IDLE)==TX_SUCCESS);
 const uint8_t command=CMD_ABORT;SedsPacketView packet={.payload=&command,.payload_len=1,.timestamp=100};
 if(!strcmp(argv[1],"automatic"))safety_request_abort(1,"first cause");
 else if(!strcmp(argv[1],"manual"))assert(Abort_handler(&packet,0)==SEDS_OK);
 else if(!strcmp(argv[1],"typed"))assert(Valve_Command_handler(&packet,0)==SEDS_OK);
 else assert(0);
 assert(recoveries==1&&finished==1);
 thread_comm_msg_t queued;while(thread_comm_receive(&queued,TX_NO_WAIT)==TX_SUCCESS)handle_command(queued);
 assert(!thread_comm_get_abort());
 /* A genuinely NEW manual Stop after Launch remains unconditional and does
  * not depend on ordinary queue capacity or an owner notification. */
 assert(thread_comm_request_manual_abort()==TX_SUCCESS);assert(thread_comm_get_abort());
 return 0;
}
'''
  control=(ROOT/'Core/Src/main_thread.c').read_text();telemetry=(ROOT/'Core/Src/telemetry.c').read_text();safety=(ROOT/'Core/Src/safety_thread.c').read_text()
  for key,value in [('ACTUAL_CONSUMER',function(control,'handle_command')),('ACTUAL_MANUAL',function(telemetry,'Abort_handler')),('ACTUAL_TYPED',function(telemetry,'Valve_Command_handler')),('ACTUAL_SAFETY',function(safety,'safety_request_abort'))]:code=code.replace(key,value)
  with tempfile.TemporaryDirectory(prefix='abort-owner-notification-')as tmp:
   p=Path(tmp);(p/'tx_api.h').write_text(tx);(p/'test.c').write_text(code);binary=p/'test'
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(p),'-I'+str(ROOT/'Core/Inc'),str(p/'test.c'),'-o',str(binary)],check=True)
   for case in ['automatic','manual','typed']:
    with self.subTest(case=case):subprocess.run([str(binary),case],check=True)
if __name__=='__main__':unittest.main()
