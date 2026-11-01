"""Execute the production backup latch, thread admission and Abort receiver.

Deterministic mutex-entry interleavings exercise both launch/automatic-abort
orders. This is host logic evidence, not a physical interrupt or backup-power test.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def function(source, name):
    at = source.index(name + '(')
    start = source.rfind('\n', 0, at) + 1
    # These public callbacks have return type on the same line.
    brace = source.index('{', at)
    end, depth = brace + 1, 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


class LaunchAbortPolicyTests(unittest.TestCase):
    def test_actual_launch_auto_manual_rollback_reset_and_admission_races(self):
        ab = ROOT.name == 'ActuatorBoard26'
        state = 'ACTUATOR' if ab else 'VALVE'
        latch = (ROOT / 'Core/Src/board_abort_latch.c').read_text()
        latch = '\n'.join(line for line in latch.splitlines() if not line.startswith('#include'))
        shared = (ROOT / 'Core/Src/thread_comm.c').read_text()
        receiver = function((ROOT / 'Core/Src/telemetry.c').read_text(), 'Abort_handler')
        command_receiver = function((ROOT / 'Core/Src/telemetry.c').read_text(), 'Valve_Command_handler')
        managed_receiver = function((ROOT / 'Core/Src/telemetry.c').read_text(), 'Flight_State_handler')
        control_source=(ROOT/'Core/Src'/('main_task.c' if ab else 'main_thread.c')).read_text()
        self.assertIn('thread_comm_apply_automatic(cmd, apply_automatic_valve)',control_source)
        apply_source=function(control_source,'apply_automatic_valve')
        code = r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include "thread_comm.h"
#include "board_commands.h"
#define RCC_APB1ENR1_PWREN 1U
#define RCC_APB1ENR1_RTCAPBEN 2U
#define PWR_CR1_DBP 4U
#define __DSB() ((void)0)
static struct {uint32_t APB1ENR1;} rcc;
static struct {uint32_t CR1;} pwr;
static struct {uint32_t BKP28R,BKP29R,BKP30R,BKP31R;} tamp;
#define RCC (&rcc)
#define PWR (&pwr)
#define TAMP (&tamp)
'''+latch+'\n'+shared+r'''
static void (*before_state_lock)(void);
static unsigned no_wait_sends, claimed_effects;
UINT tx_mutex_create(TX_MUTEX*m,const char*n,UINT i){(void)n;(void)i;m->locked=0;return TX_SUCCESS;}
UINT tx_mutex_get(TX_MUTEX*m,ULONG wait){
 (void)wait;
 if(m==&g_thread_comm_state_mutex && before_state_lock){void(*hook)(void)=before_state_lock;before_state_lock=0;hook();}
 if(m->locked)return TX_MUTEX_ERROR;
 m->locked=1;return TX_SUCCESS;
}
UINT tx_mutex_put(TX_MUTEX*m){assert(m->locked);m->locked=0;return TX_SUCCESS;}
UINT tx_semaphore_create(TX_SEMAPHORE*s,const char*n,ULONG c){(void)n;s->count=c;return TX_SUCCESS;}
UINT tx_semaphore_get(TX_SEMAPHORE*s,ULONG wait){if(wait==TX_NO_WAIT)no_wait_sends++;if(!s->count)return TX_NO_INSTANCE;s->count--;return TX_SUCCESS;}
UINT tx_semaphore_put(TX_SEMAPHORE*s){s->count++;return TX_SUCCESS;}
typedef int SedsResult;
#define SEDS_OK 0
#define SEDS_BAD_ARG -1
#define SEDS_ERR -2
#define SEDS_HANDLER_ERROR -3
typedef struct {const uint8_t*payload;size_t payload_len;uint64_t timestamp;} SedsPacketView;
static uint8_t board_recovery_command_is_stale(const SedsPacketView*p){(void)p;return 0;}
static uint8_t board_recovery_automatic_is_stale(const SedsPacketView*p){(void)p;return 0;}
static unsigned g_flight_state_handler_error_count,g_flight_state_handler_count,g_last_flight_state_packet;
static unsigned g_command_queue_drop_count,g_sim_valve_commands_received,g_sim_valve_open_seen,g_sim_groundstation_confirmation_received;
static int32_t seds_pkt_get_u8(const SedsPacketView*p,uint8_t*v,size_t n){assert(n==1);*v=p->payload[0];return 1;}
static bool is_replayed_igniter_sequence_command(const SedsPacketView*p,uint8_t c){(void)p;(void)c;return false;}
static int telemetry_broadcast_abort(const char*reason){(void)reason;return 0;}
'''+receiver+'\n'+command_receiver+'\n'+managed_receiver+r'''
CONTROL_STUBS
'''+apply_source+r'''
static void reset_power(void){
 memset(&tamp,0,sizeof tamp);latched=launch_latched=backup_ready=0;board_abort_latch_init();
 g_thread_comm_initialized=0;TX_BYTE_POOL pool={0};assert(thread_comm_init(&pool)==TX_SUCCESS);
 before_state_lock=0;no_wait_sends=claimed_effects=0;
}
static void reboot_cpu(void){
 /* Backup registers persist while ordinary software/ThreadX state restarts. */
 latched=launch_latched=backup_ready=0;board_abort_latch_init();
 g_thread_comm_initialized=0;TX_BYTE_POOL pool={0};assert(thread_comm_init(&pool)==TX_SUCCESS);
}
static void try_automatic_effect(void){if(thread_comm_request_automatic_abort())claimed_effects++;}
static void launch_before_claim(void){assert(thread_comm_set_flight_state(STATE_LAUNCH)==TX_SUCCESS);}
static UINT launch_request(void){LAUNCH_REQUEST}
int main(void){
 for(unsigned phase=0;phase<256;phase++){
  reset_power();assert(thread_comm_set_flight_state((uint8_t)phase)==TX_SUCCESS);
  const unsigned allowed=phase>STATE_STARTUP&&phase<STATE_LAUNCH;
  try_automatic_effect();assert(claimed_effects==allowed);assert(thread_comm_get_abort()==allowed);
 }
 reset_power();assert(thread_comm_set_flight_state(STATE_LAUNCH)==TX_SUCCESS);
 for(unsigned i=0;i<4;i++){
  assert(thread_comm_set_flight_state(i%2?STATE_ARMED:STATE_IDLE)==TX_SUCCESS);
  try_automatic_effect();assert(!claimed_effects&&!thread_comm_get_abort());
  reboot_cpu();assert(board_launch_latch_is_set());
 }
 /* A managed global Aborted value has no manual emergency provenance. */
 reset_power();assert(thread_comm_set_flight_state(STATE_LAUNCH)==TX_SUCCESS);
 uint8_t managed_state=STATE_ABORTED;
 SedsPacketView managed_pkt={.payload=&managed_state,.payload_len=1};
 assert(Flight_State_handler(&managed_pkt,0)!=SEDS_OK);
 assert(thread_comm_get_flight_state()==STATE_LAUNCH&&!thread_comm_get_abort());
 /* Launch winning the state mutex must suppress the same managed update. */
 reset_power();assert(thread_comm_set_flight_state(STATE_ARMED)==TX_SUCCESS);
 before_state_lock=launch_before_claim;
 assert(Flight_State_handler(&managed_pkt,0)!=SEDS_OK);
 assert(thread_comm_get_flight_state()==STATE_LAUNCH&&!thread_comm_get_abort());
 /* Exact automatic marker is suppressed after launch, including delayed retry. */
 const uint8_t automatic[]="SEDS_AUTO_ABORT_V1:late heartbeat";
 SedsPacketView pkt={.payload=automatic,.payload_len=sizeof automatic-1};assert(Abort_handler(&pkt,0)==SEDS_OK);
 assert(!thread_comm_get_abort());
 const uint8_t manual[]="operator abort";pkt.payload=manual;pkt.payload_len=sizeof manual-1;
 assert(Abort_handler(&pkt,0)==SEDS_OK&&thread_comm_get_abort());
 reboot_cpu();assert(board_abort_latch_is_set()&&board_launch_latch_is_set());
 /* Torn retained launch marker inhibits automatic abort just like full marker. */
 reset_power();tamp.BKP28R=1;reboot_cpu();assert(board_launch_latch_is_set());
 assert(thread_comm_set_flight_state(STATE_ARMED)==TX_SUCCESS);try_automatic_effect();assert(!claimed_effects);
 reset_power();assert(thread_comm_set_flight_state(STATE_ARMED)==TX_SUCCESS);
 before_state_lock=launch_before_claim;try_automatic_effect();assert(!claimed_effects&&!thread_comm_get_abort());
 /* Reverse order: actual auto claim takes the mutex first, then launch refuses. */
 reset_power();assert(thread_comm_set_flight_state(STATE_ARMED)==TX_SUCCESS);
 try_automatic_effect();assert(claimed_effects==1&&thread_comm_get_abort());
 assert(launch_request()!=TX_SUCCESS);
 reset_power();assert(thread_comm_set_flight_state(STATE_ARMED)==TX_SUCCESS);
 assert(launch_request()==TX_SUCCESS&&board_launch_latch_is_set());
 assert(thread_comm_set_flight_state(STATE_IDLE)==TX_SUCCESS);try_automatic_effect();assert(!claimed_effects);
 EXTRA
 reset_power();assert(thread_comm_set_flight_state(STATE_ARMED)==TX_SUCCESS);
 uint8_t marked[1]={AUTO_CMD|THREAD_COMM_AUTOMATIC_TAG};
 pkt=(SedsPacketView){.payload=marked,.payload_len=1,.timestamp=1234};
 assert(Valve_Command_handler(&pkt,0)==SEDS_OK);
 thread_comm_msg_t received;assert(thread_comm_receive(&received,TX_NO_WAIT)==TX_SUCCESS);CHECK_RECEIVED
 assert(thread_comm_set_flight_state(STATE_LAUNCH)==TX_SUCCESS);
 assert(Valve_Command_handler(&pkt,0)==SEDS_OK);
 assert(thread_comm_receive(&received,TX_NO_WAIT)!=TX_SUCCESS); /* stale auto no effect */
 marked[0]=AUTO_CMD;assert(Valve_Command_handler(&pkt,0)==SEDS_OK);
 assert(thread_comm_receive(&received,TX_NO_WAIT)==TX_SUCCESS);CHECK_RECEIVED
 pkt.payload_len=2;assert(Valve_Command_handler(&pkt,0)==SEDS_BAD_ARG);
 pkt.payload_len=1;marked[0]=BAD_AUTO|THREAD_COMM_AUTOMATIC_TAG;assert(Valve_Command_handler(&pkt,0)==SEDS_BAD_ARG);
 reset_power();assert(thread_comm_set_flight_state(STATE_ARMED)==TX_SUCCESS);
 marked[0]=AUTO_CMD|THREAD_COMM_AUTOMATIC_TAG;before_state_lock=launch_before_claim;
 assert(Valve_Command_handler(&pkt,0)==SEDS_OK);
 assert(thread_comm_receive(&received,TX_NO_WAIT)!=TX_SUCCESS); /* launch wins lock */
 reset_power();assert(thread_comm_set_flight_state(STATE_ARMED)==TX_SUCCESS);
 marked[0]=AUTO_CMD|THREAD_COMM_AUTOMATIC_TAG;assert(Valve_Command_handler(&pkt,0)==SEDS_OK);
 assert(thread_comm_receive(&received,TX_NO_WAIT)==TX_SUCCESS);
 assert(thread_comm_set_flight_state(STATE_LAUNCH)==TX_SUCCESS);
 physical_calls=0;assert(!thread_comm_apply_automatic(received,apply_automatic_valve));assert(!physical_calls);
 /* Same retained tag applies before launch; real callback only runs locked. */
 reset_power();assert(thread_comm_set_flight_state(STATE_ARMED)==TX_SUCCESS);
 assert(Valve_Command_handler(&pkt,0)==SEDS_OK);assert(thread_comm_receive(&received,TX_NO_WAIT)==TX_SUCCESS);
 assert(thread_comm_apply_automatic(received,apply_automatic_valve));assert(physical_calls);
 assert(!g_thread_comm_state_mutex.locked);assert(launch_request()==TX_SUCCESS);
 for(unsigned wire=128;wire<256;wire++){
  reset_power();assert(thread_comm_set_flight_state(STATE_ARMED)==TX_SUCCESS);
  marked[0]=(uint8_t)wire;pkt.payload_len=1;
  const unsigned raw=wire&127U;
  const unsigned supported=AUTO_ALLOWED;
  const int result=Valve_Command_handler(&pkt,0);
  assert(result==(supported?SEDS_OK:SEDS_BAD_ARG));
  if(supported){
   assert(thread_comm_receive(&received,TX_NO_WAIT)==TX_SUCCESS);
   physical_calls=0;assert(thread_comm_apply_automatic(received,apply_automatic_valve)&&physical_calls);
   assert(thread_comm_set_flight_state(STATE_LAUNCH)==TX_SUCCESS);
   physical_calls=0;assert(Valve_Command_handler(&pkt,0)==SEDS_OK);
   assert(thread_comm_receive(&received,TX_NO_WAIT)!=TX_SUCCESS&&!physical_calls);
  }
 }
 return 0;
}
'''
        code = re.sub(r'\bSTATE_',state+'_FLIGHT_STATE_',code)
        request = ('thread_comm_msg_t msg={.cmd=1,.timestamp_ms=42};return thread_comm_admit_launch(msg);'
                   if ab else 'return thread_comm_request_launch_sequence(42);')
        extra = r'''
 reset_power();assert(thread_comm_set_flight_state(ACTUATOR_FLIGHT_STATE_ARMED)==TX_SUCCESS);
 /* Full command queue does not clear an already admitted launch intention. */
 for(unsigned i=0;i<THREAD_COMM_QUEUE_DEPTH;i++){thread_comm_msg_t msg={.cmd=1,.timestamp_ms=i};assert(thread_comm_send(msg,TX_NO_WAIT)==TX_SUCCESS);}
 assert(launch_request()!=TX_SUCCESS&&board_launch_latch_is_set());
 try_automatic_effect();assert(!claimed_effects&&!thread_comm_get_abort());
''' if ab else r'''
 uint64_t timestamp=0;assert(thread_comm_take_launch_sequence_request(&timestamp)&&timestamp==42);
 assert(!thread_comm_take_launch_sequence_request(&timestamp));
'''
        code=code.replace('LAUNCH_REQUEST',request).replace('EXTRA',extra)
        code=code.replace('AUTO_ALLOWED','raw==9U||raw==10U||raw==12U||raw==13U' if ab else 'raw<=5U')
        code=code.replace('AUTO_CMD','CMD_NITROGEN_CLOSE' if ab else 'CMD_PILOT_CLOSE').replace('BAD_AUTO','CMD_IGNITER_SEQUENCE' if ab else 'CMD_SEQUENCE')
        code=code.replace('CHECK_RECEIVED','assert((received.cmd & ~THREAD_COMM_AUTOMATIC_TAG)==CMD_NITROGEN_CLOSE && received.timestamp_ms==1234);' if ab else 'assert((received & ~THREAD_COMM_AUTOMATIC_TAG)==CMD_PILOT_CLOSE);')
        # Replacement happens after actual source insertion; word boundary only.
        code=re.sub(r'\bSTATE_',state+'_FLIGHT_STATE_',code)
        if ab:
            code=code.replace('static unsigned g_command_queue_drop_count,g_sim_valve_commands_received,g_sim_valve_open_seen,g_sim_groundstation_confirmation_received;','static unsigned g_command_queue_drop_count;')
            code=re.sub(r'static int telemetry_broadcast_abort[^\n]+\n','',code)
        else:
            code=code.replace('static unsigned g_command_queue_drop_count,g_sim_valve_commands_received,g_sim_valve_open_seen,g_sim_groundstation_confirmation_received;','static unsigned g_sim_valve_commands_received,g_sim_valve_open_seen,g_sim_groundstation_confirmation_received;')
            code=re.sub(r'static bool is_replayed_igniter_sequence_command[^\n]+\n','',code)
        stubs = r"""
static unsigned physical_calls;
static void effect(void){assert(g_thread_comm_state_mutex.locked);physical_calls++;}
static unsigned n2_solenoid,n20_solenoid,pilot_solenoid;
static uint8_t g_nitrogen_open,g_nitrous_open,g_pilot_valve_state,g_sim_pilot_valve_state,g_nitrous_servo_open_state,g_no_servo_open_state;
static int32_t g_sim_pilot_solenoid_result;
static int solenoidOn(unsigned*hw){(void)hw;effect();return 0;}
static void solenoidOff(unsigned*hw){(void)hw;effect();}
static void dump_servo_close(void){effect();}static void dump_servo_open(void){effect();}
static void no_servo_close(void){effect();}static void no_servo_open(void){effect();}
"""
        if ab:
            stubs=stubs.replace('static unsigned n2_solenoid,n20_solenoid,pilot_solenoid;','static unsigned n2_solenoid,n20_solenoid;').replace('static uint8_t g_nitrogen_open,g_nitrous_open,g_pilot_valve_state,g_sim_pilot_valve_state,g_nitrous_servo_open_state,g_no_servo_open_state;','static uint8_t g_nitrogen_open,g_nitrous_open;').replace('static int32_t g_sim_pilot_solenoid_result;','')
            stubs=re.sub(r'static void (?:dump|no)_servo_[^\n]+','',stubs)
        else:
            stubs=stubs.replace('static unsigned n2_solenoid,n20_solenoid,pilot_solenoid;','static unsigned pilot_solenoid;').replace('static uint8_t g_nitrogen_open,g_nitrous_open,g_pilot_valve_state,g_sim_pilot_valve_state,g_nitrous_servo_open_state,g_no_servo_open_state;','static uint8_t g_pilot_valve_state,g_sim_pilot_valve_state,g_nitrous_servo_open_state,g_no_servo_open_state;')
        code=code.replace('CONTROL_STUBS',stubs)
        with tempfile.TemporaryDirectory() as tmp:
            tmp=Path(tmp)
            (tmp/'tx_api.h').write_text('''#pragma once
#include <stdint.h>
typedef unsigned UINT; typedef unsigned long ULONG; typedef void VOID;
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
UINT tx_mutex_create(TX_MUTEX*,const char*,UINT);UINT tx_mutex_get(TX_MUTEX*,ULONG);UINT tx_mutex_put(TX_MUTEX*);
UINT tx_semaphore_create(TX_SEMAPHORE*,const char*,ULONG);UINT tx_semaphore_get(TX_SEMAPHORE*,ULONG);UINT tx_semaphore_put(TX_SEMAPHORE*);
''')
            src=tmp/'test.c';src.write_text(code);exe=tmp/'test'
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-O1','-fsanitize=address,undefined','-I',str(tmp),'-I',str(ROOT/'Core/Inc'),str(src),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)


if __name__ == '__main__': unittest.main()
