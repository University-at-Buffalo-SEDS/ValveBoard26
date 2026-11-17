"""Post-Launch auto-vent countdown on the production sources.

The decision module (vent_countdown.c) is exercised directly with a 1 ms
control loop. The control-owner service, the operator command path and the
valve effects are extracted from main_thread.c and run against the real
thread_comm.c and board_abort_latch.c with inert drivers, so the vent order,
the state-owner recheck, the backup-register marker and the reset hold are the
production code. Host logic evidence only: no MCU timing or hardware.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

from test_overpressure import TX_API, CFLAGS, function

ROOT = Path(__file__).resolve().parents[1]

MODULE_HARNESS = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "vent_countdown.h"
static vent_countdown_t v;
static uint64_t now, hb, cont;
static uint8_t launched, vented_at_set;
static uint64_t vented_at;
/* One control loop per ms; apply the vent at once like the owner. */
static void run_until(uint64_t end){
 for(;now<end;now++){
  if(vent_countdown_step(&v,now,launched,hb,cont)){
   assert(v.state==VENT_CD_COUNTING);vent_countdown_vented(&v,now);
   assert(!vented_at_set);vented_at=now;vented_at_set=1;
  }
 }
}
/* Heartbeats every 500 ms while the link is up. */
static void link_up_until(uint64_t end){for(;now<end;){hb=now?now:1;run_until(now+500U>end?end:now+500U);}}
static void live(uint64_t launch_at){
 memset(&v,0,sizeof v);now=0;hb=0;cont=0;launched=0;vented_at=0;vented_at_set=0;
 vent_countdown_boot(&v,0,0,0,0);
 link_up_until(launch_at);launched=1;
}

/* Link lost after Launch: counting from the last heartbeat, vent at +150 s. */
static void loss_vents(void){
 live(10000);link_up_until(100000);const uint64_t last=hb;
 run_until(last+VENT_LINK_LOSS_DETECT_MS-1U);assert(v.state==VENT_CD_IDLE);
 run_until(last+VENT_LINK_LOSS_DETECT_MS+1U);assert(v.state==VENT_CD_COUNTING&&v.triggers==VENT_CD_LINK);
 assert(vent_countdown_remaining_ms(&v,now)==VENT_AFTER_LINK_LOSS_MS-VENT_LINK_LOSS_DETECT_MS-1U);
 run_until(last+VENT_AFTER_LINK_LOSS_MS);assert(!vented_at_set);
 run_until(last+VENT_AFTER_LINK_LOSS_MS+1U);assert(vented_at_set&&vented_at==last+VENT_AFTER_LINK_LOSS_MS);
 assert(v.state==VENT_CD_VENTED);
 /* Latched: a returning link or continuity never re-closes or re-fires. */
 link_up_until(now+60000U);assert(v.state==VENT_CD_VENTED);
}

/* A heartbeat back before expiry cancels; a later loss starts afresh. */
static void return_cancels_then_restart(void){
 live(10000);link_up_until(50000);uint64_t last=hb;
 run_until(last+140000U);assert(v.state==VENT_CD_COUNTING);
 link_up_until(now+1000U);assert(v.state==VENT_CD_IDLE&&!vented_at_set);
 link_up_until(now+20000U);last=hb;
 run_until(last+VENT_AFTER_LINK_LOSS_MS);assert(!vented_at_set&&v.state==VENT_CD_COUNTING);
 run_until(last+VENT_AFTER_LINK_LOSS_MS+1U);assert(vented_at==last+VENT_AFTER_LINK_LOSS_MS);
}

/* Never a vent within Launch + VENT_BURN_GUARD_MS, whatever the triggers. */
static void burn_guard(void){
 /* Link lost long before Launch, continuity broken at Launch. */
 const uint64_t launch_at=200000U;
 live(0);launched=0;hb=1;run_until(launch_at);launched=1;cont=launch_at;
 run_until(launch_at+VENT_BURN_GUARD_MS);assert(!vented_at_set&&v.state==VENT_CD_COUNTING&&v.triggers==3U);
 run_until(launch_at+VENT_BURN_GUARD_MS+1U);assert(vented_at==launch_at+VENT_BURN_GUARD_MS);
 /* Sweep: every loss instant around Launch. */
 for(uint64_t loss=150000U;loss<=launch_at+20000U;loss+=1000U){
  live(0);launched=0;link_up_until(loss<launch_at?loss:launch_at);launched=1;
  if(loss>launch_at)link_up_until(loss);
  const uint64_t last=hb;run_until(launch_at+300000U);
  assert(vented_at_set&&vented_at>=launch_at+VENT_BURN_GUARD_MS);
  const uint64_t expect=last+VENT_AFTER_LINK_LOSS_MS>launch_at+VENT_BURN_GUARD_MS?last+VENT_AFTER_LINK_LOSS_MS:launch_at+VENT_BURN_GUARD_MS;
  assert(vented_at==expect);
 }
}

/* An operator valve command cancels; the operator has control. */
static void operator_cancels(void){
 live(10000);link_up_until(40000);
 run_until(150000U);assert(v.state==VENT_CD_COUNTING);
 vent_countdown_operator(&v,now);assert(v.state==VENT_CD_IDLE);
 const uint64_t cmd=now;
 /* Still no heartbeat: a fresh full countdown from the command. */
 run_until(cmd+VENT_LINK_LOSS_DETECT_MS-1U);assert(v.state==VENT_CD_IDLE);
 run_until(cmd+VENT_AFTER_LINK_LOSS_MS);assert(!vented_at_set&&v.state==VENT_CD_COUNTING);
 run_until(cmd+VENT_AFTER_LINK_LOSS_MS+1U);assert(vented_at==cmd+VENT_AFTER_LINK_LOSS_MS);
 /* After a vent, an operator command (e.g. closing Vent) returns to idle. */
 vent_countdown_operator(&v,now);assert(v.state==VENT_CD_IDLE);
}

/* Continuity broken with the heartbeat present still counts down. */
static void continuity_only(void){
 live(10000);link_up_until(70000);cont=now;const uint64_t brk=cont;
 link_up_until(brk+1U);assert(v.state==VENT_CD_COUNTING&&v.triggers==VENT_CD_CONTINUITY);
 link_up_until(brk+VENT_AFTER_LINK_LOSS_MS);assert(!vented_at_set);
 link_up_until(brk+VENT_AFTER_LINK_LOSS_MS+1U);assert(vented_at==brk+VENT_AFTER_LINK_LOSS_MS);
 /* Restored continuity with the heartbeat present cancels. */
 live(10000);link_up_until(70000);cont=now;link_up_until(now+100000U);assert(v.state==VENT_CD_COUNTING);
 cont=0;link_up_until(now+1000U);assert(v.state==VENT_CD_IDLE);link_up_until(now+300000U);assert(!vented_at_set);
}

/* Both triggers; the earlier one sets the deadline, which then stays fixed. */
static void both_triggers(void){
 live(10000);link_up_until(80000);cont=now;const uint64_t brk=cont;
 link_up_until(brk+20000U);const uint64_t last=hb;run_until(last+VENT_LINK_LOSS_DETECT_MS+1U);
 assert(v.state==VENT_CD_COUNTING&&v.triggers==3U);
 char text[VENT_REPORT_MAX];vent_countdown_format_report(&v,now,text,sizeof text);
 assert(strstr(text,"link lost, continuity broken [B"));
 /* Heartbeat back but continuity still broken: keeps counting, same deadline. */
 const uint64_t deadline=v.deadline_ms;link_up_until(now+10000U);
 assert(v.state==VENT_CD_COUNTING&&v.triggers==VENT_CD_CONTINUITY&&v.deadline_ms==deadline);
 link_up_until(brk+VENT_AFTER_LINK_LOSS_MS+1U);assert(vented_at==brk+VENT_AFTER_LINK_LOSS_MS);
}

/* Continuity restored while the link is still lost: keeps counting. */
static void continuity_restored_link_lost(void){
 live(10000);link_up_until(60000);const uint64_t last=hb;
 run_until(last+VENT_LINK_LOSS_DETECT_MS+1U);assert(v.triggers==VENT_CD_LINK);
 cont=now;run_until(100000U);assert(v.state==VENT_CD_COUNTING&&v.triggers==3U);
 const uint64_t deadline=v.deadline_ms;assert(deadline==last+VENT_AFTER_LINK_LOSS_MS);
 cont=0;run_until(120000U);assert(v.state==VENT_CD_COUNTING&&v.triggers==VENT_CD_LINK&&v.deadline_ms==deadline);
 run_until(deadline+1U);assert(vented_at==deadline);
}

/* Reset after Launch: no link at boot vents 150 s after boot. */
static void reboot_no_link(void){
 for(uint32_t elapsed=0;elapsed<=VENT_GUARD_S+5U;elapsed+=5U){
  memset(&v,0,sizeof v);now=0;hb=0;cont=0;launched=1;vented_at_set=0;
  vent_countdown_boot(&v,0,1,elapsed,0);
  run_until(VENT_LINK_LOSS_DETECT_MS);assert(v.state==VENT_CD_IDLE);
  run_until(VENT_LINK_LOSS_DETECT_MS+1U);assert(v.state==VENT_CD_COUNTING);
  run_until(VENT_AFTER_LINK_LOSS_MS);assert(!vented_at_set);
  run_until(VENT_AFTER_LINK_LOSS_MS+1U);assert(vented_at==VENT_AFTER_LINK_LOSS_MS);
  assert(vent_countdown_elapsed_s(&v,now)==VENT_GUARD_S);
 }
 /* The persisted elapsed time continues the guard across the reset. */
 memset(&v,0,sizeof v);vent_countdown_boot(&v,1000,1,20,0);
 assert(v.guard_ms==1000U+(VENT_GUARD_S-20U)*1000U&&vent_countdown_elapsed_s(&v,3500)==22U);
 vent_countdown_boot(&v,1000,1,0xFFFFU,0);assert(v.guard_ms==1000U&&vent_countdown_elapsed_s(&v,1000)==VENT_GUARD_S);
 /* Continuity broken at boot, heartbeat present: vents 150 s after boot. */
 memset(&v,0,sizeof v);now=0;hb=0;cont=1;launched=1;vented_at_set=0;vent_countdown_boot(&v,0,1,0,0);
 link_up_until(VENT_AFTER_LINK_LOSS_MS+1U);assert(!vented_at_set);link_up_until(VENT_AFTER_LINK_LOSS_MS+2U);assert(vented_at==VENT_AFTER_LINK_LOSS_MS+1U);
 /* A vented board boots vented and owes an immediate report. */
 vent_countdown_boot(&v,0,1,VENT_GUARD_S,1);assert(v.state==VENT_CD_VENTED&&vent_countdown_report_due(&v,0));
 assert(!vent_countdown_step(&v,1,1,0,1));
}

/* Reset after Launch with the link present: holds closed indefinitely. */
static void reboot_link_present(void){
 memset(&v,0,sizeof v);now=0;hb=0;cont=0;launched=1;vented_at_set=0;
 vent_countdown_boot(&v,0,1,0,0);
 run_until(2000U);link_up_until(3600000U);assert(!vented_at_set&&v.state==VENT_CD_IDLE);
}

/* Before Launch nothing ever counts or vents (pre-Launch abort unchanged). */
static void no_vent_before_launch(void){
 memset(&v,0,sizeof v);now=0;hb=0;cont=1;launched=0;vented_at_set=0;
 vent_countdown_boot(&v,0,0,0,0);run_until(1000000U);assert(!vented_at_set&&v.state==VENT_CD_IDLE);
 assert(!vent_countdown_report_due(&v,now));vent_countdown_operator(&v,now);assert(v.operator_ms==0);
 /* An Abort during the countdown (launch latch without Abort is gone) idles. */
 live(10000);link_up_until(20000);run_until(100000U);assert(v.state==VENT_CD_COUNTING);
 launched=0;run_until(400000U);assert(!vented_at_set&&v.state==VENT_CD_IDLE);
}

static void reports(void){
 char t[VENT_REPORT_MAX];
 live(10000);link_up_until(20000);
 assert(!vent_countdown_report_due(&v,now));
 const uint64_t last=hb;run_until(last+VENT_LINK_LOSS_DETECT_MS+1U);
 assert(vent_countdown_report_due(&v,now));
 vent_countdown_format_report(&v,now,t,sizeof t);assert(!strcmp(t,"Valve Board auto-vent in 145 s: link lost [L145]"));
 vent_countdown_report_attempt(&v,now);assert(!vent_countdown_report_due(&v,now+50U));
 vent_countdown_report_sent(&v,now);assert(!vent_countdown_report_due(&v,now+VENT_REPORT_PERIOD_MS-1U));
 assert(vent_countdown_report_due(&v,now+VENT_REPORT_PERIOD_MS));
 cont=now;run_until(now+1U);assert(vent_countdown_report_due(&v,now));
 vent_countdown_format_report(&v,now,t,sizeof t);assert(!strcmp(t,"Valve Board auto-vent in 145 s: link lost, continuity broken [B145]"));
 vent_countdown_report_sent(&v,now);
 link_up_until(now+1000U);vent_countdown_format_report(&v,now,t,sizeof t);
 assert(!strcmp(t,"Valve Board auto-vent in 144 s: continuity broken [K144]"));
 run_until(last+VENT_AFTER_LINK_LOSS_MS+10000U);assert(v.state==VENT_CD_VENTED&&vent_countdown_report_due(&v,now));
 vent_countdown_format_report(&v,now,t,sizeof t);assert(!strcmp(t,"Valve Board auto-vented: Vent and Dump open, Pilot closed [V3]"));
 vent_countdown_operator(&v,now);vent_countdown_format_report(&v,now,t,sizeof t);
 assert(!strcmp(t,"Valve Board auto-vent cancelled [X0]")&&vent_countdown_report_due(&v,now+200U));
 vent_countdown_report_sent(&v,now+200U);assert(!vent_countdown_report_due(&v,now+60000U));
 char tiny[8];vent_countdown_format_report(&v,now,tiny,sizeof tiny);assert(strlen(tiny)==7);
}

int main(int argc,char**argv){
 if(argc<2)return 2;
 const char*c=argv[1];
 if(!strcmp(c,"loss"))loss_vents();
 else if(!strcmp(c,"return"))return_cancels_then_restart();
 else if(!strcmp(c,"guard"))burn_guard();
 else if(!strcmp(c,"operator"))operator_cancels();
 else if(!strcmp(c,"continuity"))continuity_only();
 else if(!strcmp(c,"both"))both_triggers();
 else if(!strcmp(c,"continuity_restored"))continuity_restored_link_lost();
 else if(!strcmp(c,"reboot_no_link"))reboot_no_link();
 else if(!strcmp(c,"reboot_link"))reboot_link_present();
 else if(!strcmp(c,"prelaunch"))no_vent_before_launch();
 else if(!strcmp(c,"reports"))reports();
 else return 2;
 puts("ok");return 0;
}
'''

OWNER_HARNESS = r'''
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
static bool controls_ready=true;
bool persistent_store_controls_ready(void){return controls_ready;}
bool persistent_store_completed_boundary_ready(void){return false;}
uint8_t board_recovery_manual_time_retired(uint64_t source){(void)source;return 0;}
bool persistent_store_bootstrap_pending(void){return false;}
bool persistent_store_bootstrap_metadata_ready(void){return true;}
void persistent_store_hold_bootstrap(void){}
bool persistent_store_publish_bootstrap_complete(void){return false;}
bool persistent_store_recovery_ready(void){return true;}
bool persistent_store_classify_absent_floor(uint64_t expected){(void)expected;return false;}
bool persistent_store_commit_absent_floor(const uint8_t value[16]){(void)value;return false;}
#include "thread_comm.h"
#include "board_commands.h"
#include "vent_countdown.h"
#define RCC_APB1ENR1_PWREN 1U
#define RCC_APB1ENR1_RTCAPBEN 2U
#define PWR_CR1_DBP 4U
#define __DSB() ((void)0)
static struct {uint32_t APB1ENR1;} rcc;
static struct {uint32_t CR1;} pwr;
static struct {uint32_t BKP26R,BKP27R,BKP28R,BKP29R,BKP30R,BKP31R;} tamp;
#define RCC (&rcc)
#define PWR (&pwr)
#define TAMP (&tamp)
LATCH_SOURCE
SHARED_SOURCE
static unsigned long ticks;
unsigned long tx_time_get(void){return ticks;}
UINT tx_mutex_create(TX_MUTEX*m,const char*n,UINT i){(void)n;(void)i;m->locked=0;return TX_SUCCESS;}
UINT tx_mutex_get(TX_MUTEX*m,ULONG wait){(void)wait;if(m->locked)return TX_MUTEX_ERROR;m->locked=1;return TX_SUCCESS;}
UINT tx_mutex_put(TX_MUTEX*m){assert(m->locked);m->locked=0;return TX_SUCCESS;}
UINT tx_semaphore_create(TX_SEMAPHORE*s,const char*n,ULONG c){(void)n;s->count=c;return TX_SUCCESS;}
UINT tx_semaphore_get(TX_SEMAPHORE*s,ULONG wait){(void)wait;if(!s->count)return TX_NO_INSTANCE;s->count--;return TX_SUCCESS;}
UINT tx_semaphore_put(TX_SEMAPHORE*s){s->count++;return TX_SUCCESS;}

/* ---- inert drivers: every physical write must hold the state owner and is
 * recorded in order ---- */
typedef int SedsResult;
#define SEDS_OK 0
#define SEDS_ERR -2
#define SEDS_DT_WARNING 77U
typedef unsigned SedsDataType;
static unsigned pilot_solenoid;
static uint8_t pilot_hw,dump_hw,vent_hw;
static char order[64];static unsigned order_len;
static void owned(char c){assert(g_thread_comm_state_mutex.locked);if(order_len<63U)order[order_len++]=c;}
static int solenoidOn(unsigned*h){(void)h;owned('P');pilot_hw=1;return 0;}
static void solenoidOff(unsigned*h){(void)h;owned('p');pilot_hw=0;}
static void dump_servo_close(void){owned('d');dump_hw=0;}static void dump_servo_open(void){owned('D');dump_hw=1;}
static void no_servo_close(void){owned('v');vent_hw=0;}static void no_servo_open(void){owned('V');vent_hw=1;}
static uint8_t g_pilot_valve_state,g_sim_pilot_valve_state,g_nitrous_servo_open_state,g_no_servo_open_state;
static uint8_t g_launch_sequence_active,g_launch_sequence_actuator_started,g_launch_sequence_pilot_opened,g_aborted;
static uint32_t g_launch_sequence_pilot_open_ticks,g_sim_valve_commands_executed,g_sim_last_valve_command;
static uint64_t g_sim_last_valve_command_ms;static ULONG g_abort_last_status_ticks;
static volatile uint8_t g_outputs_initialized=1;
static int32_t g_sim_pilot_solenoid_result;
/* ---- network boundary ---- */
static char reports[256][VENT_REPORT_MAX];static unsigned report_count;static int sdk_reject;
static unsigned status_count;
static SedsResult log_telemetry_string_asynchronous(SedsDataType t,const char*text){
 assert(t==SEDS_DT_WARNING&&!g_thread_comm_state_mutex.locked);if(sdk_reject)return SEDS_ERR;
 strcpy(reports[report_count++%256U],text);return SEDS_OK;}
static SedsResult publish_umbilical_status(uint8_t id,uint8_t state){(void)id;(void)state;assert(!g_thread_comm_state_mutex.locked);status_count++;return SEDS_OK;}
static uint64_t continuity_since;
uint64_t safety_launch_continuity_broken_since_ms(void){return continuity_since;}
static uint8_t overpressure_refused(thread_comm_msg_t cmd){(void)cmd;return 0;}
static void service_launch_sequence_request(void){}
#define TELEMETRY_ENABLED 1
CONTROL_SOURCE
static unsigned has(const char*text){for(unsigned i=0;i<report_count&&i<256U;i++)if(!strcmp(reports[i],text))return 1;return 0;}
static unsigned has_prefix(const char*text){for(unsigned i=0;i<report_count&&i<256U;i++)if(!strncmp(reports[i],text,strlen(text)))return 1;return 0;}
static void boot_board(void){
 latched=launch_latched=backup_ready=0;board_abort_latch_init();
 g_thread_comm_initialized=0;TX_BYTE_POOL pool={0};assert(thread_comm_init(&pool)==TX_SUCCESS);
 assert(thread_comm_bind_manual_stop_apply(manual_stop_apply_locked)==TX_SUCCESS);
 g_vent_marker_stored=UINT32_MAX;report_count=0;order_len=0;continuity_since=0;
 /* main_thread_entry's initial outputs. */
 pilot_hw=0;if(post_launch_reset_hold()){vent_hw=dump_hw=0;g_no_servo_open_state=g_nitrous_servo_open_state=0;}
 else{vent_hw=dump_hw=1;g_no_servo_open_state=g_nitrous_servo_open_state=1;}
 vent_boot();
}
static void power_on(void){memset(&tamp,0,sizeof tamp);ticks=0;boot_board();}
static void heartbeat(void){assert(thread_comm_note_groundstation_heartbeat(ticks)==TX_SUCCESS);}
static void loop_ms(unsigned n,unsigned hb_period){
 for(unsigned i=0;i<n;i++){ticks++;if(hb_period&&ticks%hb_period==0U)heartbeat();service_vent_countdown();}
}
/* Launch admission and its pilot sequence, then the closed post-Launch state. */
static void launch(void){
 assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_ARMED)==TX_SUCCESS);
 assert(thread_comm_request_launch_sequence(42)==TX_SUCCESS);assert(board_launch_latch_is_set());
 vent_hw=dump_hw=0;g_no_servo_open_state=g_nitrous_servo_open_state=0;
 thread_comm_launch_owner_t owner;assert(thread_comm_take_accepted_launch(&owner));
}
static void operator_valve(uint8_t cmd){handle_command(cmd,thread_comm_manual_epoch());}

/* Loss after Launch: vent at last heartbeat + 150 s, in the Abort order. */
static void loss_case(void){
 power_on();loop_ms(10000,500);launch();loop_ms(60000,500);const unsigned long last=ticks;
 order_len=0;loop_ms(VENT_AFTER_LINK_LOSS_MS-1U,0);assert(!vent_hw&&!dump_hw&&order_len==0);
 assert(has_prefix("Valve Board auto-vent in 145 s: link lost [L145]"));
 loop_ms(1,0);assert(ticks==last+VENT_AFTER_LINK_LOSS_MS);
 assert(vent_hw&&dump_hw&&!pilot_hw&&!strcmp(order,"VDp"));
 assert(g_vent.state==VENT_CD_VENTED&&!thread_comm_get_abort()&&board_vent_marker_vented());
 loop_ms(1000,0);assert(has("Valve Board auto-vented: Vent and Dump open, Pilot closed [V1]"));
 /* Heartbeat back: stays vented and open. */
 order_len=0;loop_ms(30000,500);assert(vent_hw&&dump_hw&&order_len==0&&g_vent.state==VENT_CD_VENTED);
 /* Reset after the vent: no re-close, still reported vented. */
 boot_board();assert(vent_hw&&dump_hw&&g_vent.state==VENT_CD_VENTED);
 loop_ms(10,0);assert(has("Valve Board auto-vented: Vent and Dump open, Pilot closed [V0]"));
}

/* Heartbeat back cancels; the operator command cancels. */
static void cancel_case(void){
 power_on();loop_ms(10000,500);launch();loop_ms(60000,500);
 loop_ms(100000,0);assert(g_vent.state==VENT_CD_COUNTING);
 loop_ms(1000,500);assert(g_vent.state==VENT_CD_IDLE&&has("Valve Board auto-vent cancelled [X0]"));
 loop_ms(100000,0);assert(g_vent.state==VENT_CD_COUNTING);
 /* An operator valve command (still no heartbeat) cancels and restarts in full. */
 operator_valve(CMD_PILOT_CLOSE);assert(g_vent.state==VENT_CD_IDLE);const unsigned long cmd=ticks;
 loop_ms(VENT_AFTER_LINK_LOSS_MS-1U,0);assert(!vent_hw&&!dump_hw);
 loop_ms(1,0);assert(vent_hw&&dump_hw&&ticks==cmd+VENT_AFTER_LINK_LOSS_MS);
 /* Operator closes Vent after the vent: control is theirs, marker cleared. */
 operator_valve(CMD_NORMALLY_OPEN_CLOSE);assert(!vent_hw&&g_vent.state==VENT_CD_IDLE&&!board_vent_marker_vented());
 boot_board();assert(!vent_hw&&!dump_hw);
}

/* Stop during the countdown: the Abort safe state, no auto-vent afterwards. */
static void stop_case(void){
 power_on();loop_ms(10000,500);launch();loop_ms(60000,500);loop_ms(20000,0);
 assert(g_vent.state==VENT_CD_COUNTING);
 assert(thread_comm_request_manual_abort()==TX_SUCCESS);manual_stop_apply_locked_check();
 loop_ms(300000,0);assert(g_vent.state==VENT_CD_IDLE&&has("Valve Board auto-vent cancelled [X0]"));
}

/* The state owner refuses during a running pilot sequence and before Launch. */
static void owner_case(void){
 power_on();static int called;
 assert(!thread_comm_post_launch_vent_effect(count_call,&called)&&!called);
 loop_ms(10000,500);
 assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_ARMED)==TX_SUCCESS);
 assert(thread_comm_request_launch_sequence(42)==TX_SUCCESS);
 /* Accepted but never started: retired so the pilot can never open after. */
 assert(thread_comm_post_launch_vent_effect(count_call,&called)&&called==1);
 thread_comm_launch_owner_t owner;assert(!thread_comm_take_accepted_launch(&owner));
 power_on();loop_ms(10000,500);launch();
 thread_comm_launch_owner_t taken={0};assert(thread_comm_launch_claim(42,thread_comm_manual_epoch(),&taken));
 assert(thread_comm_launch_begin_owned(&taken,count_call,&called)&&called==2);
 assert(!thread_comm_post_launch_vent_effect(count_call,&called)&&called==2);
 thread_comm_launch_complete_owned(&taken);
 assert(thread_comm_post_launch_vent_effect(count_call,&called)&&called==3);
 /* Aborted: refused. */
 assert(thread_comm_request_manual_abort()==TX_SUCCESS);
 assert(!thread_comm_post_launch_vent_effect(count_call,&called)&&called==3);
}

/* Reset after Launch: hold closed; vent after 150 s without link; hold with link. */
static void reboot_case(void){
 power_on();loop_ms(10000,500);launch();loop_ms(30001,500);
 assert(vent_countdown_elapsed_s(&g_vent,ticks)==30U);
 ticks=0;boot_board();assert(!vent_hw&&!dump_hw&&board_launch_latch_is_set());
 assert(g_vent.guard_ms==(VENT_GUARD_S-30U)*1000U);
 loop_ms(VENT_AFTER_LINK_LOSS_MS-1U,0);assert(!vent_hw&&!dump_hw);
 loop_ms(1,0);assert(vent_hw&&dump_hw&&!pilot_hw&&!strcmp(order,"VDp"));
 /* Link present after the reset: holds closed for an hour. */
 power_on();loop_ms(10000,500);launch();loop_ms(70000,500);
 ticks=0;boot_board();loop_ms(1500,0);loop_ms(3600000,500);assert(!vent_hw&&!dump_hw&&g_vent.state==VENT_CD_IDLE);
 /* Continuity broken at boot with the link present: vents 150 s after it. */
 ticks=0;boot_board();loop_ms(1,500);continuity_since=1;
 loop_ms(VENT_AFTER_LINK_LOSS_MS-1U,500);assert(!vent_hw);loop_ms(1,500);assert(vent_hw&&dump_hw);
 /* A torn marker reads as absent: full guard, not vented. */
 tamp.BKP27R=0;ticks=0;boot_board();assert(!vent_hw&&g_vent.state==VENT_CD_IDLE&&g_vent.guard_ms==VENT_BURN_GUARD_MS);
 /* Recovery clears the marker with the launch latch. */
 board_vent_marker_store(10,1);assert(board_abort_latch_clear_for_recovery());
 assert(!tamp.BKP26R&&!tamp.BKP27R&&!board_vent_marker_vented());
 /* A new Launch marker starts a clean vent marker. */
 power_on();board_vent_marker_store(10,1);loop_ms(10000,500);launch();assert(!board_vent_marker_vented());
}

/* Pre-Launch: no countdown; the existing heartbeat/continuity Abort applies. */
static void prelaunch_case(void){
 power_on();loop_ms(10000,500);assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_FILL)==TX_SUCCESS);
 vent_hw=dump_hw=0;continuity_since=5;loop_ms(600000,0);
 assert(!vent_hw&&!dump_hw&&order_len==0&&g_vent.state==VENT_CD_IDLE&&report_count==0);
 assert(thread_comm_request_heartbeat_abort());
}

int main(int argc,char**argv){
 if(argc<2)return 2;
 if(!strcmp(argv[1],"loss"))loss_case();
 else if(!strcmp(argv[1],"cancel"))cancel_case();
 else if(!strcmp(argv[1],"stop"))stop_case();
 else if(!strcmp(argv[1],"owner"))owner_case();
 else if(!strcmp(argv[1],"reboot"))reboot_case();
 else if(!strcmp(argv[1],"prelaunch"))prelaunch_case();
 else return 2;
 puts("ok");return 0;
}
'''

EXTRA = r'''
static void count_call(void*c){(*(int*)c)++;}
static void manual_stop_apply_locked_check(void){assert(vent_hw&&dump_hw&&!pilot_hw);}
'''


class VentCountdownTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix='vb-vent-countdown-')
        tmp = Path(cls.tmp.name)
        (tmp / 'tx_api.h').write_text(TX_API)
        (tmp / 'module.c').write_text(MODULE_HARNESS)
        cls.module = tmp / 'module'
        subprocess.run(CFLAGS + ['-I', str(ROOT / 'Core/Inc'), str(tmp / 'module.c'),
                                 str(ROOT / 'Core/Src/vent_countdown.c'), '-o', str(cls.module)], check=True)
        latch = '\n'.join(l for l in (ROOT / 'Core/Src/board_abort_latch.c').read_text().splitlines()
                          if not l.startswith('#include'))
        shared = (ROOT / 'Core/Src/thread_comm.c').read_text()
        control = (ROOT / 'Core/Src/main_thread.c').read_text()
        start = control.index('/* ---- Post-Launch auto-vent')
        end = control.index('/* Servo compare/GPIO readback')
        names = ['manual_stop_apply_locked', 'main_thread_force_outputs_safe_off', 'publish_abort_umbilical_statuses',
                 'post_launch_reset_hold', 'apply_automatic_valve', 'apply_manual_valve_locked']
        pieces = '\n'.join(function(control, n) for n in names)
        pieces += '\nstatic void vent_operator_command(void);\n' + function(control, 'handle_command')
        owner = OWNER_HARNESS.replace('LATCH_SOURCE', latch).replace('SHARED_SOURCE', shared)
        owner = owner.replace('CONTROL_SOURCE', pieces + '\n' + control[start:end] + EXTRA)
        (tmp / 'owner.c').write_text(owner)
        cls.owner = tmp / 'owner'
        subprocess.run(CFLAGS + ['-Wno-unused-function', '-I', str(tmp), '-I', str(ROOT / 'Core/Inc'),
                                 str(tmp / 'owner.c'), str(ROOT / 'Core/Src/vent_countdown.c'),
                                 '-o', str(cls.owner)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_case(self, exe, name):
        result = subprocess.run([str(exe), name], capture_output=True, text=True, timeout=300)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_link_loss_vents_150_s_after_last_heartbeat(self): self.run_case(self.module, 'loss')
    def test_heartbeat_return_cancels_and_second_loss_restarts(self): self.run_case(self.module, 'return')
    def test_never_vents_within_burn_guard(self): self.run_case(self.module, 'guard')
    def test_operator_valve_command_cancels(self): self.run_case(self.module, 'operator')
    def test_continuity_only_counts_down_with_heartbeat_present(self): self.run_case(self.module, 'continuity')
    def test_both_triggers_keep_one_deadline(self): self.run_case(self.module, 'both')
    def test_continuity_restored_while_link_lost_keeps_counting(self): self.run_case(self.module, 'continuity_restored')
    def test_reboot_without_link_vents_after_countdown(self): self.run_case(self.module, 'reboot_no_link')
    def test_reboot_with_link_holds_closed(self): self.run_case(self.module, 'reboot_link')
    def test_no_countdown_before_launch(self): self.run_case(self.module, 'prelaunch')
    def test_report_format_and_cadence(self): self.run_case(self.module, 'reports')

    def test_owner_vents_in_abort_order_and_latches_across_reset(self): self.run_case(self.owner, 'loss')
    def test_owner_heartbeat_and_operator_command_cancel(self): self.run_case(self.owner, 'cancel')
    def test_owner_stop_supersedes_countdown(self): self.run_case(self.owner, 'stop')
    def test_owner_effect_refused_during_pilot_and_after_abort(self): self.run_case(self.owner, 'owner')
    def test_owner_reboot_after_launch(self): self.run_case(self.owner, 'reboot')
    def test_owner_prelaunch_unchanged(self): self.run_case(self.owner, 'prelaunch')

    def test_production_wiring(self):
        control = (ROOT / 'Core/Src/main_thread.c').read_text()
        loop = function(control, 'main_thread_entry')
        # Runs every loop before the Abort branch, after the initial outputs.
        self.assertLess(loop.index('service_vent_countdown();'), loop.index('if (thread_comm_get_abort() != 0U)'))
        self.assertLess(loop.index('post_launch_reset_hold()'), loop.index('vent_boot();'))
        self.assertIn('vent_operator_command();', function(control, 'handle_command'))
        self.assertIn('main_thread_force_outputs_safe_off();', function(control, 'vent_locked'))
        safety = (ROOT / 'Core/Src/safety_thread.c').read_text()
        self.assertIn('board_launch_latch_is_set()', function(safety, 'safety_check_continuity'))
        self.assertIn('vent_countdown.c', (ROOT / 'CMakeLists.txt').read_text())
        header = (ROOT / 'Core/Inc/vent_countdown.h').read_text()
        self.assertIn('#define VENT_AFTER_LINK_LOSS_MS 150000U', header)
        self.assertIn('#define VENT_BURN_GUARD_MS 60000U', header)
        self.assertIn('#define VENT_LINK_LOSS_DETECT_MS 5000U', header)
        self.assertIn('#define SAFETY_HEARTBEAT_TIMEOUT_MS (5000ULL)', safety)


if __name__ == '__main__':
    unittest.main()
