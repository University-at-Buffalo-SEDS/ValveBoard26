"""Board tank overpressure protection on the production sources.

The decision module is exercised directly. The control-owner service, the
command filter and the valve effects are extracted from main_thread.c and run
against the real thread_comm.c and board_abort_latch.c with inert drivers, so
ordering, Launch exclusion, the Abort latch and Launch-admission interaction
are the production code. Host logic evidence only: no MCU timing or hardware.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def function(source, name):
    match = re.search(r'^[^\n;]*\b' + re.escape(name) + r'\([^;]*?\)\s*\{', source, re.M)
    assert match, name
    start = match.start(); brace = source.index('{', start); end, depth = brace + 1, 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}'); end += 1
    return source[start:end]


TX_API = r'''#pragma once
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
#define TX_TIMER_TICKS_PER_SECOND 1000U
unsigned long tx_time_get(void);
UINT tx_mutex_create(TX_MUTEX*,const char*,UINT);UINT tx_mutex_get(TX_MUTEX*,ULONG);UINT tx_mutex_put(TX_MUTEX*);
UINT tx_semaphore_create(TX_SEMAPHORE*,const char*,ULONG);UINT tx_semaphore_get(TX_SEMAPHORE*,ULONG);UINT tx_semaphore_put(TX_SEMAPHORE*);
'''

MODULE_HARNESS = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "overpressure.h"
#include "board_commands.h"
static overpressure_t op; static overpressure_sample_t s; static uint32_t now;
static void reset(void){memset(&op,0,sizeof op);memset(&s,0,sizeof s);now=1000U;}
/* One fresh acquisition 20 ms after the previous one (50 Hz). */
static uint32_t feed(float psi,uint8_t burn){now+=20U;s.seq++;s.at_ms=now;s.psi=psi;s.valid=1;s.raw_adc=(uint32_t)(psi*3103.0f/2000.0f);return overpressure_step(&op,&s,now,burn);}
static uint32_t feed_invalid(uint32_t raw){now+=20U;s.seq++;s.at_ms=now;s.raw_adc=raw;s.psi=2000.0f;s.valid=0;return overpressure_step(&op,&s,now,0);}
static uint32_t idle(uint32_t ms,uint8_t burn){now+=ms;return overpressure_step(&op,&s,now,burn);}
static const float MIN=0.0f,MAX=3103.0f,PSI=2000.0f;

static void thresholds(void){
 reset();assert(!feed(849.9f,0));assert(!feed(849.9f,0));assert(!feed(849.99f,0));assert(op.stage==OVERPRESSURE_IDLE);
 assert(!feed(850.0f,0));uint32_t a=feed(850.0f,0);
 assert(a==(OVERPRESSURE_ACT_RELIEF|OVERPRESSURE_ACT_REPORT)&&op.stage==OVERPRESSURE_RELIEF&&op.trips==1);
 reset();feed(949.9f,0);a=feed(949.9f,0);assert(op.stage==OVERPRESSURE_RELIEF&&!(a&OVERPRESSURE_ACT_PILOT_OPEN));
 reset();feed(950.0f,0);a=feed(950.0f,0);
 assert(a==(OVERPRESSURE_ACT_RELIEF|OVERPRESSURE_ACT_PILOT_OPEN|OVERPRESSURE_ACT_REPORT)&&op.stage==OVERPRESSURE_PILOT);
 /* Matches the GroundStation defaults. */
 assert(OVERPRESSURE_RELIEF_PSI==850.0f&&OVERPRESSURE_ABORT_PSI==950.0f&&OVERPRESSURE_PILOT_MS==3000U&&OVERPRESSURE_DEBOUNCE_SAMPLES==2U&&OVERPRESSURE_FRESH_MS==2000U);
}

static void debounce(void){
 reset();assert(!feed(1200.0f,0));assert(!feed(100.0f,0));assert(!feed(1200.0f,0));assert(op.stage==OVERPRESSURE_IDLE);
 /* A repeated sample (same sequence) never counts twice. */
 reset();feed(900.0f,0);for(int i=0;i<500;i++)assert(!idle(1,0));assert(op.stage==OVERPRESSURE_IDLE);
 /* A stale sample neither counts nor clears the streak. */
 reset();feed(900.0f,0);now+=20U;s.seq++;s.at_ms=now-2001U;s.psi=900.0f;assert(!(overpressure_step(&op,&s,now,0)&OVERPRESSURE_ACT_RELIEF));
 assert(op.stage==OVERPRESSURE_IDLE&&op.relief_streak==1);assert(feed(900.0f,0)&OVERPRESSURE_ACT_RELIEF);
 /* A future timestamp is not fresh either. */
 reset();feed(900.0f,0);now+=20U;s.seq++;s.at_ms=now+5U;assert(!overpressure_step(&op,&s,now,0)&&op.relief_streak==1);
 /* An abort-level spike after one relief sample is still only one abort sample. */
 reset();feed(900.0f,0);uint32_t a=feed(990.0f,0);assert(op.stage==OVERPRESSURE_RELIEF&&!(a&OVERPRESSURE_ACT_PILOT_OPEN));
 a=feed(990.0f,0);assert((a&OVERPRESSURE_ACT_PILOT_OPEN)&&op.stage==OVERPRESSURE_PILOT);
 /* Sequence 0 means no acquisition yet. */
 reset();s.seq=0;s.at_ms=now;s.psi=2000;s.valid=1;assert(!overpressure_step(&op,&s,now,0));assert(!overpressure_step(&op,&s,now,0));
}

static void invalid_sensor(void){
 /* Calibration constants used by the board. */
 assert(overpressure_sample_valid(1474U,950.1f,MIN,MAX,PSI));
 assert(!overpressure_sample_valid(4095U,2000.0f,MIN,MAX,PSI)); /* rail */
 assert(!overpressure_sample_valid(3103U,2000.0f,MIN,MAX,PSI)); /* calibrated full scale */
 assert(!overpressure_sample_valid(3500U,2000.0f,MIN,MAX,PSI)); /* clamped conversion */
 assert(overpressure_sample_valid(3102U,1999.3f,MIN,MAX,PSI));
 assert(!overpressure_sample_valid(1474U,NAN,MIN,MAX,PSI));
 assert(!overpressure_sample_valid(1474U,INFINITY,MIN,MAX,PSI));
 assert(!overpressure_sample_valid(1474U,-1.0f,MIN,MAX,PSI));
 assert(!overpressure_sample_valid(1474U,950.0f,MIN,0.0f,PSI));      /* zero span */
 assert(!overpressure_sample_valid(1474U,950.0f,3103.0f,MIN,PSI));   /* inverted */
 assert(!overpressure_sample_valid(1474U,950.0f,MIN,5000.0f,PSI));   /* beyond ADC */
 assert(!overpressure_sample_valid(1474U,900.0f,MIN,MAX,900.0f));   /* cannot represent abort */
 assert(!overpressure_sample_valid(1474U,950.0f,-5.0f,MAX,PSI));
 /* Full-scale fault for many samples: reported once, never trips. */
 reset();uint32_t a=feed_invalid(4095U);assert(a==OVERPRESSURE_ACT_FAULT&&op.fault==OVERPRESSURE_FAULT_INVALID&&op.fault_raw==4095U);
 for(int i=0;i<1000;i++)assert(!feed_invalid(4095U));assert(op.stage==OVERPRESSURE_IDLE&&op.trips==0);
 /* An invalid sample breaks consecutiveness. */
 reset();feed(900.0f,0);feed_invalid(3200U);assert(!(feed(900.0f,0)&OVERPRESSURE_ACT_RELIEF));assert(feed(900.0f,0)&OVERPRESSURE_ACT_RELIEF);
 assert(op.fault==OVERPRESSURE_FAULT_NONE);
 /* No acquisition at all (ADC down) is a stale fault, reported once. */
 reset();assert(!overpressure_step(&op,0,now,0));assert(!idle(2000U,0));a=idle(1U,0);assert(a==OVERPRESSURE_ACT_FAULT&&op.fault==OVERPRESSURE_FAULT_STALE);
 assert(!idle(5000U,0));feed(100.0f,0);assert(op.fault==OVERPRESSURE_FAULT_NONE);
 /* Samples that stop arriving after valid data also report once. */
 assert(!idle(2000U,0));assert(idle(30U,0)==OVERPRESSURE_ACT_FAULT);
 /* A latched relief stays latched through a later sensor fault. */
 reset();feed(900.0f,0);feed(900.0f,0);feed_invalid(4095U);assert(op.stage==OVERPRESSURE_RELIEF);
 assert(overpressure_acknowledge(&op,now)==OVERPRESSURE_ACK_NO_FRESH_SAMPLE&&op.stage==OVERPRESSURE_RELIEF);
}

static void launch_exclusion(void){
 reset();feed(990.0f,1);uint32_t a=feed(990.0f,1);
 assert(a==OVERPRESSURE_ACT_REPORT&&op.stage==OVERPRESSURE_ALARM&&op.alarm_abort);
 for(int i=0;i<300;i++){a=feed(1100.0f,1)|idle(10U,0);assert(!(a&~OVERPRESSURE_ACT_FAULT));}
 assert(!overpressure_owns_valves(&op));
 for(uint32_t c=0;c<256;c++)assert(!overpressure_refuses_command(&op,c));
 /* Relief-level alarm escalates to an abort-level report only. */
 reset();feed(900.0f,1);feed(900.0f,1);assert(op.stage==OVERPRESSURE_ALARM&&!op.alarm_abort);
 feed(990.0f,1);a=feed(990.0f,1);assert(a==OVERPRESSURE_ACT_REPORT&&op.alarm_abort);assert(!feed(990.0f,1));
 /* Relief latched before Launch: no escalation while burning. */
 reset();feed(900.0f,0);feed(900.0f,0);feed(990.0f,1);a=feed(990.0f,1);assert(!a&&op.stage==OVERPRESSURE_RELIEF);
 /* Effect refused by the state owner: demoted to report-only. */
 reset();feed(990.0f,0);feed(990.0f,0);overpressure_demote(&op);assert(op.stage==OVERPRESSURE_ALARM);
 assert(!(idle(OVERPRESSURE_PILOT_MS,0)&~OVERPRESSURE_ACT_FAULT));
}

static void pilot_sequence(void){
 for(int wrap=0;wrap<2;wrap++){
  reset();if(wrap)now=0xFFFFFF00U;
  feed(960.0f,0);uint32_t a=feed(960.0f,0);assert(a&OVERPRESSURE_ACT_PILOT_OPEN);const uint32_t opened=now;
  /* Pressure falling below relief does not end the main-valve window. */
  for(uint32_t t=0;t<OVERPRESSURE_PILOT_MS-1U;t++){now=opened+t;a=overpressure_step(&op,&s,now,0);assert(!(a&(OVERPRESSURE_ACT_PILOT_CLOSE|OVERPRESSURE_ACT_ABORT)));if(t==1500U){feed(100.0f,0);}}
  assert(op.stage==OVERPRESSURE_PILOT);
  assert(overpressure_acknowledge(&op,now)==OVERPRESSURE_ACK_PILOT_RUNNING);
  assert(overpressure_refuses_command(&op,CMD_PILOT_CLOSE)&&overpressure_refuses_command(&op,CMD_PILOT_CLOSE|0x80U));
  now=opened+OVERPRESSURE_PILOT_MS;a=overpressure_step(&op,&s,now,0);
  assert(a==(OVERPRESSURE_ACT_PILOT_CLOSE|OVERPRESSURE_ACT_ABORT|OVERPRESSURE_ACT_REPORT)&&op.stage==OVERPRESSURE_ABORTED);
  assert(!overpressure_step(&op,&s,now+1U,0));
  assert(op.stage==OVERPRESSURE_ABORTED);
 }
 /* Relief first, then escalation re-asserts relief with the main valve. */
 reset();feed(900.0f,0);feed(900.0f,0);feed(955.0f,0);uint32_t a=feed(955.0f,0);
 assert(a==(OVERPRESSURE_ACT_RELIEF|OVERPRESSURE_ACT_PILOT_OPEN|OVERPRESSURE_ACT_REPORT));
 /* A board Abort during the window supersedes it (pilot already closed). */
 assert(overpressure_abort_observed(&op)&&op.stage==OVERPRESSURE_ABORTED);assert(!(idle(OVERPRESSURE_PILOT_MS,0)&~OVERPRESSURE_ACT_FAULT));
 assert(!overpressure_abort_observed(&op));
 /* Never a second window in the same latch. */
 for(int i=0;i<10;i++)assert(!(feed(1000.0f,0)&OVERPRESSURE_ACT_PILOT_OPEN));
}

static void latch_acknowledge(void){
 reset();assert(overpressure_acknowledge(&op,now)==OVERPRESSURE_ACK_NOT_LATCHED);
 feed(900.0f,0);feed(900.0f,0);
 for(int i=0;i<100;i++)assert(!(feed(870.0f,0)&~OVERPRESSURE_ACT_REPORT));
 assert(overpressure_acknowledge(&op,now)==OVERPRESSURE_ACK_STILL_HIGH);
 /* Pressure falling never clears the latch by itself. */
 for(int i=0;i<500;i++)assert(!feed(20.0f,0));assert(op.stage==OVERPRESSURE_RELIEF);
 assert(op.peak_psi==900.0f);
 assert(overpressure_acknowledge(&op,now+2001U)==OVERPRESSURE_ACK_NO_FRESH_SAMPLE);
 assert(overpressure_acknowledge(&op,now)==OVERPRESSURE_ACK_CLEARED&&op.stage==OVERPRESSURE_IDLE);
 assert(!overpressure_owns_valves(&op)&&!overpressure_refuses_command(&op,CMD_DUMP_CLOSE));
 /* A fresh trip latches again with a new trip number. */
 feed(851.0f,0);assert(feed(851.0f,0)&OVERPRESSURE_ACT_RELIEF);assert(op.trips==2);
 /* Ack exactly at the boundary is refused. */
 feed(849.0f,0);feed(850.0f,0);assert(overpressure_acknowledge(&op,now)==OVERPRESSURE_ACK_STILL_HIGH);
 /* Abort stage acknowledges once pressure is low. */
 reset();feed(990.0f,0);feed(990.0f,0);idle(OVERPRESSURE_PILOT_MS,0);feed(10.0f,0);
 assert(overpressure_acknowledge(&op,now)==OVERPRESSURE_ACK_CLEARED);
 /* Alarm-only latch acknowledges the same way. */
 reset();feed(990.0f,1);feed(990.0f,1);feed(10.0f,1);assert(overpressure_acknowledge(&op,now)==OVERPRESSURE_ACK_CLEARED);
}

static void command_filter(void){
 reset();feed(900.0f,0);feed(900.0f,0);
 for(uint32_t tag=0;tag<2;tag++){const uint32_t t=tag?0x80U:0U;
  assert(overpressure_refuses_command(&op,CMD_DUMP_CLOSE|t)&&overpressure_refuses_command(&op,CMD_NORMALLY_OPEN_CLOSE|t));
  assert(overpressure_refuses_command(&op,CMD_PILOT_OPEN|t)&&overpressure_refuses_command(&op,CMD_SEQUENCE|t));
  assert(!overpressure_refuses_command(&op,CMD_DUMP_OPEN|t)&&!overpressure_refuses_command(&op,CMD_NORMALLY_OPEN_OPEN|t));
  assert(!overpressure_refuses_command(&op,CMD_PILOT_CLOSE|t)&&!overpressure_refuses_command(&op,CMD_ABORT|t));}
 char text[OVERPRESSURE_REPORT_MAX];
 assert(overpressure_format_report(text,sizeof text,overpressure_report_code(&op),912U,3U)==strlen("Valve Board overpressure relief: 912 psi [R3]"));
 assert(!strcmp(text,"Valve Board overpressure relief: 912 psi [R3]"));
 overpressure_format_report(text,sizeof text,'A',4294967295U,65535U);
 assert(!strcmp(text,"Valve Board overpressure relief, main valve closed, Abort: 4294967295 psi [A65535]"));
 overpressure_format_report(text,sizeof text,'F',4095U,0U);assert(!strcmp(text,"Valve Board overpressure sensor invalid, raw ADC 4095 [F0]"));
 overpressure_format_report(text,sizeof text,'S',7U,2U);assert(!strcmp(text,"Valve Board overpressure sensor stale [S2]"));
 overpressure_format_report(text,sizeof text,'L',990U,1U);assert(!strcmp(text,"Valve Board overpressure during launch, no valve action: 990 psi [L1]"));
 overpressure_format_report(text,sizeof text,'P',955U,4U);assert(!strcmp(text,"Valve Board overpressure relief, main valve open: 955 psi [P4]"));
 overpressure_format_report(text,sizeof text,'C',870U,1U);assert(!strcmp(text,"Valve Board overpressure acknowledged: 870 psi [C1]"));
 char tiny[8];overpressure_format_report(tiny,sizeof tiny,'R',123456U,1U);assert(strlen(tiny)==7);
 assert(overpressure_format_report(0,8,'R',1,1)==0&&overpressure_format_report(tiny,0,'R',1,1)==0);
}

int main(int argc,char**argv){
 assert(argc==2);
 if(!strcmp(argv[1],"thresholds"))thresholds();
 else if(!strcmp(argv[1],"debounce"))debounce();
 else if(!strcmp(argv[1],"invalid"))invalid_sensor();
 else if(!strcmp(argv[1],"launch"))launch_exclusion();
 else if(!strcmp(argv[1],"pilot"))pilot_sequence();
 else if(!strcmp(argv[1],"latch"))latch_acknowledge();
 else if(!strcmp(argv[1],"commands"))command_filter();
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
#include "overpressure.h"
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

/* ---- inert drivers: every physical write must hold the state owner ---- */
typedef int SedsResult;
#define SEDS_OK 0
#define SEDS_ERR -2
#define SEDS_DT_WARNING 77U
typedef unsigned SedsDataType;
static unsigned pilot_solenoid;
static uint8_t pilot_hw,dump_hw=1,vent_hw=1;
static unsigned physical;
static void owned(void){assert(g_thread_comm_state_mutex.locked);physical++;}
static int solenoidOn(unsigned*h){(void)h;owned();pilot_hw=1;return 0;}
static void solenoidOff(unsigned*h){(void)h;owned();pilot_hw=0;}
static void dump_servo_close(void){owned();dump_hw=0;}static void dump_servo_open(void){owned();dump_hw=1;}
static void no_servo_close(void){owned();vent_hw=0;}static void no_servo_open(void){owned();vent_hw=1;}
static uint8_t g_pilot_valve_state,g_sim_pilot_valve_state,g_nitrous_servo_open_state=1,g_no_servo_open_state=1,g_launch_sequence_active;
static int32_t g_sim_pilot_solenoid_result;
/* ---- network boundary ---- */
static char reports[64][OVERPRESSURE_REPORT_MAX];static unsigned report_count;
static uint8_t actuator[16];static unsigned actuator_count;static int sdk_reject;
static unsigned status_count;
static SedsResult log_telemetry_string_asynchronous(SedsDataType t,const char*text){
 assert(t==SEDS_DT_WARNING&&!g_thread_comm_state_mutex.locked);if(sdk_reject)return SEDS_ERR;
 assert(report_count<64);strcpy(reports[report_count++],text);return SEDS_OK;}
static SedsResult telemetry_send_actuator_command(uint8_t cmd){assert(!g_thread_comm_state_mutex.locked);if(sdk_reject)return SEDS_ERR;assert(actuator_count<16);actuator[actuator_count++]=cmd;return SEDS_OK;}
static SedsResult publish_umbilical_status(uint8_t id,uint8_t state){(void)id;(void)state;assert(!g_thread_comm_state_mutex.locked);status_count++;return SEDS_OK;}
static uint64_t telemetry_event_clock_snapshot(void){return 0;}
static overpressure_sample_t latest;
static void data_acq_latest_pressure(overpressure_sample_t*out){*out=latest;}
APPLY_SOURCE
CONTROL_SOURCE
static void reset_power(void){
 memset(&tamp,0,sizeof tamp);latched=launch_latched=backup_ready=0;board_abort_latch_init();
 g_thread_comm_initialized=0;TX_BYTE_POOL pool={0};assert(thread_comm_init(&pool)==TX_SUCCESS);
 memset(&g_overpressure,0,sizeof g_overpressure);g_overpressure_pending=0;g_overpressure_report_ticks=0;
 (void)thread_comm_take_overpressure_ack();
 memset(&latest,0,sizeof latest);ticks=1000;pilot_hw=0;dump_hw=vent_hw=1;g_launch_sequence_active=0;
 report_count=actuator_count=status_count=0;sdk_reject=0;controls_ready=true;
}
/* Control loop: 1 ms ticks, data thread publishes every 20 ms. */
static void run(float psi,unsigned ms){
 for(unsigned i=0;i<ms;i++){ticks++;
  if(ticks%20U==0U){latest.seq++;latest.at_ms=(uint32_t)ticks;latest.psi=psi;latest.raw_adc=(uint32_t)(psi*1.5515f);latest.valid=overpressure_sample_valid(latest.raw_adc,psi,0.0f,3103.0f,2000.0f);}
  service_overpressure();}
}
static int has(const char*text){for(unsigned i=0;i<report_count;i++)if(!strcmp(reports[i],text))return 1;return 0;}
static int has_prefix(const char*text){for(unsigned i=0;i<report_count;i++)if(!strncmp(reports[i],text,strlen(text)))return 1;return 0;}
static void close_dump_vent_hw(void){dump_hw=vent_hw=0;}

static void relief_case(void){
 reset_power();assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_NITROUS_FILL)==TX_SUCCESS);
 close_dump_vent_hw();run(820.0f,200);assert(!dump_hw&&!vent_hw&&!actuator_count&&!report_count);
 run(870.0f,45);
 assert(dump_hw&&vent_hw&&!pilot_hw&&!thread_comm_get_abort());
 assert(actuator_count==3&&actuator[0]==(CMD_NITROGEN_CLOSE|THREAD_COMM_AUTOMATIC_TAG)&&actuator[1]==(CMD_NITROUS_CLOSE|THREAD_COMM_AUTOMATIC_TAG));
 /* The Actuator Board also enters its Abort safe state (AB command 16). */
 assert(actuator[2]==(16U|THREAD_COMM_AUTOMATIC_TAG));
 assert(has("Valve Board overpressure relief: 870 psi [R1]"));
 /* Launch admission is refused while relief owns the valves. */
 assert(thread_comm_request_launch_sequence(42)!=TX_SUCCESS&&!board_launch_latch_is_set());
 /* Close commands for dump/vent and pilot open are refused, opens pass. */
 const thread_comm_msg_t refused[]={CMD_DUMP_CLOSE,CMD_NORMALLY_OPEN_CLOSE,CMD_PILOT_OPEN,CMD_DUMP_CLOSE|THREAD_COMM_AUTOMATIC_TAG,CMD_NORMALLY_OPEN_CLOSE|THREAD_COMM_AUTOMATIC_TAG,CMD_SEQUENCE};
 for(unsigned i=0;i<sizeof refused/sizeof refused[0];i++)assert(overpressure_refused(refused[i]));
 assert(!overpressure_refused(CMD_DUMP_OPEN)&&!overpressure_refused(CMD_ABORT)&&!overpressure_refused(CMD_PILOT_CLOSE));
 /* Periodic re-announcement keeps the latch visible. */
 const unsigned before=report_count;run(300.0f,2100);assert(report_count>=before+2&&has("Valve Board overpressure relief: 870 psi [R1]"));
 assert(dump_hw&&vent_hw&&actuator_count==3);
 /* Acknowledge at low pressure: latch clears, nothing re-closes. */
 thread_comm_note_overpressure_ack();run(300.0f,60);assert(has("Valve Board overpressure acknowledged: 870 psi [C1]"));
 assert(dump_hw&&vent_hw&&!overpressure_owns_valves(&g_overpressure));
 assert(thread_comm_request_launch_sequence(42)==TX_SUCCESS);
}

static void ack_refused_case(void){
 reset_power();assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_IDLE)==TX_SUCCESS);
 run(900.0f,60);assert(overpressure_owns_valves(&g_overpressure));
 thread_comm_note_overpressure_ack();run(900.0f,30);assert(overpressure_owns_valves(&g_overpressure));
 assert(thread_comm_request_launch_sequence(42)!=TX_SUCCESS);
 /* Stale pressure: acknowledgement refused even though the last value was low. */
 run(100.0f,40);const uint32_t seq=latest.seq;
 for(unsigned i=0;i<2100;i++){ticks++;latest.seq=seq;service_overpressure();}
 assert(has_prefix("Valve Board overpressure sensor stale [S"));
 thread_comm_note_overpressure_ack();ticks++;service_overpressure();assert(overpressure_owns_valves(&g_overpressure));
 run(100.0f,40);thread_comm_note_overpressure_ack();run(100.0f,2);assert(!overpressure_owns_valves(&g_overpressure));
}

static void pilot_abort_case(void){
 reset_power();assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_NITROUS_FILL)==TX_SUCCESS);
 close_dump_vent_hw();run(900.0f,100);assert(dump_hw&&vent_hw&&!pilot_hw);
 run(970.0f,40);assert(pilot_hw&&g_overpressure.stage==OVERPRESSURE_PILOT&&!thread_comm_get_abort());
 /* The window spans exactly 3 s of control ticks. */
 unsigned held=0;while(pilot_hw&&held<4000U){run(970.0f,1);held++;}
 assert(held>=2999U&&held<=3001U);
 assert(!pilot_hw&&thread_comm_get_abort()&&board_abort_latch_is_set());
 assert(g_safety_first_abort.reason==SAFETY_ABORT_OVERPRESSURE);
 thread_comm_safety_report_t r;assert(thread_comm_safety_report_claim(&r)&&r.reason==SAFETY_ABORT_OVERPRESSURE);
 thread_comm_safety_report_finish(r.lease,1);
 assert(has_prefix("Valve Board overpressure relief, main valve open: 970 psi [P1]")&&has_prefix("Valve Board overpressure relief, main valve closed, Abort: 970 psi [A1]"));
 assert(dump_hw&&vent_hw);
 /* Latch survives Abort; acknowledgement works while Abort is latched. */
 run(200.0f,100);thread_comm_note_overpressure_ack();run(200.0f,2);
 assert(!overpressure_owns_valves(&g_overpressure)&&thread_comm_get_abort());
}

static void direct_abort_case(void){
 /* A step straight to 950 opens dump, vent and pilot in the same loop. */
 reset_power();assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_STARTUP)==TX_SUCCESS);
 close_dump_vent_hw();run(1000.0f,40);assert(dump_hw&&vent_hw&&pilot_hw);
 /* AB Abort goes at once; only the VB's own Abort waits for the window. */
 assert(actuator_count==3&&actuator[2]==(16U|THREAD_COMM_AUTOMATIC_TAG)&&!thread_comm_get_abort());
 run(1000.0f,3000);assert(!pilot_hw&&thread_comm_get_abort());
}

static void already_aborted_case(void){
 reset_power();assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_IDLE)==TX_SUCCESS);
 assert(thread_comm_request_manual_abort()==TX_SUCCESS);
 run(1000.0f,40);assert(!pilot_hw&&dump_hw&&vent_hw&&g_overpressure.stage==OVERPRESSURE_ABORTED);
 assert(actuator_count==3);run(1000.0f,4000);assert(!pilot_hw);
}

static void manual_abort_in_window_case(void){
 reset_power();assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_IDLE)==TX_SUCCESS);
 run(1000.0f,40);assert(pilot_hw);run(1000.0f,1000);
 assert(thread_comm_request_manual_abort()==TX_SUCCESS);pilot_hw=0; /* abort_state closes it */
 run(1000.0f,1);assert(g_overpressure.stage==OVERPRESSURE_ABORTED);
 const unsigned p=physical;run(1000.0f,4000);assert(physical==p&&!pilot_hw);
}

static void launch_exclusion_case(void){
 const uint8_t burn_states[]={VALVE_FLIGHT_STATE_LAUNCH,VALVE_FLIGHT_STATE_ASCENT,VALVE_FLIGHT_STATE_COAST,VALVE_FLIGHT_STATE_APOGEE,VALVE_FLIGHT_STATE_PARACHUTE_DEPLOY,VALVE_FLIGHT_STATE_DESCENT,VALVE_FLIGHT_STATE_LANDED,VALVE_FLIGHT_STATE_RECOVERY};
 for(unsigned i=0;i<sizeof burn_states;i++){
  reset_power();assert(thread_comm_set_flight_state(burn_states[i])==TX_SUCCESS);
  close_dump_vent_hw();pilot_hw=1;const unsigned p=physical;run(1000.0f,5000);
  assert(physical==p&&pilot_hw&&!dump_hw&&!vent_hw&&!actuator_count&&!thread_comm_get_abort());
  assert(g_overpressure.stage==OVERPRESSURE_ALARM&&has_prefix("Valve Board overpressure during launch, no valve action: 1000 psi [L1]"));
  assert(!overpressure_refused(CMD_DUMP_CLOSE)&&!overpressure_refused(CMD_PILOT_CLOSE));
 }
 /* Launch admitted but still in its 10 s delay counts as launched. */
 reset_power();assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_ARMED)==TX_SUCCESS);
 assert(thread_comm_request_launch_sequence(42)==TX_SUCCESS);assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_ARMED)==TX_SUCCESS);
 close_dump_vent_hw();{const unsigned p=physical;run(1000.0f,5000);assert(physical==p&&!dump_hw&&!pilot_hw&&!actuator_count);}
 /* Local launch sequence active (main-thread view) also excludes. */
 reset_power();assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_ARMED)==TX_SUCCESS);
 g_launch_sequence_active=1;close_dump_vent_hw();{const unsigned p=physical;run(1000.0f,5000);assert(physical==p&&!dump_hw);}
 /* Retained launch latch after reboot still excludes. */
 reset_power();tamp.BKP28R=0x4c41554eU;tamp.BKP29R=~0x4c41554eU;latched=launch_latched=backup_ready=0;board_abort_latch_init();
 g_thread_comm_initialized=0;{TX_BYTE_POOL pool={0};assert(thread_comm_init(&pool)==TX_SUCCESS);}
 assert(board_launch_latch_is_set());close_dump_vent_hw();{const unsigned p=physical;run(1000.0f,5000);assert(physical==p&&!dump_hw);}
 /* Flight state Aborted is not a burn: relief acts. */
 reset_power();assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_ABORTED)==TX_SUCCESS);
 close_dump_vent_hw();run(900.0f,60);assert(dump_hw&&vent_hw);
 /* Exclusion is atomic in the owner: every state 0..255. */
 for(unsigned st=0;st<256;st++){
  reset_power();assert(thread_comm_set_flight_state((uint8_t)st)==TX_SUCCESS);
  const unsigned excluded=st>=VALVE_FLIGHT_STATE_LAUNCH&&st<=VALVE_FLIGHT_STATE_RECOVERY;
  assert(thread_comm_overpressure_excluded()==excluded);
  assert(thread_comm_request_overpressure_abort(1,950,0)==!excluded);
 }
}

static void sdk_retry_case(void){
 reset_power();assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_IDLE)==TX_SUCCESS);
 sdk_reject=1;run(900.0f,100);assert(dump_hw&&vent_hw&&!actuator_count&&!report_count);
 sdk_reject=0;run(900.0f,60);assert(actuator_count==3&&actuator[2]==(16U|THREAD_COMM_AUTOMATIC_TAG)&&has("Valve Board overpressure relief: 900 psi [R1]"));
}

static void invalid_sensor_case(void){
 reset_power();assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_NITROUS_FILL)==TX_SUCCESS);
 close_dump_vent_hw();
 /* Full-scale fault: raw rail, published 2000 psi, never trips. */
 for(unsigned i=0;i<5000;i++){ticks++;if(ticks%20U==0U){latest.seq++;latest.at_ms=(uint32_t)ticks;latest.raw_adc=4095U;latest.psi=2000.0f;latest.valid=overpressure_sample_valid(4095U,2000.0f,0.0f,3103.0f,2000.0f);}service_overpressure();}
 assert(!dump_hw&&!vent_hw&&!pilot_hw&&!actuator_count&&!thread_comm_get_abort());
 assert(has("Valve Board overpressure sensor invalid, raw ADC 4095 [F0]"));
 unsigned faults=0;for(unsigned i=0;i<report_count;i++)faults+=!strncmp(reports[i],"Valve Board overpressure sensor invalid",39);
 assert(faults==1);
}

int main(int argc,char**argv){
 assert(argc==2);
 if(!strcmp(argv[1],"relief"))relief_case();
 else if(!strcmp(argv[1],"ack_refused"))ack_refused_case();
 else if(!strcmp(argv[1],"pilot_abort"))pilot_abort_case();
 else if(!strcmp(argv[1],"direct_abort"))direct_abort_case();
 else if(!strcmp(argv[1],"already_aborted"))already_aborted_case();
 else if(!strcmp(argv[1],"manual_abort_window"))manual_abort_in_window_case();
 else if(!strcmp(argv[1],"launch_exclusion"))launch_exclusion_case();
 else if(!strcmp(argv[1],"sdk_retry"))sdk_retry_case();
 else if(!strcmp(argv[1],"invalid_sensor"))invalid_sensor_case();
 else return 2;
 puts("ok");return 0;
}
'''

CFLAGS = ['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-O1', '-fsanitize=address,undefined',
          '-fno-sanitize-recover=all', '-Wno-misleading-indentation']


class OverpressureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix='vb-overpressure-')
        tmp = Path(cls.tmp.name)
        (tmp / 'tx_api.h').write_text(TX_API)
        (tmp / 'module.c').write_text(MODULE_HARNESS)
        cls.module = tmp / 'module'
        subprocess.run(CFLAGS + ['-I', str(ROOT / 'Core/Inc'), str(tmp / 'module.c'),
                                 str(ROOT / 'Core/Src/overpressure.c'), '-lm', '-o', str(cls.module)], check=True)
        latch = '\n'.join(l for l in (ROOT / 'Core/Src/board_abort_latch.c').read_text().splitlines()
                          if not l.startswith('#include'))
        shared = (ROOT / 'Core/Src/thread_comm.c').read_text()
        control = (ROOT / 'Core/Src/main_thread.c').read_text()
        start = control.index('/* ---- Board tank overpressure protection')
        end = control.index('static void handle_command(')
        owner = OWNER_HARNESS.replace('LATCH_SOURCE', latch).replace('SHARED_SOURCE', shared)
        owner = owner.replace('APPLY_SOURCE', 'static void apply_automatic_valve(uint8_t command);\n' +
                              function(control, 'apply_automatic_valve'))
        owner = owner.replace('CONTROL_SOURCE', '#define TELEMETRY_ENABLED 1\n' + control[start:end])
        (tmp / 'owner.c').write_text(owner)
        cls.owner = tmp / 'owner'
        subprocess.run(CFLAGS + ['-Wno-unused-function', '-I', str(tmp), '-I', str(ROOT / 'Core/Inc'),
                                 str(tmp / 'owner.c'), str(ROOT / 'Core/Src/overpressure.c'), '-lm',
                                 '-o', str(cls.owner)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_case(self, exe, name):
        result = subprocess.run([str(exe), name], capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_thresholds_match_groundstation_defaults(self): self.run_case(self.module, 'thresholds')
    def test_debounce_ignores_single_repeated_and_stale_samples(self): self.run_case(self.module, 'debounce')
    def test_invalid_or_full_scale_sensor_never_trips(self): self.run_case(self.module, 'invalid')
    def test_launch_active_is_report_only(self): self.run_case(self.module, 'launch')
    def test_abort_pressure_opens_pilot_for_three_seconds_then_aborts(self): self.run_case(self.module, 'pilot')
    def test_latch_needs_low_fresh_pressure_acknowledgement(self): self.run_case(self.module, 'latch')
    def test_command_filter_and_report_format(self): self.run_case(self.module, 'commands')

    def test_owner_relief_closes_supplies_and_blocks_launch_until_ack(self): self.run_case(self.owner, 'relief')
    def test_owner_ack_refused_while_high_or_stale(self): self.run_case(self.owner, 'ack_refused')
    def test_owner_pilot_window_then_board_abort(self): self.run_case(self.owner, 'pilot_abort')
    def test_owner_direct_abort_pressure_from_startup(self): self.run_case(self.owner, 'direct_abort')
    def test_owner_already_aborted_never_opens_pilot(self): self.run_case(self.owner, 'already_aborted')
    def test_owner_operator_abort_supersedes_window(self): self.run_case(self.owner, 'manual_abort_window')
    def test_owner_launch_exclusion_is_atomic_and_report_only(self): self.run_case(self.owner, 'launch_exclusion')
    def test_owner_retries_rejected_publication(self): self.run_case(self.owner, 'sdk_retry')
    def test_owner_full_scale_fault_reported_once_without_action(self): self.run_case(self.owner, 'invalid_sensor')

    def test_production_wiring(self):
        control = (ROOT / 'Core/Src/main_thread.c').read_text()
        loop = function(control, 'main_thread_entry')
        # Runs every loop before the Abort branch, so latch/ack survive Abort.
        self.assertLess(loop.index('service_overpressure();'), loop.index('if (thread_comm_get_abort() != 0U)'))
        handler = function(control, 'handle_command')
        self.assertLess(handler.index('CMD_ABORT'), handler.index('overpressure_refused(cmd)'))
        self.assertLess(handler.index('overpressure_refused(cmd)'), handler.index('THREAD_COMM_AUTOMATIC_TAG'))
        acq = (ROOT / 'Core/Src/data_acq_thread.c').read_text()
        report = function(acq, 'data_acq_report_pressure')
        self.assertIn('overpressure_sample_valid(sample.raw_adc, sample.pressure', report)
        self.assertLess(report.index('data_acq_pressure_snapshot = snapshot'), report.index('SEDS_DT_FUEL_TANK_PRESSURE'))
        telemetry = function((ROOT / 'Core/Src/telemetry.c').read_text(), 'Valve_Command_handler')
        self.assertLess(telemetry.index('board_recovery_command_is_stale(pkt)) return SEDS_OK'),
                        telemetry.index('CMD_OVERPRESSURE_ACK'))
        self.assertIn('overpressure.c', (ROOT / 'CMakeLists.txt').read_text())
        # Normal launch PILOT timing is untouched.
        self.assertIn('#define LAUNCH_SEQUENCE_PILOT_OPEN_DURATION_MS 1500U', control)


if __name__ == '__main__':
    unittest.main()
