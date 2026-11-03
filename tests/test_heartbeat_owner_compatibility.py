from pathlib import Path
import ast,os,subprocess,tempfile,sys,re,json,hashlib
import unittest
class HeartbeatOwnerTests(unittest.TestCase):
 def test_actual_idle_active_launch_stop_owner_interleavings(self):
  w=Path(__file__).resolve().parents[2];root=Path(__file__).resolve().parents[1];b=root.name;ab=b=='ActuatorBoard26';state='ACTUATOR'if ab else'VALVE';safety=(root/'Core/Src'/('safety_task.c'if ab else'safety_thread.c')).read_text();owner=(root/'Core/Src/thread_comm.c').read_text()
  def function(source,name):
   match=re.search(r'\b'+re.escape(name)+r'\([^;{}]*\)\s*\{',source);assert match,name
   start=source.rfind('\n',0,match.start())+1;end=match.end();depth=1
   while depth:depth+=(source[end]=='{')-(source[end]=='}');end+=1
   return source[start:end]
  strings=[n.value for n in ast.walk(ast.parse((root/'tests/test_board_recovery_owner.py').read_text()))if isinstance(n,ast.Constant)and isinstance(n.value,str)];tx=next(s for s in strings if'typedef unsigned UINT;'in s);code=next(s for s in strings if'PRODUCTION'in s and'int main(void)'in s);code=code[:code.index('int main(void)')].replace('PRODUCTION',owner)
  code+=r'''
  #include <string.h>
  #include "safety_diagnostics.h"

  static uint64_t now_ms=5100,safety_heartbeat_missing_since_ms;
  static uint32_t safety_heartbeat_timeout_count;
  static unsigned clock_reads;static unsigned interleave;
  static uint64_t safety_now_ms(void){return now_ms;}
  #define SAFETY_HEARTBEAT_TIMEOUT_MS 5000ULL
  #define Continuity_GPIO_Port ((void*)0)
  #define Continuity_Pin 1U
  static uint32_t HAL_GPIO_ReadPin(void*p,unsigned pin){(void)p;(void)pin;return 1;}
  static uint64_t telemetry_event_clock_snapshot(void){return 9000;}
static uint8_t claim_with_interleave(uint32_t reason,uint64_t at,uint64_t hb,uint32_t state,uint32_t detail,uint64_t utc,uint8_t heartbeat_only){
   clock_reads++;
   if(interleave==1)assert(thread_comm_set_flight_state(STATE_FLIGHT_STATE_IDLE)==0);
   if(interleave==2){board_launch_latch_set();assert(thread_comm_set_flight_state(STATE_FLIGHT_STATE_LAUNCH)==0);}
   if(interleave==3)assert(thread_comm_request_manual_abort()==0);
   return thread_comm_request_local_abort(reason,at,hb,state,detail,utc,heartbeat_only);
  }
  #define thread_comm_request_local_abort claim_with_interleave
  REQUEST
  HEARTBEAT
  #undef thread_comm_request_local_abort
  int main(int argc,char**argv){
   assert(argc==2);TX_BYTE_POOL pool={0};assert(thread_comm_init(&pool)==0);board_recovery_bind_owner(read_outputs,prepare,finish);assert(thread_comm_set_flight_state(STATE_FLIGHT_STATE_PREFILL)==0);assert(thread_comm_note_groundstation_heartbeat(100)==0);
   if(!strcmp(argv[1],"idle")){assert(thread_comm_set_flight_state(STATE_FLIGHT_STATE_IDLE)==0);}
   if(!strcmp(argv[1],"fill_to_idle"))interleave=1;
   if(!strcmp(argv[1],"fill_to_launch"))interleave=2;
   if(!strcmp(argv[1],"manual_stop"))interleave=3;
   if(!strcmp(argv[1],"launch_latched"))board_launch_latch_set();
   if(!strcmp(argv[1],"generic_idle")){assert(thread_comm_set_flight_state(STATE_FLIGHT_STATE_IDLE)==0);assert(thread_comm_request_automatic_abort()==1&&thread_comm_get_abort());return 0;}
   if(!strcmp(argv[1],"manual_idle")){assert(thread_comm_set_flight_state(STATE_FLIGHT_STATE_IDLE)==0);assert(thread_comm_request_manual_abort()==0&&thread_comm_get_abort());return 0;}
   safety_check_heartbeat();
   if(!strcmp(argv[1],"active")){assert(thread_comm_get_abort()&&g_safety_first_abort.reason==SAFETY_ABORT_HEARTBEAT&&clock_reads==1);}
   else if(!strcmp(argv[1],"manual_stop")){assert(thread_comm_get_abort()&&g_thread_comm_recovery_epoch==2&&!g_safety_first_abort.reason);}
   else {assert(!thread_comm_get_abort()&&!g_safety_first_abort.reason);if(!strcmp(argv[1],"idle"))assert(!safety_heartbeat_timeout_count&&!clock_reads);}
   return 0;
  }
  '''
  # The AB has no continuity peripheral read in this heartbeat path.
  if ab:code=code.replace('static uint32_t HAL_GPIO_ReadPin(void*p,unsigned pin){(void)p;(void)pin;return 1;}','')
  code=code.replace('ACTUAL_CLAIM','thread_comm_request_heartbeat_abort'if'thread_comm_request_heartbeat_abort(void)'in owner else'thread_comm_request_automatic_abort').replace('STATE_FLIGHT_STATE_',state+'_FLIGHT_STATE_')
  code=re.sub(r'(?m)^[ \t]*REQUEST[ \t]*$',lambda _:function(safety,'safety_request_abort')if not ab else'',code)
  code=re.sub(r'(?m)^[ \t]*HEARTBEAT[ \t]*$',lambda _:function(safety,'safety_check_heartbeat'),code)
  cases=['idle','active','fill_to_idle','fill_to_launch','launch_latched','manual_stop','generic_idle','manual_idle'];rows=[]
  with tempfile.TemporaryDirectory(prefix='heartbeat-actual-owner-')as tmp:
   p=Path(tmp);(p/'tx_api.h').write_text(tx);(p/'test.c').write_text(code);exe=p/'test';args=['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(p),'-I'+str(root/'Core/Inc'),str(p/'test.c'),'-o',str(exe)];r=subprocess.run(args);assert r.returncode==0
   for case in cases:
    r=subprocess.run([str(exe),case]);rows.append({'case':case,'exit':r.returncode});print(b,case,r.returncode,flush=True)
  print(json.dumps({'source':str(root),'owner_sha256':hashlib.sha256(owner.encode()).hexdigest(),'safety_sha256':hashlib.sha256(safety.encode()).hexdigest(),'rows':rows}));self.assertTrue(all(r['exit']==0 for r in rows))

if __name__=='__main__':unittest.main()
