"""Actual owner output callback keeps quiescence private and evidence based."""
from pathlib import Path
import subprocess,tempfile,unittest
from test_launch_abort_policy import function
ROOT=Path(__file__).resolve().parents[1]

class BootOwnerOutputsTests(unittest.TestCase):
 def test_processed_stop_and_actual_commanded_outputs(self):
  vb=ROOT.name=='ValveBoard26'
  owner=ROOT/'Core/Src'/('main_thread.c' if vb else 'main_task.c')
  production=function(owner.read_text(),'recovery_read_outputs')
  if vb:production+='\n'+function(owner.read_text(),'current_status_read_locked')+'\n'+function(owner.read_text(),'current_status_snapshot')
  prefix=r'''
#include "board_recovery.h"
#include <assert.h>
#include <stdint.h>
static uint8_t g_aborted,g_launch_sequence_active;
typedef struct{uint32_t ODR;}Port;
static Port ports[7];
'''
  if vb:
   prefix+=r'''
enum {CMD_PILOT_OPEN=0,CMD_NORMALLY_OPEN_OPEN=1,CMD_DUMP_OPEN=2,CMD_SEQUENCE=6};
static int32_t thread_comm_status_snapshot_try(uint8_t id,uint8_t*state,uint8_t(*read)(uint8_t,uint8_t*)){return read(id,state)?0:-10;}
static uint8_t g_outputs_initialized=1;
static uint32_t compare[2]={90,90};
#define NO_SERVO_TIMER 0
#define DUMP_SERVO_TIMER 1
#define NO_SERVO_CHANNEL 0
#define DUMP_SERVO_CHANNEL 0
#define NO_SERVO_OPEN_DEGREES 90
#define NO_SERVO_CLOSE_DEGREES 0
#define NO_SERVO_ZERO_DEGREES 0
#define DUMP_SERVO_OPEN_DEGREES 90
#define DUMP_SERVO_CLOSE_DEGREES 0
#define DUMP_SERVO_ZERO_DEGREES 0
#define __HAL_TIM_GET_AUTORELOAD(timer) ((void)(timer),19999U)
#define __HAL_TIM_GET_COMPARE(timer,channel) ((void)(channel),compare[timer])
#define Solenoid_GPIO_Port (&ports[0])
#define Solenoid_Pin 1U
static unsigned servo_relative_degrees_to_us(unsigned degrees,unsigned zero){(void)zero;return degrees;}
'''
  else:
   for i,name in enumerate(['IGNITER_PIN','NITROGEN_PIN','NITROUS_PIN','BACKUP_VALVE_EN','STEPPER_SLEEP','N2_SIG','N20_SIG']):
    prefix+=f'#define {name}_GPIO_Port (&ports[{i}])\n#define {name}_Pin 1U\n'
  main=r'''
int main(void){
 uint32_t outputs=0;
 uint8_t flags=recovery_read_outputs(&outputs);
 assert((flags&BOARD_RECOVERY_SAFE_OUTPUTS)!=0);
 assert(!(flags&BOARD_RECOVERY_OWNER_STOPPED));
 g_aborted=1;flags=recovery_read_outputs(&outputs);
 assert(flags&BOARD_RECOVERY_OWNER_STOPPED);
 g_launch_sequence_active=1;flags=recovery_read_outputs(&outputs);
 assert(!(flags&BOARD_RECOVERY_OWNER_STOPPED));
 g_launch_sequence_active=0;ports[0].ODR=1;
 flags=recovery_read_outputs(&outputs);assert(!(flags&BOARD_RECOVERY_SAFE_OUTPUTS));
 return 0;
}
'''
  if vb:
   main=main.replace(' return 0;',r"""
 uint8_t state=99;
 assert(current_status_snapshot(CMD_PILOT_OPEN,&state)&&state==1);
 assert(current_status_snapshot(CMD_NORMALLY_OPEN_OPEN,&state)&&state==1);
 assert(current_status_snapshot(CMD_DUMP_OPEN,&state)&&state==1);
 /* Actual Stop hardware: pilot ODR low; servo CCRs remain at relief/open. */
 ports[0].ODR=0;
 assert(current_status_snapshot(CMD_PILOT_OPEN,&state)&&state==0);
 compare[0]=0;assert(current_status_snapshot(CMD_NORMALLY_OPEN_OPEN,&state)&&state==0);
 compare[1]=0;assert(current_status_snapshot(CMD_DUMP_OPEN,&state)&&state==0);
 g_launch_sequence_active=1;assert(current_status_snapshot(CMD_SEQUENCE,&state)&&state==1);
 compare[0]=17;assert(!current_status_snapshot(CMD_PILOT_OPEN,&state));
 compare[0]=90;assert(!current_status_snapshot(15,&state));
 g_outputs_initialized=0;assert(!current_status_snapshot(CMD_PILOT_OPEN,&state));
 return 0;
""")
  with tempfile.TemporaryDirectory(prefix='boot-owner-') as tmp:
   p=Path(tmp);(p/'test.c').write_text(prefix+production+main)
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I'+str(ROOT/'Core/Inc'),str(p/'test.c'),'-o',str(p/'test')],check=True)
   subprocess.run([str(p/'test')],check=True)

if __name__=='__main__':unittest.main()
