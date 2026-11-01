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
  with tempfile.TemporaryDirectory(prefix='boot-owner-') as tmp:
   p=Path(tmp);(p/'test.c').write_text(prefix+production+main)
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I'+str(ROOT/'Core/Inc'),str(p/'test.c'),'-o',str(p/'test')],check=True)
   subprocess.run([str(p/'test')],check=True)

if __name__=='__main__':unittest.main()
