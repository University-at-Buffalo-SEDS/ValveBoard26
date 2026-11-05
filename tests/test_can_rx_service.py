from pathlib import Path
import re,json,subprocess,hashlib,os,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
def function(s,name):
 m=re.search(r'(?m)^[A-Za-z_][^\n;]*\b'+name+r'\([^;]*?\)\s*\{',s)
 assert m,name
 i=m.end();depth=1
 while depth:
  depth+=(s[i]=='{')-(s[i]=='}');i+=1
 return s[m.start():i]
def block(s,macro):
 a=s.index('#ifndef '+macro);b=s.index('#endif',a)+len('#endif');return s[a:b]
HEADER=r'''
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
#include <stdlib.h>
typedef unsigned long ULONG;
typedef struct{unsigned unused;}TX_THREAD;
typedef struct{unsigned unused;}FDCAN_HandleTypeDef;
typedef struct{uint32_t Identifier,DataLength;}FDCAN_RxHeaderTypeDef;
typedef struct{uint32_t std_id;uint8_t len,data[64];}can_bus_rx_frame_t;
#define CAN_BUS_POLLING 0
#define HAL_OK 0
#define FDCAN_RX_FIFO0 0
#define FDCAN_RX_FIFO1 1
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1
#define FDCAN_IT_RX_FIFO1_NEW_MESSAGE 2
#define BOARD_WATCHDOG_NETWORK 1
static __attribute__((unused)) TX_THREAD telemetry_thread,main_thread,data_acq_thread,safety_thread;
static __attribute__((unused)) uint32_t g_sim_main_stack_remaining,g_sim_telemetry_stack_remaining,g_sim_data_acq_stack_remaining,g_sim_safety_stack_remaining,g_sim_pressure_sd_stack_remaining;
static FDCAN_HandleTypeDef hfdcan2,*g_hfdcan=&hfdcan2;
static volatile uint16_t g_rx_head,g_rx_tail;
static volatile uint32_t g_rx_dropped_frames,g_board_health_can_rx_drops,g_fdcan_rx_count;
static uint8_t g_can_recovering;
static unsigned mock_irq,in_isr,fifo_pending;
static uint64_t now_us,next_arrival,arrivals,handled,gaps,last_id,watchdog_limit,target_arrivals,end_us;
static uint64_t main_runs,net_turn_start,max_main_gap,refill,turn_limit,callback_us,extra_us,highwater,sdk_calls,expected_id;
static int mode; /*0 finiteone-turn,1 continuousrefill,2 offered753fps*/
static jmp_buf done;
static void inject(void);
static unsigned __get_PRIMASK(void){return mock_irq;}
static void __disable_irq(void){mock_irq=1;}
static void __set_PRIMASK(unsigned v){mock_irq=v;}
static void __DMB(void){__asm__ __volatile__("" ::: "memory");}
static uint32_t HAL_GetTick(void){return (uint32_t)(now_us/1000U);}
static size_t can_bus_dlc_to_len(uint32_t n){return n;}
static unsigned HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef*h,unsigned f){assert(h==g_hfdcan&&f<=1&&in_isr);return fifo_pending;}
static int HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef*h,unsigned f,FDCAN_RxHeaderTypeDef*hdr,uint8_t*data){assert(h==g_hfdcan&&f<=1&&fifo_pending&&in_isr);fifo_pending--;hdr->Identifier=0x123;hdr->DataLength=8;memcpy(data,&arrivals,8);return HAL_OK;}
static int can_bus_recover_if_bus_off(void){return HAL_OK;}
static void can_tx_queue_service(void){assert(!mock_irq);}
static void reasm_expire_old(uint32_t n){(void)n;assert(!mock_irq);}
static void advance(uint64_t amount){
 uint64_t until=now_us+amount;
 if(mode==2){
  while(arrivals<target_arrivals && next_arrival<=until){now_us=next_arrival;inject();next_arrival=((arrivals-32U+1U)*1000000ULL)/753U;}
 }
 now_us=until;
}
static void handle_rx_frame(const can_bus_rx_frame_t*f,uint32_t stamp){
 (void)stamp;assert(!mock_irq&&!in_isr&&f->std_id==0x123&&f->len==8);
 uint64_t id;memcpy(&id,f->data,8);if(id!=expected_id)gaps++;expected_id=id+1;last_id=id;handled++;
 if(handled>=watchdog_limit)longjmp(done,2);
 advance(callback_us);if(refill)inject();
}
'''
AFTER_RING=r'''
static void inject(void){
 assert(!mock_irq&&!in_isr);arrivals++;fifo_pending=1;in_isr=1;
 HAL_FDCAN_RxFifo0Callback(&hfdcan2,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);in_isr=0;assert(!fifo_pending);
 unsigned count=(g_rx_head+CAN_BUS_RX_RING_DEPTH-g_rx_tail)%CAN_BUS_RX_RING_DEPTH;if(count>highwater)highwater=count;
}
static __attribute__((unused)) unsigned stack_remaining(const TX_THREAD*t){(void)t;return 64;}
static __attribute__((unused)) unsigned pressure_sd_stack_remaining(void){return 64;}
static __attribute__((unused)) void sample_telemetry_stack(void){}
static int ota_stream_register_owner(void){return 0;}
static void can_bus_init(FDCAN_HandleTypeDef*h){assert(h==g_hfdcan);}
static int init_telemetry_router(void){return 0;}
static void board_watchdog_progress(unsigned n){assert(n==1);net_turn_start=now_us;}
static int telemetry_poll_discovery(void){return 0;}
static int telemetry_poll_timesync(void){return 0;}
static void ota_stream_poll(void){advance(extra_us);}
static int process_all_queues_timeout(unsigned n){assert(n==TELEMETRY_QUEUE_SERVICE_BUDGET_MS);sdk_calls++;advance((uint64_t)n*1000U);return 0;}
static void telemetry_report_safety_abort(void){}
static unsigned tx_thread_sleep(unsigned ticks){
 assert(ticks==1&&!mock_irq&&!in_isr); /* actualNETsleep allows readyMAIN6 whileNET5 blocked */
 main_runs++;advance((uint64_t)ticks*1000U);
 if(now_us-net_turn_start>max_main_gap)max_main_gap=now_us-net_turn_start;
 if(mode==2){if(now_us>=end_us&&arrivals==target_arrivals&&g_rx_head==g_rx_tail)longjmp(done,1);}
 else if(main_runs>=turn_limit)longjmp(done,1);
 return 0;
}
'''
MAIN=r'''
static void reset(void){
 g_rx_head=g_rx_tail=0;g_rx_dropped_frames=g_board_health_can_rx_drops=g_fdcan_rx_count=0;
 now_us=next_arrival=arrivals=handled=gaps=last_id=main_runs=net_turn_start=max_main_gap=refill=sdk_calls=highwater=0;
 mock_irq=in_isr=fifo_pending=0;expected_id=1;watchdog_limit=2000000;turn_limit=1;callback_us=1000;extra_us=0;mode=0;g_can_recovering=0;
}
static int run(void){int why=setjmp(done);if(!why)telemetry_thread_entry(0);return why;}
static void pending_order(void){uint64_t want=last_id+1;for(uint16_t i=g_rx_tail;i!=g_rx_head;i=rb_next(i)){uint64_t id;memcpy(&id,g_rx_ring[i].data,8);assert(id==want++);}}
static void continuous(int candidate,int ab){
 reset();mode=1;refill=1;watchdog_limit=10000;for(unsigned i=0;i<32;i++)inject();int why=run();
 if(!candidate&&!ab){assert(why==2&&main_runs==0&&handled==10000);printf("continuous oldVB: schedulerneverreached after10000frames\n");return;}
 assert(why==1&&main_runs==1&&!gaps&&!g_rx_dropped_frames&&!g_board_health_can_rx_drops);pending_order();
 if(candidate){assert(handled==16&&max_main_gap==18000);}else{assert(ab&&handled==64&&max_main_gap==66000);}
 printf("continuous %s: handled=%llu main_gap_us=%llu pending32retained\n",candidate?"candidate":"oldAB",(unsigned long long)handled,(unsigned long long)max_main_gap);
}
static void controls(void){
 reset();mode=1;refill=1;callback_us=0;for(unsigned i=0;i<32;i++)inject();assert(run()==1&&handled==64);pending_order();
 reset();mode=1;refill=1;callback_us=25000;inject();assert(run()==1&&handled==1&&max_main_gap==27000);pending_order();
 reset();mode=1;refill=1;now_us=((uint64_t)UINT32_MAX-8U)*1000U;for(unsigned i=0;i<32;i++)inject();assert(run()==1&&handled==16&&max_main_gap==18000);pending_order();
 reset();assert(run()==1&&handled==0&&sdk_calls==1&&main_runs==1);
 puts("controls: frozenclock64framecap; slowoneframeoverrun; uint32wrap; emptyring: PASS");
}
static void throughput(unsigned cost,unsigned extra,int expect_loss){
 reset();mode=2;callback_us=cost;extra_us=extra;end_us=120000000ULL;target_arrivals=753U*120U+32U;
 for(unsigned i=0;i<32;i++){inject();}
 next_arrival=1000000U/753U;
 assert(run()==1);assert(arrivals==target_arrivals&&main_runs>0&&g_rx_head==g_rx_tail);
 unsigned drops=g_rx_dropped_frames+g_board_health_can_rx_drops;assert(handled+drops==arrivals);
 if(expect_loss){assert(drops>0&&gaps>0&&handled<arrivals);}else{assert(!drops&&!gaps&&handled==arrivals);}
 /* Budget checks occur between completeframes; allow HALtickrounding+oneframe. */
 assert(max_main_gap<=16000U+1000U+cost+extra+TELEMETRY_QUEUE_SERVICE_BUDGET_MS*1000U+1000U);
 printf("753fps120s cost%uus other%ums+%uus: arrivals%llu handled%llu drops%u highwater%llu main_turns%llu maxgap_us%llu %s\n",cost,TELEMETRY_QUEUE_SERVICE_BUDGET_MS,extra,(unsigned long long)arrivals,(unsigned long long)handled,drops,(unsigned long long)highwater,(unsigned long long)main_runs,(unsigned long long)max_main_gap,expect_loss?"EXPECTED_OVERBUDGET_LOSS":"LOSSLESS");
}
int main(int argc,char**argv){assert(argc==3);int candidate=atoi(argv[1]),ab=atoi(argv[2]);continuous(candidate,ab);if(!candidate)return 0;controls();for(unsigned extra=0;extra<=3000;extra+=3000)for(unsigned c=100;c<=1000;c=(c==100?500:1000)+((c==1000)?1:0))throughput(c,extra,0);throughput(2000,3000,1);return 0;}
'''

class CanRxServiceTests(unittest.TestCase):
    def test_current_rate_retains_fifo_and_bounds_network_turn(self):
        context = Path(os.environ.get("CAN_RX_TEST_CONTEXT_ROOT", str(ROOT)))
        source = (ROOT / "Core/Src/can_bus.c").read_text()
        telemetry = (context / "Core/Src/telemetry_thread.c").read_text()
        actuator = (context / "Core/Src/main_task.c").is_file()
        main = (context / ("Core/Src/main_task.c" if actuator else "Core/Src/main_thread.c")).read_text()
        self.assertRegex(main, r"MAIN_(?:TASK|THREAD)_STACK_SIZE,\s*6,\s*6,")
        self.assertRegex(telemetry, r"TELEMETRY_THREAD_STACK_SIZE,\s*5,\s*5,")
        budget = re.search(r"^#define TELEMETRY_QUEUE_SERVICE_BUDGET_MS .*$", telemetry, re.M).group(0)
        self.assertTrue(budget.endswith("1U"))
        macros = block(source, "CAN_BUS_RX_RING_DEPTH") + "\n" + block(source, "CAN_BUS_RX_SERVICE_BUDGET_MS") + "\n" + budget
        push = "rb_push" if actuator else "rb_push_drop_oldest"
        names = ["rb_next", "rb_is_full", push, "rb_pop", "can_bus_drain_rx_fifo", "HAL_FDCAN_RxFifo0Callback", "HAL_FDCAN_RxFifo1Callback", "can_bus_process_rx"]
        functions = "\n".join(function(source, name) for name in names)
        code = HEADER + "\n" + macros + "\nstatic can_bus_rx_frame_t g_rx_ring[CAN_BUS_RX_RING_DEPTH];\n" + functions + "\n" + AFTER_RING + "\n" + function(telemetry, "telemetry_thread_entry") + "\n" + MAIN
        with tempfile.TemporaryDirectory(prefix="can-rx-service-") as directory:
            c = Path(directory) / "service.c"
            exe = Path(directory) / "service"
            c.write_text(code)
            compiled = subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-fsanitize=address,undefined", "-fno-omit-frame-pointer", str(c), "-o", str(exe)], capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stdout + compiled.stderr)
            self.assertEqual(compiled.stderr, "")
            ran = subprocess.run([str(exe), "1", str(int(actuator))], capture_output=True, text=True, timeout=60)
            self.assertEqual(ran.returncode, 0, ran.stdout + ran.stderr)
            self.assertEqual(ran.stderr, "")
            self.assertEqual(ran.stdout.count("LOSSLESS"), 6)
            self.assertIn("EXPECTED_OVERBUDGET_LOSS", ran.stdout)
            self.assertIn("pending32retained", ran.stdout)
            print(ran.stdout, end="")


    def test_real_fragment_reassembly_survives_three_budget_turns(self):
        source = (ROOT / "Core/Src/can_bus.c").read_text()
        macros = block(source, "CAN_BUS_RX_RING_DEPTH") + "\n" + block(source, "CAN_BUS_RX_SERVICE_BUDGET_MS")
        frag = source[source.index("#define CAN_BUS_FRAG_MAGIC"):source.index("// =========================\n// RX ring buffer")]
        reasm = source[source.index("#ifndef CAN_BUS_REASM_SLOTS"):source.index("static HAL_StatusTypeDef can_bus_configure_filters", source.index("#ifndef CAN_BUS_REASM_SLOTS"))]
        push = "rb_push" if "static inline void rb_push(" in source else "rb_push_drop_oldest"
        code = r'''
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#define CAN_BUS_POLLING 0
#define HAL_OK 0
static unsigned irq;
static unsigned __get_PRIMASK(void){return irq;}
static void __disable_irq(void){irq=1;}
static void __set_PRIMASK(unsigned n){irq=n;}
static void __DMB(void){__asm__ __volatile__("" ::: "memory");}
static uint32_t tick;
static uint32_t HAL_GetTick(void){uint32_t n=tick;tick+=8;return n;}
static void *g_hfdcan;
static unsigned g_can_recovering;
static int can_bus_recover_if_bus_off(void){return HAL_OK;}
static void can_tx_queue_service(void){}
static uint16_t g_rx_head,g_rx_tail;
static uint32_t g_rx_dropped_frames,g_board_health_can_rx_drops;
typedef struct{uint32_t std_id;uint8_t len,data[64];}can_bus_rx_frame_t;
static uint8_t result[128];
static unsigned calls;
static void can_bus_notify_rx(const uint8_t *p,size_t n){assert(!irq&&n==128);memcpy(result,p,n);calls++;}
''' + macros + "\n" + frag + "\n" + reasm
        code += "\nstatic can_bus_rx_frame_t g_rx_ring[CAN_BUS_RX_RING_DEPTH];\n"
        for name in ["rb_next", "rb_is_full", push, "rb_pop", "reasm_reset", "reasm_get_slot", "reasm_expire_old", "bit_test", "bit_set", "handle_rx_frame", "can_bus_process_rx"]:
            code += function(source, name) + "\n"
        code += r'''
int main(void){
 uint8_t original[128];for(unsigned i=0;i<128;i++)original[i]=(uint8_t)(i^0xa5);
 for(unsigned i=0;i<3;i++){
  uint8_t frame[64]={0};can_bus_frag_hdr_t h={0};h.magic=CAN_BUS_FRAG_MAGIC;h.source=7;h.seq=4;h.frag_idx=i;h.frag_cnt=3;h.total_len=128;
  h.flags=(i==0?CAN_BUS_FRAG_F_FIRST:0)|(i==2?CAN_BUS_FRAG_F_LAST:0);memcpy(frame,&h,sizeof(h));
  unsigned n=128-i*55;if(n>55)n=55;memcpy(frame+sizeof(h),original+i*55,n);
  INGRESS(0x123,frame,sizeof(frame));
 }
 can_bus_process_rx();assert(!calls&&g_reasm[0].active&&g_reasm[0].got_count==1&&g_rx_tail==1);
 can_bus_process_rx();assert(!calls&&g_reasm[0].active&&g_reasm[0].got_count==2&&g_rx_tail==2);
 can_bus_process_rx();assert(calls==1&&!g_reasm[0].active&&g_rx_tail==g_rx_head);
 assert(!memcmp(result,original,sizeof(result))&&!g_rx_dropped_frames&&!g_board_health_can_rx_drops&&!irq);
 puts("actualfragmentreassembly: threebudgetturns, bitmapretained,128bytesexact,once,drop0");
 return 0;
}
'''.replace("INGRESS(", push + "(")
        with tempfile.TemporaryDirectory(prefix="can-rx-fragment-") as directory:
            c = Path(directory) / "fragment.c"
            exe = Path(directory) / "fragment"
            c.write_text(code)
            compiled = subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-fsanitize=address,undefined", "-fno-omit-frame-pointer", str(c), "-o", str(exe)], capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stdout + compiled.stderr)
            self.assertEqual(compiled.stderr, "")
            ran = subprocess.run([str(exe)], capture_output=True, text=True, timeout=30)
            self.assertEqual(ran.returncode, 0, ran.stdout + ran.stderr)
            self.assertEqual(ran.stderr, "")
            print(ran.stdout, end="")

if __name__ == "__main__":
    unittest.main()
