"""An applied FLIGHT_STATE is acknowledged; durability is retried from the poll.

When the callback fails after the value is applied (storage not ready, or
persistent_store_try_set busy or failing), the SDK withholds the end-to-end
receipt: the station's ordered FLIGHT_STATE stream stalls behind that write
and newer writes, Aborted among them, queue behind it. Once applied (the
valves already follow it), the write is acknowledged and its persistence is
retried from flight_state_cache_poll at most once per second.

Builds the actual flight_state_cache.c with the test_recovery_cache_prepare
facade, plus a controllable clock and busy store.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

from test_recovery_cache_prepare import HARNESS as PREPARE_HARNESS, HEADERS

ROOT = Path(__file__).resolve().parents[1]

PRELUDE = PREPARE_HARNESS[:PREPARE_HARNESS.index("int main(int argc,char**argv){")]
assert PRELUDE.count("uint32_t HAL_GetTick(void){return 0;}") == 1
PRELUDE = PRELUDE.replace("uint32_t HAL_GetTick(void){return 0;}",
                          "static uint32_t tick;uint32_t HAL_GetTick(void){return tick;}")

CASES = r'''
static int store_busy;
int persistent_store_try_set(uint32_t k,const void*p,size_t n,bool*b){
 *b=store_busy!=0;if(store_busy){writes++;return LAUNCHCORE_PERSIST_STORAGE_ERROR;}
 return persistent_store_set(k,p,n);
}
extern volatile uint32_t g_flight_state_cache_errors;
static SedsRouter poll_router=1;
static void poll_at(uint32_t at){tick=at;assert(flight_state_cache_poll(&poll_router)==SEDS_OK);}
int main(int argc,char**argv){
 assert(argc==2);SedsRouter r=0;
 assert(flight_state_cache_init(&r)==0&&flight_state_cache_value()==5);
 tick=10000;
 if(!strcmp(argv[1],"write_failure")){
  fail_write=1;
  /* Applied and acknowledged; durability is retried, not the receipt. */
  assert(update(3)==SEDS_OK&&flight_state_cache_value()==3&&writes==1&&stored==5);
  poll_at(10999);assert(writes==1);                   /* at most once per second */
  poll_at(11000);assert(writes==2&&stored==5);
  fail_write=0;poll_at(11999);assert(writes==2);
  poll_at(12000);assert(writes==3&&stored==3);
  poll_at(30000);assert(writes==3);                   /* nothing pending */
  assert(update(3)==SEDS_OK&&writes==3&&clears==0);    /* the applied cache is kept */
  assert(g_flight_state_cache_errors==2);
 }else if(!strcmp(argv[1],"busy")){
  store_busy=1;
  assert(update(7)==SEDS_OK&&flight_state_cache_value()==7&&stored==5);
  const uint32_t errors=g_flight_state_cache_errors;
  poll_at(11000);assert(stored==5&&g_flight_state_cache_errors==errors); /* busy is no fault */
  store_busy=0;poll_at(12000);assert(stored==7);
 }else if(!strcmp(argv[1],"newer_value")){
  store_busy=1;assert(update(1)==SEDS_OK&&update(2)==SEDS_OK);
  store_busy=0;poll_at(11000);assert(stored==2);
 }else if(!strcmp(argv[1],"same_value_while_pending")){
  fail_write=1;assert(update(3)==SEDS_OK);
  fail_write=0;assert(update(3)==SEDS_OK&&stored==3);  /* a retry of the write persists */
  const unsigned w=writes;poll_at(20000);assert(writes==w);
 }else if(!strcmp(argv[1],"prepare_clears_pending")){
  /* A durable recovery preparation must never be overwritten by a stale
   * pending FLIGHT_STATE retry. */
  fail_write=1;assert(update(3)==SEDS_OK&&stored==5);
  fail_write=0;assert(flight_state_cache_prepare_recovery(0)==0&&stored==0);
  const unsigned w=writes;poll_at(20000);assert(writes==w&&stored==0);
 }else assert(!"unknown case");
 return 0;
}
'''


class FlightStatePersistenceRetryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="vb-fstate-retry-")
        p = Path(cls.tmp.name)
        for name, text in HEADERS.items():
            (p / name).write_text(text)
        (p / "harness.c").write_text(PRELUDE + CASES)
        cls.exe = p / "test"
        subprocess.run(["cc", "-DTELEMETRY_ENABLED=1", "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-Wno-unused-function", "-Wno-unused-variable",
                        "-fsanitize=address,undefined", "-I" + str(p), "-I" + str(ROOT / "Core/Inc"),
                        str(ROOT / "Core/Src/flight_state_cache.c"), str(p / "harness.c"),
                        "-o", str(cls.exe)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def case(self, name):
        subprocess.run([str(self.exe), name], check=True)

    def test_failed_write_is_acknowledged_and_retried_once_per_second(self):
        self.case("write_failure")

    def test_busy_store_is_acknowledged_and_retried_without_fault(self):
        self.case("busy")

    def test_pending_retry_persists_the_latest_applied_value(self):
        self.case("newer_value")

    def test_identical_write_while_pending_persists_it(self):
        self.case("same_value_while_pending")

    def test_recovery_preparation_cancels_the_pending_retry(self):
        self.case("prepare_clears_pending")


if __name__ == "__main__":
    unittest.main()
