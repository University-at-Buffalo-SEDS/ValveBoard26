"""Actual cache: durable preparation never masquerades as control-owner success."""
from pathlib import Path
import subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
HEADERS = {
"sedsnet.h": r'''#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
typedef int SedsResult;
typedef int SedsRouter;
typedef struct { unsigned ty; const uint8_t *payload; size_t payload_len; uint64_t timestamp; } SedsPacketView;
#define SEDS_OK 0
#define SEDS_BAD_ARG -10
#define SEDS_HANDLER_ERROR -9
SedsResult seds_router_enable_network_variable(SedsRouter*,unsigned,bool,bool);
SedsResult seds_router_on_network_variable_update(SedsRouter*,unsigned,SedsResult(*)(const SedsPacketView*,void*),void*);
SedsResult seds_router_request_managed_variable(SedsRouter*,unsigned);
void seds_router_disable_managed_variable(SedsRouter*,unsigned);
SedsResult seds_router_invalidate_network_variable_cache(SedsRouter*,unsigned,bool*);
''',
"sedsnet_config.h": '#pragma once\n#define SEDS_DT_FLIGHT_STATE 444U\n',
"main.h": '#pragma once\n#include <stdint.h>\nuint32_t HAL_GetTick(void);\n',
"board_recovery_transport.h": '#pragma once\n#include "sedsnet.h"\nuint8_t board_recovery_managed_update_allowed(const SedsPacketView*);\nSedsResult board_recovery_refused(void);\n#ifndef SEDS_PERMISSION_DENIED\n#define SEDS_PERMISSION_DENIED -6\n#endif\n',
"persistent_store.h": r'''#pragma once
#include <stddef.h>
#include <stdint.h>
typedef int launchcore_persist_status_t;
#define LAUNCHCORE_PERSIST_OK 0
#define LAUNCHCORE_PERSIST_NOT_FOUND 1
#define LAUNCHCORE_PERSIST_STORAGE_ERROR 2
int persistent_store_init(void);
int persistent_store_get(uint32_t,void*,size_t*);
int persistent_store_set(uint32_t,const void*,size_t);
'''}
HARNESS = '\n#include "flight_state_cache.h"\n#include "persistent_store.h"\n#include "sedsnet_config.h"\n#include <assert.h>\n#include <string.h>\nvolatile uint32_t g_telemetry_discovery_seen;\nstatic SedsResult (*callback)(const SedsPacketView*,void*);\nstatic unsigned writes,clears;static uint8_t stored=5,allowed=1;\nstatic unsigned fail_write,fail_read,wrong_read,fail_invalidate,fail_apply;\nuint32_t HAL_GetTick(void){return 0;}\nSedsResult flight_state_cache_apply_network_update(const SedsPacketView *p){assert(p);return fail_apply?-14:0;}\nint persistent_store_init(void){return 0;}\nint persistent_store_set(uint32_t key,const void*p,size_t n){assert(key==0x46535445u&&n==1);writes++;if(fail_write)return 2;stored=*(const uint8_t*)p;return 0;}\nint persistent_store_get(uint32_t key,void*p,size_t*n){assert(key==0x46535445u&&*n==1);if(fail_read)return 2;*(uint8_t*)p=wrong_read?15:stored;return 0;}\nstatic unsigned selective_enable_calls,selective_callback_calls;\nSedsResult seds_router_enable_network_variable(SedsRouter*r,unsigned t,bool rd,bool wr){assert(++selective_enable_calls==1U);assert(r&&t==444&&rd&&!wr);return SEDS_OK;}\nSedsResult seds_router_on_network_variable_update(SedsRouter*r,unsigned t,SedsResult(*cb)(const SedsPacketView*,void*),void*u){assert(++selective_callback_calls==1U);assert(r&&t==444&&cb&&!u);callback=cb;return 0;}\nvoid seds_router_disable_managed_variable(SedsRouter*r,unsigned t){(void)r;(void)t;assert(!"whole-service disable forbidden during selective retirement");}\nSedsResult seds_router_invalidate_network_variable_cache(SedsRouter*r,unsigned t,bool*out_invalidated){assert(out_invalidated==NULL);if(fail_invalidate)return SEDS_HANDLER_ERROR;assert(r&&t==444);clears++;return SEDS_OK;}\n\nSedsResult seds_router_request_managed_variable(SedsRouter*r,unsigned t){assert(r&&t==444);return 0;}\nuint8_t board_recovery_managed_update_allowed(const SedsPacketView*p){assert(p);return allowed;}\nstatic unsigned refusals;SedsResult board_recovery_refused(void){refusals++;return SEDS_OK;}\nstatic SedsResult update(uint8_t value){SedsPacketView p={444,&value,1,100};assert(callback);return callback(&p,0);}\nint main(int argc,char**argv){assert(argc==2);SedsRouter r=0;assert(flight_state_cache_init(&r)==0&&flight_state_cache_value()==5);\n if(!strcmp(argv[1],"prepare")){assert(flight_state_cache_prepare_recovery(0)==0&&stored==0&&writes==1&&clears==1);assert(flight_state_cache_value()==5);assert(flight_state_cache_prepare_recovery(1)==0&&stored==1&&flight_state_cache_value()==5);assert(flight_state_cache_prepare_recovery(5)==0&&stored==5&&writes==3&&flight_state_cache_value()==5);assert(flight_state_cache_prepare_recovery(7)==-10&&writes==3);}\n else if(!strcmp(argv[1],"write")){fail_write=1;assert(flight_state_cache_prepare_recovery(0)==-9&&stored==5&&flight_state_cache_value()==5&&clears==0);}\n else if(!strcmp(argv[1],"read")){fail_read=1;assert(flight_state_cache_prepare_recovery(0)==-9&&stored==0&&flight_state_cache_value()==5&&clears==0);}\n else if(!strcmp(argv[1],"mismatch")){wrong_read=1;assert(flight_state_cache_prepare_recovery(0)==-9&&flight_state_cache_value()==5&&clears==0);}\n else if(!strcmp(argv[1],"service")){fail_invalidate=1;assert(flight_state_cache_prepare_recovery(0)==-9&&flight_state_cache_value()==5);}\n else if(!strcmp(argv[1],"reject")){allowed=0;assert(update(15)==0&&refusals==1&&writes==0&&stored==5&&flight_state_cache_value()==5&&clears==1&&callback);}\n else if(!strcmp(argv[1],"apply")){fail_apply=1;assert(update(3)==-14&&writes==0&&stored==5&&flight_state_cache_value()==5&&clears==1&&callback);}\n else if(!strcmp(argv[1],"retry")){fail_write=1;assert(update(3)==-9&&flight_state_cache_value()==3&&writes==1&&clears==1);assert(update(3)==-9&&writes==2&&clears==2);fail_write=0;assert(update(3)==0&&writes==3&&stored==3);}\n else if(!strcmp(argv[1],"malformed")){assert(update(255)==0&&refusals==1&&stored==5&&writes==0&&flight_state_cache_value()==5&&clears==1&&callback);}\n else { assert(0); }\n return 0;\n}\n'
# Runtime FSTATE persistence now uses the nonwaiting wrapper. The existing
# fault/retry harness retains its exact write model through that boundary.
HEADERS['persistent_store.h'] += '\n#include <stdbool.h>\nint persistent_store_try_set(uint32_t,const void*,size_t,bool*);\n'
HARNESS += '\nint persistent_store_try_set(uint32_t k,const void*p,size_t n,bool*b){*b=false;return persistent_store_set(k,p,n);}\n'
class CachePreparationTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.tmp=tempfile.TemporaryDirectory(prefix="recovery-prepare-");p=Path(cls.tmp.name)
  for name,text in HEADERS.items():(p/name).write_text(text)
  (p/'harness.c').write_text(HARNESS);cls.exe=p/'test'
  subprocess.run(['cc','-DTELEMETRY_ENABLED=1','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(p),'-I'+str(ROOT/'Core/Inc'),str(ROOT/'Core/Src/flight_state_cache.c'),str(p/'harness.c'),'-o',str(cls.exe)],check=True)
 @classmethod
 def tearDownClass(cls):cls.tmp.cleanup()
 def test_durable_prepare_preserves_actual_cache_until_owner(self):self.case('prepare')
 def test_failed_write_never_prepares(self):self.case('write')
 def test_uncertain_readback_never_claims_prepared(self):self.case('read')
 def test_mismatched_readback_never_claims_prepared(self):self.case('mismatch')
 def test_failed_sdk_retirement_never_claims_prepared(self):self.case('service')
 def test_rejected_managed_state_retires_sdk_cache(self):self.case('reject')
 def test_failed_actual_apply_retires_sdk_cache(self):self.case('apply')
 def test_identical_failed_persist_retries_durability(self):self.case('retry')
 def test_malformed_managed_state_retires_sdk_cache(self):self.case('malformed')
 def case(self,name):subprocess.run([str(self.exe),name],check=True)
