"""Recovery receiver requires authenticated logical owner and singleton target."""
from pathlib import Path
import os,subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class RecoveryTransportTests(unittest.TestCase):
 def test_actual_transport_target_time_epoch_and_response(self):
  sdk=Path(os.environ['SEDSNET_TEST_SOURCE'])
  with tempfile.TemporaryDirectory()as directory:
   w=Path(directory)
   source=r"""
#include <assert.h>
#include <string.h>
#include "board_recovery_transport.h"
#include "board_recovery.h"
static unsigned floors,applies,statuses,prepares,cancels;static uint8_t prepared,cache_ok=1;static uint8_t floor_ok=1,complete,floor_active,allows_floor=1;static uint64_t now=10000;static uint32_t primary=42;static uint8_t response[40];
uint64_t board_recovery_boot_query_generation(void){return 9;}
uint64_t board_recovery_boot_current(void){return 9;}
uint8_t board_recovery_floor_active(void){return floor_active;}
uint8_t board_recovery_floor_allows_recovery_time(uint64_t issuer,uint64_t time){(void)issuer;(void)time;return allows_floor;}
uint8_t board_recovery_floor_is_healthy(void){return 1;}
uint8_t board_recovery_command_time_allowed(uint64_t time){return time>500;}
uint8_t board_recovery_floor_commit(uint64_t issuer,uint64_t time){assert(issuer==77&&time==10000);floors++;return floor_ok;}
int32_t board_recovery_begin_prepare(uint8_t desired,uint32_t epoch){assert(desired==0&&epoch==10);prepares++;return 0;}
void board_recovery_cancel_prepare(uint8_t desired,uint32_t epoch){assert(desired==0&&epoch==10);cancels++;}
int32_t flight_state_cache_prepare_recovery(uint8_t desired){assert(desired==0&&floors>0);prepared=1;return cache_ok?0:-9;}
int32_t board_recovery_snapshot(BoardRecoverySnapshot*out){*out=(BoardRecoverySnapshot){.actual_state=0,.flags=31,.abort_epoch=10,.actual_outputs_mask=6};return 0;}
int32_t board_recovery_apply_safe(uint8_t desired,uint32_t epoch,BoardRecoverySnapshot*out){assert(desired==0&&epoch==10&&floors>0&&prepared);applies++;if(complete){out->abort_epoch=11;out->flags=30;return 0;}return -14;}
SedsResult seds_router_resolve_hostname_address(SedsRouter*r,const char*n,size_t length,uint32_t*out){(void)r;assert(length==2&&!memcmp(n,"GS",2));*out=primary;return SEDS_OK;}
SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){(void)r;*out=now;return SEDS_OK;}
SedsResult seds_router_log_queue_typed(SedsRouter*r,SedsDataType type,const void*data,size_t count,size_t size,SedsElemKind kind){(void)r;assert(type==201&&count==40&&size==1&&kind==SEDS_EK_UNSIGNED);memcpy(response,data,40);statuses++;return SEDS_OK;}
int main(void){
 int opaque=0;SedsRouter*r=(SedsRouter*)&opaque;uint32_t endpoints[2]={11,22};
 uint8_t body[32]={1,1,0,0};body[4]=1;body[12]=9;body[20]=10;body[24]=77;
 SedsPacketView p={.ty=200,.sender="GS",.sender_len=2,.endpoints=endpoints,.num_endpoints=1,.timestamp=10000,.payload=body,.payload_len=32};
 assert(board_recovery_handle(r,&p,11,123)==SEDS_OK&&response[1]==1&&floors==1&&applies==1);
 assert(board_recovery_handle(r,&p,11,123)==SEDS_OK&&floors==1&&applies==2);complete=1;
 assert(board_recovery_handle(r,&p,11,123)==SEDS_OK&&response[1]==0&&response[20]==11&&floors==1);
 body[2]=1;assert(board_recovery_handle(r,&p,11,123)==SEDS_OK&&response[1]==6&&applies==3);body[2]=0;
 unsigned oldstatus=statuses;p.num_endpoints=2;assert(board_recovery_handle(r,&p,11,123)==SEDS_BAD_ARG&&statuses==oldstatus);p.num_endpoints=1;
 assert(board_recovery_handle(r,&p,22,123)==SEDS_BAD_ARG);p.sender="RF";assert(board_recovery_handle(r,&p,11,123)==SEDS_PERMISSION_DENIED);
 p.sender="@addr:42";p.sender_len=8;assert(board_recovery_handle(r,&p,11,123)==SEDS_OK);primary=43;assert(board_recovery_handle(r,&p,11,123)==SEDS_PERMISSION_DENIED);primary=42;
 p.sender="@addr:4294967296";p.sender_len=16;assert(board_recovery_handle(r,&p,11,123)==SEDS_PERMISSION_DENIED);
 p.sender="GS";p.sender_len=2;body[4]=2;body[12]=8;assert(board_recovery_handle(r,&p,11,123)==SEDS_OK&&response[1]==4&&floors==1);body[12]=9;
 p.timestamp=0;assert(board_recovery_handle(r,&p,11,123)==SEDS_OK&&response[1]==4&&floors==1);p.timestamp=10000;now=16000;assert(board_recovery_handle(r,&p,11,123)==SEDS_OK&&response[1]==4);now=10000;
 floor_ok=0;assert(board_recovery_handle(r,&p,11,123)==SEDS_OK&&response[1]==7&&floors==2);floor_ok=1;assert(cancels==1);assert(board_recovery_handle(r,&p,11,123)==SEDS_OK&&response[1]==7&&floors==2&&cancels==1);
 body[1]=0;body[4]=3;assert(board_recovery_handle(r,&p,11,123)==SEDS_OK&&response[1]==0&&floors==2);
 body[1]=1;body[4]=5;cache_ok=0;unsigned oldapplies=applies,oldprepares=prepares;assert(board_recovery_handle(r,&p,11,123)==SEDS_OK&&response[1]==7&&floors==3&&applies==oldapplies&&prepares==oldprepares+1);cache_ok=1;assert(board_recovery_handle(r,&p,11,123)==SEDS_OK&&response[1]==7&&floors==3&&applies==oldapplies&&prepares==oldprepares+1);
 body[1]=1;body[4]=4;allows_floor=0;assert(board_recovery_handle(r,&p,11,123)==SEDS_OK&&response[1]==4&&floors==3);
 p.timestamp=500;assert(board_recovery_command_is_stale(&p));p.timestamp=501;assert(!board_recovery_automatic_is_stale(&p));assert(board_recovery_automatic_is_stale(0));
 floor_active=1;assert(board_recovery_command_is_stale(&p));board_recovery_transport_set_router(r);
 p.timestamp=10000;assert(!board_recovery_command_is_stale(&p));p.timestamp=12000;assert(board_recovery_command_is_stale(&p));
 board_recovery_transport_set_router(0);p.timestamp=10000;assert(board_recovery_command_is_stale(&p));
 return 0;
}
"""
   (w/'test.c').write_text(source);exe=w/'test'
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(ROOT/'Core/Inc'),'-I'+str(sdk/'C-Headers'),str(w/'test.c'),str(ROOT/'Core/Src/board_recovery_wire.c'),str(ROOT/'Core/Src/board_recovery_transport.c'),'-o',str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
