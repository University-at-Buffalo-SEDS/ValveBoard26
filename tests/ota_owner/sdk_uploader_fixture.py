from pathlib import Path
import json,re,subprocess,importlib.util,os
HERE=Path(__file__).resolve().parent;P=Path(os.environ['OTA_OWNER_WORK']).resolve()/'native';spec=importlib.util.spec_from_file_location('base',HERE/'support_fixture.py');base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
old=P/'runtime';out=P/'sdk-uploader';out.mkdir(exist_ok=True)
for n in ['tx_api.h','main.h','stm32g4xx_hal.h']:(out/n).write_bytes((old/n).read_bytes())
s=(old/'test.c').read_text().replace('int main(int argc,char**argv)','int prior_runtime_main(int argc,char**argv)')
for n,proto in [('seds_router_bind_p2p_stream_port','extern SedsResult seds_router_bind_p2p_stream_port(SedsRouter*,uint16_t,SedsResult(*)(const SedsP2pStreamEventView*,void*),void*);'),('seds_router_send_p2p_stream','extern SedsResult seds_router_send_p2p_stream(SedsRouter*,uint32_t,const uint8_t*,size_t);'),('seds_router_reset_p2p_stream','extern SedsResult seds_router_reset_p2p_stream(SedsRouter*,uint32_t);')]:s=s.replace(base.function(s,n),proto)
# Interpose only the module's real SDK calls, preserving actual C ABI library.
a=s.index('#define OTA_OP_BEGIN_DELTA');b=s.index('static uint32_t req_sequence;',a)
checked='''static SedsResult checked_send(SedsRouter*r,uint32_t i,const uint8_t*p,size_t n){assert(!physical_depth&&!state_depth);return seds_router_send_p2p_stream(r,i,p,n);}
static SedsResult checked_reset(SedsRouter*r,uint32_t i){assert(!physical_depth&&!state_depth);return seds_router_reset_p2p_stream(r,i);}
static SedsResult checked_bind(SedsRouter*r,uint16_t p,SedsResult(*cb)(const SedsP2pStreamEventView*,void*),void*u){assert(!physical_depth&&!state_depth);return seds_router_bind_p2p_stream_port(r,p,cb,u);}
#define seds_router_send_p2p_stream checked_send
#define seds_router_reset_p2p_stream checked_reset
#define seds_router_bind_p2p_stream_port checked_bind
'''
s=s[:a]+checked+s[a:b]+'''\n#undef seds_router_send_p2p_stream
#undef seds_router_reset_p2p_stream
#undef seds_router_bind_p2p_stream_port
'''+s[b:]
prefix=base.function((P/'ValveBoard26/candidate/test.c').read_text(),'main');prefix=prefix[prefix.index(' assert(argc==2);'):prefix.index(' setup_outputs();')].replace(' assert(argc==2);','(void)argc;(void)argv;')
extra=r'''
extern SedsRouter*seds_router_new(uint64_t(*)(void*),void*,const void*,size_t);
extern void seds_router_free(SedsRouter*);
extern SedsResult seds_router_set_sender_id(SedsRouter*,const char*,size_t);
extern SedsResult seds_router_configure_address(SedsRouter*,uint8_t,uint32_t);
extern int32_t seds_router_add_side_packed(SedsRouter*,const char*,size_t,SedsResult(*)(const uint8_t*,size_t,void*),void*,bool);
extern SedsResult seds_router_rx_packed_packet_to_queue_from_side(SedsRouter*,uint32_t,const uint8_t*,size_t);
extern SedsResult seds_router_announce_discovery(SedsRouter*);
extern SedsResult seds_router_process_all_queues(SedsRouter*);
extern SedsResult seds_router_open_p2p_stream_to_hostname(SedsRouter*,const char*,size_t,uint16_t,uint16_t,uint32_t*);
typedef struct{SedsRouter*peer;}Link;static Link atob,btoa;
static unsigned client_connected,client_responses;static uint8_t sdk_reply[13];
static uint64_t sdk_clock(void*u){(void)u;return ota_clock;}
static SedsResult tx_wire(const uint8_t*p,size_t n,void*u){Link*l=u;assert(!physical_depth&&!state_depth);return l->peer?seds_router_rx_packed_packet_to_queue_from_side(l->peer,0,p,n):SEDS_OK;}
static SedsResult client_event(const SedsP2pStreamEventView*e,void*u){(void)u;if(e->kind==2)client_connected++;if(e->kind==3){assert(e->payload_len==13);memcpy(sdk_reply,e->payload,13);client_responses++;}return SEDS_OK;}
static void pump(SedsRouter*a,SedsRouter*b){for(unsigned i=0;i<8;i++){(void)seds_router_process_all_queues(a);(void)seds_router_process_all_queues(b);}}
static void response(SedsRouter*a,SedsRouter*b,unsigned wanted){for(unsigned i=0;i<4;i++){ota_stream_poll();pump(a,b);}assert(client_responses==wanted&&sdk_reply[1]==0);}
int main(int argc,char**argv){
PREFIX
 setup_outputs();assert(thread_comm_bind_manual_stop_apply(manual_stop_apply_locked)==TX_SUCCESS);check_gpio_owner=true;assert(ota_stream_register_owner());assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_IDLE)==TX_SUCCESS);
 SedsRouter*a=seds_router_new(sdk_clock,NULL,NULL,0),*b=seds_router_new(sdk_clock,NULL,NULL,0);assert(a&&b);atob.peer=b;btoa.peer=a;
 assert(seds_router_set_sender_id(a,"GS",2)==SEDS_OK&&seds_router_set_sender_id(b,"VB",2)==SEDS_OK);assert(seds_router_configure_address(a,1,100)==SEDS_OK&&seds_router_configure_address(b,1,200)==SEDS_OK);
 assert(seds_router_add_side_packed(a,"b",1,tx_wire,&atob,false)==0&&seds_router_add_side_packed(b,"a",1,tx_wire,&btoa,false)==0);assert(seds_router_bind_p2p_stream_port(a,49000,client_event,NULL)==SEDS_OK);assert(ota_stream_init(b)==SEDS_OK);
 assert(seds_router_announce_discovery(a)==SEDS_OK&&seds_router_announce_discovery(b)==SEDS_OK);pump(a,b);uint32_t stream;assert(seds_router_open_p2p_stream_to_hostname(a,"VB",2,4510,49000,&stream)==SEDS_OK);pump(a,b);assert(client_connected==1&&state.connected);
 uint32_t original_server_stream=state.stream_id,extra_stream=0;assert(seds_router_open_p2p_stream_to_hostname(a,"VB",2,4510,49000,&extra_stream)==SEDS_OK);pump(a,b);assert(state.stream_id==original_server_stream&&state.connected&&work.phase==WORK_EMPTY);
 pthread_t worker;assert(!pthread_mutex_lock(&gate));io_block=true;assert(!pthread_mutex_unlock(&gate));assert(!pthread_create(&worker,NULL,writer,NULL));assert(!pthread_mutex_lock(&gate));deadline_wait(&io_entered);assert(!pthread_mutex_unlock(&gate));
 uint8_t begin_q[]={1,8,0,0,0};assert(seds_router_send_p2p_stream(a,stream,begin_q,sizeof begin_q)==SEDS_OK);memset(begin_q,255,sizeof begin_q);pump(a,b);ota_stream_poll();assert(work.phase==WORK_REQUEST&&!begins&&!client_responses);
 assert(!pthread_mutex_lock(&gate));io_release=true;assert(!pthread_cond_broadcast(&condition));assert(!pthread_mutex_unlock(&gate));assert(!pthread_join(worker,NULL));response(a,b,1);assert(begins==1&&sdk_reply[0]==0x81);
 uint8_t chunk[]={2,0,0,0,0,0,1,2,3,4,5,6,7};assert(seds_router_send_p2p_stream(a,stream,chunk,sizeof chunk)==SEDS_OK);memset(chunk,255,sizeof chunk);pump(a,b);response(a,b,2);assert(writes_ota==1&&sdk_reply[0]==0x82&&sdk_reply[5]==8);
 uint8_t fin=3;assert(seds_router_send_p2p_stream(a,stream,&fin,1)==SEDS_OK);pump(a,b);response(a,b,3);assert(finishes==1&&state.reset_requested&&!resets);
 assert(thread_comm_set_flight_state(VALVE_FLIGHT_STATE_LAUNCH)==TX_SUCCESS);ota_clock+=300;ota_stream_poll();assert(!resets&&!state.reset_requested);assert(seds_router_reset_p2p_stream(a,stream)==SEDS_OK);pump(a,b);assert(terminal.phase==CLEAN_PENDING);ota_stream_poll();assert(terminal.phase==CLEAN_NONE&&!state.connected&&!state.active);atob.peer=btoa.peer=NULL;seds_router_free(a);seds_router_free(b);
 puts("GREEN: actual public438 single-request BEGIN/CHUNK/FINISH uploader succeeds across real NVM contention; no requester retry, no SDK under storage owner, acceptedLaunch blocks delayed reset only");return 0;
}
'''
s+='\n'+extra.replace('PREFIX',prefix);(out/'test.c').write_text(s)
cmd=json.loads((old/'BUILD.json').read_text())['command'];cmd=[str(out/'test.c')if x==str(old/'test.c')else'-I'+str(out)if x=='-I'+str(old)else str(out/'test')if x==str(old/'test')else x for x in cmd]
from native_sdk import prepare_native_sdk
o=prepare_native_sdk(Path(os.environ['OTA_OWNER_SOURCE']));cmd +=[o['archive'],'-lm','-ldl'];subprocess.run(cmd,check=True)
(out/'BUILD.json').write_text(json.dumps({'command':cmd,'SDK':o,'limits':'Actual public438 packed P2P roundtrip + actual OTA module/store/epoch/reset guard. LaunchCore delta bodies fault-injected, not image cryptographic/update/flash validation.'},indent=2)+'\n')
