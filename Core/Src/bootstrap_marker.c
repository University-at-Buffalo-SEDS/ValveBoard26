#include "bootstrap_marker.h"
#include "board_recovery.h"
#include <string.h>
bool bootstrap_marker_decode(const uint8_t*p,size_t n,BootstrapMarker*out){
 if((n!=32U&&n!=40U)||p[1]>1U||p[2]>BOARD_RECOVERY_MAX_STATE||p[3]>1U)return false;
 if((p[0]==1U&&(n!=32U||p[3]))||(p[0]==2U&&n!=40U)||(p[0]!=1U&&p[0]!=2U))return false;
 BootstrapMarker m={.state=p[1],.desired=p[2],.version=p[0],.kind=p[3],.first_epoch=bootstrap_read32(p+4),.first_time=bootstrap_read64(p+8),.first_nonce=bootstrap_read64(p+16),.first_issuer=bootstrap_read64(p+24),.first_expected_generation=p[0]==1U?UINT64_MAX:bootstrap_read64(p+32)};
 if(!m.first_epoch||m.first_epoch==UINT32_MAX||!m.first_time||m.first_time==UINT64_MAX||!m.first_nonce||!m.first_issuer||!bootstrap_marker_size(&m))return false;
 *out=m;return true;
}

size_t bootstrap_marker_size(const BootstrapMarker*m){
 if(m->kind>1U||!m->first_expected_generation)return 0;
 if(m->version==1)return m->kind==0&&m->first_expected_generation==UINT64_MAX?32U:0;
 return m->version==2?40U:0;
}
void bootstrap_marker_encode(uint8_t*p,const BootstrapMarker*m){
 size_t size=bootstrap_marker_size(m);if(!size)return;
 p[0]=m->version;p[1]=m->state;p[2]=m->desired;p[3]=m->kind;bootstrap_write32(p+4,m->first_epoch);bootstrap_write64(p+8,m->first_time);bootstrap_write64(p+16,m->first_nonce);bootstrap_write64(p+24,m->first_issuer);if(size==40U)bootstrap_write64(p+32,m->first_expected_generation);
}
/* Fixed-width unsigned fields cover every byte. These assertions forbid
 * padding or a future layout change from silently weakening field equality. */
_Static_assert(sizeof(BootstrapMarker)==40U&&offsetof(BootstrapMarker,state)==0U&&offsetof(BootstrapMarker,desired)==1U&&offsetof(BootstrapMarker,version)==2U&&offsetof(BootstrapMarker,kind)==3U&&offsetof(BootstrapMarker,first_epoch)==4U&&offsetof(BootstrapMarker,first_time)==8U&&offsetof(BootstrapMarker,first_nonce)==16U&&offsetof(BootstrapMarker,first_issuer)==24U&&offsetof(BootstrapMarker,first_expected_generation)==32U,"marker equality requires an exact no-padding layout");
bool bootstrap_marker_same(const BootstrapMarker*a,const BootstrapMarker*b){return memcmp(a,b,sizeof *a)==0;}

/* Shared field mapping only; callers retain every independent admission and
 * version/kind decision. Original request and source stay immutable. */
void bootstrap_marker_request(BootstrapMarker*out,const BoardRecoveryRequest*r,uint64_t source,uint8_t version,uint8_t kind){
 *out=(BootstrapMarker){.state=1,.desired=r->desired_state,.version=version,.kind=kind,.first_epoch=r->expected_abort_epoch,.first_time=source,.first_nonce=r->nonce,.first_issuer=r->issuer_session,.first_expected_generation=r->expected_boot_generation};
}

/* Pure request-time validation shared by storage admission and QUERY arming. */
bool bootstrap_request_fresh(uint64_t source,uint64_t now){return source&&source!=UINT64_MAX&&now&&now!=UINT64_MAX&&(now>=source?now-source<=15000U:source-now<=1000U);}
