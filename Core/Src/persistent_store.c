#include "metadata_atomic.h"
#include "persistent_store_health.h"
#include "bootstrap_marker.h"
#include "persistent_store.h"
#include "launchcore/storage.h"
#include "tx_api.h"
#include "main.h"
#include <stdbool.h>
#include <stdatomic.h>
#include <string.h>
extern const launchcore_storage_driver_t launchcore_board_storage_driver;
static TX_MUTEX mutex;
static uint8_t phase; /* 0 unavailable, 1 explicit exclusive boot, 2 runtime */
static _Atomic bool controls_ready;
static _Atomic bool bootstrap_hold, recovery_metadata_ready, storage_faulted;
/*0revoked,1strict COMPLETE ticket awaiting owned publication,2published.
 * Every readiness/hold/integrity invalidation revokes FIRST; only the checked
 * owner publishes2 LAST. Exclusive boot may restore full strict COMPLETE. */
static _Atomic uint8_t bootstrap_complete_verified;
volatile uint32_t g_persistent_store_errors;

static __attribute__((noinline,noclone)) void revoke_controls(void){bootstrap_complete_verified=0U;metadata_store_bool(&controls_ready, (false));}
static void transient_fault(void) {
 revoke_controls();metadata_store_bool(&recovery_metadata_ready, (false));
 }
static void control_fault(void){transient_fault();metadata_store_bool(&storage_faulted, (true));}
static bool ownership_result(bool success)
{
    if (!success) {
        control_fault();
        ++g_persistent_store_errors;
    }
    return success;
}
bool persistent_store_enter(void)
{
    if(metadata_load_bool(&storage_faulted))return false;
    if (__get_IPSR() != 0U) return ownership_result(false);
    if (phase == 1U) return ownership_result(tx_thread_identify() == TX_NULL);
    if(phase!=2U||tx_thread_identify()==TX_NULL)return ownership_result(false);
    UINT status=tx_mutex_get(&mutex,TX_WAIT_FOREVER);
    if(status==TX_SUCCESS)return true;
    if(status==TX_NOT_AVAILABLE||status==TX_WAIT_ABORTED){persistent_store_note_failure();return false;}
    return ownership_result(false);
}
bool persistent_store_try_enter(bool*busy)
{
 if(!busy)return ownership_result(false);
 *busy=false;
 if(metadata_load_bool(&storage_faulted))return false;
 if(__get_IPSR()!=0U||phase!=2U||tx_thread_identify()==TX_NULL)return ownership_result(false);
 UINT status=tx_mutex_get(&mutex,TX_NO_WAIT);
 if(status==TX_SUCCESS)return true;
 if(status==TX_NOT_AVAILABLE){*busy=true;return false;}
 if(status==TX_WAIT_ABORTED){persistent_store_note_failure();return false;}
 return ownership_result(false);
}
bool persistent_store_leave(void)
{
    if (__get_IPSR() != 0U) return ownership_result(false);
    if (phase == 1U) return ownership_result(tx_thread_identify() == TX_NULL);
    return ownership_result(phase == 2U && tx_thread_identify() != TX_NULL &&
        tx_mutex_put(&mutex) == TX_SUCCESS);
}
static launchcore_persist_status_t result(launchcore_persist_status_t status)
{
    const bool released = persistent_store_leave();
    if (!released && status == LAUNCHCORE_PERSIST_OK) status = LAUNCHCORE_PERSIST_STORAGE_ERROR;
    if (status == LAUNCHCORE_PERSIST_CORRUPT) {control_fault();++g_persistent_store_errors;}
    else if (status == LAUNCHCORE_PERSIST_STORAGE_ERROR) persistent_store_note_failure();
    return status;
}
static launchcore_persist_status_t inspect(void)
{
    launchcore_storage_set_driver(&launchcore_board_storage_driver);
    if (launchcore_storage_init() != LAUNCHCORE_STORAGE_OK) return LAUNCHCORE_PERSIST_STORAGE_ERROR;
    launchcore_persist_strict_info_t info;
    launchcore_persist_status_t status = launchcore_persist_probe_strict(&info);
    if (status == LAUNCHCORE_PERSIST_OK &&
        info.health != LAUNCHCORE_PERSIST_HEALTH_HEALTHY &&
        info.health != LAUNCHCORE_PERSIST_HEALTH_VIRGIN) {
        revoke_controls();
        return LAUNCHCORE_PERSIST_CORRUPT;
    }
    return status;
}
static launchcore_persist_status_t get_checked(uint32_t key,void*value,size_t*size,launchcore_persist_strict_info_t*info) {
 launchcore_persist_status_t status=launchcore_persist_get_strict(key,value,size,info);
 if(status==LAUNCHCORE_PERSIST_CORRUPT)control_fault();else if(status==LAUNCHCORE_PERSIST_STORAGE_ERROR)transient_fault();
 return status;
}
typedef struct {uint64_t generation;const uint8_t*floor;const uint8_t*marker;size_t marker_size;} ControlExpectation;
/* Same checked physical owner spans all three full strict reads. Optional
 * expectation selects exact grouped comparison, never a cheaper key/health
 * path; generic boot/readiness still validates every critical key and codec. */
static bool validate_controls_expected(const ControlExpectation*expected){
 uint8_t bytes[BOOTSTRAP_MARKER_SIZE];size_t size=8U;launchcore_persist_strict_info_t info;
 transient_fault();
 if(get_checked(UINT32_C(0x42524745),bytes,&size,&info)!=LAUNCHCORE_PERSIST_OK||size!=8U)return false;
 uint64_t decoded_generation=bootstrap_read64(bytes);
 if(decoded_generation==0U||decoded_generation==UINT64_MAX||(expected&&decoded_generation!=expected->generation))return false;
 size=16U;
 const launchcore_persist_status_t floor_status=get_checked(UINT32_C(0x4252464c),bytes,&size,&info);
 const bool no_prior_recovery=floor_status==LAUNCHCORE_PERSIST_NOT_FOUND&&
     info.health==LAUNCHCORE_PERSIST_HEALTH_HEALTHY&&info.record==LAUNCHCORE_PERSIST_RECORD_ABSENT;
 if(no_prior_recovery){if(expected)return false;}
 else{
  if(floor_status!=LAUNCHCORE_PERSIST_OK||size!=16U)return false;
  uint64_t decoded_issuer=bootstrap_read64(bytes),decoded_time=bootstrap_read64(bytes+8U);
  if(decoded_issuer==0U||decoded_time==0U||decoded_time==UINT64_MAX||(expected&&memcmp(bytes,expected->floor,16U)))return false;
 }
 bool cold_complete=false;
 size=sizeof bytes;launchcore_persist_status_t status=get_checked(BOOTSTRAP_MARKER_KEY,bytes,&size,&info);
 if(status==LAUNCHCORE_PERSIST_NOT_FOUND&&info.health==LAUNCHCORE_PERSIST_HEALTH_HEALTHY&&info.record==LAUNCHCORE_PERSIST_RECORD_ABSENT){if(expected)return false;}
 else{
  BootstrapMarker marker;if(status!=LAUNCHCORE_PERSIST_OK||!bootstrap_marker_decode(bytes,size,&marker)||(expected&&(size!=expected->marker_size||memcmp(bytes,expected->marker,size))))return false;
  if(marker.state)metadata_store_bool(&bootstrap_hold, (true));else cold_complete=expected==NULL&&phase==1U;
 }
 metadata_store_bool(&recovery_metadata_ready, (true));metadata_store_bool(&controls_ready, (!metadata_load_bool(&bootstrap_hold)));
 if(cold_complete&&metadata_load_bool(&controls_ready)&&!metadata_load_bool(&storage_faulted))bootstrap_complete_verified=2U;
 return true;
}
static void validate_controls(void){if(!validate_controls_expected(NULL)){control_fault();++g_persistent_store_errors;}}
bool persistent_store_controls_ready(void) { return metadata_load_bool(&controls_ready)&&!metadata_load_bool(&storage_faulted); }

/* Normal exclusive boot establishes bookkeeping, not application recovery.
 * Strict ABSENT permits first initialization; tombstones, ambiguous banks and
 * corrupt records are never repaired or reclassified as new storage. */
static launchcore_persist_status_t initialize_boot_identity(void)
{
    if (!persistent_store_enter()) return LAUNCHCORE_PERSIST_STORAGE_ERROR;
    uint8_t identity[8]; size_t size=sizeof identity;
    launchcore_persist_strict_info_t info;
    launchcore_persist_status_t status=launchcore_persist_get_strict(UINT32_C(0x42524745),identity,&size,&info);
    if(status==LAUNCHCORE_PERSIST_NOT_FOUND&&info.record==LAUNCHCORE_PERSIST_RECORD_ABSENT&&
       (info.health==LAUNCHCORE_PERSIST_HEALTH_VIRGIN||info.health==LAUNCHCORE_PERSIST_HEALTH_HEALTHY)) {
        const bool virgin=info.health==LAUNCHCORE_PERSIST_HEALTH_VIRGIN;
        bootstrap_write64(identity,1U);
        status=virgin?launchcore_persist_provision_virgin_strict(UINT32_C(0x42524745),identity,sizeof identity,&info):
            launchcore_persist_set_strict(UINT32_C(0x42524745),identity,sizeof identity,&info);
        if(status==LAUNCHCORE_PERSIST_OK) {
            size=sizeof identity;
            status=launchcore_persist_get_strict(UINT32_C(0x42524745),identity,&size,&info);
        }
    }
    if(status==LAUNCHCORE_PERSIST_OK&&(size!=sizeof identity||bootstrap_read64(identity)==0U||bootstrap_read64(identity)==UINT64_MAX))
        status=LAUNCHCORE_PERSIST_CORRUPT;
    if(status==LAUNCHCORE_PERSIST_NOT_FOUND) status=LAUNCHCORE_PERSIST_CORRUPT;
    if(status!=LAUNCHCORE_PERSIST_OK){control_fault();++g_persistent_store_errors;}
    return result(status);
}

/* MAIN is the only caller, before any task/SDK/OTA owner exists. No ISR calls
 * any storage API. End this phase before creating application tasks. */
launchcore_persist_status_t persistent_store_boot_begin(void)
{
    if (phase != 0U || __get_IPSR() != 0U || tx_thread_identify() != TX_NULL)
        return LAUNCHCORE_PERSIST_STORAGE_ERROR;
    phase = 1U;
    launchcore_persist_status_t status = persistent_store_init();
    if (status == LAUNCHCORE_PERSIST_OK) status=initialize_boot_identity();
    if (status == LAUNCHCORE_PERSIST_OK) validate_controls();
    return status;
}
UINT persistent_store_runtime_start(void)
{
    if(phase!=1U)bootstrap_complete_verified=0U;
    metadata_store_bool(&controls_ready, (metadata_load_bool(&controls_ready) && phase == 1U));
    phase = 0U;
    if (__get_IPSR() != 0U) {revoke_controls();return TX_CALLER_ERROR;}
    UINT status = tx_mutex_create(&mutex, "persist", TX_INHERIT);
    if (status == TX_SUCCESS) phase = 2U;
    else { revoke_controls(); ++g_persistent_store_errors; }
    return status;
}
launchcore_persist_status_t persistent_store_init(void)
{
    if (!persistent_store_enter()) return LAUNCHCORE_PERSIST_STORAGE_ERROR;
    return result(inspect());
}
launchcore_persist_status_t persistent_store_get(uint32_t key, void *value, size_t *size)
{
    if (!persistent_store_enter()) return LAUNCHCORE_PERSIST_STORAGE_ERROR;
    launchcore_persist_strict_info_t info;
    launchcore_persist_status_t status = launchcore_persist_get_strict(key, value, size, &info);
    const bool ordinary_absent = status == LAUNCHCORE_PERSIST_NOT_FOUND &&
        info.health == LAUNCHCORE_PERSIST_HEALTH_HEALTHY && info.record == LAUNCHCORE_PERSIST_RECORD_ABSENT &&
        (key == UINT32_C(0x4252464c) || key == BOOTSTRAP_MARKER_KEY);
    if ((key == UINT32_C(0x42524745) || key == UINT32_C(0x4252464c) || key == BOOTSTRAP_MARKER_KEY) &&
        status != LAUNCHCORE_PERSIST_OK && !ordinary_absent) revoke_controls();
    return result(status);
}
static launchcore_persist_status_t set_owned(uint32_t key,const void*value,size_t size)
{
    launchcore_persist_strict_info_t info;
    const bool critical=key==UINT32_C(0x42524745)||key==UINT32_C(0x4252464c)||key==BOOTSTRAP_MARKER_KEY;
    if(critical)bootstrap_complete_verified=0U; /* BEFORE uncertain effects. */
    launchcore_persist_status_t status = launchcore_persist_set_strict(key, value, size, &info);
    if (status == LAUNCHCORE_PERSIST_OK && critical)
        validate_controls(); /* Does not clear the independent Stop/reset/launch inhibit. */
    return result(status);
}
launchcore_persist_status_t persistent_store_set(uint32_t key,const void*value,size_t size){
 if(!persistent_store_enter())return LAUNCHCORE_PERSIST_STORAGE_ERROR;
 return set_owned(key,value,size);
}
launchcore_persist_status_t persistent_store_try_set(uint32_t key,const void*value,size_t size,bool*busy){
 if(!persistent_store_try_enter(busy))return LAUNCHCORE_PERSIST_STORAGE_ERROR;
 return set_owned(key,value,size);
}

bool persistent_store_recovery_ready(void){return !metadata_load_bool(&storage_faulted)&&(metadata_load_bool(&controls_ready)||(metadata_load_bool(&bootstrap_hold)&&metadata_load_bool(&recovery_metadata_ready)));}

void persistent_store_note_failure(void) { transient_fault();++g_persistent_store_errors; }

void persistent_store_hold_bootstrap(void) { bootstrap_complete_verified=0U;metadata_store_bool(&bootstrap_hold, (true));metadata_store_bool(&controls_ready, (false)); }
bool persistent_store_bootstrap_pending(void) { return metadata_load_bool(&bootstrap_hold); }
bool persistent_store_bootstrap_faulted(void) { return metadata_load_bool(&storage_faulted); }
bool persistent_store_bootstrap_metadata_ready(void) { return metadata_load_bool(&recovery_metadata_ready); }
/* Exact grouped readbacks. Call after explicit provisioning or owner-completed
 * marker clear, never from QUERY. Physical lock is released before publication. */
bool persistent_store_verify_bootstrap(uint64_t generation,const uint8_t floor[16],const uint8_t*marker,bool complete) {
 bootstrap_complete_verified=0U;
 if(!persistent_store_enter())return false;
 BootstrapMarker expected;size_t marker_size=marker[0]==1U?32U:marker[0]==2U?40U:0U;
 if(!marker_size||!bootstrap_marker_decode(marker,marker_size,&expected)||(complete&&expected.state!=0U)){(void)persistent_store_leave();return false;}
 const ControlExpectation group={generation,floor,marker,marker_size};
 bool valid=validate_controls_expected(&group);
 bool released=persistent_store_leave();
 if(!valid||!released||!metadata_load_bool(&recovery_metadata_ready)){transient_fault();return false;}
 bootstrap_complete_verified=complete?1U:0U;return true;
}
bool persistent_store_publish_bootstrap_complete(void) {
 if(metadata_load_bool(&storage_faulted)||bootstrap_complete_verified!=1U||!metadata_load_bool(&recovery_metadata_ready)||!metadata_load_bool(&bootstrap_hold))return false;
 metadata_store_bool(&bootstrap_hold, (false));metadata_store_bool(&controls_ready, (true));bootstrap_complete_verified=2U;return true;
}

bool persistent_store_completed_boundary_ready(void){return bootstrap_complete_verified==2U;}
