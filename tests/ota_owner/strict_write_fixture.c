/* Reuse the existing physical-granule fault driver without changing its
 * source or any legacy assertions. This executable adds strict write cases. */
#define main strict_reader_existing_main
#include "test_persist_strict.c"
#undef main

static void small_reset(unsigned width) {
    reset(width);geometry.persistent_data_size=1024u;geometry.persistent_data_erase_size=512u;
}
static void canary(void) {
    uint8_t cal[68];size_t n=sizeof cal;launchcore_persist_strict_info_t info;
    assert(launchcore_persist_get_strict(CAL,cal,&n,&info)==LAUNCHCORE_PERSIST_OK&&n==sizeof cal);
    for(unsigned j=0;j<sizeof cal;++j)assert(cal[j]==0x5A);
}
static void strict_seed(unsigned width) {
    small_reset(width);launchcore_persist_strict_info_t info;uint64_t value=1;
    assert(launchcore_persist_provision_virgin_strict(KEY,&value,sizeof value,&info)==LAUNCHCORE_PERSIST_OK);
    uint8_t cal[68];memset(cal,0x5A,sizeof cal);
    assert(launchcore_persist_set_strict(CAL,cal,sizeof cal,&info)==LAUNCHCORE_PERSIST_OK);
    assert(init_calls==0);canary();
}
static void fill_for_gc(void) {
    uint32_t used=aligned(20)+aligned(32)+aligned(24+68);uint64_t value=0;
    launchcore_persist_strict_info_t info;
    while(used+aligned(32)<=geometry.persistent_data_size/2u) {
        assert(launchcore_persist_set_strict(77,&value,sizeof value,&info)==LAUNCHCORE_PERSIST_OK);
        used+=aligned(32);
    }
}
static void write_controls(unsigned width) {
    small_reset(width);launchcore_persist_strict_info_t info;uint64_t value=1;
    assert(launchcore_persist_set_strict(KEY,&value,sizeof value,&info)==LAUNCHCORE_PERSIST_NOT_FOUND);
    assert(launchcore_persist_delete_strict(KEY,&info)==LAUNCHCORE_PERSIST_NOT_FOUND);no_mutation(0,0,0);
    strict_seed(width);unsigned w=writes,e=erases;
    assert(launchcore_persist_provision_virgin_strict(KEY,&value,sizeof value,&info)==LAUNCHCORE_PERSIST_INVALID_ARGUMENT);no_mutation(0,w,e);
    assert(launchcore_persist_set_strict(KEY,NULL,1,&info)==LAUNCHCORE_PERSIST_INVALID_ARGUMENT);no_mutation(0,w,e);
    assert(launchcore_persist_set_strict(KEY,&value,257,&info)==LAUNCHCORE_PERSIST_INVALID_ARGUMENT);no_mutation(0,w,e);
    value=2;assert(launchcore_persist_set_strict(KEY,&value,sizeof value,&info)==LAUNCHCORE_PERSIST_OK);canary();
    assert(launchcore_persist_delete_strict(KEY,&info)==LAUNCHCORE_PERSIST_OK);
    value=99;assert(strict_get(KEY,&value,&info)==LAUNCHCORE_PERSIST_NOT_FOUND&&info.record==LAUNCHCORE_PERSIST_RECORD_DELETED);
    for(unsigned n=0;n<40;++n){value=n;assert(launchcore_persist_set_strict(77,&value,sizeof value,&info)==LAUNCHCORE_PERSIST_OK);}
    value=99;assert(strict_get(KEY,&value,&info)==LAUNCHCORE_PERSIST_NOT_FOUND&&info.record==LAUNCHCORE_PERSIST_RECORD_DELETED);canary();
    /* Existing adjacent-bank missing-key proof must not be erased by a write. */
    strict_seed(width);uint8_t header[32];uint32_t bank_size=geometry.persistent_data_size/2u;
    assert(backing->read(geometry.persistent_data_base,header,aligned(20))==LAUNCHCORE_STORAGE_OK);
    assert(backing->write(geometry.persistent_data_base+bank_size,header,aligned(20))==LAUNCHCORE_STORAGE_OK);bank_generation(1,2);
    counters();assert(launchcore_persist_set_strict(555,&value,sizeof value,&info)==LAUNCHCORE_PERSIST_CORRUPT);no_mutation(0,0,0);
    /* Exhaustion never wraps. */
    strict_seed(width);record_header rh;uint32_t at=last_record(CAL);
    assert(backing->read(at,&rh,sizeof rh)==LAUNCHCORE_STORAGE_OK);rh.sequence=UINT32_MAX-1u;rh.header_crc=launchcore_crc32(&rh,offsetof(record_header,header_crc));poke(at,&rh,sizeof rh);
    counters();assert(launchcore_persist_set_strict(KEY,&value,sizeof value,&info)==LAUNCHCORE_PERSIST_CORRUPT);no_mutation(0,0,0);
    printf("strict write granule%u controls/provision/delete/tombstones/history/sequence: PASS\n",width);
}
static void write_faults(unsigned width,unsigned kind) {
    if(kind==2u)small_reset(width);else{strict_seed(width);if(kind==1u)fill_for_gc();}
    capture();uint8_t original_active[512];memcpy(original_active,saved,sizeof original_active);
    launchcore_persist_strict_info_t info;uint64_t value=2;events=read_calls=0;
    launchcore_persist_status_t status=kind==2u?launchcore_persist_provision_virgin_strict(KEY,&value,sizeof value,&info):launchcore_persist_set_strict(KEY,&value,sizeof value,&info);
    assert(status==LAUNCHCORE_PERSIST_OK);unsigned event_count=events,read_count=read_calls;
    assert(event_count>0&&read_count>0);
    for(unsigned p=0;p<=event_count;++p) {
        restore();cut_after=p;
        status=kind==2u?launchcore_persist_provision_virgin_strict(KEY,&value,sizeof value,&info):launchcore_persist_set_strict(KEY,&value,sizeof value,&info);
        cut_after=UINT32_MAX;assert(init_calls==0);
        if(kind==1u){uint8_t active[512];assert(backing->read(geometry.persistent_data_base,active,sizeof active)==LAUNCHCORE_STORAGE_OK);assert(memcmp(active,original_active,sizeof active)==0);}
        launchcore_persist_status_t health=launchcore_persist_probe_strict(&info);uint64_t got=99;
        if(health==LAUNCHCORE_PERSIST_OK&&info.health==LAUNCHCORE_PERSIST_HEALTH_HEALTHY) {
            assert(strict_get(KEY,&got,&info)==LAUNCHCORE_PERSIST_OK&&(got==1||got==2));
            if(kind!=2u)canary();
            if(status==LAUNCHCORE_PERSIST_OK)assert(got==2);
        }else assert(status!=LAUNCHCORE_PERSIST_OK);
    }
    /* Fail every read in a successful operation, including partial-buffer IO.
     * No read error may be treated as an absent/latest-older record or success. */
    for(unsigned p=1;p<=read_count;++p) {
        restore();fail_read=p;partial_read=1;
        status=kind==2u?launchcore_persist_provision_virgin_strict(KEY,&value,sizeof value,&info):launchcore_persist_set_strict(KEY,&value,sizeof value,&info);
        assert(status!=LAUNCHCORE_PERSIST_OK);assert(read_calls>=p&&partial_prefixes==1);assert(init_calls==0);
        fail_read=UINT32_MAX;
        if(kind==1u){uint8_t active[512];assert(backing->read(geometry.persistent_data_base,active,sizeof active)==LAUNCHCORE_STORAGE_OK);assert(memcmp(active,original_active,sizeof active)==0);}
    }
    printf("strict write granule%u kind%u granulecuts%u partialreadfaults%u: PASS\n",width,kind,event_count+1u,read_count);
}
static unsigned skip_write_call, write_call;
static launchcore_storage_status_t silent_write(uint32_t a,const void *p,uint32_t n) {
    if(++write_call==skip_write_call)return LAUNCHCORE_STORAGE_OK;
    return program(a,p,n);
}
static void silent_write_faults(unsigned width,unsigned kind) {
    if(kind==2u)small_reset(width);else{strict_seed(width);if(kind==1u)fill_for_gc();}
    capture();launchcore_persist_strict_info_t info;uint64_t value=2;writes=0;
    assert((kind==2u?launchcore_persist_provision_virgin_strict(KEY,&value,sizeof value,&info):
        launchcore_persist_set_strict(KEY,&value,sizeof value,&info))==LAUNCHCORE_PERSIST_OK);
    const unsigned count=writes;
    for(unsigned k=1;k<=count;++k) {
        restore();launchcore_storage_driver_t driver=fake;driver.write=silent_write;
        skip_write_call=k;write_call=0;launchcore_storage_set_driver(&driver);
        launchcore_persist_status_t status=kind==2u?launchcore_persist_provision_virgin_strict(KEY,&value,sizeof value,&info):
            launchcore_persist_set_strict(KEY,&value,sizeof value,&info);
        assert(status!=LAUNCHCORE_PERSIST_OK);assert(init_calls==0);
        assert(info.health==LAUNCHCORE_PERSIST_HEALTH_CORRUPT||info.health==LAUNCHCORE_PERSIST_HEALTH_UNKNOWN);
        launchcore_storage_set_driver(&fake);
    }
    printf("strict write granule%u kind%u silent-program faults%u: PASS\n",width,kind,count);
}
/* Application-only prototype, not a new LaunchCore locking ABI or board opt-in.
 * A real owner supplies checked mutex operations and holds it across readback. */
static unsigned lock_calls,unlock_calls;static bool owner_locked,lock_failure,unlock_failure;
static bool owner_enter(void){++lock_calls;if(lock_failure)return false;assert(!owner_locked);owner_locked=true;return true;}
static bool owner_leave(void){++unlock_calls;assert(owner_locked);owner_locked=false;return !unlock_failure;}
static launchcore_persist_status_t checked_owner_put(uint32_t key,const void *value,size_t n,launchcore_persist_strict_info_t *info) {
    if(!owner_enter()){*info=(launchcore_persist_strict_info_t){0};return LAUNCHCORE_PERSIST_STORAGE_ERROR;}
    launchcore_persist_status_t status=launchcore_persist_set_strict(key,value,n,info);
    bool released=owner_leave();
    if(status==LAUNCHCORE_PERSIST_OK&&!released){info->health=LAUNCHCORE_PERSIST_HEALTH_UNKNOWN;return LAUNCHCORE_PERSIST_STORAGE_ERROR;}
    return status;
}
static void owner_controls(void) {
    strict_seed(8);launchcore_persist_strict_info_t info;uint64_t value=2;
    counters();lock_failure=true;assert(checked_owner_put(KEY,&value,sizeof value,&info)==LAUNCHCORE_PERSIST_STORAGE_ERROR);
    no_mutation(0,0,0);assert(read_calls==0&&lock_calls==1&&unlock_calls==0&&!owner_locked);
    lock_failure=false;assert(checked_owner_put(KEY,&value,sizeof value,&info)==LAUNCHCORE_PERSIST_OK&&!owner_locked);
    value=3;unlock_failure=true;assert(checked_owner_put(KEY,&value,sizeof value,&info)==LAUNCHCORE_PERSIST_STORAGE_ERROR&&!owner_locked);
    /* A failed unlock is not a successful transaction report even if durable. */
    uint64_t got=99;assert(strict_get(KEY,&got,&info)==LAUNCHCORE_PERSIST_OK&&got==3);
    unlock_failure=false;counters();fail_read=1;assert(checked_owner_put(KEY,&value,sizeof value,&info)==LAUNCHCORE_PERSIST_STORAGE_ERROR&&!owner_locked);
    no_mutation(0,0,0);assert(lock_calls==4&&unlock_calls==3);
    puts("application checked-lock prototype failure/no IO, balanced leave, verified durability: PASS");
}
int inherited_strict_write_main(int argc,char **argv) {
    assert(launchcore_storage_init()==LAUNCHCORE_STORAGE_OK);backing=launchcore_storage_driver();
    if(argc>1&&strcmp(argv[1],"legacy-negative")==0){
        strict_seed(8);counters();fail_read=1;uint64_t value=2;
        launchcore_persist_status_t status=launchcore_persist_set(KEY,&value,sizeof value);
        printf("legacy failed-first-read status%d writes%u erases%u (strict target error+zero mutations)\n",status,writes,erases);
        return status!=LAUNCHCORE_PERSIST_OK&&writes==0&&erases==0?0:1;
    }
    write_controls(8);write_controls(16);
    for(unsigned width=8;width<=16;width+=8)for(unsigned kind=0;kind<3;++kind)write_faults(width,kind);
    for(unsigned width=8;width<=16;width+=8)for(unsigned kind=0;kind<3;++kind)silent_write_faults(width,kind);
    owner_controls();puts("strict selected-bank writes: PASS");return 0;
}
