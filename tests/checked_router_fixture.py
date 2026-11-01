"""Live FC checked ownership functions with controlled ThreadX primitives."""
from pathlib import Path
from router_lifecycle_fixture import function

def checked_hooks(root):
    source=(Path(root)/'Core/Src/telemetry_hooks.c').read_text()
    return r"""
#include <stddef.h>
#define TX_SUCCESS 0U
#define TX_WAIT_FOREVER (~0U)
#define TX_NULL NULL
#define TX_MUTEX_ID 0x4D555445U
typedef struct { unsigned tx_mutex_id; } TX_MUTEX;
static TX_MUTEX g_telemetry_mutex={TX_MUTEX_ID};
static unsigned g_telemetry_mutex_ready=1,g_telemetry_lock_get_fail,g_telemetry_lock_put_fail;
static unsigned fixture_isr,fixture_get_failure,fixture_put_failure;
static void *tx_thread_identify(void){return (void*)1;}
static unsigned __get_IPSR(void){return fixture_isr;}
static unsigned tx_mutex_get(TX_MUTEX *mu,unsigned timeout){assert(mu==&g_telemetry_mutex&&timeout==TX_WAIT_FOREVER);if(fixture_get_failure)return 1;telemetry_lock();return TX_SUCCESS;}
static unsigned tx_mutex_put(TX_MUTEX *mu){assert(mu==&g_telemetry_mutex);telemetry_unlock();return fixture_put_failure?1:TX_SUCCESS;}
"""+'\n'.join(function(source,name) for name in ('telemetry_router_context_ready','telemetry_router_enter','telemetry_router_leave'))
