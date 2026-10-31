#pragma once
#include <stddef.h>
#include <stdint.h>
typedef int SedsResult;typedef struct{unsigned unused;}SedsRouter;
typedef struct{size_t reliable_max_end_to_end_pending;}SedsRuntimeTuningConfig;
#define SEDS_OK 0
#define SEDS_IO (-14)
#define SEDS_ERR (-2)
SedsResult seds_get_runtime_tuning_config(SedsRuntimeTuningConfig*);
int seds_router_copy_queue_counters(SedsRouter*,uint32_t*,size_t);
