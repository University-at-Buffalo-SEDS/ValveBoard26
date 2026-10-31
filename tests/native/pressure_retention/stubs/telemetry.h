#pragma once
#include "sedsnet.h"
#define SEDS_DT_VALVE_PRESSURE_SD_V1 157797238
static struct{SedsRouter*r;}g_router={(SedsRouter*)1};
static int init_telemetry_router(void);
static uint64_t telemetry_unix_ms(void);
static int log_telemetry_asynchronous(unsigned,const void*,size_t,size_t);
