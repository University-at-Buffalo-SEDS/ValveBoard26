#ifndef VALVE_PRESSURE_SD_H
#define VALVE_PRESSURE_SD_H
#include "tx_api.h"
#include "pressure_transducer_driver.h"
uint32_t pressure_sd_stack_remaining(void);
UINT pressure_sd_init(pressure_transducer_t *sensor);
#endif
