#pragma once
#include <stdint.h>
typedef struct{unsigned remaining;}DMA_HandleTypeDef;
typedef struct{DMA_HandleTypeDef*DMA_Handle;}ADC_HandleTypeDef;
typedef struct{unsigned unused;}TIM_HandleTypeDef;
typedef int HAL_StatusTypeDef;
#define __HAL_DMA_GET_COUNTER(d) ((d)->remaining)
uint32_t HAL_GetTick(void);
