#pragma once

#include "stm32g4xx_hal.h"
#include <stdint.h>

#ifndef PRESSURE_TRANSDUCER_MIN_ADC
#define PRESSURE_TRANSDUCER_MIN_ADC (0.0f)
#endif

#ifndef PRESSURE_TRANSDUCER_ZERO_ADC_DEADBAND
#define PRESSURE_TRANSDUCER_ZERO_ADC_DEADBAND (8.0f)
#endif

#ifndef PRESSURE_TRANSDUCER_MIN_PRESSURE
#define PRESSURE_TRANSDUCER_MIN_PRESSURE (0.0f)
#endif

#ifndef PRESSURE_TRANSDUCER_MAX_ADC
#define PRESSURE_TRANSDUCER_MAX_ADC (3103.0f)
#endif

#ifndef PRESSURE_TRANSDUCER_MAX_PRESSURE
#define PRESSURE_TRANSDUCER_MAX_PRESSURE (2000.0f)
#endif

#ifndef PRESSURE_TRANSDUCER_ADC_TIMEOUT_MS
#define PRESSURE_TRANSDUCER_ADC_TIMEOUT_MS (10U)
#endif

#ifndef PRESSURE_TRANSDUCER_DMA_BUFFER_LEN
#define PRESSURE_TRANSDUCER_DMA_BUFFER_LEN (16U)
#endif

typedef struct {
    ADC_HandleTypeDef *hadc;
    TIM_HandleTypeDef *trigger_timer;
    uint32_t           channel;
    float              min_adc;
    float              max_adc;
    float              min_pressure;
    float              max_pressure;
    uint32_t           timeout_ms;
    volatile uint16_t  dma_samples[PRESSURE_TRANSDUCER_DMA_BUFFER_LEN];
    uint8_t            dma_running;
} pressure_transducer_t;

typedef struct {
    uint32_t raw_adc;
    float    pressure;
} pressure_transducer_sample_t;

typedef struct {
    uint32_t dma_remaining;
    uint32_t adc_state;
    uint32_t adc_error;
    uint32_t timer_counter;
    uint32_t timer_state;
    uint16_t dma_samples[PRESSURE_TRANSDUCER_DMA_BUFFER_LEN];
} pressure_transducer_debug_t;

HAL_StatusTypeDef pressureTransducerInit(pressure_transducer_t *sensor);
HAL_StatusTypeDef pressureTransducerRead(pressure_transducer_t *sensor,
                                         pressure_transducer_sample_t *sample);
HAL_StatusTypeDef pressureTransducerDebug(const pressure_transducer_t *sensor,
                                          pressure_transducer_debug_t *debug);
