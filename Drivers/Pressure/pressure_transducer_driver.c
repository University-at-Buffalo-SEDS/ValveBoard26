#include "pressure_transducer_driver.h"

static uint32_t pressure_adc_full_scale(const ADC_HandleTypeDef *hadc)
{
    switch (hadc->Init.Resolution) {
    case ADC_RESOLUTION_12B:
        return 4095U;
    case ADC_RESOLUTION_10B:
        return 1023U;
    case ADC_RESOLUTION_8B:
        return 255U;
    case ADC_RESOLUTION_6B:
        return 63U;
    default:
        return 4095U;
    }
}

static float pressure_from_voltage(const pressure_transducer_t *sensor, float voltage)
{
    const float voltage_span = sensor->max_voltage - sensor->min_voltage;
    const float pressure_span = sensor->max_pressure - sensor->min_pressure;

    if (voltage_span == 0.0f) {
        return sensor->min_pressure;
    }

    return sensor->min_pressure +
           ((voltage - sensor->min_voltage) * pressure_span / voltage_span);
}

static HAL_StatusTypeDef pressure_configure_channel(pressure_transducer_t *sensor)
{
    ADC_ChannelConfTypeDef channel_config = {0};

    channel_config.Channel = sensor->channel;
    channel_config.Rank = ADC_REGULAR_RANK_1;
    channel_config.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    channel_config.SingleDiff = ADC_SINGLE_ENDED;
    channel_config.OffsetNumber = ADC_OFFSET_NONE;
    channel_config.Offset = 0;

    return HAL_ADC_ConfigChannel(sensor->hadc, &channel_config);
}

static uint32_t pressure_latest_raw_adc(const pressure_transducer_t *sensor)
{
    uint32_t write_index;
    uint32_t latest_index;
    uint32_t remaining = __HAL_DMA_GET_COUNTER(sensor->hadc->DMA_Handle);

    write_index = (PRESSURE_TRANSDUCER_DMA_BUFFER_LEN - remaining) %
                  PRESSURE_TRANSDUCER_DMA_BUFFER_LEN;
    latest_index = (write_index + PRESSURE_TRANSDUCER_DMA_BUFFER_LEN - 1U) %
                   PRESSURE_TRANSDUCER_DMA_BUFFER_LEN;

    return sensor->dma_samples[latest_index];
}

HAL_StatusTypeDef pressureTransducerInit(pressure_transducer_t *sensor)
{
    if ((sensor == NULL) || (sensor->hadc == NULL) || (sensor->trigger_timer == NULL)) {
        return HAL_ERROR;
    }

    if (sensor->vref == 0.0f) {
        sensor->vref = PRESSURE_TRANSDUCER_ADC_VREF;
    }
    if (sensor->min_voltage == 0.0f) {
        sensor->min_voltage = PRESSURE_TRANSDUCER_MIN_VOLTAGE;
    }
    if (sensor->max_voltage == 0.0f) {
        sensor->max_voltage = PRESSURE_TRANSDUCER_MAX_VOLTAGE;
    }
    if (sensor->max_pressure == 0.0f) {
        sensor->max_pressure = PRESSURE_TRANSDUCER_MAX_PRESSURE;
    }
    if (sensor->timeout_ms == 0U) {
        sensor->timeout_ms = PRESSURE_TRANSDUCER_ADC_TIMEOUT_MS;
    }

    (void)HAL_ADC_Stop(sensor->hadc);
    (void)HAL_ADC_Stop_DMA(sensor->hadc);
    sensor->dma_running = 0U;
    for (uint32_t i = 0U; i < PRESSURE_TRANSDUCER_DMA_BUFFER_LEN; i++) {
        sensor->dma_samples[i] = 0U;
    }

    HAL_StatusTypeDef status = HAL_ADCEx_Calibration_Start(sensor->hadc, ADC_SINGLE_ENDED);
    if (status != HAL_OK) {
        return status;
    }

    status = pressure_configure_channel(sensor);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_ADC_Start_DMA(sensor->hadc, (uint32_t *)sensor->dma_samples,
                               PRESSURE_TRANSDUCER_DMA_BUFFER_LEN);
    if (status != HAL_OK) {
        return status;
    }

    if (sensor->hadc->DMA_Handle != NULL) {
        __HAL_DMA_DISABLE_IT(sensor->hadc->DMA_Handle, DMA_IT_HT | DMA_IT_TC);
    }

    status = HAL_TIM_Base_Start(sensor->trigger_timer);
    if (status != HAL_OK) {
        (void)HAL_ADC_Stop_DMA(sensor->hadc);
        return status;
    }

    sensor->dma_running = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef pressureTransducerRead(pressure_transducer_t *sensor,
                                         pressure_transducer_sample_t *sample)
{
    uint32_t raw_adc;
    uint32_t full_scale;

    if ((sensor == NULL) || (sensor->hadc == NULL) || (sample == NULL)) {
        return HAL_ERROR;
    }

    if ((sensor->dma_running == 0U) || (sensor->hadc->DMA_Handle == NULL)) {
        return HAL_BUSY;
    }

    raw_adc = pressure_latest_raw_adc(sensor);
    full_scale = pressure_adc_full_scale(sensor->hadc);
    sample->raw_adc = raw_adc;
    sample->voltage = ((float)raw_adc * sensor->vref) / (float)full_scale;
    sample->pressure = pressure_from_voltage(sensor, sample->voltage);

    return HAL_OK;
}

HAL_StatusTypeDef pressureTransducerDebug(const pressure_transducer_t *sensor,
                                          pressure_transducer_debug_t *debug)
{
    if ((sensor == NULL) || (sensor->hadc == NULL) || (debug == NULL)) {
        return HAL_ERROR;
    }

    debug->dma_remaining = 0U;
    debug->adc_state = HAL_ADC_GetState(sensor->hadc);
    debug->adc_error = HAL_ADC_GetError(sensor->hadc);
    debug->timer_counter = 0U;
    debug->timer_state = 0U;

    if (sensor->hadc->DMA_Handle != NULL) {
        debug->dma_remaining = __HAL_DMA_GET_COUNTER(sensor->hadc->DMA_Handle);
    }

    if (sensor->trigger_timer != NULL) {
        debug->timer_counter = __HAL_TIM_GET_COUNTER(sensor->trigger_timer);
        debug->timer_state = HAL_TIM_Base_GetState(sensor->trigger_timer);
    }

    for (uint32_t i = 0U; i < PRESSURE_TRANSDUCER_DMA_BUFFER_LEN; i++) {
        debug->dma_samples[i] = sensor->dma_samples[i];
    }

    return HAL_OK;
}
