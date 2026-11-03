#include "telemetry_event_clock.h"
#include "main.h"
#include "tx_api.h"
/* Single-core STM32 task/IRQ ownership. No SDK/NVM calls while masked. */
static volatile uint64_t observed_utc;
static volatile uint32_t observed_tick;
static volatile uint8_t observed_valid;
void telemetry_event_clock_observe(uint64_t utc, uint32_t tick)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    observed_valid = utc != 0U && utc != UINT64_MAX && utc >= observed_utc;
    if (observed_valid) { observed_utc = utc; observed_tick = tick; }
    __set_PRIMASK(mask);
}
uint64_t telemetry_event_clock_snapshot(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint64_t utc = observed_valid ? observed_utc : 0U;
    uint32_t tick = observed_tick;
    __set_PRIMASK(mask);
    uint32_t now = (uint32_t)tx_time_get();
    return (uint32_t)(now - tick) <= TX_TIMER_TICKS_PER_SECOND ? utc : 0U;
}
