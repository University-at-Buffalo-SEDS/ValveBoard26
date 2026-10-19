#include "board_abort_latch.h"
#include "main.h"

/* Reserve TAMP backup registers 30/31. They survive NRST, NVIC reset and IWDG.
 * Only loss of backup-domain power resets them; no RCC reset flag clears abort.
 * Ordinary RAM and the bootloader do not own these registers. No flash writes,
 * allocator calls, RTC oscillator or RTOS services are required. */
#define ABORT_MAGIC UINT32_C(0x41425254)
static uint8_t latched;
static uint8_t backup_ready;

void board_abort_latch_init(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN | RCC_APB1ENR1_RTCAPBEN;
    (void)RCC->APB1ENR1;
    PWR->CR1 |= PWR_CR1_DBP;
    uint32_t attempts = 100000U;
    while ((PWR->CR1 & PWR_CR1_DBP) == 0U && attempts != 0U) {
        --attempts;
    }
    backup_ready = (PWR->CR1 & PWR_CR1_DBP) != 0U;
    if (!backup_ready) {
        latched = 1U; /* Fail closed, keeping the scheduler/network alive. */
        return;
    }
    /* A torn marker write also remains aborted until power is removed. */
    latched = (TAMP->BKP30R != 0U || TAMP->BKP31R != 0U);
}

void board_abort_latch_set(void)
{
    latched = 1U;
    if (backup_ready) {
        TAMP->BKP30R = ABORT_MAGIC;
        TAMP->BKP31R = ~ABORT_MAGIC;
        __DSB();
    }
}

uint8_t board_abort_latch_is_set(void)
{
    return latched;
}
