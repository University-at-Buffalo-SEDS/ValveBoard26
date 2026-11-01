#include "board_abort_latch.h"
#include "main.h"

/* Reserve TAMP backup registers 30/31. They survive NRST, NVIC reset and IWDG.
 * Only loss of backup-domain power resets them; no RCC reset flag clears abort.
 * Ordinary RAM and the bootloader do not own these registers. No flash writes,
 * allocator calls, RTC oscillator or RTOS services are required. */
#define ABORT_MAGIC UINT32_C(0x41425254)
/* Launch inhibition owns backup registers 28/29; abort owns 30/31. */
static uint8_t launch_latched;
static uint8_t latched;
static uint8_t backup_ready;

static uint8_t enable_backup_writes(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN | RCC_APB1ENR1_RTCAPBEN;
    (void)RCC->APB1ENR1;
    PWR->CR1 |= PWR_CR1_DBP;
    uint32_t attempts = 100000U;
    while ((PWR->CR1 & PWR_CR1_DBP) == 0U && attempts != 0U) {
        --attempts;
    }
    backup_ready = (PWR->CR1 & PWR_CR1_DBP) != 0U;
    return backup_ready;
}

void board_abort_latch_init(void)
{
    (void)enable_backup_writes();
    if (!backup_ready) {
        launch_latched = 1U;
        latched = 1U; /* Fail closed, keeping the scheduler/network alive. */
        return;
    }
    /* A torn marker write also remains aborted until power is removed. */
    launch_latched = (TAMP->BKP28R != 0U || TAMP->BKP29R != 0U);
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

/* Explicit manual recovery only. A fresh attempt may repair transient backup
 * write permission; never clear a prior RAM latch while retrying availability. */
uint8_t board_abort_latch_prepare_recovery(void)
{
    latched = 1U;
    if (!enable_backup_writes()) return 0U;
    board_abort_latch_set();
    return TAMP->BKP30R == ABORT_MAGIC && TAMP->BKP31R == ~ABORT_MAGIC;
}

uint8_t board_abort_latch_is_set(void)
{
    return latched;
}

/* Physical launch authority requires a verified persistent marker. Keep RAM
 * inhibition and attempt Abort retention on failure; never authorize launch
 * merely because backup access succeeded earlier at boot. */
uint8_t board_launch_latch_set_verified(void)
{
    launch_latched = 1U;
    if (!enable_backup_writes()) {
        board_abort_latch_set();
        return 0U;
    }
    TAMP->BKP28R = UINT32_C(0x4c41554e);
    TAMP->BKP29R = ~UINT32_C(0x4c41554e);
    __DSB();
    if (TAMP->BKP28R != UINT32_C(0x4c41554e) ||
        TAMP->BKP29R != ~UINT32_C(0x4c41554e)) {
        board_abort_latch_set();
        return 0U;
    }
    return 1U;
}
/* Compatibility helper; physical admissions use the checked API. */
void board_launch_latch_set(void)
{
    (void)board_launch_latch_set_verified();
}
uint8_t board_launch_latch_is_set(void)
{
    return launch_latched;
}

/* Called only by explicit, authenticated recovery after the control owner has
 * cancelled its sequences, purged stale commands, and verified safe outputs. */
uint8_t board_abort_latch_clear_for_recovery(void)
{
    if (!backup_ready) return 0U;
    TAMP->BKP28R = 0U;
    TAMP->BKP29R = 0U;
    TAMP->BKP30R = 0U;
    TAMP->BKP31R = 0U;
    __DSB();
    if (TAMP->BKP28R != 0U || TAMP->BKP29R != 0U ||
        TAMP->BKP30R != 0U || TAMP->BKP31R != 0U) return 0U;
    launch_latched = 0U;
    latched = 0U;
    return 1U;
}
