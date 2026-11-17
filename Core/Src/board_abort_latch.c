#include "board_abort_latch.h"
#include "main.h"

/* Reserve TAMP backup registers 30/31. They survive NRST, NVIC reset and IWDG.
 * Only loss of backup-domain power resets them; no RCC reset flag clears abort.
 * Ordinary RAM and the bootloader do not own these registers. No flash writes,
 * allocator calls, RTC oscillator or RTOS services are required. */
#define ABORT_MAGIC UINT32_C(0x41425254)
/* Launch inhibition owns backup registers 28/29; abort owns 30/31; the
 * post-Launch auto-vent marker owns 26/27: tag 0x56 in the top byte, bit 16
 * vented, low 16 bits Launch elapsed seconds; 27 holds the complement. */
#define VENT_MARKER_TAG UINT32_C(0x56000000)
static uint8_t launch_latched;
static uint8_t launch_verified;
static uint8_t latched;
static uint8_t backup_ready;
/* A fresh 0->1 latch owes the station one unsolicited status. */
static volatile uint8_t push_pending;

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
    launch_verified = 0U;
    (void)enable_backup_writes();
    if (!backup_ready) {
        /* Fail closed with the RAM launch inhibit only: the state may be after
         * Launch, where no automatic Abort is allowed. */
        launch_latched = 1U;
        return;
    }
    /* A torn marker write also remains aborted until power is removed. */
    launch_latched = (TAMP->BKP28R != 0U || TAMP->BKP29R != 0U);
    launch_verified = TAMP->BKP28R == UINT32_C(0x4c41554e) &&
        TAMP->BKP29R == ~UINT32_C(0x4c41554e);
    latched = (TAMP->BKP30R != 0U || TAMP->BKP31R != 0U);
}

void board_abort_latch_set(void)
{
    /* Recovery preparation latches first, so it never owes a push. */
    if (!latched) push_pending = 1U;
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

uint8_t board_abort_latch_push_pending(void)
{
    return push_pending;
}

void board_abort_latch_push_sent(void)
{
    push_pending = 0U;
}

/* Physical launch authority requires a verified persistent marker. Keep RAM
 * inhibition on failure, never an Abort (none after Launch); never authorize
 * launch merely because backup access succeeded earlier at boot. */
uint8_t board_launch_latch_set_verified(void)
{
    launch_latched = 1U;
    if (!enable_backup_writes()) return 0U;
    /* A new Launch starts with no elapsed time and no vent. */
    if (TAMP->BKP28R != UINT32_C(0x4c41554e)) {
        TAMP->BKP26R = 0U;
        TAMP->BKP27R = 0U;
    }
    TAMP->BKP28R = UINT32_C(0x4c41554e);
    TAMP->BKP29R = ~UINT32_C(0x4c41554e);
    __DSB();
    if (TAMP->BKP28R != UINT32_C(0x4c41554e) ||
        TAMP->BKP29R != ~UINT32_C(0x4c41554e)) return 0U;
    launch_verified = 1U;
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
uint8_t board_launch_latch_was_verified(void)
{
    return launch_verified;
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
    TAMP->BKP26R = 0U;
    TAMP->BKP27R = 0U;
    __DSB();
    if (TAMP->BKP28R != 0U || TAMP->BKP29R != 0U ||
        TAMP->BKP30R != 0U || TAMP->BKP31R != 0U) return 0U;
    launch_verified = 0U;
    launch_latched = 0U;
    latched = 0U;
    return 1U;
}

/* Absent, torn or foreign: no elapsed time, not vented (the guard then runs
 * in full from boot and the board holds closed until the countdown ends). */
uint8_t board_vent_marker_load(uint32_t *elapsed_s, uint8_t *vented)
{
    *elapsed_s = 0U;
    *vented = 0U;
    if (!backup_ready) return 0U;
    const uint32_t m = TAMP->BKP26R;
    if ((m & UINT32_C(0xFF000000)) != VENT_MARKER_TAG || TAMP->BKP27R != ~m) return 0U;
    *elapsed_s = m & 0xFFFFU;
    *vented = (uint8_t)((m >> 16) & 1U);
    return 1U;
}
void board_vent_marker_store(uint32_t elapsed_s, uint8_t vented)
{
    if (!backup_ready) return;
    if (elapsed_s > 0xFFFFU) elapsed_s = 0xFFFFU;
    const uint32_t m = VENT_MARKER_TAG | (vented ? UINT32_C(0x10000) : 0U) | elapsed_s;
    /* Invalidate first: a reset between the two writes reads as absent. */
    TAMP->BKP27R = 0U;
    TAMP->BKP26R = m;
    TAMP->BKP27R = ~m;
    __DSB();
}
uint8_t board_vent_marker_vented(void)
{
    uint32_t elapsed;
    uint8_t vented;
    return board_vent_marker_load(&elapsed, &vented) && vented;
}
