#include "power_i2c_recovery.h"
#include "tx_api.h"

#define POWER_SDA GPIO_PIN_8
#define POWER_SCL GPIO_PIN_9

volatile uint32_t g_power_i2c_recovery_attempts;
volatile uint32_t g_power_i2c_recovery_ok;
volatile uint32_t g_power_i2c_recovery_fail;

static void pause_ms(void)
{
    if (tx_thread_identify() != TX_NULL) {
        tx_thread_sleep((TX_TIMER_TICKS_PER_SECOND + 999U) / 1000U);
    } else {
        HAL_Delay(1U);
    }
}

static bool scl_released(void)
{
    for (unsigned wait = 0U; wait < 3U; ++wait) {
        if (HAL_GPIO_ReadPin(GPIOA, POWER_SCL) == GPIO_PIN_SET) return true;
        pause_ms();
    }
    return HAL_GPIO_ReadPin(GPIOA, POWER_SCL) == GPIO_PIN_SET;
}

bool power_i2c_needs_recovery(I2C_HandleTypeDef *hi2c)
{
    return hi2c && hi2c->Instance == I2C2 &&
        (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) != 0U ||
         HAL_GPIO_ReadPin(GPIOA, POWER_SDA) == GPIO_PIN_RESET ||
         HAL_GPIO_ReadPin(GPIOA, POWER_SCL) == GPIO_PIN_RESET);
}

HAL_StatusTypeDef power_i2c_recover(I2C_HandleTypeDef *hi2c)
{
    if (!hi2c || hi2c->Instance != I2C2) return HAL_ERROR;
    ++g_power_i2c_recovery_attempts;
    (void)HAL_I2C_DeInit(hi2c);
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /* SET releases an open-drain pin; never drive either line high. */
    HAL_GPIO_WritePin(GPIOA, POWER_SDA | POWER_SCL, GPIO_PIN_SET);
    GPIO_InitTypeDef pins = {0};
    pins.Pin = POWER_SDA | POWER_SCL;
    pins.Mode = GPIO_MODE_OUTPUT_OD;
    pins.Pull = GPIO_NOPULL;
    pins.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &pins);
    bool clear = scl_released();
    for (unsigned pulse = 0U; clear && pulse < 9U &&
         HAL_GPIO_ReadPin(GPIOA, POWER_SDA) == GPIO_PIN_RESET; ++pulse) {
        HAL_GPIO_WritePin(GPIOA, POWER_SCL, GPIO_PIN_RESET);
        pause_ms();
        HAL_GPIO_WritePin(GPIOA, POWER_SCL, GPIO_PIN_SET);
        clear = scl_released();
        pause_ms();
    }
    clear = clear && HAL_GPIO_ReadPin(GPIOA, POWER_SDA) == GPIO_PIN_SET;
    if (clear) {
        /* Complete a STOP after the target releases its interrupted byte. */
        HAL_GPIO_WritePin(GPIOA, POWER_SCL, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, POWER_SDA, GPIO_PIN_RESET);
        pause_ms();
        HAL_GPIO_WritePin(GPIOA, POWER_SCL, GPIO_PIN_SET);
        clear = scl_released();
        HAL_GPIO_WritePin(GPIOA, POWER_SDA, GPIO_PIN_SET);
        pause_ms();
    }
    /* Always restore alternate-function pins and controller state, including
     * when a physical short or an unpowered device keeps a line low. */
    const HAL_StatusTypeDef initialized = HAL_I2C_Init(hi2c);
    HAL_StatusTypeDef analog = HAL_ERROR, digital = HAL_ERROR;
    if (initialized == HAL_OK) {
        analog = HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE);
        digital = HAL_I2CEx_ConfigDigitalFilter(hi2c, 0U);
    }
    clear = clear && initialized == HAL_OK && analog == HAL_OK && digital == HAL_OK &&
        !power_i2c_needs_recovery(hi2c);
    if (clear) ++g_power_i2c_recovery_ok;
    else ++g_power_i2c_recovery_fail;
    return clear ? HAL_OK : HAL_ERROR;
}
