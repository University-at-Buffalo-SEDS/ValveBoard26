#include <stdint.h>
typedef struct { int initialized; } DMA_HandleTypeDef;
typedef struct { int initialized; } FDCAN_HandleTypeDef;
typedef struct { int initialized; } PCD_HandleTypeDef;
typedef struct { int initialized; } TIM_HandleTypeDef;
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *);
void HAL_FDCAN_IRQHandler(FDCAN_HandleTypeDef *);
void HAL_PCD_IRQHandler(PCD_HandleTypeDef *);
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *);
void HAL_NVIC_DisableIRQ(int);
void HAL_NVIC_ClearPendingIRQ(int);
enum { USB_LP_IRQn = 20 };
