#include "main.h"
#include <stdlib.h>
#include <stdio.h>
DMA_HandleTypeDef hdma_adc3;
FDCAN_HandleTypeDef hfdcan2;
PCD_HandleTypeDef hpcd_USB_FS;
TIM_HandleTypeDef htim6;
static unsigned calls[4], disable_count, clear_count, order;
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *p) { if (p != &hdma_adc3) exit(40); calls[0]++; }
void HAL_FDCAN_IRQHandler(FDCAN_HandleTypeDef *p) { if (p != &hfdcan2) exit(41); calls[1]++; }
void HAL_PCD_IRQHandler(PCD_HandleTypeDef *p) {
    if (p != &hpcd_USB_FS) exit(42);
    if (!p->initialized) { puts("UNINITIALIZED_USB_HANDLE_REACHED_HAL"); exit(37); }
    calls[2]++;
}
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *p) { if (p != &htim6) exit(43); calls[3]++; }
void HAL_NVIC_DisableIRQ(int irq) {
    if (irq != USB_LP_IRQn || order != 0) exit(44);
    order=1; disable_count++;
}
void HAL_NVIC_ClearPendingIRQ(int irq) {
    if (irq != USB_LP_IRQn || order != 1) exit(45);
    order=0; clear_count++;
}
void USB_LP_IRQHandler(void);
void DMA1_Channel1_IRQHandler(void);
void FDCAN2_IT0_IRQHandler(void);
void FDCAN2_IT1_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);
int main(void) {
    hpcd_USB_FS.initialized = FIRMWARE_USB_DEBUG_ENABLED;
    for (int i=0; i<3; i++) USB_LP_IRQHandler();
    DMA1_Channel1_IRQHandler(); FDCAN2_IT0_IRQHandler(); FDCAN2_IT1_IRQHandler(); TIM6_DAC_IRQHandler();
    if (calls[0]!=1 || calls[1]!=2 || calls[3]!=1) return 46;
    if (FIRMWARE_USB_DEBUG_ENABLED) {
        if (calls[2]!=3 || disable_count!=0 || clear_count!=0) return 47;
    } else if (calls[2]!=0 || disable_count!=3 || clear_count!=3 || order!=0) return 48;
    printf("USB_PROFILE_PASS enabled=%d pcd=%u disabled=%u cleared=%u CAN=%u DMA=%u TIM=%u\n",
        FIRMWARE_USB_DEBUG_ENABLED,calls[2],disable_count,clear_count,calls[1],calls[0],calls[3]);
    return 0;
}
