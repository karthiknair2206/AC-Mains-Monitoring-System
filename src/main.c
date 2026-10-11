#include "stm32f4xx_hal.h"

int main(void) {
    HAL_Init();

    // Enable the hardware clocks for both Port C and Port A
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure PC13 for the onboard LED
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 
    GPIO_InitStruct.Pull = GPIO_NOPULL;         
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Reuse the exact same struct configuration, but apply it to PA2 for the buzzer
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    while (1) {
        // Toggle the onboard LED (PC13)
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);  
        
        // Toggle the active buzzer (PA2)
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);   
        
        // Wait 500 milliseconds
        HAL_Delay(500);                         
    }
}

// System timer interrupt handler
void SysTick_Handler(void) {
    HAL_IncTick();
}