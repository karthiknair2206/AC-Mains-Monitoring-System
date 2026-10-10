#include "stm32f4xx_hal.h"

int main(void) {
    // 1. Initialize the HAL and the 1ms timer
    HAL_Init();

    // 2. Wake up Port C by enabling its clock
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // 3. Create a configuration struct and set up PC13
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 
    GPIO_InitStruct.Pull = GPIO_NOPULL;         
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    // Apply the struct settings to Port C
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // 4. The main application loop
    while (1) {
        // Toggle the onboard LED
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);  
        
        // Wait 500 milliseconds
        HAL_Delay(1000);                         
    }
}

// 5. Background interrupt that increments the 1ms timer
void SysTick_Handler(void) {
    HAL_IncTick();
}