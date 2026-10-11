#include "stm32f4xx_hal.h"

ADC_HandleTypeDef hadc1;

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

    // 1. Enable ADC1 Clock
    __HAL_RCC_ADC1_CLK_ENABLE();

    // 2. Configure PA0 as Analog Input
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 3. Configure the ADC Peripheral
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B; // 12-bit means readings from 0 to 4095
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    HAL_ADC_Init(&hadc1);

    // 4. Map PA0 (Channel 0) to the ADC
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_0; 
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);


    while (1) {
        // Toggle the onboard LED (PC13)
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);  
        
        // Toggle the active buzzer (PA2)
        // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);   
        
        volatile uint32_t sensor_value = 0;
        
        // Start the ADC, wait for it to finish, and read the result
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            sensor_value = HAL_ADC_GetValue(&hadc1);
        }
        // Wait 500 milliseconds
        HAL_Delay(500);                         
    }
}

// System timer interrupt handler
void SysTick_Handler(void) {
    HAL_IncTick();
}