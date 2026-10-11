#include "stm32f4xx_hal.h"

ADC_HandleTypeDef hadc1;

int main(void) {
    HAL_Init();

    // Enable hardware clocks
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    // Configure PC13 (LED) and PA2 (Buzzer)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 
    GPIO_InitStruct.Pull = GPIO_NOPULL;         
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure PA0 (Voltage) and PA1 (Current) as Analog Inputs
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Initialize the ADC
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B; 
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    HAL_ADC_Init(&hadc1);

    // Keep variables in memory for the debugger
    volatile uint32_t voltage_value = 0;
    volatile uint32_t current_value = 0;
    
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

    while (1) {
        // --- 1. Read ZMPT101B Voltage Sensor (PA0) ---
        sConfig.Channel = ADC_CHANNEL_0; 
        HAL_ADC_ConfigChannel(&hadc1, &sConfig);
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            voltage_value = HAL_ADC_GetValue(&hadc1);
        }

        // --- 2. Read ACS712 Current Sensor (PA1) ---
        sConfig.Channel = ADC_CHANNEL_1; 
        HAL_ADC_ConfigChannel(&hadc1, &sConfig);
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            current_value = HAL_ADC_GetValue(&hadc1);
        }

        // Toggle LED/Buzzer to show the loop is running
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);  
        //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);   
        
        HAL_Delay(500);                         
    }
}

void SysTick_Handler(void) {
    HAL_IncTick();
}