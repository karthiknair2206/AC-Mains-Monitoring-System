#include "stm32f4xx_hal.h"
#include <math.h>

// Your specific hardware zero points
#define VOLTAGE_ZERO 2064.0
#define CURRENT_ZERO 1620.0

ADC_HandleTypeDef hadc1;

// Variables for the debugger to watch
volatile double rms_voltage_raw = 0;
volatile double rms_current_raw = 0;
volatile uint32_t samples_taken = 0;
    
// Add these here so they are permanently in scope
volatile double real_amps = 0;
volatile double real_volts = 0;
volatile double real_watts = 0;

int main(void) {
    HAL_Init();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 
    GPIO_InitStruct.Pull = GPIO_NOPULL;         
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

    // Variables for the debugger to watch
    rms_voltage_raw = 0;
    rms_current_raw = 0;
    samples_taken = 0;

    while (1) {
        double sum_v_sq = 0;
        double sum_i_sq = 0;
        uint32_t sample_count = 0;

        // Record the start time
        uint32_t start_time = HAL_GetTick();

        // Sample continuously for exactly one 50Hz AC cycle (20ms)
        while ((HAL_GetTick() - start_time) < 20) {
            
            // 1. Read Voltage (PA0)
            sConfig.Channel = ADC_CHANNEL_0; 
            HAL_ADC_ConfigChannel(&hadc1, &sConfig);
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 1);
            double v_raw = HAL_ADC_GetValue(&hadc1);

            // 2. Read Current (PA1)
            sConfig.Channel = ADC_CHANNEL_1; 
            HAL_ADC_ConfigChannel(&hadc1, &sConfig);
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 1);
            double i_raw = HAL_ADC_GetValue(&hadc1);

            // 3. Shift the wave to 0, square it, and add to the accumulator
            sum_v_sq += (v_raw - VOLTAGE_ZERO) * (v_raw - VOLTAGE_ZERO);
            sum_i_sq += (i_raw - CURRENT_ZERO) * (i_raw - CURRENT_ZERO);
            sample_count++;
        }

       // Calculate Root Mean Square
        if (sample_count > 0) {
            rms_voltage_raw = sqrt(sum_v_sq / sample_count);
            rms_current_raw = sqrt(sum_i_sq / sample_count);
            samples_taken = sample_count;

            // --- DEADBAND FILTER ---
            // If the raw noise is below our known idle levels, force it to zero
            if (rms_voltage_raw < 15.0) rms_voltage_raw = 0;
            if (rms_current_raw < 65.0) rms_current_raw = 0;

            // --- CONVERT TO REAL AMPS ---
            // 5A ACS712 (185mV/A) divided by 2 via 10k resistors = 92.5mV/A (0.0925V/A)
            real_amps = (rms_current_raw * 3.3 / 4095.0) / 0.0925;

            // --- CONVERT TO REAL VOLTS ---
            // The ZMPT101B requires manual calibration against a multimeter. 
            // We use a temporary multiplier of 1.0 until we apply live AC.
            real_volts = rms_voltage_raw * 1.0; 
            
            // Calculate Power (Watts)
            real_watts = real_volts * real_amps;
        }

        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);  
        //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);   
        
        // Wait briefly before taking the next batch of samples
        HAL_Delay(250);                         
    }
}

void SysTick_Handler(void) {
    HAL_IncTick();
}