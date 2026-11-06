#include "stm32f4xx_hal.h"
#include <math.h>

// --- HARDWARE HANDLES ---
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim3;

// --- SYSTEM VARIABLES ---
#define BUFFER_SIZE 320 // 80 samples per cycle (40 Voltage, 40 Current)
volatile uint16_t adc_buffer[BUFFER_SIZE];
volatile uint8_t process_first_half = 0;
volatile uint8_t process_second_half = 0;

volatile double real_volts = 0;
volatile double real_amps = 0;
volatile double real_watts = 0;
volatile double apparent_power = 0;
volatile double power_factor = 0;

// --- FUNCTION PROTOTYPES ---
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);

// --- MAIN ---
int main(void) {
    HAL_Init();
    SystemClock_Config();

    // Initialize all hardware peripherals
    MX_GPIO_Init();
    MX_DMA_Init();   // MUST be initialized before ADC
    MX_ADC1_Init();
    MX_TIM3_Init();

    // Start ADC in DMA mode and Timer to trigger it
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, BUFFER_SIZE);
    HAL_TIM_Base_Start(&htim3);

    while (1) {
        if (process_first_half || process_second_half) {
            
            // Determine which half of the buffer is ready
            int start_idx = process_first_half ? 0 : (BUFFER_SIZE / 2);
            process_first_half = 0;
            process_second_half = 0;

            float v_sum = 0, i_sum = 0;
            
            // 1. Calculate dynamic DC Offset (Mean) for this exact 20ms cycle
            for(int i = 0; i < 80; i++) {
                v_sum += adc_buffer[start_idx + (i*2)];     
                i_sum += adc_buffer[start_idx + (i*2) + 1]; 
            }
            float v_mean = v_sum / 80.0;
            float i_mean = i_sum / 80.0;

           // 2. Shift, Square, and Instantaneous Power with Phase Compensation
            float sum_v_sq = 0, sum_i_sq = 0, sum_p_inst = 0;
            const float phase_cal = 0.027f; // Adjust this if your ADC clock changes

            for(int i = 0; i < 80; i++) {
                float v_shifted_current = (float)adc_buffer[start_idx + (i*2)] - v_mean;
                float i_shifted = (float)adc_buffer[start_idx + (i*2) + 1] - i_mean;
                float v_shifted_next;

                // Get the next voltage sample for interpolation
                if (i < 79) {
                    v_shifted_next = (float)adc_buffer[start_idx + ((i+1)*2)] - v_mean;
                } else {
                    // For the very last sample in the buffer, wrap around to the start
                    // (Assuming the waveform is continuous, the next sample is at index 0 or BUFFER_SIZE/2)
                    v_shifted_next = (float)adc_buffer[start_idx] - v_mean; 
                }

                // Interpolate what the voltage was at the exact microsecond the current was sampled
                float v_aligned = v_shifted_current + ((v_shifted_next - v_shifted_current) * phase_cal);

                sum_v_sq += (v_shifted_current * v_shifted_current);
                sum_i_sq += (i_shifted * i_shifted);
                
                // Calculate Active Power using the TIME-ALIGNED voltage
                sum_p_inst += (v_aligned * i_shifted); 
            }

            // 3. Root Mean Square
            float rms_v_raw = sqrtf(sum_v_sq / 80.0);
            float rms_i_raw = sqrtf(sum_i_sq / 80.0);

            // --- SMOOTHING FILTER ---
            static float smoothed_v_raw = 0;
            static float smoothed_i_raw = 0;
            static float smoothed_p_inst = 0;
            
            if (smoothed_v_raw == 0) { 
                smoothed_v_raw = rms_v_raw;
                smoothed_i_raw = rms_i_raw;
                smoothed_p_inst = (sum_p_inst / 80.0);
            } else {
                smoothed_v_raw = (smoothed_v_raw * 0.95) + (rms_v_raw * 0.05);
                smoothed_i_raw = (smoothed_i_raw * 0.95) + (rms_i_raw * 0.05);
                smoothed_p_inst = (smoothed_p_inst * 0.95) + ((sum_p_inst / 80.0) * 0.05);
            }

           // 4. Calibrate to Real Units
            real_volts = smoothed_v_raw * 0.6557f; 
            float raw_amps = (smoothed_i_raw * 3.3f / 4095.0f) / 0.185f; 

            // --- QUADRATIC NOISE SUBTRACTION ---
            float noise_floor_amps = 0.16f; // The exact baseline noise you see at 0W

            if (raw_amps > noise_floor_amps) {
                // Subtract the noise squared from the signal squared
                real_amps = sqrtf((raw_amps * raw_amps) - (noise_floor_amps * noise_floor_amps));
            } else {
                real_amps = 0.0f; // True deadband for 0W state
            }

            // 5. Power Math
            // Note: real_watts uses the RAW smoothed power, because the noise naturally 
            // averages out in the instantaneous (V * I) calculation.
            real_watts = smoothed_p_inst * 0.6557f * ((3.3f / 4095.0f) / 0.185f); 
            
            // Ensure real_watts doesn't show negative floating point drift at 0W
            if (real_amps == 0.0f) real_watts = 0.0f;

            apparent_power = real_volts * real_amps;
            power_factor = (apparent_power > 0) ? (real_watts / apparent_power) : 0;
            
            // Clamp Power Factor to 1.0 (sometimes noise can push it to 1.01)
            if (power_factor > 1.0f) power_factor = 1.0f;

            // 6. SMPS Protection Logic (Mapped to PA2, PA3, PA4, PA5)
            if (real_volts > 260.0 || real_volts < 190.0 || real_amps > 4.5) {
                // CRITICAL FAULT
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   // Red ON
                //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);   // Buzzer ON
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Orange OFF
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); // Green OFF
            } 
            else if ((real_volts >= 190.0 && real_volts < 210.0) || (real_volts > 250.0 && real_volts <= 260.0)) {
                // WARNING STRESS
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // Orange ON
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
            } 
            else {
                // HEALTHY
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);   // Green ON
            }
        }
    }
}

// --- INITIALIZATION FUNCTIONS ---

static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // MISSING CODE: Configure PA0 and PA1 as Analog for sensors
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PA2(Buzzer), PA3(Green), PA4(Orange), PA5(Red)
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_DMA_Init(void) {
    __HAL_RCC_DMA2_CLK_ENABLE();
    // ADC1 uses DMA2, Stream 0, Channel 0 on STM32F4
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

static void MX_ADC1_Init(void) {
    __HAL_RCC_ADC1_CLK_ENABLE();
    
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE; // REQUIRED for multiple channels
    hadc1.Init.ContinuousConvMode = DISABLE; // Let timer trigger it
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO; // Triggered by TIM3
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 2; // Voltage and Current
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = DISABLE;
    HAL_ADC_Init(&hadc1);

    // Link DMA to ADC
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR; // Loops automatically
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_adc1);
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    // Rank 1: Voltage (PA0)
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // Rank 2: Current (PA1)
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = 2;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

static void MX_TIM3_Init(void) {
    __HAL_RCC_TIM3_CLK_ENABLE();
    
    // Assuming 16MHz default HSI clock. 16,000,000 / 16 / 250 = 4000 Hz
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 16 - 1; 
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 250 - 1;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim3);

    // Set trigger output (TRGO) to fire on Update Event
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
}

void SystemClock_Config(void) {
    // Basic HSI config (16MHz) standard for PlatformIO if no HSE is specified
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

// --- INTERRUPT HANDLERS & CALLBACKS ---

// DMA IRQ Handler MUST be present in PlatformIO to clear the flags
void DMA2_Stream0_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_adc1);
}

// HAL Tick handler
void SysTick_Handler(void) {
    HAL_IncTick();
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc->Instance == ADC1) process_first_half = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc->Instance == ADC1) process_second_half = 1;
}