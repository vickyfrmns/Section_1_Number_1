#include <string.h>
#include <stdio.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_gpio.h"

UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim3;
ADC_HandleTypeDef hadc1;

uint32_t adc_value = 0;
uint8_t received_data;

void SystemClock_Config(void);
void GPIO_Init(void);
void UART2_Init(void);
void ADC1_Init(void);
void TIM3_Init(void);
void UART_Receive_Data(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    UART2_Init();
    ADC1_Init();
    TIM3_Init();

    HAL_TIM_Base_Start_IT(&htim3); // Start timer with Interupt
    HAL_ADC_Start(&hadc1);

    HAL_UART_Receive_IT(&huart2, &received_data, 1); // Receive UART with Interupt

    while (1)
    {
        __WFI(); /// Change to mode sleep untill interrupt started
    }
}

/**
 * Callback for Interrupt Timer (Activate Send and Recieve with UART).
 * @param htim    parameter to start configuration Timer with Interrupt
 *
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) // Make sure timer on TIM3
    {
        // Read ADC Value
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        adc_value = HAL_ADC_GetValue(&hadc1);

        // Send ADC value across UART
        char buffer[50];
        sprintf(buffer, "ADC Value: %lu\r\n", adc_value);

        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

        // Receive data from UART
        UART_Receive_Data();
    }
}

// Function to receive data with interrupt
void UART_Receive_Data()
{
    HAL_UART_Receive_IT(&huart2, &received_data, 1); // Receive UART using Interupt
    char buffer[50];
    sprintf(buffer, "Data diterima: %c\r\n", received_data);
}

// KConfiguration Clock System for STM32F4
void SystemClock_Config()
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

// GPIO Initialization
void GPIO_Init()
{
    __HAL_RCC_GPIOA_CLK_ENABLE(); // Active RCC Clock for GPIOA

    // GPIO Configuration for UART pin
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3; // Decalre PA2 for TX and PA3 for RX
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2; // Pin GPIOA 2 and 3 use GPIO_AF7 based on datasheet
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // GPIO Configuration for ADC pin
    GPIO_InitStruct.Pin = GPIO_PIN_1; // Decalre PA1 for Analog Mode (ADC mode)
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// UART2 Initialization
void UART2_Init()
{
    __HAL_RCC_USART2_CLK_ENABLE(); // Active RCC Clock for UART2
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

// ADC1 Initialization
void ADC1_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE(); // Active RCC Clock for ADC1
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

// Inisialisasi Timer3
void TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE(); // Active RCC Clock for TIM3

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 8400 - 1; // 42 MHz / 8400 = 5000 Hz (each tick 0.2 ms)
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 2500 - 1; // 5000 Hz / 2500 = 500 ms (2500 ticks)
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim3);
}
