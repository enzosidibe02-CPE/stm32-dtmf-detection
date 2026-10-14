/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "arm_const_structs.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE     2048
#define SAMPLE_RATE     20000
#define BIN_HZ          ((float32_t)SAMPLE_RATE / (float32_t)BUFFER_SIZE)  // 9.77 Hz/bin

#define PASSCODE_LEN    4
#define TONE_TOLERANCE  45.0f      // matching tolerance
#define MAG_THRESHOLD   60.0f      // rejects below this threshold
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t  adc_buffer[BUFFER_SIZE];
float32_t fft_input[BUFFER_SIZE];
float32_t fft_output[BUFFER_SIZE];
float32_t fft_magnitude[BUFFER_SIZE / 2];

volatile uint8_t acquisition_done = 0;

/* DTMF reference frequencies */
static const float32_t low_freqs[4]  = { 697.0f, 770.0f, 852.0f,  941.0f };
static const float32_t high_freqs[4] = { 1209.0f, 1336.0f, 1477.0f, 1633.0f };

static const char dtmf_keypad[4][4] =
{
		{ '1', '2', '3', 'A' },
		{ '4', '5', '6', 'B' },
		{ '7', '8', '9', 'C' },
		{ '*', '0', '#', 'D' }
};

// Predefined access code
static const char passcode[PASSCODE_LEN + 1] = "1256";

// Passcode state
static char    sequence[PASSCODE_LEN + 1] = {0};
static uint8_t seq_index = 0;

// Debug print buffer
static char dbg[160];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_DAC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Send a string over USART2 (115200 baud)
void uart_print(const char *s)
	{
	HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), HAL_MAX_DELAY);
	}

// Runs FFT on adc_buffer and identify the DTMF key pressed and
// returns the keypad character, or '?' if no valid tone is detected

char processFFT(uint16_t *inBuffer, uint16_t size)
	{
	int32_t   sum = 0;
	float32_t mean;
	float32_t low_peak_mag = 0.0f, high_peak_mag = 0.0f;
	uint16_t  low_peak_bin = 0, high_peak_bin = 0;
	float32_t low_hz, high_hz;
	int8_t    row = -1, col = -1;
	uint16_t  raw_min = 0xFFFF, raw_max = 0;
	arm_rfft_fast_instance_f32 rfft;

	// DTMF band limits (in FFT bins)
	uint16_t low_start  = (uint16_t)(650.0f  / BIN_HZ);
	uint16_t low_end    = (uint16_t)(990.0f  / BIN_HZ);
	uint16_t high_start = (uint16_t)(1150.0f / BIN_HZ);
	uint16_t high_end   = (uint16_t)(1700.0f / BIN_HZ);

	// Compute DC offset (mean) and track signal range
	for (uint16_t n = 0; n < size; n++)
		{
		sum += (int32_t) inBuffer[n];
		if (inBuffer[n] < raw_min) raw_min = inBuffer[n];
		if (inBuffer[n] > raw_max) raw_max = inBuffer[n];
		}
	mean = (float32_t) sum / (float32_t) size;

	snprintf(dbg, sizeof(dbg),
		"  ADC: min=%u max=%u peak-peak=%u mean=%d\r\n",
		raw_min, raw_max, raw_max - raw_min, (int)mean);
	uart_print(dbg);

	// Remove DC offset, convert to float
	for (uint16_t n = 0; n < size; n++)
		{
		fft_input[n] = (float32_t) inBuffer[n] - mean;
		}

	// Do Real FFT
	arm_rfft_fast_init_f32(&rfft, BUFFER_SIZE);
	arm_rfft_fast_f32(&rfft, fft_input, fft_output, 0);

	// Zero out the Nyquist bin packed into output[1]
	fft_output[1] = 0.0f;

	// Magnitude spectrum
	arm_cmplx_mag_f32(fft_output, fft_magnitude, BUFFER_SIZE / 2);
	fft_magnitude[0] = 0.0f;

	// Search low DTMF band
	for (uint16_t k = low_start; k <= low_end; k++)
		{
		if (fft_magnitude[k] > low_peak_mag)
			{
			low_peak_mag = fft_magnitude[k];
			low_peak_bin = k;
			}
		}

	// Search high DTMF band
	for (uint16_t k = high_start; k <= high_end; k++)
		{
		if (fft_magnitude[k] > high_peak_mag)
			{
			high_peak_mag = fft_magnitude[k];
			high_peak_bin = k;
			}
		}

	low_hz  = (float32_t) low_peak_bin  * BIN_HZ;
	high_hz = (float32_t) high_peak_bin * BIN_HZ;

	snprintf(dbg, sizeof(dbg),
		"  FFT: low_peak=%dHz mag=%d  |  high_peak=%dHz mag=%d\r\n",
		(int)low_hz, (int)low_peak_mag,
		(int)high_hz, (int)high_peak_mag);
	uart_print(dbg);

	// Reject weak signals
	if (low_peak_mag < MAG_THRESHOLD || high_peak_mag < MAG_THRESHOLD)
		{
		uart_print("  REJECT: signal below MAG_THRESHOLD\r\n");
		return '?';
		}

	// Match low frequency
	for (int8_t i = 0; i < 4; i++)
		{
		if (fabsf(low_hz - low_freqs[i]) <= TONE_TOLERANCE)
			{
			row = i;
			break;
			}
		}

	// Match high frequency
	for (int8_t i = 0; i < 4; i++)
		{
		if (fabsf(high_hz - high_freqs[i]) <= TONE_TOLERANCE)
			{
			col = i;
			break;
			}
		}

	if (row < 0 || col < 0)
		{
		snprintf(dbg, sizeof(dbg),
			"  REJECT: no DTMF match (row=%d col=%d, tol=%dHz)\r\n",
			row, col, (int)TONE_TOLERANCE);
		uart_print(dbg);
		return '?';
		}

	return dtmf_keypad[row][col];
	}

// LD2 solid ON for 2 seconds (correct passcode)
void feedback_correct(void)
	{
	uart_print(">>> ACCESS GRANTED\r\n\r\n");
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	}

//  Blink LD2 at 5 Hz for 1 second (incorrect passcode)
void feedback_incorrect(void)
	{
	uart_print(">>> ACCESS DENIED\r\n\r\n");
	for (uint8_t i = 0; i < 10; i++)
		{
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		HAL_Delay(100);
		}
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
	uart_print(">> Button pressed\r\n");
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, BUFFER_SIZE);
	}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
	{
	HAL_ADC_Stop_DMA(&hadc1);
	acquisition_done = 1;
	}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_TIM_Base_Start(&htim6);

  uart_print("\r\n=== DTMF Entry System (DEBUG) ===\r\n");
  uart_print("Fs = 20 kHz, FFT = 2048, bin = ~9.77 Hz\r\n");
  uart_print("Passcode: 6\r\n");
  uart_print("Press B1 once per digit while playing the tone.\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while(1) {
	if (acquisition_done)
		{
		acquisition_done = 0;

		uart_print("--- Capture complete, running FFT ---\r\n");

		char key = processFFT(adc_buffer, BUFFER_SIZE);

		if (key != '?')
			{
			snprintf(dbg, sizeof(dbg), "  KEY DETECTED: '%c'\r\n", key);
			uart_print(dbg);

			sequence[seq_index++] = key;
			sequence[seq_index]   = '\0';

			snprintf(dbg, sizeof(dbg),
				"  Sequence: %s  (%u/%u)\r\n",
				sequence, seq_index, PASSCODE_LEN);
			uart_print(dbg);

			if (seq_index >= PASSCODE_LEN)
				{
				if (strcmp(sequence, passcode) == 0)
					{
					feedback_correct();
					}
				else
					{
					feedback_incorrect();
					}
				seq_index = 0;
				memset(sequence, 0, sizeof(sequence));
				}
			}
		uart_print("\r\n");
		}
  /* USER CODE BEGIN 3 */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* USER CODE END WHILE */
	}

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 79;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 49;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
