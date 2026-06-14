/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    main.c
 * @brief   DTMF Audio-Activated Entry System
 *          STM32 Nucleo-L476RG + TI BoostXL-Audio shield
 *
 *          Samples audio at 20 kHz via ADC1 + DMA, runs a 2048-point real FFT
 *          (CMSIS-DSP), detects DTMF tone pairs by peak search in the low/high
 *          frequency bands, maps to a keypad character, and validates a 4-digit
 *          passcode. Correct -> LD2 solid 2 s. Incorrect -> LD2 blinks.
 *
 * @author  Etienne Enzo Sidibe
 * @course  CPE 3500 - Embedded Digital Signal Processing
 ******************************************************************************
 * NOTE: This file was reconstructed from the project report. The STM32CubeMX
 * peripheral init functions (SystemClock_Config, MX_*_Init) are declared but
 * their bodies must be regenerated from the .ioc file in CubeIDE, or pasted
 * back in from the original project. The DSP / application logic below is
 * complete as authored.
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "arm_const_structs.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define BUFFER_SIZE     2048
#define SAMPLE_RATE     20000
#define BIN_HZ          ((float32_t)SAMPLE_RATE / (float32_t)BUFFER_SIZE)  // 9.77 Hz/bin

#define PASSCODE_LEN    4
#define TONE_TOLERANCE  45.0f    // matching tolerance
#define MAG_THRESHOLD   60.0f    // rejects below this threshold
/* USER CODE END PD */

/* Peripheral handles -------------------------------------------------------*/
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

/* Private function prototypes ----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_DAC1_Init(void);

/* USER CODE BEGIN 0 */

// Send a string over USART2 (115200 baud)
void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), HAL_MAX_DELAY);
}

// Runs FFT on adc_buffer and identifies the DTMF key pressed and
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
        sum += (int32_t)inBuffer[n];
        if (inBuffer[n] < raw_min) raw_min = inBuffer[n];
        if (inBuffer[n] > raw_max) raw_max = inBuffer[n];
    }
    mean = (float32_t)sum / (float32_t)size;

    snprintf(dbg, sizeof(dbg),
        "  ADC: min=%u max=%u peak-peak=%u mean=%d\r\n",
        raw_min, raw_max, raw_max - raw_min, (int)mean);
    uart_print(dbg);

    // Remove DC offset, convert to float
    for (uint16_t n = 0; n < size; n++)
    {
        fft_input[n] = (float32_t)inBuffer[n] - mean;
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

    low_hz  = (float32_t)low_peak_bin  * BIN_HZ;
    high_hz = (float32_t)high_peak_bin * BIN_HZ;

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

// Blink LD2 at 5 Hz for 1 second (incorrect passcode)
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

// Button press starts a capture
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uart_print(">> Button pressed\r\n");
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, BUFFER_SIZE);
}

// ADC/DMA transfer-complete: a full buffer is ready
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    HAL_ADC_Stop_DMA(&hadc1);
    acquisition_done = 1;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART2_UART_Init();
    MX_ADC1_Init();
    MX_TIM6_Init();
    MX_DAC1_Init();

    /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start(&htim6);
    uart_print("Press B1 once per digit while playing the tone.\r\n\r\n");
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
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
    }
    /* USER CODE END WHILE */
}

/* USER CODE BEGIN 4 */
/*
 * Peripheral init functions (SystemClock_Config, MX_GPIO_Init, MX_DMA_Init,
 * MX_USART2_UART_Init, MX_ADC1_Init, MX_TIM6_Init, MX_DAC1_Init) are generated
 * by STM32CubeMX. Key configuration from the report:
 *
 *   ADC1  : 12-bit, single channel IN5/PA0, ext-trigger TIM6 TRGO rising edge,
 *           DMA1 Ch1, half-word width, normal mode.
 *   TIM6  : PSC=79, ARR=49, master output trigger on update -> 20 kHz TRGO.
 *   DMA1  : Ch1, peripheral-to-memory, half-word, normal mode, high priority,
 *           transfer-complete interrupt enabled.
 *   GPIO  : PA5 (LD2) output push-pull. PC13 (B1) EXTI falling edge,
 *           EXTI15_10_IRQn.
 *   USART2: 115200 8N1, async, debug console over ST-Link VCP.
 *
 * Regenerate these from the .ioc file in STM32CubeIDE.
 */
/* USER CODE END 4 */
