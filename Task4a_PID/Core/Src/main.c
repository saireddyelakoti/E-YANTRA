/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file : main.c
  * @brief : CropDrop Bot (CB) - eYRC 2025-26 PID Line Follower
  * White line on black surface - Continuous following (no stops)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// PID parameters - tuned for better responsiveness on turns: higher KP for quicker correction
#define KP 115.7f // Increased to 50 for faster response to errors
#define KI 0.028f // Keep low to avoid windup
#define KD 200.0f // Good damping
// Base speed increased for momentum
#define BASE_SPEED 50 // Higher base for speed on straights
// Reduced base speed during sharp turns, but higher than before for control
#define TURN_BASE_SPEED 400 // Increased from 30 to maintain some speed in turns
// Increased turn difference for sharper turns
#define MAX_TURN_DIFF 500 // Slightly higher for aggressive corrections
// Turn boost: lower threshold to trigger earlier, higher factor for stronger turns
#define SHARP_TURN_THRESHOLD 0.8f // Lowered to trigger boost sooner
#define TURN_BOOST_FACTOR 2.2f // Increased for more aggressive turning
// Position smoothing filter
#define FILTER_ALPHA 0.45f // 95% previous + 5% new for smooth position
// Calibration (tune black_min/white_max based on your hardware readings if needed)
const uint16_t black_min[5] = {2250, 2400, 2600, 2400, 1400};
const uint16_t white_max[5] = {4095, 3800, 4095, 4095, 4095};
// Weights: negative left, positive right
const float weights[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
// Loop timing: slightly faster for responsiveness
#define LOOP_DELAY_MS 8
// Lost line threshold: lowered to be less sensitive (allow lower den at turns)
#define LINE_LOST_THRESHOLD 15.0f // Lowered to handle thin lines/turns better
// Integral windup limit: tighter
#define INTEGRAL_LIMIT 25.0f // Slightly tighter
// Stop condition: unchanged
#define WHITE_THRESHOLD 100.0f
/* USER CODE END PD */
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */
uint16_t adcBuffer[5];
float reflectance[5];
float integral = 0.0f;
float prev_error = 0.0f;
float filtered_position = 0.0f;
volatile uint8_t adc_complete = 0;
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
float NormalizeReflectance(int idx, uint16_t raw);
float CalculatePosition(float *den_out);
void SetMotorSpeeds(int16_t left, int16_t right);
char* FloatToString(float value, int precision, char* buffer, int buf_size); // New: Custom float printer
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float NormalizeReflectance(int idx, uint16_t raw)
{
  uint16_t min_v = black_min[idx];
  uint16_t max_v = white_max[idx];
  if (raw <= min_v) return 0.0f;
  if (raw >= max_v) return 100.0f;
  return ((float)(raw - min_v) / (max_v - min_v)) * 100.0f;
}
float CalculatePosition(float *den_out)
{
  float num = 0.0f, den = 0.0f;
  for (int i = 0; i < 5; i++) {
    float strength = reflectance[i];
    num += strength * weights[i];
    den += strength;
  }
  *den_out = den;
  float raw_pos = (den < LINE_LOST_THRESHOLD) ? filtered_position : (den > 0.0f ? (num / den) : 0.0f);
  // Rate limit: slightly looser for turns (allow up to 0.6 jump)
  float delta = raw_pos - filtered_position;
  if (fabsf(delta) > 0.6f) {
    raw_pos = filtered_position + (delta > 0 ? 0.6f : -0.6f);
  }
  // Apply low-pass filter for smoothness
  filtered_position = FILTER_ALPHA * filtered_position + (1.0f - FILTER_ALPHA) * raw_pos;
  return filtered_position;
}
void SetMotorSpeeds(int16_t left, int16_t right)
{
  if (left < 0) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint8_t)(-left));
  } else {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint8_t)left);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  }
  if (right < 0) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint8_t)(-right));
  } else {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint8_t)right);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
  }
}

// New function: Custom float to string converter (avoids %f issues in lightweight printf)
char* FloatToString(float value, int precision, char* buffer, int buf_size) {
  if (precision < 0) precision = 0;
  if (precision > 6) precision = 6; // Limit to reasonable precision

  // Handle sign
  bool negative = value < 0.0f;
  if (negative) value = -value;

  // Extract integer part
  int int_part = (int)value;
  value -= int_part;

  // Extract fractional part (scale to int)
  int frac_part = (int)(value * powf(10.0f, precision) + 0.5f); // Round

  // Build string
  if (negative) {
    snprintf(buffer, buf_size, "-%d.%0*d", int_part, precision, frac_part);
  } else {
    snprintf(buffer, buf_size, "%d.%0*d", int_part, precision, frac_part);
  }

  // Handle -0.00
  if (int_part == 0 && frac_part == 0 && negative) {
    buffer[0] = '0';
    buffer[1] = '.';
    for (int i = 0; i < precision; i++) buffer[2 + i] = '0';
    buffer[2 + precision] = '\0';
  }

  return buffer;
}
/* USER CODE END 0 */
/**
  * @brief The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, 5);
  /* USER CODE END 2 */
  /* Infinite loop */
  while (1)
  {
    if (adc_complete) {
        adc_complete = 0;
        for (int i = 0; i < 5; i++) {
            reflectance[i] = NormalizeReflectance(i, adcBuffer[i]);
        }
        // Check for stop condition: all sensors detect white (transition to full white area)
        bool stop_condition = true;
        for (int i = 0; i < 5; i++) {
            if (reflectance[i] < WHITE_THRESHOLD) {
                stop_condition = false;
                break;
            }
        }
        float den_local = 0.0f;
        char buf[300]; // Increased buffer size for safety with manual strings
        char pos_str[20], err_str[20], pid_str[20], den_str[20]; // Temp buffers for floats
        if (stop_condition) {
            // Stop the bot
            SetMotorSpeeds(0, 0);
            // Use integer-only snprintf for stop message
            snprintf(buf, sizeof(buf),
                     "IR 1=%4d IR 2=%4d IR 3=%4d IR 4=%4d IR 5=%4d | STOPPED (ALL WHITE)\r\n",
                     adcBuffer[0], adcBuffer[1], adcBuffer[2],
                     adcBuffer[3], adcBuffer[4]);
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
        } else {
            // Continue line following
            float position = CalculatePosition(&den_local);
            // Error sign: position positive = line right = turn right (left faster, right slower)
            float error = position; // Keep as is - confirmed correct
            float dt = LOOP_DELAY_MS / 1000.0f;
            // Anti-windup: reset integral if error too large (prevents over-correction at turns)
            if (fabsf(error) > 1.5f) {
                integral = 0.0f;
            }
            integral += error * dt;
            if (fabsf(integral) > INTEGRAL_LIMIT)
                integral = (integral > 0.0f ? INTEGRAL_LIMIT : -INTEGRAL_LIMIT);
            float derivative = (error - prev_error) / dt;
            prev_error = error;
            float pid_out = KP * error + KI * integral + KD * derivative;
            // Sharp turn boost: trigger earlier and stronger for better turn response
            bool is_sharp_turn = fabsf(error) > SHARP_TURN_THRESHOLD;
            if (is_sharp_turn) {
                pid_out *= TURN_BOOST_FACTOR;
            }
            // Base speed: reduce only on very sharp turns
            int16_t current_base_speed = is_sharp_turn ? TURN_BASE_SPEED : BASE_SPEED;
            int16_t left_speed = current_base_speed + (int16_t)pid_out;
            int16_t right_speed = current_base_speed - (int16_t)pid_out;
            // Straight-line boost: slight increase when centered (for speed on straights)
            if (fabsf(error) < 0.3f) {
                left_speed = BASE_SPEED + (int16_t)pid_out;
                right_speed = BASE_SPEED - (int16_t)pid_out;
            }
            // Limit differential
            int16_t diff = left_speed - right_speed;
            if (diff > MAX_TURN_DIFF) {
                left_speed = current_base_speed + MAX_TURN_DIFF / 2;
                right_speed = current_base_speed - MAX_TURN_DIFF / 2;
            } else if (diff < -MAX_TURN_DIFF) {
                left_speed = current_base_speed - MAX_TURN_DIFF / 2;
                right_speed = current_base_speed + MAX_TURN_DIFF / 2;
            }
            // Minimal reverse for recovery
            if (left_speed < -20) left_speed = -10;
            if (right_speed < -20) right_speed = -10;
            // Clamp
            if (left_speed > 255) left_speed = 255;
            if (right_speed > 255) right_speed = 255;
            SetMotorSpeeds(left_speed, right_speed);

            // Convert floats to strings using custom function
            FloatToString(position, 2, pos_str, sizeof(pos_str)); // %.2f
            FloatToString(error, 2, err_str, sizeof(err_str)); // %.2f
            FloatToString(pid_out, 1, pid_str, sizeof(pid_str)); // %.1f
            FloatToString(den_local, 1, den_str, sizeof(den_str)); // %.1f

            // Build the full string with integer snprintf + manual float inserts
            snprintf(buf, sizeof(buf),
                     "IR 1=%4d IR 2=%4d IR 3=%4d IR 4=%4d IR 5=%4d | Pos=%s Err=%s PID=%s L=%d R=%d Den=%s\r\n",
                     adcBuffer[0], adcBuffer[1], adcBuffer[2],
                     adcBuffer[3], adcBuffer[4], pos_str, err_str, pid_str, left_speed, right_speed, den_str);
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
        }
    }
    HAL_Delay(LOOP_DELAY_MS);
  }
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim2);
}
/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOA, Electromagnet_Pin|LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, S0_Pin|S1_Pin|S2_Pin|S3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, RED_Pin|GREEN_Pin|BLUE_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = Electromagnet_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = S0_Pin|S1_Pin|S2_Pin|S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = RED_Pin|GREEN_Pin|BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = Box_detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Box_detect_GPIO_Port, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        adc_complete = 1;
    }
}
/* USER CODE END 4 */
/**
  * @brief This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
