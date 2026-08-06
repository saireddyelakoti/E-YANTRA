/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "q_table10.h"
#include <stdlib.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    int n_states;
    int n_actions;
    float lr;
    float gamma;
    float epsilon_start;
    float epsilon_min;
    float epsilon_decay;
    float epsilon;
    char* filename;
    float q_table[6][7];
    int action_list[7];
    float last_direction;
    int no_line_count;
    int stop_threshold;
    float prev_error;
    float smooth_error;
    float error_smoothing_alpha;
    float current_error;
    float error_sign;
    int prev_state;
} QLearningController;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_SENSORS         5
#define WHITE_THRESH        0.50f
#define FULL_WHITE_THRESH  0.85f
#define NO_LINE_SUM_THRESH  0.20f     // lowered → detect "no line" sooner
#define NO_LINE_MAX_THRESH  0.06f     // lowered → detect "no line" sooner
#define NO_LINE_TOLERANCE   60        // ~600 ms straight at 10 ms loop → good for zebra crossings
#define BASE_SPEED_SCALE    6.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

uint16_t adc_buf[NUM_SENSORS];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

void init_QLearningController(QLearningController *self);
int Get_state(QLearningController *self, float values[NUM_SENSORS]);
int choose_action(QLearningController *self, int state);
void perform_action(int action, float *left_speed, float *right_speed);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  QLearningController ql;
  init_QLearningController(&ql);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, NUM_SENSORS);

  srand(HAL_GetTick());

  while (1)
  {
    float values[NUM_SENSORS];
    for(int i = 0; i < NUM_SENSORS; i++) {
        values[i] = (float)adc_buf[i] / 4095.0f;
    }

    // Stop condition: all sensors see full white (finish line)
    int all_white = 1;
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (values[i] < FULL_WHITE_THRESH) {
            all_white = 0;
            break;
        }
    }
    if (all_white) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
        while (1) HAL_Delay(1000);
    }

    int state = Get_state(&ql, values);
    int action = choose_action(&ql, state);

    // Update last direction for recovery bias
    if (action == 1 || action == 3) ql.last_direction = -1.0f;
    else if (action == 2 || action == 4) ql.last_direction = 1.0f;

    float left_speed, right_speed;
    perform_action(action, &left_speed, &right_speed);

    int left_pwm  = (int)((fabsf(left_speed)  / BASE_SPEED_SCALE) * 255.0f);
    int right_pwm = (int)((fabsf(right_speed) / BASE_SPEED_SCALE) * 255.0f);

    // Clamp PWM values
    if (left_pwm  > 255) left_pwm  = 255;
    if (left_pwm  <   0) left_pwm  =   0;
    if (right_pwm > 255) right_pwm = 255;
    if (right_pwm <   0) right_pwm =   0;

    // Apply to motors (forward / backward)
    if (left_speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, left_pwm);
    } else {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left_pwm);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    }

    if (right_speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, right_pwm);
    } else {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, right_pwm);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    }

    HAL_Delay(10);

    ql.prev_state = state;
  }
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*   Peripheral initialization functions                                      */
/* ─────────────────────────────────────────────────────────────────────────── */

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}

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

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

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
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

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
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;

  sConfig.Channel = ADC_CHANNEL_10; sConfig.Rank = ADC_REGULAR_RANK_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

  sConfig.Channel = ADC_CHANNEL_11; sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

  sConfig.Channel = ADC_CHANNEL_12; sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

  sConfig.Channel = ADC_CHANNEL_8;  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

  sConfig.Channel = ADC_CHANNEL_9;  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

  HAL_TIM_MspPostInit(&htim2);
}

static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
}

/* USER CODE BEGIN 4 */
void init_QLearningController(QLearningController *self) {
    self->n_states = 6;
    self->n_actions = 7;
    self->lr = 0.2f;
    self->gamma = 0.95f;
    self->epsilon_start = 0.5f;
    self->epsilon_min = 0.01f;
    self->epsilon_decay = 0.995f;
    self->epsilon = self->epsilon_start;
    self->filename = "q_table.pkl";

    for (int s = 0; s < 6; s++)
        for (int a = 0; a < 7; a++)
            self->q_table[s][a] = q_table[s][a];

    for (int i = 0; i < 7; i++)
        self->action_list[i] = i;

    self->last_direction = 1.0f;
    self->no_line_count = 0;
    self->stop_threshold = NO_LINE_TOLERANCE;
    self->prev_error = 0.0f;
    self->smooth_error = 0.0f;
    self->error_smoothing_alpha = 0.7f;
    self->prev_state = 2;
}

int Get_state(QLearningController *self, float values[NUM_SENSORS]) {
    float raw_sum = 0.0f, max_val = 0.0f;
    for (int i = 0; i < NUM_SENSORS; i++) {
        raw_sum += values[i];
        if (values[i] > max_val) max_val = values[i];
    }

    if (raw_sum < NO_LINE_SUM_THRESH || max_val < NO_LINE_MAX_THRESH) {
        self->current_error = 2.0f;
        self->no_line_count++;
        self->smooth_error = self->error_smoothing_alpha * self->smooth_error +
                             (1.0f - self->error_smoothing_alpha) * 2.0f;
        return 5;
    } else {
        self->no_line_count = 0;
    }

    float weighted_sum = 0.0f;
    for (int i = 0; i < NUM_SENSORS; i++)
        weighted_sum += (float)i * values[i];

    float position = (raw_sum > 0.0f) ? (weighted_sum / raw_sum) : 2.0f;
    position += 0.1f;  // small left bias to counter drift

    float center = 2.0f;
    float error = center - position;
    self->current_error = fabsf(error);
    self->error_sign = (error > 0) ? 1.0f : (error < 0 ? -1.0f : 0.0f);

    self->smooth_error = self->error_smoothing_alpha * self->smooth_error +
                         (1.0f - self->error_smoothing_alpha) * self->current_error;

    float abs_error = fabsf(error);
    if (abs_error > 0.2f) return (error > 0) ? 4 : 0;
    if (abs_error > 0.03f) return (error > 0) ? 3 : 1;
    return 2;
}

int choose_action(QLearningController *self, int state) {
    self->epsilon *= self->epsilon_decay;
    self->epsilon = fmaxf(self->epsilon, self->epsilon_min);

    // Immediate hard corrections when far off
    if (state == 0) return 4;
    if (state == 4) return 3;
    if (state == 1) return 2;
    if (state == 3) return 1;

    // ───────────────────────────── Dotted / Zebra line handling ─────────────────────────────
    if (state == 5) {
        if (self->no_line_count < NO_LINE_TOLERANCE) {
            return 0;                       // always straight during short gaps
        } else {
            // Long gap → very strong straight preference
            int rnd = rand() % 100;
            if (rnd < 94) {                 // 94% straight
                return 0;
            } else if (rnd < 97) {          // 3% gentle correction in last direction
                return (self->last_direction > 0.0f) ? 2 : 1;
            } else {                        // 3% gentle correction opposite direction
                return (self->last_direction > 0.0f) ? 1 : 2;
            }
        }
    }

    // Normal epsilon-greedy when line is visible
    if ((float)rand() / RAND_MAX < self->epsilon) {
        return rand() % self->n_actions;
    }

    // Greedy from Q-table
    float max_q = self->q_table[state][0];
    int best = 0;
    for (int a = 1; a < self->n_actions; a++) {
        if (self->q_table[state][a] > max_q) {
            max_q = self->q_table[state][a];
            best = a;
        }
    }
    return best;
}

void perform_action(int action, float *left_speed, float *right_speed) {
    float base    = 2.2f;      // lower base speed → more time to detect next stripe
    float mild    = 2.2f;
    float hard    = 3.4f;      // reduced hard turn → less overshoot

    switch (action) {
        case 0:  *left_speed = base;          *right_speed = base;         break;
        case 1:  *left_speed = base - mild;   *right_speed = base + mild;  break;
        case 2:  *left_speed = base + mild;   *right_speed = base - mild;  break;
        case 3:  *left_speed = base - hard;   *right_speed = base + hard;  break;
        case 4:  *left_speed = base + hard;   *right_speed = base - hard;  break;
        case 5:  *left_speed = 0.0f;          *right_speed = 0.0f;         break;
        case 6:  *left_speed = base * 1.15f;  *right_speed = base * 1.15f; break;
        default: *left_speed = 0.0f;          *right_speed = 0.0f;         break;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
