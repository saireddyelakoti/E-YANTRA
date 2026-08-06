/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * Team ID         :3568
 * Author list     :E.Venkata Sai Sudharshan Reddy, P.Kusuma Venkata Siva Sai, K.Prakash Reddy, P.Akhil Naidu.
 * @file           : main.c(Task6)
 * @brief          : 3-box PID line follower – random colour placement
 * Theme           : eYRC Crop Drop Bot(CB)
 * Functions       :SystemClock_Config, MX_GPIO_Init, MX_DMA_Init, MX_USART2_UART_Init, MX_ADC1_Init, MX_TIM2_Init, MX_TIM3_Init, NormalizeReflectance, CalculatePosition, ComputePID, SetMotorSpeeds,FloatToString, Check_Surface_Mode, Local_Color_Detect, Do_Pickup_Routine, SetColorLED,HAL_ADC_ConvCpltCallback, Error_Handler, assert_failed.

 * Global Variables: ADC_HandleTypeDef hadc1, DMA_HandleTypeDef hdma_adc1, TIM_HandleTypeDef htim2, TIM_HandleTypeDef htim3, UART_HandleTypeDef huart2, uint16_t adcBuffer[5], float reflectance[5], float voltages[5], float integral, float prev_error, float filtered_position, volatile uint8_t adc_complete, RobotState current_state, bool is_black_line_mode, InternalBoxColor picked_color, uint8_t boxes_delivered, uint8_t scan_count, InternalBoxColor scanned_colors[2], static float prev_reflect[5].
 *
 *  Drop-zone routing (decided by colour sensor at pickup, not search order):
 *    RED  → node 1 on black line → right turn → drop
 *    BLUE → node 2 on black line → left  turn → drop
 *    GREEN→ node 3 on black line → right turn → drop
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
#include "Color.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 *  Simplified 7-state machine.
 *  The same STATE_SEARCH / STATE_RETURN / STATE_BRANCH_OUT / STATE_BRANCH_RET
 *  are reused for all 3 trips.  Routing is driven by picked_color, not by
 *  which trip number we are on.
 */
typedef enum {
  STATE_PRE_SCAN,     // Scan first 2 boxes WITHOUT picking up; count colours
  STATE_SEARCH,       // Follow white line, detect & pick up 3rd box
  STATE_RETURN,       // Carry box on black line, count nodes to drop zone
  STATE_BRANCH_OUT,   // Drive down branch to drop point, release box
  STATE_BRANCH_RET,   // Spin and return from branch back to main line
  STATE_TRANSITION,   // Cross black-line area → white-line source area
  STATE_FINAL_RETURN, // All 3 boxes dropped; return to start & stop
  STATE_DONE,         // Motors off, white LED solid ON
  STATE_BLINK         // Blink white LED after final stop
} RobotState;

typedef enum { BOX_NONE = 0, BOX_RED, BOX_GREEN, BOX_BLUE } InternalBoxColor;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ── PID Gains ──────────────────────────────────────────────────────────────
#define KP 60.0f
#define KI 0.05f
#define KD 30.0f

#define KP_WL 180.00f
#define KI_WL 10.095f
#define KD_WL 45.500f

// ── Speed Settings ──────────────────────────────────────────────────────────
#define BASE_SPEED 250
#define BASE_SPEED_WL 270
#define MAX_TURN_DIFF 400

// ── Turning Assistance – Black Line ─────────────────────────────────────────
#define SHARP_TURN_THRESH_BL 0.8f
#define TURN_BOOST_BL 2.50f
#define HARD_TURN_ERR_BL 0.6f
#define EXTREME_TURN_ERR_BL 1.8f
#define HARD_TURN_ADD_BL 180
#define HARD_TURN_SUB_BL 120
#define EXTREME_TURN_L_BL 255
#define EXTREME_TURN_R_BL -250

// ── Turning Assistance – White Line ─────────────────────────────────────────
#define SHARP_TURN_THRESH_WL 0.9f
#define TURN_BOOST_WL 3.0f
#define HARD_TURN_ERR_WL 0.60f
#define EXTREME_TURN_ERR_WL 1.8f
#define HARD_TURN_ADD_WL 210
#define HARD_TURN_SUB_WL 190
#define EXTREME_TURN_L_WL 250
#define EXTREME_TURN_R_WL -250

// ── Line / Node Detection ───────────────────────────────────────────────────
#define LINE_LOST_THRESHOLD 12.0f
#define BLACK_LINE_THRESH 12.0f // reflectance % below this → on black line
#define WHITE_NODE_THRESH 60.0f // reflectance % above this → white-line node

// ── Mode Switching ──────────────────────────────────────────────────────────
#define SURFACE_THRESH 55.0f
#define MODE_DEBOUNCE_COUNT 8

// ── Control Loop ────────────────────────────────────────────────────────────
#define LOOP_DELAY_MS 1.5f
#define INTEGRAL_LIMIT 22.50f
#define FILTER_ALPHA 0.5f

// ── Color Detection Gains (used in Local_Color_Detect) ──────────────────────
#ifndef COLOR_R_GAIN
#define COLOR_R_GAIN 1.8f
#endif
#ifndef COLOR_G_GAIN
#define COLOR_G_GAIN 2.9f
#endif
#ifndef COLOR_B_GAIN
#define COLOR_B_GAIN 1.99f
#endif
#ifndef COLOR_DIFF
#define COLOR_DIFF 200
#endif

// ── Sensor Calibration ──────────────────────────────────────────────────────
const uint16_t black_min_WL[5] = {3250, 3295, 3200, 3200, 1450};
const uint16_t white_max_WL[5] = {4095, 4095, 4095, 4095, 4000};

const uint16_t black_min_BL[5] = {3450,3295, 3680, 3920, 1450};
const uint16_t white_max_BL[5] = {4095, 4095, 4095, 4095, 4095};

const float weights[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

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

/* USER CODE BEGIN PV */
uint16_t adcBuffer[5];
float reflectance[5];
float voltages[5];
float integral = 0.0f;
float prev_error = 0.0f;
float filtered_position = 0.0f;
volatile uint8_t adc_complete = 0;

RobotState current_state = STATE_PRE_SCAN; /* start in pre-scan phase */
bool is_black_line_mode = false;
InternalBoxColor picked_color = BOX_NONE;
uint8_t boxes_delivered = 0; /* counts delivered boxes (0→3) */
uint8_t scan_count = 0;      /* how many boxes scanned without pickup */
InternalBoxColor scanned_colors[2] = {BOX_NONE, BOX_NONE}; /* colours seen */

static float prev_reflect[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
float NormalizeReflectance(int idx, uint16_t raw, bool black_line_mode);
float CalculatePosition(float *den_out, bool black_line);
float ComputePID(float error, float *integral, float *prev_error, float dt,
                 float kp, float ki, float kd);
void SetMotorSpeeds(int16_t left, int16_t right);
char *FloatToString(float value, int precision, char *buf, int sz);
void Check_Surface_Mode(void);
InternalBoxColor Local_Color_Detect(void);
void Do_Pickup_Routine(void);
void SetColorLED(InternalBoxColor color);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ────────────────────────────────────────────────────────────────────────────
 * SetColorLED
 * Uses the RGB LED pins defined in Color.h  (PC6 = R, PC8 = G, PC9 = B).
 * LEDs are COMMON ANODE → GPIO_PIN_RESET = ON, GPIO_PIN_SET = OFF.
 *
 *   BOX_RED   → red   LED only
 *   BOX_GREEN → green LED only
 *   BOX_BLUE  → blue  LED only
 *   BOX_NONE  → all 3 ON (white) – shown after each box is dropped
 * ────────────────────────────────────────────────────────────────────────── */
void SetColorLED(InternalBoxColor color) {
  /* Default: all OFF (common-anode → SET = off) */
  GPIO_PinState r = GPIO_PIN_SET;
  GPIO_PinState g = GPIO_PIN_SET;
  GPIO_PinState b = GPIO_PIN_SET;

  switch (color) {
  case BOX_RED:
    r = GPIO_PIN_RESET;
    break; /* only red on   */
  case BOX_GREEN:
    g = GPIO_PIN_RESET;
    break; /* only green on */
  case BOX_BLUE:
    b = GPIO_PIN_RESET;
    break; /* only blue on  */
  default: /* BOX_NONE → white (all on) */
    r = GPIO_PIN_RESET;
    g = GPIO_PIN_RESET;
    b = GPIO_PIN_RESET;
    break;
  }
  /* Write to the colour-sensor LED pins (defined in Color.h) */
  HAL_GPIO_WritePin(COLOR_LED_R_PORT, COLOR_LED_R_PIN, r); /* PC6 */
  HAL_GPIO_WritePin(COLOR_LED_G_PORT, COLOR_LED_G_PIN, g); /* PC8 */
  HAL_GPIO_WritePin(COLOR_LED_B_PORT, COLOR_LED_B_PIN, b); /* PC9 */
}

/* ────────────────────────────────────────────────────────────────────────────
 * Local_Color_Detect
 * Reads TCS3200 once per channel and identifies the dominant colour.
 *
 * WHY we do NOT call Color_Detect() here:
 *   Color_Detect() internally calls Color_ReadRed/Green/Blue() again,
 *   each with a 50 ms HAL_Delay → 150 ms extra.  Because the TCS3200
 *   IC_CaptureCallback updates the shared `frequency` variable on every
 *   rising edge, calling Read() twice returns stale data from the PREVIOUS
 *   filter selection, producing wrong colours (pink / yellow / sky-blue).
 *
 * Solution: one clean set of 3 reads, same gains/threshold as Color.c.
 * ────────────────────────────────────────────────────────────────────────── */
InternalBoxColor Local_Color_Detect(void) {
  /* Select each filter, wait 50 ms for the frequency to stabilise, read */
  uint32_t r = Color_ReadRed();   /* S2=L S3=L → Red   photodiodes */
  uint32_t g = Color_ReadGreen(); /* S2=H S3=H → Green photodiodes */
  uint32_t b = Color_ReadBlue();  /* S2=L S3=H → Blue  photodiodes */

  /* Apply the same normalisation gains as Color.c (from Color.h defines) */
  uint32_t r_n =
      (uint32_t)(r * COLOR_R_GAIN); /* = r * 1.8f  – red is low on TCS3200 */
  uint32_t g_n = (uint32_t)(g * COLOR_G_GAIN); /* = g * 2.9f  */
  uint32_t b_n = (uint32_t)(b * COLOR_B_GAIN); /* = b * 1.99f */

  /* Same thresholds as Color_Detect() in Color.c */
  if ((r_n > g_n + COLOR_DIFF - 100) && (r_n > b_n + COLOR_DIFF - 95))
    return BOX_RED;
  if ((g_n > r_n + COLOR_DIFF) && (g_n > b_n + COLOR_DIFF))
    return BOX_GREEN;
  if ((b_n > r_n + COLOR_DIFF) && (b_n > g_n + COLOR_DIFF))
    return BOX_BLUE;

  return BOX_NONE; /* unrecognised mix – robot keeps electromagnet ON */
}

/* ────────────────────────────────────────────────────────────────────────────
 * NormalizeReflectance  –  raw ADC → 0..100 %
 * ────────────────────────────────────────────────────────────────────────── */
float NormalizeReflectance(int idx, uint16_t raw, bool black_line_mode) {
  uint16_t min_v = black_line_mode ? black_min_BL[idx] : black_min_WL[idx];
  uint16_t max_v = black_line_mode ? white_max_BL[idx] : white_max_WL[idx];
  if (raw <= min_v)
    return 0.0f;
  if (raw >= max_v)
    return 100.0f;
  return ((float)(raw - min_v) / (float)(max_v - min_v)) * 100.0f;
}

/* ────────────────────────────────────────────────────────────────────────────
 * Check_Surface_Mode  –  debounced black/white surface detection
 * ────────────────────────────────────────────────────────────────────────── */
void Check_Surface_Mode(void) {
  float outer_avg = (reflectance[0] + reflectance[4]) / 2.0f;
  static uint32_t stable_count = 0;
  static bool potential_mode = false;

  bool detected_white_surface = (outer_avg > SURFACE_THRESH);

  if (detected_white_surface != is_black_line_mode) {
    if (detected_white_surface == potential_mode) {
      if (++stable_count > MODE_DEBOUNCE_COUNT) {
        is_black_line_mode = detected_white_surface;
        stable_count = 0;
        integral = 0.0f;
        prev_error = 0.0f;
      }
    } else {
      potential_mode = detected_white_surface;
      stable_count = 0;
    }
  } else {
    stable_count = 0;
  }
}

/* ────────────────────────────────────────────────────────────────────────────
 * CalculatePosition  –  weighted-average position in [-2, +2], 0 = centred
 * ────────────────────────────────────────────────────────────────────────── */
float CalculatePosition(float *den_out, bool black_line) {
  float num = 0.0f, den = 0.0f;
  for (int i = 0; i < 5; i++) {
    float s = black_line ? (100.0f - reflectance[i]) : reflectance[i];
    num += s * weights[i];
    den += s;
  }
  *den_out = den;

  float raw_pos = (den > 0.0f) ? (num / den) : 0.0f;
  if (den < LINE_LOST_THRESHOLD)
    raw_pos = filtered_position;

  float delta = raw_pos - filtered_position;
  if (fabsf(delta) > 0.5f)
    raw_pos = filtered_position + (delta > 0 ? 0.5f : -0.5f);

  filtered_position =
      FILTER_ALPHA * filtered_position + (1.0f - FILTER_ALPHA) * raw_pos;
  return filtered_position;
}

/* ────────────────────────────────────────────────────────────────────────────
 * SetMotorSpeeds  –  TIM2 CH1/CH2 = Left motor, CH3/CH4 = Right motor
 * ────────────────────────────────────────────────────────────────────────── */
void SetMotorSpeeds(int16_t left, int16_t right) {
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

/* ────────────────────────────────────────────────────────────────────────────
 * FloatToString  –  lightweight float formatter
 * ────────────────────────────────────────────────────────────────────────── */
char *FloatToString(float value, int precision, char *buf, int sz) {
  if (precision < 0)
    precision = 0;
  if (precision > 6)
    precision = 6;
  bool neg = (value < 0.0f);
  if (neg)
    value = -value;
  int ip = (int)value;
  int fp = (int)((value - ip) * powf(10.0f, precision) + 0.5f);
  snprintf(buf, sz, neg ? "-%d.%0*d" : "%d.%0*d", ip, precision, fp);
  return buf;
}

/* ────────────────────────────────────────────────────────────────────────────
 * ComputePID  –  discrete P·I·D with anti-windup
 * ────────────────────────────────────────────────────────────────────────── */
float ComputePID(float error, float *integral, float *prev_error, float dt,
                 float kp, float ki, float kd) {
  if (fabsf(error) > 1.5f)
    *integral = 0.0f;
  *integral += error * dt;
  if (*integral > INTEGRAL_LIMIT)
    *integral = INTEGRAL_LIMIT;
  else if (*integral < -INTEGRAL_LIMIT)
    *integral = -INTEGRAL_LIMIT;
  float deriv = (error - *prev_error) / dt;
  *prev_error = error;
  return kp * error + ki * (*integral) + kd * deriv;
}

/* ────────────────────────────────────────────────────────────────────────────
 * Do_Pickup_Routine
 *   1. Stop.
 *   2. Activate electromagnet.
 *   3. Detect colour → light matching LED.
 *   4. Spin 180° (same motion for every colour).
 *   5. Resume forward.
 * ────────────────────────────────────────────────────────────────────────── */
void Do_Pickup_Routine(void) {
  SetMotorSpeeds(0, 0);
  HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, GPIO_PIN_SET);
  picked_color = Local_Color_Detect();
  SetColorLED(picked_color); /* Show detected colour               */
  /* Skip the 180° spin for the 1st pickup (boxes_delivered == 0).
   * After the pre-scan, the robot already spun 180° and is facing
   * the drop zones, so no extra spin is needed.
   * For the 2nd and 3rd pickups the spin is required. */
  if (boxes_delivered >= 1) {
    SetMotorSpeeds(-255, 255); /* 180° spin */
    HAL_Delay(640);
  }
  SetMotorSpeeds(BASE_SPEED, BASE_SPEED);
}

/* USER CODE END 0 */

/**
 * @brief  Application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  Color_Init(&htim3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuffer, 5);
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */

    if (adc_complete) {
      adc_complete = 0;

      /* ── 1. Normalise & low-pass filter sensor readings ─────── */
      float den = 0.0f;
      for (int i = 0; i < 5; i++) {
        float raw = NormalizeReflectance(i, adcBuffer[i], is_black_line_mode);
        reflectance[i] =
            FILTER_ALPHA * raw + (1.0f - FILTER_ALPHA) * prev_reflect[i];
        prev_reflect[i] = reflectance[i];
        voltages[i] = (adcBuffer[i] / 4095.0f) * 3.3f;
        den += reflectance[i];
      }

      /* ── 2. Surface-mode detection ──────────────────────────── */
      Check_Surface_Mode();

      /* ── 3. State machine ───────────────────────────────────── */
      static int node_count = 0;
      static uint32_t last_node_time = 0;
      static uint32_t branch_timer = 0;

      switch (current_state) {
      /* ══════════════════════════════════════════════════════════
       * STATE_PRE_SCAN
       * Follow white line at full speed.
       * Each time box-detect fires: increment scan_count (no stop,
       * no colour read, no LED).
       * After 2 counts AND all sensors white: spin 180° → STATE_SEARCH.
       * ══════════════════════════════════════════════════════════ */
      case STATE_PRE_SCAN: {
        static uint32_t scan_debounce = 0;

        /* Quick count – no stop, no colour read */
        if (HAL_GPIO_ReadPin(Box_detect_GPIO_Port, Box_detect_Pin) ==
                GPIO_PIN_RESET &&
            (HAL_GetTick() - scan_debounce > 600)) {
          scan_count++;
          scan_debounce = HAL_GetTick();
        }

        /* After both boxes counted: wait for all-white, then spin + go */
        if (scan_count >= 2) {
          float total_reflect = 0.0f;
          for (int i = 0; i < 5; i++)
            total_reflect += reflectance[i];

          if ((total_reflect / 5.0f) > 70.0f) {
            SetMotorSpeeds(0, 0);
            HAL_Delay(00);
            SetMotorSpeeds(-255, 255); /* spin 180° */
            HAL_Delay(650);
            SetMotorSpeeds(BASE_SPEED_WL, BASE_SPEED_WL);
            HAL_Delay(00);
            current_state = STATE_SEARCH; /* 3rd box – pick it up */
          }
        }
        break;
      }
      /* ══════════════════════════════════════════════════════════
       *  STATE_SEARCH
       *  Follow white line until the box-detect sensor fires.
       *  Pickup, identify colour, light LED, spin 180°.
       * ══════════════════════════════════════════════════════════ */
      case STATE_SEARCH:
        if (HAL_GPIO_ReadPin(Box_detect_GPIO_Port, Box_detect_Pin) ==
            GPIO_PIN_RESET) {
          Do_Pickup_Routine(); /* stops, picks, lights LED, spins    */
          node_count = 0;
          last_node_time = HAL_GetTick();
          current_state = STATE_RETURN;
        }
        break;

      /* ══════════════════════════════════════════════════════════
       *  STATE_RETURN
       *  Navigate on the black line by counting nodes.
       *  Drop-zone routing decided entirely by picked_color:
       *
       *    RED   → node 1, turn RIGHT  (SetMotorSpeeds 250, -250)
       *    BLUE  → node 2, turn LEFT   (SetMotorSpeeds -255, 255)
       *    GREEN → node 3, turn RIGHT  (SetMotorSpeeds 250, -250)
       * ══════════════════════════════════════════════════════════ */
      case STATE_RETURN:
        if (is_black_line_mode) {
          int black_count = 0;
          for (int i = 0; i < 5; i++)
            if (reflectance[i] < BLACK_LINE_THRESH)
              black_count++;

          if (black_count >= 3 && (HAL_GetTick() - last_node_time > 400)) {
            node_count++;
            last_node_time = HAL_GetTick();

            /* Determine target node and turn direction for this colour */
            int target_node = 0;
            bool turn_right = true;

            if (picked_color == BOX_RED) {
              target_node = 1;
              turn_right = true;
            } else if (picked_color == BOX_BLUE) {
              target_node = 2;
              turn_right = false;
            } else if (picked_color == BOX_GREEN) {
              target_node = 3;
              turn_right = true;
            }

            if (node_count == target_node) {
              /* Creep forward a little so the front wheel clears the node */
              SetMotorSpeeds(BASE_SPEED, BASE_SPEED);
              HAL_Delay(120);

              /* Turn into the branch */
              if (turn_right) {
                SetMotorSpeeds(255, -255); /* right */
                HAL_Delay(360);
              } else {
                SetMotorSpeeds(-255, 255); /* left  */
                HAL_Delay(340);
              }

              current_state = STATE_BRANCH_OUT;
              branch_timer = HAL_GetTick();
            }
          }
        }
        break;

      /* ══════════════════════════════════════════════════════════
       *  STATE_BRANCH_OUT
       *  Follow the branch until the next node (the drop point).
       *  Release box → white LED.
       *  Spin 180° to face back → STATE_BRANCH_RET.
       *
       *  Guard time is colour-specific:
       *    GREEN → 800 ms (branch is farthest, node 3)
       *    RED / BLUE → 400 ms
       * ══════════════════════════════════════════════════════════ */
      case STATE_BRANCH_OUT:
        if (is_black_line_mode) {
          int black_count = 0;
          for (int i = 0; i < 5; i++)
            if (reflectance[i] < BLACK_LINE_THRESH)
              black_count++;

          /* Longer guard for GREEN so the junction doesn't false-trigger */
          uint32_t branch_guard = (picked_color == BOX_GREEN) ? 800u : 400u;

          if (black_count >= 3 &&
              (HAL_GetTick() - branch_timer > branch_guard)) {
            /* Drop box */
            SetMotorSpeeds(0, 0);
            HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin,
                              GPIO_PIN_RESET);
            SetColorLED(BOX_NONE); /* White LED: box delivered           */
            boxes_delivered++;

            HAL_Delay(20);
            SetMotorSpeeds(-255, 255);
            HAL_Delay(620); /* 180° spin         */
            SetMotorSpeeds(BASE_SPEED, BASE_SPEED);
            HAL_Delay(60);

            current_state = STATE_BRANCH_RET;
            branch_timer = HAL_GetTick();
          }
        }
        break;

      /* ══════════════════════════════════════════════════════════
       *  STATE_BRANCH_RET
       *  Follow line back until we hit the main-line junction.
       *  Re-align to face toward the source (white-line side).
       *
       *  Turn to re-enter main line:
       *    BLUE  (came via left turn)  → realign with RIGHT turn
       *    RED / GREEN (came via right)→ realign with LEFT turn
       *
       *  After re-alignment:
       *    boxes_delivered < 3 → STATE_TRANSITION  (get next box)
       *    boxes_delivered == 3→ STATE_FINAL_RETURN (all done)
       * ══════════════════════════════════════════════════════════ */
      case STATE_BRANCH_RET:
        if (is_black_line_mode) {
          int black_count = 0;
          for (int i = 0; i < 5; i++)
            if (reflectance[i] < BLACK_LINE_THRESH)
              black_count++;

          if (black_count >= 4 && (HAL_GetTick() - branch_timer > 400)) {
            SetMotorSpeeds(BASE_SPEED, BASE_SPEED);
            HAL_Delay(30);

            /* Re-align: opposite of how we entered the branch */
            if (picked_color == BOX_BLUE) {
              SetMotorSpeeds(255, -255);
              HAL_Delay(310); /* right re-align  */
            } else {
              /* RED and GREEN both entered via right turn                  */
              SetMotorSpeeds(-250, 250);
              HAL_Delay(350); /* left re-align   */
            }

            if (boxes_delivered >= 3)
              current_state = STATE_FINAL_RETURN;
            else
              current_state = STATE_TRANSITION;

            node_count = 0;
          }
        }
        break;

      /* ══════════════════════════════════════════════════════════
       *  STATE_TRANSITION
       *  Robot is on the black-line side; follow line until the
       *  surface switches to white (is_black_line_mode goes false).
       *  Debounce, then go back to STATE_SEARCH for the next box.
       * ══════════════════════════════════════════════════════════ */
      case STATE_TRANSITION:
        if (!is_black_line_mode) {
          static int t_confirm = 0;
          if (++t_confirm > 25) {
            t_confirm = 0;
            node_count = 0;
            current_state = STATE_SEARCH;
          }
        }
        break;

      /* ══════════════════════════════════════════════════════════
       *  STATE_FINAL_RETURN  (all 3 boxes delivered)
       *
       *  Timer-based stop: 12 seconds after entering this state
       *  (i.e. after exiting the 3rd drop node) the robot stops
       *  automatically regardless of sensor readings.
       *  The robot follows the line normally during this time.
       * ══════════════════════════════════════════════════════════ */
      case STATE_FINAL_RETURN: {
        static uint32_t entry_time = 0;
        if (entry_time == 0) {
          entry_time = HAL_GetTick();
        }
        uint32_t elapsed = HAL_GetTick() - entry_time;
        if (elapsed > 10500) {
          SetMotorSpeeds(0, 0);
          entry_time = 0;
          current_state = STATE_BLINK;
        }
        break;
      }

      /* ══════════════════════════════════════════════════════════
       * STATE_DONE – task complete (legacy, not used)
       * Motors off. White LED stays solid ON forever.
       * ══════════════════════════════════════════════════════════ */
      case STATE_DONE:
        SetMotorSpeeds(0, 0);
        SetColorLED(BOX_NONE); /* solid white LED */
        break;
      /* ══════════════════════════════════════════════════════════
       * STATE_BLINK – blink white LED every 0.5s after final stop
       * Motors off. Toggle white LED on/off.
       * ══════════════════════════════════════════════════════════ */
      case STATE_BLINK: {
        SetMotorSpeeds(0, 0);
        static uint32_t blink_time = 0;
        static bool led_on = true;
        if (blink_time == 0) {
          blink_time = HAL_GetTick();
          SetColorLED(BOX_NONE); /* start with white ON */
          led_on = true;
        }
        uint32_t elapsed = HAL_GetTick() - blink_time;
        if (elapsed >= 500) {
          blink_time = HAL_GetTick();
          if (led_on) {
            HAL_GPIO_WritePin(COLOR_LED_R_PORT, COLOR_LED_R_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(COLOR_LED_G_PORT, COLOR_LED_G_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(COLOR_LED_B_PORT, COLOR_LED_B_PIN, GPIO_PIN_SET);
            led_on = false;
          } else {
            SetColorLED(BOX_NONE);
            led_on = true;
          }
        }
        break;
      }

      } /* end switch */

      /* ── 4. PID line-following (disabled in DONE/BLINK states) ─────── */
      if (current_state != STATE_DONE && current_state != STATE_BLINK) {
        float pos_den = 0.0f;
        float position = CalculatePosition(&pos_den, is_black_line_mode);
        float error = position;
        float dt = LOOP_DELAY_MS / 1000.0f;

        float kp = is_black_line_mode ? KP : KP_WL;
        float ki = is_black_line_mode ? KI : KI_WL;
        float kd = is_black_line_mode ? KD : KD_WL;
        float pid = ComputePID(error, &integral, &prev_error, dt, kp, ki, kd);

        float sharp_thresh =
            is_black_line_mode ? SHARP_TURN_THRESH_BL : SHARP_TURN_THRESH_WL;
        float turn_boost = is_black_line_mode ? TURN_BOOST_BL : TURN_BOOST_WL;
        float hard_err =
            is_black_line_mode ? HARD_TURN_ERR_BL : HARD_TURN_ERR_WL;
        float extreme_err =
            is_black_line_mode ? EXTREME_TURN_ERR_BL : EXTREME_TURN_ERR_WL;
        int hard_add = is_black_line_mode ? HARD_TURN_ADD_BL : HARD_TURN_ADD_WL;
        int hard_sub = is_black_line_mode ? HARD_TURN_SUB_BL : HARD_TURN_SUB_WL;
        int ext_l = is_black_line_mode ? EXTREME_TURN_L_BL : EXTREME_TURN_L_WL;
        int ext_r = is_black_line_mode ? EXTREME_TURN_R_BL : EXTREME_TURN_R_WL;

        bool sharp = fabsf(error) > sharp_thresh;
        if (sharp)
          pid *= turn_boost;
        /* Curve base: lower than straight so robot can pivot at 90° corners */
        int16_t base =
            sharp ? 175 : (is_black_line_mode ? BASE_SPEED : BASE_SPEED_WL);
        /* Line-lost recovery: spin hard toward last known error direction */
        if (pos_den < LINE_LOST_THRESHOLD) {
          if (filtered_position > 0.0f) {
            SetMotorSpeeds(255, -255); /* spin right */
          } else {
            SetMotorSpeeds(-255, 255); /* spin left  */
          }
          goto skip_pid_output;
        }
        int16_t l = base + (int16_t)pid;
        int16_t r = base - (int16_t)pid;

        if (error > hard_err) {
          l += hard_add;
          r -= hard_sub;
        }
        if (error > extreme_err) {
          l = ext_l;
          r = ext_r;
        }
        if (error < -hard_err) {
          r += hard_add;
          l -= hard_sub;
        }

        if (l - r > MAX_TURN_DIFF) {
          l = base + MAX_TURN_DIFF / 2;
          r = base - MAX_TURN_DIFF / 2;
        }
        if (r - l > MAX_TURN_DIFF) {
          r = base + MAX_TURN_DIFF / 2;
          l = base - MAX_TURN_DIFF / 2;
        }

        if (l > 255)
          l = 255;
        if (l < -255)
          l = -255;
        if (r > 255)
          r = 255;
        if (r < -255)
          r = -255;

        SetMotorSpeeds(l, r);
      skip_pid_output:; /* line-lost recovery jumps here */
      }

      /* ── 5. UART debug output ───────────────────────────────── */
      char buf[128];
      snprintf(buf, sizeof(buf), "M:%s C:%d S:%d N:%d D:%d\r\n",
               is_black_line_mode ? "BLK" : "WHT", (int)picked_color,
               (int)current_state, node_count, (int)boxes_delivered);
      HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), HAL_MAX_DELAY);
    }

    HAL_Delay(LOOP_DELAY_MS);
    /* USER CODE END 3 */
  }
}

/**
 * @brief System Clock Configuration
 */
void SystemClock_Config(void) {
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
    Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    Error_Handler();

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    Error_Handler();
}

/* ── Peripheral Init Functions (CubeMX generated) ──────────────────────── */

static void MX_ADC1_Init(void) {
  /* USER CODE BEGIN ADC1_Init 0 */
  /* USER CODE END ADC1_Init 0 */
  ADC_ChannelConfTypeDef sConfig = {0};
  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
    Error_Handler();

  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    Error_Handler();
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    Error_Handler();
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    Error_Handler();
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    Error_Handler();
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    Error_Handler();
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */
}

static void MX_TIM2_Init(void) {
  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    Error_Handler();

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    Error_Handler();
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}

static void MX_TIM3_Init(void) {
  /* USER CODE BEGIN TIM3_Init 0 */
  /* USER CODE END TIM3_Init 0 */
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  /* USER CODE BEGIN TIM3_Init 1 */
  /* USER CODE END TIM3_Init 1 */

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
    Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    Error_Handler();

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
    Error_Handler();
  /* USER CODE BEGIN TIM3_Init 2 */
  /* USER CODE END TIM3_Init 2 */
}

static void MX_USART2_UART_Init(void) {
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
    Error_Handler();
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */
}

static void MX_DMA_Init(void) {
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, Electromagnet_Pin | LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, S0_Pin | S1_Pin | S2_Pin | S3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, RED_Pin | GREEN_Pin | BLUE_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = Electromagnet_Pin | LD2_Pin;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = S0_Pin | S1_Pin | S2_Pin | S3_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RED_Pin | GREEN_Pin | BLUE_Pin;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = Box_detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Box_detect_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1)
    adc_complete = 1;
}
/* USER CODE END 4 */

void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif
