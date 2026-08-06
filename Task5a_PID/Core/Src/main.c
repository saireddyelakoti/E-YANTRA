/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file : main.c
 * @brief : Main program body
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
#include "q_table10.h"

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
    int prev_state;  // Added for dotted line continuity if needed
} QLearningController;


/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  STATE_SEARCH_1,
  STATE_RETURN_1,
  STATE_BRANCH_OUT_1, // Navigate branch for Box 1
  STATE_BRANCH_RET_1, // Return from branch for Box 1
  STATE_TRANSITION_TO_SOURCE,
  STATE_SEARCH_2,
  STATE_RETURN_2,
  STATE_BRANCH_OUT_2, // Navigate branch for Box 2
  STATE_DONE
} RobotState;

typedef enum { BOX_NONE = 0, BOX_RED, BOX_GREEN, BOX_BLUE } InternalBoxColor;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// PID Gains for Black Line on White Surface (BL mode)
#define KP 50.0f // Tuned: Between Snip(22) and Old(35)
#define KI 0.01f
#define KD 52.0f // Tuned: Between Snip(38) and Old(65)

// PID Gains for White Line on Black Surface (WL mode)
#define KP_WL 60.0f
#define KI_WL 0.11f
#define KD_WL 40.0f

// Speed Settings
// Speed Settings
#define BASE_SPEED 22
0    // Targeted for ~15s completion (2x snippet, 0.6x old)
#define BASE_SPEED_WL 140
#define TURN_BASE_SPEED 150
#define MAX_TURN_DIFF 250 // Limit set by User

// Turning Assistance
// (Handled by Mode Logic below)

// Control Loop
#define LOOP_DELAY_MS 1.5 // Faster loop for quicker reaction
#define INTEGRAL_LIMIT 20.0f
#define FILTER_ALPHA 0.5f // Reduced from 0.98 to 0.7 for FASTER response

// --- NEW SEPARATE TURN LOGIC CONSTANTS ---
// Black Line (BL) - Original "Perfect" values
#define SHARP_TURN_THRESH_BL 0.8f // Relaxed to snippet value
#define TURN_BOOST_BL 2.5f        // Reduced from 3.0 for stability
#define HARD_TURN_ERR_BL 0.6f
#define EXTREME_TURN_ERR_BL 1.8f
#define HARD_TURN_ADD_BL 100
#define HARD_TURN_SUB_BL 60
#define EXTREME_TURN_L_BL 220
#define EXTREME_TURN_R_BL -150

// White Line (WL) - Tuned for potentially different dynamics (Start with same,
// adjust here)
#define SHARP_TURN_THRESH_WL 0.8f
#define TURN_BOOST_WL 2.5f
#define HARD_TURN_ERR_WL 0.5f     // React sooner
#define EXTREME_TURN_ERR_WL 1.6f
#define HARD_TURN_ADD_WL 110 // Agreessive turn
#define HARD_TURN_SUB_WL 70
#define EXTREME_TURN_L_WL 230
#define EXTREME_TURN_R_WL -160


// ... (Rest of defines omitted for brevity in replace, but ensuring context
// matches if I can't replace scattered defines) Actually, sticking to the motor
// block replacement is safer if I can't guarantee define locations. I will just
// replace the Motor Logic block and assume I can update defines separately or
// just use literals/locals if needed. Wait, replace_file_content needs exact
// target match. I will update the DEFINES first.

// Line Detection
#define LINE_LOST_THRESHOLD 12.0f
#define BLACK_LINE_THRESH 13.0f

// Mode Switching Constants
#define SURFACE_THRESH 55.0f // Reduced from 80.0 for easier detection
#define MODE_DEBOUNCE_COUNT 15 // Reduced from 50 (400ms) to 12 (~96ms) for FAST switching

// RL Controller Macros
#define NUM_SENSORS 5
#define WHITE_THRESH 0.5f  // Threshold for white (high reflection)
#define FULL_WHITE_THRESH 0.85f
#define NO_LINE_SUM_THRESH 0.3f
#define NO_LINE_MAX_THRESH 0.1f
#define NO_LINE_TOLERANCE 5   // Tolerance for short no-line gaps (dotted lines)
#define BASE_SPEED_SCALE 5.0f  // Scale for PWM conversion

// Toggle between PID and RL (0 = PID, 1 = RL)
#define USE_RL_CONTROL 0 

// Calibration 1: White Line on Black Surface (WL)
const uint16_t black_min_WL[5] = {3250, 3250, 3200, 3200, 1450};
const uint16_t white_max_WL[5] = {4095, 4095, 4095, 4095, 4095};

// Calibration 2: Black Line on White Surface (BL)
// Note: User requested separate calibration. Adjust these values for the White
// Surface!
const uint16_t black_min_BL[5] = {3450, 4095, 3680, 3920, 1450};
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

RobotState current_state = STATE_SEARCH_1;
bool is_black_line_mode = false;
InternalBoxColor picked_color = BOX_NONE;

static float prev_reflect[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

QLearningController ql; // Global instance for RL
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

// RL Function Prototypes
void init_QLearningController(QLearningController *self);
int Get_state(QLearningController *self, float values[NUM_SENSORS]);
int choose_action(QLearningController *self, int state);
void perform_action(int action, float *left_speed, float *right_speed);
/* USER CODE BEGIN PFP */
float NormalizeReflectance(int idx, uint16_t raw, bool is_black_line_mode);
float CalculatePosition(float *den_out, bool black_line);
float ComputePID(float error, float *integral, float *prev_error, float dt,
                 float kp, float ki, float kd);
void SetMotorSpeeds(int16_t left, int16_t right);
char *FloatToString(float value, int precision, char *buffer, int buf_size);
void Check_Surface_Mode(void);
InternalBoxColor Local_Color_Detect(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Reimplement Color Logic Locally to avoid changing Color.h
InternalBoxColor Local_Color_Detect(void) {
  uint32_t r = Color_ReadRed();
  uint32_t g = Color_ReadGreen();
  uint32_t b = Color_ReadBlue();

  uint32_t r_n = r;
  uint32_t g_n = g * 2.9f;  // Same gain as Color.h
  uint32_t b_n = b * 1.99f; // Same gain as Color.h
  int diff = 110;           // Same diff

  // LEDs are manually controlled here if needed, or we rely on Color_Detect()
  // for visual
  Color_Detect();

  if ((r_n > g_n + diff - 100) && (r_n > b_n + diff - 95))
    return BOX_RED;
  else if ((g_n > r_n + diff) && (g_n > b_n + diff))
    return BOX_GREEN;
  else if ((b_n > r_n + diff) && (b_n > g_n + diff))
    return BOX_BLUE;

  return BOX_NONE;
}

float NormalizeReflectance(int idx, uint16_t raw, bool is_black_line_mode) {
  uint16_t min_v, max_v;

  if (is_black_line_mode) {
    // We are on White Surface (following Black Line)
    min_v = black_min_BL[idx];
    max_v = white_max_BL[idx];
  } else {
    // We are on Black Surface (following White Line)
    min_v = black_min_WL[idx];
    max_v = white_max_WL[idx];
  }

  if (raw <= min_v)
    return 0.0f;
  if (raw >= max_v)
    return 100.0f;
  return ((float)(raw - min_v) / (max_v - min_v)) * 100.0f;
}

void Check_Surface_Mode(void) {
  // Use Normalized Reflectance (0 = Black Reading, 100 = White Reading)
  // Average of outer sensors to determine background color
  float outer_avg = (reflectance[0] + reflectance[4]) / 2.0f;

  static uint32_t stable_count = 0;
  static bool potential_mode = false;

  // Logic:
  // If > THRESH (45) : White Surface (Background is White -> Black Line Mode)
  // If <= THRESH (45): Black Surface (Background is Black -> White Line Mode)
  bool detected_is_white_surface = (outer_avg > SURFACE_THRESH);

  // existing mode: is_black_line_mode (True = Black Line on White Surface)

  if (detected_is_white_surface != is_black_line_mode) {
    if (detected_is_white_surface == potential_mode) {
      stable_count++;
      if (stable_count > MODE_DEBOUNCE_COUNT) {
        // MODE CHANGE DETECTED!
        is_black_line_mode = detected_is_white_surface;
        stable_count = 0;

        // CRITICAL: Reset PID Memory so we don't carry error/integral
        integral = 0.0f;
        prev_error = 0.0f;

        // Indicate mode switch (optional flash or just UART)
        // HAL_GPIO_TogglePin(GPIOC, RED_Pin);
      }
    } else {
      potential_mode = detected_is_white_surface;
      stable_count = 0;
    }
  } else {
    stable_count = 0;
  }
}

float CalculatePosition(float *den_out, bool black_line) {
  float num = 0.0f, den = 0.0f;
  for (int i = 0; i < 5; i++) {
    float strength = black_line ? (100.0f - reflectance[i]) : reflectance[i];
    num += strength * weights[i];
    den += strength;
  }
  *den_out = den;

  float raw_pos = (den > 0.0f ? (num / den) : 0.0f);

  if (den < LINE_LOST_THRESHOLD) {
    raw_pos = filtered_position;
  }

  float delta = raw_pos - filtered_position;
  if (fabsf(delta) > 0.5f) {
    raw_pos = filtered_position + (delta > 0 ? 0.5f : -0.5f);
  }

  filtered_position =
      FILTER_ALPHA * filtered_position + (1.0f - FILTER_ALPHA) * raw_pos;
  return filtered_position;
}

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

char *FloatToString(float value, int precision, char *buffer, int buf_size) {
  if (precision < 0)
    precision = 0;
  if (precision > 6)
    precision = 6;
  bool negative = value < 0.0f;
  if (negative)
    value = -value;
  int int_part = (int)value;
  value -= int_part;
  int frac_part = (int)(value * powf(10.0f, precision) + 0.5f);
  if (negative)
    snprintf(buffer, buf_size, "-%d.%0*d", int_part, precision, frac_part);
  else
    snprintf(buffer, buf_size, "%d.%0*d", int_part, precision, frac_part);
  if (int_part == 0 && frac_part == 0 && negative) {
    buffer[0] = '0';
    buffer[1] = '.';
    for (int i = 0; i < precision; i++)
      buffer[2 + i] = '0';
    buffer[2 + precision] = '\0';
  }
  return buffer;
}

float ComputePID(float error, float *integral, float *prev_error, float dt,
                 float kp, float ki, float kd) {
  if (fabsf(error) > 1.5f)
    *integral = 0.0f;
  *integral += error * dt;
  if (*integral > INTEGRAL_LIMIT)
    *integral = INTEGRAL_LIMIT;
  else if (*integral < -INTEGRAL_LIMIT)
    *integral = -INTEGRAL_LIMIT;
  float derivative = (error - *prev_error) / dt;
  *prev_error = error;
  return kp * error + ki * (*integral) + kd * derivative;
}

void Do_Pickup_Routine() {
  SetMotorSpeeds(0, 0);
  HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin, GPIO_PIN_SET);
  picked_color = Local_Color_Detect(); // Read Color AND Turn on LEDs
  HAL_Delay(50);
  SetMotorSpeeds(-240, 240); // Spin 180 Fast
  HAL_Delay(350);
  SetMotorSpeeds(BASE_SPEED, BASE_SPEED); // Exit
  HAL_Delay(50);
}

void Do_Light_Blink(void) {
  while (1) {
    SetMotorSpeeds(0, 0);
    HAL_GPIO_WritePin(GPIOC, RED_Pin | GREEN_Pin | BLUE_Pin, GPIO_PIN_SET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOC, RED_Pin | GREEN_Pin | BLUE_Pin, GPIO_PIN_RESET);
    HAL_Delay(500);
  }
}
/* USER CODE END 0 */

// --- RL Helper Functions Implementation ---
void init_QLearningController(QLearningController *self) {
    self->n_states = 6;
    self->n_actions = 7;
    self->lr = 0.2f;           // Learning rate (not used in inference)
    self->gamma = 0.95f;       // Discount factor
    self->epsilon_start = 0.5f;
    self->epsilon_min = 0.01f;
    self->epsilon_decay = 0.995f;
    self->epsilon = self->epsilon_start;  
    self->filename = "q_table.pkl"; 
    
    // Load pre-trained Q-table
    for (int s = 0; s < 6; s++) {
        for (int a = 0; a < 7; a++) {
            self->q_table[s][a] = q_table[s][a];
        }
    }
    for (int i = 0; i < 7; i++) {
        self->action_list[i] = i; 
    }
    self->last_direction = 1.0f; 
    self->no_line_count = 0;
    self->stop_threshold = NO_LINE_TOLERANCE;
    self->prev_error = 0.0f;
    self->smooth_error = 0.0f;
    self->error_smoothing_alpha = 0.7f;  
    self->prev_state = 2; 
}

int Get_state(QLearningController *self, float values[NUM_SENSORS]) {
    float raw_sum = 0.0f;
    float max_val = 0.0f;
    for (int i = 0; i < NUM_SENSORS; i++) {
        raw_sum += values[i];
        if (values[i] > max_val) max_val = values[i];
    }

    if (raw_sum < NO_LINE_SUM_THRESH || max_val < NO_LINE_MAX_THRESH) {
        self->current_error = 2.0f;
        self->no_line_count++;
        self->smooth_error = self->error_smoothing_alpha * self->smooth_error + (1.0f - self->error_smoothing_alpha) * 2.0f;
        return 5;  // No line state
    } else {
        self->no_line_count = 0;  // Reset on detection
    }

    float weighted_sum = 0.0f;
    for (int i = 0; i < NUM_SENSORS; i++) {
        weighted_sum += (float)i * values[i];
    }
    float position = (raw_sum > 0.0f) ? (weighted_sum / raw_sum) : 2.0f;
    position += 0.1f;  // Small left bias offset
    float center = 2.0f;
    float error = center - position;  
    self->current_error = fabsf(error);
    self->error_sign = (error > 0.0f) ? 1.0f : ((error < 0.0f) ? -1.0f : 0.0f);

    self->smooth_error = self->error_smoothing_alpha * self->smooth_error + (1.0f - self->error_smoothing_alpha) * self->current_error;

    float abs_error = fabsf(error);
    if (abs_error > 0.2f) {  
        return (error > 0.0f) ? 4 : 0;  
    } else if (abs_error > 0.03f) {  
        return (error > 0.0f) ? 3 : 1;  
    } else {
        return 2;  
    }
}

int choose_action(QLearningController *self, int state) {
    self->epsilon *= self->epsilon_decay;
    self->epsilon = fmaxf(self->epsilon, self->epsilon_min);

    if (state == 0) return 4;
    if (state == 4) return 3;
    if (state == 1) return 2;
    if (state == 3) return 1;

    if (state == 5) {  
        if (self->no_line_count < NO_LINE_TOLERANCE) {
            return 0;  
        } else {
            if (rand() % 100 < 50) {  
                return rand() % 5;
            } else {
                return (self->last_direction > 0.0f) ? 4 : 3; 
            }
        }
    }

    float r = (float)rand() / RAND_MAX;
    if (r < self->epsilon) {
        return self->action_list[rand() % self->n_actions]; 
    } else {
        float max_q = self->q_table[state][0];
        int max_a = 0;
        for (int a = 1; a < self->n_actions; a++) {
            if (self->q_table[state][a] > max_q) {
                max_q = self->q_table[state][a];
                max_a = a;
            }
        }
        return max_a;
    }
    return 0;
}

void perform_action(int action, float *left_speed, float *right_speed) {
    float base_speed = 1.4f;   
    float mild_adjust = 1.0f;  
    float hard_adjust = 1.60f; 

    switch (action) {
        case 0:  
            *left_speed = base_speed;
            *right_speed = base_speed;
            break;
        case 1:  
            *left_speed = base_speed - mild_adjust;
            *right_speed = base_speed + mild_adjust;
            break;
        case 2:  
            *left_speed = base_speed + mild_adjust;
            *right_speed = base_speed - (mild_adjust) ;
            break;
        case 3:  
            *left_speed = base_speed - hard_adjust;
            *right_speed = base_speed + hard_adjust;
            break;
        case 4:  
            *left_speed = base_speed + hard_adjust;
            *right_speed = base_speed - (hard_adjust+10);
            break;
        case 5:  
            *left_speed = 0.0f;
            *right_speed = 0.0f;
            break;
        case 6:  
            *left_speed = base_speed * 1.2f;
            *right_speed = base_speed * 1.2f;
            break;
        default:
            *left_speed = 0.0f;
            *right_speed = 0.0f;
            break;
    }
}
// ------------------------------------------

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  
  init_QLearningController(&ql); // Init RL Controller
  srand(HAL_GetTick());

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  Color_Init(&htim3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuffer, 5);

  /* USER CODE BEGIN WHILE */
  while (1) {
    if (adc_complete) {
      adc_complete = 0;
      float den = 0.0f;
      float rl_input_values[5]; // Normalized 0-1 for RL
      
      for (int i = 0; i < 5; i++) {
        float raw = NormalizeReflectance(i, adcBuffer[i], is_black_line_mode);
        reflectance[i] =
            FILTER_ALPHA * raw + (1.0f - FILTER_ALPHA) * prev_reflect[i];
        prev_reflect[i] = reflectance[i];
        voltages[i] = (adcBuffer[i] / 4095.0f) * 3.3f;
        den += reflectance[i];
        
        // Prepare data for RL (Expects 0.0-1.0 float where 1.0 is White/Line?) 
        // RL Code expected raw ADC / 4095.0f. 
        // But our system creates `reflectance` 0-100 logic.
        // The provided RL code used: values[i] = (float)adc_buf[i] / 4095.0f;
        // So it likely worked on Raw Normalized or Raw voltages.
        // Let's use the Raw Normalized values scaled to 0-1 for RL compatibility
        rl_input_values[i] = reflectance[i] / 100.0f; 
      }

      Check_Surface_Mode();
      
      // ... State Machine Logic (Search, Return, Branch) ...
      // We keep the state machine for high level logic (When to pick, drop, turn)
      // But we can SWAP the motor control part.

      static int node_count = 0;
      static uint32_t last_node_time = 0;
      static uint32_t branch_timer = 0;

      switch (current_state) {  


      case STATE_SEARCH_1:
      case STATE_SEARCH_2:
        if (HAL_GPIO_ReadPin(Box_detect_GPIO_Port, Box_detect_Pin) ==
            GPIO_PIN_RESET) {
          Do_Pickup_Routine();
          node_count = 0;
          last_node_time = HAL_GetTick();
          current_state = (current_state == STATE_SEARCH_1) ? STATE_RETURN_1
                                                            : STATE_RETURN_2;
        }
        break;

      case STATE_RETURN_1:
      case STATE_RETURN_2:
        // CRITICAL: Only detect nodes if we have successfully switched to
        // White Surface (Black Line Mode).
        if (is_black_line_mode) {
          int black_count = 0;
          for (int i = 0; i < 5; i++)
            if (reflectance[i] < BLACK_LINE_THRESH)
              black_count++;

          if (black_count >= 3 && (HAL_GetTick() - last_node_time > 400)) {
            node_count++;
            last_node_time = HAL_GetTick();

            if (picked_color == BOX_BLUE && node_count == 2) {
              SetMotorSpeeds(BASE_SPEED, BASE_SPEED);
              HAL_Delay(0);
              SetMotorSpeeds(-200, 200);
              HAL_Delay(120); // LEFT Turn

              // Now Switch to Branch Navigation Phase
              current_state = (current_state == STATE_RETURN_1)
                                  ? STATE_BRANCH_OUT_1
                                  : STATE_BRANCH_OUT_2;
              branch_timer =
                  HAL_GetTick(); // Reset debounce for next node detection
            } else if (picked_color == BOX_GREEN && node_count == 3) {
              SetMotorSpeeds(BASE_SPEED, BASE_SPEED);
              HAL_Delay(200);
              SetMotorSpeeds(200, -200);
              HAL_Delay(180); // RIGHT Turn

              current_state = (current_state == STATE_RETURN_1)
                                  ? STATE_BRANCH_OUT_1
                                  : STATE_BRANCH_OUT_2;
              branch_timer = HAL_GetTick();
            }
          }
        }
        break;

      case STATE_BRANCH_OUT_1: // Box 1 Drop Logic
      case STATE_BRANCH_OUT_2: // Box 2 Drop Logic
        // We are on the branch line. Find the Drop Node.
        // Assume "Node" here matches the line color (Black Line Mode)
        if (is_black_line_mode) {
          int black_count = 0;
          for (int i = 0; i < 5; i++)
            if (reflectance[i] < BLACK_LINE_THRESH)
              black_count++;

          // Debounce the first second after turning to avoid detecting the turn
          // itself as a Node
          if (black_count >= 3 && (HAL_GetTick() - branch_timer > 400)) {
            // NODE DETECTED -> DROP
            SetMotorSpeeds(0, 0);
            HAL_GPIO_WritePin(Electromagnet_GPIO_Port, Electromagnet_Pin,
                              GPIO_PIN_RESET);

            if (current_state == STATE_BRANCH_OUT_2) {
              Do_Light_Blink(); // Stop for Box 2
            } else {
              // Box 1: Spin and Return
              HAL_Delay(150);

              // NEW: Turn off LEDs after dropping Box 1
              HAL_GPIO_WritePin(GPIOC, RED_Pin | GREEN_Pin | BLUE_Pin,
                                GPIO_PIN_RESET);

              SetMotorSpeeds(-240, 240);
              HAL_Delay(350); // Spin 180
              SetMotorSpeeds(BASE_SPEED, BASE_SPEED);
              HAL_Delay(150);
              current_state = STATE_BRANCH_RET_1;
              branch_timer = HAL_GetTick();
            }
          }
        }
        break;

      case STATE_BRANCH_RET_1:
        // Returning from Branch to Main Line
        // We look for intersection (All Black)
        if (is_black_line_mode) {
          int black_count = 0;
          for (int i = 0; i < 5; i++)
            if (reflectance[i] < BLACK_LINE_THRESH)
              black_count++;

          if (black_count >= 4 && (HAL_GetTick() - branch_timer > 400)) {
            // Intersection Reached
            SetMotorSpeeds(BASE_SPEED, BASE_SPEED);
            HAL_Delay(150); // Drive slightly past center

            // Turn back towards Source
            // If we turned LEFT to enter (Blue), we turn RIGHT to exit.
            // If we turned RIGHT to enter (Green), we turn LEFT to exit.
            if (picked_color == BOX_BLUE) {
              SetMotorSpeeds(230, -230);
              HAL_Delay(220); // Right
            } else {
              SetMotorSpeeds(-230, 230);
              HAL_Delay(220); // Left
            }

            current_state = STATE_TRANSITION_TO_SOURCE;
          }
        }
        break;

      case STATE_TRANSITION_TO_SOURCE:
        if (!is_black_line_mode) {
          static int transition_confirm = 0;
          transition_confirm++;
          if (transition_confirm > 50) {
            current_state = STATE_SEARCH_2;
            transition_confirm = 0;
          }
        }
        break;

      case STATE_DONE:
        Do_Light_Blink();
        break;
      }

      
      // Calculate PID outputs regardless (for logging or fallback)
      float pos_den = 0;
      float position = CalculatePosition(&pos_den, is_black_line_mode);
      float error = position;
      float dt = LOOP_DELAY_MS / 1000.0f;
      float kp = is_black_line_mode ? KP : KP_WL;
      float ki = is_black_line_mode ? KI : KI_WL;
      float kd = is_black_line_mode ? KD : KD_WL;
      float pid = ComputePID(error, &integral, &prev_error, dt, kp, ki, kd);

      int16_t l = 0, r = 0;

      #if USE_RL_CONTROL
          // --- RL CONTROL LOGIC ---
          int state = Get_state(&ql, rl_input_values);
          int action = choose_action(&ql, state);

          if (action == 1 || action == 3) ql.last_direction = -1.0f;
          else if (action == 2 || action == 4) ql.last_direction = 1.0f;

          float rl_l_speed, rl_r_speed;
          perform_action(action, &rl_l_speed, &rl_r_speed);

          // Convert RL output (approx 0-2.0) to PWM (0-255)
          int left_pwm = (int)((fabsf(rl_l_speed) / BASE_SPEED_SCALE) * 255.0f);
          int right_pwm = (int)((fabsf(rl_r_speed) / BASE_SPEED_SCALE) * 255.0f);
          
          // Apply direction
          l = (rl_l_speed >= 0) ? left_pwm : -left_pwm;
          r = (rl_r_speed >= 0) ? right_pwm : -right_pwm;
          
          ql.prev_state = state;
          // ------------------------
      #else
          // --- PID CONTROL LOGIC (Existing) ---
          float sharp_thresh = is_black_line_mode ? SHARP_TURN_THRESH_BL : SHARP_TURN_THRESH_WL;
          float turn_boost = is_black_line_mode ? TURN_BOOST_BL : TURN_BOOST_WL;
          float hard_err = is_black_line_mode ? HARD_TURN_ERR_BL : HARD_TURN_ERR_WL;
          float extreme_err = is_black_line_mode ? EXTREME_TURN_ERR_BL : EXTREME_TURN_ERR_WL;
          int hard_add = is_black_line_mode ? HARD_TURN_ADD_BL : HARD_TURN_ADD_WL;
          int hard_sub = is_black_line_mode ? HARD_TURN_SUB_BL : HARD_TURN_SUB_WL;
          int ext_l = is_black_line_mode ? EXTREME_TURN_L_BL : EXTREME_TURN_L_WL;
          int ext_r = is_black_line_mode ? EXTREME_TURN_R_BL : EXTREME_TURN_R_WL;

          bool sharp = fabsf(error) > sharp_thresh;
          if (sharp) pid *= turn_boost;

          int16_t base = sharp ? 70 : (is_black_line_mode ? BASE_SPEED : BASE_SPEED_WL);
          l = base + (int16_t)pid;
          r = base - (int16_t)pid;

          if (error > hard_err) { l += hard_add; r -= hard_sub; }
          if (error > extreme_err) { l = ext_l; r = ext_r; }
          if (error < -hard_err) { r += hard_add; l -= hard_sub; }

          if (l - r > MAX_TURN_DIFF) { l = base + MAX_TURN_DIFF / 2; r = base - MAX_TURN_DIFF / 2; }
          if (r - l > MAX_TURN_DIFF) { r = base + MAX_TURN_DIFF / 2; l = base - MAX_TURN_DIFF / 2; }
          // ------------------------------------
      #endif

      if (l > 255) l = 255;
      if (l < -255) l = -255;
      if (r > 255) r = 255;
      if (r < -255) r = -255;

      SetMotorSpeeds(l, r);

      char buf[128];
      snprintf(buf, sizeof(buf), "M:%s N:%d C:%d CTL:%s\r\n",
               is_black_line_mode ? "BLK" : "WHT", node_count, picked_color,
               USE_RL_CONTROL ? "RL " : "PID");
      HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), HAL_MAX_DELAY);
    }
    HAL_Delay(LOOP_DELAY_MS);
  }
}

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
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}

static void MX_ADC1_Init(void) {
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  HAL_ADC_Init(&hadc1);
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

static void MX_TIM2_Init(void) {
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&htim2);
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);
  HAL_TIM_MspPostInit(&htim2);
}

static void MX_TIM3_Init(void) {
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_IC_Init(&htim3);
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);
}

static void MX_USART2_UART_Init(void) {
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
}

static void MX_DMA_Init(void) {
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
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

  GPIO_InitStruct.Pin = Electromagnet_Pin | LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = S0_Pin | S1_Pin | S2_Pin | S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RED_Pin | GREEN_Pin | BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = Box_detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Box_detect_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1)
    adc_complete = 1;
}

void Error_Handler(void) {
  __disable_irq();
  while (1)
    ;
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif



