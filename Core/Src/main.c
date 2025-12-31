/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : PID Control with Logging & Direction Fix
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* USER CODE BEGIN PD */
#define AS5600_ADDR (0x36 << 1)
#define AS5600_RAW_ANGLE_REG 0x0C
#define GEAR_RATIO 10.56f
#define RX_BUFFER_SIZE 20

// --- IMPORTANT: CHANGE THIS IF MOTOR RUNS AWAY ---
// Set to 0 or 1 to flip motor direction logic
#define MOTOR_INVERT_DIR 0
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart3;
PCD_HandleTypeDef hpcd_USB_OTG_FS;

// --- PID Settings ---
typedef struct {
    float Kp, Ki, Kd;
    float prevError, integral;
    float outputLimit, minOutput;
} PID_Controller;

PID_Controller pid = {
    .Kp = 50.0f,      // Proportional Gain
    .Ki = 0.00f,      // Integral (Keep 0 for testing)
    .Kd = 0.00f,      // Derivative (Keep 0 for testing)
    .outputLimit = 4000.0f, // Max Speed (Hz)
    .minOutput = 100.0f     // Min Speed (Hz)
};

// --- Global Variables ---
volatile float target_angle = 0.0f;
volatile uint8_t new_command_received = 0;
float current_output_angle = 0.0f;

long global_revolutions = 0;
uint16_t prev_raw_angle = 0;
int first_reading = 1;

// UART Variables
uint8_t rx_byte;
char rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);

// --- 1. UART INTERRUPT ---
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        if (rx_byte == '\n' || rx_byte == '\r') {
            rx_buffer[rx_index] = '\0';
            if (rx_index > 0) {
                target_angle = atof(rx_buffer);
                pid.prevError = 0;
                pid.integral = 0;
                new_command_received = 1;
            }
            rx_index = 0;
        } else {
            if (rx_index < RX_BUFFER_SIZE - 1) rx_buffer[rx_index++] = rx_byte;
        }
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}

// --- 2. SAFE PRINTF ---
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, 10);
    return len;
}

// --- 3. AS5600 Read ---
uint16_t AS5600_ReadAngle(void) {
    uint8_t rawData[2];
    if(HAL_I2C_Master_Transmit(&hi2c1, AS5600_ADDR, (uint8_t[]){AS5600_RAW_ANGLE_REG}, 1, 10) != HAL_OK) return 9999;
    if (HAL_I2C_Master_Receive(&hi2c1, AS5600_ADDR, rawData, 2, 10) == HAL_OK) {
        return ((uint16_t)rawData[0] << 8) | rawData[1];
    }
    return 9999;
}

// --- 4. PID Compute ---
float PID_Compute(float setpoint, float measured) {
    float error = setpoint - measured;
    float P = pid.Kp * error;

    // Integral (Simplified windup guard)
    pid.integral += error;
    if (pid.integral > 500) pid.integral = 500;
    if (pid.integral < -500) pid.integral = -500;

    float I = pid.Ki * pid.integral;
    float D = pid.Kd * (error - pid.prevError);
    pid.prevError = error;

    return P + I + D;
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  printf("\r\n--- SYSTEM REBOOT ---\r\n");
  /* USER CODE END 2 */

  while (1)
  {
    // A. Heartbeat
    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);

    // B. Command Confirmation
    if (new_command_received) {
        printf("\r\n>>> New Target: %.2f <<<\r\n", target_angle);
        new_command_received = 0;
    }

    // C. Read Position
    uint16_t curr_raw = AS5600_ReadAngle();

    if (curr_raw != 9999) {
        // Multi-Turn Logic
        if (first_reading) { prev_raw_angle = curr_raw; first_reading = 0; }
        int delta = (int)curr_raw - (int)prev_raw_angle;
        if (delta < -2048) global_revolutions++;
        else if (delta > 2048) global_revolutions--;
        prev_raw_angle = curr_raw;

        float total_motor_deg = (global_revolutions * 360.0f) + ((curr_raw * 360.0f) / 4096.0f);
        current_output_angle = total_motor_deg / GEAR_RATIO;

        // D. PID Loop
        float pid_output = PID_Compute(target_angle, current_output_angle);

        // E. Motor Control
        if (fabs(target_angle - current_output_angle) < 0.5f) {
            // STOP Condition
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            pid.integral = 0;
        }
        else {
            // DIRECTION Logic (With Invert Switch)
            // If pid_output is positive, we want to go UP.
            // If MOTOR_INVERT_DIR is 0: Pos = High, Neg = Low
            // If MOTOR_INVERT_DIR is 1: Pos = Low, Neg = High

            GPIO_PinState dir_state;
            if (pid_output > 0) {
                dir_state = (MOTOR_INVERT_DIR) ? GPIO_PIN_RESET : GPIO_PIN_SET;
            } else {
                dir_state = (MOTOR_INVERT_DIR) ? GPIO_PIN_SET : GPIO_PIN_RESET;
            }
            HAL_GPIO_WritePin(STEP_DIR_GPIO_Port, STEP_DIR_Pin, dir_state);

            // SPEED Logic
            float freq = fabs(pid_output);
            if (freq > pid.outputLimit) freq = pid.outputLimit;
            if (freq < pid.minOutput) freq = pid.minOutput;

            uint32_t arr_val = (1000000 / (uint32_t)freq) - 1;
            __HAL_TIM_SET_AUTORELOAD(&htim3, arr_val);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, arr_val / 2);
        }

        // F. CONTINUOUS LOGGING (Every 200ms)
        static uint32_t last_print = 0;
        if (HAL_GetTick() - last_print > 200) {
            // Cast to int for safe printing
            int t_int = (int)target_angle;
            int c_int = (int)current_output_angle;
            int c_dec = (int)((current_output_angle - c_int) * 100);
            if(c_dec < 0) c_dec = -c_dec;

            printf("Targ:%d | Curr:%d.%02d | PWM:%d \r\n",
                   t_int, c_int, c_dec, (int)fabs(pid_output));

            last_print = HAL_GetTick();
        }
    }
    HAL_Delay(10);
  }
}

// [KEEP ALL INIT FUNCTIONS BELOW AS IS]
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}
static void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);
}
static void MX_TIM3_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 89;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim3);
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);
  HAL_TIM_PWM_Init(&htim3);
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_MspPostInit(&htim3);
}
static void MX_USART3_UART_Init(void) {
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);
}
static void MX_USB_OTG_FS_PCD_Init(void) {
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  HAL_PCD_Init(&hpcd_USB_OTG_FS);
}
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_WritePin(STEP_DIR_GPIO_Port, STEP_DIR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = STEP_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STEP_DIR_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);
}
void Error_Handler(void) {
  __disable_irq();
  while (1) {}
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
