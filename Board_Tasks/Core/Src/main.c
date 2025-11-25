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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdio.h>
#include <math.h>
#include <stdarg.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
  int16_t raw_ax, raw_ay, raw_az;
  float ax, ay, az;
  float ax_offset, ay_offset, az_offset;

  int16_t raw_gx, raw_gy, raw_gz;
  float gx, gy, gz;
  float gy_offset;
} SensorData;

typedef enum
{
  MOTOR_RIGHT = 0,
  MOTOR_LEFT = 1
} MotorID;

typedef enum
{
  DIR_FORWARD = 1,
  DIR_BACKWARD = -1,
  DIR_BRAKE = 0
} MotorDirection;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Accelerometer (LSM303AGR) over I2C
#define LSM303AGR_ACC_ADDR (0x19 << 1)
#define CTRL_REG1_A 0x20
#define CTRL_REG4_A 0x23
#define OUT_X_L_A 0x28

// Gyro (I3G4250D) over SPI
#define WHO_AM_I_G 0x0F
#define CTRL_REG1_G 0x20
#define CTRL_REG4_G 0x23
#define OUT_Y_L_G 0x2A
#define OUT_Y_H_G 0x2B

// Gyro SPI CS on PE3 (built-in on F3-Discovery)
#define CS_LOW() HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin, GPIO_PIN_SET)

// LEDs we use for GPIO tasks
#define LED_100HZ_PORT GPIOE // LD3 -> PE9
#define LED_100HZ_PIN GPIO_PIN_9

#define LED_10HZ_PORT GPIOE // LD4 -> PE8
#define LED_10HZ_PIN GPIO_PIN_8

#define BT_BUFFER_SIZE 128

#define PWM_MIN 200 // don't go below ~20% or motor may stall
#define PWM_MAX 999
#define KP 8.0f // start with these, we'll tune live
#define KI 0.5f
#define KD 0.0f
#define DT 0.001f // control loop runs every 50 ms (20 Hz)

volatile char bt_rx_buffer[BT_BUFFER_SIZE];
volatile uint8_t bt_rx_index = 0;
volatile uint8_t bt_data_ready = 0;
volatile uint8_t imu_streaming_enabled = 0;
volatile uint8_t imu_send_counter = 0;

// Balance PID (angle -> target speed)
typedef struct
{
  float setpoint; // Target angle (typically 0 degrees = upright)
  float kp, ki, kd;
  float integral;
  float prev_error;
  float output; // Output is target motor speed
} BalancePID_t;

// Motor Speed PID (RPM -> PWM)
typedef struct
{
  float setpoint; // Target RPM from balance PID
  float kp, ki, kd;
  float integral;
  float prev_error;
  float output; // Output is PWM value
} SpeedPID_t;

// Initialize PIDs
BalancePID_t balance_pid = {
    .setpoint = 0.0f, // 0 degrees = upright
    .kp = 1.0f,      // Start values - tune these!
    .ki = 0.0f,       // Start with 0, add later if needed
    .kd = 1.5f,
    .integral = 0.0f,
    .prev_error = 0.0f};

SpeedPID_t speed_pid_left = {
    .setpoint = 0.0f,
    .kp = 3.0f,
    .ki = 0.2f,
    .kd = 0.0f,
    .integral = 0.0f,
    .prev_error = 0.0f};

SpeedPID_t speed_pid_right = {
    .setpoint = 0.0f,
    .kp = 3.0f,
    .ki = 0.2f,
    .kd = 0.0f,
    .integral = 0.0f,
    .prev_error = 0.0f};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

// Encoder setup
#define ENCODER_Pin GPIO_PIN_12
#define ENCODER_PORT GPIOD

// Constants
#define PPR 330 // pulses per revolution
#define RPM_CALC_INTERVAL 100

// Globals
uint32_t last_pid_time = 0;

volatile uint32_t last_rpm_calc_time = 0;
// Right motor encoder
volatile uint32_t encoder_right_count = 0;
volatile float rpm_right = 0.0f;

// Left motor encoder (NEW)
volatile uint32_t encoder_left_count = 0;
volatile float rpm_left = 0.0f;

SensorData sensor;

// shared between ISR and main
volatile float accAngleY = 0.0f; // tilt from accelerometer (deg)
volatile float angleX = 0.0f;    // fused angle (deg)
float dt = 0.01f;                // 100 Hz

volatile uint8_t flag_10Hz = 0; // set by ISR every 10 periods

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

// printf support over USART2
int _write(int file, char *ptr, int len);

// Bluetooth functions
void BT_SendString(char *str);
void ProcessBluetoothCommand(char *cmd);
void SendIMUDataBT(void);

// Sensor functions
void Init_LSM(void);
void Offset_LSM(SensorData *data);
void Read_Accel(SensorData *data, float apply_offset);

void spi_write(uint8_t reg, uint8_t value);
uint8_t spi_read(uint8_t reg);
void Init_Gyro(void);
void Read_Gyro(SensorData *s);
void Calibrate_Gyro(SensorData *s);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ===== BALANCE PID COMPUTE =====
float Balance_PID_Compute(BalancePID_t *pid, float angle, float dt)
{
  // Error = target - current
  float error = pid->setpoint - angle;

  // Integral with anti-windup
  pid->integral += error * dt;
  if (pid->integral > 100.0f)
    pid->integral = 100.0f;
  if (pid->integral < -100.0f)
    pid->integral = -100.0f;

  // Derivative
  float derivative = (error - pid->prev_error) / dt;
  pid->prev_error = error;

  // PID output (target motor speed in RPM)
  float output = pid->kp * error +
                 pid->ki * pid->integral +
                 pid->kd * derivative;

  // Limit output to reasonable motor speeds
  if (output > 150.0f)
    output = 150.0f;
  if (output < -150.0f)
    output = -150.0f;

  pid->output = output;
  return output;
}

// ===== MOTOR SPEED PID COMPUTE =====
float Speed_PID_Compute(SpeedPID_t *pid, float measured_rpm, float dt)
{
  float error = pid->setpoint - measured_rpm;

  pid->integral += error * dt;

  // Anti-windup
  if (pid->integral > 300.0f)
    pid->integral = 300.0f;
  if (pid->integral < -300.0f)
    pid->integral = -300.0f;

  float derivative = (error - pid->prev_error) / dt;
  pid->prev_error = error;

  float output = pid->kp * error +
                 pid->ki * pid->integral +
                 pid->kd * derivative;

  // Convert to PWM (0-999)
  if (output > 999.0f)
    output = 999.0f;
  if (output < -999.0f)
    output = -999.0f;

  pid->output = output;
  return output;
}

// ===== SAFETY: Check if robot has fallen =====
uint8_t Is_Fallen(float angle)
{
  // If angle is beyond ±45 degrees, robot has fallen
  if (fabsf(angle) > 45.0f)
  {
    return 1; // Fallen
  }
  return 0; // Still balancing
}

// Set motor direction
void Motor_SetDirection(MotorID motor, MotorDirection dir)
{
  if (motor == MOTOR_RIGHT)
  {
    if (dir == DIR_FORWARD)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   // AIN1 = 1
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET); // AIN2 = 0
    }
    else if (dir == DIR_BACKWARD)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // AIN1 = 0
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_SET);   // AIN2 = 1
    }
    else
    { // BRAKE
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);
    }
  }
  else if (motor == MOTOR_LEFT)
  {
    if (dir == DIR_FORWARD)
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);   // BIN1 = 1
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // BIN2 = 0
    }
    else if (dir == DIR_BACKWARD)
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); // BIN1 = 0
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);   // BIN2 = 1
    }
    else
    { // BRAKE
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    }
  }
}

// Set motor PWM (0-1000)
void Motor_SetPWM(MotorID motor, uint16_t pwm_value)
{
  // Clamp to max
  if (pwm_value > 1000)
    pwm_value = 1000;

  // Scale to TIM3 period (65535)
  uint32_t compare = (pwm_value * 65535) / 1000;

  if (motor == MOTOR_RIGHT)
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, compare);
  }
  else if (motor == MOTOR_LEFT)
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, compare);
  }
}

// Set motor speed with direction (-1000 to +1000)
void Motor_SetSpeed(MotorID motor, int16_t speed)
{
  if (speed > 0)
  {
    Motor_SetDirection(motor, DIR_FORWARD);
    Motor_SetPWM(motor, speed);
  }
  else if (speed < 0)
  {
    Motor_SetDirection(motor, DIR_BACKWARD);
    Motor_SetPWM(motor, -speed); // Make positive
  }
  else
  {
    Motor_SetDirection(motor, DIR_BRAKE);
    Motor_SetPWM(motor, 0);
  }
}

int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}

void BT_SendString(char *str)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
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

  /* Reset of all peripherals, Initializes the Flash interface and the SyHAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USB_PCD_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 32768); // 50% of 65535
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 32768);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 800); // 50% duty cycle
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);        // Right encoder

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);   // BIN1
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // BIN2
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);             // Left PWM
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 800);    // 50% duty
  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);           // Left encoder

  HAL_UART_Receive_IT(&huart2, (uint8_t *)&bt_rx_buffer[bt_rx_index], 1);

  // 3 & 4: Enable accelerometer and gyro
  Init_LSM();
  Init_Gyro();

  // Calibrate sensors
  Offset_LSM(&sensor);
  Calibrate_Gyro(&sensor);

  // Start TIM2 with interrupt -> ISR at 100 Hz
  HAL_TIM_Base_Start_IT(&htim2);

  printf("\r\nSTM32F3 Discovery + HC-05 Bluetooth Ready\r\n");
  printf("System Running at 100 Hz ISR, 10 Hz main loop\r\n\r\n");

  BT_SendString("STM32 Connected!\r\n");
  BT_SendString("Commands: IMU_START, IMU_STOP, STATUS, HELP\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Calculate RPM for both motors every 100ms
    // === 100ms RPM calculation + PID control ===
    uint32_t now = HAL_GetTick();
    if (now - last_rpm_calc_time >= RPM_CALC_INTERVAL)
    {
      last_rpm_calc_time = now;
      float dt = RPM_CALC_INTERVAL / 1000.0f; // 0.1 seconds

      // --- Calculate RPM from encoders ---
      float pps_right = (encoder_right_count * 1000.0f) / RPM_CALC_INTERVAL;
      float pps_left = (encoder_left_count * 1000.0f) / RPM_CALC_INTERVAL;

      rpm_right = (pps_right / PPR) * 60.0f;
      rpm_left = (pps_left / PPR) * 60.0f;

      // Reset counters
      encoder_right_count = 0;
      encoder_left_count = 0;

      // --- SAFETY CHECK ---
      if (Is_Fallen(angleX))
      {
        // Robot has fallen - stop motors
        Motor_SetSpeed(MOTOR_LEFT, 0);
        Motor_SetSpeed(MOTOR_RIGHT, 0);

        // Reset PID integrals
        balance_pid.integral = 0.0f;
        speed_pid_left.integral = 0.0f;
        speed_pid_right.integral = 0.0f;

        printf("FALLEN! Angle: %.1f deg\r\n", angleX);
        continue; // Skip control loop
      }

      // --- STEP 1: Balance PID (Angle -> Target Speed) ---
      float target_speed = Balance_PID_Compute(&balance_pid, angleX, dt);

      // Set target speeds for both motors
      speed_pid_left.setpoint = target_speed;
      speed_pid_right.setpoint = target_speed;

      // --- STEP 2: Speed PID (RPM -> PWM) ---
      float pwm_left = Speed_PID_Compute(&speed_pid_left, rpm_left, dt);
      float pwm_right = Speed_PID_Compute(&speed_pid_right, rpm_right, dt);

      // --- STEP 3: Apply PWM to motors ---
      Motor_SetSpeed(MOTOR_LEFT, (int16_t)pwm_left);
      Motor_SetSpeed(MOTOR_RIGHT, (int16_t)pwm_right);

      // --- Debug Output ---
      printf("Ang:%.1f | Tgt:%.0f | L:%.0fRPM(%.0f) R:%.0fRPM(%.0f)\r\n",
             angleX, target_speed,
             rpm_left, pwm_left,
             rpm_right, pwm_right);
    }

    if (bt_data_ready)
    {
      bt_data_ready = 0;

      // Echo to debug terminal
      printf("BT CMD: %s\r\n", (char *)bt_rx_buffer);

      // Process command
      ProcessBluetoothCommand((char *)bt_rx_buffer);

      // Clear buffer
      memset((void *)bt_rx_buffer, 0, BT_BUFFER_SIZE);
      bt_rx_index = 0;
    }

    // 2,3,4: 10 Hz tasks in main loop (GPIO toggle + UART + sensor read)
    if (flag_10Hz)
    {
      flag_10Hz = 0;

      // 2. Toggle another GPIO at 10 Hz
      HAL_GPIO_TogglePin(LED_10HZ_PORT, LED_10HZ_PIN);

      // 3. Read accelerometer @10 Hz
      Read_Accel(&sensor, 1); // apply offset

      // 4. Read gyro @10 Hz
      Read_Gyro(&sensor);

      // Compute tilt from accelerometer (Y axis)
      float y_norm = sensor.ay / 9.81f;
      if (y_norm > 1.0f)
        y_norm = 1.0f;
      if (y_norm < -1.0f)
        y_norm = -1.0f;

      accAngleY = asinf(y_norm) * 180.0f / M_PI;

      // Send UART at 10 Hz
      printf("%.2f\t %.2f\t %.2f\t FusedX=%.2f\r\n", sensor.ay, sensor.gy, accAngleY, angleX);
    }

    // Send IMU data via Bluetooth if streaming enabled
    if (imu_streaming_enabled)
    {
      SendIMUDataBT();
    }
    // background work here if needed (non-blocking)
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_TIM8;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4799;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 71;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
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
  huart2.Init.BaudRate = 9600;
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
 * @brief USB Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin | GPIO_PIN_8 | GPIO_PIN_9 | LD5_Pin | LD7_Pin | LD9_Pin | LD10_Pin | LD8_Pin | LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin | MEMS_INT3_Pin | MEMS_INT4_Pin | MEMS_INT1_Pin | MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin PE8 PE9 LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin | GPIO_PIN_8 | GPIO_PIN_9 | LD5_Pin | LD7_Pin | LD9_Pin | LD10_Pin | LD8_Pin | LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PF4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  // Right motor encoder (TIM4)
  if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    encoder_right_count++;
  }

  // Left motor encoder (TIM8) - NEW
  if (htim->Instance == TIM8 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    encoder_left_count++;
  }
}

// ===== UART Receive Callback (Bluetooth) =====
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    char c = bt_rx_buffer[bt_rx_index];

    // Check for command terminator
    if (c == '\n' || c == '\r')
    {
      if (bt_rx_index > 0)
      {
        bt_rx_buffer[bt_rx_index] = '\0';
        bt_data_ready = 1;
      }
    }
    else
    {
      bt_rx_index++;
      if (bt_rx_index >= BT_BUFFER_SIZE - 1)
      {
        bt_rx_index = 0; // Overflow protection
      }
    }

    // Re-enable interrupt for next byte
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&bt_rx_buffer[bt_rx_index], 1);
  }
}

// ===== Send IMU Data via Bluetooth =====
void SendIMUDataBT(void)
{
  char buffer[128];

  // Format as JSON for easy parsing on mobile
  sprintf(buffer,
          "{\"ay\":%.2f,\"gy\":%.2f,\"angle_acc\":%.2f,\"angle_fused\":%.2f}\r\n",
          sensor.ay, sensor.gy, accAngleY, angleX);

  BT_SendString(buffer);
}

// ===== Process Bluetooth Commands =====
void ProcessBluetoothCommand(char *cmd)
{
  char response[128];
  float temp_val;

  // NEW: PID Tuning Commands
  if (sscanf(cmd, "KP %f", &temp_val) == 1)
  {
    balance_pid.kp = temp_val;
    sprintf(response, "Balance Kp = %.2f\r\n", balance_pid.kp);
    BT_SendString(response);
  }
  else if (sscanf(cmd, "KI %f", &temp_val) == 1)
  {
    balance_pid.ki = temp_val;
    sprintf(response, "Balance Ki = %.2f\r\n", balance_pid.ki);
    BT_SendString(response);
  }
  else if (sscanf(cmd, "KD %f", &temp_val) == 1)
  {
    balance_pid.kd = temp_val;
    sprintf(response, "Balance Kd = %.2f\r\n", balance_pid.kd);
    BT_SendString(response);
  }
  else if (strcmp(cmd, "RESET") == 0)
  {
    // Reset all PID integrals
    balance_pid.integral = 0.0f;
    speed_pid_left.integral = 0.0f;
    speed_pid_right.integral = 0.0f;
    BT_SendString("PIDs reset\r\n");
  }
  else if (strcmp(cmd, "STOP") == 0)
  {
    Motor_SetSpeed(MOTOR_LEFT, 0);
    Motor_SetSpeed(MOTOR_RIGHT, 0);
    BT_SendString("Motors stopped\r\n");
  }
  else if (strcmp(cmd, "PARAMS") == 0)
  {
    //sprintf(response, "Balance: Kp=%.2f Ki=%.2f Kd=%.2f\r\n""Speed: Kp=%.2f Ki=%.2f\r\n",balance_pid.kp, balance_pid.ki, balance_pid.kd,speed_pid_left.kp, speed_pid_left.ki);
    BT_SendString(response);
  }

  // if (strcmp(cmd, "IMU_START") == 0)
  // {
  //   imu_streaming_enabled = 1;
  //   printf("BT: IMU streaming started\r\n");
  //   BT_SendString("IMU streaming started at 10Hz\r\n");
  // }
  // else if (strcmp(cmd, "IMU_STOP") == 0)
  // {
  //   imu_streaming_enabled = 0;
  //   printf("BT: IMU streaming stopped\r\n");
  //   BT_SendString("IMU streaming stopped\r\n");
  // }
  // else if (strcmp(cmd, "STATUS") == 0)
  // {
  //   sprintf(response, "System OK | IMU: %s | Angle: %.2f deg\r\n",
  //           imu_streaming_enabled ? "ON" : "OFF", angleX);
  //   BT_SendString(response);
  // }
  // else if (strcmp(cmd, "HELP") == 0)
  // {
  //   BT_SendString("Commands:\r\n");
  //   BT_SendString("  IMU_START - Start streaming\r\n");
  //   BT_SendString("  IMU_STOP - Stop streaming\r\n");
  //   BT_SendString("  STATUS - System status\r\n");
  // }
  else
  {
    BT_SendString("Unknown command. Send HELP\r\n");
  }
}

// 1 & 5: 100 Hz timer ISR: toggle LED + loop filter + 10 Hz flag
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    // 1. GPIO toggle at 100 Hz
    HAL_GPIO_TogglePin(LED_100HZ_PORT, LED_100HZ_PIN);

    // 5. Complementary loop filter at 100 Hz
    angleX = 0.98f * (angleX + sensor.gy * dt) + 0.02f * accAngleY;

    // generate 10 Hz flag
    static uint8_t tick = 0;
    tick++;
    if (tick >= 10)
    {
      tick = 0;
      flag_10Hz = 1;
    }
  }
}

// ==== Accelerometer (I2C) ====
void Init_LSM(void)
{
  uint8_t data;

  // CTRL_REG1_A: 0x67 = 100 Hz, all axes enabled
  data = 0x67;
  HAL_I2C_Mem_Write(&hi2c1, LSM303AGR_ACC_ADDR, CTRL_REG1_A,
                    I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

  // CTRL_REG4_A: ±2g, continuous update
  data = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, LSM303AGR_ACC_ADDR, CTRL_REG4_A,
                    I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

  HAL_Delay(50);

  printf("LSM303AGR Accelerometer Initialized\r\n");
}

void Read_Accel(SensorData *data, float apply_offset)
{
  uint8_t raw[6];
  HAL_I2C_Mem_Read(&hi2c1, 0x32, OUT_X_L_A | 0x80, I2C_MEMADD_SIZE_8BIT, raw, 6, HAL_MAX_DELAY);

  data->raw_ay = (int16_t)((raw[3] << 8) | raw[2]);

  data->raw_ay = (data->raw_ay) >> 6;

  float ay_scaled = data->raw_ay * 0.004f * 9.81f;

  if (apply_offset)
  {
    data->ay = ay_scaled - data->ay_offset;
  }
  else
  {
    data->ay = ay_scaled;
  }
}

void Offset_LSM(SensorData *data)
{
  float sum_y = 0;
  uint8_t i;

  printf("Calibrating offsets... Keep sensor still.\r\n");

  for (i = 0; i < 20; i++)
  {
    Read_Accel(data, 0); // Read without subtracting offset
    sum_y += data->ay;
    HAL_Delay(100);
  }

  data->ay_offset = sum_y / 20.0f;

  printf("Offsets -> X: %.2f, Y: %.2f, Z: %.2f\r\n", data->ax_offset, data->ay_offset, data->az_offset);
}

// ==== Gyro (SPI) ====
void spi_write(uint8_t reg, uint8_t value)
{
  uint8_t data[2] = {(uint8_t)(reg & 0x7F), value}; // write: MSB=0
  CS_LOW();
  HAL_SPI_Transmit(&hspi1, data, 2, HAL_MAX_DELAY);
  CS_HIGH();
}

uint8_t spi_read(uint8_t reg)
{
  uint8_t tx[2];
  uint8_t rx[2];

  tx[0] = reg | 0x80; // read: MSB=1
  tx[1] = 0x00;

  CS_LOW();
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
  CS_HIGH();

  return rx[1];
}

void Init_Gyro(void)
{
  HAL_Delay(100);

  uint8_t whoami = spi_read(WHO_AM_I_G);
  if (whoami != 0xD3)
  {
    printf("Gyro WHO_AM_I error: 0x%02X (expected 0xD3)\r\n", whoami);
  }
  else
  {
    printf("I3G4250D Gyro WHO_AM_I: 0x%02X OK\r\n", whoami);
  }

  // CTRL_REG1_G: power on, 100 Hz, all axes
  spi_write(CTRL_REG1_G, 0x0F);

  // CTRL_REG4_G: ±245 dps
  spi_write(CTRL_REG4_G, 0x00);

  HAL_Delay(50);
  printf("I3G4250D Gyroscope Initialized\r\n");
}

void Read_Gyro(SensorData *s)
{
  uint8_t yl = spi_read(OUT_Y_L_G);
  uint8_t yh = spi_read(OUT_Y_H_G);

  int16_t gy_raw = (int16_t)((yh << 8) | yl);
  s->raw_gy = gy_raw;

  // 8.75 mdps/LSB -> deg/s
  s->gy = (gy_raw * 8.75f / 1000.0f) - s->gy_offset;

  // dead-zone
  if (fabsf(s->gy) < 0.1f)
    s->gy = 0.0f;
}

void Calibrate_Gyro(SensorData *s)
{
  float sum_gy = 0.0f;
  uint16_t samples = 500;

  printf("Calibrating gyroscope in 2 seconds... keep still.\r\n");
  HAL_Delay(2000);
  printf("Calibrating gyro now...\r\n");

  for (uint16_t i = 0; i < samples; i++)
  {
    uint8_t yl = spi_read(OUT_Y_L_G);
    uint8_t yh = spi_read(OUT_Y_H_G);

    int16_t gy_raw = (int16_t)((yh << 8) | yl);
    float gy_dps = gy_raw * 8.75f / 1000.0f;
    sum_gy += gy_dps;

    HAL_Delay(10);
  }

  s->gy_offset = sum_gy / samples;
  printf("Gyro Y offset: %.3f dps\r\n", s->gy_offset);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
