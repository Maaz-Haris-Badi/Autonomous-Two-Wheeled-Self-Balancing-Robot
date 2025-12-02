
#include "motors.h"

extern TIM_HandleTypeDef htim3;

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