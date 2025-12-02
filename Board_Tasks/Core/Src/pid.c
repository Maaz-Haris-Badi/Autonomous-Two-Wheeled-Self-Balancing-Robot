#include "pid.h"
#include <math.h>


float Balance_PID_Compute(BalancePID_t *pid, float angle, float dt)
{
  float error = pid->setpoint - angle;

  // Deadband - stop if nearly balanced
  if (fabsf(error) < 0.3f)
  {
    pid->integral = 0.0f;
    pid->prev_error = error;
    return 0.0f; // Stop motors when balanced
  }

  // Integral with anti-windup
  pid->integral += error * dt;
  if (pid->integral > 50.0f)
    pid->integral = 50.0f;
  if (pid->integral < -50.0f)
    pid->integral = -50.0f;

  // Derivative
  float derivative = (error - pid->prev_error) / dt;
  pid->prev_error = error;

  // PID output (PWM value)
  float output = pid->kp * error +
                 pid->ki * pid->integral +
                 pid->kd * derivative;

  // Limit to PWM range
  if (output > 999.0f)
    output = 999.0f;
  if (output < -999.0f)
    output = -999.0f;

  // PWM deadband
  if (fabsf(output) < 100.0f)
    output = 0.0f;

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