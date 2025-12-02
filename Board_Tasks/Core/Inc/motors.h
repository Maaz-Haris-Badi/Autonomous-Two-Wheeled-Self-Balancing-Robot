#ifndef MOTORS_H
#define MOTORS_H

#include "main.h"

// ============================ MOTOR ENUMS ============================
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

void Motor_SetDirection(MotorID motor, MotorDirection dir);
void Motor_SetPWM(MotorID motor, uint16_t pwm_value);
void Motor_SetSpeed(MotorID motor, int16_t speed);  // -1000..+1000

#endif /* MOTORS_H */