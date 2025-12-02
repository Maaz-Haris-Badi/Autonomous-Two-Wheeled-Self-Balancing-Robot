#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"
#include "main.h"

// === LSM303AGR Accelerometer Registers ===
#define LSM303AGR_ACC_ADDR      (0x19 << 1)
#define CTRL_REG1_A             0x20
#define CTRL_REG4_A             0x23
#define OUT_X_L_A               0x28
#define OUT_X_H_A               0x29
#define OUT_Y_L_A               0x2A
#define OUT_Y_H_A               0x2B
#define OUT_Z_L_A               0x2C
#define OUT_Z_H_A               0x2D

// === I3G4250D Gyro Registers ===
#define I3G4250D_GYRO_ADDR     (0x69 << 1)
#define WHO_AM_I_G              0x0F
#define CTRL_REG1_G             0x20
#define CTRL_REG4_G             0x23
#define OUT_X_L_G               0x28
#define OUT_X_H_G               0x29
#define OUT_Y_L_G               0x2A
#define OUT_Y_H_G               0x2B
#define OUT_Z_L_G               0x2C
#define OUT_Z_H_G               0x2D

// CS Pin for Gyro (PE3)
#define CS_GYRO_LOW()  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)
#define CS_GYRO_HIGH() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)

void Init_LSM(void);
void Offset_LSM(SensorData *data);
void Read_Accel(SensorData *data, float apply_offset);

void Init_Gyro(void);
void Calibrate_Gyro(SensorData *s);
void Read_Gyro(SensorData *s);

#endif