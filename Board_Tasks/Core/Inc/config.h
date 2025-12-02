#ifndef CONFIG_H
#define CONFIG_H

#include "main.h"
#include <stdint.h>

// ============================ SYSTEM CONSTANTS ============================
#define CONTROL_LOOP_DT         0.01f     // 100 Hz = 10 ms
#define FILTER_ALPHA            0.98f     // Complementary filter (0.98 = trust gyro more)

#define PWM_MIN                 200
#define PWM_MAX                 999

#define FALL_ANGLE_THRESHOLD    45.0f     // degrees

// ============================ PID STRUCTURE ============================
typedef struct {
    float setpoint;      // Target angle in degrees (0 = upright)
    float kp, ki, kd;
    float integral;
    float prev_error;
    float output;
} BalancePID_t;

// ============================ SENSOR DATA STRUCTURE ============================
typedef struct
{
  int16_t raw_ax, raw_ay, raw_az;
  float ax, ay, az;
  float ax_offset, ay_offset, az_offset;

  int16_t raw_gx, raw_gy, raw_gz;
  float gx, gy, gz;
  float gy_offset;
} SensorData;

// ============================ GLOBAL VARIABLES (EXTERN) ============================
// Declared here, defined ONCE in main.c

extern BalancePID_t balance_pid;

extern SensorData sensor;

extern float angleX;          // Fused angle (complementary filter)
extern float accAngleY;       // Angle from accelerometer only
extern float dt;

extern volatile uint8_t flag_10Hz;
extern volatile uint8_t imu_streaming_enabled;
extern volatile uint8_t imu_send_counter;

// Bluetooth
#define BT_BUFFER_SIZE 128
extern volatile char     bt_rx_buffer[BT_BUFFER_SIZE];
extern volatile uint8_t bt_rx_index;
extern volatile uint8_t bt_data_ready;

#endif /* CONFIG_H */