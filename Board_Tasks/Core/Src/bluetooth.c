
#include "bluetooth.h"
#include "motors.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>

extern UART_HandleTypeDef huart2;

void BT_SendString(const char *str)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
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

void ProcessBluetoothCommand(char *cmd)
{
  char response[128];
  float temp_val = 0.0f;

  // 1. Trim trailing \r \n and spaces from original cmd
  char *end = cmd + strlen(cmd) - 1;
  while (end >= cmd && (*end == '\r' || *end == '\n' || *end == ' ' || *end == '\t'))
  {
    *end-- = '\0';
  }

  // 2. Create uppercase version AFTER trimming
  char cmd_upper[BT_BUFFER_SIZE];
  strncpy(cmd_upper, cmd, BT_BUFFER_SIZE - 1);
  cmd_upper[BT_BUFFER_SIZE - 1] = '\0';

  for (int i = 0; cmd_upper[i]; i++)
  {
    if (cmd_upper[i] >= 'a' && cmd_upper[i] <= 'z')
      cmd_upper[i] -= 32; // convert to uppercase
  }
  printf("BT CMD UPPER: '%s'\r\n", cmd_upper);
  // Now use cmd_upper for all comparisons and sscanf

  // === PID TUNING - WORKS 100% with "KP12", "KP 12", "kp  15.5", etc. ===
  if (strncmp(cmd_upper, "KP", 2) == 0)
  {
    if (sscanf(cmd_upper + 2, " %f", &temp_val) == 1) // ← space added here!
    {
      balance_pid.kp = temp_val;
      balance_pid.integral = 0.0f;
      sprintf(response, "OK: Kp = %.3f\r\n", temp_val);
      BT_SendString(response);
      printf("BT: Kp set to %.3f\r\n", temp_val);
      return;
    }
  }
  else if (strncmp(cmd_upper, "KI", 2) == 0)
  {
    if (sscanf(cmd_upper + 2, " %f", &temp_val) == 1)
    {
      balance_pid.ki = temp_val;
      balance_pid.integral = 0.0f;
      sprintf(response, "OK: Ki = %.3f\r\n", temp_val);
      BT_SendString(response);
      printf("BT: Ki set to %.3f\r\n", temp_val);
      return;
    }
  }
  else if (strncmp(cmd_upper, "KD", 2) == 0)
  {
    if (sscanf(cmd_upper + 2, " %f", &temp_val) == 1)
    {
      balance_pid.kd = temp_val;
      sprintf(response, "OK: Kd = %.3f\r\n", temp_val);
      BT_SendString(response);
      printf("BT: Kd set to %.3f\r\n", temp_val);
      return;
    }
  }

  // === SET SETPOINT (target angle) ===
  else if (sscanf(cmd_upper, "SETPOINT %f", &temp_val) == 1)
  {
    balance_pid.setpoint = temp_val;
    sprintf(response, "OK: Setpoint = %.2f deg\r\n", balance_pid.setpoint);
    BT_SendString(response);
    printf("BT: Set Setpoint = %.2f\r\n", balance_pid.setpoint);
    return;
  }

  // === MOTOR CONTROL ===
  else if (strcmp(cmd_upper, "STOP") == 0)
  {
    Motor_SetSpeed(MOTOR_LEFT, 0);
    Motor_SetSpeed(MOTOR_RIGHT, 0);
    balance_pid.integral = 0.0f;
    BT_SendString("OK: Motors stopped\r\n");
    printf("BT: Motors stopped\r\n");
  }

  // === STATUS & INFO ===
  else if (strcmp(cmd_upper, "STATUS") == 0)
  {
    sprintf(response, "Angle: %.2f° | Kp:%.1f Ki:%.1f Kd:%.1f\r\n",
            angleX, balance_pid.kp, balance_pid.ki, balance_pid.kd);
    BT_SendString(response);
  }
  else if (strcmp(cmd_upper, "PARAMS") == 0)
  {
    sprintf(response,
            "=== PID Parameters ===\r\n"
            "Balance PID:\r\n"
            "  Kp = %.2f\r\n"
            "  Ki = %.2f\r\n"
            "  Kd = %.2f\r\n"
            "  Setpoint = %.2f deg\r\n"
            "Current Angle = %.2f deg\r\n",
            balance_pid.kp, balance_pid.ki, balance_pid.kd,
            balance_pid.setpoint, angleX);
    BT_SendString(response);
  }

  // === IMU DATA ===
  else if (strcmp(cmd_upper, "IMU") == 0)
  {
    sprintf(response,
            "AccY: %.2f m/s² | GyroY: %.2f °/s\r\n"
            "AccAngle: %.2f° | FusedAngle: %.2f°\r\n",
            sensor.ay, sensor.gy, accAngleY, angleX);
    BT_SendString(response);
  }

  // === STREAMING CONTROL ===
  else if (strcmp(cmd_upper, "STREAM ON") == 0 || strcmp(cmd_upper, "IMU_START") == 0)
  {
    imu_streaming_enabled = 1;
    BT_SendString("OK: IMU streaming started\r\n");
    printf("BT: IMU streaming enabled\r\n");
  }
  else if (strcmp(cmd_upper, "STREAM OFF") == 0 || strcmp(cmd_upper, "IMU_STOP") == 0)
  {
    imu_streaming_enabled = 0;
    BT_SendString("OK: IMU streaming stopped\r\n");
    printf("BT: IMU streaming disabled\r\n");
  }

  // === RESET ===
  else if (strcmp(cmd_upper, "RESET") == 0)
  {
    balance_pid.integral = 0.0f;
    balance_pid.prev_error = 0.0f;
    Motor_SetSpeed(MOTOR_LEFT, 0);
    Motor_SetSpeed(MOTOR_RIGHT, 0);
    BT_SendString("OK: PID reset, motors stopped\r\n");
    printf("BT: System reset\r\n");
  }

  // === HELP ===
  else if (strcmp(cmd_upper, "HELP") == 0 || strcmp(cmd_upper, "?") == 0)
  {
    BT_SendString(
        "=== Available Commands ===\r\n"
        "PID Tuning:\r\n"
        "  KP <value>  - Set Kp gain\r\n"
        "  KI <value>  - Set Ki gain\r\n"
        "  KD <value>  - Set Kd gain\r\n"
        "  SETPOINT <angle> - Set target angle\r\n"
        "\r\n"
        "Control:\r\n"
        "  STOP        - Stop motors\r\n"
        "  RESET       - Reset PID & stop\r\n"
        "\r\n"
        "Info:\r\n"
        "  STATUS      - Quick status\r\n"
        "  PARAMS      - Show all parameters\r\n"
        "  IMU         - Show IMU readings\r\n"
        "\r\n"
        "Streaming:\r\n"
        "  STREAM ON   - Start IMU stream\r\n"
        "  STREAM OFF  - Stop IMU stream\r\n"
        "  HELP or ?   - Show this help\r\n");
  }

  // === UNKNOWN COMMAND ===
  else
  {
    sprintf(response, "ERROR: Unknown command '%s'\r\nSend HELP for commands\r\n", cmd_upper);
    BT_SendString(response);
    printf("BT: Unknown command: %s\r\n", cmd_upper);
  }
}
