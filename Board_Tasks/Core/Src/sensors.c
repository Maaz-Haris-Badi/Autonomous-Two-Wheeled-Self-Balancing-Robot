#include "sensors.h"
#include <stdio.h>
#include <math.h>


extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;

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
static void spi_write(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value};
    CS_GYRO_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    CS_GYRO_HIGH();
}

static uint8_t spi_read(uint8_t reg)
{
    uint8_t tx[2] = {reg | 0x80, 0x00};
    uint8_t rx[2] = {0};
    CS_GYRO_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    CS_GYRO_HIGH();
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