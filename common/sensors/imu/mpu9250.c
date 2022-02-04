/**
 * @file    mpu9250.c
 * @author  Raymond Oung
 * @date    2014.05.11
 * @brief   MPU9150 (rate-gyro / accelerometer / magnetometer) functions
 *
 * MIT License
 *
 * Copyright (c) 2022 Raymond Oung
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Includes ------------------------------------------------------------------*/
#include "mpu9250.h"
#include "clock.h"
#include "stm32f4xx.h"
#include <math.h>

#ifdef _VERBOSE
#include "uart.h"
#include <stdio.h>
#endif
/** @addtogroup Source
 * @{
 */

/** @addtogroup Utilities
 * @{
 */

/** @defgroup MPU9250
 * @brief Functions for reading MPU9250 (rate-gyro + accelerometer)
 * @{
 */

/** @defgroup MPU9250_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @defgroup MPU9250_Private_Defines
 * @{
 */
#ifdef _VERBOSE
#define VERBOSE_UART_ID 1u
#endif

#define MPU9250_ADDR 0x69 // b0110 1001 (AD0 = 1)
#define AK8963_ADDR                                                            \
  0x0C // Magnetometer address, see Section 7.15 of PS-MPU-9150A-00 rev. 4.3

#define MPU9250_XG_OFFSET_H 0x13
#define MPU9250_XG_OFFSET_L 0x14
#define MPU9250_YG_OFFSET_H 0x15
#define MPU9250_YG_OFFSET_L 0x16
#define MPU9250_ZG_OFFSET_H 0x17
#define MPU9250_ZG_OFFSET_L 0x18

#define MPU9250_CONFIG_REG 0x1A       // rate-gyro and temperature bandwidth
#define MPU9250_GYRO_CONFIG_REG 0x1B  // rate-gyro range
#define MPU9250_ACCL_CONFIG_REG 0x1C  // accelerometer bandwidth
#define MPU9250_ACCL_CONFIG2_REG 0x1D // accelerometer range

#define MPU9250_ACCL_DATA_REG 0x3B // start register of accelerometer data
#define MPU9250_GYRO_DATA_REG 0x43 // start register of gyro data
#define MPU9250_TEMP_DATA_REG 0x41 // start register of temperature data

#define MPU9250_INT_PIN_CFG_REG 0x37 // pin configuration register
#define MPU9250_INT_PIN_CFG_VAL                                                \
  0x02 // enable I2C bypass/pass-through (needed for magnetomer)

#define MPU9250_USER_CTRL_REG 0x6A // user control register
#define MPU9250_USER_CTRL_VAL                                                  \
  0x00 // auxiliary I2C bus is logically driven by the primary I2C bus

#define MPU9250_PWR_MGMT_1_REG 0x6B // power management register
#define MPU9250_PWR_MGMT_1_VAL                                                 \
  0x01 // auto selects the best available clock source

#define AK8963_ST1_REG 0x02 // status 1 register; data-ready
#define AK8963_HXL_REG 0x03 // start register of magnetometer data
#define AK8963_ST2_REG                                                         \
  0x09 // status 2 register; magnetic sensor overflow (must be read)
#define AK8963_ASAX_REG 0x10 // start register of sensitivity adjustment values

#define AK8963_CNTL1_REG 0x0A // control register
/**
 * @}
 */

/** @defgroup MPU9250_Private_Macros
 * @{
 */
/**
 * @}
 */

/** @defgroup MPU9250_Private_Variables
 * @{
 */
static I2C_TypeDef *MPU9250_I2C;
static const float MPU9250_TEMP_BIAS = 21.0f; // [deg C]
static const float MPU9250_TEMP_SENSITIVITY =
    340.0f; // [LSB/deg C] @TODO This is the temperature sensitivity of MPU9250,
            // not MPU9250. No value for MPU9250 was found.
static volatile float MPU9250_GYRO_SENSITIVITY; // [LSB/rad/s]
static volatile float MPU9250_ACCL_SENSITIVITY; // [LSB/mg]
static volatile float AK8963_LSB2uT_X;          // [uT/LSB]
static volatile float AK8963_LSB2uT_Y;          // [uT/LSB]
static volatile float AK8963_LSB2uT_Z;          // [uT/LSB]

static volatile struct MPU9250_data_t {
  float timestamp; // [msec]
  int16_t gyro[3]; // raw rate-gyro data
  int16_t accl[3]; // raw accelerometer data
  int16_t temp;    // raw temperature data
} MPU9250_data;

static volatile struct AK8963_data_t {
  float timestamp; // [msec]
  int16_t magn[3]; // raw magnetometer data
  int8_t status;   // data ready status
} AK8963_data;
/**
 * @}
 */

/* Global variables  ---------------------------------------------------------*/

/** @defgroup MPU9250_Private_Functions
 * @{
 */
static void MPU9250_TxByte(uint8_t reg, uint8_t val);
static void MPU9250_RxData(uint8_t reg, uint8_t *data, uint8_t num);
static void AK8963_TxByte(uint8_t reg, uint8_t val);
static void AK8963_RxData(uint8_t reg, uint8_t *data, uint8_t num);
/**
 * @}
 */

/**
 * @brief  Initializes MPU9250.
 * @param  I2Cx  I2C perihperal type [I2C1..I2C2]
 * @param  config Value of MPU9250 CONFIG (0x1A) register (rate-gyro and
 * temperature bandwidth)
 * @param  gyro_config Value of MPU9250 GYRO_CONFIG (0x1B) register (rate-gyro
 * range)
 * @param  accl_config Value of MPU9250 ACCEL_CONFIG (0x1C) register
 * (accelerometer bandwidth)
 * @param  accl_config2 Value of MPU9250 ACCEL_CONFIG2 (0x1D) register
 * (accelerometer range)
 * @retval 0=Failure, 1=Success
 */
uint8_t MPU9250_Init(I2C_TypeDef *I2Cx, uint8_t config, uint8_t gyro_config,
                     uint8_t accl_config, uint8_t accl_config2,
                     uint8_t magn_config) {
  MPU9250_I2C = I2Cx;

  // initialize MPU9250
  MPU9250_TxByte(MPU9250_PWR_MGMT_1_REG, MPU9250_PWR_MGMT_1_VAL);
  MPU9250_TxByte(MPU9250_CONFIG_REG, config);
  MPU9250_TxByte(MPU9250_GYRO_CONFIG_REG, gyro_config);
  MPU9250_TxByte(MPU9250_ACCL_CONFIG_REG, accl_config);
  MPU9250_TxByte(MPU9250_ACCL_CONFIG2_REG, accl_config2);

  switch (gyro_config) {
  case MPU9250_GYRO_RANGE_250: // ± 250 deg/s range
    MPU9250_GYRO_SENSITIVITY =
        131.0f * 180 / M_PI; // 131.0f [LSB/deg/s] * 180/pi [deg/rad]
    break;
  case MPU9250_GYRO_RANGE_500: // ± 500 deg/s range
    MPU9250_GYRO_SENSITIVITY =
        65.5f * 180 / M_PI; // 65.5f [LSB/deg/s] * 180/pi [deg/rad]
    break;
  case MPU9250_GYRO_RANGE_1000: // ± 1000 deg/s range
    MPU9250_GYRO_SENSITIVITY =
        32.75f * 180 / M_PI; // 32.75f [LSB/deg/s] * 180/pi [deg/rad]
    break;
  case MPU9250_GYRO_RANGE_2000: // ± 2000 deg/s range
    MPU9250_GYRO_SENSITIVITY =
        16.375 * 180 / M_PI; // 16.375f [LSB/deg/s] * 180/pi [deg/rad]
    break;
  default:
#ifdef _VERBOSE
    UART_Print(VERBOSE_UART_ID, "ERROR: Gyro-Config\r\n");
#endif
    while (1)
      ; // error
    break;
  }

  switch (accl_config2) {
  case MPU9250_ACCL_RANGE_2:             // ± 2 g range
    MPU9250_ACCL_SENSITIVITY = 16384.0f; // [LSB/mg]
    break;
  case MPU9250_ACCL_RANGE_4:            // ± 4 g range
    MPU9250_ACCL_SENSITIVITY = 8192.0f; // [LSB/mg]
    break;
  case MPU9250_ACCL_RANGE_8:            // ± 8 g range
    MPU9250_ACCL_SENSITIVITY = 4096.0f; // [LSB/mg]
    break;
  case MPU9250_ACCL_RANGE_16:           // ± 16 g range
    MPU9250_ACCL_SENSITIVITY = 2048.0f; // [LSB/mg]
    break;
  default:
#ifdef _VERBOSE
    UART_Print(VERBOSE_UART_ID, "ERROR: Accl-Config2\r\n");
#endif
    while (1)
      ; // error
    break;
  }

  // enable auxiliary I2C needed for magnetometer
  MPU9250_TxByte(MPU9250_USER_CTRL_REG, MPU9250_USER_CTRL_VAL);
  MPU9250_TxByte(MPU9250_INT_PIN_CFG_REG, MPU9250_INT_PIN_CFG_VAL);

  // Get AK8963 sensitivity adjustment values
  uint8_t data[3];
  AK8963_RxData(AK8963_ASAX_REG, data, 3);

  // Compute sensitivity adjustment values and conversion factors
  AK8963_LSB2uT_X = (((float)data[0] - 128.0f) * 0.5f) / 128.0f +
                    1.0f; // ((ASA-128)*0.5)/128+1
  AK8963_LSB2uT_Y = (((float)data[1] - 128.0f) * 0.5f) / 128.0f + 1.0f;
  AK8963_LSB2uT_Z = (((float)data[2] - 128.0f) * 0.5f) / 128.0f + 1.0f;
  char str[100];
  sprintf(str, "%.3f %.3f %.3f\r\n", AK8963_LSB2uT_X, AK8963_LSB2uT_Y,
          AK8963_LSB2uT_Z);
  UART_Print(1, str);

  // call for new magnetometer measurement
  AK8963_TxByte(AK8963_CNTL1_REG, magn_config);

  return 1u;
}

/**
 * @brief  Send a byte of data to MPU9250
 * @param  reg Register for writing the data
 * @param  val Value to be written
 * @retval None
 */
static void MPU9250_TxByte(uint8_t reg, uint8_t val) {
  // Send START condition
  I2C_GenerateSTART(MPU9250_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send Slave MPU9250 Address
  I2C_Send7bitAddress(MPU9250_I2C, MPU9250_ADDR << 1u,
                      I2C_Direction_Transmitter);

  // Test on EV6 and clear it
  while (
      !I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    ;

  // Send the internal MPU9250 Address to write to
  I2C_SendData(MPU9250_I2C, (uint8_t)reg);

  // Test on EV8 and clear it
  while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  // Send register content
  I2C_SendData(MPU9250_I2C, (uint8_t)val);

  // Test on EV8 and clear it
  while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  // Send STOP Condition
  I2C_GenerateSTOP(MPU9250_I2C, ENABLE);
  while (I2C_GetFlagStatus(MPU9250_I2C, I2C_FLAG_STOPF))
    ;
  while (I2C_GetFlagStatus(MPU9250_I2C, I2C_FLAG_BUSY))
    ;
}

/**
 * @brief  Read rate-gyro or accelerometer data from MPU9250
 * @param  reg Register to read
 * @param  data Pointer to data to save the data
 * @param  num Number of bytes to read
 * @retval None
 */
static void MPU9250_RxData(uint8_t reg, uint8_t *data, uint8_t num) {
  uint8_t i;

  // Send START condition
  I2C_GenerateSTART(MPU9250_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send slave MPU9250 address for writing
  I2C_Send7bitAddress(MPU9250_I2C, MPU9250_ADDR << 1u,
                      I2C_Direction_Transmitter);

  // Test on EV6 and clear it
  while (
      !I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    ;

  // Send the internal MPU9250 address to read from
  I2C_SendData(MPU9250_I2C, reg);

  // Test on EV8 and clear it
  while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  // Send START condition a second time
  I2C_GenerateSTART(MPU9250_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send slave MPU9250 address for reading
  I2C_Send7bitAddress(MPU9250_I2C, MPU9250_ADDR << 1u, I2C_Direction_Receiver);

  // Test on EV6 and clear it
  while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    ;

  // Read N bytes
  for (i = 0u; i < num - 1u; i++) {
    // Test on EV7 and clear it
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
      ;

    // Read a byte
    data[i] = I2C_ReceiveData(MPU9250_I2C);
  }

  // Disable Acknowledgement on the last byte
  I2C_AcknowledgeConfig(MPU9250_I2C, DISABLE);

  // Test on EV7 and clear it
  while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
    ;

  // Read a byte
  data[i] = I2C_ReceiveData(MPU9250_I2C);

  // Send STOP Condition
  I2C_GenerateSTOP(MPU9250_I2C, ENABLE);
  while (I2C_GetFlagStatus(MPU9250_I2C, I2C_FLAG_STOPF))
    ;
  while (I2C_GetFlagStatus(MPU9250_I2C, I2C_FLAG_BUSY))
    ;

  // Enable Acknowledgement to be ready for another reception
  I2C_AcknowledgeConfig(MPU9250_I2C, ENABLE);
}

/**
 * @brief  Send a byte of data to AK8963
 * @param  reg Register for writing the data
 * @param  val Value to be written
 * @retval None
 */
static void AK8963_TxByte(uint8_t reg, uint8_t val) {
  // Send START condition
  I2C_GenerateSTART(MPU9250_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send Slave AK8963 address
  I2C_Send7bitAddress(MPU9250_I2C, AK8963_ADDR << 1u,
                      I2C_Direction_Transmitter);

  // Test on EV6 and clear it
  while (
      !I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    ;

  // Send the internal AK8963 address to write to
  I2C_SendData(MPU9250_I2C, (uint8_t)reg);

  // Test on EV8 and clear it
  while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  // Send register content
  I2C_SendData(MPU9250_I2C, (uint8_t)val);

  // Test on EV8 and clear it
  while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  // Send STOP Condition
  I2C_GenerateSTOP(MPU9250_I2C, ENABLE);
  while (I2C_GetFlagStatus(MPU9250_I2C, I2C_FLAG_STOPF))
    ;
  while (I2C_GetFlagStatus(MPU9250_I2C, I2C_FLAG_BUSY))
    ;
}

/**
 * @brief  Read magnetometer data from AK8963
 * @param  reg Register to read
 * @param  data Pointer to data to save the data
 * @param  num Number of bytes to read
 * @retval None
 */
static void AK8963_RxData(uint8_t reg, uint8_t *data, uint8_t num) {
  uint8_t i;

  // Send START condition
  I2C_GenerateSTART(MPU9250_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send slave AK8963 address for writing
  I2C_Send7bitAddress(MPU9250_I2C, AK8963_ADDR << 1u,
                      I2C_Direction_Transmitter);

  // Test on EV6 and clear it
  while (
      !I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    ;

  // Send the internal AK8963 address to read from
  I2C_SendData(MPU9250_I2C, reg);

  // Test on EV8 and clear it
  while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  // Send START condition a second time
  I2C_GenerateSTART(MPU9250_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send slave AK8963 address for reading
  I2C_Send7bitAddress(MPU9250_I2C, AK8963_ADDR << 1u, I2C_Direction_Receiver);

  // Test on EV6 and clear it
  while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    ;

  // Read N bytes
  for (i = 0u; i < num - 1u; i++) {
    // Test on EV7 and clear it
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
      ;

    // Read a byte
    data[i] = I2C_ReceiveData(MPU9250_I2C);
  }

  // Disable Acknowledgement on the last byte
  I2C_AcknowledgeConfig(MPU9250_I2C, DISABLE);

  // Test on EV7 and clear it
  while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
    ;

  // Read a byte
  data[i] = I2C_ReceiveData(MPU9250_I2C);

  // Send STOP Condition
  I2C_GenerateSTOP(MPU9250_I2C, ENABLE);
  while (I2C_GetFlagStatus(MPU9250_I2C, I2C_FLAG_STOPF))
    ;
  while (I2C_GetFlagStatus(MPU9250_I2C, I2C_FLAG_BUSY))
    ;

  // Enable Acknowledgement to be ready for another reception
  I2C_AcknowledgeConfig(MPU9250_I2C, ENABLE);
}

/**
 * @brief  MPU9250 Interrupt service routine (Blocking)
 * @retval None
 */
void MPU9250_ReadSensor(void) {
  static uint8_t data[14];

  // Get MPU9250 data (accelerometer and rate-gyro)
  MPU9250_data.timestamp = GetTimeU();
  MPU9250_RxData(MPU9250_ACCL_DATA_REG, data, 14u);

  // extract raw accelerometer data
  MPU9250_data.accl[0] = ((uint16_t)data[0] << 8u | (uint16_t)data[1]);
  MPU9250_data.accl[1] = ((uint16_t)data[2] << 8u | (uint16_t)data[3]);
  MPU9250_data.accl[2] = ((uint16_t)data[4] << 8u | (uint16_t)data[5]);

  // extract raw temperature data
  MPU9250_data.temp = ((int16_t)data[6] << 8u | (int16_t)data[7]);

  // extract raw rate-gyroscope data
  MPU9250_data.gyro[0] = ((uint16_t)data[8] << 8u | (uint16_t)data[9]);
  MPU9250_data.gyro[1] = ((uint16_t)data[10] << 8u | (uint16_t)data[11]);
  MPU9250_data.gyro[2] = ((uint16_t)data[12] << 8u | (uint16_t)data[13]);
}

/**
 * @brief  AK8963 Interrupt service routine (Blocking)
 * @retval None
 */
void AK8963_ReadSensor(void) {
  uint8_t ready;
  uint8_t data[7] = {0u, 0u, 0u, 0u, 0u, 0u, 0u};

  AK8963_RxData(AK8963_ST1_REG, &ready, 1);

  if (ready) {
    // Get AK8963 data (magnetometer)
    AK8963_data.timestamp = GetTimeU();
    AK8963_RxData(AK8963_HXL_REG, data, 6u);

    // extract raw magnetometer data
    AK8963_data.magn[0] = ((uint16_t)data[1] << 8u | (uint16_t)data[0]);
    AK8963_data.magn[1] = ((uint16_t)data[3] << 8u | (uint16_t)data[2]);
    AK8963_data.magn[2] = ((uint16_t)data[5] << 8u | (uint16_t)data[4]);
  }
}

/**
 * @brief  MPU9250 Interrupt service routine (I2C Interrupt)
 * @param  None
 * @retval None
 */
uint8_t MPU9250_ReadSensorIT(void) {
  static uint8_t direction = I2C_Direction_Transmitter; // I2C direction
  static uint8_t i = 0u;                                // byte counter
  static uint8_t data[14]; // read a total of 14 bytes

  switch (I2C_GetLastEvent(MPU9250_I2C)) {
  // Test on EV5 and clear it
  case I2C_EVENT_MASTER_MODE_SELECT:
    // Send Slave MPU9250 Address
    I2C_Send7bitAddress(MPU9250_I2C, MPU9250_ADDR << 1, direction);
    break;

  // Test on EV6 and clear it
  case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
    // Send the internal MPU9250 address to read from
    I2C_SendData(MPU9250_I2C, MPU9250_ACCL_DATA_REG);
    MPU9250_data.timestamp = GetTimeU();
    break;

  case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
    I2C_ITConfig(MPU9250_I2C, I2C_IT_BUF, ENABLE);
    // Send START condition a second time
    I2C_GenerateSTART(MPU9250_I2C, ENABLE);
    direction = I2C_Direction_Receiver;
    break;

  case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
    break;

  // Test on EV7 and clear it
  case I2C_EVENT_MASTER_BYTE_RECEIVED:
    // Read a byte
    data[i++] = I2C_ReceiveData(MPU9250_I2C);

    // Disable Acknowledgement on the last byte
    if (i == 13u) {
      // Disable Acknowledgement on the last byte
      I2C_AcknowledgeConfig(MPU9250_I2C, DISABLE);
    }

    if (i == 14u) {
      // Send STOP Condition
      I2C_GenerateSTOP(MPU9250_I2C, ENABLE);
      while (I2C_GetFlagStatus(MPU9250_I2C, I2C_FLAG_STOPF))
        ;
      while (I2C_GetFlagStatus(MPU9250_I2C, I2C_FLAG_BUSY))
        ;

      // reset
      I2C_AcknowledgeConfig(MPU9250_I2C, ENABLE);
      direction = I2C_Direction_Transmitter;
      i = 0u;

      // extract raw accelerometer data
      MPU9250_data.accl[0] = ((uint16_t)data[0] << 8u | (uint16_t)data[1]);
      MPU9250_data.accl[1] = ((uint16_t)data[2] << 8u | (uint16_t)data[3]);
      MPU9250_data.accl[2] = ((uint16_t)data[4] << 8u | (uint16_t)data[5]);

      // extract raw temperature data
      MPU9250_data.temp = ((int16_t)data[6] << 8u | (int16_t)data[7]);

      // extract raw rate-gyroscope data
      MPU9250_data.gyro[0] = ((uint16_t)data[8] << 8u | (uint16_t)data[9]);
      MPU9250_data.gyro[1] = ((uint16_t)data[10] << 8u | (uint16_t)data[11]);
      MPU9250_data.gyro[2] = ((uint16_t)data[12] << 8u | (uint16_t)data[13]);

      return 1u;
    }
    break;

  default:
    break;
  }
  return 0u;
}

/**
 * @brief  AK8963 Interrupt service routine (I2C Interrupt)
 * @param  None
 * @retval None
 */
uint8_t AK8963_ReadSensorIT(void) {
  static uint8_t direction = I2C_Direction_Transmitter; // I2C direction
  static uint8_t i = 0u;                                // byte counter
  static uint8_t data[8];        // read a total of 8 bytes
  static float timestamp = 0.0f; // timestamp [msec]

  switch (I2C_GetLastEvent(MPU9250_I2C)) {
  // Test on EV5 and clear it
  case I2C_EVENT_MASTER_MODE_SELECT:
    // Send Slave MPU9250 Address
    I2C_Send7bitAddress(MPU9250_I2C, AK8963_ADDR << 1, direction);
    break;

  // Test on EV6 and clear it
  case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
    // Send the internal MPU9250 address to read from
    I2C_SendData(MPU9250_I2C, AK8963_ST1_REG);
    timestamp = GetTimeU();
    break;

  case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
    I2C_ITConfig(MPU9250_I2C, I2C_IT_BUF, ENABLE);
    // Send START condition a second time
    I2C_GenerateSTART(MPU9250_I2C, ENABLE);
    direction = I2C_Direction_Receiver;
    break;

  case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
    break;

  // Test on EV7 and clear it
  case I2C_EVENT_MASTER_BYTE_RECEIVED:
    // Read a byte
    data[i++] = I2C_ReceiveData(MPU9250_I2C);

    // Disable Acknowledgement on the last byte
    if (i == 7u) {
      // Disable Acknowledgement on the last byte
      I2C_AcknowledgeConfig(MPU9250_I2C, DISABLE);
    }

    if (i == 8u) {
      // Send STOP Condition
      I2C_GenerateSTOP(MPU9250_I2C, ENABLE);
      while (I2C_GetFlagStatus(MPU9250_I2C, I2C_FLAG_STOPF))
        ;
      while (I2C_GetFlagStatus(MPU9250_I2C, I2C_FLAG_BUSY))
        ;

      // reset
      I2C_AcknowledgeConfig(MPU9250_I2C, ENABLE);
      direction = I2C_Direction_Transmitter;
      i = 0u;

      // extract raw magnetometer data
      // if data is ready, then fill
      if ((0x01 & data[0]) && !data[7]) {
        AK8963_data.timestamp = timestamp;
        AK8963_data.magn[0] = ((uint16_t)data[2] << 8u | (uint16_t)data[1]);
        AK8963_data.magn[1] = ((uint16_t)data[4] << 8u | (uint16_t)data[3]);
        AK8963_data.magn[2] = ((uint16_t)data[6] << 8u | (uint16_t)data[5]);
      }

      return 1u;
    }
    break;

  default:
    break;
  }
  return 0u;
}

float MPU9250_GetTemp(void) {
  return MPU9250_data.temp / MPU9250_TEMP_SENSITIVITY + MPU9250_TEMP_BIAS;
} // [deg C]

float MPU9250_GetGyroX(void) {
  return MPU9250_data.gyro[0] / MPU9250_GYRO_SENSITIVITY;
} // [rad/s]
float MPU9250_GetGyroY(void) {
  return MPU9250_data.gyro[1] / MPU9250_GYRO_SENSITIVITY;
} // [rad/s]
float MPU9250_GetGyroZ(void) {
  return MPU9250_data.gyro[2] / MPU9250_GYRO_SENSITIVITY;
} // [rad/s]

float MPU9250_GetAcclX(void) {
  return MPU9250_data.accl[0] / MPU9250_ACCL_SENSITIVITY;
} // [g]
float MPU9250_GetAcclY(void) {
  return MPU9250_data.accl[1] / MPU9250_ACCL_SENSITIVITY;
} // [g]
float MPU9250_GetAcclZ(void) {
  return MPU9250_data.accl[2] / MPU9250_ACCL_SENSITIVITY;
} // [g]

float MPU9250_GetMagnX(void) {
  return AK8963_data.magn[0] * AK8963_LSB2uT_X;
} // [uT]
float MPU9250_GetMagnY(void) {
  return AK8963_data.magn[1] * AK8963_LSB2uT_Y;
} // [uT]
float MPU9250_GetMagnZ(void) {
  return AK8963_data.magn[2] * AK8963_LSB2uT_Z;
} // [uT]
/**
 * @}
 */
