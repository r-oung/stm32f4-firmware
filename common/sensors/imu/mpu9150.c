/**
 * @file    mpu9150.c
 * @author  Raymond Oung
 * @date    2012.03.21
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
#include "mpu9150.h"
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

/** @defgroup MPU9150
 * @brief Functions for reading MPU9150 (rate-gyro + accelerometer)
 * @{
 */

/** @defgroup MPU9150_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @defgroup MPU9150_Private_Defines
 * @{
 */
#ifdef _VERBOSE
#define VERBOSE_UART_ID 1u
#endif

#define MPU9150_ADDR 0x68 // AD0 = 0
#define AK8975_ADDR                                                            \
  0x0C // Magnetometer address, see Section 7.15 of PS-MPU-9150A-00 rev. 4.3

#define MPU9150_CONFIG_REG 0x1A
#define MPU9150_GYRO_CONFIG_REG 0x1B
#define MPU9150_ACCL_CONFIG_REG 0x1C

#define MPU9150_ACCL_DATA_REG 0x3B // start register of accelerometer data
#define MPU9150_GYRO_DATA_REG 0x43 // start register of gyro data
#define MPU9150_TEMP_DATA_REG 0x41 // start register of temperature data

#define MPU9150_INT_PIN_CFG_REG 0x37 // pin configuration register
#define MPU9150_INT_PIN_CFG_VAL                                                \
  0x02 // enable I2C bypass/pass-through (needed for magnetomer)

#define MPU9150_USER_CTRL_REG 0x6A // user control register
#define MPU9150_USER_CTRL_VAL                                                  \
  0x00 // auxiliary I2C bus is logically driven by the primary I2C bus

#define MPU9150_PWR_MGMT_1_REG 0x6B // power management register
#define MPU9150_PWR_MGMT_1_VAL 0x01 // use gyroscope-based clock source (x-axis)

#define AK8975_ST1_REG 0x02  // status 1 register (data-ready)
#define AK8975_HXL_REG 0x03  // start register of magnetometer data
#define AK8975_ASAX_REG 0x10 // start register of sensitivity adjustment values

#define AK8975_CNTL_REG 0x0A // control register
#define AK8975_CNTL_VAL 0x02 // single measurement mode
/**
 * @}
 */

/** @defgroup MPU9150_Private_Macros
 * @{
 */
/**
 * @}
 */

/** @defgroup MPU9150_Private_Variables
 * @{
 */
static I2C_TypeDef *MPU9150_I2C;
static const float MPU9150_TEMP_BIAS = 36.53f; // [deg C]
static const float MPU9150_TEMP_SENSITIVITY =
    340.0f; // [LSB/deg C] @TODO This is the temperature sensitivity of MPU9150,
            // not MPU9250. No value for MPU9250 was found.
static volatile float MPU9150_GYRO_SENSITIVITY =
    7505.74711621f; // for +/- 250 deg/s range; 131.0f [LSB/deg/s] * 180/pi
                    // [deg/rad]
static volatile float MPU9150_ACCL_SENSITIVITY =
    4096.0f;                                  // for +/- 8 g range [LSB/mg]
static volatile float AK8975_LSB2uT_X = 0.0f; // [uT/LSB]
static volatile float AK8975_LSB2uT_Y = 0.0f; // [uT/LSB]
static volatile float AK8975_LSB2uT_Z = 0.0f; // [uT/LSB]

static volatile struct MPU9150_data_t {
  float timestamp; // [msec]
  int16_t gyro[3]; // raw rate-gyro data
  int16_t accl[3]; // raw accelerometer data
  int16_t temp;    // raw temperature data
} MPU9150_data;

static volatile struct AK8975_data_t {
  float timestamp; // [msec]
  int16_t magn[3]; // raw magnetometer data
  int8_t status;   // data ready status
} AK8975_data;
/**
 * @}
 */

/* Global variables  ---------------------------------------------------------*/

/** @defgroup MPU9150_Private_Functions
 * @{
 */
static void MPU9150_TxByte(uint8_t reg, uint8_t val);
static void MPU9150_RxData(uint8_t reg, uint8_t *data, uint8_t num);
static void AK8975_TxByte(uint8_t reg, uint8_t val);
static void AK8975_RxData(uint8_t reg, uint8_t *data, uint8_t num);
/**
 * @}
 */

/**
 * @brief  Initializes MPU9150.
 * @param  I2Cx  I2C perihperal type [I2C1..I2C2]
 * @retval None
 */
void MPU9150_Init(I2C_TypeDef *I2Cx, uint8_t bandwidth, uint8_t gyro_range,
                  uint8_t accl_range) {
  MPU9150_I2C = I2Cx;

  // initialize MPU9150
  MPU9150_TxByte(MPU9150_PWR_MGMT_1_REG, MPU9150_PWR_MGMT_1_VAL);
  MPU9150_TxByte(MPU9150_CONFIG_REG, bandwidth);
  MPU9150_TxByte(MPU9150_GYRO_CONFIG_REG, gyro_range);
  MPU9150_TxByte(MPU9150_ACCL_CONFIG_REG, accl_range);

  switch (gyro_range) {
  case MPU9150_GYRO_RANGE_250: // +/- 250 deg/s range
    MPU9150_GYRO_SENSITIVITY =
        131.0f * 180 / M_PI; // 131.0f [LSB/deg/s] * 180/pi [deg/rad]
    break;
  case MPU9150_GYRO_RANGE_500: // +/- 500 deg/s range
    MPU9150_GYRO_SENSITIVITY =
        65.5f * 180 / M_PI; // 65.5f [LSB/deg/s] * 180/pi [deg/rad]
    break;
  case MPU9150_GYRO_RANGE_1000: // +/- 1000 deg/s range
    MPU9150_GYRO_SENSITIVITY =
        32.75f * 180 / M_PI; // 32.75f [LSB/deg/s] * 180/pi [deg/rad]
    break;
  case MPU9150_GYRO_RANGE_2000: // +/- 2000 deg/s range
    MPU9150_GYRO_SENSITIVITY =
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

  switch (accl_range) {
  case MPU9150_ACCL_RANGE_2:             // +/- 2 g range
    MPU9150_ACCL_SENSITIVITY = 16384.0f; // [LSB/mg]
    break;
  case MPU9150_ACCL_RANGE_4:            // +/- 4 g range
    MPU9150_ACCL_SENSITIVITY = 8192.0f; // [LSB/mg]
    break;
  case MPU9150_ACCL_RANGE_8:            // +/- 8 g range
    MPU9150_ACCL_SENSITIVITY = 4096.0f; // [LSB/mg]
    break;
  case MPU9150_ACCL_RANGE_16:           // +/- 16 g range
    MPU9150_ACCL_SENSITIVITY = 2048.0f; // [LSB/mg]
    break;
  default:
#ifdef _VERBOSE
    UART_Print(VERBOSE_UART_ID, "ERROR: Accl-Config2\r\n");
#endif
    while (1)
      ; // error
    break;
  }

  MPU9150_TxByte(MPU9150_USER_CTRL_REG,
                 MPU9150_USER_CTRL_VAL); // needed for magnetometer
  MPU9150_TxByte(MPU9150_INT_PIN_CFG_REG,
                 MPU9150_INT_PIN_CFG_VAL); // needed for magnetometer

  // Get AK8975 sensitivity adjustment values
  uint8_t data[3];
  AK8975_RxData(AK8975_ASAX_REG, data, 3);

  // Compute sensitivity adjustment values and conversion factors
  AK8975_LSB2uT_X = 0.3f * ((((float)data[0] - 128.0f) * 0.5f) / 128.0f +
                            1.0f); // 0.3f [uT/LSB] * (((ASA-128)*0.5)/128+1)
  AK8975_LSB2uT_Y =
      0.3f * ((((float)data[1] - 128.0f) * 0.5f) / 128.0f +
              1.0f); // see Section 6.11 of RM-MPU-9150A-00 rev. 4.2
  AK8975_LSB2uT_Z = 0.3f * ((((float)data[2] - 128.0f) * 0.5f) / 128.0f + 1.0f);
  // char str[100]; sprintf(str, "sensitivity: %.3f %.3f %.3f\r\n",
  // AK8975_LSB2uT_X, AK8975_LSB2uT_Y, AK8975_LSB2uT_Z); UART_WriteTxt(4, str);
  // sprintf(str, "sensitivity: %x %x %x\r\n", data[0], data[1], data[2]);
  // UART_WriteTxt(4, str);

  // call for new magnetometer measurement
  AK8975_TxByte(AK8975_CNTL_REG, AK8975_CNTL_VAL);
}

/**
 * @brief  Send a byte of data to MPU9150
 * @param  reg Register for writing the data
 * @param  val Value to be written
 * @retval None
 */
static void MPU9150_TxByte(uint8_t reg, uint8_t val) {
  // Send START condition
  I2C_GenerateSTART(MPU9150_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(MPU9150_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send Slave MPU9150 Address
  I2C_Send7bitAddress(MPU9150_I2C, MPU9150_ADDR << 1u,
                      I2C_Direction_Transmitter);

  // Test on EV6 and clear it
  while (
      !I2C_CheckEvent(MPU9150_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    ;

  // Send the internal MPU9150 Address to write to
  I2C_SendData(MPU9150_I2C, (uint8_t)reg);

  // Test on EV8 and clear it
  while (!I2C_CheckEvent(MPU9150_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  // Send register content
  I2C_SendData(MPU9150_I2C, (uint8_t)val);

  // Test on EV8 and clear it
  while (!I2C_CheckEvent(MPU9150_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  // Send STOP Condition
  I2C_GenerateSTOP(MPU9150_I2C, ENABLE);
  while (I2C_GetFlagStatus(MPU9150_I2C, I2C_FLAG_STOPF))
    ;
  while (I2C_GetFlagStatus(MPU9150_I2C, I2C_FLAG_BUSY))
    ;
}

/**
 * @brief  Read rate-gyro or accelerometer data from MPU9150
 * @param  reg Register to read
 * @param  data Pointer to data to save the data
 * @param  num Number of bytes to read
 * @retval None
 */
static void MPU9150_RxData(uint8_t reg, uint8_t *data, uint8_t num) {
  uint8_t i;

  // Send START condition
  I2C_GenerateSTART(MPU9150_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(MPU9150_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send slave MPU9150 address for writing
  I2C_Send7bitAddress(MPU9150_I2C, MPU9150_ADDR << 1u,
                      I2C_Direction_Transmitter);

  // Test on EV6 and clear it
  while (
      !I2C_CheckEvent(MPU9150_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    ;

  // Send the internal MPU9150 address to read from
  I2C_SendData(MPU9150_I2C, reg);

  // Test on EV8 and clear it
  while (!I2C_CheckEvent(MPU9150_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  // Send START condition a second time
  I2C_GenerateSTART(MPU9150_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(MPU9150_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send slave MPU9150 address for reading
  I2C_Send7bitAddress(MPU9150_I2C, MPU9150_ADDR << 1u, I2C_Direction_Receiver);

  // Test on EV6 and clear it
  while (!I2C_CheckEvent(MPU9150_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    ;

  // Read N bytes
  for (i = 0u; i < num - 1u; i++) {
    // Test on EV7 and clear it
    while (!I2C_CheckEvent(MPU9150_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
      ;

    // Read a byte
    data[i] = I2C_ReceiveData(MPU9150_I2C);
  }

  // Disable Acknowledgement on the last byte
  I2C_AcknowledgeConfig(MPU9150_I2C, DISABLE);

  // Test on EV7 and clear it
  while (!I2C_CheckEvent(MPU9150_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
    ;

  // Read a byte
  data[i] = I2C_ReceiveData(MPU9150_I2C);

  // Send STOP Condition
  I2C_GenerateSTOP(MPU9150_I2C, ENABLE);
  while (I2C_GetFlagStatus(MPU9150_I2C, I2C_FLAG_STOPF))
    ;
  while (I2C_GetFlagStatus(MPU9150_I2C, I2C_FLAG_BUSY))
    ;

  // Enable Acknowledgement to be ready for another reception
  I2C_AcknowledgeConfig(MPU9150_I2C, ENABLE);
}

/**
 * @brief  Send a byte of data to AK8975
 * @param  reg Register for writing the data
 * @param  val Value to be written
 * @retval None
 */
static void AK8975_TxByte(uint8_t reg, uint8_t val) {
  // Send START condition
  I2C_GenerateSTART(MPU9150_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(MPU9150_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send Slave AK8975 address
  I2C_Send7bitAddress(MPU9150_I2C, AK8975_ADDR << 1u,
                      I2C_Direction_Transmitter);

  // Test on EV6 and clear it
  while (
      !I2C_CheckEvent(MPU9150_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    ;

  // Send the internal AK8975 address to write to
  I2C_SendData(MPU9150_I2C, (uint8_t)reg);

  // Test on EV8 and clear it
  while (!I2C_CheckEvent(MPU9150_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  // Send register content
  I2C_SendData(MPU9150_I2C, (uint8_t)val);

  // Test on EV8 and clear it
  while (!I2C_CheckEvent(MPU9150_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  // Send STOP Condition
  I2C_GenerateSTOP(MPU9150_I2C, ENABLE);
  while (I2C_GetFlagStatus(MPU9150_I2C, I2C_FLAG_STOPF))
    ;
  while (I2C_GetFlagStatus(MPU9150_I2C, I2C_FLAG_BUSY))
    ;
}

/**
 * @brief  Read magnetometer data from AK8975
 * @param  reg Register to read
 * @param  data Pointer to data to save the data
 * @param  num Number of bytes to read
 * @retval None
 */
static void AK8975_RxData(uint8_t reg, uint8_t *data, uint8_t num) {
  uint8_t i;

  // Send START condition
  I2C_GenerateSTART(MPU9150_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(MPU9150_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send slave AK8975 address for writing
  I2C_Send7bitAddress(MPU9150_I2C, AK8975_ADDR << 1u,
                      I2C_Direction_Transmitter);

  // Test on EV6 and clear it
  while (
      !I2C_CheckEvent(MPU9150_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    ;

  // Send the internal AK8975 address to read from
  I2C_SendData(MPU9150_I2C, reg);

  // Test on EV8 and clear it
  while (!I2C_CheckEvent(MPU9150_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  // Send START condition a second time
  I2C_GenerateSTART(MPU9150_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(MPU9150_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send slave AK8975 address for reading
  I2C_Send7bitAddress(MPU9150_I2C, AK8975_ADDR << 1u, I2C_Direction_Receiver);

  // Test on EV6 and clear it
  while (!I2C_CheckEvent(MPU9150_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    ;

  // Read N bytes
  for (i = 0u; i < num - 1u; i++) {
    // Test on EV7 and clear it
    while (!I2C_CheckEvent(MPU9150_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
      ;

    // Read a byte
    data[i] = I2C_ReceiveData(MPU9150_I2C);
  }

  // Disable Acknowledgement on the last byte
  I2C_AcknowledgeConfig(MPU9150_I2C, DISABLE);

  // Test on EV7 and clear it
  while (!I2C_CheckEvent(MPU9150_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
    ;

  // Read a byte
  data[i] = I2C_ReceiveData(MPU9150_I2C);

  // Send STOP Condition
  I2C_GenerateSTOP(MPU9150_I2C, ENABLE);
  while (I2C_GetFlagStatus(MPU9150_I2C, I2C_FLAG_STOPF))
    ;
  while (I2C_GetFlagStatus(MPU9150_I2C, I2C_FLAG_BUSY))
    ;

  // Enable Acknowledgement to be ready for another reception
  I2C_AcknowledgeConfig(MPU9150_I2C, ENABLE);
}

/**
 * @brief  MPU9150 Interrupt service routine (Blocking)
 * @retval None
 */
void MPU9150_ReadSensor(void) {
  static uint8_t data[14];

  // Get MPU9150 data (accelerometer and rate-gyro)
  MPU9150_data.timestamp = GetTimeU();
  MPU9150_RxData(MPU9150_ACCL_DATA_REG, data, 14u);

  // extract raw accelerometer data
  MPU9150_data.accl[0] = ((uint16_t)data[0] << 8u | (uint16_t)data[1]);
  MPU9150_data.accl[1] = ((uint16_t)data[2] << 8u | (uint16_t)data[3]);
  MPU9150_data.accl[2] = ((uint16_t)data[4] << 8u | (uint16_t)data[5]);

  // extract raw temperature data
  MPU9150_data.temp = ((int16_t)data[6] << 8u | (int16_t)data[7]);

  // extract raw rate-gyroscope data
  MPU9150_data.gyro[0] = ((uint16_t)data[8] << 8u | (uint16_t)data[9]);
  MPU9150_data.gyro[1] = ((uint16_t)data[10] << 8u | (uint16_t)data[11]);
  MPU9150_data.gyro[2] = ((uint16_t)data[12] << 8u | (uint16_t)data[13]);
}

/**
 * @brief  AK8975 Interrupt service routine (Blocking)
 * @retval None
 */
void AK8975_ReadSensor(void) {
  static uint8_t ready = 0u;
  uint8_t data[6] = {0u, 0u, 0u, 0u, 0u, 0u};

  // wait for data to be ready
  if (!ready) {
    AK8975_RxData(AK8975_ST1_REG, &ready, 1);
  } else {
    ready = 0u; // reset

    // Get AK8975 data (magnetometer)
    AK8975_data.timestamp = GetTimeU();
    AK8975_RxData(AK8975_HXL_REG, data, 6);

    // extract raw magnetometer data
    AK8975_data.magn[0] = ((uint16_t)data[1] << 8 | (uint16_t)data[0]);
    AK8975_data.magn[1] = ((uint16_t)data[3] << 8 | (uint16_t)data[2]);
    AK8975_data.magn[2] = ((uint16_t)data[5] << 8 | (uint16_t)data[4]);

    // call for new magnetometer measurement
    AK8975_TxByte(AK8975_CNTL_REG, AK8975_CNTL_VAL);
  }
}

/**
 * @brief  MPU9150 Interrupt service routine (I2C Interrupt)
 * @param  None
 * @retval None
 */
uint8_t MPU9150_ReadSensorIT(void) {
  static uint8_t direction = I2C_Direction_Transmitter; // I2C direction
  static uint8_t i = 0u;                                // byte counter
  static uint8_t data[14]; // read a total of 14 bytes

  switch (I2C_GetLastEvent(MPU9150_I2C)) {
  // Test on EV5 and clear it
  case I2C_EVENT_MASTER_MODE_SELECT:
    // Send Slave MPU9150 Address
    I2C_Send7bitAddress(MPU9150_I2C, MPU9150_ADDR << 1, direction);
    break;

  // Test on EV6 and clear it
  case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
    // Send the internal MPU9150 address to read from
    I2C_SendData(MPU9150_I2C, MPU9150_ACCL_DATA_REG);
    MPU9150_data.timestamp = GetTimeU();
    break;

  case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
    I2C_ITConfig(MPU9150_I2C, I2C_IT_BUF, ENABLE);
    // Send START condition a second time
    I2C_GenerateSTART(MPU9150_I2C, ENABLE);
    direction = I2C_Direction_Receiver;
    break;

  case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
    break;

  // Test on EV7 and clear it
  case I2C_EVENT_MASTER_BYTE_RECEIVED:
    // Read a byte
    data[i++] = I2C_ReceiveData(MPU9150_I2C);

    // Disable Acknowledgement on the last byte
    if (i == 13u) {
      // Disable Acknowledgement on the last byte
      I2C_AcknowledgeConfig(MPU9150_I2C, DISABLE);
    }

    if (i == 14u) {
      // Send STOP Condition
      I2C_GenerateSTOP(MPU9150_I2C, ENABLE);
      while (I2C_GetFlagStatus(MPU9150_I2C, I2C_FLAG_STOPF))
        ;
      while (I2C_GetFlagStatus(MPU9150_I2C, I2C_FLAG_BUSY))
        ;

      // reset
      I2C_AcknowledgeConfig(MPU9150_I2C, ENABLE);
      direction = I2C_Direction_Transmitter;
      i = 0u;

      // extract raw accelerometer data
      MPU9150_data.accl[0] = ((uint16_t)data[0] << 8u | (uint16_t)data[1]);
      MPU9150_data.accl[1] = ((uint16_t)data[2] << 8u | (uint16_t)data[3]);
      MPU9150_data.accl[2] = ((uint16_t)data[4] << 8u | (uint16_t)data[5]);

      // extract raw temperature data
      MPU9150_data.temp = ((int16_t)data[6] << 8u | (int16_t)data[7]);

      // extract raw rate-gyroscope data
      MPU9150_data.gyro[0] = ((uint16_t)data[8] << 8u | (uint16_t)data[9]);
      MPU9150_data.gyro[1] = ((uint16_t)data[10] << 8u | (uint16_t)data[11]);
      MPU9150_data.gyro[2] = ((uint16_t)data[12] << 8u | (uint16_t)data[13]);

      return 1u;
    }
    break;

  default:
    break;
  }
  return 0u;
}

/**
 * @brief  AK8975 Interrupt service routine (I2C Interrupt)
 * @param  None
 * @retval None
 */
uint8_t AK8975_ReadSensorIT(void) {
  static uint8_t direction = I2C_Direction_Transmitter; // I2C direction
  static uint8_t i = 0;                                 // byte counter
  static uint8_t data[8];        // read a total of 8 bytes
  static float timestamp = 0.0f; // timestamp [msec]

  switch (I2C_GetLastEvent(MPU9150_I2C)) {
  // Test on EV5 and clear it
  case I2C_EVENT_MASTER_MODE_SELECT:
    // Send Slave MPU9150 Address
    I2C_Send7bitAddress(MPU9150_I2C, AK8975_ADDR << 1, direction);
    break;

  // Test on EV6 and clear it
  case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
    // Send the internal MPU9150 address to read from
    // I2C_SendData(MPU9150_I2C, AK8975_HXL_REG);
    I2C_SendData(MPU9150_I2C, AK8975_ST1_REG);
    timestamp = GetTimeU();
    break;

  case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
    I2C_ITConfig(MPU9150_I2C, I2C_IT_BUF, ENABLE);
    // Send START condition a second time
    I2C_GenerateSTART(MPU9150_I2C, ENABLE);
    direction = I2C_Direction_Receiver;
    break;

  case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
    break;

  // Test on EV7 and clear it
  case I2C_EVENT_MASTER_BYTE_RECEIVED:
    // Read a byte
    data[i++] = I2C_ReceiveData(MPU9150_I2C);

    // Disable Acknowledgement on the last byte
    if (i == 7u) {
      // Disable Acknowledgement on the last byte
      I2C_AcknowledgeConfig(MPU9150_I2C, DISABLE);
    }

    if (i == 8u) {
      // Send STOP Condition
      I2C_GenerateSTOP(MPU9150_I2C, ENABLE);
      while (I2C_GetFlagStatus(MPU9150_I2C, I2C_FLAG_STOPF))
        ;
      while (I2C_GetFlagStatus(MPU9150_I2C, I2C_FLAG_BUSY))
        ;

      // reset
      I2C_AcknowledgeConfig(MPU9150_I2C, ENABLE);
      direction = I2C_Direction_Transmitter;
      i = 0u;

      // extract raw magnetometer data
      // if data is ready, then fill
      if ((0x01 & data[0]) & !data[7]) {
        AK8975_data.timestamp = timestamp;
        AK8975_data.magn[0] = ((uint16_t)data[2] << 8u | (uint16_t)data[1]);
        AK8975_data.magn[1] = ((uint16_t)data[4] << 8u | (uint16_t)data[3]);
        AK8975_data.magn[2] = ((uint16_t)data[6] << 8u | (uint16_t)data[5]);
      }

      return 1u;
    }
    break;

  default:
    break;
  }
  return 0u;
}

float MPU9150_GetTemp(void) {
  return MPU9150_data.temp / MPU9150_TEMP_SENSITIVITY + MPU9150_TEMP_BIAS;
} // [deg C]

float MPU9150_GetGyroX(void) {
  return MPU9150_data.gyro[0] / MPU9150_GYRO_SENSITIVITY;
} // [rad/s]
float MPU9150_GetGyroY(void) {
  return MPU9150_data.gyro[1] / MPU9150_GYRO_SENSITIVITY;
} // [rad/s]
float MPU9150_GetGyroZ(void) {
  return MPU9150_data.gyro[2] / MPU9150_GYRO_SENSITIVITY;
} // [rad/s]

float MPU9150_GetAcclX(void) {
  return MPU9150_data.accl[0] / MPU9150_ACCL_SENSITIVITY;
} // [g]
float MPU9150_GetAcclY(void) {
  return MPU9150_data.accl[1] / MPU9150_ACCL_SENSITIVITY;
} // [g]
float MPU9150_GetAcclZ(void) {
  return MPU9150_data.accl[2] / MPU9150_ACCL_SENSITIVITY;
} // [g]

float MPU9150_GetMagnX(void) {
  return AK8975_data.magn[0] * AK8975_LSB2uT_X;
} // [uT]
float MPU9150_GetMagnY(void) {
  return AK8975_data.magn[1] * AK8975_LSB2uT_Y;
} // [uT]
float MPU9150_GetMagnZ(void) {
  return AK8975_data.magn[2] * AK8975_LSB2uT_Z;
} // [uT]
/**
 * @}
 */

/**
 * @}
 */
