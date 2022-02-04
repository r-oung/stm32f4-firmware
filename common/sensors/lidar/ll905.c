/**
 * @file    ll905.c
 * @author  Raymond Oung
 * @date    2014.04.19
 * @brief   LL905 LIDAR functions
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
#include "ll905.h"
#include "clock.h"

#ifdef _VERBOSE
#include "uart.h"
#include <stdio.h>
#include <stdlib.h>
#endif

/** @addtogroup Source
 * @{
 */

/** @addtogroup LL905
 * @{
 */

/** @defgroup LL905
 * @brief Functions that handle the LL905 pressure sensor
 * @{
 */

/** @defgroup LL905_Private_Defines
 * @{
 */
#ifdef _VERBOSE
#define VERBOSE_portID 1u
#endif

#define LL905_ADDR 0xC4 // Default I2C Address of LIDAR-Lite (0x62 << 1)
#define LL905_READ 0x01
#define LL905_WRITE 0x00

#define LL905_MEASURE_REG 0x00 // Register to write to initiate ranging
#define LL905_MEASURE_VAL 0x04 // Value to initiate ranging

#define LL905_READ_REG 0x8F // Register to get both High and Low bytes in 1 call

/**
 * @}
 */

/** @defgroup LL905_Private_Variables
 * @{
 */
static I2C_TypeDef *LL905_I2C;
/**
 * @}
 */

/** @defgroup LL905_Private_Functions
 * @{
 */
void LL905_StartMeasurement(void);
void LL905_ReadData(uint8_t cmd, void *data, uint8_t num);
/**
 * @}
 */

/**
 * @brief  Initialises LL905
 * @param  I2Cx  I2C perihperal type [I2C1..I2C2]
 * @retval 0=Failure, 1=Success
 */
uint8_t LL905_Init(I2C_TypeDef *I2Cx) {
  uint8_t retval = 1u;

  LL905_I2C = I2Cx;

  Delay(100);

  while (1) {
    uint8_t buf[2];
    LL905_ReadData(0x8f, buf, 2);

    uint16_t range = (buf[0] << 8u) | buf[1]; // [m]

    char str[100];
    sprintf(str, "0x%02x 0x%02x | range: %u cm\r\n", buf[0], buf[1], range);
    UART_Print(1, str);

    LL905_StartMeasurement();
    Delay(50);
  }

  return retval;
}

/**
 * @brief  Send a byte of data to LL905
 * @param  cmd Command
 * @retval None
 */
void LL905_StartMeasurement(void) {
  // Send START condition
  I2C_GenerateSTART(LL905_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(LL905_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send Slave LL905 Address
  I2C_Send7bitAddress(LL905_I2C, LL905_ADDR, I2C_Direction_Transmitter);

  // Test on EV6 and clear it
  while (!I2C_CheckEvent(LL905_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    ;

  // Send LL905 command
  I2C_SendData(LL905_I2C, LL905_MEASURE_REG);

  // Test on EV8 and clear it
  while (!I2C_CheckEvent(LL905_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  // Send LL905 command
  I2C_SendData(LL905_I2C, LL905_MEASURE_VAL);

  // Test on EV8 and clear it
  while (!I2C_CheckEvent(LL905_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  // Send STOP Condition
  I2C_GenerateSTOP(LL905_I2C, ENABLE);
  while (I2C_GetFlagStatus(LL905_I2C, I2C_FLAG_STOPF))
    ;
  while (I2C_GetFlagStatus(LL905_I2C, I2C_FLAG_BUSY))
    ;
}

/**
 * @brief  Read data from LL905
 * @param  cmd Command
 * @param  data Pointer to data to save the data
 * @param  num Number of bytes to read
 * @retval None
 */
void LL905_ReadData(uint8_t cmd, void *data, uint8_t num) {
  uint8_t i;

  // Send START condition
  I2C_GenerateSTART(LL905_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(LL905_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send slave LL905 address for writing
  I2C_Send7bitAddress(LL905_I2C, LL905_ADDR, I2C_Direction_Transmitter);

  // Test on EV6 and clear it
  while (!I2C_CheckEvent(LL905_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    ;

  // Send LL905 command
  I2C_SendData(LL905_I2C, cmd);

  // Test on EV8 and clear it
  while (!I2C_CheckEvent(LL905_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  Delay(20);

  // Send START condition a second time
  I2C_GenerateSTART(LL905_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(LL905_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send slave LL905 address for reading
  I2C_Send7bitAddress(LL905_I2C, LL905_ADDR, I2C_Direction_Receiver);

  // Test on EV6 and clear it
  while (!I2C_CheckEvent(LL905_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    ;

  // Read N bytes
  for (i = 0; i < num - 1; i++) {
    // Test on EV7 and clear it
    while (!I2C_CheckEvent(LL905_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
      ;

    // Read a byte
    *((uint8_t *)data + i) = I2C_ReceiveData(LL905_I2C);
  }

  // Disable Acknowledgement on the last byte
  I2C_AcknowledgeConfig(LL905_I2C, DISABLE);

  // Test on EV7 and clear it
  while (!I2C_CheckEvent(LL905_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
    ;

  // Read a byte
  *((uint8_t *)data + i) = I2C_ReceiveData(LL905_I2C);

  // Send STOP Condition
  I2C_GenerateSTOP(LL905_I2C, ENABLE);
  while (I2C_GetFlagStatus(LL905_I2C, I2C_FLAG_STOPF))
    ;
  while (I2C_GetFlagStatus(LL905_I2C, I2C_FLAG_BUSY))
    ;

  // Enable Acknowledgement to be ready for another reception
  I2C_AcknowledgeConfig(LL905_I2C, ENABLE);
}

/**
 * @}
 */

/**
 * @}
 */
