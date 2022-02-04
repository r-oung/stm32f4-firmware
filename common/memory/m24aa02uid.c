/**
 * @file    m24aa02uid.c
 * @author  Raymond Oung
 * @date    2014.12.10
 * @brief   2K I2C Serial EEPROM with Unique 32-bit Serial Number
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
#include "m24aa02uid.h"
#include "clock.h"
#include "stm32f4xx.h"

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
 * @brief Functions for reading MPU9150
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
#define VERBOSE_UART_ID 2u
#endif

#define M24AA02UID_ADDR 0xA0

#define M24AA02UID_MANUFACTURER_CODE_ADDR 0xFA
#define M24AA02UID_DEVICE_ID_ADDR 0xFB
#define M24AA02UID_UID_ADDR 0xFC
/**
 * @}
 */

/** @defgroup MPU9150_Private_Macros
 * @{
 */
/**
 * @}
 */

/** @defgroup MPU9150_Private_Function_Prototypes
 * @{
 */
uint8_t M24AA02UID_Read(uint8_t addr, uint8_t *data, uint16_t size);
/**
 * @}
 */

/** @defgroup MPU9150_Private_Variables
 * @{
 */
static I2C_TypeDef *M24AA02UID_I2C;

static uint8_t M24AA02UID_manufacturer_code;
static uint8_t M24AA02UID_device_id;
static uint32_t M24AA02UID_uid;
/**
 * @}
 */

/**
 * @brief  Initialize M24AA02UID
 * @param  I2Cx I2C peripheral type
 * @retval None
 */
void M24AA02UID_Init(I2C_TypeDef *I2Cx) {
  M24AA02UID_I2C = I2Cx;

  M24AA02UID_Read(M24AA02UID_MANUFACTURER_CODE_ADDR,
                  &M24AA02UID_manufacturer_code, 1u);
  Delay(10);
  M24AA02UID_Read(M24AA02UID_DEVICE_ID_ADDR, &M24AA02UID_device_id, 1u);
  M24AA02UID_Read(M24AA02UID_UID_ADDR, (uint8_t *)&M24AA02UID_uid, 4u);

#ifdef _VERBOSE
  // Manufacturer Code: 0x29, Device Code: 0x41
  char str[100];
  sprintf(str, "Manufacturer Code: 0x%02x\r\nDevice Code: 0x%02x\r\n",
          M24AA02UID_manufacturer_code, M24AA02UID_device_id);
  UART_Print(VERBOSE_UART_ID, str);
  sprintf(str, "UID: %u (0x%X)\r\n", (unsigned int)M24AA02UID_uid,
          (unsigned int)M24AA02UID_uid);
  UART_Print(VERBOSE_UART_ID, str);
#endif
}

/**
 * @brief  Read as many contiguous bytes from the M24AA02UID (Blocking)
 * @param  addr 8-bit start address [0x00..0xFF]
 * @param  data Pointer to a contiguous array of data
 * @param  size Number of bytes to read
 * @retval None
 */
uint8_t M24AA02UID_Read(uint8_t addr, uint8_t *data, uint16_t size) {
  uint16_t i;

  // Wait if the bus is busy
  while (I2C_GetFlagStatus(M24AA02UID_I2C, I2C_FLAG_BUSY))
    ;

  // Send START condition
  I2C_GenerateSTART(M24AA02UID_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(M24AA02UID_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send device control byte for writing
  I2C_Send7bitAddress(M24AA02UID_I2C, M24AA02UID_ADDR | 0x00,
                      I2C_Direction_Transmitter);

  // Test on EV6 and clear it
  while (!I2C_CheckEvent(M24AA02UID_I2C,
                         I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    ;

  // Send byte address
  I2C_SendData(M24AA02UID_I2C, addr);

  // Test on EV8 and clear it
  while (!I2C_CheckEvent(M24AA02UID_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  // Send START condition
  I2C_GenerateSTART(M24AA02UID_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(M24AA02UID_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send device control byte for reading
  I2C_Send7bitAddress(M24AA02UID_I2C, M24AA02UID_ADDR | 0x01,
                      I2C_Direction_Receiver);

  // Test on EV6 and clear it
  while (
      !I2C_CheckEvent(M24AA02UID_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    ;

  // Read data
  for (i = 0; i < size - 1; i++) {
    // Test on EV7 and clear it
    while (!I2C_CheckEvent(M24AA02UID_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
      ;

    // Read a byte
    data[i] = I2C_ReceiveData(M24AA02UID_I2C);
  }

  // Disable Acknowledgement on the last byte
  I2C_AcknowledgeConfig(M24AA02UID_I2C, DISABLE);

  // Test on EV7 and clear it
  while (!I2C_CheckEvent(M24AA02UID_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
    ;

  // Read the last byte
  data[i] = I2C_ReceiveData(M24AA02UID_I2C);

  // Send STOP Condition
  I2C_GenerateSTOP(M24AA02UID_I2C, ENABLE);
  while (I2C_GetFlagStatus(M24AA02UID_I2C, I2C_FLAG_STOPF))
    ;
  while (I2C_GetFlagStatus(M24AA02UID_I2C, I2C_FLAG_BUSY))
    ;

  // Enable Acknowledgement to be ready for another reception
  I2C_AcknowledgeConfig(M24AA02UID_I2C, ENABLE);

  return size;
}

/**
 * @brief  Read 32-bit unique serial number
 * @param  None
 * @retval Unique serial number
 */
uint32_t M24AA02UID_GetUID(void) { return M24AA02UID_uid; }
/**
 * @}
 */

/**
 * @}
 */
