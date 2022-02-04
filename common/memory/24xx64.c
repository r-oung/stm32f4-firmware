/**
 * @file    24xx64.c
 * @author  Raymond Oung
 * @date    2013.09.08
 * @brief   Functions that handle the 24AA64/24LC64/24FC64 EEPROM
 *          Memory is organized as a single block of 8K x 8-bit memory
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
#include "24xx64.h"
#include "circular_buffer.h"
#include "clock.h"
#include "i2c.h"

#ifdef _VERBOSE
#include "uart.h"
#include <stdio.h>
#include <stdlib.h>
#define VERBOSE_UART_ID 1u
#endif

/** @defgroup EEPROM_Private_Variables
 * @{
 */
static I2C_TypeDef *EEPROM_I2C;
static volatile uint8_t EEPROM_ADDR = 0x00;
static CircularBuffer_t EEPROM_writeBufIT;
static volatile uint16_t EEPROM_writeAddrIT = 0x00;
static volatile uint32_t EEPROM_writeT0 =
    0u; // timestamp of last write operation [msec]
/**
 * @}
 */

/** @defgroup EEPROM_Private_Defines
 * @{
 */
#ifdef _VERBOSE
#define VERBOSE_portID 1u
#endif

#define EEPROM_WRITE_DELAY                                                     \
  5u // minimum period between consecutive writes/reads [msec]
#define EEPROM_WRITE_MAX_BYTES                                                 \
  32u // maximum number of bytes that can be written in a cycle
/**
 * @}
 */

/**
 * @brief  Initialize EEPROM
 * @param  I2Cx I2C peripheral type
 * @param  addr EEPROM address
 * @retval None
 */
void EEPROM_Init(I2C_TypeDef *I2Cx, uint8_t addr) {
  EEPROM_I2C = I2Cx;
  EEPROM_ADDR = addr;
  CB_Init(&EEPROM_writeBufIT, 0xFF);
}

/**
 * @brief  Read as many contiguous bytes from the EEPROM (Blocking);
 *         Note: the internal address pointer will automatically roll over from
 *         address 0x1FFF to address 0x0000
 * @param  addr 12-bit start address [0x0000..0x1FFF]
 * @param  data Pointer to a contiguous array of data
 * @param  size Number of bytes to read
 * @retval None
 */
uint8_t EEPROM_Read(uint16_t addr, uint8_t *data, uint16_t size) {
  uint16_t i;

  // Wait if the bus is busy
  while (I2C_GetFlagStatus(EEPROM_I2C, I2C_FLAG_BUSY))
    ;

  // Send START condition
  I2C_GenerateSTART(EEPROM_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send device select code for writing
  I2C_Send7bitAddress(EEPROM_I2C, EEPROM_ADDR, I2C_Direction_Transmitter);

  // Test on EV6 and clear it
  while (
      !I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    ;

  // Send byte address (MSB)
  I2C_SendData(EEPROM_I2C, (uint8_t)(addr >> 8u));

  // Test on EV8 and clear it
  while (!I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  // Send byte address (LSB)
  I2C_SendData(EEPROM_I2C, (uint8_t)addr);

  // Test on EV8 and clear it
  while (!I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  // Send START condition
  I2C_GenerateSTART(EEPROM_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send device select code for reading
  I2C_Send7bitAddress(EEPROM_I2C, EEPROM_ADDR | 0x01, I2C_Direction_Receiver);

  // Test on EV6 and clear it
  while (!I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    ;

  // Read data
  for (i = 0; i < size - 1; i++) {
    // Test on EV7 and clear it
    while (!I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
      ;

    // Read a byte
    data[i] = I2C_ReceiveData(EEPROM_I2C);
  }

  // Disable Acknowledgement on the last byte
  I2C_AcknowledgeConfig(EEPROM_I2C, DISABLE);

  // Test on EV7 and clear it
  while (!I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
    ;

  // Read the last byte
  data[i] = I2C_ReceiveData(EEPROM_I2C);

  // Send STOP Condition
  I2C_GenerateSTOP(EEPROM_I2C, ENABLE);
  while (I2C_GetFlagStatus(EEPROM_I2C, I2C_FLAG_STOPF))
    ;
  while (I2C_GetFlagStatus(EEPROM_I2C, I2C_FLAG_BUSY))
    ;

  // Enable Acknowledgement to be ready for another reception
  I2C_AcknowledgeConfig(EEPROM_I2C, ENABLE);

  return size;
}

/**
 * @brief  Write up to 32 contiguous bytes to the EEPROM (Blocking);
 *         Note: a roll-over occurs if more than 32 bytes are sent, and the
 *         previously received data will be overwritten
 * @note   Write cycle time is 5 msec; so you'll need to wait that long between
 * consecutive writes/reads
 * @param  addr 12-bit start address [0x0000..0x1FFF]
 * @param  data Pointer to a contiguous array of data
 * @param  size Number of bytes contained in the array of data to be written
 * [<=32]
 * @retval None
 */
uint8_t EEPROM_Write(uint16_t addr, uint8_t *data, uint16_t size) {
  uint16_t i;

  // Wait if the bus is busy
  while (I2C_GetFlagStatus(EEPROM_I2C, I2C_FLAG_BUSY))
    ;

  // Send START condition
  I2C_GenerateSTART(EEPROM_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send device select code for writing
  I2C_Send7bitAddress(EEPROM_I2C, EEPROM_ADDR, I2C_Direction_Transmitter);

  // Test on EV6 and clear it
  while (
      !I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    ;

  // Send byte address (MSB)
  I2C_SendData(EEPROM_I2C, (uint8_t)(addr >> 8u));

  // Test on EV8 and clear it
  while (!I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  // Send byte address (LSB)
  I2C_SendData(EEPROM_I2C, (uint8_t)addr);

  // Test on EV8 and clear it
  while (!I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  // Write data
  for (i = 0; i < size; i++) {
    // Send data
    I2C_SendData(EEPROM_I2C, data[i]);

    // Test on EV8 and clear it
    while (!I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
      ;
  }

  // Send STOP Condition
  I2C_GenerateSTOP(EEPROM_I2C, ENABLE);
  while (I2C_GetFlagStatus(EEPROM_I2C, I2C_FLAG_STOPF))
    ;
  while (I2C_GetFlagStatus(EEPROM_I2C, I2C_FLAG_BUSY))
    ;

  return size;
}

/**
 * @brief  Write up to 32 contiguous bytes to the EEPROM (Non-Blocking);
 *         Note: the internal address pointer will automatically roll over from
 *         address 0x1FFF to address 0x0000
 * @note   Write cycle time is 5 msec; so you'll need to wait that long between
 * consecutive writes
 * @param  addr 12-bit start address [0x0000..0x1FFF]
 * @param  data Pointer to a contiguous array of data
 * @param  size Number of bytes to write
 * @retval None
 */
uint8_t EEPROM_WriteIT(uint16_t addr, uint8_t *data, uint16_t size) {
  // there needs to be a minimum delay between consecutive writes/reads
  if (GetTime() - EEPROM_writeT0 < EEPROM_WRITE_DELAY)
    return 0u;

  // do not exceed maximum write capacity
  if (size > EEPROM_WRITE_MAX_BYTES)
    return 0u;

  uint16_t i;
  for (i = 0; i < size; i++) {
    CB_Write(&EEPROM_writeBufIT, &data[i]);
  }

  EEPROM_writeAddrIT = addr;
  I2C_SetIT(EEPROM_I2C, ENABLE);
  I2C_GenerateSTART(EEPROM_I2C, ENABLE);
  EEPROM_writeT0 = GetTime();

  return size;
}

/**
 * @brief  EEPROM Interrupt service routine for non-blocking write
 * @param  None
 * @retval None
 */
uint8_t EEPROM_WriteIT_ISR(void) {
  static uint8_t state = 0u; // transmission state
  uint8_t byte;

  switch (I2C_GetLastEvent(EEPROM_I2C)) {
  // Test on EV5 and clear it
  case I2C_EVENT_MASTER_MODE_SELECT:
    // Send device select code for writing
    I2C_Send7bitAddress(EEPROM_I2C, EEPROM_ADDR, I2C_Direction_Transmitter);
    break;

  // Test on EV6 and clear it
  case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
    // Send byte address (MSB)
    I2C_SendData(EEPROM_I2C, (uint8_t)(EEPROM_writeAddrIT >> 8u));
    state = 0u; // reset state
    break;

  case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
    switch (state) {
    case 0: // Send byte address (LSB)
      I2C_SendData(EEPROM_I2C, (uint8_t)EEPROM_writeAddrIT);
      state++;
      break;

    case 1: // Send data
      CB_Read(&EEPROM_writeBufIT, &byte);
      I2C_SendData(EEPROM_I2C, byte);

      // stop transmission once everything has been transmitted
      if (CB_IsEmpty(&EEPROM_writeBufIT)) {
        state++;
      }
      break;

    default: // Send STOP Condition
      I2C_GenerateSTOP(EEPROM_I2C, ENABLE);
      while (I2C_GetFlagStatus(EEPROM_I2C, I2C_FLAG_STOPF))
        ;
      EEPROM_writeT0 = GetTime();
      I2C_SetIT(EEPROM_I2C, DISABLE);
      I2C_ITStop(EEPROM_I2C);
      return 1u;
      break;
    }
    break;

  default:
    break;
  }

  return 0u;
}

#ifdef _VERBOSE
/**
 * @brief  Print contiguous array of EEPROM memory (Blocking)
 * @param  s_addr Start address
 * @param  e_addr End address
 * @retval None
 */
void EEPROM_Print(uint16_t s_addr, uint16_t e_addr) {
  uint8_t *data;
  uint16_t size;

  if (e_addr < s_addr) {
    UART_Print(VERBOSE_UART_ID,
               "Invalid Request: start-address must be of smaller or equal "
               "value to the end-address\r\n");
    return;
  }

  // there needs to be a minimum delay between consecutive writes/reads
  Delay(EEPROM_WRITE_DELAY);

  size = e_addr - s_addr + 1;
  data = (uint8_t *)calloc(size, sizeof(uint8_t));
  EEPROM_Read(s_addr, data, size);

  char str[100];
  sprintf(str, "EEPROM | 0x%04x -> 0x%04x\r\n", s_addr, e_addr);
  UART_Print(VERBOSE_UART_ID, str);
  uint16_t i;
  for (i = 0; i < size; i++) {
    sprintf(str, "0x%02x ", data[i]);
    UART_Print(VERBOSE_UART_ID, str);
    Delay(1);
  }
  UART_Print(VERBOSE_UART_ID, "\r\n");

  free(data);
}
#endif