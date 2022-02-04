/**
 * @file    rm3100.c
 * @author  Raymond Oung
 * @date    2014.11.04
 * @brief   RM3100 functions
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
#include "rm3100.h"
#include "bits.h"

#ifdef _VERBOSE
#include "uart.h"
#include <stdio.h>
#include <string.h>
#endif

#include "clock.h" // debug
#include "led.h"   // debug

/** @addtogroup Source
 * @{
 */

/** @addtogroup Low_Level
 * @{
 */

/** @defgroup RM3100
 * @brief RM3100 setup
 * @{
 */

/** @defgroup RM3100_Private_Defines
 * @{
 */
#ifdef _VERBOSE
#define VERBOSE_UART_ID 1u
#endif

#define RM3100_POLL 0x00 // polls for a single measurement
#define RM3100_CMM 0x01  // initiates continuous measurement mode
#define RM3100_CCX 0x04  // cycle counter register -- X-axis
#define RM3100_CCY 0x06  // cycle counter register -- Y-axis
#define RM3100_CCZ 0x08  // cycle counter register -- Z-axis
#define RM3100_TMRC 0x0B // sets continuous measurement mode data rate

#define RM3100_MX 0x24 // measurement results -- X-axis
#define RM3100_MY 0x27 // measurement results -- Y-axis
#define RM3100_MZ 0x2A // measurement results -- Z-axis

#define RM3100_BIST 0x33   // built-in selft test
#define RM3100_STATUS 0x34 // status of DRDY
#define RM3100_HSHAKE 0x35 // handshake register
#define RM3100_REVID 0x36  // revision identification

#define RM3100_CMM600 0x92 // update rate of 600 Hz
#define RM3100_CMM300 0x93 // update rate of 300 Hz
#define RM3100_CMM150 0x94 // update rate of 150 Hz
#define RM3100_CMM75 0x95  // update rate of 75 Hz
#define RM3100_CMM37 0x96  // update rate of 37 Hz
#define RM3100_CMM18 0x97  // update rate of 18 Hz
#define RM3100_CMM9 0x98   // update rate of 9 Hz
/**
 * @}
 */

/** @defgroup RM3100_Private_Variables
 * @{
 */
static SPI_TypeDef *RM3100_SPIx;
static GPIO_TypeDef *RM3100_CS_port;
static volatile uint16_t RM3100_CS_pin;
static volatile int32_t RM3100_magn[3];
/**
 * @}
 */

/** @defgroup RM3100_Private_Function_Prototypes
 * @{
 */
static void RM3100_CS(uint8_t val);
static uint8_t RM3100_ReadByte(void);
static uint8_t RM3100_WriteByte(uint8_t byte);
static void RM3100_Read(uint8_t reg, uint8_t *buf, uint8_t size);
static void RM3100_Write(uint8_t reg, uint8_t val);
static int32_t RM3100_convert(uint32_t val);
static uint8_t RM3100_IsDataReady(void);

uint8_t RM3100_BuiltInSelfTest(void);
/**
 * @}
 */

/**
 * @brief  Initializes RM3100
 * @param  SPIx SPI peripheral type
 * @param  cs_clk Chip select clock source
 * @param  cs_port Chip select GPIO port
 * @param  cs_pin Chip select GPIO pin
 * @retval None
 */
void RM3100_Init(SPI_TypeDef *SPIx, uint32_t cs_clk, GPIO_TypeDef *cs_port,
                 uint16_t cs_pin) {
  RCC_AHB1PeriphClockCmd(cs_clk, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = cs_pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(cs_port, &GPIO_InitStructure);

  RM3100_SPIx = SPIx;
  RM3100_CS_port = cs_port;
  RM3100_CS_pin = cs_pin;

  // set the CMM update rate
  RM3100_Write(RM3100_TMRC, RM3100_CMM9);

  // initiate continuous measurement mode
  // use default cycle count value: 200 [min: 30, max: 400]
  // no alarm, read all sensor axes
  RM3100_Write(RM3100_CMM, 0x79);
}

/**
 * @brief  Toggle chip select
 * @param  val 1=High; 0=Low
 * @retval None
 */
static void RM3100_CS(uint8_t val) {
  switch (val) {
  case 0u:
    GPIO_ResetBits(RM3100_CS_port, RM3100_CS_pin);
    break;

  case 1u:
    GPIO_SetBits(RM3100_CS_port, RM3100_CS_pin);
    break;

  default:
    break;
  }
}

/**
 * @brief  Read a single byte
 * @param  None
 * @retval Value of the received byte
 */
static uint8_t RM3100_ReadByte(void) { return RM3100_WriteByte(0x00); }

/**
 * @brief  Write a single byte
 * @param  byte Byte to send
 * @retval Value of the received byte
 */
static uint8_t RM3100_WriteByte(uint8_t byte) {
  // Loop while DR register in not empty
  while (SPI_I2S_GetFlagStatus(RM3100_SPIx, SPI_I2S_FLAG_TXE) == RESET)
    ;

  // Send byte through the SPI peripheral
  SPI_I2S_SendData(RM3100_SPIx, byte);

  // Wait to receive a byte
  while (SPI_I2S_GetFlagStatus(RM3100_SPIx, SPI_I2S_FLAG_RXNE) == RESET)
    ;

  // Return the byte read from the SPI bus
  return SPI_I2S_ReceiveData(RM3100_SPIx);
}

//--------------------------------------------------------------------------------

/**
 * @brief  Read from RM3100's register(s)
 * @param  reg Register of memory buffer to be read
 * @param  buf Pointer to read buffer
 * @param  size Size of the data to be read
 * @retval None
 */
static void RM3100_Read(uint8_t reg, uint8_t *buf, uint8_t size) {
  RM3100_CS(0);

  // send register address to be read
  RM3100_WriteByte(0x80 | reg);

  // read data
  uint8_t i;
  for (i = 0; i < size; i++) {
    buf[i] = RM3100_ReadByte();
  }

  RM3100_CS(1);
}

/**
 * @brief  Write to RM3100's register
 * @param  reg Register of memory buffer to be written
 * @param  val Value to be written
 * @retval None
 */
static void RM3100_Write(uint8_t reg, uint8_t val) {
  RM3100_CS(0);

  // send register address to be read
  RM3100_WriteByte(0x00 | reg);

  // write data
  RM3100_WriteByte(val);

  RM3100_CS(1);
}

//--------------------------------------------------------------------------------

/**
 * @brief  Convert 24-bit 2's complement value to 32-bit 2's complement value
 * @param  24-bit 2's complement value
 * @retval 32-bit 2's complement value
 */
static int32_t RM3100_convert(uint32_t val) {
  if (BITVAL(val, 23u)) {
    // this is a negative number
    return -((~val & 0x00ffffff) + 1u);
  } else {
    return val;
  }
}

/**
 * @brief  Read RM3100's status register (data ready)
 * @param  None
 * @retval 0=Data Unavailable, 1=Data Available
 */
static uint8_t RM3100_IsDataReady(void) {
  uint8_t retval = 0u;
  RM3100_Read(RM3100_STATUS, &retval, 1u);

  return BITVAL(retval, 7u);
}

/**
 * @brief  Read measurement data
 * @param  None
 * @retval None
 */
static void RM3100_ReadMeasurement(void) {
  uint8_t data[9];
  RM3100_Read(RM3100_MX, data, 9u);

  RM3100_magn[0] = RM3100_convert((uint32_t)data[0] << 16u |
                                  (uint32_t)data[1] << 8u | data[2]);
  RM3100_magn[1] = RM3100_convert((uint32_t)data[3] << 16u |
                                  (uint32_t)data[4] << 8u | data[5]);
  RM3100_magn[2] = RM3100_convert((uint32_t)data[6] << 16u |
                                  (uint32_t)data[7] << 8u | data[8]);

  // char str[100]; sprintf(str,"%u %u %u %u\r\n", GetTime(), data[0], data[1],
  // data[2]); UART_Print(VERBOSE_UART_ID,str);
  LED_Blink(0, 100);
}

/**
 * @brief  Read a single measurement
 * @param  None
 * @retval None
 */
void RM3100_InitSingleMeasurement(void) {
  // initiate a single measurement across all sensor axes
  RM3100_Write(RM3100_POLL, 0x70);

  while (!RM3100_IsDataReady())
    ;
  RM3100_ReadMeasurement();
}

/**
 * @brief  Interrupt service routine for continuous read
 * @param  None
 * @retval 0=Data Unavailable, 1=Data Available
 */
uint8_t RM3100_ISR(void) {
  if (RM3100_IsDataReady()) {
    RM3100_ReadMeasurement();
    return 1u;
  }

  return 0u;
}

//--------------------------------------------------------------------------------

/**
 * @brief  Run built-in self test
 * @param  None
 * @retval 0=Failure, 1=Success
 */
uint8_t RM3100_BuiltInSelfTest(void) {
  uint8_t data = 0u;

  // initiate build-in self test
  RM3100_Write(RM3100_BIST, 0x87);
  RM3100_Write(RM3100_POLL, 0x70);
  while (!RM3100_IsDataReady())
    ;
  RM3100_Read(RM3100_BIST, &data, 1u);

#ifdef _VERBOSE
  char str[100];
  sprintf(str, "XOK: %u, YOK: %u, ZOK: %u\r\n", BITVAL(data, 4u),
          BITVAL(data, 5u), BITVAL(data, 6u));
  UART_Print(VERBOSE_UART_ID, str);
#endif

  return BITVAL(data, 4u) & BITVAL(data, 5u) & BITVAL(data, 6u);
}

//--------------------------------------------------------------------------------

/**
 * @brief  Read revision identification
 * @param  None
 * @retval Revision ID
 */
uint8_t RM3100_ReadRevID(void) {
  uint8_t data = 0u;
  RM3100_Read(RM3100_REVID, &data, 1u);
  return data;
}

//--------------------------------------------------------------------------------

int32_t RM3100_GetMagnX(void) { return +RM3100_magn[0]; }
int32_t RM3100_GetMagnY(void) { return -RM3100_magn[1]; }
int32_t RM3100_GetMagnZ(void) { return +RM3100_magn[2]; }

float RM3100_GetMagnXf(void) { return (float)+RM3100_magn[0] / 0x800000; }
float RM3100_GetMagnYf(void) { return (float)-RM3100_magn[1] / 0x800000; }
float RM3100_GetMagnZf(void) { return (float)+RM3100_magn[2] / 0x800000; }
/**
 * @}
 */

/**
 * @}
 */