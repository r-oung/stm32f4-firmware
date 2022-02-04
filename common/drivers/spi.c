/**
 * @file    spi.c
 * @author  Raymond Oung
 * @date    2009.07.25
 * @brief   SPI interface functions
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
#include "spi.h"

/** @addtogroup Source
 * @{
 */

/** @addtogroup Low_Level
 * @{
 */

/** @defgroup SPI
 * @brief SPI setup
 * @{
 */

/** @defgroup SPI_Private_Function_Prototypes
 * @{
 */
static uint8_t SPI_type2buf(SPI_TypeDef *SPIx);
static uint8_t SPI_id2buf(uint8_t id);
/**
 * @}
 */

/** @defgroup SPI_Private_Variables
 * @{
 */
static struct _SPI_config_t {
  SPI_TypeDef *SPIx;
  SPI_InitTypeDef SPI_InitStructure;
} SPI_config[2];
/**
 * @}
 */

/**
 * @brief  Initializes the SPI peripheral
 * @param  None
 * @retval None
 */
void SPI_PeriphInit(SPI_TypeDef *SPIx, uint16_t baud, uint16_t cpol,
                    uint16_t cpha, uint32_t mosi_clk, GPIO_TypeDef *mosi_port,
                    uint16_t mosi_pin, uint8_t mosi_pin_src, uint32_t miso_clk,
                    GPIO_TypeDef *miso_port, uint16_t miso_pin,
                    uint8_t miso_pin_src, uint32_t sclk_clk,
                    GPIO_TypeDef *sclk_port, uint16_t sclk_pin,
                    uint8_t sclk_pin_src) {
  uint8_t n = SPI_type2buf(SPIx);

  RCC_AHB1PeriphClockCmd(mosi_clk | miso_clk | sclk_clk, ENABLE);
  if (SPIx == SPI1)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  if (SPIx == SPI2)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  if (SPIx == SPI1) {
    GPIO_PinAFConfig(mosi_port, mosi_pin_src, GPIO_AF_SPI1);
    GPIO_PinAFConfig(miso_port, miso_pin_src, GPIO_AF_SPI1);
    GPIO_PinAFConfig(sclk_port, sclk_pin_src, GPIO_AF_SPI1);
  }

  if (SPIx == SPI2) {
    GPIO_PinAFConfig(mosi_port, mosi_pin_src, GPIO_AF_SPI2);
    GPIO_PinAFConfig(miso_port, miso_pin_src, GPIO_AF_SPI2);
    GPIO_PinAFConfig(sclk_port, sclk_pin_src, GPIO_AF_SPI2);
  }

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

  GPIO_InitStructure.GPIO_Pin = sclk_pin;
  GPIO_Init(sclk_port, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = miso_pin;
  GPIO_Init(miso_port, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = mosi_pin;
  GPIO_Init(mosi_port, &GPIO_InitStructure);

  SPI_InitTypeDef SPI_InitStructure;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = cpol;
  SPI_InitStructure.SPI_CPHA = cpha;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = baud;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7u;
  SPI_Init(SPIx, &SPI_InitStructure);

  SPI_Cmd(SPIx, ENABLE);

  SPI_config[n].SPIx = SPIx;
  SPI_config[n].SPI_InitStructure = SPI_InitStructure;
}

/**
 * @brief  SPI type to buffer index
 * @param  SPIx SPI type [SPI1, SPI2]
 * @retval Buffer index [0,1]
 */
static uint8_t SPI_type2buf(SPI_TypeDef *SPIx) {
  uint8_t b = 0u;

  if (SPIx == SPI1)
    b = 0u;
  if (SPIx == SPI2)
    b = 1u;

  return b;
}

/**
 * @brief  SPI identifier to buffer index
 * @param  id SPI identifier [1,2]
 * @retval Buffer index [0,1]
 */
static uint8_t SPI_id2buf(uint8_t id) {
  uint8_t b = 0u;

  switch (id) {
  case 1:
    b = 0u;
    break;
  case 2:
    b = 1u;
    break;
  default:
    break;
  }

  return b;
}

/**
 * @brief  Adjust SPI baud prescaler value
 * @param  p Peripheral number
 * @param  baud Baud rate prescalar
 * @retval None
 */
void SPI_SetBaudPrescaler(uint8_t p, uint16_t baud) {
  uint8_t n = SPI_id2buf(p);

  SPI_Cmd(SPI_config[n].SPIx, DISABLE);
  SPI_config[n].SPI_InitStructure.SPI_BaudRatePrescaler = baud;
  SPI_Init(SPI_config[n].SPIx, &SPI_config[n].SPI_InitStructure);
  SPI_Cmd(SPI_config[n].SPIx, ENABLE);
}
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */