/**
 * @file    spi.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H
#define __SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/** @addtogroup Source
 * @{
 */

/** @addtogroup Low_Level
 * @{
 */

/** @addtogroup SPI
 * @{
 */

/** @defgroup SPI_Exported_Defines
 * @{
 */
#define SPI_CPOL0 SPI_CPOL_Low
#define SPI_CPOL1 SPI_CPOL_High
#define SPI_CPHA0 SPI_CPHA_1Edge
#define SPI_CPHA1 SPI_CPHA_2Edge

#define SPI_BAUD_SLOW SPI_BaudRatePrescaler_256
#define SPI_BAUD_FAST SPI_BaudRatePrescaler_2
/**
 * @}
 */

/** @defgroup SPI_Exported_Functions
 * @{
 */
void SPI_PeriphInit(SPI_TypeDef *SPIx, uint16_t baud, uint16_t cpol,
                    uint16_t cpha, uint32_t mosi_clk, GPIO_TypeDef *mosi_port,
                    uint16_t mosi_pin, uint8_t mosi_pin_src, uint32_t miso_clk,
                    GPIO_TypeDef *miso_port, uint16_t miso_pin,
                    uint8_t miso_pin_src, uint32_t sclk_clk,
                    GPIO_TypeDef *sclk_port, uint16_t sclk_pin,
                    uint8_t sclk_pin_src);
void SPI_SetBaudPrescaler(uint8_t p, uint16_t baud);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */