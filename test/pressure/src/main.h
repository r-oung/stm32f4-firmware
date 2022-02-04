/**
 * @file    main.h
 * @author  Raymond Oung
 * @date    2014.07.27
 * @brief   Main program
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
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/** @addtogroup Main
 * @{
 */

/* Public typedef ------------------------------------------------------------*/
/* Public define -------------------------------------------------------------*/
#define LED1_CLK RCC_AHB1Periph_GPIOC
#define LED1_PORT GPIOC
#define LED1_PIN GPIO_Pin_13

#define I2C_EN_CLK RCC_AHB1Periph_GPIOB
#define I2C_EN_PORT GPIOB
#define I2C_EN_PIN GPIO_Pin_15

#define I2C2_SPEED 400000 // [Hz]
#define I2C2_SDA_CLK RCC_AHB1Periph_GPIOB
#define I2C2_SCL_CLK RCC_AHB1Periph_GPIOB
#define I2C2_PCLK RCC_APB1Periph_I2C2
#define I2C2_SDA_PORT GPIOB
#define I2C2_SDA_PIN GPIO_Pin_11
#define I2C2_SDA_PIN_SOURCE GPIO_PinSource11
#define I2C2_SCL_PORT GPIOB
#define I2C2_SCL_PIN GPIO_Pin_10
#define I2C2_SCL_PIN_SOURCE GPIO_PinSource10
#define I2C2_AF GPIO_AF_I2C2
#define I2C2_IRQ I2C2_EV_IRQn

#define UART1_BAUD 115200
#define UART1_DMA_BUF_SIZE 0xFF
#define UART1_TX_CLK RCC_AHB1Periph_GPIOA
#define UART1_TX_PORT GPIOA
#define UART1_TX_PIN GPIO_Pin_9
#define UART1_TX_PIN_SRC GPIO_PinSource9
#define UART1_RX_CLK RCC_AHB1Periph_GPIOA
#define UART1_RX_PORT GPIOA
#define UART1_RX_PIN GPIO_Pin_10
#define UART1_RX_PIN_SRC GPIO_PinSource10

// STATIC PRESSURE SENSOR (MS5611)
#define MS5611_I2C                  I2C2
// #define MS5611_PRES_OSR             0x40 // OSR 256
// #define MS5611_PRES_OSR             0x42 // OSR 512
// #define MS5611_PRES_OSR             0x44 // OSR 1024
// #define MS5611_PRES_OSR             0x46 // OSR 2048
#define MS5611_PRES_OSR             0x48 // OSR 4096
// #define MS5611_TEMP_OSR             0x50 // OSR 256
// #define MS5611_TEMP_OSR             0x52 // OSR 512
// #define MS5611_TEMP_OSR             0x54 // OSR 1024
// #define MS5611_TEMP_OSR             0x56 // OSR 2048
#define MS5611_TEMP_OSR             0x58 // OSR 4096

#endif /* __MAIN_H */
