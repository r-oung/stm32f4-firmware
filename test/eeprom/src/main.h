/**
 * @file    main.h
 * @author  Raymond Oung
 * @date    2013.09.08
 * @brief   Main program body
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
#define LED1_CLK RCC_AHB1Periph_GPIOB
#define LED1_PORT GPIOB
#define LED1_PIN GPIO_Pin_4

#define LED2_CLK RCC_AHB1Periph_GPIOB
#define LED2_PORT GPIOB
#define LED2_PIN GPIO_Pin_3

#define LED3_CLK RCC_AHB1Periph_GPIOC
#define LED3_PORT GPIOC
#define LED3_PIN GPIO_Pin_12

#define LED4_CLK RCC_AHB1Periph_GPIOC
#define LED4_PORT GPIOC
#define LED4_PIN GPIO_Pin_11

#define I2C_EN_CLK RCC_AHB1Periph_GPIOB
#define I2C_EN_PORT GPIOB
#define I2C_EN_PIN GPIO_Pin_5

#define I2C1_SPEED 400000 // [Hz]
#define I2C1_SDA_CLK RCC_AHB1Periph_GPIOB
#define I2C1_SDA_PORT GPIOB
#define I2C1_SDA_PIN GPIO_Pin_7
#define I2C1_SDA_PIN_SOURCE GPIO_PinSource7
#define I2C1_SCL_CLK RCC_AHB1Periph_GPIOB
#define I2C1_SCL_PORT GPIOB
#define I2C1_SCL_PIN GPIO_Pin_6
#define I2C1_SCL_PIN_SOURCE GPIO_PinSource6

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

#define EEPROM_I2C I2C1
#define EEPROM_ADDRESS 0xA0

#endif /* __MAIN_H */
