/**
 * @file    main.h
 * @author  Raymond Oung
 * @date    2014.05.13
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

#define SPI1_MOSI_CLK RCC_AHB1Periph_GPIOA
#define SPI1_MOSI_PORT GPIOA
#define SPI1_MOSI_PIN GPIO_Pin_7
#define SPI1_MOSI_PIN_SRC GPIO_PinSource7
#define SPI1_MISO_CLK RCC_AHB1Periph_GPIOA
#define SPI1_MISO_PORT GPIOA
#define SPI1_MISO_PIN GPIO_Pin_6
#define SPI1_MISO_PIN_SRC GPIO_PinSource6
#define SPI1_SCLK_CLK RCC_AHB1Periph_GPIOA
#define SPI1_SCLK_PORT GPIOA
#define SPI1_SCLK_PIN GPIO_Pin_5
#define SPI1_SCLK_PIN_SRC GPIO_PinSource5

#define I2C2_SPEED 400000 // [Hz]
#define I2C2_SDA_CLK RCC_AHB1Periph_GPIOB
#define I2C2_SDA_PORT GPIOB
#define I2C2_SDA_PIN GPIO_Pin_11
#define I2C2_SDA_PIN_SOURCE GPIO_PinSource11
#define I2C2_SCL_CLK RCC_AHB1Periph_GPIOB
#define I2C2_SCL_PORT GPIOB
#define I2C2_SCL_PIN GPIO_Pin_10
#define I2C2_SCL_PIN_SOURCE GPIO_PinSource10

#define CAN1_CLK RCC_APB1Periph_CAN1
#define CAN1_GPIO_CLK RCC_AHB1Periph_GPIOB
#define CAN1_TX_PORT GPIOB
#define CAN1_TX_PIN GPIO_Pin_9
#define CAN1_TX_PIN_SOURCE GPIO_PinSource9
#define CAN1_RX_PORT GPIOB
#define CAN1_RX_PIN GPIO_Pin_8
#define CAN1_RX_PIN_SOURCE GPIO_PinSource8
#define CAN1_FILT_NUM 0

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

#define RM3100_SPI SPI1
#define RM3100_CLK RCC_AHB1Periph_GPIOA
#define RM3100_PORT GPIOA
#define RM3100_PIN GPIO_Pin_2

#endif /* __MAIN_H */
