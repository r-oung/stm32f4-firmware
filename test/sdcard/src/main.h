/**
 * @file    main.h
 * @author  Raymond Oung
 * @date    2014.03.19
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
#define LED1_PIN GPIO_Pin_14

#define LED2_CLK RCC_AHB1Periph_GPIOC
#define LED2_PORT GPIOC
#define LED2_PIN GPIO_Pin_13

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

#define SDIO_D0_CLK RCC_AHB1Periph_GPIOC
#define SDIO_D0_PORT GPIOC
#define SDIO_D0_PIN GPIO_Pin_8
#define SDIO_D0_PIN_SRC GPIO_PinSource8
#define SDIO_D1_CLK RCC_AHB1Periph_GPIOC
#define SDIO_D1_PORT GPIOC
#define SDIO_D1_PIN GPIO_Pin_9
#define SDIO_D1_PIN_SRC GPIO_PinSource9
#define SDIO_D2_CLK RCC_AHB1Periph_GPIOC
#define SDIO_D2_PORT GPIOC
#define SDIO_D2_PIN GPIO_Pin_10
#define SDIO_D2_PIN_SRC GPIO_PinSource10
#define SDIO_D3_CLK RCC_AHB1Periph_GPIOC
#define SDIO_D3_PORT GPIOC
#define SDIO_D3_PIN GPIO_Pin_11
#define SDIO_D3_PIN_SRC GPIO_PinSource11
#define SDIO_CMD_CLK RCC_AHB1Periph_GPIOD
#define SDIO_CMD_PORT GPIOD
#define SDIO_CMD_PIN GPIO_Pin_2
#define SDIO_CMD_PIN_SRC GPIO_PinSource2
#define SDIO_CLK_CLK RCC_AHB1Periph_GPIOC
#define SDIO_CLK_PORT GPIOC
#define SDIO_CLK_PIN GPIO_Pin_12
#define SDIO_CLK_PIN_SRC GPIO_PinSource12

#endif /* __MAIN_H */
