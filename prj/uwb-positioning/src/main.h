/**
 * @file    main.h
 * @author  Raymond Oung
 * @date    2014.08.11
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
#define _LNS_V03 // old modules

/* Public define -------------------------------------------------------------*/
#ifdef _LNS_V03

#define LED1_CLK RCC_AHB1Periph_GPIOB
#define LED1_PORT GPIOB
#define LED1_PIN GPIO_Pin_12

#define ADDR0_CLK RCC_AHB1Periph_GPIOB
#define ADDR0_PORT GPIOB
#define ADDR0_PIN GPIO_Pin_7

#define ADDR1_CLK RCC_AHB1Periph_GPIOB
#define ADDR1_PORT GPIOB
#define ADDR1_PIN GPIO_Pin_6

#define ADDR2_CLK RCC_AHB1Periph_GPIOB
#define ADDR2_PORT GPIOB
#define ADDR2_PIN GPIO_Pin_5

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

#define CAN1_CLK RCC_APB1Periph_CAN1
#define CAN1_GPIO_CLK RCC_AHB1Periph_GPIOA
#define CAN1_TX_PORT GPIOA
#define CAN1_TX_PIN GPIO_Pin_12
#define CAN1_TX_PIN_SOURCE GPIO_PinSource12
#define CAN1_RX_PORT GPIOA
#define CAN1_RX_PIN GPIO_Pin_11
#define CAN1_RX_PIN_SOURCE GPIO_PinSource11
#define CAN1_FILT_NUM 0

#define UART4_BAUD 115200
#define UART4_DMA_BUF_SIZE 165u
#define UART4_TX_CLK RCC_AHB1Periph_GPIOC
#define UART4_TX_PORT GPIOC
#define UART4_TX_PIN GPIO_Pin_10
#define UART4_TX_PIN_SRC GPIO_PinSource10
#define UART4_RX_CLK RCC_AHB1Periph_GPIOC
#define UART4_RX_PORT GPIOC
#define UART4_RX_PIN GPIO_Pin_11
#define UART4_RX_PIN_SRC GPIO_PinSource11

#define DW1000_SPI SPI1

#define DW1000_EN_CLK RCC_AHB1Periph_GPIOC
#define DW1000_EN_PORT GPIOC
#define DW1000_EN_PIN GPIO_Pin_0

#define DW1000_CS_CLK RCC_AHB1Periph_GPIOA
#define DW1000_CS_PORT GPIOA
#define DW1000_CS_PIN GPIO_Pin_4

#define DW1000_WAKE_CLK RCC_AHB1Periph_GPIOC
#define DW1000_WAKE_PORT GPIOC
#define DW1000_WAKE_PIN GPIO_Pin_1

#define DW1000_NRST_CLK RCC_AHB1Periph_GPIOA
#define DW1000_NRST_PORT GPIOA
#define DW1000_NRST_PIN GPIO_Pin_2

#define DW1000_SYNC_CLK RCC_AHB1Periph_GPIOA
#define DW1000_SYNC_PORT GPIOA
#define DW1000_SYNC_PIN GPIO_Pin_1

#define DW1000_IRQ_CLK RCC_AHB1Periph_GPIOA
#define DW1000_IRQ_PORT GPIOA
#define DW1000_IRQ_PIN GPIO_Pin_3
#define DW1000_IRQ_PORT_SRC EXTI_PortSourceGPIOA
#define DW1000_IRQ_PIN_SRC GPIO_PinSource3
#define DW1000_IRQ_LINE EXTI_Line3
#define DW1000_IRQ_IRQ EXTI3_IRQn

#define DW1000_RXOK_CLK RCC_AHB1Periph_GPIOC
#define DW1000_RXOK_PORT GPIOC
#define DW1000_RXOK_PIN GPIO_Pin_0
#define DW1000_RXOK_PORT_SRC EXTI_PortSourceGPIOC
#define DW1000_RXOK_PIN_SRC GPIO_PinSource0
#define DW1000_RXOK_LINE EXTI_Line0
#define DW1000_RXOK_IRQ EXTI0_IRQn

#define DW1000_SFD_CLK RCC_AHB1Periph_GPIOC
#define DW1000_SFD_PORT GPIOC
#define DW1000_SFD_PIN GPIO_Pin_1
#define DW1000_SFD_PORT_SRC EXTI_PortSourceGPIOC
#define DW1000_SFD_PIN_SRC GPIO_PinSource1
#define DW1000_SFD_LINE EXTI_Line1
#define DW1000_SFD_IRQ EXTI1_IRQn

#define DW1000_RX_CLK RCC_AHB1Periph_GPIOC
#define DW1000_RX_PORT GPIOC
#define DW1000_RX_PIN GPIO_Pin_2
#define DW1000_RX_PORT_SRC EXTI_PortSourceGPIOC
#define DW1000_RX_PIN_SRC GPIO_PinSource2
#define DW1000_RX_LINE EXTI_Line2
#define DW1000_RX_IRQ EXTI2_IRQn

#define DW1000_TX_CLK RCC_AHB1Periph_GPIOC
#define DW1000_TX_PORT GPIOC
#define DW1000_TX_PIN GPIO_Pin_4
#define DW1000_TX_PORT_SRC EXTI_PortSourceGPIOC
#define DW1000_TX_PIN_SRC GPIO_PinSource4
#define DW1000_TX_LINE EXTI_Line4
#define DW1000_TX_IRQ EXTI4_IRQn

#else // _DWM1000_V02

#define LED1_CLK RCC_AHB1Periph_GPIOB
#define LED1_PORT GPIOB
#define LED1_PIN GPIO_Pin_0

#define I2C1_SPEED 100000 // [Hz]
#define I2C1_SDA_CLK RCC_AHB1Periph_GPIOB
#define I2C1_SDA_PORT GPIOB
#define I2C1_SDA_PIN GPIO_Pin_9
#define I2C1_SDA_PIN_SOURCE GPIO_PinSource9
#define I2C1_SCL_CLK RCC_AHB1Periph_GPIOB
#define I2C1_SCL_PORT GPIOB
#define I2C1_SCL_PIN GPIO_Pin_8
#define I2C1_SCL_PIN_SOURCE GPIO_PinSource8

#define SPI2_MOSI_CLK RCC_AHB1Periph_GPIOB
#define SPI2_MOSI_PORT GPIOB
#define SPI2_MOSI_PIN GPIO_Pin_15
#define SPI2_MOSI_PIN_SRC GPIO_PinSource15
#define SPI2_MISO_CLK RCC_AHB1Periph_GPIOB
#define SPI2_MISO_PORT GPIOB
#define SPI2_MISO_PIN GPIO_Pin_14
#define SPI2_MISO_PIN_SRC GPIO_PinSource14
#define SPI2_SCLK_CLK RCC_AHB1Periph_GPIOB
#define SPI2_SCLK_PORT GPIOB
#define SPI2_SCLK_PIN GPIO_Pin_13
#define SPI2_SCLK_PIN_SRC GPIO_PinSource13

#define UART2_BAUD 115200
#define UART2_DMA_BUF_SIZE 165u
#define UART2_TX_CLK RCC_AHB1Periph_GPIOA
#define UART2_TX_PORT GPIOA
#define UART2_TX_PIN GPIO_Pin_2
#define UART2_TX_PIN_SRC GPIO_PinSource2
#define UART2_RX_CLK RCC_AHB1Periph_GPIOA
#define UART2_RX_PORT GPIOA
#define UART2_RX_PIN GPIO_Pin_3
#define UART2_RX_PIN_SRC GPIO_PinSource3

#define DW1000_SPI SPI2

#define DW1000_EN_CLK RCC_AHB1Periph_GPIOB
#define DW1000_EN_PORT GPIOB
#define DW1000_EN_PIN GPIO_Pin_3

#define DW1000_CS_CLK RCC_AHB1Periph_GPIOC
#define DW1000_CS_PORT GPIOC
#define DW1000_CS_PIN GPIO_Pin_6

#define DW1000_WAKE_CLK RCC_AHB1Periph_GPIOB
#define DW1000_WAKE_PORT GPIOB
#define DW1000_WAKE_PIN GPIO_Pin_2

#define DW1000_NRST_CLK RCC_AHB1Periph_GPIOB
#define DW1000_NRST_PORT GPIOB
#define DW1000_NRST_PIN GPIO_Pin_10

#define DW1000_EXTON_CLK RCC_AHB1Periph_GPIOB
#define DW1000_EXTON_PORT GPIOB
#define DW1000_EXTON_PIN GPIO_Pin_1
#define DW1000_EXTON_PORT_SRC EXTI_PortSourceGPIOB
#define DW1000_EXTON_PIN_SRC GPIO_PinSource1
#define DW1000_EXTON_LINE EXTI_Line1
#define DW1000_EXTON_IRQ EXTI1_IRQn

#define DW1000_IRQ_CLK RCC_AHB1Periph_GPIOB
#define DW1000_IRQ_PORT GPIOB
#define DW1000_IRQ_PIN GPIO_Pin_12
#define DW1000_IRQ_PORT_SRC EXTI_PortSourceGPIOB
#define DW1000_IRQ_PIN_SRC GPIO_PinSource12
#define DW1000_IRQ_LINE EXTI_Line12
#define DW1000_IRQ_IRQ EXTI15_10_IRQn

#define DW1000_RXOK_CLK RCC_AHB1Periph_GPIOC
#define DW1000_RXOK_PORT GPIOC
#define DW1000_RXOK_PIN GPIO_Pin_7
#define DW1000_RXOK_PORT_SRC EXTI_PortSourceGPIOC
#define DW1000_RXOK_PIN_SRC GPIO_PinSource7
#define DW1000_RXOK_LINE EXTI_Line7
#define DW1000_RXOK_IRQ EXTI9_5_IRQn

#define DW1000_TX_CLK RCC_AHB1Periph_GPIOB
#define DW1000_TX_PORT GPIOB
#define DW1000_TX_PIN GPIO_Pin_11
#define DW1000_TX_PORT_SRC EXTI_PortSourceGPIOB
#define DW1000_TX_PIN_SRC GPIO_PinSource11
#define DW1000_TX_LINE EXTI_Line11
#define DW1000_TX_IRQ EXTI15_10_IRQn

#define M24AA02UID_I2C I2C1
#define MPU9150_I2C I2C1
#endif

#ifdef __cplusplus
extern "C" {
#endif
// float getElapsedTime(void);
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
