/**
 * @file    main.h
 * @author  Raymond Oung
 * @date    2013.08.16
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
#define LED1_CLK RCC_AHB1Periph_GPIOB
#define LED1_PORT GPIOB
#define LED1_PIN GPIO_Pin_4

#define LED2_CLK RCC_AHB1Periph_GPIOB
#define LED2_PORT GPIOB
#define LED2_PIN GPIO_Pin_3

#define LED3_CLK RCC_AHB1Periph_GPIOB
#define LED3_PORT GPIOB
#define LED3_PIN GPIO_Pin_1

#define LED4_CLK RCC_AHB1Periph_GPIOB
#define LED4_PORT GPIOB
#define LED4_PIN GPIO_Pin_0

#define I2C2_SPEED 400000 // [Hz]
#define I2C2_SDA_CLK RCC_AHB1Periph_GPIOB
#define I2C2_SDA_PORT GPIOB
#define I2C2_SDA_PIN GPIO_Pin_11
#define I2C2_SDA_PIN_SOURCE GPIO_PinSource11
#define I2C2_SCL_CLK RCC_AHB1Periph_GPIOB
#define I2C2_SCL_PORT GPIOB
#define I2C2_SCL_PIN GPIO_Pin_10
#define I2C2_SCL_PIN_SOURCE GPIO_PinSource10

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

#define SENSORS_I2C I2C2

// #define MPU9250_CONFIG_VAL          0x00 // Gyro Filter: 250 Hz Bandwidth, 8 kHz Sampling Rate
// #define MPU9250_CONFIG_VAL          0x01 // Gyro Filter: 184 Hz Bandwidth, 1 kHz Sampling Rate
#define MPU9250_CONFIG_VAL          0x02 // Gyro Filter: 92 Hz Bandwidth, 1 kHz Sampling Rate
// #define MPU9250_CONFIG_VAL          0x03 // Gyro Filter: 41 Hz Bandwidth, 1 kHz Sampling Rate
// #define MPU9250_CONFIG_VAL          0x04 // Gyro Filter: 20 Hz Bandwidth, 1 kHz Sampling Rate
// #define MPU9250_CONFIG_VAL          0x05 // Gyro Filter: 10 Hz Bandwidth, 1 kHz Sampling Rate
// #define MPU9250_CONFIG_VAL          0x06 // Gyro Filter: 5 Hz Bandwidth, 1 kHz Sampling Rate
// #define MPU9250_CONFIG_VAL          0x07 // Gyro Filter: 3600 Hz Bandwidth, 8 kHz Sampling Rate

// #define MPU9250_GYRO_CONFIG_VAL     0x00 // Gyro Full Scale: ±250 deg/s
#define MPU9250_GYRO_CONFIG_VAL     0x08 // Gyro Full Scale: ±500 deg/s
// #define MPU9250_GYRO_CONFIG_VAL     0x10 // Gyro Full Scale: ±1000 deg/s
// #define MPU9250_GYRO_CONFIG_VAL     0x18 // Gyro Full Scale: ±2000 deg/s

// #define MPU9250_ACCL_CONFIG_VAL     0x00 // Accelerometer Full Scale: ±2 g
// #define MPU9250_ACCL_CONFIG_VAL     0x08 // Accelerometer Full Scale: ±4 g
#define MPU9250_ACCL_CONFIG_VAL     0x10 // Accelerometer Full Scale: ±8 g
// #define MPU9250_ACCL_CONFIG_VAL     0x18 // Accelerometer Full Scale: ±16 g

// #define MPU9250_ACCL_CONFIG2_VAL    0x00 // Accelerometer Filter: 460 Hz Bandwidth, 1 kHz Sampling Rate
// #define MPU9250_ACCL_CONFIG2_VAL    0x01 // Accelerometer Filter: 184 Hz Bandwidth, 1 kHz Sampling Rate
#define MPU9250_ACCL_CONFIG2_VAL    0x02 // Accelerometer Filter: 92 Hz Bandwidth, 1 kHz Sampling Rate
// #define MPU9250_ACCL_CONFIG2_VAL    0x03 // Accelerometer Filter: 41 Hz Bandwidth, 1 kHz Sampling Rate
// #define MPU9250_ACCL_CONFIG2_VAL    0x04 // Accelerometer Filter: 20 Hz Bandwidth, 1 kHz Sampling Rate
// #define MPU9250_ACCL_CONFIG2_VAL    0x05 // Accelerometer Filter: 10 Hz Bandwidth, 1 kHz Sampling Rate
// #define MPU9250_ACCL_CONFIG2_VAL    0x06 // Accelerometer Filter: 5 Hz Bandwidth, 1 kHz Sampling Rate
// #define MPU9250_ACCL_CONFIG2_VAL    0x07 // Accelerometer Filter: 460 Hz Bandwidth, 1 kHz Sampling Rate

#endif /* __MAIN_H */
