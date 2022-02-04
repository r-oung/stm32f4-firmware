/**
 * @file    main.h
 * @author  Raymond Oung
 * @date    2013.07.03
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
#define LED1_PIN GPIO_Pin_15

#define LED2_CLK RCC_AHB1Periph_GPIOC
#define LED2_PORT GPIOC
#define LED2_PIN GPIO_Pin_15

#define LED3_CLK RCC_AHB1Periph_GPIOC
#define LED3_PORT GPIOC
#define LED3_PIN GPIO_Pin_14

#define LED4_CLK RCC_AHB1Periph_GPIOC
#define LED4_PORT GPIOC
#define LED4_PIN GPIO_Pin_13

#define PWM1_TIM TIM4
#define PWM1_TIM_CH 2u
#define PWM1_CLK RCC_AHB1Periph_GPIOB
#define PWM1_PORT GPIOB
#define PWM1_PIN GPIO_Pin_7
#define PWM1_PIN_SRC GPIO_PinSource7

#define PWM2_TIM TIM3
#define PWM2_TIM_CH 2u
#define PWM2_CLK RCC_AHB1Periph_GPIOB
#define PWM2_PORT GPIOB
#define PWM2_PIN GPIO_Pin_5
#define PWM2_PIN_SRC GPIO_PinSource5

#define PWM3_TIM TIM3
#define PWM3_TIM_CH 3u
#define PWM3_CLK RCC_AHB1Periph_GPIOB
#define PWM3_PORT GPIOB
#define PWM3_PIN GPIO_Pin_0
#define PWM3_PIN_SRC GPIO_PinSource0

#define PWM4_TIM TIM4
#define PWM4_TIM_CH 1u
#define PWM4_CLK RCC_AHB1Periph_GPIOB
#define PWM4_PORT GPIOB
#define PWM4_PIN GPIO_Pin_6
#define PWM4_PIN_SRC GPIO_PinSource6

#endif /* __MAIN_H */
