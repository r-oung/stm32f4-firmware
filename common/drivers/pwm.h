/**
 * @file    pwm.h
 * @author  Raymond Oung
 * @date    2009.11.02
 * @brief   PWM interface functions
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
#ifndef __PWM_H
#define __PWM_H

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

/** @addtogroup PWM
 * @{
 */

/** @defgroup PWM_Exported_Functions
 * @{
 */
void PWM_Init(TIM_TypeDef *TIMx, uint8_t channel, uint16_t pulsewidth_max,
              uint16_t frequency, uint32_t clk, GPIO_TypeDef *port,
              uint16_t pin, uint8_t pin_src);

void PWM_SetPulseWidth(uint8_t i, uint16_t pulsewidth_usec);
uint16_t PWM_GetPulseWidth(uint8_t i);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __PWM_H */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */