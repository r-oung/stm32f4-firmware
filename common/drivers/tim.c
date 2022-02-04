/**
 * @file    tim.c
 * @author  Raymond Oung
 * @date    2009.11.01
 * @brief   Timer functions
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
#include "tim.h"

/** @addtogroup Source
 * @{
 */

/** @addtogroup Low_Level
 * @{
 */

/** @defgroup TIM
 * @brief Timer setup; functions as scheduler
 * @{
 */

/** @defgroup TIM_Private_Variables
 * @{
 */
static const uint32_t TIM_CounterClock = 42000000u; // [Hz]
/**
 * @}
 */

/**
 * @brief  Initializes timer interrupt
 * @param  None
 * @retval None
 */
void TIM_Init(TIM_TypeDef *TIMx, float freq, uint8_t premptPriority) {
  if (TIMx == TIM2)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  if (TIMx == TIM3)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  if (TIMx == TIM4)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  if (TIMx == TIM5)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  if (TIMx == TIM2)
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  if (TIMx == TIM3)
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  if (TIMx == TIM4)
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  if (TIMx == TIM5)
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = premptPriority;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); // fill with default values

  // TIM2 and TIM5 are 32-bit timers
  // All other TIMx are 16-bit timers
  if (TIMx == TIM2 || TIMx == TIM5) {
    TIM_TimeBaseStructure.TIM_Period = (uint32_t)(TIM_CounterClock / freq);
    TIM_TimeBaseStructure.TIM_Prescaler = 1u;
  } else {
    // @TODO This is not super precise
    TIM_TimeBaseStructure.TIM_Period =
        (uint16_t)(TIM_CounterClock / freq / 600.0f);
    TIM_TimeBaseStructure.TIM_Prescaler = 1180u;
  }

  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

  // TIM Interrupts enable
  TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);

  // TIMx enable counter
  TIM_Cmd(TIMx, ENABLE);
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
