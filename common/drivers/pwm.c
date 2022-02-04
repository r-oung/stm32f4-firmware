/**
 * @file    pwm.c
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

/* Includes ------------------------------------------------------------------*/
#include "pwm.h"
#include <math.h>

/** @addtogroup Source
 * @{
 */

/** @addtogroup Low_Level
 * @{
 */

/** @defgroup PWM
 * @brief PWM setup
 * @{
 */

/** @defgroup PWM_Private_Defines
 * @{
 */
#define PWM_PULSEWIDTH_MIN 1000u
#define PWM_NUM_MAX 16u

/** @defgroup PWM_Private_Variables
 * @{
 */
static volatile uint16_t PWM_pulsewidth[PWM_NUM_MAX]; // [usec]
static volatile struct {
  TIM_TypeDef *TIMx;
  uint8_t channel;
  uint16_t pulsewidth_max; // [usec]
  uint32_t TIM_period;     // [cycles]
} PWM_config[PWM_NUM_MAX];
/**
 * @}
 */

/**
 * @brief  Initializes the PWM peripherals
 * @param  TIMx              Timer
 * @param  channel           Timer channel
 * @param  pulsewidth_max    PWM maximum pulsewidth [usec]
 * @param  frequency         PWM frequency [Hz]
 * @param  clk               GPIO clock
 * @param  port              Tx GPIO port
 * @param  pin               Tx GPIO pin
 * @param  pin_src           Tx GPIO pin source
 * @retval None
 */
void PWM_Init(TIM_TypeDef *TIMx, uint8_t channel, uint16_t pulsewidth_max,
              uint16_t frequency, uint32_t clk, GPIO_TypeDef *port,
              uint16_t pin, uint8_t pin_src) {
  static uint8_t PWM_num = 0u;
  if (PWM_num >= PWM_NUM_MAX)
    return;

  RCC_AHB1PeriphClockCmd(clk, ENABLE);

  if (TIMx == TIM3)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  if (TIMx == TIM4)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  if (TIMx == TIM5)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  if (TIMx == TIM8)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

  if (TIMx == TIM3)
    GPIO_PinAFConfig(port, pin_src, GPIO_AF_TIM3);
  if (TIMx == TIM4)
    GPIO_PinAFConfig(port, pin_src, GPIO_AF_TIM4);
  if (TIMx == TIM5)
    GPIO_PinAFConfig(port, pin_src, GPIO_AF_TIM5);
  if (TIMx == TIM8)
    GPIO_PinAFConfig(port, pin_src, GPIO_AF_TIM8);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = pin;
  GPIO_Init(port, &GPIO_InitStructure);

  // configure timer clock
  // @TODO Improve clock precision and use frequency variable
  // this depends on the RCC Peripheral Clock configurations (Peripheral Clock
  // 1)
  float RCC_param =
      2.0f; // for 36 MHz peripheral clock use 1.0f, for 9 MHz use 2.0f
  float TIM_CounterClock = 10000.0f; // desired timer clock [Hz]
  uint16_t prescalerVal =
      ((uint16_t)((SystemCoreClock / RCC_param) / TIM_CounterClock) -
       1u); // prescalar to get the desired timer clock
  PWM_config[PWM_num].TIM_period =
      (uint32_t)(TIM_CounterClock * pulsewidth_max / 1000.0f /
                 1000.0f); // PWM signal period [cycles]
  PWM_config[PWM_num].TIMx = TIMx;
  PWM_config[PWM_num].channel = channel;
  PWM_config[PWM_num].pulsewidth_max = pulsewidth_max;

  // Time Base Configuration
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); // fill with default values

  // TIM2 and TIM5 are 32-bit timers
  // All other TIMx are 16-bit timers
  if (TIMx == TIM2 || TIMx == TIM5) {
    TIM_TimeBaseStructure.TIM_Period = (uint32_t)PWM_config[PWM_num].TIM_period;
    TIM_TimeBaseStructure.TIM_Prescaler = prescalerVal;
  } else {
    TIM_TimeBaseStructure.TIM_Period = (uint16_t)PWM_config[PWM_num].TIM_period;
    TIM_TimeBaseStructure.TIM_Prescaler = prescalerVal;
  }

  TIM_TimeBaseStructure.TIM_ClockDivision = 0u;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0u;
  TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

  // PWM Mode Configuration
  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0u;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  // Initialize Channel
  switch (channel) {
  case 1:
    TIM_OC1Init(TIMx, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);
    break;
  case 2:
    TIM_OC2Init(TIMx, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);
    break;
  case 3:
    TIM_OC3Init(TIMx, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);
    break;
  case 4:
    TIM_OC4Init(TIMx, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable);
    break;
  }

  TIM_ARRPreloadConfig(TIMx, ENABLE);
  TIM_Cmd(TIMx, ENABLE);            // Enable Counter
  TIM_CtrlPWMOutputs(TIMx, ENABLE); // Enable PWM Output

  // zero duty-cycle
  PWM_SetPulseWidth(PWM_num, PWM_PULSEWIDTH_MIN);

  PWM_num++;
}

/**
 * @brief  Set PWM pulse-width
 * @param  pulsewidth Pulse-width [usec]
 * @retval None
 */
void PWM_SetPulseWidth(uint8_t i, uint16_t pulsewidth) {
  // Saturate Pulse-Width
  if (pulsewidth < PWM_PULSEWIDTH_MIN)
    pulsewidth = PWM_PULSEWIDTH_MIN;
  if (pulsewidth > PWM_config[i].pulsewidth_max)
    pulsewidth = PWM_config[i].pulsewidth_max;

  PWM_pulsewidth[i] = pulsewidth;

  uint32_t val = nearbyint((float)PWM_config[i].TIM_period * pulsewidth /
                           PWM_config[i].pulsewidth_max);
  switch (PWM_config[i].channel) {
  case 1u:
    PWM_config[i].TIMx->CCR1 = val;
    break;
  case 2u:
    PWM_config[i].TIMx->CCR2 = val;
    break;
  case 3u:
    PWM_config[i].TIMx->CCR3 = val;
    break;
  case 4u:
    PWM_config[i].TIMx->CCR4 = val;
    break;
  default:
    break;
  }
}

/**
 * @brief  Get pulse-width
 * @retval Pulse-width [usec]
 */
uint16_t PWM_GetPulseWidth(uint8_t i) { return PWM_pulsewidth[i]; }
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
