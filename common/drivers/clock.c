/**
 * @file    clock.c
 * @author  Raymond Oung
 * @date    2014.10.30
 * @brief   Clock functions
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
#include "clock.h"

/** @addtogroup Source
 * @{
 */

/** @addtogroup Low_Level
 * @{
 */

/** @defgroup CLOCK
 * @brief Clock functions
 * @{
 */

/** @defgroup CLOCK_Private_Variables
 * @{
 */
static volatile uint32_t SysTick_TimerVal = 0u;
/**
 * @}
 */

/**
 * @brief  Initializes the system's timer
 * @param  None
 * @retval None
 * NOTE: To change clock source, edit:
 * Makefile -DHSE_VALUE=
 * stm32f4xx_conf.h -- line 149 (PLL_M, PLL_N)
 */
void CLOCK_Init(void) {
  // Setup SysTick Timer for 1 msec interrupts
  SysTick_Config(SystemCoreClock / 1000);

  // Select HCLK as SysTick clock source
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

  // Set SysTick Interrupt Vector Priority
  NVIC_SetPriority(SysTick_IRQn,
                   NVIC_EncodePriority(NVIC_PriorityGroup_4, 0x00, 0x00));
}

/**
 * @brief  Increments SysTick_TimerVal
 * @param  None
 * @retval None
 */
__inline void SysTick_TimerValInc(void) { SysTick_TimerVal++; }

/**
 * @brief  Delay (Blocking)
 * @param  dt Delay time length [msec]
 * @retval None
 */
void Delay(uint32_t dt) {
  uint32_t delay = GetTime() + dt;

  while (GetTime() < delay)
    ;
}

/**
 * @brief  Delay (Blocking)
 * @param  dt Delay time length [msec]
 * @retval None
 */
void DelayU(float dt) {
  float delay = GetTimeU() + dt;

  while (GetTimeU() < delay)
    ;
}

/**
 * @brief  Get current time [msec]
 * @param  None
 * @retval None
 */
uint32_t GetTime(void) { return SysTick_TimerVal; }

/**
 * @brief  Get current time with microsecond resolution [msec]
 * @param  None
 * @retval None
 */
float GetTimeU(void) {
  // return SysTick_TimerVal + (168000 - SysTick->VAL)/168000.0f;
  return SysTick_TimerVal + 5.95238095238095e-6f * (168000 - SysTick->VAL);
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
