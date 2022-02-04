/**
 * @file    led.c
 * @author  Raymond Oung
 * @date    2009.11.04
 * @brief   LED functions
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
#include "led.h"
#include "clock.h"

/** @addtogroup Source
 * @{
 */

/** @addtogroup Low_Level
 * @{
 */

/** @defgroup LED
 * @brief This file provides firmware functions to manage LEDs.
 * @{
 */

/** @defgroup LED_Private_Defines
 * @{
 */
#define LED_NUM_MAX 8u
/**
 * @}
 */

/** @defgroup LED_Private_Variables
 * @{
 */
static struct _LED_state_t {
  uint32_t clk;
  GPIO_TypeDef *port;
  uint16_t pin;
  uint32_t t0;
} LED_state[LED_NUM_MAX];
/**
 * @}
 */

/**
 * @brief  Initializes the LEDs
 * @param  clk  Peripheral clock
 * @param  port GPIO port
 * @param  pin  GPIO pin
 * @retval None
 */
void LED_Init(uint32_t clk, GPIO_TypeDef *port, uint16_t pin) {
  static uint8_t LED_num = 0u;

  RCC_AHB1PeriphClockCmd(clk, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(port, &GPIO_InitStructure);

  // initialise state
  LED_state[LED_num].clk = clk;
  LED_state[LED_num].port = port;
  LED_state[LED_num].pin = pin;
  LED_state[LED_num].t0 = 0u;
  LED_Off(LED_num);
  LED_num++;
}

/**
 * @brief  Switches selected LED On
 * @param  i Specifies the LED to be turned on
 * @retval None
 */
void LED_On(uint8_t i) { GPIO_ResetBits(LED_state[i].port, LED_state[i].pin); }

/**
 * @brief  Switches selected LED Off
 * @param  i Specifies the LED to turned off
 * @retval None
 */
void LED_Off(uint8_t i) { GPIO_SetBits(LED_state[i].port, LED_state[i].pin); }

/**
 * @brief  Toggles the selected LED
 * @param  i Specifies the LED to be toggled
 * @retval None
 */
void LED_Toggle(uint8_t i) { LED_state[i].port->ODR ^= LED_state[i].pin; }

/**
 * @brief  Switches all LEDs On
 * @param  None
 * @retval None
 */
void LED_OnAll(void) {
  uint8_t i;
  for (i = 0; i < LED_NUM_MAX; i++) {
    LED_On(i);
  }
}

/**
 * @brief  Switches all LEDs Off
 * @param  None
 * @retval None
 */
void LED_OffAll(void) {
  uint8_t i;
  for (i = 0; i < LED_NUM_MAX; i++) {
    LED_Off(i);
  }
}

/**
 * @brief  Toggles all LEDs
 * @param  None
 * @retval None
 */
void LED_ToggleAll(void) {
  uint8_t i;
  for (i = 0; i < LED_NUM_MAX; i++) {
    LED_Toggle(i);
  }
}

/**
 * @brief  Blink LED at a particular period
 *         Note: This function must be placed in a loop to blink
 * @param  i Specifies the LED to be toggled.
 * @param  Ts Blinking period [msec]
 * @retval None
 */
void LED_Blink(uint8_t i, uint32_t Ts) {
  if (GetTime() - LED_state[i].t0 >= Ts) {
    LED_state[i].t0 = GetTime();
    LED_Toggle(i);
  }
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
