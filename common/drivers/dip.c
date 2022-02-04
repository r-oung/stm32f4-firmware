/**
 * @file    dip.c
 * @author  Raymond Oung
 * @date    2014.02.10
 * @brief   DIP switch functions
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
#include "dip.h"
#include <stdlib.h>

/** @addtogroup Source
 * @{
 */

/** @addtogroup Utilities
 * @{
 */

/** @defgroup DIP
 * @brief This is a template.
 * @{
 */

/** @defgroup DIP_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @defgroup DIP_Private_Defines
 * @{
 */
/**
 * @}
 */

/** @defgroup DIP_Private_Macros
 * @{
 */
/**
 * @}
 */

/** @defgroup DIP_Private_Variables
 * @{
 */
static uint8_t DIP_num = 0u;
static GPIO_TypeDef **DIP_port;
static uint32_t *DIP_clk;
static uint16_t *DIP_pin;
static uint8_t DIP_val = 0u;
/**
 * @}
 */

/** @defgroup DIP_Private_Functions
 * @{
 */
void LED_RCC_Configuration(uint32_t clk);
void LED_GPIO_Configuration(GPIO_TypeDef *port, uint16_t pin);
/**
 * @}
 */

/**
 * @brief  Configures the different system clocks.
 * @param  clk Peripheral clock
 * @retval None
 */
void DIP_RCC_Configuration(uint32_t clk) {
  RCC_APB2PeriphClockCmd(clk, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 * @param  port  GPIO port
 * @param  pin   GPIO pin
 * @retval None
 */
void DIP_GPIO_Configuration(GPIO_TypeDef *port, uint16_t pin) {
  GPIO_InitTypeDef GPIO_InitStructure;

  // Configure output to push-pull mode
  GPIO_InitStructure.GPIO_Pin = pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(port, &GPIO_InitStructure);
}

/**
 * @brief  Initializes the LEDs
 * @param  clk  Peripheral clock
 * @param  port GPIO port
 * @param  pin  GPIO pin
 * @retval None
 */
void DIP_Init(uint32_t clk, GPIO_TypeDef *port, uint16_t pin) {
  // extend variable size
  DIP_num++;
  DIP_port = realloc(DIP_port, DIP_num * sizeof(*DIP_port));
  DIP_clk = realloc(DIP_clk, DIP_num * sizeof(*DIP_clk));
  DIP_pin = realloc(DIP_pin, DIP_num * sizeof(*DIP_pin));

  // initialise variable
  DIP_port[DIP_num - 1] = port;
  DIP_clk[DIP_num - 1] = clk;
  DIP_pin[DIP_num - 1] = pin;

  // System clocks configuration
  DIP_RCC_Configuration(clk);

  // GPIO Configuration
  DIP_GPIO_Configuration(port, pin);

  // Read DIP switch
  DIP_val |= GPIO_ReadInputDataBit(DIP_port[DIP_num - 1], DIP_pin[DIP_num - 1])
             << (DIP_num - 1);
}

/**
 * @brief  Get DIP value
 * @param  None
 * @retval None
 */
uint8_t DIP_GetVal(void) { return DIP_val; }

/**
 * @brief  Set DIP value
 * @param  None
 * @retval None
 */
void DIP_SetVal(uint8_t val) { DIP_val = val; }
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
