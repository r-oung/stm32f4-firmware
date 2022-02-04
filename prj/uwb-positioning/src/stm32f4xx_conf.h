/**
 * @file    stm32f4xx_conf.c
 * @author  Raymond Oung
 * @date    2014.08.11
 * @brief   Library configuration file
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
#ifndef __STM32F4xx_CONF_H
#define __STM32F4xx_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Uncomment the line below to enable peripheral header file inclusion */
// #include "stm32f4xx_adc.h"
// #include "stm32f4xx_can.h"
// #include "stm32f4xx_crc.h"
// #include "stm32f4xx_cryp.h"
// #include "stm32f4xx_dac.h"
// #include "stm32f4xx_dbgmcu.h"
// #include "stm32f4xx_dcmi.h"
// #include "stm32f4xx_dma.h"
#include "stm32f4xx_exti.h"
// #include "stm32f4xx_flash.h"
// #include "stm32f4xx_fsmc.h"
// #include "stm32f4xx_hash.h"
#include "stm32f4xx_gpio.h"
// #include "stm32f4xx_i2c.h"
// #include "stm32f4xx_iwdg.h"
// #include "stm32f4xx_pwr.h"
// #include "stm32f4xx_rcc.h"
// #include "stm32f4xx_rng.h"
// #include "stm32f4xx_rtc.h"
// #include "stm32f4xx_sdio.h"
// #include "stm32f4xx_spi.h"
#include "stm32f4xx_syscfg.h"
// #include "stm32f4xx_tim.h"
// #include "stm32f4xx_usart.h"
// #include "stm32f4xx_wwdg.h"
// #include "misc.h" /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* If an external clock source is used, then the value of the following define
   should be set to the value of the external clock source, else, if no external
   clock is used, keep this define commented */
/*#define I2S_EXTERNAL_CLOCK_VAL   12288000 */ /* Value of the external clock in
                                                  Hz */

/* Uncomment the line below to expanse the "assert_param" macro in the
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */

/* Exported macro ------------------------------------------------------------*/
#ifdef USE_FULL_ASSERT

/**
 * @brief  The assert_param macro is used for function's parameters check.
 * @param  expr: If expr is false, it calls assert_failed function
 *   which reports the name of the source file and the source
 *   line number of the call that failed.
 *   If expr is true, it returns no value.
 * @retval None
 */
#define assert_param(expr)                                                     \
  ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F4xx_CONF_H */
