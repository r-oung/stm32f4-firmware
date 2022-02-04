/**
 * @file    m24aa02uid.h
 * @author  Raymond Oung
 * @date    2014.12.10
 * @brief   2K I2C Serial EEPROM with Unique 32-bit Serial Number
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
#ifndef __M24AA02UID_H
#define __M24AA02UID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/** @addtogroup Source
 * @{
 */

/** @addtogroup Utilities
 * @{
 */

/** @addtogroup M24AA02UID
 * @{
 */

/** @defgroup M24AA02UID_Exported_Defines
 * @{
 */
/**
 * @}
 */

/** @defgroup M24AA02UID_Exported_Functions
 * @{
 */
void M24AA02UID_Init(I2C_TypeDef *I2Cx);
uint32_t M24AA02UID_GetUID(void);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __M24AA02UID_H */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
