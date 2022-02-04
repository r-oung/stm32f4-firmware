/**
 * @file    24xx64.h
 * @author  Raymond Oung
 * @date    2013.09.08
 * @brief   Functions that handle the 24AA64/24LC64/24FC64 EEPROM
 *          Memory is organized as a single block of 8K x 8-bit memory
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
#ifndef __EEPROM_H
#define __EEPROM_H

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

/** @addtogroup EEPROM
 * @{
 */

/** @defgroup EEPROM_Exported_Functions
 * @{
 */
void EEPROM_Init(I2C_TypeDef *I2Cx, uint8_t addr);
uint8_t EEPROM_Read(uint16_t addr, uint8_t *data, uint16_t size);
uint8_t EEPROM_Write(uint16_t addr, uint8_t *data, uint16_t size);
uint8_t EEPROM_WriteIT(uint16_t addr, uint8_t *data, uint16_t size);
uint8_t EEPROM_WriteIT_ISR(void);

#ifdef _VERBOSE
void EEPROM_Print(uint16_t s_addr, uint16_t e_addr);
#endif
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_H */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */