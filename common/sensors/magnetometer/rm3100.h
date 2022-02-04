/**
 * @file    rm3100.h
 * @author  Raymond Oung
 * @date    2014.11.04
 * @brief   RM3100 functions
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
#ifndef __RM3100_H
#define __RM3100_H

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

/** @addtogroup RM3100
 * @{
 */

/** @defgroup RM3100_Exported_Functions
 * @{
 */
void RM3100_Init(SPI_TypeDef *SPIx, uint32_t cs_clk, GPIO_TypeDef *cs_port,
                 uint16_t cs_pin);

void RM3100_InitSingleMeasurement(void);
uint8_t RM3100_ISR(void);

uint8_t RM3100_ReadRevID(void);

int32_t RM3100_GetMagnX(void);
int32_t RM3100_GetMagnY(void);
int32_t RM3100_GetMagnZ(void);

float RM3100_GetMagnXf(void);
float RM3100_GetMagnYf(void);
float RM3100_GetMagnZf(void);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __RM3100_H */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */