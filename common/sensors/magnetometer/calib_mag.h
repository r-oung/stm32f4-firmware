/**
 * @file    calib_mag.h
 * @author  Raymond Oung
 * @date    2014.07.17
 * @brief   Magnetometer hard-iron calibration functions
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
#ifndef __CMAGN_H
#define __CMAGN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
/** @addtogroup Source
 * @{
 */

/** @addtogroup High_Level
 * @{
 */

/** @addtogroup CMAGN
 * @{
 */

/** @defgroup CMAGN_Exported_Functions
 * @{
 */
void CMAGN_Sample(float Bx, float By, float Bz);
void CMAGN_ComputeParameters(void);
void CMAGN_Reset(void);

__inline float CMAGN_GetVx(void) { return CMAGN_V[0]; }
__inline float CMAGN_GetVy(void) { return CMAGN_V[1]; }
__inline float CMAGN_GetVz(void) { return CMAGN_V[2]; }
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __CMAGN_H */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */