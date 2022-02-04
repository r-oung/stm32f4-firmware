/**
 * @file    trilateration.h
 * @author  Raymond Oung
 * @date    2014.12.04
 * @brief   Trilateration algorithm
 * @ref     Efficient solution and performance analysis of 3D position
 *          estimation by trilateration by Dmitris E. Manolakis
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
#ifndef __TRILAT_H
#define __TRILAT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup Source
 * @{
 */

/** @addtogroup Low_Level
 * @{
 */

/** @addtogroup TRILAT
 * @{
 */

/** @defgroup TRILAT_Exported_Structures
 * @{
 */
/**
 * @}
 */

/** @defgroup TRILAT_Exported_Functions
 * @{
 */
#define TRILAT_MAX_STATIC_NODES 16u
#define TRILAT_MIN_STATIC_NODES 3u
/**
 * @}
 */

/** @defgroup TRILAT_Exported_Functions
 * @{
 */
int8_t TRILAT_AddStaticNode(float x, float y, float z);

int8_t TRILAT_UpdateStaticNodes(void);
int8_t TRILAT_UpdateMobileNode(float *range);

float TRILAT_GetX(void);
float TRILAT_GetY(void);
float TRILAT_GetZ(void);
uint8_t TRILAT_GetNumNodes(void);
/**
 * @}
 */
#ifdef __cplusplus
}
#endif

#endif /* __TRILAT_H */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */