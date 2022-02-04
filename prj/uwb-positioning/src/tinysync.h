/**
 * @file    tinysync.h
 * @author  Raymond Oung
 * @date    2014.09.03
 * @brief   Tiny-sync time synchronisation algorithm
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
#ifndef __TINYSYNC_H
#define __TINYSYNC_H

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

/** @addtogroup TINYSYNC
 * @{
 */

/** @defgroup TINYSYNC_Exported_Structures
 * @{
 */
/**
 * @}
 */

/** @defgroup TINYSYNC_Exported_Functions
 * @{
 */
void TINYSYNC_Init(void);

void TINYSYNC_Update(uint8_t node_id, uint64_t ts0, uint64_t ts1, uint64_t ts2,
                     uint64_t ts3, uint8_t cnt);

double TINYSYNC_GetClockSkew(uint8_t node_id);
double TINYSYNC_GetClockOffset(uint8_t node_id);
/**
 * @}
 */
#ifdef __cplusplus
}
#endif

#endif /* __TINYSYNC_H */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */