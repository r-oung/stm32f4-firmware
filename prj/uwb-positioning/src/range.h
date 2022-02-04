/**
 * @file    range.h
 * @author  Raymond Oung
 * @date    2014.09.03
 * @brief   Ranging protocol
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
#ifndef __RANGE_H
#define __RANGE_H

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

/** @addtogroup RANGE
 * @{
 */

/** @defgroup RANGE_Exported_Functions
 * @{
 */
void RANGE_Init(uint32_t id);
int8_t RANGE_AddStaticNode(uint32_t uid);
int8_t RANGE_UpdateStaticNodes(void);
void RANGE_Arbiter(void *buf, uint64_t ts_rx);
void RANGE_SendProbe(uint32_t dst_id, uint32_t tkn_id);

void RANGE_SetToken(void);
void RANGE_ClrToken(void);
uint8_t RANGE_GetToken(void);

void NODE_SetTxReady(void);
void NODE_ClrTxReady(void);
uint8_t NODE_GetTxReady(void);
float NODE_GetRange(uint32_t node_id);
float NODE_GetSNR(uint32_t node_id);
float NODE_GetPwrFP(uint32_t node_id);
float NODE_GetPwrRX(uint32_t node_id);
float NODE_GetClkSkew(uint32_t node_id);
void NODE_WatchdogISR(uint32_t node_id);
/**
 * @}
 */
#ifdef __cplusplus
}
#endif

#endif /* __RANGE_H */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */