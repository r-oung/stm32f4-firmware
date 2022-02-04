/**
 * @file    double_buffer.h
 * @author  Raymond Oung
 * @date    2009.10.28
 * @brief   Double buffer
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
#ifndef __DB_H
#define __DB_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @defgroup DB_Exported_Structures
 * @{
 */
typedef struct _DoubleBuffer_t {
  uint16_t halfsize;  // half the number of total elements
  uint8_t writeBufID; // write buffer ID in use
  uint16_t write0;    // index at which to write new element
  uint16_t write1;    // index at which to write new element
  uint8_t *elems0;    // vector of buffer-1 elements
  uint8_t *elems1;    // vector of buffer-2 elements
} DoubleBuffer_t;

/**
 * @}
 */

/** @defgroup DB_Exported_Functions
 * @{
 */
void DB_Init(DoubleBuffer_t *db, uint16_t halfsize);
uint8_t DB_IsFull(DoubleBuffer_t *db, uint8_t i);
uint8_t DB_IsEmpty(DoubleBuffer_t *db);
uint16_t DB_FreeElems(DoubleBuffer_t *db);
void DB_Clear(DoubleBuffer_t *db, uint8_t i);
void DB_Write(DoubleBuffer_t *db, uint8_t *elem);
uint8_t *DB_GetBuf(DoubleBuffer_t *db, uint8_t i);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __DB_H */