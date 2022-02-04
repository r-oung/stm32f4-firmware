/**
 * @file    circular_buffer.h
 * @author  Raymond Oung
 * @date    2009.10.27
 * @brief   Circular buffer
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
#ifndef __CB_H
#define __CB_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @defgroup CB_Exported_Structures
 * @{
 */
typedef struct _CircularBuffer_t {
  uint16_t size;  // maximum number of elements
  uint16_t start; // index of the oldest element
  uint16_t end;   // index at which to write new element
  uint8_t s_msb;  // number of wraps made by 'start'
  uint8_t e_msb;  // number of wraps made by 'end'
  uint8_t *elems; // vector of elements
} CircularBuffer_t;
/**
 * @}
 */

/** @defgroup CB_Exported_Functions
 * @{
 */
void CB_Init(CircularBuffer_t *cb, uint16_t size);
uint8_t CB_IsEmpty(CircularBuffer_t *cb);
uint16_t CB_FreeElems(CircularBuffer_t *cb);
void CB_Write(CircularBuffer_t *cb, uint8_t *elem);
void CB_Read(CircularBuffer_t *cb, uint8_t *elem);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __CB_H */