/**
 * @file    circular_buffer.c
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

/* Includes ------------------------------------------------------------------*/
#include "circular_buffer.h"
#include <stdlib.h>

/** @defgroup CB_Private_Functions
 * @{
 */
uint8_t CB_IsFull(CircularBuffer_t *cb);
void CB_Inc(CircularBuffer_t *cb, uint16_t *p, uint8_t *msb, uint16_t blkSize);
/**
 * @}
 */

/**
 * @brief  Initialise circular buffer
 * @param  cb Pointer to circular buffer
 * @param  size Size of circular buffer
 * @retval None
 */
void CB_Init(CircularBuffer_t *cb, uint16_t size) {
  cb->size = size;
  cb->start = 0u;
  cb->end = 0u;
  cb->s_msb = 0u;
  cb->e_msb = 0u;
  cb->elems = (uint8_t *)calloc(cb->size, sizeof(uint8_t));
}

/**
 * @brief  Check if circular buffer is full
 * @param  cb Pointer to circular buffer
 * @retval 0 = Not Full, 1 = Full
 */
uint8_t CB_IsFull(CircularBuffer_t *cb) {
  return cb->end == cb->start && cb->e_msb != cb->s_msb;
}

/**
 * @brief  Check if circular buffer is empty
 * @param  cb Pointer to circular buffer
 * @retval 0 = Not Empty, 1 = Empty
 */
uint8_t CB_IsEmpty(CircularBuffer_t *cb) {
  return cb->end == cb->start && cb->e_msb == cb->s_msb;
}

/**
 * @brief  Free elements of the circular buffer
 * @param  cb Pointer to circular buffer
 * @retval Number of free elements
 */
uint16_t CB_FreeElems(CircularBuffer_t *cb) {
  if (cb->e_msb == cb->s_msb) {
    return cb->size - (cb->end - cb->start);
  } else {
    return cb->start - cb->end;
  }
}

/**
 * @brief  Increment internal circular buffer pointers
 * @param  cb Pointer to circular buffer
 * @param  p Pointer to element that will be incremented
 * @param  msb Pointer to overflow bit
 * @param  blkSize Number of elements to increment
 * @retval None
 */
void CB_Inc(CircularBuffer_t *cb, uint16_t *p, uint8_t *msb, uint16_t blkSize) {
  uint16_t tmp;
  tmp = *p;

  *p += blkSize;
  if (*p >= cb->size) {
    *msb ^= 1u;
    *p = blkSize - (cb->size - tmp);
  }
}

/**
 * @brief  Write a single element to the circular buffer
 * @param  cb Pointer to circular buffer
 * @param  elem Pointer to elements that should be writen
 * @retval None
 */
void CB_Write(CircularBuffer_t *cb, uint8_t *elem) {
  cb->elems[cb->end] = *elem;
  if (CB_IsFull(cb)) // full, overwrite moves start pointer
  {
    CB_Inc(cb, &cb->start, &cb->s_msb, 1u);
  }
  CB_Inc(cb, &cb->end, &cb->e_msb, 1u);
}

/**
 * @brief  Read a single element from the circular buffer
 * @param  cb Pointer to circular buffer
 * @param  elem Pointer to a buffer containing elements read
 * @retval None
 */
void CB_Read(CircularBuffer_t *cb, uint8_t *elem) {
  *elem = cb->elems[cb->start];
  CB_Inc(cb, &cb->start, &cb->s_msb, 1u);
}