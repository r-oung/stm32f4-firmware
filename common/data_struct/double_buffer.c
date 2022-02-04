/**
 * @file    double_buffer.c
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

/* Includes ------------------------------------------------------------------*/
#include "double_buffer.h"
#include <stdlib.h>

/**
 * @brief  Initialise double buffer
 * @param  db Pointer to double buffer
 * @param  halfsize Half-size of double buffer [0..255]
 * @retval None
 */
void DB_Init(DoubleBuffer_t *db, uint16_t halfsize) {
  db->halfsize = halfsize;
  db->writeBufID = 0u;
  db->write0 = 0u;
  db->write1 = 0u;
  db->elems0 = (uint8_t *)calloc(db->halfsize, sizeof(uint8_t));
  db->elems1 = (uint8_t *)calloc(db->halfsize, sizeof(uint8_t));
}

/**
 * @brief  Check if double buffer is full
 * @param  db Pointer to double buffer
 * @param  0 = First Buffer, 1 = Second Buffer
 * @retval 0 = Not Full, 1 = Full
 */
uint8_t DB_IsFull(DoubleBuffer_t *db, uint8_t i) {
  switch (i) {
  case 0:
    return db->write0 == db->halfsize;
    break;

  case 1:
    return db->write1 == db->halfsize;
    break;

  default:
    return 0u;
    break;
  }
}

/**
 * @brief  Check if double buffer is empty
 * @param  db Pointer to double buffer
 * @retval 0 = Not Empty, 1 = Empty
 */
uint8_t DB_IsEmpty(DoubleBuffer_t *db) {
  return db->write0 == 0u && db->write1 == 0u;
}

/**
 * @brief  Free elements of the double buffer
 * @param  db Pointer to double buffer
 * @retval Number of free elements
 */
uint16_t DB_FreeElems(DoubleBuffer_t *db) {
  return (db->halfsize - db->write0) + (db->halfsize - db->write1);
}

/**
 * @brief  Clear double buffer
 * @param  db Pointer to double buffer
 * @param  0 = First Buffer, 1 = Second Buffer
 * @retval None
 */
void DB_Clear(DoubleBuffer_t *db, uint8_t i) {
  switch (i) {
  case 0:
    db->write0 = 0u;
    break;
  case 1:
    db->write1 = 0u;
    break;
  default:
    break;
  }
}

/**
 * @brief  Write a single element to the double buffer
 * @param  db Pointer to double buffer
 * @param  elem Pointer to elements that should be writen
 * @retval None
 */
void DB_Write(DoubleBuffer_t *db, uint8_t *elem) {
  switch (db->writeBufID) {
  case 0:
    db->elems0[db->write0++] = *elem;
    if (db->write0 >= db->halfsize) {
      db->writeBufID = 1u; // switch to next buffer
      db->write1 = 0u;     // reset index of the next buffer
    }
    break;

  case 1:
    db->elems1[db->write1++] = *elem;
    if (db->write1 >= db->halfsize) {
      db->writeBufID = 0u; // switch to next buffer
      db->write0 = 0u;     // reset index of the next buffer
    }
    break;

  default:
    break;
  }
}

/**
 * @brief  Read a single element from the double buffer
 * @param  db Pointer to double buffer
 * @param  0 = First Buffer, 1 = Second Buffer
 * @retval None
 */
uint8_t *DB_GetBuf(DoubleBuffer_t *db, uint8_t i) {
  switch (i) {
  case 0:
    return db->elems0;
    break;
  case 1:
    return db->elems1;
    break;
  default:
    break;
  }

  return NULL;
}