/**
 * @file    ubx.c
 * @author  Raymond Oung
 * @date    2013.11.06
 * @brief   ublox UBX binary protocol (version 14) functions
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
#include "ubx.h"

/** @addtogroup Source
 * @{
 */

/** @addtogroup Utilities
 * @{
 */

/** @defgroup UBX
 * @brief Functions that handle the UBX pressure sensor
 * @{
 */

/** @defgroup UBX_Private_Variables
 * @{
 */
UBX_pkt_t UBX_pkt;

/** @defgroup UART_Private_Functions
 * @{
 */
void UBX_SetHeader(uint8_t *buffer, uint8_t msgClass, uint8_t msgID,
                   uint16_t length) {
  buffer[0] = 0xB5;
  buffer[1] = 0x62;
  buffer[2] = msgClass;
  buffer[3] = msgID;
  buffer[4] = (uint8_t)(length >> 0u);
  buffer[5] = (uint8_t)(length >> 8u);
}

void UBX_SetCheckSum(uint8_t *buffer, uint8_t msgLength) {
  uint8_t ck_a = 0u, ck_b = 0u;
  uint8_t i;

  for (i = 2u; i < (msgLength + UBX_HDR_SIZE); i++) {
    ck_a += buffer[i];
    ck_b += ck_a;
  }

  buffer[msgLength + UBX_HDR_SIZE] = ck_a;
  buffer[msgLength + UBX_HDR_SIZE + 1u] = ck_b;
}

uint8_t UBX_ValidCheckSum(uint8_t *buffer, uint8_t msgLength) {
  if (msgLength > (UBX_MAX_DATA_SIZE + UBX_OVERHEAD))
    return 0;

  uint8_t ck_a = 0u, ck_b = 0u;
  uint8_t i;

  for (i = 2u; i < (msgLength + UBX_HDR_SIZE); i++) {
    ck_a += buffer[i];
    ck_b += ck_a;
  }

  return (buffer[msgLength + UBX_HDR_SIZE] == ck_a &&
          buffer[msgLength + UBX_HDR_SIZE + 1u] == ck_b);
}

uint8_t UBX_ValidCheckSum2(uint8_t *buffer, uint8_t msgClass, uint8_t msgID,
                           uint16_t msgLength) {
  uint8_t ck_a = 0u, ck_b = 0u;
  uint8_t i;

  ck_a += msgClass;
  ck_b += ck_a;

  ck_a += msgID;
  ck_b += ck_a;

  ck_a += (uint8_t)(msgLength >> 0u);
  ck_b += ck_a;

  ck_a += (uint8_t)(msgLength >> 8u);
  ck_b += ck_a;

  for (i = 0; i < msgLength; i++) {
    ck_a += buffer[i];
    ck_b += ck_a;
  }

  return (buffer[msgLength] == ck_a && buffer[msgLength + 1u] == ck_b);
}

/**
 * @brief  Get packet size of msgClass | msgID | msgLength (refer to UBX
 * protocol documentation)
 * @param  msgClass Message Class
 * @param  msgID Message ID
 * @param  msgLength Message Length
 * @retval Returns packet size [bytes]
 */
uint8_t UBX_GetPacketSize(uint8_t msgClass, uint8_t msgID, uint16_t msgLength) {
  uint8_t val = 0u;

  switch (msgClass) {
  case UBX_CLASS_ACK:
    switch (msgID) {
    case UBX_ID_ACK_NAK:
      val = sizeof(UBX_ACK_NAK_t);
      break;
    case UBX_ID_ACK_ACK:
      val = sizeof(UBX_ACK_ACK_t);
      break;
    default:
      val = 0u;
      break;
    }
    break;

  case UBX_CLASS_NAV:
    switch (msgID) {
    case UBX_ID_NAV_DOP:
      val = sizeof(UBX_NAV_DOP_t);
      break;
    case UBX_ID_NAV_POSECEF:
      val = sizeof(UBX_NAV_POSECEF_t);
      break;
    case UBX_ID_NAV_POSLLH:
      val = sizeof(UBX_NAV_POSLLH_t);
      break;
    case UBX_ID_NAV_PVT:
      val = sizeof(UBX_NAV_PVT_t);
      break;
    case UBX_ID_NAV_SOL:
      val = sizeof(UBX_NAV_SOL_t);
      break;
    case UBX_ID_NAV_SVINFO:
      val = sizeof(UBX_NAV_SVINFO_t);
      break;
    case UBX_ID_NAV_VELECEF:
      val = sizeof(UBX_NAV_VELECEF_t);
      break;
    case UBX_ID_NAV_VELNED:
      val = sizeof(UBX_NAV_VELNED_t);
      break;

    default:
      val = 0u;
      break;
    }
    break;

  default:
    val = 0u;
    break;
  }

  return val;
}
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
