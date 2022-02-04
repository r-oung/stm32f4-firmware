/**
 * @file    range.c
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

/**
 * Usage
 *    1. Add at least one static nodes by calling RANGE_AddStaticNode()
 *    2. Call RANGE_UpdateStaticNodes()
 *    3. Call RANGE_SendProbe()
 */

/* Includes ------------------------------------------------------------------*/
#include "range.h"
#include "clock.h"
#include "dw1000.h"
#include "tinysync.h"
#include <math.h>

// DEBUG
#include "uart.h"
#include <stdio.h>
#include <string.h>

/** @addtogroup Source
 * @{
 */

/** @addtogroup Low_Level
 * @{
 */

/** @defgroup RANGE
 * @brief This file provides firmware functions to manage ranging protocol
 * @{
 */
/** @defgroup RANGE_Private_Defines
 * @{
 */
#ifdef _VERBOSE
#ifdef _LNS_V03
#define VERBOSE_UART_ID 4u // LNS-v03
#else
#define VERBOSE_UART_ID 2u // DWM1000-v02
#endif

#endif

#define CONST_UWB_FREQUENCY_MIN 3494400000 // [Hz]
#define CONST_C0 299792458.0               // speed of light in a vacuum [m/s]
#define CONST_N_air 1.000293 // refractive index of air for visible light [m/]s
#define CONST_C CONST_C0 / CONST_N_air // speed of light in air [m/s]
#define CONST_LAMBDA_MAX                                                       \
  CONST_C / CONST_UWB_FREQUENCY_MIN // maximum wavelength [m]

#define MAX_STATIC_NODES 16u                     // [nodes]
#define WATCHDOG_THRESHOLD 16u                   // [counts]
#define FAR_FIELD_THRESHOLD 2 * CONST_LAMBDA_MAX // [m]
/**
 * @}
 */

/** @defgroup RANGE_Private_Structures
 * @{
 */
#define RANGE_PKT_TYPE_PROBE 0xA0
#define RANGE_PKT_TYPE_RESPONSE 0xB1
typedef struct _RANGE_msg_t {
  uint8_t type;    // message type
  uint32_t src_id; // source/sender identifier
  uint32_t dst_id; // destination/receiver identifier
  uint32_t tkn_id; // token identifier
  uint8_t cnt;     // message counter
  uint64_t ts_rx;  // receive timestamp
  uint64_t ts_tx;  // transmit timestamp
} __attribute__((__packed__)) RANGE_msg_t;
/**
 * @}
 */

/** @defgroup RANGE_Private_Variables
 * @{
 */
static struct node_info_t {
  float range;    // distance from mobile node to static node [m]
  float snr;      // signal to noise ratio
  float pwr_fp;   // first path power
  float pwr_rx;   // receiver power
  float clk_skew; // clock skew
  uint8_t los;    // line of sight flag

  uint8_t watchdog;
} node_info[MAX_STATIC_NODES];
/**
 * @}
 */

/** @defgroup RANGE_Private_Variables
 * @{
 */
static volatile uint32_t RANGE_this_node_uid; // this node's unique identifier

static volatile uint32_t RANGE_static_node_id[MAX_STATIC_NODES];
static volatile uint32_t RANGE_static_node_id_new[MAX_STATIC_NODES];
static volatile uint8_t RANGE_static_node_cnt =
    0u; // total number of static nodes
static volatile uint8_t RANGE_static_node_cnt_new =
    0u; // total number of new static nodes

static volatile uint8_t RANGE_msg_cnt[MAX_STATIC_NODES];
static volatile uint64_t RANGE_ts_tx[MAX_STATIC_NODES];

static volatile uint8_t RANGE_tkn_holder = 0u;

/**
 * @}
 */

/** @defgroup RANGE_Private_Function_Prototypes
 * @{
 */
static int8_t RANGE_UID2BID(uint32_t uid);
static void RANGE_SendResponse(RANGE_msg_t *pkt, uint64_t ts);
static void RANGE_Update(uint8_t buf_id, uint64_t ts0, uint64_t ts1,
                         uint64_t ts2, uint64_t ts3, uint8_t msg_cnt);
/**
 * @}
 */

/**
 * @brief  Ranging protocol initialisation
 * @param  id This node's unique identifier
 * @retval None
 */
// @TODO To handle multiple source IDs on static end,
// mobile IDs should be allocated to a limited range 0-255 or even 0-65535
// static IDs should be everything else
void RANGE_Init(uint32_t id) { RANGE_this_node_uid = id; }

/**
 * @brief  Add static node
 * @param  uid This node's unique identifier
 * @retval -1=Failure, 1=Success
 */
int8_t RANGE_AddStaticNode(uint32_t uid) {
  if (RANGE_static_node_cnt_new >= MAX_STATIC_NODES)
    return -1;

  RANGE_static_node_id[RANGE_static_node_cnt_new] = uid;
  RANGE_static_node_cnt_new++;

  return 1;
}

/**
 * @brief  Ranging protocol initialisation
 * @param  None
 * @retval -1=Failure, 1=Success
 */
int8_t RANGE_UpdateStaticNodes(void) {
  // update number of nodes for ranging
  RANGE_static_node_cnt = RANGE_static_node_cnt_new;
  RANGE_static_node_cnt_new = 0u; // empty FILO list

  // if insufficient static nodes, exit
  if (RANGE_static_node_cnt == 0u)
    return -1;

  // reset variables
  uint8_t i;
  for (i = 0u; i < MAX_STATIC_NODES; i++) {
    node_info[i].range = 0.0f;
    node_info[i].snr = 0.0f;
    node_info[i].pwr_fp = 0.0f;
    node_info[i].pwr_rx = 0.0f;
    node_info[i].clk_skew = 0.0f;

    RANGE_msg_cnt[i] = 0u;
    RANGE_ts_tx[i] = 0u;
  }
  TINYSYNC_Init();

  return 1;
}

/**
 * @brief  Map static node unique identifier (UID) to local buffer identifier
 * (BID)
 * @param  uid This node's unique identifier
 * @retval -1=Failure, 0=<Success
 */
static int8_t RANGE_UID2BID(uint32_t uid) {
  uint8_t i;
  for (i = 0u; i < MAX_STATIC_NODES; i++) {
    if (RANGE_static_node_id[i] == uid)
      return i;
  }

  return -1;
}

/**
 * @brief  Arbitrate data packet
 * @param  buf Pointer to data buffer
 * @param  ts_rx Receive timestamp [DW1000 time units]
 * @retval None
 */
// See pg. 42/213 of DW1000 User Manual
// As a rule of thumb, if the difference between RX_POWER and FP_POWER,
// i.e. RX_POWER - FP_POWER, is less than 6 dBm the channel is likely to be LOS,
// while if the difference is greater than 10 dBm the channel is likely to be
// NLOS.
#define MAX_RX_FP_POWER_THRESHOLD 10.0f // [dBm]
#define MIN_RX_FP_POWER_THRESHOLD 0.0f  // [dBm]
#define MIN_RX_SNR 15.0f                // minimum signal noise ratio
void RANGE_Arbiter(void *buf, uint64_t ts_rx) {
  RANGE_msg_t *msg;
  msg = (RANGE_msg_t *)buf;

  // if the token ID matches our ID, then we are the new token holder
  if (msg->tkn_id == RANGE_this_node_uid) {
    RANGE_SetToken();
  }

  // only handle the packet if this is the correct receiver
  float rx_fp_pwr = DW1000_GetRxPower() - DW1000_GetFPPower();
  if (msg->dst_id == RANGE_this_node_uid) {
    switch (msg->type) {
    case RANGE_PKT_TYPE_PROBE: {
      if ((rx_fp_pwr > MIN_RX_FP_POWER_THRESHOLD) &&
          (rx_fp_pwr < MAX_RX_FP_POWER_THRESHOLD) &&
          (DW1000_GetSNR() > MIN_RX_SNR)) {
        RANGE_SendResponse(msg, ts_rx);
      }
      DW1000_ReceiverOn();
    } break;

    case RANGE_PKT_TYPE_RESPONSE: {
      uint8_t buf_id = RANGE_UID2BID(msg->src_id);

      if ((rx_fp_pwr > MIN_RX_FP_POWER_THRESHOLD) &&
          (rx_fp_pwr < MAX_RX_FP_POWER_THRESHOLD) &&
          (DW1000_GetSNR() > MIN_RX_SNR)) {
        TINYSYNC_Update(buf_id, RANGE_ts_tx[buf_id], msg->ts_rx, msg->ts_tx,
                        ts_rx, msg->cnt);
        RANGE_Update(buf_id, RANGE_ts_tx[buf_id], msg->ts_rx, msg->ts_tx, ts_rx,
                     msg->cnt);
      }

      // save node info
      node_info[buf_id].snr = DW1000_GetSNR();
      node_info[buf_id].pwr_fp = DW1000_GetFPPower();
      node_info[buf_id].pwr_rx = DW1000_GetRxPower();
      node_info[buf_id].clk_skew = (float)TINYSYNC_GetClockSkew(buf_id);

      node_info[buf_id].watchdog = 0u; // reset watchdog
      NODE_SetTxReady(); // set a flag indicating that response was received,
                         // i.e. ready to transmit
    } break;

    default:
      break;
    }
  } else {
    DW1000_ReceiverOn();
  }
}

/**
 * @brief  Send a probe
 * @param  dst_id Destination/Receiver identifier
 * @param  tkn_id Token identifier
 * @retval None
 */
void RANGE_SendProbe(uint32_t dst_id, uint32_t tkn_id) {
  uint8_t buf_id = RANGE_UID2BID(dst_id);

  RANGE_msg_t msg;
  msg.type = RANGE_PKT_TYPE_PROBE;
  msg.src_id = RANGE_this_node_uid;
  msg.dst_id = dst_id;
  msg.tkn_id = tkn_id;
  msg.cnt = ++RANGE_msg_cnt[buf_id];
  msg.ts_rx = 0u;
  msg.ts_tx = 0u;
  DW1000_Transmit((uint8_t *)&msg, sizeof(RANGE_msg_t));
  RANGE_ts_tx[buf_id] = DW1000_GetTxTimestamp();

  NODE_ClrTxReady();

  if (tkn_id != RANGE_this_node_uid)
    RANGE_ClrToken();
}

/**
 * @brief  Send response to probe
 * @param  pkt Pointer to range message
 * @param  ts Time of arrival [DW1000 time units]
 * @retval None
 */
static void RANGE_SendResponse(RANGE_msg_t *pkt, uint64_t ts) {
  static uint64_t ts_rx[MAX_STATIC_NODES];
  static uint64_t ts_tx[MAX_STATIC_NODES];

  RANGE_msg_t msg;
  msg.type = RANGE_PKT_TYPE_RESPONSE;
  msg.src_id = RANGE_this_node_uid;
  msg.dst_id = pkt->src_id;
  msg.tkn_id = pkt->tkn_id;
  msg.cnt = pkt->cnt;
  msg.ts_rx = ts_rx[pkt->src_id];
  msg.ts_tx = ts_tx[pkt->src_id];
  DW1000_Transmit((uint8_t *)&msg, sizeof(RANGE_msg_t));

  // save timestamp set
  ts_rx[pkt->src_id] = ts;                      // receive timestamp
  ts_tx[pkt->src_id] = DW1000_GetTxTimestamp(); // transmit timestamp
}

/**
 * @brief  Compute range
 * @param  buf_id Local buffer identifier
 * @param  ts0 Timestamp when mobile-node transmitted the probe [DW1000 time
 * units]
 * @param  ts1 Timestamp when static-node received the probe [DW1000 time units]
 * @param  ts2 Timestamp when static-node sent a response [DW1000 time units]
 * @param  ts3 Timestamp when mobile-node received the response [DW1000 time
 * units]
 * @param  msg_cnt Message counter
 * @retval None
 */
#define RANGE_MIN 0.0f // [m]
// #define RANGE_MAX 100.0f // [m] @TODO Figure out why we get extremly large
// ranges once in a while
static void RANGE_Update(uint8_t buf_id, uint64_t ts0, uint64_t ts1,
                         uint64_t ts2, uint64_t ts3, uint8_t msg_cnt) {
  static uint8_t msg_cnt_old[MAX_STATIC_NODES];
  static uint64_t ts0_old[MAX_STATIC_NODES];
  static uint64_t ts3_old[MAX_STATIC_NODES];

  // if this message ID is one step greater than the last,
  // then the dt0 value is valid, otherwise toss it out
  if (msg_cnt == msg_cnt_old[buf_id] + 1u) {
    uint64_t dt_static;
    uint64_t dt_mobile;

    // adjust for timestamp wrap-around on static node
    if (ts2 >= ts1) {
      dt_static = ts2 - ts1; // [DW1000 timestamp units]
    } else {
      dt_static =
          (DW1000_MAX_TIMESTAMP - ts1) + ts2; // [DW1000 timestamp units]
    }

    // compute time-of-flight
    // adjust for timestamp wrap-around on mobile node
    if (ts3_old[buf_id] >= ts0_old[buf_id]) {
      dt_mobile =
          (ts3_old[buf_id] - ts0_old[buf_id]); // [DW1000 timestamp units]
    } else {
      dt_mobile = (DW1000_MAX_TIMESTAMP - ts0_old[buf_id]) +
                  ts3_old[buf_id]; // [DW1000 timestamp units]
    }

    // compute range
    node_info[buf_id].range = CONST_C * DW1000_TIME2SEC * (dt_mobile >> 1u) -
                              CONST_C * DW1000_TIME2SEC *
                                  TINYSYNC_GetClockSkew(buf_id) *
                                  (dt_static >> 1u); // [m]
    if (node_info[buf_id].range < RANGE_MIN)
      node_info[buf_id].range = 0.0f; // prevent negative range

    // without compensating for skew
    // @DEBUG
    // uint64_t tof = dt_mobile - dt_static; // [DW1000 timestamp units]
    // float range_normal;
    // range_normal = (tof>>1u)*CONST_C*DW1000_TIME2SEC; // [m]
    // if (node_info[buf_id].range > 10.0f)
    // {
    //   char str[100]; sprintf(str,"%u %f %f %f %f %f %f\r\n", (unsigned
    //   int)GetTime(), ts0*DW1000_TIME2SEC, ts1*DW1000_TIME2SEC,
    //   ts2*DW1000_TIME2SEC, ts3*DW1000_TIME2SEC,
    //                                                         range_normal,
    //                                                         node_info[buf_id].range);
    //                                                         UART_Print(VERBOSE_UART_ID,str);
    // }
  }

  // save values for next update
  msg_cnt_old[buf_id] = msg_cnt;
  ts0_old[buf_id] = ts0;
  ts3_old[buf_id] = ts3;
}

void RANGE_SetToken(void) { RANGE_tkn_holder = 1u; }
void RANGE_ClrToken(void) { RANGE_tkn_holder = 0u; }
uint8_t RANGE_GetToken(void) { return RANGE_tkn_holder; }

static volatile uint8_t NODE_tx_ready = 1u;
void NODE_SetTxReady(void) { NODE_tx_ready = 1u; }
void NODE_ClrTxReady(void) { NODE_tx_ready = 0u; }
uint8_t NODE_GetTxReady(void) { return NODE_tx_ready; }
float NODE_GetRange(uint32_t node_id) {
  return node_info[RANGE_UID2BID(node_id)].range;
}
float NODE_GetSNR(uint32_t node_id) {
  return node_info[RANGE_UID2BID(node_id)].snr;
}
float NODE_GetPwrFP(uint32_t node_id) {
  return node_info[RANGE_UID2BID(node_id)].pwr_fp;
}
float NODE_GetPwrRX(uint32_t node_id) {
  return node_info[RANGE_UID2BID(node_id)].pwr_rx;
}
float NODE_GetClkSkew(uint32_t node_id) {
  return node_info[RANGE_UID2BID(node_id)].clk_skew;
}
void NODE_WatchdogISR(uint32_t node_id) {
  uint8_t buf_id = RANGE_UID2BID(node_id);

  if (node_info[buf_id].watchdog++ >= WATCHDOG_THRESHOLD) {
    node_info[buf_id].range =
        0.0f; // distance from mobile node to static node [m]
    node_info[buf_id].snr = 0.0f;      // signal to noise ratio
    node_info[buf_id].pwr_fp = 0.0f;   // first path power
    node_info[buf_id].pwr_rx = 0.0f;   // receiver power
    node_info[buf_id].clk_skew = 0.0f; // clock skew
    node_info[buf_id].watchdog = WATCHDOG_THRESHOLD;
  }
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

/**
 * @}
 */
