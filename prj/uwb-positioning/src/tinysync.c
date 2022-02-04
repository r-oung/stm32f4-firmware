/**
 * @file    tinysync.c
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

/* Includes ------------------------------------------------------------------*/
#include "tinysync.h"
#include <math.h>

// DEBUG
#ifdef _VERBOSE
#include "clock.h"
#include "dw1000.h"
#include "led.h"
#include "main.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>
#endif

/** @addtogroup Source
 * @{
 */

/** @addtogroup Low_Level
 * @{
 */

/** @defgroup TINYSYNC
 * @brief This file provides firmware functions to manage tiny-sync algorithm
 * @{
 */
typedef struct _model_t {
  double a_max;
  double a_min;
  double b_max;
  double b_min;
  uint8_t flag;
} model_t;

/** @defgroup TINYSYNC_Private_Defines
 * @{
 */
#ifdef _VERBOSE
#ifdef _LNS_V03
#define VERBOSE_UART_ID 4u // LNS-v03
#else
#define VERBOSE_UART_ID 2u // DWM1000-v02
#endif

#endif

#define THRESHOLD 0.005f
#define MAX_STATIC_NODES 16u
/**
 * @}
 */

/** @defgroup TINYSYNC_Private_Function_Prototypes
 * @{
 */
static double TINYSYNC_GetSlope(uint64_t Px, uint64_t Py, uint64_t Qx,
                                uint64_t Qy);
static double TINYSYNC_GetOffset(uint64_t Px, uint64_t Py, double a);
static model_t TINYSYNC_EstimateClockModel(uint32_t node_id, uint64_t ts0,
                                           uint64_t ts1, uint64_t ts2,
                                           uint64_t ts3, uint8_t reset);
/**
 * @}
 */

/** @defgroup TINYSYNC_Private_Variables
 * @{
 */
static double TINYSYNC_clockSkew[MAX_STATIC_NODES];
static double TINYSYNC_clockOffset[MAX_STATIC_NODES];

static double TINYSYNC_a_max[MAX_STATIC_NODES]; // max slope
static double TINYSYNC_a_min[MAX_STATIC_NODES]; // min slope
static double TINYSYNC_b_max[MAX_STATIC_NODES]; // max offset
static double TINYSYNC_b_min[MAX_STATIC_NODES]; // min offset
/**
 * @}
 */

/**
 * @brief  Tiny-sync initialisation
 * @param  None
 * @retval None
 */
void TINYSYNC_Init(void) {
  uint8_t i;
  for (i = 0u; i < MAX_STATIC_NODES; i++) {
    TINYSYNC_clockSkew[i] = 1.0f;

    TINYSYNC_a_max[i] = +MAXFLOAT; // max slope
    TINYSYNC_a_min[i] = -MAXFLOAT; // min slope
    TINYSYNC_b_max[i] = +MAXFLOAT; // max offset
    TINYSYNC_b_min[i] = -MAXFLOAT; // min offset
  }
}

/**
 * @brief  Return slope between point-1 and point-2
 * @param  pt1 Point constraint 1
 * @param  pt1 Point constraint 2
 * @retval Slope
 */
static double TINYSYNC_GetSlope(uint64_t Px, uint64_t Py, uint64_t Qx,
                                uint64_t Qy) {
  return (double)(Qy - Py) / (Qx - Px);
}

/**
 * @brief  Return offset for a line with slope 'a' running through point pt
 * @param  pt Point constraint
 * @param  a  Slope of a line
 * @retval Offset
 */
static double TINYSYNC_GetOffset(uint64_t Px, uint64_t Py, double a) {
  return Py - a * Px;
}

/**
 * @brief  Estimate clock model
 *         Ref: Tiny-Sync by S. Yoon
 * @param  node_id Node identifier
 * @param  ts0 Timestamp when mobile-node transmitted the probe [DW1000 time
 * units]
 * @param  ts1 Timestamp when static-node received the probe [DW1000 time units]
 * @param  ts2 Timestamp when static-node sent a response [DW1000 time units]
 * @param  ts3 Timestamp when mobile-node received the response [DW1000 time
 * units]
 * @param  reset 0=Continue using persistent variables, 1=Reset persistent
 * variables
 * @retval Clock model parameters
 */
static model_t TINYSYNC_EstimateClockModel(uint32_t node_id, uint64_t ts0,
                                           uint64_t ts1, uint64_t ts2,
                                           uint64_t ts3, uint8_t reset) {
  static double a_max[MAX_STATIC_NODES]; // max slope
  static double a_min[MAX_STATIC_NODES]; // min slope
  static double b_max[MAX_STATIC_NODES]; // max offset
  static double b_min[MAX_STATIC_NODES]; // min offset

  static uint64_t Ax[MAX_STATIC_NODES]
                    [3]; // saved constraints (data points) comprising t2 and t0
  static uint64_t Ay[MAX_STATIC_NODES]
                    [3]; // saved constraints (data points) comprising t2 and t0
  static uint64_t Bx[MAX_STATIC_NODES]
                    [3]; // saved constraints (data points) comprising t2 and t3
  static uint64_t By[MAX_STATIC_NODES]
                    [3]; // saved constraints (data points) comprising t2 and t3

  static uint8_t cnt[MAX_STATIC_NODES]; // number of constrains (data points)

  if (reset) {
    // reset persistent variables
    uint8_t i;
    for (i = 0u; i < MAX_STATIC_NODES; i++) {
      a_max[i] = 0u;
      a_min[i] = 0u;
      b_max[i] = 0u;
      b_min[i] = 0u;
      cnt[i] = 0u;

      uint8_t j;
      for (j = 0u; j < 3u; j++) {
        Ax[i][j] = 0u;
        Ay[i][j] = 0u;
        Bx[i][j] = 0u;
        By[i][j] = 0u;
      }
    }
  } else {
    // estimate processing time
    uint64_t static_dt = 0u;
    uint64_t tof = 0u;
    // tof = 0.5*((ts3-ts0) - (ts2-ts1))/2; // take 50// of the time of flight
    // static_dt = 0.95f*(ts2 - ts1); // note: this is in the static-node's
    // timeframe can't use 100// because of precision error

    // save constraints
    if (cnt[node_id] < 2u) // phase-1
    {
      Ax[node_id][cnt[node_id]] = ts2 - static_dt - tof;
      Ay[node_id][cnt[node_id]] = ts0;

      Bx[node_id][cnt[node_id]] = ts2 + tof;
      By[node_id][cnt[node_id]] = ts3;

      // increment constraint (data point) counter
      cnt[node_id]++;

      // when there is a sufficient number of constraints, estimate the slope
      if (cnt[node_id] >= 2u) {
        double a;

        a = TINYSYNC_GetSlope(Ax[node_id][0], Ay[node_id][0], Bx[node_id][1],
                              By[node_id][1]);
        if (a > (1.0 - THRESHOLD)) {
          a_max[node_id] = a; // good slope estimate, continue to phase-2
        } else {
          cnt[node_id] = 0u; // poor slope estimate, so reset initial
                             // constraints and redo phase-1
        }

        a = TINYSYNC_GetSlope(Bx[node_id][0], By[node_id][0], Ax[node_id][1],
                              Ay[node_id][1]);
        if (a < (1.0 + THRESHOLD)) {
          a_min[node_id] = a; // good slope estimate, continue to phase-2
        } else {
          cnt[node_id] = 0u; // poor slope estimate, so reset initial
                             // constraints and redo phase-1
        }
      }
    } else // phase-2
    {
      double a;
      double A1x, A1y, A2x, A2y, B1x, B1y, B2x, B2y;

      uint8_t new_max_slope_constraints = 0u; // reset flag
      uint8_t new_min_slope_constraints = 0u; // reset flag

      // get new constraint
      Ax[node_id][2] = ts2 - static_dt - tof;
      Ay[node_id][2] = ts0;

      Bx[node_id][2] = ts2 + tof;
      By[node_id][2] = ts3;

      // find new maximum slope
      // current max slope defined by A1 and B2
      // 1. try A1 and B3
      a = TINYSYNC_GetSlope(Ax[node_id][0], Ay[node_id][0], Bx[node_id][2],
                            By[node_id][2]);
      if (a < a_max[node_id] && a > (1.0 - THRESHOLD)) {
        // keep A1 as is and replace B2 with B3
        A1x = Ax[node_id][0];
        A1y = Ay[node_id][0];
        B2x = Bx[node_id][2];
        B2y = By[node_id][2];
        a_max[node_id] = a;
        new_max_slope_constraints = 1u;
      }

      // 2. try A2 and B3
      a = TINYSYNC_GetSlope(Ax[node_id][1], Ay[node_id][1], Bx[node_id][2],
                            By[node_id][2]);
      if (a < a_max[node_id] && a > (1.0 - THRESHOLD)) {
        // replace A1 with A2 and B2 with B3
        A1x = Ax[node_id][1];
        A1y = Ay[node_id][1];
        B2x = Bx[node_id][2];
        B2y = By[node_id][2];
        a_max[node_id] = a;
        new_max_slope_constraints = 1u;
      }

      // find new minimum slope
      // current min slope defined by B1 and A2
      // 1. try B1 and A3
      a = TINYSYNC_GetSlope(Bx[node_id][0], By[node_id][0], Ax[node_id][2],
                            Ay[node_id][2]);
      if (a > a_min[node_id] && a < (1.0 + THRESHOLD)) {
        // keep B1 as is and replace A2 with A3
        B1x = Bx[node_id][0];
        B1y = By[node_id][0];
        A2x = Ax[node_id][2];
        A2y = Ay[node_id][2];
        a_min[node_id] = a;
        new_min_slope_constraints = 1u;
      }

      // 2. try B2 and A3
      a = TINYSYNC_GetSlope(Bx[node_id][1], By[node_id][1], Ax[node_id][2],
                            Ay[node_id][2]);
      if (a > a_min[node_id] && a < (1.0 + THRESHOLD)) {
        // replace B1 with B2 and A2 with A3
        B1x = Bx[node_id][1];
        B1y = By[node_id][1];
        A2x = Ax[node_id][2];
        A2y = Ay[node_id][2];
        a_min[node_id] = a;
        new_min_slope_constraints = 1u;
      }

      // update constraint set with new constraints
      if (new_max_slope_constraints) {
        Ax[node_id][0] = A1x;
        Ay[node_id][0] = A1y;
        Bx[node_id][1] = B2x;
        By[node_id][1] = B2y;
      }

      if (new_min_slope_constraints) {
        Ax[node_id][1] = A2x;
        Ay[node_id][1] = A2y;
        Bx[node_id][0] = B1x;
        By[node_id][0] = B1y;
      }

      // get best offset
      b_max[node_id] =
          TINYSYNC_GetOffset(Bx[node_id][0], By[node_id][0], a_min[node_id]);
      b_min[node_id] =
          TINYSYNC_GetOffset(Ax[node_id][0], Ay[node_id][0], a_max[node_id]);
    }
  }

  model_t model;
  if (cnt[node_id] >= 2u) {
    model.a_max = a_max[node_id];
    model.a_min = a_min[node_id];
    model.b_max = b_max[node_id];
    model.b_min = b_min[node_id];
    model.flag = 1u;
  } else {
    model.a_max = 1u;
    model.a_min = 1u;
    model.b_max = 0u;
    model.b_min = 0u;
    model.flag = 0u;
  }

  return model;
}

/**
 * @brief  Tiny-sync update
 *         Ref: Tiny-sync by S. Yoon
 * @param  node_id Node identifier
 * @param  ts0 Timestamp when mobile-node transmitted the probe [DW1000 time
 * units]
 * @param  ts1 Timestamp when static-node received the probe [DW1000 time units]
 * @param  ts2 Timestamp when static-node sent a response [DW1000 time units]
 * @param  ts3 Timestamp when mobile-node received the response [DW1000 time
 * units]
 * @param  msg_cnt Message counter
 * @retval None
 */
void TINYSYNC_Update(uint8_t node_id, uint64_t ts0, uint64_t ts1, uint64_t ts2,
                     uint64_t ts3, uint8_t msg_cnt) {
  static uint8_t msg_cnt_old[MAX_STATIC_NODES]; // message counter
  static double ts0_old[MAX_STATIC_NODES];      // old ts0 values
  static double ts2_old[MAX_STATIC_NODES];      // old ts2 values
  static double ts3_old[MAX_STATIC_NODES];      // old ts3 values

  model_t model;

  // use counter to filter out bad messages
  if ((msg_cnt == msg_cnt_old[node_id] + 1u)) {
    // detect the special case, i.e. when one clock starts wrapping around, but
    // the other one hasn't yet
    if (((ts2 < ts2_old[node_id]) && (ts0 > ts0_old[node_id])) ||
        ((ts0 < ts0_old[node_id]) && (ts2 > ts2_old[node_id]))) {
      model = TINYSYNC_EstimateClockModel(node_id, ts0_old[node_id], ts1, ts2,
                                          ts3_old[node_id],
                                          1u); // reset persistent variables
    } else {
      // main time synchronisation algorithm
      model = TINYSYNC_EstimateClockModel(
          node_id, ts0_old[node_id], ts1, ts2, ts3_old[node_id],
          0u); // continue using persistent variables
    }

    // select best slope and offset
    if (model.flag) {
      if (model.a_max < TINYSYNC_a_max[node_id]) {
        TINYSYNC_a_max[node_id] = model.a_max;
        TINYSYNC_b_min[node_id] = model.b_min;
      }

      if (model.a_min > TINYSYNC_a_min[node_id]) {
        TINYSYNC_a_min[node_id] = model.a_min;
        TINYSYNC_b_max[node_id] = model.b_max;
      }

      TINYSYNC_clockSkew[node_id] =
          (TINYSYNC_a_max[node_id] + TINYSYNC_a_min[node_id]) / 2.0;
      TINYSYNC_clockOffset[node_id] =
          (TINYSYNC_b_max[node_id] + TINYSYNC_b_min[node_id]) / 2.0;
    }

    // char str[100]; sprintf(str,"%u %f %f %f %f %f\r\n", (unsigned
    // int)GetTime(), model.a_max, model.a_min, TINYSYNC_a_max[node_id],
    // TINYSYNC_a_min[node_id], TINYSYNC_clockSkew[node_id]);
    // UART_Print(VERBOSE_UART_ID,str);
  }
  // char str[100]; sprintf(str,"%u %f %f %f %f %f %f\r\n", GetTime(),
  // ts0*DW1000_TIME2SEC, ts1*DW1000_TIME2SEC, ts2*DW1000_TIME2SEC,
  // ts3*DW1000_TIME2SEC, model.a_max, model.a_min);
  // UART_Print(VERBOSE_UART_ID,str);

  // save values for the next update
  msg_cnt_old[node_id] = msg_cnt;
  ts0_old[node_id] = ts0;
  ts2_old[node_id] = ts2;
  ts3_old[node_id] = ts3;
}

double TINYSYNC_GetClockSkew(uint8_t node_id) {
  return TINYSYNC_clockSkew[node_id];
}
double TINYSYNC_GetClockOffset(uint8_t node_id) {
  return TINYSYNC_clockOffset[node_id];
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
