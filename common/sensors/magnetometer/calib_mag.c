/**
 * @file    calib_mag.c
 * @author  Raymond Oung
 * @date    2014.07.17
 * @brief   Magnetometer hard-iron calibration functions
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
#include "calib_mag.h"
#include "arm_math.h"
#include <math.h>

#ifdef _VERBOSE
#include "uart.h"
#include <stdio.h>
#include <string.h>
#endif

/** @addtogroup Source
 * @{
 */

/** @addtogroup High_Level
 * @{
 */

/** @defgroup CMAGN
 * @brief This file provides functions for flight control
 * @{
 */

/** @defgroup CMAGN_API_Private_Defines
 * @{
 */
#ifdef _VERBOSE
#define VERBOSE_UART_ID 1u
#endif
/**
 * @}
 */

/** @defgroup CMAGN_Private_Variables
 * @{
 */
static float CMAGN_V[3] = {0.0f, 0.0f, 0.0f};
static uint8_t CMAGN_first = 1u;

static float32_t XTX_f32[16] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static float32_t XTY_f32[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static arm_matrix_instance_f32 XTX; // Matrix X-Transpose-X
static arm_matrix_instance_f32 XTY; // Matrix X-Transpose-Y
/**
 * @}
 */

/**
 * @brief  Hard-Iron Calibration of the magnetometer
 * @ref    http://www.freescale.com/files/sensors/doc/app_note/AN4246.pdf
 * @param  B Magnetometer measurements along sensor's X-axis [uT]
 * @param  B Magnetometer measurements along sensor's Y-axis [uT]
 * @param  B Magnetometer measurements along sensor's Z-axis [uT]
 * @retval None
 */
void CMAGN_Sample(float Bx, float By, float Bz) {
  if (CMAGN_first) {
    CMAGN_first = 0u;
    arm_mat_init_f32(&XTX, 4u, 4u, (float32_t *)XTX_f32);
    arm_mat_init_f32(&XTY, 4u, 1u, (float32_t *)XTY_f32);
  }

  // compute Eqn. 35 of AN4246
  XTX_f32[0] += Bx * Bx;
  XTX_f32[1] += Bx * By;
  XTX_f32[2] += Bx * Bz;
  XTX_f32[3] += Bx;

  XTX_f32[4] += Bx * By;
  XTX_f32[5] += By * By;
  XTX_f32[6] += By * Bz;
  XTX_f32[7] += By;

  XTX_f32[8] += Bx * Bz;
  XTX_f32[9] += By * Bz;
  XTX_f32[10] += Bz * Bz;
  XTX_f32[11] += Bz;

  XTX_f32[12] += Bx;
  XTX_f32[13] += By;
  XTX_f32[14] += Bz;
  XTX_f32[15] += 1.0f;

  // compute Eq. 36 of AN4246
  float Bxyz2 = Bx * Bx + By * By + Bz * Bz;
  XTY_f32[0] += Bx * Bxyz2;
  XTY_f32[1] += By * Bxyz2;
  XTY_f32[2] += Bz * Bxyz2;
  XTY_f32[3] += Bxyz2;
}

/**
 * @brief  Compute hard-iron calibration parameters
 * @ref    http://www.freescale.com/files/sensors/doc/app_note/AN4246.pdf
 * @param  None
 * @retval None
 */
void CMAGN_ComputeParameters(void) {
  arm_status status;
  float32_t beta_f32[4];
  float32_t XTXI_f32[16];

  arm_matrix_instance_f32 beta;
  arm_matrix_instance_f32 XTXI;
  arm_mat_init_f32(&beta, 4u, 1u, (float32_t *)beta_f32);
  arm_mat_init_f32(&XTXI, 4u, 4u, (float32_t *)XTXI_f32);

  // allocate memory
  status = arm_mat_inverse_f32(&XTX, &XTXI);
  status = arm_mat_mult_f32(&XTXI, XTY, beta);

  CMAGN_V[0] = beta_f32[0] / 2.0f;
  CMAGN_V[1] = beta_f32[1] / 2.0f;
  CMAGN_V[2] = beta_f32[2] / 2.0f;

  // geomagnetic field strength
  float B_geo = sqrtf(beta[3] + CMAGN_V[0] * CMAGN_V[0] +
                      CMAGN_V[1] * CMAGN_V[1] + CMAGN_V[2] * CMAGN_V[2]);

#ifdef _VERBOSE
  char str[100];
  sprintf(str, "V: %+f %+f %+f, B: %+f\r\n", CMAGN_V[0], CMAGN_V[1], CMAGN_V[2],
          B_geo);
  UART_WriteTxt(1, str);
#endif

  return;
}

/**
 * @brief Reset calibration
 * @param  None
 * @retval None
 */
void CMAGN_Reset(void) {
  if (CMAGN_first)
    return;

  uint8_t i, j;
  for (i = 0u; i < 4u; i++) {
    XTY[i] = 0u;
    for (j = 0u; j < 4u; j++) {
      XtX[i][j] = 0u;
    }
  }
}

__inline float CMAGN_GetVx(void) { CMAGN_V[0]; }
__inline float CMAGN_GetVy(void) { CMAGN_V[1]; }
__inline float CMAGN_GetVz(void) { CMAGN_V[2]; }
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */