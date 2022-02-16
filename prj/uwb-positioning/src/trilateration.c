/**
 * @file    trilateration.c
 * @author  Raymond Oung
 * @date    2014.12.04
 * @brief   Trilateration algorithm
 * @ref     Efficient solution and performance analysis of 3D position
 *          estimation by trilateration by Dmitris E. Manolakis
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
 *    1. Add at least 3 static nodes by calling TRILAT_AddStaticNode()
 *    2. Call TRILAT_UpdateStaticNodes()
 *    3. Call TRILAT_UpdateMobileNode()
 *
 * Note
 *    - If using only 3 nodes, 2D position (x,y) is computed
 *    - If using >3 nodes, 3D position (x,y,z) is computed
 */

/* Includes ------------------------------------------------------------------*/
#include "trilateration.h"
#include "arm_math.h"
#include "linalgebra.h" // using this because arm_math.h sucks ass
#include <math.h>

// DEBUG
#ifdef _VERBOSE
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

/** @defgroup TRILAT_Private_Defines
 * @{
 */
#ifdef _VERBOSE
#define VERBOSE_UART_ID 4u
#endif

#define NEWTON_RAPHSON_ITERATIONS 0u
#define NEWTON_RAPHSON_DELTA 0.001f
/**
 * @}
 */

/** @defgroup TRILAT_Private_Types
 * @{
 */
typedef struct _pos3_t {
  float x; // [m]
  float y; // [m]
  float z; // [m]
} pos3_t;
/**
 * @}
 */

/** @defgroup TRILAT_Private_Variables
 * @{
 */
static pos3_t node[TRILAT_MAX_STATIC_NODES];     // static nodes
static pos3_t node_new[TRILAT_MAX_STATIC_NODES]; // new static nodes
static pos3_t position;                          // 3D position

static volatile uint8_t node_cnt = 0u;     // total number of static nodes
static volatile uint8_t node_cnt_new = 0u; // total number of new static nodes

// arm-math pointers
static arm_matrix_instance_f32 A;
static arm_matrix_instance_f32 Ainv;
static arm_matrix_instance_f32 AT;
static arm_matrix_instance_f32 AT_A;
static arm_matrix_instance_f32 AT_A_inv;
static arm_matrix_instance_f32 b;
static arm_matrix_instance_f32 x;
static arm_matrix_instance_f32 JTJinv;
static arm_matrix_instance_f32 JTJinv_JTf;

// arm-math buffers
static float32_t A_f32[(TRILAT_MAX_STATIC_NODES - 1u) *
                       3u]; // [(TRILAT_MAX_STATIC_NODES - 1) x 3]
static float32_t Ainv_f32[3u * (TRILAT_MAX_STATIC_NODES -
                                1u)]; // [3 x (TRILAT_MAX_STATIC_NODES - 1)]
static float32_t AT_f32[3u * (TRILAT_MAX_STATIC_NODES -
                              1u)];     // [3 x (TRILAT_MAX_STATIC_NODES - 1)]
static float32_t AT_A_f32[3u * 3u];     // [3 x 3]
static float32_t AT_A_inv_f32[3u * 3u]; // [3 x 3]
static float32_t
    b_f32[TRILAT_MAX_STATIC_NODES - 1u]; // [(TRILAT_MAX_STATIC_NODES - 1) x 1]
static float32_t x_f32[3];               // [3 x 1]
static float32_t di1_sqrd[TRILAT_MAX_STATIC_NODES -
                          1u];      // [(TRILAT_MAX_STATIC_NODES - 1) x 1]
static float32_t JTJinv_f32[9];     // [3 x 3]
static float32_t JTJinv_JTf_f32[3]; // [3 x 1]

/**
 * @}
 */

/** @defgroup TRILAT_Private_Function_Prototypes
 * @{
 */
static void TRILAT_AllocateMemory(uint8_t num_nodes);
static void TRILAT_PrintARMStatus(arm_status status);
/**
 * @}
 */

/**
 * @brief  Initialise memory
 * @param  None
 * @retval None
 */
void TRILAT_AllocateMemory(uint8_t num_nodes) {
  if (num_nodes > TRILAT_MIN_STATIC_NODES) {
    arm_mat_init_f32(&A, num_nodes - 1u, 3u,
                     A_f32); // [(TRILAT_MAX_STATIC_NODES - 1) x 3]
    arm_mat_init_f32(&Ainv, 3u, num_nodes - 1u,
                     Ainv_f32); // [3 x (TRILAT_MAX_STATIC_NODES - 1)]
    arm_mat_init_f32(&AT, 3u, num_nodes - 1u,
                     AT_f32); // [3 x (TRILAT_MAX_STATIC_NODES - 1)]
    arm_mat_init_f32(&AT_A, 3u, 3u, AT_A_f32);         // [3 x 3]
    arm_mat_init_f32(&AT_A_inv, 3u, 3u, AT_A_inv_f32); // [3 x 3]
    arm_mat_init_f32(&b, num_nodes - 1u, 1u,
                     b_f32);             // [(TRILAT_MAX_STATIC_NODES - 1) x 1]
    arm_mat_init_f32(&x, 3u, 1u, x_f32); // [3 x 1]
    arm_mat_init_f32(&JTJinv, 3u, 3u, JTJinv_f32);         // [3 x 3]
    arm_mat_init_f32(&JTJinv_JTf, 3u, 1u, JTJinv_JTf_f32); // [3 x 3]
  } else // num_nodes == TRILAT_MIN_STATIC_NODES
  {
    arm_mat_init_f32(&A, 2u, 2u, A_f32); // [(TRILAT_MAX_STATIC_NODES - 1) x 3]
    arm_mat_init_f32(&Ainv, 2u, 2u,
                     Ainv_f32); // [3 x (TRILAT_MAX_STATIC_NODES - 1)]
    arm_mat_init_f32(&AT, 2u, 2u,
                     AT_f32); // [3 x (TRILAT_MAX_STATIC_NODES - 1)]
    arm_mat_init_f32(&AT_A, 2u, 2u, AT_A_f32);         // [3 x 3]
    arm_mat_init_f32(&AT_A_inv, 2u, 2u, AT_A_inv_f32); // [3 x 3]
    arm_mat_init_f32(&b, 2u, 1u, b_f32); // [(TRILAT_MAX_STATIC_NODES - 1) x 1]
    arm_mat_init_f32(&x, 2u, 1u, x_f32); // [3 x 1]
  }
}

/**
 * @brief  Add new static node into FILO list
 * @param  x Position of node wrt inertial reference frame (along X-axis) [m]
 * @param  y Position of node wrt inertial reference frame (along Y-axis) [m]
 * @param  z Position of node wrt inertial reference frame (along Z-axis) [m]
 * @retval -1=Failure, 1=Success
 */
int8_t TRILAT_AddStaticNode(float x, float y, float z) {
  if (node_cnt_new >= TRILAT_MAX_STATIC_NODES)
    return -1;

  node_new[node_cnt_new].x = x; // [m]
  node_new[node_cnt_new].y = y; // [m]
  node_new[node_cnt_new].z = z; // [m]
  node_cnt_new++;

  return 1;
}

/**
 * @brief  Update static nodes
 * @param  None
 * @retval -1=Failure, 1=Success
 */
// @TODO Handle matrix singularity of A
int8_t TRILAT_UpdateStaticNodes(void) {
  // update number of nodes for trilateration
  node_cnt = node_cnt_new;
  node_cnt_new = 0u; // empty FILO list

  // if insufficient static nodes, exit
  if (node_cnt < TRILAT_MIN_STATIC_NODES)
    return -1;

  TRILAT_AllocateMemory(node_cnt); // re-initialise memory size

  // update static nodes with those kept in memory
  uint8_t i = 0u;
  for (i = 0u; i < node_cnt; i++) {
    node[i].x = node_new[i].x; // [m]
    node[i].y = node_new[i].y; // [m]
    node[i].z = node_new[i].z; // [m]
  }

  // construct A-matrix
  uint8_t k = 0u;
  for (i = 1u; i < node_cnt; i++) {
    A_f32[k++] = node[i].x - node[0].x;
    A_f32[k++] = node[i].y - node[0].y;
    if (node_cnt > TRILAT_MIN_STATIC_NODES) {
      A_f32[k++] = node[i].z - node[0].z;
    }
  }

  // pre-compute di_sqrd-vector
  for (i = 1u; i < node_cnt; i++) {
    di1_sqrd[i - 1] = (node[i].x - node[0].x) * (node[i].x - node[0].x) +
                      (node[i].y - node[0].y) * (node[i].y - node[0].y);
    if (node_cnt > TRILAT_MIN_STATIC_NODES) {
      di1_sqrd[i - 1] += (node[i].z - node[0].z) * (node[i].z - node[0].z);
    }
  }

#if 0 // use arm_math library
  // @TODO: arm_math can't handle A=[0,1;1,1] -> Ainv=[-1,1;1,0]
  arm_status status;

  // pre-compute inverse A-matrix
  // NOTE: arm_mat_inverse_f32() overwrites the data from the source matrix
  //       arm_mat_inverse_f32() return SINGULAR for diagonal matrices, see: http://community.arm.com/thread/6570
  if (node_cnt < 5u) // regular inverse for [3x3]
  {
    // @TODO: even though A may be linearly dependent, this will still return a valid matrix
    status = arm_mat_inverse_f32(&A, &Ainv); if (status != ARM_MATH_SUCCESS && status != ARM_MATH_SINGULAR) return -1;
  } 
  else // pseudo-inverse with linearly independent columns, A^-1 = (A'*A)^-1 * A'
  {
    status = arm_mat_trans_f32(&A, &AT); TRILAT_PrintARMStatus(status); if (status != ARM_MATH_SUCCESS) return -1;
    status = arm_mat_mult_f32(&AT, &A, &AT_A); TRILAT_PrintARMStatus(status);  if (status != ARM_MATH_SUCCESS) return -1;
    status = arm_mat_inverse_f32(&AT_A, &AT_A_inv); if (status != ARM_MATH_SUCCESS && status != ARM_MATH_SINGULAR) return -1;
    status = arm_mat_mult_f32(&AT_A_inv, &AT, &Ainv); TRILAT_PrintARMStatus(status);  if (status != ARM_MATH_SUCCESS) return -1;
  }
#else // use linalgebra library
  // fill linalgebra data structure
  float **TEMP = LIN_ALG_fmatrix(A.numRows, A.numCols);
  uint8_t m, n;
  k = 0u;
  for (m = 0u; m < A.numRows; m++) {
    for (n = 0u; n < A.numCols; n++) {
      TEMP[m][n] = A.pData[k++];
    }
  }

  // UART_Print(VERBOSE_UART_ID,"A:\r\n");
  // for (m=0u; m<A.numRows; m++)
  // {
  //   for (n=0u; n<A.numCols; n++)
  //   {
  //     char str[100]; sprintf(str,"%.3f ",TEMP[m][n]);
  //     UART_Print(VERBOSE_UART_ID,str);
  //   }
  //   UART_Print(VERBOSE_UART_ID,"\r\n");
  // }

  // solve for inverse
  float **TEMPinv = LIN_ALG_fmatrix(A.numCols, A.numRows);
  if (node_cnt < 5u) // regular inverse for [3x3]
  {
    LIN_ALG_inverse(TEMP, TEMPinv, A.numRows);
  } else {
    LIN_ALG_Lpseudoinverse(TEMP, TEMPinv, A.numRows, A.numCols);
  }

  // fill arm_math data structure
  k = 0u;
  for (m = 0u; m < A.numCols; m++) {
    for (n = 0u; n < A.numRows; n++) {
      Ainv_f32[k++] = TEMPinv[m][n];
    }
  }

  // UART_Print(VERBOSE_UART_ID,"Ainv:\r\n");
  // for (m=0u; m<A.numCols; m++)
  // {
  //   for (n=0u; n<A.numRows; n++)
  //   {
  //     char str[100]; sprintf(str,"%.3f ",TEMPinv[m][n]);
  //     UART_Print(VERBOSE_UART_ID,str);
  //   }
  //   UART_Print(VERBOSE_UART_ID,"\r\n");
  // }

  LIN_ALG_free_fmatrix(TEMP, A.numRows, A.numCols);
  LIN_ALG_free_fmatrix(TEMPinv, A.numCols, A.numRows);
#endif

  return 1;
}

/**
 * @brief  Update mobile nodes
 * @param  range Pointer to range of all static nodes
 * @param  size Size of pointer list
 * @retval -1=Failure, 1=Success
 */
// @TODO Handle matrix singularity of JTJ
int8_t TRILAT_UpdateMobileNode(float *range) {
  arm_status status;
  uint8_t valid = 1u;

  // if insufficient static nodes, exit
  if (node_cnt < TRILAT_MIN_STATIC_NODES)
    return -1;

  // construct b-vector
  uint8_t i;
  for (i = 1u; i < node_cnt; i++) {
    b_f32[i - 1] =
        0.5f * (range[0] * range[0] - range[i] * range[i] + di1_sqrd[i - 1]);
  }

  // solve least squares
  status = arm_mat_mult_f32(&Ainv, &b, &x);
  TRILAT_PrintARMStatus(status);
  if (status != ARM_MATH_SUCCESS)
    valid = -1;

  // shift by node-1
  x_f32[0] += node[0].x;
  x_f32[1] += node[0].y;
  if (node_cnt > TRILAT_MIN_STATIC_NODES) {
    x_f32[2] += node[0].z;
  }

  // refine solution
  if (node_cnt > TRILAT_MIN_STATIC_NODES) {
    // use least square method as an initial guess
    // to non-linear least squares using Newton-Raphson Method
    uint8_t itr;
    for (itr = 0u; itr < NEWTON_RAPHSON_ITERATIONS; itr++) {
      // construct JTJ and JTf
      arm_matrix_instance_f32 JTJ;
      arm_matrix_instance_f32 JTf;
      float32_t JTJ_f32[9] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                              0.0f, 0.0f, 0.0f, 0.0f}; // [3 x 3]
      float32_t JTf_f32[3] = {0.0f, 0.0f, 0.0f};       // [3 x 1]
      arm_mat_init_f32(&JTJ, 3u, 3u, JTJ_f32);         // [3 x 3]
      arm_mat_init_f32(&JTf, 3u, 1u, JTf_f32);         // [3 x 1]
      for (i = 0u; i < node_cnt; i++) {
        float x_xi = x_f32[0] - node[i].x;
        float y_yi = x_f32[1] - node[i].y;
        float z_zi = x_f32[2] - node[i].z;

        float di_sqrd = x_xi * x_xi + y_yi * y_yi + z_zi * z_zi;

        float x_xi_over_di_sqrd = x_xi / di_sqrd;
        float y_yi_over_di_sqrd = y_yi / di_sqrd;
        float z_zi_over_di_sqrd = z_zi / di_sqrd;

        JTJ_f32[0] += x_xi * x_xi_over_di_sqrd;
        JTJ_f32[1] += x_xi * y_yi_over_di_sqrd;
        JTJ_f32[2] += x_xi * z_zi_over_di_sqrd;

        JTJ_f32[3] += y_yi * x_xi_over_di_sqrd;
        JTJ_f32[4] += y_yi * y_yi_over_di_sqrd;
        JTJ_f32[5] += y_yi * z_zi_over_di_sqrd;

        JTJ_f32[6] += z_zi * x_xi_over_di_sqrd;
        JTJ_f32[7] += z_zi * y_yi_over_di_sqrd;
        JTJ_f32[8] += z_zi * z_zi_over_di_sqrd;

        float di = sqrtf(di_sqrd);
        float fi_over_di = (di - range[i]) / di;

        JTf_f32[0] += x_xi * fi_over_di;
        JTf_f32[1] += y_yi * fi_over_di;
        JTf_f32[2] += z_zi * fi_over_di;
      }

      // save x
      float x_old[3] = {x_f32[0], x_f32[1], x_f32[2]};

      // solve non-linear least squares
      status = arm_mat_inverse_f32(&JTJ, &JTJinv);
      if (status != ARM_MATH_SUCCESS && status != ARM_MATH_SINGULAR)
        valid = -1;
      status = arm_mat_mult_f32(&JTJinv, &JTf, &JTJinv_JTf);
      TRILAT_PrintARMStatus(status);
      if (status != ARM_MATH_SUCCESS)
        valid = -1;
      x_f32[0] -= JTJinv_JTf_f32[0];
      x_f32[1] -= JTJinv_JTf_f32[1];
      x_f32[2] -= JTJinv_JTf_f32[2];

      // compute change
      float x_xo = x_f32[0] - x_old[0];
      float y_yo = x_f32[1] - x_old[1];
      float z_zo = x_f32[2] - x_old[2];
      float norm2 = x_xo * x_xo + y_yo * y_yo + z_zo * z_zo;

      if (norm2 < NEWTON_RAPHSON_DELTA)
        break;
    }
  }

  if (!valid) {
    position.x = 0.0f;
    position.y = 0.0f;
    position.z = 0.0f;
  } else {
    position.x = x_f32[0];
    position.y = x_f32[1];
    if (node_cnt > TRILAT_MIN_STATIC_NODES) {
      position.z = x_f32[2];
    } else {
      position.z = 0.0;
    }
  }

  return valid;
}

float TRILAT_GetX(void) { return position.x; }
float TRILAT_GetY(void) { return position.y; }
float TRILAT_GetZ(void) { return position.z; }
uint8_t TRILAT_GetNumNodes(void) { return node_cnt_new; }

void TRILAT_PrintARMStatus(arm_status status) {
#ifdef _VERBOSE
  switch (status) {
  case ARM_MATH_SUCCESS:
    break; // No error
  case ARM_MATH_ARGUMENT_ERROR:
    UART_Print(VERBOSE_UART_ID, "ARM-MATH: ARGUMENT ERROR\r\n");
    break; // One or more arguments are incorrect
  case ARM_MATH_LENGTH_ERROR:
    UART_Print(VERBOSE_UART_ID, "ARM-MATH: LENGTH ERROR\r\n");
    break; // Length of data buffer is incorrect
  case ARM_MATH_SIZE_MISMATCH:
    UART_Print(VERBOSE_UART_ID, "ARM-MATH: SIZE MISMATCH\r\n");
    break; // Size of matrices is not compatible with the operation.
  case ARM_MATH_NANINF:
    UART_Print(VERBOSE_UART_ID, "ARM-MATH: NANINF\r\n");
    break; // Not-a-number (NaN) or infinity is generated
  case ARM_MATH_SINGULAR:
    UART_Print(VERBOSE_UART_ID, "ARM-MATH: SINGULAR\r\n");
    break; // Generated by matrix inversion if the input matrix is singular and
           // cannot be inverted.
  case ARM_MATH_TEST_FAILURE:
    UART_Print(VERBOSE_UART_ID, "ARM-MATH: TEST FAILURE\r\n");
    break; // Test Failed
  }
#endif
}
/**
 * @}
 */

/**
 * @}
 */
