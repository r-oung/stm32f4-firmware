/**
 * @file    linalgebra_util.c
 * @author  Maximilian Kriegleder
 * @author  Raymond Oung
 * @date    2011.02.10
 * @brief   Linear algebar functions
 * @ref 		Numerical recipes in C
 *
 * MIT License
 *
 * Copyright (c) 2022 Maximilian Kriegleder & Raymond Oung
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
#include "linalgebra.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

uint16_t LIN_ALG_memory;

/** @addtogroup Source
 * @{
 */

/** @addtogroup Utilities
 * @{
 */

/** @addtogroup LIN_ALGEBRA_UTIL
 * @{
 */

/** @defgroup LIN_ALGEBRA_Private_Macros
 * @{
 */
#define TINY 1.0e-20f;
#define ROTATE(a, i, j, k, l)                                                  \
  g = a[i][j];                                                                 \
  h = a[k][l];                                                                 \
  a[i][j] = g - s * (h + g * tau);                                             \
  a[k][l] = h + s * (g - h * tau);
/**
 * @}
 */

/** @defgroup FMATH_Private_Variables
 * @{
 */

/**
 * @}
 */

/** @defgroup FMATH_Private_Functions
 * @{
 */
/**
 * @}
 */

// @todo When allocating memory for LIN_ALG variables, we should check whether
// memory has already been allocated or use realloc

/**
 * @brief Allocate memory for a float vector
 * @param[in] n dimension of vector
 * @retval v pointer to allocated memory
 */
float *LIN_ALG_fvector(uint16_t n) {
  float *v;
  uint16_t i;

  v = (float *)malloc((unsigned)n * sizeof(float));

  if (v != NULL) {
    for (i = 0; i < n; i++)
      v[i] = 0.0f;

    LIN_ALG_memory = LIN_ALG_memory + n * sizeof(float);
  }

  return v;
}

/**
 * @brief Allocate memory for a uint8_t vector
 * @param[in] n dimension of vector
 * @retval v pointer to allocated memory
 */
uint8_t *LIN_ALG_ui8vector(uint16_t n) {
  uint8_t *v;
  uint16_t i;

  v = (uint8_t *)malloc((unsigned)n * sizeof(uint8_t));

  if (v != NULL) {
    for (i = 0; i < n; i++)
      v[i] = 0;

    LIN_ALG_memory = LIN_ALG_memory + n * sizeof(uint8_t);
  }

  return v;
}

/**
 * @brief Allocate memory for a uint16_t vector
 * @param[in] n dimension of vector
 * @retval v pointer to allocated memory
 */
uint16_t *LIN_ALG_ui16vector(uint16_t n) {
  uint16_t *v;
  uint16_t i;

  v = (uint16_t *)malloc((unsigned)n * sizeof(uint16_t));

  if (v != NULL) {
    for (i = 0; i < n; i++)
      v[i] = 0;

    LIN_ALG_memory = LIN_ALG_memory + n * sizeof(uint16_t);
  }

  return v;
}

/**
 * @brief Allocate memory for a float matrix
 * @param[in] nr number of rows
 * @param[in] nc number of cols
 * @retval m pointer to allocated memory
 */
float **LIN_ALG_fmatrix(uint16_t nr, uint16_t nc) {
  uint16_t i, j;
  float **m;

  m = (float **)malloc((unsigned)nr * sizeof(float *));

  LIN_ALG_memory = LIN_ALG_memory + nr * sizeof(float *);

  for (i = 0; i < nr; i++) {
    m[i] = (float *)malloc((unsigned)nc * sizeof(float));
    LIN_ALG_memory = LIN_ALG_memory + nc * sizeof(float);
  }

  if (m != NULL) {
    for (i = 0; i < nr; i++)
      if (m[i] != NULL) {
        for (j = 0; j < nc; j++)
          m[i][j] = 0.0f;
      }
  }

  return m;
}

/**
 * @brief Allocate memory for a uint16_t matrix
 * @param[in] nr number of rows
 * @param[in] nc number of cols
 * @retval m pointer to allocated memory
 */
uint16_t **LIN_ALG_ui16matrix(uint16_t nr, uint16_t nc) {
  uint16_t i, j;
  uint16_t **m;

  m = (uint16_t **)malloc((unsigned)nr * sizeof(uint16_t *));
  LIN_ALG_memory = LIN_ALG_memory + nr * sizeof(uint16_t *);

  for (i = 0; i < nr; i++) {
    m[i] = (uint16_t *)malloc((unsigned)nc * sizeof(uint16_t));
    LIN_ALG_memory = LIN_ALG_memory + nc * sizeof(uint16_t);
  }

  if (m != NULL) {
    for (i = 0; i < nr; i++)
      if (m[i] != NULL) {
        for (j = 0; j < nc; j++)
          m[i][j] = 0;
      }
  }

  return m;
}

/**
 * @brief Free memory of a float vector
 * @param[in] v vector
 * @retval None
 */
void LIN_ALG_free_fvector(float *v, uint16_t n) {
  if (v == NULL)
    return;

  free(v);
  v = NULL;
  LIN_ALG_memory = LIN_ALG_memory - n * sizeof(float);
}

/**
 * @brief Free memory of a uint8_t vector
 * @param[in] v vector
 * @retval None
 */
void LIN_ALG_free_ui8vector(uint8_t *v, uint16_t n) {
  if (v == NULL)
    return;

  free(v);
  v = NULL;
  LIN_ALG_memory = LIN_ALG_memory - n * sizeof(uint8_t);
}

/**
 * @brief Free memory of a uint16_t vector
 * @param[in] v vector
 * @retval None
 */
void LIN_ALG_free_ui16vector(uint16_t *v, uint16_t n) {
  if (v == NULL)
    return;

  free(v);
  v = NULL;
  LIN_ALG_memory = LIN_ALG_memory - n * sizeof(uint16_t);
}

/**
 * @brief Free memory of a float matrix
 * @param[in] m matrix
 * @param[in] nr number of rows
 * @retval None
 */
void LIN_ALG_free_fmatrix(float **m, uint16_t nr, uint16_t nc) {
  uint16_t i;

  if (m == NULL)
    return;

  for (i = 0; i < nr; i++) {
    free(m[nr - 1 - i]);
    LIN_ALG_memory = LIN_ALG_memory - nc * sizeof(float);
  }

  free(m);
  m = NULL;
  LIN_ALG_memory = LIN_ALG_memory - nr * sizeof(float *);
}

/**
 * @brief Free memory of a uint16_t matrix
 * @param[in] m matrix
 * @param[in] nr number of rows
 * @retval None
 */
void LIN_ALG_free_ui16matrix(uint16_t **m, uint16_t nr, uint16_t nc) {
  uint16_t i;

  if (m == NULL)
    return;

  for (i = 0; i < nr; i++) {
    free(m[nr - 1 - i]);
    LIN_ALG_memory = LIN_ALG_memory - nc * sizeof(uint16_t);
  }

  free(m);
  m = NULL;
  LIN_ALG_memory = LIN_ALG_memory - nr * sizeof(uint16_t);
}

/**
 * @brief Create identity matrix
 * @param[in] m matrix
 * @param[in] nr number of rows
 * @retval None
 */
void LIN_ALG_identity(float **m, uint16_t nr) {
  uint8_t i;
  uint8_t j;

  for (i = 0; i < nr; i++) {
    for (j = 0; j < nr; j++) {
      if (i == j) {
        m[i][j] = 1;
      }
    }
  }
}

/**
 * @brief LU - Decomposition of square matrices
 * @param[in/out] a matrix to be decomposed
 * @param[in] n dimension
 * @param[out] indx tracking of row permutation
 * @param[out] d tracking if number of permutations is odd/even
 * @todo prevent decomposition of singular matrices
 * @retval None
 */
void LIN_ALG_ludcmp(float **a, uint16_t n, uint16_t *indx, float *d) {
  uint8_t i = 0u, imax = 0u, j = 0u, k = 0u;
  float big, dum, sum, temp;
  float *vv;

  vv = LIN_ALG_fvector((uint16_t)n);
  *d = 1.0;
  for (i = 0; i < n; i++) {
    big = 0.0;
    for (j = 0; j < n; j++)
      if ((temp = (float)fabs(a[i][j])) > big)
        big = temp;

    // if(big == 0.0) singular!
    vv[i] = 1.0f / big;
  }
  for (j = 0; j < n; j++) {
    for (i = 0; i < j; i++) {
      sum = a[i][j];
      for (k = 0; k < i; k++)
        sum -= a[i][k] * a[k][j];
      a[i][j] = sum;
    }
    big = 0.0;
    for (i = j; i < n; i++) {
      sum = a[i][j];
      for (k = 0; k < j; k++)
        sum -= a[i][k] * a[k][j];
      a[i][j] = sum;
      if ((dum = (float)(vv[i] * fabs(sum))) >= big) {
        big = dum;
        imax = i;
      }
    }
    if (j != imax) {
      for (k = 0; k < n; k++) {
        dum = a[imax][k];
        a[imax][k] = a[j][k];
        a[j][k] = dum;
      }
      *d = -(*d);
      vv[imax] = vv[j];
    }
    indx[j] = imax;
    if (a[j][j] == 0.0f)
      a[j][j] = TINY;
    if (j != (n - 1)) {
      dum = 1.0f / (a[j][j]);
      for (i = j + 1; i < n; i++)
        a[i][j] *= dum;
    }
  }
  LIN_ALG_free_fvector(vv, (uint16_t)n);
}

/**
 * @brief Backsubstitution of LU-decomposed matrix to solve A*x = b
 * @param[in] a decomposed matrix
 * @param[in] n dimension
 * @param[in] indx tracking of row permutation
 * @param[in/out] b in: b, out: x
 * @retval None
 */
void LIN_ALG_lubksb(float **a, uint16_t n, uint16_t *indx, float *b) {
  int8_t i, ip, j, ii = -1;

  float sum;

  for (i = 0; i < n; i++) {
    ip = indx[i];
    sum = b[ip];
    b[ip] = b[i];

    if (ii != -1)
      for (j = ii; j <= (i - 1); j++)
        sum -= a[i][j] * b[j];
    else if (sum)
      ii = i;
    b[i] = sum;
  }
  for (i = (n - 1); i >= 0; i--) {
    sum = b[i];
    for (j = (i + 1); j < n; j++)
      sum -= a[i][j] * b[j];
    b[i] = sum / a[i][i];
  }
}

/**
 * @brief retrieve sorted eigenvalues/-vectors of REAL SYMMETRIC matrices (with
 * low dimensions)
 * @param[in] a matrix
 * @param[in] n dimension
 * @param[out] d eigenvalues
 * @param[out] v eigenvectors
 * @retval None
 */
void LIN_ALG_jacobi_srt(float **a, uint16_t n, float *d, float **v) {
  uint16_t nrot;

  uint16_t i, j;
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      if (fabs(a[i][j]) < 0.000001) // force small values to zero
      {
        a[i][j] = 0.0f;
      }
    }
  }
  LIN_ALG_jacobi(a, n, d, v, &nrot);
  LIN_ALG_eigsrt(d, v, n);
}

/**
 * @brief retrieve eigenvalues/-vectors of REAL SYMMETRIC matrices (with low
 * dimensions)
 * @param[in] a matrix
 * @param[in] n dimension
 * @param[out] d eigenvalues
 * @param[out] v eigenvectors
 * @param[out] nrot number of roations to get diagonal matrix
 * @retval None
 */
void LIN_ALG_jacobi(float **a, uint16_t n, float *d, float **v,
                    uint16_t *nrot) {
  uint16_t j, iq, ip, i;
  float tresh, theta, tau, t, sm, s, h, g, c, *b, *z;

  b = LIN_ALG_fvector((uint16_t)n);
  z = LIN_ALG_fvector((uint16_t)n);

  for (ip = 0; ip < n; ip++) {
    for (iq = 0; iq < n; iq++)
      v[ip][iq] = 0.0;
    v[ip][ip] = 1.0;
  }

  for (ip = 0; ip < n; ip++) {
    b[ip] = d[ip] = a[ip][ip];
    z[ip] = 0.0;
  }

  *nrot = 0;
  for (i = 0; i < 50; i++) {
    sm = 0.0;
    for (ip = 0; ip < n - 1; ip++) {
      for (iq = ip + 1; iq < n; iq++)
        sm += (float)fabs(a[ip][iq]);
    }
    if (sm == 0.0) {
      LIN_ALG_free_fvector(z, (uint16_t)n);
      LIN_ALG_free_fvector(b, (uint16_t)n);
      return;
    }
    if (i < 3)
      tresh = 0.2f * sm / (n * n);
    else
      tresh = 0.0f;

    for (ip = 0; ip < n - 1; ip++) {
      for (iq = ip + 1; iq < n; iq++) {
        g = (float)(100.0 * fabs(a[ip][iq]));
        if (i > 3 && fabs(d[ip]) + g == fabs(d[ip]) &&
            fabs(d[iq]) + g == fabs(d[iq]))
          a[ip][iq] = 0.0f;
        else if (fabs(a[ip][iq]) > tresh) {
          h = d[iq] - d[ip];
          if (fabs(h) + g == fabs(h))
            t = (a[ip][iq]) / h;
          else {
            theta = 0.5f * h / (a[ip][iq]);
            t = (float)(1.0 / (fabs(theta) + sqrtf(1.0f + theta * theta)));
            if (theta < 0.0f)
              t = -t;
          }

          c = (float)(1.0 / sqrtf(1 + t * t));
          s = t * c;
          tau = s / (1.0f + c);
          h = t * a[ip][iq];
          z[ip] -= h;
          z[iq] += h;
          d[ip] -= h;
          d[iq] += h;
          a[ip][iq] = 0.0f;

          for (j = 0; j <= ip - 1; j++) {
            ROTATE(a, j, ip, j, iq);
          }
          for (j = ip + 1; j <= iq - 1; j++) {
            ROTATE(a, ip, j, j, iq);
          }
          for (j = iq + 1; j < n; j++) {
            ROTATE(a, ip, j, iq, j);
          }
          for (j = 0; j < n; j++) {
            ROTATE(v, j, ip, j, iq);
          }
          ++(*nrot);
        }
      }
    }
    for (ip = 0; ip < n; ip++) {
      b[ip] += z[ip];
      d[ip] = b[ip];
      z[ip] = 0.0;
    }
  }
}

/**
 * @brief sort eigenvalues in ascending order and adjust eigenvectors
 * accordingly
 * @param[in/out] d eigenvalues
 * @param[in/out] v eigenvectors
 * @param[in] n dimension
 * @retval None
 */
void LIN_ALG_eigsrt(float *d, float **v, uint16_t n) {
  uint8_t k, j, i;
  float p;

  for (i = 0; i < n - 1; i++) {
    p = d[k = i];
    for (j = i + 1; j < n; j++)
      if (d[j] <= p)
        p = d[k = j];
    if (k != i) {
      d[k] = d[i];
      d[i] = p;
      for (j = 0; j < n; j++) {
        p = v[j][i];
        v[j][i] = v[j][k];
        v[j][k] = p;
      }
    }
  }
}

/**
 * @brief invert a square matrix
 * @param[in] a decomposed matrix
 * @param[out] a_inv inverted matrix
 * @param[in] n dimension
 * @retval None
 */
void LIN_ALG_inverse(float **a, float **a_inv, uint16_t n) {
  float *col;
  float **a_temp = LIN_ALG_fmatrix(n, n);
  uint16_t i, j;
  uint16_t *indx = LIN_ALG_ui16vector(n);
  float d;

  for (i = 0; i < n; i++)
    for (j = 0; j < n; j++)
      a_temp[i][j] = a[i][j];

  LIN_ALG_ludcmp(a_temp, n, indx, &d);
  col = LIN_ALG_fvector((uint16_t)n);

  for (j = 0; j < n; j++) {
    for (i = 0; i < n; i++)
      col[i] = 0.0f;
    col[j] = 1.0;
    LIN_ALG_lubksb(a_temp, n, indx, col);
    for (i = 0; i < n; i++)
      a_inv[i][j] = col[i];
  }

  LIN_ALG_free_fmatrix(a_temp, n, n);
  LIN_ALG_free_fvector(col, n);
  LIN_ALG_free_ui16vector(indx, n);
}

/**
 * @brief invert a non-square matrix (with linearly independent rows) A_pinv =
 * A'*(A'*A)^-1
 * @param[in] a decomposed matrix [m x n]
 * @param[out] a_pinv inverted matrix [n x m]
 * @param[in] nra number of rows of matrix a [m-rows]
 * @param[in] nca number of columns of matrix a [n-columns]
 * @retval None
 */
void LIN_ALG_Rpseudoinverse(float **a, float **a_pinv, uint16_t nra,
                            uint16_t nca) {
  float **a_trans = LIN_ALG_fmatrix(nca, nra);
  float **a_mul = LIN_ALG_fmatrix(nra, nra);
  float **a_inv = LIN_ALG_fmatrix(nra, nra);

  LIN_ALG_transpose_m2m(a, a_trans, nra, nca);
  LIN_ALG_mm_mul_inner(a, a_trans, a_mul, nra, nca, nra);
  LIN_ALG_inverse(a_mul, a_inv, nra);
  LIN_ALG_mm_mul_inner(a_trans, a_inv, a_pinv, nca, nra, nra);

  LIN_ALG_free_fmatrix(a_trans, nca, nra);
  LIN_ALG_free_fmatrix(a_mul, nra, nra);
  LIN_ALG_free_fmatrix(a_inv, nra, nra);
}

/**
 * @brief invert a non-square matrix (with linearly independent cols) A_pinv =
 * (A'*A)^-1*A'
 * @param[in] a decomposed matrix [m x n]
 * @param[out] a_pinv inverted matrix [n x m]
 * @param[in] nra number of rows of matrix a [m-rows]
 * @param[in] nca number of columns of matrix a [n-columns]
 * @retval None
 */
void LIN_ALG_Lpseudoinverse(float **a, float **a_pinv, uint16_t nra,
                            uint16_t nca) {
  float **a_trans = LIN_ALG_fmatrix(nca, nra);
  float **a_mul = LIN_ALG_fmatrix(nca, nca);
  float **a_inv = LIN_ALG_fmatrix(nca, nca);

  LIN_ALG_transpose_m2m(a, a_trans, nra, nca);
  LIN_ALG_mm_mul_inner(a_trans, a, a_mul, nca, nra, nca);
  LIN_ALG_inverse(a_mul, a_inv, nca);
  LIN_ALG_mm_mul_inner(a_inv, a_trans, a_pinv, nca, nca, nra);

  LIN_ALG_free_fmatrix(a_trans, nca, nra);
  LIN_ALG_free_fmatrix(a_mul, nca, nca);
  LIN_ALG_free_fmatrix(a_inv, nca, nca);
}

/**
 * @brief transpose a matrix
 * @param[in] a matrix
 * @param[out] b transpose
 * @param[in] nra number of rows A
 * @param[in] nca number of cols A
 * @retval None
 */
void LIN_ALG_transpose_m2m(float **a, float **b, uint16_t nra, uint16_t nca) {
  uint8_t i, j;
  float **temp = LIN_ALG_fmatrix(nca, nra);

  for (i = 0; i < nra; i++)
    for (j = 0; j < nca; j++)
      temp[j][i] = a[i][j];

  for (i = 0; i < nra; i++)
    for (j = 0; j < nca; j++)
      b[j][i] = temp[j][i];

  LIN_ALG_free_fmatrix(temp, nca, nra);
}

/**
 * @brief transpose row vector to col vector
 * @param[in] a row vector
 * @param[out] b col vector
 * @param[in] nca dimension of row vector
 * @retval None
 */
void LIN_ALG_transpose_v2m(float *a, float **b, uint16_t nca) {
  uint8_t i;

  for (i = 0; i < nca; i++)
    b[i][0] = a[i];
}

/**
 * @brief transpose col vector to row vector
 * @param[in] a col vector
 * @param[out] b row vector
 * @param[in] nra dimension of col vector
 * @retval None
 */
void LIN_ALG_transpose_m2v(float **a, float *b, uint16_t nra) {
  uint8_t i;

  for (i = 0; i < nra; i++)
    b[i] = a[i][0];
}

/**
 * @brief matrix (nra x nca) and vector (nra x 1) multiplication
 * @param[in] a matrix A
 * @param[in] b vector B
 * @param[out] c vector C
 * @param[in] nra number of rows A
 * @param[in] nca number of cols A
 * @retval None
 */
void LIN_ALG_mv_mul(float **a, float *b, float *c, uint16_t nra, uint16_t nca) {
  uint8_t i, j;
  float sum;
  float *temp = LIN_ALG_fvector(nra);

  for (i = 0; i < nra; i++) {
    sum = 0.0f;
    for (j = 0; j < nca; j++) {
      sum += a[i][j] * b[j];
    }
    temp[i] = sum;
  }

  for (i = 0; i < nra; i++) {
    c[i] = temp[i];
  }

  LIN_ALG_free_fvector(temp, nra);
}

/**
 * @brief matrix (3x3) and vector (3x1) multiplication
 * @param[in] a matrix A (3x3)
 * @param[in] b vector B (3x1)
 * @param[out] c vector C (3x1)
 * @retval None
 */
void LIN_ALG_mv_mul_3x1(float **a, float *b, float *c) {
  uint8_t i, j;

  float temp[3]; // this is needed in case *b and *c are the same pointers
  for (i = 0; i < 3; i++) {
    temp[i] = 0.0f;
    for (j = 0; j < 3; j++) {
      temp[i] += a[i][j] * b[j];
    }
  }

  for (i = 0; i < 3; i++) {
    c[i] = temp[i];
  }
}

/**
 * @brief matrix (4x4) and vector (4x1) multiplication
 * @param[in] a matrix A (4x4)
 * @param[in] b vector B (4x1)
 * @param[out] c vector C (4x1)
 * @retval None
 */
void LIN_ALG_mv_mul_4x1(float **a, float *b, float *c) {
  uint8_t i, j;

  for (i = 0; i < 4; i++) {
    c[i] = 0.0f;
    for (j = 0; j < 4; j++) {
      c[i] += a[i][j] * b[j];
    }
  }
}

/**
 * @brief inner product of two matrices of dimension nra x nca and nca x ncb,
 * C=AxB
 * @param[in] a matrix A
 * @param[in] b matrix B
 * @param[out] c matrix C
 * @param[in] nra number of rows A
 * @param[in] nca number of cols A
 * @param[in] ncb number of cols B
 * @retval None
 */
void LIN_ALG_mm_mul_inner(float **a, float **b, float **c, uint16_t nra,
                          uint16_t nca, uint16_t ncb) {
  uint8_t i, j, k;
  float sum;
  float **temp = LIN_ALG_fmatrix(nra, ncb);

  for (i = 0; i < nra; i++) {
    for (j = 0; j < ncb; j++) {
      sum = 0;
      for (k = 0; k < nca; k++)
        sum += a[i][k] * b[k][j];
      temp[i][j] = sum;
    }
  }

  for (i = 0; i < nra; i++)
    for (j = 0; j < ncb; j++)
      c[i][j] = temp[i][j];

  LIN_ALG_free_fmatrix(temp, nra, ncb);
}

/**
 * @brief product of matrix of dimension nra x nca with a scalar
 * @param[in] a matrix A
 * @param[in] b scalar
 * @param[out] c matrix C
 * @param[in] nra number of rows A
 * @param[in] nca number of cols A
 * @todo commit changes
 * @retval None
 */
void LIN_ALG_mm_mul_scalar(float **a, float b, float **c, uint16_t nra,
                           uint16_t nca) {
  uint8_t i, j;

  for (i = 0; i < nra; i++)
    for (j = 0; j < nca; j++)
      c[i][j] = b * a[i][j];
}

/**
 * @brief inner product two row vectors of dimension 1 x nca and 1 x nca
 * @param[in] a vector A
 * @param[in] b vector B
 * @param[out] c vector C
 * @param[in] nca number of cols A
 * @retval None
 */
void LIN_ALG_vv_mul_inner(float *a, float *b, float *c, uint16_t nca) {
  uint8_t i;

  *c = 0;

  for (i = 0; i < nca; i++)
    *c += a[i] * b[i];
}

/**
 * @brief outer product two row vectors of dimension 1 x nca and 1 x ncb
 * @param[in] a vector A
 * @param[in] b vector B
 * @param[out] c vector C
 * @param[in] nca number of cols A
 * @param[in] ncb number of cols B
 * @retval None
 */
void LIN_ALG_vv_mul_outer(float *a, float *b, float **c, uint16_t nca,
                          uint16_t ncb) {
  uint8_t i, j;

  for (i = 0; i < nca; i++) {
    for (j = 0; j < ncb; j++) {
      c[i][j] = a[i] * b[j];
    }
  }
}

/**
 * @brief cross product two row vectors of dimension 1 x nca and 1 x ncb
 * @param[in] a vector A
 * @param[in] b vector B
 * @param[out] c vector C
 * @retval None
 */
void LIN_ALG_vv_mul_cross(float *a, float *b, float *c) {
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}

/**
 * @brief add two matrices of dimension nra x nca and nra x nca
 * @param[in] a matrix A
 * @param[in] b matrix B
 * @param[out] c matrix C
 * @param[in] nra number of rows A/B
 * @param[in] nca number of cols A/B
 * @retval None
 */
void LIN_ALG_mm_add(float **a, float **b, float **c, uint16_t nra,
                    uint16_t nca) {
  uint8_t i, j;

  for (i = 0; i < nra; i++) {
    for (j = 0; j < nca; j++)
      c[i][j] = a[i][j] + b[i][j];
  }
}

/**
 * @brief subtract two matrices of dimension nra x nca and nra x nca
 * @param[in] a matrix A
 * @param[in] b matrix B
 * @param[out] c matrix C
 * @param[in] nra number of rows A/B
 * @param[in] nca number of cols A/B
 * @retval None
 */
void LIN_ALG_mm_sub(float **a, float **b, float **c, uint16_t nra,
                    uint16_t nca) {
  uint8_t i, j;

  for (i = 0; i < nra; i++) {
    for (j = 0; j < nca; j++)
      c[i][j] = a[i][j] - b[i][j];
  }
}

/**
 * @brief add two vectors of dimension nca
 * @param[in] a vector A
 * @param[in] b vector B
 * @param[out] c vector C
 * @param[in] nca number of cols A/B
 * @retval None
 */
void LIN_ALG_vv_add(float *a, float *b, float *c, uint16_t nca) {
  uint8_t i;

  for (i = 0; i < nca; i++)
    c[i] = a[i] + b[i];
}

/**
 * @brief subtract two vectors of dimension nca
 * @param[in] a vector A
 * @param[in] b vector B
 * @param[out] c vector C
 * @param[in] nca number of cols A/B
 * @retval None
 */
void LIN_ALG_vv_sub(float *a, float *b, float *c, uint16_t nca) {
  uint8_t i;

  for (i = 0; i < nca; i++)
    c[i] = a[i] - b[i];
}

/**
 * @brief sum all values in the vector
 * @param[in] vector of values
 * @param[in] nca number of cols A
 * @retval sum
 */
float LIN_ALG_v_sum(float *a, uint16_t nca) {
  uint8_t i;
  float sum = 0.0f;

  for (i = 0; i < nca; i++) {
    sum += a[i];
  }

  return sum;
}

/**
 * @brief compute norm of the vector
 * @param[in] vector of values
 * @param[in] nca number of cols A
 * @retval norm
 */
float LIN_ALG_v_norm(float *a, uint16_t nca) {
  uint8_t i;
  float sum = 0.0f;

  for (i = 0; i < nca; i++) {
    sum += a[i] * a[i];
  }

  return sqrtf(sum);
}

#ifdef _SIM
/**
 * @brief add two matrices of dimension nra x nca and nra x nca
 * @param[in] m matrix to be printed
 * @param[in] nra number of rows
 * @param[in] nca number of cols
 * @retval None
 */
void LIN_ALG_m_print(float **m, uint8_t nra, uint8_t nca) {
  uint8_t i, j;
  for (i = 0; i < nra; i++) {
    printf("[");
    for (j = 0; j < nca; j++) {
      printf("%+.3f ", m[i][j]);
    }
    printf("]\n");
  }
  printf("\n");
}

/**
 * @brief add two matrices of dimension nra x nca and nra x nca
 * @param[in] v vector to be printed
 * @param[in] nra number of rows
 * @retval None
 */
void LIN_ALG_v_print(float *v, uint8_t nra) {
  uint8_t i;
  printf("[");
  for (i = 0; i < nra; i++) {
    printf("%+.5f ", v[i]);
  }
  printf("]\n\n");
}
#endif
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
