/**
 * @file    linalgebra_util.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LINALGEBRA_UTIL_H
#define __LINALGEBRA_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup Source
 * @{
 */

/** @addtogroup Utilities
 * @{
 */

/** @addtogroup LIN_ALGEBRA_UTIL
 * @{
 */

/** @defgroup LIN_ALGEBRA_Exported_Macros
 * @{
 */
/**
 * @}
 */

extern uint16_t LIN_ALG_memory;

/** @defgroup LIN_ALGEBRA_Exported_Functions
 * @{
 */

float *LIN_ALG_fvector(uint16_t n);
uint8_t *LIN_ALG_ui8vector(uint16_t n);
uint16_t *LIN_ALG_ui16vector(uint16_t n);
float **LIN_ALG_fmatrix(uint16_t nr, uint16_t nc);
uint16_t **LIN_ALG_ui16matrix(uint16_t nr, uint16_t nc);
void LIN_ALG_free_fvector(float *v, uint16_t n);
void LIN_ALG_free_ui8vector(uint8_t *v, uint16_t n);
void LIN_ALG_free_ui16vector(uint16_t *v, uint16_t n);
void LIN_ALG_free_fmatrix(float **m, uint16_t nr, uint16_t nc);
void LIN_ALG_free_ui16_tmatrix(uint16_t **m, uint16_t nr, uint16_t nc);
void LIN_ALG_identity(float **m, uint16_t nr);
void LIN_ALG_ludcmp(float **a, uint16_t n, uint16_t *indx, float *d);
void LIN_ALG_lubksb(float **a, uint16_t n, uint16_t *indx, float *b);
void LIN_ALG_jacobi(float **a, uint16_t n, float *d, float **v, uint16_t *nrot);
void LIN_ALG_jacobi_srt(float **a, uint16_t n, float *d, float **v);
void LIN_ALG_eigsrt(float *d, float **v, uint16_t n);
void LIN_ALG_inverse(float **a, float **a_inv, uint16_t n);
void LIN_ALG_Rpseudoinverse(float **a, float **a_pinv, uint16_t nra,
                            uint16_t nca);
void LIN_ALG_Lpseudoinverse(float **a, float **a_pinv, uint16_t nra,
                            uint16_t nca);
void LIN_ALG_transpose_m2m(float **a, float **b, uint16_t nra, uint16_t nca);
void LIN_ALG_transpose_v2m(float *a, float **b, uint16_t nca);
void LIN_ALG_transpose_m2v(float **a, float *b, uint16_t nra);
void LIN_ALG_mv_mul(float **a, float *b, float *c, uint16_t nra, uint16_t nca);
void LIN_ALG_mv_mul_3x1(float **a, float *b, float *c);
void LIN_ALG_mv_mul_4x1(float **a, float *b, float *c);
void LIN_ALG_mm_mul_inner(float **a, float **b, float **c, uint16_t nra,
                          uint16_t nca, uint16_t ncb);
void LIN_ALG_mm_mul_scalar(float **a, float b, float **c, uint16_t nra,
                           uint16_t nca);
void LIN_ALG_vv_mul_inner(float *a, float *b, float *c, uint16_t nca);
void LIN_ALG_vv_mul_outer(float *a, float *b, float **c, uint16_t nca,
                          uint16_t ncb);
void LIN_ALG_vv_mul_cross(float *a, float *b, float *c);
void LIN_ALG_mm_add(float **a, float **b, float **c, uint16_t nra,
                    uint16_t nca);
void LIN_ALG_mm_sub(float **a, float **b, float **c, uint16_t nra,
                    uint16_t nca);
void LIN_ALG_vv_add(float *a, float *b, float *c, uint16_t nca);
void LIN_ALG_vv_sub(float *a, float *b, float *c, uint16_t nca);
float LIN_ALG_v_sum(float *a, uint16_t nca);
float LIN_ALG_v_norm(float *a, uint16_t nca);
#ifdef _SIM
void LIN_ALG_m_print(float **m, uint8_t nra, uint8_t nca);
void LIN_ALG_v_print(float *v, uint8_t nra);
#endif
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __LINALGEBRA H */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
