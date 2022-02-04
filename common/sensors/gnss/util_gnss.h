/**
 * @file    util_gnss.h
 * @author  Raymond Oung
 * @date    2013.10.29
 * @brief   GNSS utilities
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
#ifndef __UTIL_GNSS_H
#define __UTIL_GNSS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#ifdef STM32F10X_MD
#include "stm32f10x.h"
#elif STM32F40_41xxx
#include "stm32f4xx.h"
#endif
/** @addtogroup Source
 * @{
 */

/** @addtogroup Utilities
 * @{
 */

/** @addtogroup UTIL_GNSS
 * @{
 */

/** @defgroup UTIL_GNSS_TypesDefinitions
 * @{
 */

/**
 * @}
 */

/** @defgroup UTIL_GNSS_Exported_Constants
 * @{
 */
/**
 * @}
 */

/** @defgroup UTIL_GNSS_Exported_Macros
 * @{
 */
/**
 * @}
 */

/** @defgroup UTIL_GNSS_Exported_Functions
 * @{
 */
void ecef2enu_p(float ecef_x, float ecef_y, float ecef_z, float p_x, float p_y,
                float p_z, float *enu_e, float *enu_n, float *enu_u);
void ecef2enu_v(float ecef_dx, float ecef_dy, float ecef_dz, float p_x,
                float p_y, float p_z, float *enu_de, float *enu_dn,
                float *enu_du);
void enu2ecef_p(float enu_e, float enu_n, float enu_u, float p_x, float p_y,
                float p_z, float *ecef_x, float *ecef_y, float *ecef_z);
void ecef2lla(float x, float y, float z, float *lat, float *lon, float *alt);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __UTIL_GNSS_H */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
