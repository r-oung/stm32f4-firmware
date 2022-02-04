/**
 * @file    navigation.h
 * @author  Raymond Oung
 * @date    2014.10.02
 * @brief   Navigation: IMU position estimation
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
#ifndef __NAV_H
#define __NAV_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/** @addtogroup Source
 * @{
 */

/** @addtogroup Utilities
 * @{
 */

/** @addtogroup EST
 * @{
 */

/** @defgroup NAV_TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @defgroup NAV_Exported_Constants
 * @{
 */
/**
 * @}
 */

/** @defgroup NAV_Exported_Macros
 * @{
 */
/**
 * @}
 */

/** @defgroup NAV_Exported_Functions
 * @{
 */
void NAV_Init(void);

void NAV_SetAcclCalib(float A00, float A01, float A02, float A10, float A11,
                      float A12, float A20, float A21, float A22, float P0,
                      float P1, float P2);
void NAV_SetMagnCalib(float V0, float V1, float V2);

uint8_t NAV_CalibrateStatic(uint16_t t);
void NAV_CalibrateDynamic(void);

void NAV_KfPredict_xyz(uint8_t i, float meas, float varMeas, float varErr);
void NAV_KfPredict_dxyz(uint8_t i, float meas, float varMeas, float varErr);
void NAV_KfPredict_gba(uint8_t i, float meas, float varMeas, float varErr);
void NAV_KfCorrect_xyz(uint8_t i, float meas, float var);
void NAV_KfCorrect_gba(uint8_t i, float meas, float var);
void NAV_KfCorrect_dxyz(uint8_t i, float meas, float var);

void NAV_SensGyro(float gx, float gy, float gz);
void NAV_SensAccl(float ax, float ay, float az);
void NAV_SensMagn(float mx, float my, float mz);
void NAV_SensBaro(float raw_baro);
void NAV_SensECEFp(float *raw_ecef_p);
void NAV_SensECEFv(float *raw_ecef_v, float *raw_ecef_p);
void NAV_SensNEDv(float *raw_ned_v);

void NAV_SetStateXYZ(uint8_t i, float val);
void NAV_SetStateDXYZ(uint8_t i, float val);
void NAV_SetStateGBA(uint8_t i, float val);
void NAV_SetStateDGBA(uint8_t i, float val);
void NAV_SetStateDGBAbias(uint8_t i, float val);

float *NAV_GetStateXYZ(void);
float *NAV_GetStateDXYZ(void);
float *NAV_GetStateGBA(void);
float *NAV_GetStateDGBA(void);
float *NAV_GetStateDGBAbias(void);

float *NAV_GetKfStateXYZ(void);
float *NAV_GetKfStateDXYZ(void);
float *NAV_GetKfStateGBA(void);
float *NAV_GetKfStateDGBA(void);
float *NAV_GetKfStateDGBAbias(void);

float NAV_GetAcclX(void);
float NAV_GetAcclY(void);
float NAV_GetAcclZ(void);
float NAV_GetAcclDX(void);
float NAV_GetAcclDY(void);
float NAV_GetAcclDZ(void);
float NAV_GetAcclRoll(void);
float NAV_GetAcclPitch(void);
float NAV_GetMagnYaw(void);
float NAV_GetMagnHeading(void);
float NAV_GetMagnHeadingInit(void);
void NAV_SetBodyHeadingInit(float val);

float NAV_GetBaroZ(void);
float NAV_GetBaroDZ(void);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __NAV_H */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
