/**
 * @file    ms5611.h
 * @author  Raymond Oung
 * @date    2014.05.25
 * @brief   MS5611 pressure sensor functions
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
#ifndef __MS5611_H
#define __MS5611_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/** @addtogroup Source
 * @{
 */

/** @addtogroup Low_Level
 * @{
 */

/** @addtogroup MS5611
 * @{
 */

/** @defgroup MS5607_Exported_Defines
 * @{
 */
#define MS5607_PRES_OSR_256 0x40  // OSR 256
#define MS5607_PRES_OSR_512 0x42  // OSR 512
#define MS5607_PRES_OSR_1024 0x44 // OSR 1024
#define MS5607_PRES_OSR_2048 0x46 // OSR 2048
#define MS5607_PRES_OSR_4096 0x48 // OSR 4096

#define MS5607_TEMP_OSR_256 0x50  // OSR 256
#define MS5607_TEMP_OSR_512 0x52  // OSR 512
#define MS5607_TEMP_OSR_1024 0x54 // OSR 1024
#define MS5607_TEMP_OSR_2048 0x56 // OSR 2048
#define MS5607_TEMP_OSR_4096 0x58 // OSR 4096
/**
 * @}
 */

/** @defgroup MS5611_Exported_Functions
 * @{
 */
uint8_t MS5611_Init(I2C_TypeDef *I2Cx, uint8_t pres_OSR, uint8_t temp_OSR);
void MS5611_Reset(void);

void MS5611_ReadSensor(void);
uint8_t MS5611_ReadSensorIT(void);

float MS5611_GetPressure(void);         // [Pa]
float MS5611_GetTemperature(void);      // [deg C]
float MS5611_GetConversionPeriod(void); // [msec]
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __MS5611_H */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */