/**
 * @file    mpu9250.h
 * @author  Raymond Oung
 * @date    2014.05.11
 * @brief   MPU9150 (rate-gyro / accelerometer / magnetometer) functions
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
#ifndef __MPU9250_H
#define __MPU9250_H

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

/** @addtogroup MPU9250
 * @{
 */

/** @defgroup MPU9250_TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @defgroup MPU9250_Exported_Constants
 * @{
 */
/**
 * @}
 */

/** @defgroup MPU9250_Exported_Defines
 * @{
 */
#define MPU9250_GYRO_BW_5                                                      \
  0x06 // 5 Hz Bandwidth of Rate-Gyro and Temperature sensor at 1KHz sampling
       // rate
#define MPU9250_GYRO_BW_10                                                     \
  0x05 // 10 Hz Bandwidth of Rate-Gyro and Temperature sensor at 1KHz sampling
       // rate
#define MPU9250_GYRO_BW_20                                                     \
  0x04 // 20 Hz Bandwidth of Rate-Gyro and Temperature sensor at 1KHz sampling
       // rate
#define MPU9250_GYRO_BW_41                                                     \
  0x03 // 41 Hz Bandwidth of Rate-Gyro and Temperature sensor at 1KHz sampling
       // rate
#define MPU9250_GYRO_BW_92                                                     \
  0x02 // 92 Hz Bandwidth of Rate-Gyro and Temperature sensor at 1KHz sampling
       // rate
#define MPU9250_GYRO_BW_184                                                    \
  0x01 // 184 Hz Bandwidth of Rate-Gyro and Temperature sensor at 1KHz sampling
       // rate
#define MPU9250_GYRO_BW_250                                                    \
  0x00 // 250 Hz Bandwidth of Rate-Gyro and Temperature sensor at 8KHz sampling
       // rate (need an external crystal for this)

#define MPU9250_GYRO_RANGE_250 0x00  // ±250 deg/s
#define MPU9250_GYRO_RANGE_500 0x08  // ±500 deg/s
#define MPU9250_GYRO_RANGE_1000 0x10 // ±1000 deg/s
#define MPU9250_GYRO_RANGE_2000 0x18 // ±2000 deg/s

#define MPU9250_ACCL_BW_5                                                      \
  0x06 // 5 Hz Bandwidth of Rate-Gyro and Temperature sensor at 1KHz sampling
       // rate
#define MPU9250_ACCL_BW_10                                                     \
  0x05 // 10 Hz Bandwidth of Rate-Gyro and Temperature sensor at 1KHz sampling
       // rate
#define MPU9250_ACCL_BW_20                                                     \
  0x04 // 20 Hz Bandwidth of Rate-Gyro and Temperature sensor at 1KHz sampling
       // rate
#define MPU9250_ACCL_BW_41                                                     \
  0x03 // 41 Hz Bandwidth of Rate-Gyro and Temperature sensor at 1KHz sampling
       // rate
#define MPU9250_ACCL_BW_92                                                     \
  0x02 // 92 Hz Bandwidth of Rate-Gyro and Temperature sensor at 1KHz sampling
       // rate
#define MPU9250_ACCL_BW_184                                                    \
  0x01 // 184 Hz Bandwidth of Rate-Gyro and Temperature sensor at 1KHz sampling
       // rate
#define MPU9250_ACCL_BW_460                                                    \
  0x00 // 460 Hz Bandwidth of Rate-Gyro and Temperature sensor at 8KHz sampling
       // rate (need an external crystal for this)

#define MPU9250_ACCL_RANGE_2 0x00  // ±2 g
#define MPU9250_ACCL_RANGE_4 0x08  // ±4 g
#define MPU9250_ACCL_RANGE_8 0x10  // ±8 g
#define MPU9250_ACCL_RANGE_16 0x18 // ±16 g

#define MPU9250_MAGN_RATE_8                                                    \
  0x12 // 8 Hz (continuous measurement mode 1) with 16-bit Resolution
#define MPU9250_MAGN_RATE_100                                                  \
  0x16 // 100 Hz (continuous measurement mode 2) with 16-bit Resolution
/**
 * @}
 */

/** @defgroup MPU9250_Exported_Functions
 * @{
 */
uint8_t MPU9250_Init(I2C_TypeDef *I2Cx, uint8_t config, uint8_t gyro_config,
                     uint8_t accl_config, uint8_t accl_config2,
                     uint8_t magn_config);

void MPU9250_ReadSensor(void);
void AK8963_ReadSensor(void);

uint8_t MPU9250_ReadSensorIT(void);
uint8_t AK8963_ReadSensorIT(void);

float MPU9250_GetTemp(void); // [deg C]

float MPU9250_GetGyroX(void); // [rad/s]
float MPU9250_GetGyroY(void); // [rad/s]
float MPU9250_GetGyroZ(void); // [rad/s]

float MPU9250_GetAcclX(void); // [g]
float MPU9250_GetAcclY(void); // [g]
float MPU9250_GetAcclZ(void); // [g]

float MPU9250_GetMagnX(void); // [uT]
float MPU9250_GetMagnY(void); // [uT]
float MPU9250_GetMagnZ(void); // [uT]
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __MPU9250_H */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
