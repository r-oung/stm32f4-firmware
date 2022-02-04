/**
 * @file    mpu9150.h
 * @author  Raymond Oung
 * @date    2012.03.21
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
#ifndef __MPU9150_H
#define __MPU9150_H

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

/** @addtogroup MPU9150
 * @{
 */

/** @defgroup MPU9150_Exported_Defines
 * @{
 */
#define MPU9150_BANDWIDTH_5                                                    \
  0x06 // 5 Hz Bandwidth of Rate-Gyro and Temperature sensor at 1KHz sampling
       // rate
#define MPU9150_BANDWIDTH_10                                                   \
  0x05 // 10 Hz Bandwidth of Rate-Gyro and Temperature sensor at 1KHz sampling
       // rate
#define MPU9150_BANDWIDTH_20                                                   \
  0x04 // 20 Hz Bandwidth of Rate-Gyro and Temperature sensor at 1KHz sampling
       // rate
#define MPU9150_BANDWIDTH_42                                                   \
  0x03 // 42 Hz Bandwidth of Rate-Gyro and Temperature sensor at 1KHz sampling
       // rate
#define MPU9150_BANDWIDTH_98                                                   \
  0x02 // 98 Hz Bandwidth of Rate-Gyro and Temperature sensor at 1KHz sampling
       // rate
#define MPU9150_BANDWIDTH_188                                                  \
  0x01 // 188 Hz Bandwidth of Rate-Gyro and Temperature sensor at 1KHz sampling
       // rate
#define MPU9150_BANDWIDTH_256                                                  \
  0x00 // 256 Hz Bandwidth of Rate-Gyro and Temperature sensor at 8KHz sampling
       // rate (need an external crystal for this)

#define MPU9150_GYRO_RANGE_250 0x00  // ±250 deg/s
#define MPU9150_GYRO_RANGE_500 0x08  // ±500 deg/s
#define MPU9150_GYRO_RANGE_1000 0x10 // ±1000 deg/s
#define MPU9150_GYRO_RANGE_2000 0x18 // ±2000 deg/s

#define MPU9150_ACCL_RANGE_2 0x00  // ±2 g
#define MPU9150_ACCL_RANGE_4 0x08  // ±4 g
#define MPU9150_ACCL_RANGE_8 0x10  // ±8 g
#define MPU9150_ACCL_RANGE_16 0x18 // ±16 g
/**
 * @}
 */

/** @defgroup MPU9150_Exported_Functions
 * @{
 */
void MPU9150_Init(I2C_TypeDef *I2Cx, uint8_t bandwidth, uint8_t gyro_range,
                  uint8_t accl_range);

void MPU9150_ReadSensor(void);
void AK8975_ReadSensor(void);

uint8_t MPU9150_ReadSensorIT(void);
uint8_t AK8975_ReadSensorIT(void);

float MPU9150_GetTemp(void); // [deg C]

float MPU9150_GetGyroX(void); // [rad/s]
float MPU9150_GetGyroY(void); // [rad/s]
float MPU9150_GetGyroZ(void); // [rad/s]

float MPU9150_GetAcclX(void); // [g]
float MPU9150_GetAcclY(void); // [g]
float MPU9150_GetAcclZ(void); // [g]

float MPU9150_GetMagnX(void); // [uT]
float MPU9150_GetMagnY(void); // [uT]
float MPU9150_GetMagnZ(void); // [uT]
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __MPU9150_H */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
