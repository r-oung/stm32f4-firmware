/**
 * @file    ms5611.c
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

/* Includes ------------------------------------------------------------------*/
#include "ms5611.h"
#include "clock.h"

#ifdef _VERBOSE
#include "uart.h"
#include <stdio.h>
#include <stdlib.h>
#endif

/** @addtogroup Source
 * @{
 */

/** @addtogroup MS5611
 * @{
 */

/** @defgroup MS5611
 * @brief Functions that handle the MS5611 pressure sensor
 * @{
 */

/** @defgroup MS5611_Private_Defines
 * @{
 */
#ifdef _VERBOSE
#define VERBOSE_portID 1u
#endif

#define MS5611_ADDR 0xEF // b1110 111x
#define MS5611_RESET 0x1E

// PROM coefficients
#define MS5611_PROM_FACTORY 0xA0
#define MS5611_PROM_COEF_1H 0xA2
#define MS5611_PROM_COEF_2H 0xA4
#define MS5611_PROM_COEF_3H 0xA6
#define MS5611_PROM_COEF_4H 0xA8
#define MS5611_PROM_COEF_5H 0xAA
#define MS5611_PROM_COEF_6H 0xAC
#define MS5611_PROM_CRCH 0xAE

// pressure
#define MS5611_PRES_OSR_256 0x40
#define MS5611_PRES_OSR_512 0x42
#define MS5611_PRES_OSR_1024 0x44
#define MS5611_PRES_OSR_2048 0x46
#define MS5611_PRES_OSR_4096 0x48

// temperature
#define MS5611_TEMP_OSR_256 0x50
#define MS5611_TEMP_OSR_512 0x52
#define MS5611_TEMP_OSR_1024 0x54
#define MS5611_TEMP_OSR_2048 0x56
#define MS5611_TEMP_OSR_4096 0x58

#define MS5611_READ_ADC 0x00
/**
 * @}
 */

/** @defgroup MS5611_Private_Variables
 * @{
 */
static volatile uint8_t MS5611_init =
    0u; // MS5611 initialisation flag (0=Failure, 1=Success)
static I2C_TypeDef *MS5611_I2C;
static volatile uint8_t MS5611_PRES = MS5611_PRES_OSR_4096;
static volatile uint8_t MS5611_TEMP = MS5611_TEMP_OSR_4096;
static uint16_t MS5611_C[8]; // PROM coefficients and CRC
static volatile float MS5611_presConversionPeriod = 0.0f; // [msec]
static volatile float MS5611_tempConversionPeriod = 0.0f; // [msec]
static volatile float MS5611_conversionPeriod = 0.0f;     // [msec]
static volatile struct MS5611_data_t {
  float tPres;      // pressure sample timestamp [msec]
  float tTemp;      // temperature sample timestamp [msec]
  uint32_t rawPres; // raw pressure
  uint32_t rawTemp; // raw temperature
} MS5611_data;
static volatile float MS5611_pressure[2];    // circular buffer
static volatile float MS5611_temperature[2]; // circular buffer
static volatile uint8_t MS5611_rw = 0u;      // read/write buffer index
/**
 * @}
 */

/** @defgroup MS5611_Private_Functions
 * @{
 */
uint16_t MS5611_CRC4(uint16_t *prom);
void MS5611_TxByte(uint8_t cmd);
void MS5611_RxData(uint8_t cmd, void *data, uint8_t num);
void MS5611_CalcPressureTemp(void);
/**
 * @}
 */

/**
 * @brief  Initialises MS5611
 * @param  I2Cx  I2C perihperal type [I2C1..I2C2]
 * @param  pres_OSR Pressure over-sampling ratio
 * @param  temp_OSR Temperature over-sampling ratio
 * @retval 0=Failure, 1=Success
 */
uint8_t MS5611_Init(I2C_TypeDef *I2Cx, uint8_t pres_OSR, uint8_t temp_OSR) {
  uint8_t retval = 1u;

  MS5611_I2C = I2Cx;
  MS5611_PRES = pres_OSR;
  MS5611_TEMP = temp_OSR;

  switch (pres_OSR) {
  case MS5611_PRES_OSR_256:
    MS5611_presConversionPeriod = 0.6f; // [msec]
    break;
  case MS5611_PRES_OSR_512:
    MS5611_presConversionPeriod = 1.17f; // [msec]
    break;
  case MS5611_PRES_OSR_1024:
    MS5611_presConversionPeriod = 2.28f; // [msec]
    break;
  case MS5611_PRES_OSR_2048:
    MS5611_presConversionPeriod = 4.54f; // [msec]
    break;
  case MS5611_PRES_OSR_4096:
    MS5611_presConversionPeriod = 9.04f; // [msec]
    break;
  default:
    break;
  }

  switch (temp_OSR) {
  case MS5611_TEMP_OSR_256:
    MS5611_tempConversionPeriod = 0.6f; // [msec]
    break;
  case MS5611_TEMP_OSR_512:
    MS5611_tempConversionPeriod = 1.17f; // [msec]
    break;
  case MS5611_TEMP_OSR_1024:
    MS5611_tempConversionPeriod = 2.28f; // [msec]
    break;
  case MS5611_TEMP_OSR_2048:
    MS5611_tempConversionPeriod = 4.54f; // [msec]
    break;
  case MS5611_TEMP_OSR_4096:
    MS5611_tempConversionPeriod = 9.04f; // [msec]
    break;
  default:
    break;
  }

  // Reset
  // Sent once after power-on to make sure that the calibration PROM gets loaded
  // into the internal register it can also be used to reset the device ROM from
  // an unknown condition
  MS5611_Reset();
  Delay(5);

  // Read PROM
  uint8_t tmp[2];
  MS5611_RxData(MS5611_PROM_FACTORY, tmp, 2);
  MS5611_C[0] =
      (((uint16_t)tmp[0]) << 8u) | tmp[1]; // 16-bit reserved for manufacturer
  MS5611_RxData(MS5611_PROM_COEF_1H, tmp, 2);
  MS5611_C[1] =
      (((uint16_t)tmp[0]) << 8u) | tmp[1]; // pressure sensitivity (SENS)
  MS5611_RxData(MS5611_PROM_COEF_2H, tmp, 2);
  MS5611_C[2] = (((uint16_t)tmp[0]) << 8u) | tmp[1]; // pressure offset (OFF)
  MS5611_RxData(MS5611_PROM_COEF_3H, tmp, 2);
  MS5611_C[3] = (((uint16_t)tmp[0]) << 8u) |
                tmp[1]; // temperature coefficient of pressure sensitivity (TCS)
  MS5611_RxData(MS5611_PROM_COEF_4H, tmp, 2);
  MS5611_C[4] = (((uint16_t)tmp[0]) << 8u) |
                tmp[1]; // temperature coefficient of pressure offset (TCO)
  MS5611_RxData(MS5611_PROM_COEF_5H, tmp, 2);
  MS5611_C[5] =
      (((uint16_t)tmp[0]) << 8u) | tmp[1]; // reference temperature (Tref)
  MS5611_RxData(MS5611_PROM_COEF_6H, tmp, 2);
  MS5611_C[6] = (((uint16_t)tmp[0]) << 8u) |
                tmp[1]; // temperature coefficient of the temperature (TEMPSENS)
  MS5611_RxData(MS5611_PROM_CRCH, tmp, 2);
  MS5611_C[7] = (((uint16_t)tmp[0]) << 8u) | tmp[1]; // 4-bit CRC

  if (MS5611_CRC4(MS5611_C) != (0x000F & MS5611_C[7])) {
#ifdef _VERBOSE
    UART_Print(VERBOSE_portID, "ERROR: MS5611 failed CRC check\r\n");
#endif
    retval = 0u;
  }

  // initiate uncompensated pressure conversion (rawPres)
  MS5611_TxByte(MS5611_PRES);

  MS5611_init = retval;
  return retval;
}

/**
 * @brief Calculate MS5611 4-bitCRC code
 * @note uint16_t n_prom[] =
 * {0x3132,0x3334,0x3536,0x3738,0x3940,0x4142,0x4344,0x4500}; resulting CRC
 * should be 0x000B
 * @param None
 */
uint16_t MS5611_CRC4(uint16_t *prom) {
  uint8_t cnt;
  uint16_t n_rem;
  uint16_t crc_read;
  uint8_t n_bit;

  n_rem = 0x00;
  crc_read = prom[7];
  prom[7] = 0xFF00 & prom[7];

  for (cnt = 0u; cnt < 16u; cnt++) {
    if (cnt % 2 == 1u) {
      n_rem ^= (uint16_t)(prom[cnt >> 1u] & 0x00FF);
    } else {
      n_rem ^= (uint16_t)(prom[cnt >> 1u] >> 8u);
    }

    for (n_bit = 8u; n_bit > 0u; n_bit--) {
      if (n_rem & 0x8000) {
        n_rem = (n_rem << 1u) ^ 0x3000;
      } else {
        n_rem = (n_rem << 1u);
      }
    }
  }

  n_rem = 0x000F & (n_rem >> 12u);
  prom[7] = crc_read;

  return n_rem ^ 0x0;
}

/**
 * @brief Reset MS5611
 * @param None
 */
void MS5611_Reset(void) {
  MS5611_data.tPres = 0.0f;
  MS5611_data.rawPres = 0u;
  MS5611_data.tTemp = 0.0f;
  MS5611_data.rawTemp = 0u;

  MS5611_TxByte(MS5611_RESET);
}
/**
 * @brief  Send a byte of data to MS5611
 * @param  cmd Command
 * @retval None
 */
void MS5611_TxByte(uint8_t cmd) {
  // Send START condition
  I2C_GenerateSTART(MS5611_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(MS5611_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send Slave MS5611 Address
  I2C_Send7bitAddress(MS5611_I2C, MS5611_ADDR, I2C_Direction_Transmitter);

  // Test on EV6 and clear it
  while (
      !I2C_CheckEvent(MS5611_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    ;

  // Send MS5611 command
  I2C_SendData(MS5611_I2C, cmd);

  // Test on EV8 and clear it
  while (!I2C_CheckEvent(MS5611_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  // Send STOP Condition
  I2C_GenerateSTOP(MS5611_I2C, ENABLE);
  while (I2C_GetFlagStatus(MS5611_I2C, I2C_FLAG_STOPF))
    ;
  while (I2C_GetFlagStatus(MS5611_I2C, I2C_FLAG_BUSY))
    ;
}

/**
 * @brief  Read data from MS5611
 * @param  cmd Command
 * @param  data Pointer to data to save the data
 * @param  num Number of bytes to read
 * @retval None
 */
void MS5611_RxData(uint8_t cmd, void *data, uint8_t num) {
  uint8_t i;

  // Send START condition
  I2C_GenerateSTART(MS5611_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(MS5611_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send slave MS5611 address for writing
  I2C_Send7bitAddress(MS5611_I2C, MS5611_ADDR, I2C_Direction_Transmitter);

  // Test on EV6 and clear it
  while (
      !I2C_CheckEvent(MS5611_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    ;

  // Send MS5611 command
  I2C_SendData(MS5611_I2C, cmd);

  // Test on EV8 and clear it
  while (!I2C_CheckEvent(MS5611_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  // Send START condition a second time
  I2C_GenerateSTART(MS5611_I2C, ENABLE);

  // Test on EV5 and clear it
  while (!I2C_CheckEvent(MS5611_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  // Send slave MS5611 address for reading
  I2C_Send7bitAddress(MS5611_I2C, MS5611_ADDR, I2C_Direction_Receiver);

  // Test on EV6 and clear it
  while (!I2C_CheckEvent(MS5611_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    ;

  // Read N bytes
  for (i = 0; i < num - 1; i++) {
    // Test on EV7 and clear it
    while (!I2C_CheckEvent(MS5611_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
      ;

    // Read a byte
    *((uint8_t *)data + i) = I2C_ReceiveData(MS5611_I2C);
  }

  // Disable Acknowledgement on the last byte
  I2C_AcknowledgeConfig(MS5611_I2C, DISABLE);

  // Test on EV7 and clear it
  while (!I2C_CheckEvent(MS5611_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
    ;

  // Read a byte
  *((uint8_t *)data + i) = I2C_ReceiveData(MS5611_I2C);

  // Send STOP Condition
  I2C_GenerateSTOP(MS5611_I2C, ENABLE);
  while (I2C_GetFlagStatus(MS5611_I2C, I2C_FLAG_STOPF))
    ;
  while (I2C_GetFlagStatus(MS5611_I2C, I2C_FLAG_BUSY))
    ;

  // Enable Acknowledgement to be ready for another reception
  I2C_AcknowledgeConfig(MS5611_I2C, ENABLE);
}

/**
 * @brief  MS5611 Interrupt service routine (Blocking)
 * @retval None
 */
void MS5611_ReadSensor(void) {
  if (!MS5611_init)
    return;

  static uint8_t data[3];
  static uint8_t toggle = 1u;
  static float t0 = 0.0f;
  static float period = 0.0f; // [msec]
  static float tPres = 0.0f;  // [msec]
  static float tTemp = 0.0f;  // [msec]

  if (GetTimeU() - t0 < period)
    return;

  // Get MS5611 data
  MS5611_RxData(MS5611_READ_ADC, data, 3u);

  // send conversion command
  // @TODO Don't toggle because temperature dynamics is much slower than
  // pressure
  if (toggle) {
    toggle = 0u;
    period = MS5611_tempConversionPeriod;

    // read uncompensated pressure conversion (rawPres)
    MS5611_data.tPres = tPres; // pressure measurement timestamp [msec]
    MS5611_data.rawPres =
        (uint32_t)data[0] << 16u | (uint32_t)data[1] << 8u | (uint32_t)data[2];

    // initiate uncompensated temperature conversion (rawTemp)
    MS5611_TxByte(MS5611_TEMP);
    tPres = GetTimeU();
  } else {
    toggle = 1u;
    period = MS5611_presConversionPeriod;

    // read uncompensated temperature conversion (rawTemp)
    MS5611_data.tTemp = tTemp; // temperature measurement timestamp [msec]
    MS5611_data.rawTemp =
        (uint32_t)data[0] << 16u | (uint32_t)data[1] << 8u | (uint32_t)data[2];

    // initiate uncompensated pressure conversion (rawPres)
    MS5611_TxByte(MS5611_PRES);
    tTemp = GetTimeU();
  }

  // calculate compensated pressure and temperature
  MS5611_CalcPressureTemp();

  t0 = GetTimeU();
}

/**
 * @brief  MS5611 Interrupt service routine (Non-Blocking)
 * @param  None
 * @retval None
 */
uint8_t MS5611_ReadSensorIT(void) {
  static uint8_t direction = I2C_Direction_Transmitter; // I2C direction
  static uint8_t i = 0u;                                // byte counter
  static uint8_t data[3]; // read a total of 3 bytes

  static uint8_t toggle_pt =
      0u; // toggle between pressure and temperature sensor measurements
  static uint8_t toggle_tr =
      0u; // switch between transmitting command and receiving data

  static float tPres = 0.0f; // [msec]
  static float tTemp = 0.0f; // [msec]

  if (toggle_tr) // transmit command
  {
    switch (I2C_GetLastEvent(MS5611_I2C)) {
    // Test on EV5 and clear it
    case I2C_EVENT_MASTER_MODE_SELECT: {
      // Send Slave MS5611 Address
      I2C_Send7bitAddress(MS5611_I2C, MS5611_ADDR, I2C_Direction_Transmitter);
    } break;

    // Test on EV6 and clear it
    case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED: {
      // Send command
      if (toggle_pt) {
        toggle_pt = 0u;
        I2C_SendData(MS5611_I2C, MS5611_TEMP);
        tPres = GetTimeU();
      } else {
        toggle_pt = 1u;
        I2C_SendData(MS5611_I2C, MS5611_PRES);
        tTemp = GetTimeU();
      }
    } break;

    // Test on EV8 and clear it
    case I2C_EVENT_MASTER_BYTE_TRANSMITTED: {
      toggle_tr = 0u; // reset

      // Send STOP Condition
      I2C_GenerateSTOP(MS5611_I2C, ENABLE);
      while (I2C_GetFlagStatus(MS5611_I2C, I2C_FLAG_STOPF))
        ;
      while (I2C_GetFlagStatus(MS5611_I2C, I2C_FLAG_BUSY))
        ;
      return 1u;
    } break;

    default:
      break;
    }
  } else // receive data
  {
    switch (I2C_GetLastEvent(MS5611_I2C)) {
    // Test on EV5 and clear it
    case I2C_EVENT_MASTER_MODE_SELECT:
      // Send Slave MS5611 Address
      I2C_Send7bitAddress(MS5611_I2C, MS5611_ADDR, direction);
      break;

    // Test on EV6 and clear it
    case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
      // Send the internal MS5611 address to read from
      I2C_SendData(MS5611_I2C, MS5611_READ_ADC);
      break;

    case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
      I2C_ITConfig(MS5611_I2C, I2C_IT_BUF, ENABLE);
      // Send START condition a second time
      I2C_GenerateSTART(MS5611_I2C, ENABLE);
      direction = I2C_Direction_Receiver;
      break;

    case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
      break;

    // Test on EV7 and clear it
    case I2C_EVENT_MASTER_BYTE_RECEIVED:
      // Read a byte
      data[i++] = I2C_ReceiveData(MS5611_I2C);

      // Disable Acknowledgement on the last byte
      if (i == 2u) {
        // Disable Acknowledgement on the last byte
        I2C_AcknowledgeConfig(MS5611_I2C, DISABLE);
      }

      if (i == 3u) {
        // Send STOP Condition
        I2C_GenerateSTOP(MS5611_I2C, ENABLE);
        while (I2C_GetFlagStatus(MS5611_I2C, I2C_FLAG_STOPF))
          ;
        while (I2C_GetFlagStatus(MS5611_I2C, I2C_FLAG_BUSY))
          ;

        // reset
        I2C_AcknowledgeConfig(MS5611_I2C, ENABLE);
        direction = I2C_Direction_Transmitter;
        i = 0u;

        // send conversion command
        if (toggle_pt) {
          MS5611_conversionPeriod = MS5611_tempConversionPeriod;

          // read uncompensated pressure conversion (rawPres)
          MS5611_data.tPres = tPres; // pressure measurement timestamp [msec]
          MS5611_data.rawPres = (uint32_t)data[0] << 16u |
                                (uint32_t)data[1] << 8u | (uint32_t)data[2];
        } else {
          MS5611_conversionPeriod = MS5611_presConversionPeriod;

          // read uncompensated temperature conversion (rawTemp)
          MS5611_data.tTemp = tTemp; // temperature measurement timestamp [msec]
          MS5611_data.rawTemp = (uint32_t)data[0] << 16u |
                                (uint32_t)data[1] << 8u | (uint32_t)data[2];
        }

        // calculate compensated pressure and temperature only if raw data is
        // valid
        if (MS5611_data.rawPres == 0u && MS5611_data.rawTemp == 0u) {
          MS5611_pressure[MS5611_rw] = 0u;
          MS5611_temperature[MS5611_rw] = 0u;
        } else {
          MS5611_CalcPressureTemp();
        }

        I2C_GenerateSTART(MS5611_I2C, ENABLE);
        toggle_tr = 1u;
      }
      break;

    default:
      break;
    }
  }

  return 0u;
}

/**
 * @brief  Calculate compensated pressure and temperature
 * @param  pres  Pressure [Pa]
 * @param  temp  Temperature [deg C]
 * @retval
 */
void MS5611_CalcPressureTemp(void) {
  int32_t dT = MS5611_data.rawTemp - ((int32_t)MS5611_C[5] << 8u);
  int32_t TEMP = 2000u + ((dT * (int32_t)MS5611_C[6]) >> 23u);
  int64_t OFF = ((int64_t)MS5611_C[2] << 16u) +
                (((int64_t)MS5611_C[4] * (int64_t)dT) >> 7u);
  int64_t SENS = ((int64_t)MS5611_C[1] << 15u) +
                 (((int64_t)MS5611_C[3] * (int64_t)dT) >> 8u);

  // second order temperature compensation
  int32_t T2 = 0u;
  int64_t OFF2 = 0u;
  int64_t SENS2 = 0u;
  if (TEMP < 2000u) // less than 20 deg C
  {
    T2 = (dT * dT) >> 31u;
    OFF2 = (5 * (TEMP - 2000u) * (TEMP - 2000u)) >> 1u;
    SENS2 = OFF2 >> 1u;

    if (TEMP < -1500u) {
      int32_t tmp = TEMP + 1500u;
      int64_t tmpSqrd = tmp * tmp;
      OFF2 += 7u * tmpSqrd;
      SENS2 += (11u * tmpSqrd) >> 1u;
    }
  }
  TEMP -= T2;
  OFF -= OFF2;
  SENS -= SENS2;

  MS5611_pressure[MS5611_rw] =
      (float)(((((int64_t)MS5611_data.rawPres * SENS) >> 21u) - OFF) >>
              15u);                                       // [0.01 mbar / Pa]
  MS5611_temperature[MS5611_rw] = (float)(TEMP / 100.0f); // [deg C]
  MS5611_rw = !MS5611_rw; // toggle read/write buffer index
}

float MS5611_GetPressure(void) {
  return MS5611_pressure[!MS5611_rw];
} // [Pa]
float MS5611_GetTemperature(void) {
  return MS5611_temperature[!MS5611_rw];
} // [deg C]
float MS5611_GetConversionPeriod(void) {
  return MS5611_conversionPeriod;
} // [msec]
/**
 * @}
 */

/**
 * @}
 */
