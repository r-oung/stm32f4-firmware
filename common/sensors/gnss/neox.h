/**
 * @file    neox.h
 * @author  Raymond Oung
 * @date    2013.10.29
 * @brief   NEOx global navigation satellite system (GNSS) functions
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
#ifndef __NEOx_H
#define __NEOx_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "ubx.h"

/** @addtogroup Source
  * @{
  */ 

/** @addtogroup Low_Level
  * @{
  */ 

/** @addtogroup NEOx
  * @{
  */ 

enum {
  NEOx_SBAS_N_AMERICA = 0, 
  NEOx_SBAS_EUROPE, 
  NEOx_SBAS_ASIA
};

enum {
  NEOx_PORTABLE = 0, // Max. Altitude: 12000 m, Max. Horz. Vel. 310 m/s, Max. Vert. Vel. 50 m/s
  NEOx_STATIONARY, // Max. Altitude: 9000 m, Max. Horz. Vel. 10 m/s, Max. Vert. Vel. 6 m/s
  NEOx_PEDESTRIAN, // Max. Altitude: 9000 m, Max. Horz. Vel. 30 m/s, Max. Vert. Vel. 20 m/s
  NEOx_AUTOMOTIVE, // Max. Altitude: 6000 m, Max. Horz. Vel. 84 m/s, Max. Vert. Vel. 15 m/s
  NEOx_SEA, // Max. Altitude: 500 m, Max. Horz. Vel. 25 m/s, Max. Vert. Vel. 5 m/s
  NEOx_AIRBORNE_1G, // Max. Altitude: 50000 m, Max. Horz. Vel. 100 m/s, Max. Vert. Vel. 100 m/s
  NEOx_AIRBORNE_2G, // Max. Altitude: 50000 m, Max. Horz. Vel. 250 m/s, Max. Vert. Vel. 100 m/s
  NEOx_AIRBORNE_4G // Max. Altitude: 50000 m, Max. Horz. Vel. 500 m/s, Max. Vert. Vel. 100 m/s
};

enum {
  NEOx_PPP_DISABLE = 0u,
  NEOx_PPP_ENABLE = 1u
};

/** @defgroup NEOx_Exported_Functions
  * @{
  */ 
uint8_t NEOx_Init(uint8_t p, uint16_t measPeriod, uint8_t sbas, uint8_t dynMode);
void NEOx_ISR_IT(uint8_t byte);
void NEOx_ISR_DMA(void *buffer);

uint8_t NEOx_SetDynMode(uint8_t dynMode);

__inline float NEOx_GetEcefPX(void); // ECEF X coordinate [m]
__inline float NEOx_GetEcefPY(void); // ECEF Y coordinate [m]
__inline float NEOx_GetEcefPZ(void); // ECEF Z coordinate [m]
__inline float NEOx_GetEcefPAcc(void); // position accuracy estimate [m]

__inline float NEOx_GetEcefVX(void); // ECEF X velocity [m/s]
__inline float NEOx_GetEcefVY(void); // ECEF Y velocity [m/s]
__inline float NEOx_GetEcefVZ(void); // ECEF Z velocity [m/s]
__inline float NEOx_GetEcefVAcc(void); // velocity accuracy estimate [m/s]

__inline float NEOx_GetLon(void); // longitude, scaling 1e-7 [deg]
__inline float NEOx_GetLat(void); // latitude, scaling 1e-7 [deg]
__inline float NEOx_GetMSL(void); // height above mean sea level [m]
__inline float NEOx_GetHAcc(void); // horizontal accuracy estimate [m]
__inline float NEOx_GetVAcc(void); // vertical accuracy estimate [m]

__inline float NEOx_GetVelN(void); // north velocity component [m/s]
__inline float NEOx_GetVelE(void); // east velocity component [m/s]
__inline float NEOx_GetVelD(void); // down velocity component [m/s]

__inline float NEOx_GetHeading(void); // heading of motion (2D), scaling 1e-5 [deg]

__inline float NEOx_GetGDOP(void); // geometric DOP
__inline float NEOx_GetPDOP(void); // position DOP
__inline float NEOx_GetTDOP(void); // time DOP
__inline float NEOx_GetVDOP(void); // vertical DOP
__inline float NEOx_GetHDOP(void); // horizontal DOP
__inline float NEOx_GetNDOP(void); // northing DOP
__inline float NEOx_GetEDOP(void); // easting DOP

__inline uint8_t NEOx_GetSV(void); // number of satellites used in navigation solution

/**
  * @}
  */ 
    
#ifdef __cplusplus
}
#endif
  
#endif /* __NEOx_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 
