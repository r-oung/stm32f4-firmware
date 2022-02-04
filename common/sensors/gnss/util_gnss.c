/**
 * @file    util_gnss.c
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

/* Includes ------------------------------------------------------------------*/
#include "util_gnss.h"
#include <math.h>

/** @addtogroup Source
 * @{
 */

/** @addtogroup Utilities
 * @{
 */

/** @defgroup UTIL_GNSS
 * @brief This file contains utility functions for GNSS data.
 * @{
 */

/** @defgroup UTIL_GNSS_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @defgroup UTIL_GNSS_Private_Defines
 * @{
 */
/**
 * @}
 */

/** @defgroup UTIL_GNSS_Private_Macros
 * @{
 */
/**
 * @}
 */

/** @defgroup UTIL_GNSS_Private_Variables
 * @{
 */
/**
 * @}
 */

/** @defgroup UTIL_GNSS_Private_Functions
 * @{
 */
/**
 * @}
 */

/**
 * @brief Transform coordinates of a point Q wrt to P in ECEF system
 * 	     to coordinates of a point Q' in ENU system, Q' = M^T*(Q-P), where M
 * is a transformation matrix
 * @reference: http://www.nosco.ch/mathematics/notes/earth-coordinates.php
 * @param ecef_x Coordinate of a point Q in ECEF x-coordinate frame [m]
 * @param ecef_y Coordinate of a point Q in ECEF y-coordinate frame [m]
 * @param ecef_z Coordinate of a point Q in ECEF z-coordinate frame [m]
 * @param p_x    Point of origin in ECEF x-coordinate frame [m]
 * @param p_y    Point of origin in ECEF y-coordinate frame [m]
 * @param p_z    Point of origin in ECEF z-coordinate frame [m]
 * @param enu_e  Coordinate of a point Q in ENU East-coordinate frame [m]
 * @param enu_n  Coordinate of a point Q in ENU North-coordinate frame [m]
 * @param enu_u  Coordinate of a point Q in ENU Up-coordinate frame [m]
 * @retval None
 */
void ecef2enu_p(float ecef_x, float ecef_y, float ecef_z, float p_x, float p_y,
                float p_z, float *enu_e, float *enu_n, float *enu_u) {
  float r = 0.0f;
  float phi = 0.0f;
  float lambda = 0.0f;

  // @TODO This does not need to be computed at each instance
  // obtain spherical coordinates of P (r,phi,lambda)
  r = sqrtf(p_x * p_x + p_y * p_y + p_z * p_z); // [m]
  phi = asinf(p_z / r);                         // lattitude [-pi/2, +pi/2]
  lambda = atan2f(p_y, p_x);                    // longitude (-pi, pi]

  // pre-compute some parameters
  float cosphi = cosf(phi);
  float sinphi = sinf(phi);
  float coslambda = cosf(lambda);
  float sinlambda = sinf(lambda);

  // change in position
  float dx = ecef_x - p_x; // [m]
  float dy = ecef_y - p_y; // [m]
  float dz = ecef_z - p_z; // [m]

  // transformation from ECEF to ENU
  *enu_e = -dx * sinlambda + dy * coslambda; // [m]
  *enu_n =
      -dx * sinphi * coslambda - dy * sinphi * sinlambda + dz * cosphi; // [m]
  *enu_u =
      +dx * cosphi * coslambda + dy * cosphi * sinlambda + dz * sinphi; // [m]
}

/**
 * @brief Transform velocity of a point Q at P in ECEF system
 * 	     to velocity of a point Q' at P in ENU system, Q' = M^T*(Q-P), where
 * M is a transformation matrix
 * @reference: http://www.nosco.ch/mathematics/notes/earth-coordinates.php
 * @param ecef_dx Velocity of a point Q in ECEF x-coordinate frame [m/s]
 * @param ecef_dy Velocity of a point Q in ECEF y-coordinate frame [m/s]
 * @param ecef_dz Velocity of a point Q in ECEF z-coordinate frame [m/s]
 * @param p_x     Position of a point Q in ECEF x-coordinate frame [m]
 * @param p_y     Position of a point Q ECEF y-coordinate frame [m]
 * @param p_z     Position of a point Q ECEF z-coordinate frame [m]
 * @param enu_de  Velocity of a point Q in ENU East-coordinate frame [m/s]
 * @param enu_dn  Velocity of a point Q in ENU North-coordinate frame [m/s]
 * @param enu_du  Velocity of a point Q in ENU Up-coordinate frame [m/s]
 * @retval None
 */
void ecef2enu_v(float ecef_dx, float ecef_dy, float ecef_dz, float p_x,
                float p_y, float p_z, float *enu_de, float *enu_dn,
                float *enu_du) {
  // obtain spherical coordinates of P (r,phi,lambda)
  float r = sqrtf(p_x * p_x + p_y * p_y + p_z * p_z); // [m]
  float phi = asinf(p_z / r);      // lattitude [-pi/2, +pi/2]
  float lambda = atan2f(p_y, p_x); // longitude (-pi, pi]

  // pre-compute some parameters
  float cosphi = cosf(phi);
  float sinphi = sinf(phi);
  float coslambda = cosf(lambda);
  float sinlambda = sinf(lambda);

  // transformation from ECEF to ENU
  *enu_de = -ecef_dx * sinlambda + ecef_dy * coslambda; // [m]
  *enu_dn = -ecef_dx * sinphi * coslambda - ecef_dy * sinphi * sinlambda +
            ecef_dz * cosphi; // [m]
  *enu_du = +ecef_dx * cosphi * coslambda + ecef_dy * cosphi * sinlambda +
            ecef_dz * sinphi; // [m]
}

/**
 * @brief Transform coordinates of a point Q' wrt P in ENU system
 *        to coordinates of a point Q in ECEF system, Q = MQ' + P, where M is a
 * transformation matrix
 * @reference: http://www.nosco.ch/mathematics/notes/earth-coordinates.php
 * @param enu_e  Coordinate of a point Q in ENU East-coordinate frame [m]
 * @param enu_n  Coordinate of a point Q in ENU North-coordinate frame [m]
 * @param enu_u  Coordinate of a point Q in ENU Up-coordinate frame [m]
 * @param p_x    Point of origin in ECEF x-coordinate frame [m]
 * @param p_y    Point of origin in ECEF y-coordinate frame [m]
 * @param p_z    Point of origin in ECEF z-coordinate frame [m]
 * @param ecef_x Coordinate of a point Q in ECEF x-coordinate frame [m]
 * @param ecef_y Coordinate of a point Q in ECEF y-coordinate frame [m]
 * @param ecef_z Coordinate of a point Q in ECEF z-coordinate frame [m]
 * @retval None
 */
void enu2ecef_p(float enu_e, float enu_n, float enu_u, float p_x, float p_y,
                float p_z, float *ecef_x, float *ecef_y, float *ecef_z) {
  // obtain spherical coordinates of P (r,phi,lambda)
  float r = sqrtf(p_x * p_x + p_y * p_y + p_z * p_z); // [m]
  float phi = asinf(p_z / r);      // lattitude [-pi/2, +pi/2]
  float lambda = atan2f(p_y, p_x); // longitude (-pi, pi]

  // pre-compute some parameters
  float cosphi = cosf(phi);
  float sinphi = sinf(phi);
  float coslambda = cosf(lambda);
  float sinlambda = sinf(lambda);

  // transformation from ENU to ECEF
  // rotation
  *ecef_x = -enu_e * sinlambda - enu_n * sinphi * coslambda +
            enu_u * cosphi * coslambda; // [m]
  *ecef_y = enu_e * coslambda - enu_n * sinphi * sinlambda +
            enu_u * cosphi * sinlambda;      // [m]
  *ecef_z = enu_n * cosphi + enu_u * sinphi; // [m]

  // translation
  *ecef_x += p_x; // [m]
  *ecef_y += p_y; // [m]
  *ecef_z += p_z; // [m]
}

/**
 * @brief Transform ECEF to latitude, longitude, and altitude (ASL)
 * @param x  Coordinate of a point Q in ECEF x-coordinate frame [m]
 * @param y  Coordinate of a point Q in ECEF y-coordinate frame [m]
 * @param z  Coordinate of a point Q in ECEF z-coordinate frame [m]
 * @param lat    Latitude
 * @param lon    Longitude
 * @param alt    Altitude above sea level (ASL)
 * @param ecef_x Coordinate of a point Q in ECEF x-coordinate frame [m]
 * @param ecef_y Coordinate of a point Q in ECEF y-coordinate frame [m]
 * @param ecef_z Coordinate of a point Q in ECEF z-coordinate frame [m]
 * @retval None
 */
void ecef2lla(float x, float y, float z, float *lat, float *lon, float *alt) {
  /* Constants (WGS ellipsoid) */
  const float a = 6378137.0f;
  const float e = 8.1819190842622e-2f;

  float b, ep, p, th, n;

  /* Calculation */
  b = sqrtf(powf(a, 2) * (1 - powf(e, 2)));
  ep = sqrtf((powf(a, 2) - powf(b, 2)) / powf(b, 2));
  p = sqrtf(powf(x, 2) + powf(y, 2));
  th = atan2f(a * z, b * p);
  *lon = atan2f(y, x);
  *lat = atan2f((z + ep * ep * b * powf(sinf(th), 3)),
                (p - e * e * a * powf(cosf(th), 3)));
  n = a / sqrtf(1 - e * e * powf(sinf(*lat), 2));
  *alt = p / cosf(*lat) - n;
  *lat = *lat * 180.0f / M_PI;
  *lon = *lon * 180.0f / M_PI;
}
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */