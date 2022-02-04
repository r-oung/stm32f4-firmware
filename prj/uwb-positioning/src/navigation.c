/**
 * @file    navigation.c
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

#include <math.h>

#include "clock.h"
#include "navigation.h"
#include "util_gnss.h"

#define BITVAL(reg, bit) ((reg >> bit) & 0x01)
#define BITSET(reg, bit) (reg |= 0x01 << bit)
#define BITCLR(reg, bit) (reg &= ~(0x01 << bit))
#define BITTOG(reg, bit) (reg ^= 0x01 << bit)

static const float CONST_G = 9.80665f;  // acceleration due to gravity [m/s/s]
static const float CONST_Tb = 288.15f;  // standard temperature [K]
static const float CONST_Lb = -0.0065f; // standard temperature lapse rate [K/m]
static const float CONST_Pb = 101325.0f; // static pressure at sea level [Pa]
static const float CONST_Rb = 8.31432f;  // [N.m/mol/K]
static const float CONST_M = 0.0289644f; // [kg/mol]

enum {
  NAV_CALIB_GYRO = 0u,
  NAV_CALIB_ACCL,
  NAV_CALIB_MAGN,
  NAV_CALIB_BARO,
  NAV_CALIB_ECEFp,
  NAV_CALIB_ECEFv,
  NAV_CALIB_NEDv
};
static volatile uint16_t NAV_calibStatus = 0u;
static volatile uint32_t NAV_calibTime = 0u; // [msec]

typedef struct {
  float xyz[3];
  float gba[3];
  float dxyz[3];
  float dgba[3];
  float ddxyz[3];
  float bias_dxyz[3];
  float bias_ddxyz[3];
  float bias_dgba[3];
  float covar_xyz[2][2][3];
  float covar_dxyz[2][2][3];
  float covar_gba[2][2][3];
  float K_xyz[2][3];
  float K_dxyz[2][3];
  float K_gba[2][3];
} NAV_kfState_t;
static NAV_kfState_t NAV_kfState;

typedef struct {
  float xyz[3];
  float dxyz[3];
  float gba[3];
  float dgba[3];
  float bias_dxyz[3];
  float bias_ddxyz[3];
  float bias_dgba[3];
} NAV_state_t;
static NAV_state_t NAV_state;

typedef struct {
  float xyz[3];
  float dxyz[3];
  float ddxyz[3];
} NAV_accl_t;
static NAV_accl_t NAV_accl;

typedef struct {
  float z;
  float dz;
} NAV_alt_t;
static volatile NAV_alt_t NAV_baro;

static volatile float NAV_gyroBiasDyn[3] = {
    0.0f, 0.0f, 0.0f}; // rate-gyro bias estimate (dynamic) [rad/s]
static volatile float NAV_gyroBiasSta[3] = {
    0.0f, 0.0f, 0.0f}; // rate-gyro bias estimate (static) [rad/s]

static volatile float NAV_acclRoll = 0.0f;    // [rad]
static volatile float NAV_acclPitch = 0.0f;   // [rad]
static volatile float NAV_magnYaw = 0.0f;     // [rad]
static volatile float NAV_magnHeading = 0.0f; // [rad]

static volatile float NAV_acclRoll_init = 0.0f;  // [rad]
static volatile float NAV_acclPitch_init = 0.0f; // [rad]
static volatile float NAV_bodyHeading = 0.0f;    // [rad]

static volatile float NAV_filtAlpha_accl = 0.0f;
static volatile float NAV_filtAlpha_baro = 0.0f;
static volatile float NAV_filtAlpha_gyroBias = 0.0f;

static volatile float NAV_calibAcclA[3][3];
static volatile float NAV_calibAcclP[3];
static volatile float NAV_calibMagnV[3];
/**
 * @}
 */

/** @defgroup NAV_Private_Functions
 * @{
 */
void NAV_reset(void);
float NAV_pres2altASL(float pres);

/**
 * @}
 */

/**
 * @brief Compute running mean
 * @param[in] mean_prev running mean
 * @param[in] x new sample to be included in the mean
 * @param[in] n total samples thus far
 * @retval Running mean.
 */
float FILT_RunningMean(float mean_prev, float x, uint32_t n) {
  return mean_prev + (x - mean_prev) / n;
}

/**
 * @brief Compute running mean and variance
 * @param[in] mean_prev running mean at t-1
 * @param[in] var_prev running variance at t-1
 * @param[in] x new sample to be included in the average at t
 * @param[in] n total samples thus far at t
 * @param[out] mean at t
 * @param[out] variance at t
 * @retval Running average.
 */
void FILT_RunningMeanVariance(float mean_prev, float var_prev, float x,
                              uint32_t n, float *mean, float *var) {
  float delta = x - mean_prev;

  *mean = mean_prev + delta / n;
  *var = (var_prev * (n - 1) + delta * (x - mean_prev)) / n;
}

/**
 * @brief Compute running average
 * @param[in] y old and new filtered data
 * @param[in] x new sample to be filtered
 * @param[in] alpha smoothing factor, i.e. (sample_time)/(RC + sample_time)
 * @retval 1st Order Low-Pass filtered result.
 */
float FILT_1stOrderLP(float y, float x, float alpha) {
  return (y + alpha * (x - y));
}

/**
 * @brief Compute alpha smoothing factor, i.e. (sample_time)/(RC + sample_time)
 * @param[in] f_cutoff cut-off frequency [Hz]
 * @param[in] f_sample sampling frequency [Hz]
 * @retval 1st Order Low-Pass smoothing factor, alpha.
 */
float FILT_1stOrderLP_ComputeAlpha(float f_cutoff, float f_sample) {
  float t_sample = 1.0f / f_sample;                 // [sec]
  float rc = (float)(1.0f / (2 * M_PI * f_cutoff)); // [rad/s]
  return t_sample / (rc + t_sample);
}

/**
 * @brief  Get status flag
 * @retval -
 */
uint8_t NAV_calibGetStatus(uint8_t bit) { return BITVAL(NAV_calibStatus, bit); }

/**
 * @brief  Set status flag
 * @retval -
 */
uint8_t NAV_calibSetStatus(uint8_t bit) { return BITSET(NAV_calibStatus, bit); }

/**
 * @brief  Clear status flag
 * @retval -
 */
uint8_t NAV_calibClrStatus(uint8_t bit) { return BITCLR(NAV_calibStatus, bit); }

/**
 * @brief  Toggle status flag
 * @retval -
 */
uint8_t NAV_calibTogStatus(uint8_t bit) { return BITTOG(NAV_calibStatus, bit); }
/**
 * @}
 */

/**
 * @brief  Estimate altitude above sea level from pressure measurement
 * @param  pres Pressure [Pa]
 * @retval Estimated altitude above sea level [m]
 */
float NAV_pres2altASL(float pres) {
  return CONST_Tb / CONST_Lb *
         (powf((CONST_Pb / pres), CONST_Rb * CONST_Lb / CONST_G / CONST_M) -
          1.0f); // [m]
}

/**
 * @brief  Estimator initialisation
 * @param  None
 * @retval None
 */
void NAV_Init(void) {
  // @TODO initialise orientation of sensors wrt body-coordinate frame

  NAV_filtAlpha_accl = FILT_1stOrderLP_ComputeAlpha(10.0f, 1000.0f);
  NAV_filtAlpha_baro = FILT_1stOrderLP_ComputeAlpha(10.0f, 100.0f);
  NAV_filtAlpha_gyroBias = FILT_1stOrderLP_ComputeAlpha(0.1f, 1000.0f);

  NAV_reset();
}

void NAV_SetAcclCalib(float Axx, float Axy, float Axz, float Ayx, float Ayy,
                      float Ayz, float Azx, float Azy, float Azz, float Px,
                      float Py, float Pz) {
  NAV_calibAcclA[0][0] = Axx;
  NAV_calibAcclA[0][1] = Axy;
  NAV_calibAcclA[0][2] = Axz;

  NAV_calibAcclA[1][0] = Ayx;
  NAV_calibAcclA[1][1] = Ayy;
  NAV_calibAcclA[1][2] = Ayz;

  NAV_calibAcclA[2][0] = Azx;
  NAV_calibAcclA[2][1] = Azy;
  NAV_calibAcclA[2][2] = Azz;

  NAV_calibAcclP[0] = Px;
  NAV_calibAcclP[1] = Py;
  NAV_calibAcclP[2] = Pz;
}

void NAV_SetMagnCalib(float Vx, float Vy, float Vz) {
  NAV_calibMagnV[0] = Vx;
  NAV_calibMagnV[1] = Vy;
  NAV_calibMagnV[2] = Vz;
}

/**
 * @brief  Sensor calibration (non-blocking)
 * @param  t Calibration time [msec]
 * @retval 1 = Complete; 0 = Still calibrating
 */
uint8_t NAV_CalibrateStatic(uint16_t t) {
  static uint8_t first = 1u;
  static uint32_t t0 = 0u;

  if (first) {
    first = 0u;

    NAV_calibTime = t;
    NAV_calibSetStatus(NAV_CALIB_GYRO);
    NAV_calibSetStatus(NAV_CALIB_ACCL);
    NAV_calibSetStatus(NAV_CALIB_MAGN);
    // NAV_calibSetStatus(NAV_CALIB_BARO);
    // NAV_calibSetStatus(NAV_CALIB_ECEFp);
    // NAV_calibSetStatus(NAV_CALIB_ECEFv);
    // NAV_calibSetStatus(NAV_CALIB_NEDv);

    t0 = GetTime();
  }

  // finish calibration when all sensors are calibrated or after a 1000 msec
  // timeout
  if ((!NAV_calibGetStatus(NAV_CALIB_GYRO) &&
       !NAV_calibGetStatus(NAV_CALIB_ACCL) &&
       !NAV_calibGetStatus(NAV_CALIB_MAGN) &&
       !NAV_calibGetStatus(NAV_CALIB_BARO) &&
       !NAV_calibGetStatus(NAV_CALIB_ECEFp) &&
       !NAV_calibGetStatus(NAV_CALIB_NEDv)) ||
      ((GetTime() - t0) > (NAV_calibTime + 1000u))) {
    first = 1u;

    // reset some states
    NAV_reset();

    // initialise some states
    NAV_kfState.gba[0] = -NAV_acclRoll_init;  // [rad]
    NAV_kfState.gba[1] = -NAV_acclPitch_init; // [rad]
    NAV_state.gba[0] = -NAV_acclRoll_init;    // [rad]
    NAV_state.gba[1] = -NAV_acclPitch_init;   // [rad]

    return 1u;
  }

  return 0u;
}

/**
 * @brief  Set static rate-gyroscope bias to the current dynamic rate-gyroscope
 * bias estimate
 * @param  None
 * @retval None
 */
void NAV_CalibrateDynamic(void) {
  uint8_t i;
  for (i = 0; i < 3; i++) {
    NAV_state.gba[i] = NAV_kfState.gba[i];
    NAV_gyroBiasSta[i] = NAV_gyroBiasDyn[i];
  }
}

/**
 * @brief Reset Kalman filter state estimate
 * @param None
 * @retval None
 */
void NAV_reset(void) {
  uint8_t i;
  for (i = 0; i < 3; i++) {
    NAV_kfState.gba[i] = 0.0f;             // GBA
    NAV_kfState.bias_dgba[i] = 0.0f;       // dGBA_offset
    NAV_kfState.covar_gba[0][0][i] = 0.0f; // GBA covariance
    NAV_kfState.covar_gba[0][1][i] = 0.0f;
    NAV_kfState.covar_gba[1][0][i] = 0.0f;
    NAV_kfState.covar_gba[1][1][i] = 0.0f;
    NAV_kfState.K_gba[0][i] = 0.0f; // GBA Kalman gain
    NAV_kfState.K_gba[1][i] = 0.0f;
    NAV_kfState.dgba[i] = 0.0f; // dGBA

    NAV_kfState.xyz[i] = 0.0f;             // XYZ
    NAV_kfState.bias_dxyz[i] = 0.0f;       // dXYZ_offset
    NAV_kfState.covar_xyz[0][0][i] = 0.0f; // XYZ covariance
    NAV_kfState.covar_xyz[0][1][i] = 0.0f;
    NAV_kfState.covar_xyz[1][0][i] = 0.0f;
    NAV_kfState.covar_xyz[1][1][i] = 0.0f;
    NAV_kfState.K_xyz[0][i] = 0.0f; // XYZ Kalman gain
    NAV_kfState.K_xyz[1][i] = 0.0f;
    NAV_kfState.dxyz[i] = 0.0f; // dXYZ

    NAV_state.gba[i] = 0.0f;
    NAV_state.dgba[i] = 0.0f;
    NAV_state.xyz[i] = 0.0f;
    NAV_state.dxyz[i] = 0.0f;

    NAV_accl.xyz[i] = 0.0f;
    NAV_accl.dxyz[i] = 0.0f;
    NAV_accl.ddxyz[i] = 0.0f;

    NAV_gyroBiasDyn[i] = 0.0f;
  }

  NAV_baro.z = 0.0f;
  NAV_baro.dz = 0.0f;
}

/**
 * @brief Kalman filter prediction step for states xyz
 * @param i State ID (e.g. 0=x, 1=y, 2=z)
 * @param meas Measurement
 * @param varMeas Measurement variance
 * @param varErr Measurement error variance
 * @retval None
 */
void NAV_KfPredict_xyz(uint8_t i, float meas, float varMeas, float varErr) {
  static float t0[3] = {0.0f, 0.0f, 0.0f};

  // remove estimated bias error from state estimate
  meas += NAV_kfState.bias_dxyz[i];

  float dt = (GetTimeU() - t0[i]) / 1000.0f; // time difference [sec]
  t0[i] = GetTimeU(); // save timestamp for next prediction [msec]

  // predict state
  NAV_kfState.xyz[i] +=
      meas *
      dt; // translation estimate (xyz) + (dxyz_measured + dxyz_offset)*dt [m]
  NAV_kfState.dxyz[i] = meas; // translational velocity estimate [m/s]

  // predict covariance
  NAV_kfState.covar_xyz[0][0][i] += NAV_kfState.covar_xyz[0][1][i] * dt +
                                    NAV_kfState.covar_xyz[1][0][i] * dt +
                                    NAV_kfState.covar_xyz[1][1][i] * dt * dt +
                                    varMeas * dt * dt;
  NAV_kfState.covar_xyz[0][1][i] += NAV_kfState.covar_xyz[1][1][i] * dt;
  NAV_kfState.covar_xyz[1][0][i] += NAV_kfState.covar_xyz[1][1][i] * dt;
  NAV_kfState.covar_xyz[1][1][i] += varErr * dt * dt;
}

/**
 * @brief Kalman filter prediction step for states dxyz
 * @param i State ID (e.g. 0=x, 1=y, 2=z)
 * @param meas Measurement
 * @param varMeas Measurement variance
 * @param varErr Measurement error variance
 * @retval None
 */
void NAV_KfPredict_dxyz(uint8_t i, float meas, float varMeas, float varErr) {
  static float t0[3] = {0.0f, 0.0f, 0.0f};

  // remove estimated bias error from state estimate
  meas += NAV_kfState.bias_dxyz[i];

  float dt = (GetTimeU() - t0[i]) / 1000.0f; // time difference [sec]
  t0[i] = GetTimeU(); // save timestamp for next prediction [msec]

  // predict state
  NAV_kfState.dxyz[i] += meas * dt; // translation velocity estimate (xyz) +
                                    // (ddxyz_measured + ddxyz_offset)*dt [m]
  NAV_kfState.ddxyz[i] = meas; // translational acceleration estimate [m/s/s]

  // predict covariance
  NAV_kfState.covar_dxyz[0][0][i] += NAV_kfState.covar_dxyz[0][1][i] * dt +
                                     NAV_kfState.covar_dxyz[1][0][i] * dt +
                                     NAV_kfState.covar_dxyz[1][1][i] * dt * dt +
                                     varMeas * dt * dt;
  NAV_kfState.covar_dxyz[0][1][i] += NAV_kfState.covar_dxyz[1][1][i] * dt;
  NAV_kfState.covar_dxyz[1][0][i] += NAV_kfState.covar_dxyz[1][1][i] * dt;
  NAV_kfState.covar_dxyz[1][1][i] += varErr * dt * dt;
}

/**
 * @brief Kalman filter prediction step for states gba
 * @param i State ID (e.g. 0=G, 1=B, 2=A)
 * @param meas Measurement
 * @param varMeas Measurement variance
 * @param varErr Measurement error variance
 * @retval None
 */
void NAV_KfPredict_gba(uint8_t i, float meas, float varMeas, float varErr) {
  static float t0[3] = {0.0f, 0.0f, 0.0f};

  // remove estimated bias error from state estimate
  meas += NAV_kfState.bias_dgba[i];

  float dt = (GetTimeU() - t0[i]) / 1000.0f; // time difference [sec]
  t0[i] = GetTimeU(); // save timestamp for next prediction [msec]

  // predict state
  NAV_kfState.gba[i] +=
      meas *
      dt; // translation estimate (gba) + (dgba_measured + dgba_offset)*dt [m]
  NAV_kfState.dgba[i] = meas; // translational velocity estimate [m/s]

  // predict covariance
  NAV_kfState.covar_gba[0][0][i] += NAV_kfState.covar_gba[0][1][i] * dt +
                                    NAV_kfState.covar_gba[1][0][i] * dt +
                                    NAV_kfState.covar_gba[1][1][i] * dt * dt +
                                    varMeas * dt * dt;
  NAV_kfState.covar_gba[0][1][i] += NAV_kfState.covar_gba[1][1][i] * dt;
  NAV_kfState.covar_gba[1][0][i] += NAV_kfState.covar_gba[1][1][i] * dt;
  NAV_kfState.covar_gba[1][1][i] += varErr * dt * dt;

  // post-process additional data
  NAV_gyroBiasDyn[i] =
      FILT_1stOrderLP(NAV_gyroBiasDyn[i], NAV_kfState.bias_dgba[i],
                      NAV_filtAlpha_gyroBias); // [rad/s]

  if (NAV_kfState.gba[i] > +M_PI)
    NAV_kfState.gba[i] -= M_PI + M_PI;
  if (NAV_kfState.gba[i] < -M_PI)
    NAV_kfState.gba[i] += M_PI + M_PI;
}

/**
 * @brief Kalman filter correction step for states xyz
 * @param i State ID (e.g. 0=x, 1=y, 2=z)
 * @param meas Measurement
 * @param var Measurement variance
 * @retval None
 */
void NAV_KfCorrect_xyz(uint8_t i, float meas, float var) {
  // compute Kalman gain
  float den = NAV_kfState.covar_xyz[0][0][i] + var;
  NAV_kfState.K_xyz[0][i] = NAV_kfState.covar_xyz[0][0][i] / den;
  NAV_kfState.K_xyz[1][i] = NAV_kfState.covar_xyz[1][0][i] / den;

  // update state estimate
  float res = meas - NAV_kfState.xyz[i];               // residual
  NAV_kfState.xyz[i] += NAV_kfState.K_xyz[0][i] * res; // XYZ estimate [m]
  NAV_kfState.bias_dxyz[i] +=
      NAV_kfState.K_xyz[1][i] * res; // dXYZ bias error estimate [m/s]

  // update covariance estimate
  NAV_kfState.covar_xyz[0][0][i] =
      (1 - NAV_kfState.K_xyz[0][i]) * NAV_kfState.covar_xyz[0][0][i];
  NAV_kfState.covar_xyz[0][1][i] =
      (1 - NAV_kfState.K_xyz[0][i]) * NAV_kfState.covar_xyz[0][1][i];
  NAV_kfState.covar_xyz[1][0][i] =
      -NAV_kfState.K_xyz[1][i] * NAV_kfState.covar_xyz[0][0][i] +
      NAV_kfState.covar_xyz[1][0][i];
  NAV_kfState.covar_xyz[1][1][i] =
      -NAV_kfState.K_xyz[1][i] * NAV_kfState.covar_xyz[0][1][i] +
      NAV_kfState.covar_xyz[1][1][i];
}

/**
 * @brief Kalman filter correction step for states dxyz
 * @param i State ID (e.g. 0=x, 1=y, 2=z)
 * @param meas Measurement
 * @param var Measurement variance
 * @retval None
 */
void NAV_KfCorrect_dxyz(uint8_t i, float meas, float var) {
  // compute Kalman gain
  float den = NAV_kfState.covar_dxyz[0][0][i] + var;
  NAV_kfState.K_dxyz[0][i] = NAV_kfState.covar_dxyz[0][0][i] / den;
  NAV_kfState.K_dxyz[1][i] = NAV_kfState.covar_dxyz[1][0][i] / den;

  // update state estimate
  float res = meas - NAV_kfState.dxyz[i];                // residual
  NAV_kfState.dxyz[i] += NAV_kfState.K_dxyz[0][i] * res; // XYZ estimate [m]
  NAV_kfState.bias_dxyz[i] +=
      NAV_kfState.K_dxyz[1][i] * res; // dXYZ bias error estimate [m/s]

  // update covariance estimate
  NAV_kfState.covar_dxyz[0][0][i] =
      (1 - NAV_kfState.K_dxyz[0][i]) * NAV_kfState.covar_dxyz[0][0][i];
  NAV_kfState.covar_dxyz[0][1][i] =
      (1 - NAV_kfState.K_dxyz[0][i]) * NAV_kfState.covar_dxyz[0][1][i];
  NAV_kfState.covar_dxyz[1][0][i] =
      -NAV_kfState.K_dxyz[1][i] * NAV_kfState.covar_dxyz[0][0][i] +
      NAV_kfState.covar_dxyz[1][0][i];
  NAV_kfState.covar_dxyz[1][1][i] =
      -NAV_kfState.K_dxyz[1][i] * NAV_kfState.covar_dxyz[0][1][i] +
      NAV_kfState.covar_dxyz[1][1][i];
}

/**
 * @brief Kalman filter correction step for states gba
 * @param i State ID (e.g. 0=g, 1=b, 2=a)
 * @param meas Measurement
 * @param var Measurement variance
 * @retval None
 */
void NAV_KfCorrect_gba(uint8_t i, float meas, float var) {
  // compute Kalman gain
  float den = NAV_kfState.covar_gba[0][0][i] + var;
  NAV_kfState.K_gba[0][i] = NAV_kfState.covar_gba[0][0][i] / den;
  NAV_kfState.K_gba[1][i] = NAV_kfState.covar_gba[1][0][i] / den;

  // update state estimate
  float res = meas - NAV_kfState.gba[i];               // residual
  NAV_kfState.gba[i] += NAV_kfState.K_gba[0][i] * res; // GBA estimate [m]
  NAV_kfState.bias_dgba[i] +=
      NAV_kfState.K_gba[1][i] * res; // dGBA bias error estimate [m/s]

  // update covariance estimate
  NAV_kfState.covar_gba[0][0][i] =
      (1 - NAV_kfState.K_gba[0][i]) * NAV_kfState.covar_gba[0][0][i];
  NAV_kfState.covar_gba[0][1][i] =
      (1 - NAV_kfState.K_gba[0][i]) * NAV_kfState.covar_gba[0][1][i];
  NAV_kfState.covar_gba[1][0][i] =
      -NAV_kfState.K_gba[1][i] * NAV_kfState.covar_gba[0][0][i] +
      NAV_kfState.covar_gba[1][0][i];
  NAV_kfState.covar_gba[1][1][i] =
      -NAV_kfState.K_gba[1][i] * NAV_kfState.covar_gba[0][1][i] +
      NAV_kfState.covar_gba[1][1][i];

  if (NAV_kfState.gba[i] > +M_PI)
    NAV_kfState.gba[i] -= M_PI + M_PI;
  if (NAV_kfState.gba[i] < -M_PI)
    NAV_kfState.gba[i] += M_PI + M_PI;
}

/**
 * @brief  Estimator interrupt service routine for rate-gyroscope data
 * @param  gx, gy, gz Raw rate-gyroscope data in sensor frame [rad/s] [3x1]
 * @retval None
 */
void NAV_SensGyro(float gx, float gy, float gz) {
  static uint8_t reset = 1u;     // calibration reset flag
  static uint32_t cnt = 0u;      // calibration running average counter
  static uint32_t t0_calib = 0u; // calibration timer [msec]

  static float t0 = 0.0f;                         // [msec]
  static float gyro_init[3] = {0.0f, 0.0f, 0.0f}; // body angular-rate [rad/s]
  float gyro[3]; // unbiased body angular-rate [rad/s]

  // @TODO Automate orientation
  gyro[0] = +(gx - gyro_init[0]);
  gyro[1] = -(gy - gyro_init[1]);
  gyro[2] = -(gz - gyro_init[2]);

  // Body-rates -> Euler-rates [rad/s]
  // save unfiltered estimate
#if 0
  NAV_state.dgba[0] = gyro[0] + NAV_state.gba[1]*gyro[2];
  NAV_state.dgba[1] = gyro[1] - NAV_state.gba[0]*gyro[2];
  NAV_state.dgba[2] = gyro[2] + NAV_state.gba[0]*gyro[1];
#else
  NAV_state.dgba[0] = gyro[0] + NAV_kfState.gba[1] * gyro[2];
  NAV_state.dgba[1] = gyro[1] - NAV_kfState.gba[0] * gyro[2];
  NAV_state.dgba[2] = gyro[2] + NAV_kfState.gba[0] * gyro[1];
#endif

  // estimate time difference between this measurement and the last
  float dt = (GetTimeU() - t0) / 1000.0f; // time difference [sec]
  t0 = GetTimeU(); // save timestamp for next prediction [msec]

  uint8_t i;
  for (i = 0; i < 3; i++) {
    NAV_state.gba[i] +=
        NAV_state.dgba[i] *
        dt; // angle estimate (GBA) + (dGBA_measured + dGBA_offset)*dt [rad]

#if 1 // keep within +/-180 deg range
    if (NAV_state.gba[i] > +M_PI)
      NAV_state.gba[i] -= M_PI + M_PI;
    if (NAV_state.gba[i] < -M_PI)
      NAV_state.gba[i] += M_PI + M_PI;
#endif
  }

  // CALIBRATE
  if (NAV_calibGetStatus(NAV_CALIB_GYRO)) {
    // reset some variables
    if (reset) {
      reset = 0u;
      cnt = 0u;             // reset running average counter
      gyro_init[0] = 0.0f;  // reset initial angular rate [rad/s]
      gyro_init[1] = 0.0f;  // reset initial angular rate [rad/s]
      gyro_init[2] = 0.0f;  // reset initial angular rate [rad/s]
      t0_calib = GetTime(); // start of calibration time
    }

    // compute bias using a running average
    cnt++;
    gyro_init[0] = FILT_RunningMean(gyro_init[0], gx, cnt); // [rad/s]
    gyro_init[1] = FILT_RunningMean(gyro_init[1], gy, cnt); // [rad/s]
    gyro_init[2] = FILT_RunningMean(gyro_init[2], gz, cnt); // [rad/s]

    // set new offsets
    if (GetTime() - t0_calib > NAV_calibTime) {
      NAV_state.bias_dgba[0] = gyro_init[0]; // [rad/s]
      NAV_state.bias_dgba[1] = gyro_init[1]; // [rad/s]
      NAV_state.bias_dgba[2] = gyro_init[2]; // [rad/s]

      NAV_calibClrStatus(NAV_CALIB_GYRO); // clear completion flag
      reset = 1u;                         // set flag to default state
    }
  }
}

/**
 * @brief  Estimator interrupt service routine for accelerometer data
 * @param  ax, ay, az Raw accelerometer data in sensor frame [g] [3x1]
 * @retval None
 */
#define ACCL_THRESHOLD 0.15f // [g]
void NAV_SensAccl(float ax, float ay, float az) {
  static uint8_t reset = 1u;     // calibration reset flag
  static uint32_t cnt = 0u;      // calibration running average counter
  static uint32_t t0_calib = 0u; // calibration timer [msec]

  static float t0 = 0.0f;                         // [msec]
  static float accl_init[3] = {0.0f, 0.0f, 0.0f}; // body acceleration [m/s/s]
  static float accl[3]; // calibrated accelerometer data [g]
  float cAccl[3];       // calibrated acclerometer data [g]

  // calibrate accelerometer data [g]
  cAccl[0] = NAV_calibAcclA[0][0] * ax + NAV_calibAcclA[0][1] * ay +
             NAV_calibAcclA[0][2] * az + NAV_calibAcclP[0]; // [g]
  cAccl[1] = NAV_calibAcclA[1][0] * ax + NAV_calibAcclA[1][1] * ay +
             NAV_calibAcclA[1][2] * az + NAV_calibAcclP[1]; // [g]
  cAccl[2] = NAV_calibAcclA[2][0] * ax + NAV_calibAcclA[2][1] * ay +
             NAV_calibAcclA[2][2] * az + NAV_calibAcclP[2]; // [g]

  // @TODO Automate orientation
  accl[0] = +cAccl[0];
  accl[1] = -cAccl[1];
  accl[2] = -cAccl[2];

  NAV_accl.ddxyz[0] = (accl[0] - accl_init[0]) * CONST_G; // [m/s/s]
  NAV_accl.ddxyz[1] = (accl[1] - accl_init[1]) * CONST_G; // [m/s/s]
  NAV_accl.ddxyz[2] = (accl[2] - accl_init[2]) * CONST_G; // [m/s/s]

  // estimate time difference between this measurement and the last
  float dt = (GetTimeU() - t0) / 1000.0f; // time difference [sec]
  t0 = GetTimeU(); // save timestamp for next prediction [msec]
  NAV_accl.dxyz[0] += NAV_accl.ddxyz[0] * dt; // [m/s]
  NAV_accl.dxyz[1] += NAV_accl.ddxyz[1] * dt; // [m/s]
  NAV_accl.dxyz[2] += NAV_accl.ddxyz[2] * dt; // [m/s]
  NAV_accl.xyz[0] += NAV_accl.dxyz[0] * dt;   // [m]
  NAV_accl.xyz[1] += NAV_accl.dxyz[1] * dt;   // [m]
  NAV_accl.xyz[2] += NAV_accl.dxyz[2] * dt;   // [m]

  // check acceleration
  float totAccl =
      accl[0] * accl[0] + accl[1] * accl[1] + accl[2] * accl[2]; // [g^2]
  if (totAccl > 1.0 + ACCL_THRESHOLD || totAccl < 1.0 - ACCL_THRESHOLD)
    return;

  // compute roll and pitch
  // ref:
  // http://www.st.com/web/en/resource/technical/document/application_note/CD00268887.pdf
  NAV_acclRoll =
      atan2f(+accl[1], sqrtf(accl[0] * accl[0] +
                             accl[2] * accl[2])); // (-pi/2,+pi/2) [rad]
  NAV_acclPitch =
      atan2f(-accl[0], sqrtf(accl[1] * accl[1] +
                             accl[2] * accl[2])); // (-pi/2,+pi/2) [rad]

  // CALIBRATE
  if (NAV_calibGetStatus(NAV_CALIB_ACCL)) {
    // reset some variables
    if (reset) {
      reset = 0u;                // reset flag
      cnt = 0u;                  // reset running average counter
      NAV_acclRoll_init = 0.0f;  // reset initial roll angle [rad]
      NAV_acclPitch_init = 0.0f; // reset initial pitch angle [rad]
      accl_init[0] = 0.0f;       // reset initial acceleration [m/s/s]
      accl_init[1] = 0.0f;       // reset initial acceleration [m/s/s]
      accl_init[2] = 0.0f;       // reset initial acceleration [m/s/s]
      t0_calib = GetTime();      // start of calibration time
    }

    // compute current state over a period of time using a running average
    cnt++;
    NAV_acclRoll_init =
        FILT_RunningMean(NAV_acclRoll_init, NAV_acclRoll, cnt); // [rad]
    NAV_acclPitch_init =
        FILT_RunningMean(NAV_acclPitch_init, NAV_acclPitch, cnt); // [rad]

    // compute bias using a running average
    accl_init[0] = FILT_RunningMean(accl_init[0], accl[0], cnt); // [m/s/s]
    accl_init[1] = FILT_RunningMean(accl_init[1], accl[1], cnt); // [m/s/s]
    accl_init[2] = FILT_RunningMean(accl_init[2], accl[2], cnt); // [m/s/s]

    // set new offsets
    if (GetTime() - t0_calib > NAV_calibTime) {
      NAV_state.bias_ddxyz[0] = accl_init[0]; // [m/s/s]
      NAV_state.bias_ddxyz[1] = accl_init[1]; // [m/s/s]
      NAV_state.bias_ddxyz[2] = accl_init[2]; // [m/s/s]

      NAV_calibClrStatus(NAV_CALIB_ACCL); // clear completion flag
      reset = 1u;                         // set flag to default state
    }
  }
}

/**
 * @brief  Estimator interrupt service routine for magnetometer data
 * @param  mx, my, mz Raw magnetometer data in sensor frame [uT] [3x1]
 * @retval None
 */
float debug_Bfx = 0.0f;
float debug_Bfy = 0.0f;
float debug_Bfz = 0.0f;
void NAV_SensMagn(float mx, float my, float mz) {
  static uint8_t reset = 1u;     // calibration reset flag
  static uint32_t cnt = 0u;      // calibration running average counter
  static uint32_t t0_calib = 0u; // calibration timer [msec]

  static float heading_init = 0.0f; // [rad]
  float magn[3];                    // unbiased magnetometer data [uT]

  // @TODO Automate orientation
  // remove magnetometer bias (sensor frame) [uT]
  magn[0] = mx - NAV_calibMagnV[0];
  magn[1] = my - NAV_calibMagnV[1];
  magn[2] = mz - NAV_calibMagnV[2];

  // TILT COMPENSATED HEADING
  // ref: http://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf
  float sin_g = sinf(+NAV_kfState.gba[0]);
  float cos_g = cosf(+NAV_kfState.gba[0]);
  float sin_b = sinf(-NAV_kfState.gba[1]);
  float cos_b = cosf(-NAV_kfState.gba[1]);

  float Bfx =
      +magn[0] * cos_b + magn[1] * sin_b * sin_g + magn[2] * sin_b * cos_g;
  float Bfy = +magn[1] * cos_g - magn[2] * sin_g;
  // float Bfz = -magn[0]*sin_b + magn[1]*cos_b*sin_g + magn[2]*cos_b*cos_g;
  float heading = -atan2f(-Bfy, Bfx); // tilt-compensated heading or yaw-angle
                                      // wrt magnetic north (-pi,+pi] [rad]

  // char str[100]; sprintf (str, "Bf: %+.3f %+.3f %+.3f | heading: %+.3f\r\n",
  // Bfx, Bfy, Bfz, heading); UART_WriteTxt(1, str); char str[100]; sprintf
  // (str, "heading: %.3f [deg]\r\n", heading*180/M_PI); UART_WriteTxt(1, str);

  // save unfiltered estimate
  NAV_magnHeading = heading;
  NAV_magnYaw =
      heading -
      heading_init; // compute yaw angle in body coordinate frame [rad]

  // CALIBRATE
  if (NAV_calibGetStatus(NAV_CALIB_MAGN)) {
    // reset some variables
    if (reset) {
      reset = 0u;
      cnt = 0u;             // reset running average counter
      heading_init = 0.0f;  // reset initial heading angle
      t0_calib = GetTime(); // start of calibration time
    }

    // compute current state over a period of time using a running average
    heading_init = FILT_RunningMean(
        heading_init, heading,
        ++cnt); // tilt-compensated heading (wrt magnetic north) (-pi,+pi] [rad]

    // set new offsets
    if (GetTime() - t0_calib > NAV_calibTime) {
      NAV_calibClrStatus(NAV_CALIB_MAGN); // clear completion flag
      reset = 1u;                         // set flag to default state
    }
  }
}

/**
 * @brief  Estimator interrupt service routine for barometric pressure data
 * @param  raw_baro Raw barometric data [Pa]
 * @retval None
 */
void NAV_SensBaro(float raw_baro) {
  static uint8_t reset = 1u;     // calibration reset flag
  static uint32_t cnt = 0u;      // calibration running average counter
  static uint32_t t0_calib = 0u; // calibration timer [msec]

  static float baro_init = 0u;
  static float t0 = 0.0f; // [msec]
  static float z0 = 0.0f; // [m]
  float asl = 0.0f;       // unbiased ASL estimate [m]

  // remove bias
  asl = NAV_pres2altASL(raw_baro) - baro_init; // [m]
  if (isnan(asl))
    return;

  float dt = (GetTimeU() - t0) / 1000.0f; // [sec]
  t0 = GetTimeU();                        // [msec]

#if 0
  // save unfiltered estimate
  NAV_baro.z = asl; // [m]
#else
  // save filtered estimate
  NAV_baro.z = FILT_1stOrderLP(NAV_baro.z, asl, NAV_filtAlpha_baro); // [m]
#endif
  NAV_baro.dz = (NAV_baro.z - z0) / dt; // [m/s]
  z0 = NAV_baro.z;

  // CALIBRATE
  if (NAV_calibGetStatus(NAV_CALIB_BARO)) {
    // reset some variables
    if (reset) {
      reset = 0u;
      cnt = 0u;             // reset running average counter
      baro_init = 0.0f;     // reset initial barometer reading
      t0_calib = GetTime(); // start of calibration time
    }

    // compute current state over a period of time using a running average
    baro_init = FILT_RunningMean(baro_init, NAV_pres2altASL(raw_baro),
                                 ++cnt); // ASL data [m]
    // char str[100]; sprintf(str,"%f %f\r\n", baro_init,
    // NAV_pres2altASL(raw_baro)); UART_WriteTxt(1,str); set new offsets
    if (GetTime() - t0_calib > NAV_calibTime) {
      NAV_calibClrStatus(NAV_CALIB_BARO); // clear completion flag
      reset = 1u;                         // set flag to default state
    }
  }
}

/**
 * @brief  Estimator interrupt service routine for GNSS ECEF position data
 * @param  raw_ecef_p ECEF coordinate position [m]
 * @retval None
 */
void NAV_SensECEFp(float *raw_ecef_p) {
  static uint8_t reset = 1u;     // calibration reset flag
  static uint32_t cnt = 0u;      // calibration running average counter
  static uint32_t t0_calib = 0u; // calibration timer [msec]

  static float ecef_init[3] = {0.0f, 0.0f,
                               0.0f};  // position in ECEF coordinates [m]
  float enu_p[3] = {0.0f, 0.0f, 0.0f}; // position in ENU coordinates [m]
  float xyz[3];                        // position in body coordinate frame [m]

  ecef2enu_p(raw_ecef_p[0], raw_ecef_p[1], raw_ecef_p[2], ecef_init[0],
             ecef_init[1], ecef_init[2], &enu_p[0], &enu_p[1], &enu_p[2]);

  // rotate ENU position to vehicle's body-coordinate frame
  float cos_a =
      cosf(NAV_GetMagnHeadingInit() + M_PI_2); // X-axis aligns with East
  float sin_a =
      sinf(NAV_GetMagnHeadingInit() + M_PI_2);   // X-axis aligns with East
  xyz[0] = +enu_p[0] * cos_a + enu_p[1] * sin_a; // [m]
  xyz[1] = -enu_p[0] * sin_a + enu_p[1] * cos_a; // [m]
  xyz[2] = +enu_p[2];                            // [m]

  // save unfiltered estimate
  NAV_state.xyz[0] = xyz[0];
  NAV_state.xyz[1] = xyz[1];
  NAV_state.xyz[2] = xyz[2];

  // CALIBRATE
  if (NAV_calibGetStatus(NAV_CALIB_ECEFp)) {
    // reset some variables
    if (reset) {
      reset = 0u;
      cnt = 0u;             // reset running average counter
      ecef_init[0] = 0.0f;  // reset initial ECEF position vector [m]
      ecef_init[1] = 0.0f;  // reset initial ECEF position vector [m]
      ecef_init[2] = 0.0f;  // reset initial ECEF position vector [m]
      t0_calib = GetTime(); // start of calibration time
    }

    // compute bias using a running average
    cnt++;
    ecef_init[0] = FILT_RunningMean(ecef_init[0], raw_ecef_p[0], cnt); // [m]
    ecef_init[1] = FILT_RunningMean(ecef_init[1], raw_ecef_p[1], cnt); // [m]
    ecef_init[2] = FILT_RunningMean(ecef_init[2], raw_ecef_p[2], cnt); // [m]

    // set new offsets
    if (GetTime() - t0_calib > NAV_calibTime) {
      NAV_calibClrStatus(NAV_CALIB_ECEFp); // clear completion flag
      reset = 1u;                          // set flag to default state
    }
  }
}

/**
 * @brief  Estimator interrupt service routine for GNSS ECEF velocity data
 * @param  raw_ecef_v ECEF velocity [m/s]
 * @param  raw_ecef_p ECEF coordinate position [m]
 * @retval None
 */
void NAV_SensECEFv(float *raw_ecef_v, float *raw_ecef_p) {
  static uint8_t reset = 1u;       // calibration reset flag
  static uint32_t cnt = 0u;        // calibration running average counter
  static uint32_t t0_calib = 0.0f; // calibration timer [msec]

  static float ecef_p_init[3] = {0.0f, 0.0f,
                                 0.0f}; // position in ECEF coordinates [m]
  static float ecef_v_init[3] = {0.0f, 0.0f,
                                 0.0f}; // velocity in ECEF frame [m/s]
  // float ecef_p[3] = {0.0f,0.0f,0.0f}; // unbiased ECEF position measurements
  // [m]
  float ecef_v[3] = {0.0f, 0.0f,
                     0.0f}; // unbiased ECEF velocity measurements [m/s]
  float enu_v[3];           // velocity in ENU coordinates [m/s]
  float dxyz[3];            // velocity in body coordinate frame [m/s]

  // remove ECEF position bias [m]
  // ecef_p[0] = raw_ecef_p[0] - ecef_p_init[0];
  // ecef_p[1] = raw_ecef_p[1] - ecef_p_init[1];
  // ecef_p[2] = raw_ecef_p[2] - ecef_p_init[2];

  // remove ECEF velocity bias [m/s]
  ecef_v[0] = raw_ecef_v[0] - ecef_v_init[0];
  ecef_v[1] = raw_ecef_v[1] - ecef_v_init[1];
  ecef_v[2] = raw_ecef_v[2] - ecef_v_init[2];

  ecef2enu_v(ecef_v[0], ecef_v[1], ecef_v[2], ecef_p_init[0], ecef_p_init[1],
             ecef_p_init[2], &enu_v[0], &enu_v[1], &enu_v[2]);

  // rotate ENU velocity to vehicle's body-coordinate frame
  float cos_a =
      cosf(NAV_GetMagnHeadingInit() + M_PI_2); // X-axis aligns with East
  float sin_a =
      sinf(NAV_GetMagnHeadingInit() + M_PI_2);    // X-axis aligns with East
  dxyz[0] = +enu_v[0] * cos_a + enu_v[1] * sin_a; // [m/s]
  dxyz[1] = -enu_v[0] * sin_a + enu_v[1] * cos_a; // [m/s]
  dxyz[2] = +enu_v[2];                            // [m/s]

  // save unfiltered estimate
  NAV_state.dxyz[0] = dxyz[0];
  NAV_state.dxyz[1] = dxyz[1];
  NAV_state.dxyz[2] = dxyz[2];

  // CALIBRATE
  if (NAV_calibGetStatus(NAV_CALIB_ECEFv)) {
    // reset some variables
    if (reset) {
      reset = 0u;
      cnt = 0u;              // reset running average counter
      ecef_p_init[0] = 0.0f; // reset initial ECEF position vector [m]
      ecef_p_init[1] = 0.0f; // reset initial ECEF position vector [m]
      ecef_p_init[2] = 0.0f; // reset initial ECEF position vector [m]
      ecef_v_init[0] = 0.0f; // reset initial ECEF velocity vector [m/s]
      ecef_v_init[1] = 0.0f; // reset initial ECEF velocity vector [m/s]
      ecef_v_init[2] = 0.0f; // reset initial ECEF velocity vector [m/s]
      t0_calib = GetTime();  // start of calibration time
    }

    // compute bias using a running average
    cnt++;
    ecef_p_init[0] =
        FILT_RunningMean(ecef_p_init[0], raw_ecef_p[0], cnt); // [m]
    ecef_p_init[1] =
        FILT_RunningMean(ecef_p_init[1], raw_ecef_p[1], cnt); // [m]
    ecef_p_init[2] =
        FILT_RunningMean(ecef_p_init[2], raw_ecef_p[2], cnt); // [m]
    ecef_v_init[0] =
        FILT_RunningMean(ecef_v_init[0], raw_ecef_v[0], cnt); // [m/s]
    ecef_v_init[1] =
        FILT_RunningMean(ecef_v_init[1], raw_ecef_v[1], cnt); // [m/s]
    ecef_v_init[2] =
        FILT_RunningMean(ecef_v_init[2], raw_ecef_v[2], cnt); // [m/s]

    // set new offsets
    if (GetTime() - t0_calib > NAV_calibTime) {
      NAV_calibClrStatus(NAV_CALIB_ECEFv); // clear completion flag
      reset = 1u;                          // set flag to default state
    }
  }
}

/**
 * @brief  Estimator interrupt service routine for GNSS NED velocity data
 * @param  raw_ned_v NED velocity [m/s]
 * @retval None
 */
void NAV_SensNEDv(float *raw_ned_v) {
  static uint8_t reset = 1u;       // calibration reset flag
  static uint32_t cnt = 0u;        // calibration running average counter
  static uint32_t t0_calib = 0.0f; // calibration timer [msec]

  static float ned_v_init[3] = {0.0f, 0.0f,
                                0.0f}; // velocity in NED frame [m/s]
  float ned_v[3];                      // velocity in NED coordinates [m/s]
  float dxyz[3]; // velocity in body coordinate frame [m/s]

  // remove bias
  ned_v[0] = raw_ned_v[0] - ned_v_init[0];
  ned_v[1] = raw_ned_v[1] - ned_v_init[1];
  ned_v[2] = raw_ned_v[2] - ned_v_init[2];

  // rotate NED velocity to vehicle's body-coordinate frame
  float cos_a =
      cosf(NAV_GetMagnHeadingInit() + M_PI_2); // X-axis aligns with East
  float sin_a =
      sinf(NAV_GetMagnHeadingInit() + M_PI_2);    // X-axis aligns with East
  dxyz[0] = +ned_v[1] * cos_a + ned_v[0] * sin_a; // [m/s]
  dxyz[1] = -ned_v[1] * sin_a + ned_v[0] * cos_a; // [m/s]
  dxyz[2] = -ned_v[2];                            // [m/s]

  // save unfiltered estimate
  NAV_state.dxyz[0] = dxyz[0];
  NAV_state.dxyz[1] = dxyz[1];
  NAV_state.dxyz[2] = dxyz[2];

  // CALIBRATE
  if (NAV_calibGetStatus(NAV_CALIB_NEDv)) {
    // reset some variables
    if (reset) {
      reset = 0u;
      cnt = 0u;             // reset running average counter
      ned_v_init[0] = 0.0f; // reset initial ECEF velocity vector [m/s]
      ned_v_init[1] = 0.0f; // reset initial ECEF velocity vector [m/s]
      ned_v_init[2] = 0.0f; // reset initial ECEF velocity vector [m/s]
      t0_calib = GetTime(); // start of calibration time
    }

    // compute bias using a running average
    cnt++;
    ned_v_init[0] = FILT_RunningMean(ned_v_init[0], raw_ned_v[0], cnt); // [m/s]
    ned_v_init[1] = FILT_RunningMean(ned_v_init[1], raw_ned_v[1], cnt); // [m/s]
    ned_v_init[2] = FILT_RunningMean(ned_v_init[2], raw_ned_v[2], cnt); // [m/s]

    // set new offsets
    if (GetTime() - t0_calib > NAV_calibTime) {
      NAV_calibClrStatus(NAV_CALIB_NEDv); // clear completion flag
      reset = 1u;                         // set flag to default state
    }
  }
}

void NAV_SetStateXYZ(uint8_t i, float val) { NAV_state.xyz[i] = val; }
void NAV_SetStateDXYZ(uint8_t i, float val) { NAV_state.dxyz[i] = val; }
void NAV_SetStateGBA(uint8_t i, float val) { NAV_state.gba[i] = val; }
void NAV_SetStateDGBA(uint8_t i, float val) { NAV_state.dgba[i] = val; }
void NAV_SetStateDGBAbias(uint8_t i, float val) {
  NAV_state.bias_dgba[i] = val;
}

float *NAV_GetStateXYZ(void) { return NAV_state.xyz; }
float *NAV_GetStateDXYZ(void) { return NAV_state.dxyz; }
float *NAV_GetStateGBA(void) { return NAV_state.gba; }
float *NAV_GetStateDGBA(void) { return NAV_state.dgba; }
float *NAV_GetStateDGBAbias(void) { return NAV_state.bias_dgba; }

float *NAV_GetKfStateXYZ(void) { return NAV_kfState.xyz; }
float *NAV_GetKfStateDXYZ(void) { return NAV_kfState.dxyz; }
float *NAV_GetKfStateGBA(void) { return NAV_kfState.gba; }
float *NAV_GetKfStateDGBA(void) { return NAV_kfState.dgba; }
float *NAV_GetKfStateDGBAbias(void) { return NAV_kfState.bias_dgba; }

float NAV_GetAcclX(void) { return NAV_accl.xyz[0]; }
float NAV_GetAcclY(void) { return NAV_accl.xyz[1]; }
float NAV_GetAcclZ(void) { return NAV_accl.xyz[2]; }
float NAV_GetAcclDX(void) { return NAV_accl.dxyz[0]; }
float NAV_GetAcclDY(void) { return NAV_accl.dxyz[1]; }
float NAV_GetAcclDZ(void) { return NAV_accl.dxyz[2]; }
float NAV_GetAcclRoll(void) { return NAV_acclRoll; }
float NAV_GetAcclPitch(void) { return NAV_acclPitch; }
float NAV_GetMagnYaw(void) { return NAV_magnYaw; }
float NAV_GetMagnHeading(void) { return NAV_magnHeading; }
float NAV_GetMagnHeadingInit(void) { return NAV_bodyHeading; }
void NAV_SetBodyHeadingInit(float val) { NAV_bodyHeading = val; }

float NAV_GetBaroZ(void) { return NAV_baro.z; }
float NAV_GetBaroDZ(void) { return NAV_baro.dz; }
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
