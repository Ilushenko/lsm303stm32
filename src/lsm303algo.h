/// \file lsm303algo.h
/// \brief This file is part of LSM303DLHC Library for STM32 Nucleo L4
/// \copyright &copy; https://github.com/Ilushenko Oleksandr Ilushenko
///	\author Oleksandr Ilushenko
///	\date 2024
#ifndef __LSM303_ALGO_H__
#define __LSM303_ALGO_H__

#include "stm32l4xx_hal.h"

#define CNTSETUP 32U

/// \defgroup lsm303algo 3. LSM303 Algorithmes
/// \brief LSM303 Algorithmes functions

/// \brief Motion detection by linear accelerometer
/// \details Using \b low_pass \b filter for exclude reaction on shocks
/// \param x X axis
/// \param y Y axis
/// \param z Z axis
/// \param alpha Coefficient of the low-pass filter. (1 > a > 0)
/// \param delta Trigger threshold
/// \param sample Samples for checks (duration of measurement)
/// \return \c 0.0F if motion don't detected or value of the trigger
/// \ingroup lsm303algo
float motionLP(const float x, const float y, const float z, const float alpha, const float delta, const uint8_t sample);

/// \brief Motion detection by linear accelerometer
/// \details Using \b Kalman \b filter for exclude reaction on shocks
/// \param x X axis
/// \param y Y axis
/// \param z Z axis
/// \param Q Process covariance
/// \param R Measurement covariance
/// \param E Error prediction
/// \param delta Trigger threshold
/// \param sample Samples for checks (duration of measurement)
/// \return \c 0.0F if motion don't detected or value of the trigger
/// \ingroup lsm303algo
float motionK(const float x, const float y, const float z, const float Q, const float R, const float E, const float delta, const uint8_t sample);

/// \brief Detection of magnetic field distortion
/// \details Using \b high-pass \b filter for detection
/// \param x X axis
/// \param y Y axis
/// \param z Z axis
/// \param alpha Coefficient of the high-pass filter. (1 > a > 0)
/// \param delta Trigger threshold
/// \return \c 0.0F if distortion don't detected or value of the trigger
/// \ingroup lsm303algo
float distortionHP(const float x, const float y, const float z, const float alpha, const float delta);

/// \brief Detection of magnetic field distortion
/// \details Using \b low-pass \b filter for detection
/// \param x X axis
/// \param y Y axis
/// \param z Z axis
/// \param alpha Coefficient of the low-pass filter. Smaller - more smoothing. (1 > a > 0)
/// \param delta Trigger threshold
/// \return \c 0.0F if distortion don't detected or value of the trigger
/// \ingroup lsm303algo
float distortionLP(const float x, const float y, const float z, const float alpha, const float delta);

/// \brief Orientation by linear accelerometer and magnetometer
/// \details Using \b low-pass \b filter
/// \param a Array of linear accelerometer axis
/// \param m Array of magnetic field axis
/// \param alpha Coefficient of the low-pass filter. Smaller - more smoothing. (1 > a > 0)
/// \param pitch Pitch pointer
/// \param roll Roll pointer
/// \param yaw Yaw pointer
/// \return \c 0U if \b pitch, \b roll and \b yaw haz calculated or \c 1U is not calculated (no accumulated data for calculation)
/// \ingroup lsm303algo
uint8_t orientLP(const float a[3], const float m[3], const float alpha, float* pitch, float* roll, float* yaw);

/// \brief Orientation by linear accelerometer and magnetometer
/// \details Using \b Kalman \b filter
/// \param a Array of linear accelerometer axis
/// \param m Array of magnetic field axis
/// \param Q Process covariance
/// \param R Measurement covariance
/// \param E Error prediction
/// \param pitch Pitch pointer
/// \param roll Roll pointer
/// \param yaw Yaw pointer
/// \return \c 0U if \b pitch, \b roll and \b yaw haz calculated or \c 1U is not calculated (no accumulated data for calculation)
/// \ingroup lsm303algo
uint8_t orientK(const float a[3], const float m[3], const float Q, const float R, const float E, float* pitch, float* roll, float* yaw);

/// \brief Incline angle by linear accelerometer
/// \details Using \b low-pass \b filter
/// \param x X axis
/// \param y Y axis
/// \param z Z axis
/// \param alpha Coefficient of the low-pass filter. Smaller - more smoothing. (1 > a > 0)
/// \param delta Trigger threshold: angle limit in degrees for triggering (used absolute value)
/// \return \c 0.0F if angle less then \c delta or angle in degrees
/// \ingroup lsm303algo
float inclineLP(const float x, const float y, const float z, const float alpha, const float delta);

/// \brief Accelerometer Detection Stage
/// \ingroup lsm303algo
typedef enum {
    STAGE_INIT,             ///< Initial accelerometer stage: motion or rest
    STAGE_WEIGHLESSNESS,    ///< Weighlessness stage
    STAGE_FALL              ///< Fall stage: impact after weighlessness
} stage_t;

/// \brief Fall detection
/// \details Fall detection by accelerometer.
/// \param x X axis
/// \param y Y axis
/// \param z Z axis
/// \param wThs Weighlessness threshould
/// \param iThs Impact threshould
/// \note Use \c wThs == \c 0.0 and \c iThs == \c 0.0 for reset stage after fall detection
/// \return Stage
/// \ingroup lsm303algo
stage_t detectFall(const float x, const float y, const float z, const float wThs, const float iThs);

/// \brief Calculate filter coefficient
/// \param rate Sampling frequency
/// \param cutoff Cutoff frequency
/// \return Filter coefficient
/// \ingroup lsm303algo
float getAlpha(const float rate, const float cutoff);

#endif // __LSM303_ALGO_H__