/// \file lsm303algo.c
/// \brief This file is part of LSM303DLHC Library for STM32 Nucleo L4
/// \copyright &copy; https://github.com/Ilushenko Oleksandr Ilushenko
///	\author Oleksandr Ilushenko
///	\date 2024
#include "lsm303algo.h"
#include "log.h"

#include <math.h>

#ifndef M_PI
# define M_PI 3.14159265358979323846F
#endif

#ifndef DEG2RAD
# define DEG2RAD 0.017453292519943295769236907684886F
#endif

#ifndef RAD2DEG
# define RAD2DEG 57.295779513082320876798154814105F
#endif

float motionLP(const float x, const float y, const float z, const float alpha, const float delta, const uint8_t sample)
{
    static uint8_t setup = 0;
    static uint8_t smpl = 0;
    // preview data
    static float pX = 0.0;
    static float pY = 0.0;
    static float pZ = 0.0;
    // filtered data
    static float fX = 0.0;
    static float fY = 0.0;
    static float fZ = 0.0;
    // first iteration
    if (setup == 0) {
        fX = x;
        fY = y;
        fZ = z;
        setup++;
        return 0.0F;
    }
    // Low-pass filter
    fX = alpha * x + (1.0F - alpha) * fX;
    fY = alpha * y + (1.0F - alpha) * fY;
    fZ = alpha * z + (1.0F - alpha) * fZ;
    // Accumulation
    if (setup < CNTSETUP) {
        pX = fX;
        pY = fY;
        pZ = fZ;
        setup++;
        return 0.0F;
    }
    // Samples
    if (smpl++ < sample) return 0.0;
    smpl = 0;
    // Delta
    const float dX = fabsf(fX - pX);
    const float dY = fabsf(fY - pY);
    const float dZ = fabsf(fZ - pZ);
    // Magnitude
    const float m = sqrtf(dX * dX + dY * dY + dZ * dZ);
    if (m > delta) {
        setup = 0;
        xDebug("%f, %f, %f\tD: %f\n", x, y, z, m);
        return m;
    }
    return 0.0F;
}

float motionK(const float x, const float y, const float z, const float Q, const float R, const float E, const float delta, const uint8_t sample)
{
    static uint8_t setup = 0;
    static uint8_t smpl = 0;
    // Prediction estimate (filtered data)
    static float fX = 0.0;
    static float fY = 0.0;
    static float fZ = 0.0;
    // Prediction error
    static float eX = 0.0;
    static float eY = 0.0;
    static float eZ = 0.0;
    // preview data
    static float pX = 0.0;
    static float pY = 0.0;
    static float pZ = 0.0;
    // first iteration
    if (setup == 0) {
        // Prediction estimate
        fX = x;
        fY = y;
        fZ = z;
        // Prediction error
        eX = eY = eZ = E;
        setup++;
        return 0.0F;
    }
    // Kalman filter X
    eX += Q;                // Prediction of a new error
    float K = eX / (eX + R);// Calculation of the Kalman coefficient
    fX += K * (x - fX);     // Estimate update
    eX *= (1.0 - K);        // Error update
    // Kalman filter Y
    eY += Q;                // Prediction of a new error
    K = eY / (eY + R);      // Calculation of the Kalman coefficient
    fY += K * (y - fY);     // Estimate update
    eY *= (1.0 - K);        // Error update
    // Kalman filter Z
    eZ += Q;                // Prediction of a new error
    K = eZ / (eZ + R);      // Calculation of the Kalman coefficient
    fZ += K * (z - fZ);     // Estimate update
    eZ *= (1.0 - K);        // Error update
    //xTrace("%f, %f, %f, %f, %f, %f\n", x, y, z, fX + 10.0, fY + 10.0, fZ + 10.0);
    // Accumulation
    if (setup < CNTSETUP) {
        pX = fX;
        pY = fY;
        pZ = fZ;
        setup++;
        return 0.0F;
    }
    // Samples
    if (smpl++ < sample) return 0.0;
    smpl = 0;
    // Delta
    const float dX = fabsf(fX - pX);
    const float dY = fabsf(fY - pY);
    const float dZ = fabsf(fZ - pZ);
    // Magnitude
    const float m = sqrtf(dX * dX + dY * dY + dZ * dZ);
    if (m > delta && m < 1.0) {
        setup = 0;
        xDebug("%f, %f, %f\tD: %f\n", x, y, z, m);
        return m;
    }
    return 0.0F;
}

float distortionHP(const float x, const float y, const float z, const float alpha, const float delta)
{
    static uint8_t setup = 0;
    // prev input
    static float iX = 0.0F;
    static float iY = 0.0F;
    static float iZ = 0.0F;
    // prev output
    static float oX = 0.0F;
    static float oY = 0.0F;
    static float oZ = 0.0F;
    // prev magnitude
    static float M = 0.0F;
    // high-pass filter
    oX = alpha * (oX + x - iX);
    oY = alpha * (oY + y - iY);
    oZ = alpha * (oZ + z - iZ);
    iX = x;
    iY = y;
    iZ = z;
    // difference
    const float dX = iX - oX;
    const float dY = iY - oY;
    const float dZ = iZ - oZ;
    // magnitude
    const float m = sqrtf(dX * dX + dY * dY + dZ * dZ);
    // average magnitude
    if (setup == 0U) {
        M = m;
        setup++;
		return 0.0F;
    }
    // magnitude low-pass filter
    if (setup < CNTSETUP) {
        M = alpha * m + (1.0F - alpha) * M;
        setup++;
        return 0.0F;
    }
    // check
    const float d = fabsf(M - m);
    if (d > delta) {
        xDebug("%f, %f, %f\tM: %f m: %f D: %f\n", x, y, z, M, m, d);
        setup = 0;
        return d;
    }
    return 0.0F;
}

float distortionLP(const float x, const float y, const float z, const float alpha, const float delta)
{
    static uint8_t setup = 0;
    // average
    static float aX = 0.0F;
    static float aY = 0.0F;
    static float aZ = 0.0F;
    // first iteration
    if (setup == 0) {
        aX = x;
        aY = y;
        aZ = z;
        setup++;
        return 0.0F;
    }
    // difference
    const float dX = x - aX;
    const float dY = y - aY;
    const float dZ = z - aZ;
    // low-pass
    aX = alpha * x + (1.0F - alpha) * aX;
    aY = alpha * y + (1.0F - alpha) * aY;
    aZ = alpha * z + (1.0F - alpha) * aZ;
    // accumulate
    if (setup < CNTSETUP) {
        setup++;
        return 0.0F;
    }
    // magnitude
    const float m = sqrtf(dX * dX + dY * dY + dZ * dZ);
    if (m > delta) {
        xDebug("%f, %f, %f\tD: %f\n", x, y, z, m);
        setup = 0;
        return m;
    }
    return 0.0F;
}

uint8_t orientLP(const float a[3], const float m[3], const float alpha, float* pitch, float* roll, float* yaw)
{
    enum { X, Y, Z };
    static uint8_t setup = 0U;
    // filtered data
    static float aX = 0.0;
    static float aY = 0.0;
    static float aZ = 0.0;
    static float mX = 0.0;
    static float mY = 0.0;
    static float mZ = 0.0;
    // first iteration
    if (setup == 0U) {
        aX = a[X];
        aY = a[Y];
        aZ = a[Z];
        mX = m[X];
        mY = m[Y];
        mZ = m[Z];
        setup++;
        return 1U;
    }
    // Low-pass filter
    aX = alpha * a[X] + (1.0F - alpha) * aX;
    aY = alpha * a[Y] + (1.0F - alpha) * aY;
    aZ = alpha * a[Z] + (1.0F - alpha) * aZ;
    mX = alpha * m[X] + (1.0F - alpha) * mX;
    mY = alpha * m[Y] + (1.0F - alpha) * mY;
    mZ = alpha * m[Z] + (1.0F - alpha) * mZ;
    // accumulate
    if (setup < CNTSETUP) {
        setup++;
        return 1U;
    }
    // pitch & roll
    *pitch = atan2f(aX, sqrtf(aY * aY + aZ * aZ)) * RAD2DEG;
    *roll = atan2f(aY, sqrtf(aX * aX + aZ * aZ)) * RAD2DEG;
    // normalize accelerometer
    float N = sqrtf(aX * aX + aY * aY + aZ * aZ);
    aX /= N;
    aY /= N;
    aZ /= N;
    // normalize magnetometer
    N = sqrtf(mX * mX + mY * mY + mZ * mZ);
    mX /= N;
    mY /= N;
    mZ /= N;
    // magnetic field horizontal projection
    const float Mx = mX * aZ - mZ * aX;
    const float My = mY * aZ - mZ * aY;
    // yaw
    *yaw = atan2f(My, Mx) * RAD2DEG;
    xDebug("Pitch: %.02f°, Roll: %.02f°, Yaw: %.02f°\n", *pitch, *roll, *yaw);
    return 0U;
}

uint8_t orientK(const float a[3], const float m[3], const float Q, const float R, const float E, float *pitch, float *roll, float *yaw)
{
    enum { X, Y, Z };
    static uint8_t setup = 0U;
    // Prediction estimate (filtered data)
    static float fA[3] = { 0 };
    static float fM[3] = { 0 };
    // Prediction error
    static float eA[3] = { 0 };
    static float eM[3] = { 0 };
    // First iteration
    if (setup == 0U) {
        // Prediction estimate
        fA[X] = a[X];
        fA[Y] = a[Y];
        fA[Z] = a[Z];
        fM[X] = m[X];
        fM[Y] = m[Y];
        fM[Z] = m[Z];
        // Prediction error
        eA[X] = eA[Y] = eA[Z] = E;
        eM[X] = eM[Y] = eM[Z] = E;
        setup++;
        return 1U;
    }
    // Kalman filter
    float N = 0.0F;
    for (uint8_t i = 0; i < 3; ++i) {
        // accelerometer
        eA[i] += Q;                     // Prediction of a new error
        N = eA[i] / (eA[i] + R);        // Calculation of the Kalman coefficient
        fA[i] += N * (a[i] - fA[i]);    // Estimate update
        eA[i] *= (1.0 - N);             // Error update
        // magnetometer
        eM[i] += Q;                     // Prediction of a new error
        N = eM[i] / (eM[i] + R);        // Calculation of the Kalman coefficient
        fM[i] += N * (m[i] - fM[i]);    // Estimate update
        eM[i] *= (1.0 - N);             // Error update
    }
    // accumulate
    if (setup < CNTSETUP) {
        setup++;
        return 1U;
    }
    // pitch & roll
    *pitch = atan2f(fA[X], sqrtf(fA[Y] * fA[Y] + fA[Z] * fA[Z])) * RAD2DEG;
    *roll = atan2f(fA[Y], sqrtf(fA[X] * fA[X] + fA[Z] * fA[Z])) * RAD2DEG;
    // normalize accelerometer
    N = sqrtf(fA[X] * fA[X] + fA[Y] * fA[Y] + fA[Z] * fA[Z]);
    fA[X] /= N;
    fA[Y] /= N;
    fA[Z] /= N;
    // normalize magnetometer
    N = sqrtf(fM[X] * fM[X] + fM[Y] * fM[Y] + fM[Z] * fM[Z]);
    fM[X] /= N;
    fM[Y] /= N;
    fM[Z] /= N;
    // magnetic field horizontal projection
    const float Mx = fM[X] * fA[Z] - fM[Z] * fA[X];
    const float My = fM[Y] * fA[Z] - fM[Z] * fA[Y];
    // yaw
    *yaw = atan2f(My, Mx) * RAD2DEG;
    xDebug("Pitch: %.02f°, Roll: %.02f°, Yaw: %.02f°\n", *pitch, *roll, *yaw);
    return 0U;
}

float inclineLP(const float x, const float y, const float z, const float alpha, const float delta)
{
    static uint8_t setup = 0U;
    // filtered data
    static float aX = 0.0;
    static float aY = 0.0;
    static float aZ = 0.0;
    // first iteration
    if (setup == 0U) {
        aX = x;
        aY = y;
        aZ = z;
        setup++;
        return 0.0F;
    }
    // Low-pass filter
    aX = alpha * x + (1.0F - alpha) * aX;
    aY = alpha * y + (1.0F - alpha) * aY;
    aZ = alpha * z + (1.0F - alpha) * aZ;
    // accumulate
    if (setup < CNTSETUP) {
        setup++;
        return 0.0;
    }
    // angle
    const float theta = acosf(aZ / sqrtf(aX * aX + aY *aY + aZ * aZ)) * RAD2DEG;
    if (theta > fabsf(delta)) {
        xDebug("%f, %f, %f\tA: %.02f°\n", x, y, z, theta);
        setup = 0;
        return theta;
    }
    return 0.0F;
}

stage_t detectFall(const float x, const float y, const float z, const float wThs, const float iThs)
{
    static stage_t stage = STAGE_INIT;
    static uint8_t smpl = 0;
    const float M = sqrtf(x * x + y * y + z * z);
    switch (stage) {
    case STAGE_INIT:
        if (M < wThs) {
            stage = STAGE_WEIGHLESSNESS;
            xDebug("WEIGHLESSNESS: %f\n", M);
        }
        break;
    case STAGE_WEIGHLESSNESS:
        if (M > iThs) {
            stage = STAGE_FALL;
            xDebug("FALL: %f\n", M);
        }
        break;
    case STAGE_FALL:
        if (wThs + iThs == 0.0) {
            stage = STAGE_INIT;
            xDebug("Reset Stage to INIT\n");
        }
    }
    return stage;
}

float getAlpha(const float rate, const float cutoff)
{
    const float rc = 1.0 / (2.0 * M_PI * cutoff);
    const float dt = 1.0 / rate;
    return dt / (rc + dt);
}
