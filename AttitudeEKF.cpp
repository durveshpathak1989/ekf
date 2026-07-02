/*
 * Name: AttitudeEKF.cpp
 * Use: Implementation of the full attitude EKF used by the flight controller.
 * Version: 4.0.0
 * Created by: Durvesh Pathak dp676@cornell.edu
 */

#include "AttitudeEKF.h"

static constexpr float EKF_GRAVITY_MPS2 = 9.80665f;

static float ekfClampFloat(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

AttitudeEKF::AttitudeEKF()
{
    _angleQ = 0.0008f;       // process noise per 400 Hz update, tune from logs
    _biasQ = 0.000001f;      // slow gyro-bias wander
    _accelAngleR = 0.06f;    // larger = trust accel less under vibration
    _magYawR = 0.20f;        // larger = trust mag less near motors/wires
    _declinationRad = 0.0f;  // set local declination later if you want true north
    _magYawOffsetRad = 0.0f; // set after bench heading test if needed
    _magYawSign = 1.0f;      // change to -1 if yaw correction moves the wrong way
    _altAccelQ = 0.35f;      // vertical accel/process uncertainty
    _baroAltR = 1.25f;       // BMP280 is broad/noisy but long-range
    _tofAltR = 0.025f;       // VL53L4CX is strong close to ground
    reset();
}

void AttitudeEKF::reset()
{
    _roll = _pitch = _yaw = 0.0f;
    _bgx = _bgy = _bgz = 0.0f;
    _lastMagAccepted = false;
    resetPositionVelocity();
    _lastBmpAltM = _lastTofAltM = 0.0f;
    _lastBmpValid = _lastTofValid = false;
    _lastAltTsMs = 0;

    for (int r = 0; r < 6; r++) {
        for (int c = 0; c < 6; c++) _cov[r][c] = 0.0f;
    }

    _cov[0][0] = 0.05f; // roll uncertainty
    _cov[1][1] = 0.05f; // pitch uncertainty
    _cov[2][2] = 0.20f; // yaw uncertainty
    _cov[3][3] = 0.10f; // gx bias uncertainty
    _cov[4][4] = 0.10f; // gy bias uncertainty
    _cov[5][5] = 0.10f; // gz bias uncertainty
}

void AttitudeEKF::resetPositionVelocity()
{
    _posVel = PositionVelocityEstimate();
    _altZ = 0.0f;
    _altVz = 0.0f;
    _altP00 = 4.0f;
    _altP01 = 0.0f;
    _altP10 = 0.0f;
    _altP11 = 1.0f;
    _baroRefAltM = 0.0f;
    _altValid = false;
    _baroRefValid = false;
    _altLastUpdateMs = 0;
}

void AttitudeEKF::setProcessNoise(float angleQ, float biasQ)
{
    _angleQ = angleQ;
    _biasQ = biasQ;
}

void AttitudeEKF::setAccelMeasurementNoise(float accelAngleR)
{
    _accelAngleR = accelAngleR;
}

void AttitudeEKF::setMagMeasurementNoise(float magYawR)
{
    _magYawR = magYawR;
}

void AttitudeEKF::setMagDeclinationDeg(float declinationDeg)
{
    _declinationRad = declinationDeg * AHRS_DEG_TO_RAD;
}

void AttitudeEKF::setMagYawOffsetDeg(float offsetDeg)
{
    _magYawOffsetRad = offsetDeg * AHRS_DEG_TO_RAD;
}

void AttitudeEKF::setMagYawSign(float sign)
{
    _magYawSign = (sign < 0.0f) ? -1.0f : 1.0f;
}

void AttitudeEKF::setAltitudeEstimatorNoise(float accelQ, float baroAltR, float tofAltR)
{
    _altAccelQ = ekfClampFloat(accelQ, 0.0001f, 20.0f);
    _baroAltR = ekfClampFloat(baroAltR, 0.001f, 100.0f);
    _tofAltR = ekfClampFloat(tofAltR, 0.0001f, 10.0f);
}

void AttitudeEKF::_predictCovariance(float dt)
{
    // Model for each angle/bias pair:
    // angle_k = angle + dt * (gyro - bias)
    // bias_k  = bias
    // F pair = [ 1  -dt ]
    //          [ 0    1 ]
    for (int axis = 0; axis < 3; axis++) {
        const int a = axis;      // 0 roll, 1 pitch, 2 yaw
        const int b = axis + 3;  // 3 bgx, 4 bgy, 5 bgz

        const float Paa = _cov[a][a];
        const float Pab = _cov[a][b];
        const float Pba = _cov[b][a];
        const float Pbb = _cov[b][b];

        _cov[a][a] = Paa - dt*Pba - dt*Pab + dt*dt*Pbb + _angleQ;
        _cov[a][b] = Pab - dt*Pbb;
        _cov[b][a] = Pba - dt*Pbb;
        _cov[b][b] = Pbb + _biasQ;
    }
}

void AttitudeEKF::_updateScalar(int measIndex, float measurementRad, float R)
{
    // Measurement model: z = selected_angle + noise
    float* anglePtr = nullptr;
    if      (measIndex == 0) anglePtr = &_roll;
    else if (measIndex == 1) anglePtr = &_pitch;
    else                     anglePtr = &_yaw;

    float innovation = measurementRad - *anglePtr;
    if (measIndex == 2) {
        innovation = ahrsWrap180(innovation * AHRS_RAD_TO_DEG) * AHRS_DEG_TO_RAD;
    }

    const float S = _cov[measIndex][measIndex] + R;
    if (S <= 1e-9f) return;

    float K[6];
    for (int i = 0; i < 6; i++) K[i] = _cov[i][measIndex] / S;

    _roll  += K[0] * innovation;
    _pitch += K[1] * innovation;
    _yaw   += K[2] * innovation;
    _bgx   += K[3] * innovation;
    _bgy   += K[4] * innovation;
    _bgz   += K[5] * innovation;

    float oldRow[6];
    for (int j = 0; j < 6; j++) oldRow[j] = _cov[measIndex][j];

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            _cov[i][j] -= K[i] * oldRow[j];
        }
    }
}

void AttitudeEKF::_updatePositionVelocity(const AHRSInput& in, float dt)
{
    const float a2 = in.ax_g*in.ax_g + in.ay_g*in.ay_g + in.az_g*in.az_g;
    if (a2 <= 1e-6f) {
        _posVel.valid = false;
        return;
    }

    const float cr = cosf(_roll);
    const float sr = sinf(_roll);
    const float cp = cosf(_pitch);
    const float sp = sinf(_pitch);
    const float cy = cosf(_yaw);
    const float sy = sinf(_yaw);

    // Rotate body-frame accelerometer into a local world frame:
    // X forward/north-ish, Y right/east-ish, Z up. Accelerometer includes
    // gravity, so subtract +1g from world Z before integrating.
    const float axWorldG = (cy*cp) * in.ax_g
                         + (cy*sp*sr - sy*cr) * in.ay_g
                         + (cy*sp*cr + sy*sr) * in.az_g;
    const float ayWorldG = (sy*cp) * in.ax_g
                         + (sy*sp*sr + cy*cr) * in.ay_g
                         + (sy*sp*cr - cy*sr) * in.az_g;
    const float azWorldG = (-sp) * in.ax_g
                         + (cp*sr) * in.ay_g
                         + (cp*cr) * in.az_g;

    _posVel.accelX_mps2 = axWorldG * EKF_GRAVITY_MPS2;
    _posVel.accelY_mps2 = ayWorldG * EKF_GRAVITY_MPS2;
    _posVel.accelZ_mps2 = (azWorldG - 1.0f) * EKF_GRAVITY_MPS2;

    _posVel.velX_mps += _posVel.accelX_mps2 * dt;
    _posVel.velY_mps += _posVel.accelY_mps2 * dt;
    if (!_altValid) {
        _posVel.velZ_mps += _posVel.accelZ_mps2 * dt;
        _posVel.posZ_m += _posVel.velZ_mps * dt;
    }

    _posVel.posX_m += _posVel.velX_mps * dt;
    _posVel.posY_m += _posVel.velY_mps * dt;
    _posVel.valid = true;
}

void AttitudeEKF::_predictAltitude(float dt)
{
    const float az = _posVel.accelZ_mps2;

    _altZ += _altVz * dt + 0.5f * az * dt * dt;
    _altVz += az * dt;

    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt2 * dt2;

    const float q00 = 0.25f * dt4 * _altAccelQ;
    const float q01 = 0.5f * dt3 * _altAccelQ;
    const float q10 = q01;
    const float q11 = dt2 * _altAccelQ;

    const float p00 = _altP00 + dt * (_altP10 + _altP01) + dt2 * _altP11 + q00;
    const float p01 = _altP01 + dt * _altP11 + q01;
    const float p10 = _altP10 + dt * _altP11 + q10;
    const float p11 = _altP11 + q11;

    _altP00 = p00;
    _altP01 = p01;
    _altP10 = p10;
    _altP11 = p11;
}

void AttitudeEKF::_updateAltitudeScalar(float measurementM, float R)
{
    const float S = _altP00 + R;
    if (S <= 1e-9f) return;

    const float innovation = measurementM - _altZ;
    const float k0 = _altP00 / S;
    const float k1 = _altP10 / S;

    _altZ += k0 * innovation;
    _altVz += k1 * innovation;

    const float p00 = _altP00;
    const float p01 = _altP01;
    const float p10 = _altP10;
    const float p11 = _altP11;

    _altP00 = p00 - k0 * p00;
    _altP01 = p01 - k0 * p01;
    _altP10 = p10 - k1 * p00;
    _altP11 = p11 - k1 * p01;

    if (_altP00 < 1e-6f) _altP00 = 1e-6f;
    if (_altP11 < 1e-6f) _altP11 = 1e-6f;
}

bool AttitudeEKF::_magYawRad(const AHRSInput& in, float& yawRad) const
{
    const float m2 = in.mx_uT*in.mx_uT + in.my_uT*in.my_uT + in.mz_uT*in.mz_uT;
    if (!in.magValid || m2 < 1.0f) return false;

    const float cr = cosf(_roll);
    const float sr = sinf(_roll);
    const float cp = cosf(_pitch);
    const float sp = sinf(_pitch);

    // Tilt-compensated magnetometer heading. This is intentionally isolated
    // here because board orientation/sign may need one bench correction.
    const float Xh = in.mx_uT * cp + in.mz_uT * sp;
    const float Yh = in.mx_uT * sr * sp + in.my_uT * cr - in.mz_uT * sr * cp;

    yawRad = atan2f(-Yh, Xh);
    yawRad = _magYawSign * yawRad + _declinationRad + _magYawOffsetRad;
    yawRad = ahrsWrap180(yawRad * AHRS_RAD_TO_DEG) * AHRS_DEG_TO_RAD;
    return true;
}

bool AttitudeEKF::update(const AHRSInput& in, float dt, AttitudeEstimate& out)
{
    if (dt <= 0.0f || dt > 0.05f) dt = 0.0025f;

    const float gx = in.gx_dps * AHRS_DEG_TO_RAD;
    const float gy = in.gy_dps * AHRS_DEG_TO_RAD;
    const float gz = in.gz_dps * AHRS_DEG_TO_RAD;

    // Prediction: gyro integration minus estimated gyro bias.
    _roll  += (gx - _bgx) * dt;
    _pitch += (gy - _bgy) * dt;
    _yaw   += (gz - _bgz) * dt;
    _yaw = ahrsWrap180(_yaw * AHRS_RAD_TO_DEG) * AHRS_DEG_TO_RAD;

    _predictCovariance(dt);

    // Measurement update 1: accelerometer gives roll and pitch. As accel norm
    // moves away from 1g, increase R so transient acceleration is trusted less.
    const float a2 = in.ax_g*in.ax_g + in.ay_g*in.ay_g + in.az_g*in.az_g;
    if (a2 > 1e-6f) {
        const float accelNorm = sqrtf(a2);
        const float accelErr = fabsf(accelNorm - 1.0f);

        float activeAccelR = _accelAngleR;
        if (accelErr < 0.10f) {
            activeAccelR = _accelAngleR;
        } else if (accelErr < 0.20f) {
            activeAccelR = fmaxf(_accelAngleR, 0.15f);
        } else if (accelErr < 0.35f) {
            activeAccelR = fmaxf(_accelAngleR, 0.50f);
        } else {
            activeAccelR = 2.0f;
        }

        float accRollDeg = 0.0f, accPitchDeg = 0.0f;
        ahrsAccelAnglesDeg(in.ax_g, in.ay_g, in.az_g, accRollDeg, accPitchDeg);
        _updateScalar(0, accRollDeg  * AHRS_DEG_TO_RAD, activeAccelR);
        _updateScalar(1, accPitchDeg * AHRS_DEG_TO_RAD, activeAccelR);
    }

    // Measurement update 2: magnetometer gives absolute yaw, correcting yaw drift.
    _lastMagAccepted = false;
    float magYaw = 0.0f;
    if (_magYawRad(in, magYaw)) {
        _updateScalar(2, magYaw, _magYawR);
        _lastMagAccepted = true;
    }

    _yaw = ahrsWrap180(_yaw * AHRS_RAD_TO_DEG) * AHRS_DEG_TO_RAD;
    _updatePositionVelocity(in, dt);

    _quatFromEuler(_roll, _pitch, _yaw, out);
    out.roll_deg  = _roll  * AHRS_RAD_TO_DEG;
    out.pitch_deg = _pitch * AHRS_RAD_TO_DEG;
    out.yaw_deg   = ahrsWrap360(_yaw * AHRS_RAD_TO_DEG);
    return true;
}

void AttitudeEKF::updateAltitudeSensors(float bmpAltM, bool bmpValid,
                                        float tofAltM, bool tofValid, uint32_t tsMs)
{
    const bool haveMeasurement = bmpValid || tofValid;
    if (_altValid && _altLastUpdateMs != 0 && tsMs > _altLastUpdateMs) {
        const float dt = (float)(tsMs - _altLastUpdateMs) * 0.001f;
        if (dt > 0.0f && dt < 1.0f) {
            _predictAltitude(dt);
        }
    }

    if (!haveMeasurement) {
        if (_altValid) {
            _posVel.posZ_m = _altZ;
            _posVel.velZ_mps = _altVz;
            _posVel.altitudeValid = true;
            _posVel.valid = true;
            _altLastUpdateMs = tsMs;
        }
        _lastBmpAltM = bmpAltM;
        _lastTofAltM = tofAltM;
        _lastBmpValid = false;
        _lastTofValid = false;
        return;
    }

    if (!_altValid) {
        if (tofValid) {
            _altZ = tofAltM;
            _altVz = 0.0f;
        } else {
            _altZ = 0.0f;
            _altVz = 0.0f;
        }
        _altP00 = tofValid ? _tofAltR : _baroAltR;
        _altP01 = 0.0f;
        _altP10 = 0.0f;
        _altP11 = 1.0f;
        _altValid = true;
    }

    if (bmpValid && !_baroRefValid) {
        _baroRefAltM = bmpAltM - _altZ;
        _baroRefValid = true;
    }

    if (bmpValid && _baroRefValid) {
        _updateAltitudeScalar(bmpAltM - _baroRefAltM, _baroAltR);
    }
    if (tofValid) {
        _updateAltitudeScalar(tofAltM, _tofAltR);
    }

    _posVel.posZ_m = _altZ;
    _posVel.velZ_mps = _altVz;
    _posVel.altitudeValid = _altValid;
    _posVel.valid = true;

    _lastBmpAltM = bmpAltM;
    _lastTofAltM = tofAltM;
    _lastBmpValid = bmpValid;
    _lastTofValid = tofValid;
    _lastAltTsMs = tsMs;
    _altLastUpdateMs = tsMs;
}

void AttitudeEKF::_quatFromEuler(float roll, float pitch, float yaw, AttitudeEstimate& out)
{
    const float cr = cosf(roll * 0.5f);
    const float sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f);
    const float sp = sinf(pitch * 0.5f);
    const float cy = cosf(yaw * 0.5f);
    const float sy = sinf(yaw * 0.5f);

    out.q0 = cr*cp*cy + sr*sp*sy;
    out.q1 = sr*cp*cy - cr*sp*sy;
    out.q2 = cr*sp*cy + sr*cp*sy;
    out.q3 = cr*cp*sy - sr*sp*cy;
}
