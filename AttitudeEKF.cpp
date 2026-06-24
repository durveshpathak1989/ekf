#include "AttitudeEKF.h"

AttitudeEKF::AttitudeEKF()
{
    _angleQ = 0.0008f;       // process noise per 400 Hz update, tune from logs
    _biasQ = 0.000001f;      // slow gyro-bias wander
    _accelAngleR = 0.06f;    // larger = trust accel less under vibration
    _magYawR = 0.20f;        // larger = trust mag less near motors/wires
    _declinationRad = 0.0f;  // set local declination later if you want true north
    _magYawOffsetRad = 0.0f; // set after bench heading test if needed
    _magYawSign = 1.0f;      // change to -1 if yaw correction moves the wrong way
    reset();
}

void AttitudeEKF::reset()
{
    _roll = _pitch = _yaw = 0.0f;
    _bgx = _bgy = _bgz = 0.0f;
    _lastMagAccepted = false;
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

    // Measurement update 1: accelerometer gives roll and pitch when accel norm is sane.
    const float a2 = in.ax_g*in.ax_g + in.ay_g*in.ay_g + in.az_g*in.az_g;
    const bool accelUsable = (a2 > 0.64f && a2 < 1.44f); // reject high vibration/accel spikes
    if (accelUsable) {
        float accRollDeg = 0.0f, accPitchDeg = 0.0f;
        ahrsAccelAnglesDeg(in.ax_g, in.ay_g, in.az_g, accRollDeg, accPitchDeg);
        _updateScalar(0, accRollDeg  * AHRS_DEG_TO_RAD, _accelAngleR);
        _updateScalar(1, accPitchDeg * AHRS_DEG_TO_RAD, _accelAngleR);
    }

    // Measurement update 2: magnetometer gives absolute yaw, correcting yaw drift.
    _lastMagAccepted = false;
    float magYaw = 0.0f;
    if (_magYawRad(in, magYaw)) {
        _updateScalar(2, magYaw, _magYawR);
        _lastMagAccepted = true;
    }

    _yaw = ahrsWrap180(_yaw * AHRS_RAD_TO_DEG) * AHRS_DEG_TO_RAD;

    _quatFromEuler(_roll, _pitch, _yaw, out);
    out.roll_deg  = _roll  * AHRS_RAD_TO_DEG;
    out.pitch_deg = _pitch * AHRS_RAD_TO_DEG;
    out.yaw_deg   = ahrsWrap360(_yaw * AHRS_RAD_TO_DEG);
    return true;
}

void AttitudeEKF::updateAltitudeSensors(float bmpAltM, bool bmpValid,
                                        float tofAltM, bool tofValid, uint32_t tsMs)
{
    _lastBmpAltM = bmpAltM;
    _lastTofAltM = tofAltM;
    _lastBmpValid = bmpValid;
    _lastTofValid = tofValid;
    _lastAltTsMs = tsMs;
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
