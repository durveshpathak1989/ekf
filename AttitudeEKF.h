/*
  AttitudeEKF.h
  6-state attitude EKF for the ESP32 quadcopter flight controller.

  State:
    x = [ roll, pitch, yaw, bgx, bgy, bgz ]

  Units:
    state angles: radians
    gyro bias: radians/sec
    inputs gyro: deg/sec
    accel: g
    mag: uT
    outputs: degrees

  Scope:
    - This is an attitude/heading EKF, not a full navigation EKF.
    - It corrects roll/pitch with accelerometer tilt.
    - It corrects yaw with tilt-compensated magnetometer heading.
    - It exposes a clean altitude update hook for BMP280 + VL53L4CX, but
      altitude hold should be a second estimator/controller layer.
*/

#pragma once
#ifndef ATTITUDE_EKF_H
#define ATTITUDE_EKF_H

#include "AHRSCommon.h"

class AttitudeEKF {
public:
    AttitudeEKF();

    void reset();

    void setProcessNoise(float angleQ, float biasQ);
    void setAccelMeasurementNoise(float accelAngleR);
    void setMagMeasurementNoise(float magYawR);
    void setMagDeclinationDeg(float declinationDeg);
    void setMagYawOffsetDeg(float offsetDeg);
    void setMagYawSign(float sign);

    bool update(const AHRSInput& in, float dt, AttitudeEstimate& out);

    float rollBiasDps() const  { return _bgx * AHRS_RAD_TO_DEG; }
    float pitchBiasDps() const { return _bgy * AHRS_RAD_TO_DEG; }
    float yawBiasDps() const   { return _bgz * AHRS_RAD_TO_DEG; }
    bool  lastMagAccepted() const { return _lastMagAccepted; }

    // Future altitude EKF hook. For now it only stores validity and latest readings
    // so the flight controller architecture is ready for BMP + VL53L4CX fusion.
    void updateAltitudeSensors(float bmpAltM, bool bmpValid,
                               float tofAltM, bool tofValid, uint32_t tsMs);

private:
    float _roll;
    float _pitch;
    float _yaw;
    float _bgx;
    float _bgy;
    float _bgz;

    float _cov[6][6];

    float _angleQ;
    float _biasQ;
    float _accelAngleR;
    float _magYawR;
    float _declinationRad;
    float _magYawOffsetRad;
    float _magYawSign;

    bool  _lastMagAccepted;
    float _lastBmpAltM;
    float _lastTofAltM;
    bool  _lastBmpValid;
    bool  _lastTofValid;
    uint32_t _lastAltTsMs;

    void _predictCovariance(float dt);
    void _updateScalar(int measIndex, float measurementRad, float R);
    bool _magYawRad(const AHRSInput& in, float& yawRad) const;
    static void _quatFromEuler(float roll, float pitch, float yaw, AttitudeEstimate& out);
};

#endif // ATTITUDE_EKF_H
