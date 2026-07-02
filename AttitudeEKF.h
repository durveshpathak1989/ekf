/*
 * Name: AttitudeEKF.h
 * Use: Declaration for the full attitude EKF used by the flight controller.
 * Version: 4.0.0
 * Created by: Durvesh Pathak dp676@cornell.edu
 */

/*
  AttitudeEKF.h
  6-state attitude EKF for the ESP32 quadcopter flight controller, with a
  separate dead-reckoned position/velocity estimate for logging/future work.

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
    - Position/velocity are estimated separately and are not used by flight
      control; without GPS/optical-flow fusion they will drift.
    - It fuses BMP280 + VL53L4CX into a separate vertical EKF state
      for height and vertical speed. Altitude hold remains a controller layer.
*/

#pragma once
#ifndef ATTITUDE_EKF_H
#define ATTITUDE_EKF_H

#include "../AHRS/AHRSCommon.h"

struct PositionVelocityEstimate {
    float posX_m = 0.0f;
    float posY_m = 0.0f;
    float posZ_m = 0.0f;

    float velX_mps = 0.0f;
    float velY_mps = 0.0f;
    float velZ_mps = 0.0f;

    float accelX_mps2 = 0.0f;
    float accelY_mps2 = 0.0f;
    float accelZ_mps2 = 0.0f;

    bool valid = false;
    bool altitudeValid = false;
};

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
    void setAltitudeEstimatorNoise(float accelQ, float baroAltR, float tofAltR);

    bool update(const AHRSInput& in, float dt, AttitudeEstimate& out);

    float rollBiasDps() const  { return _bgx * AHRS_RAD_TO_DEG; }
    float pitchBiasDps() const { return _bgy * AHRS_RAD_TO_DEG; }
    float yawBiasDps() const   { return _bgz * AHRS_RAD_TO_DEG; }
    bool  lastMagAccepted() const { return _lastMagAccepted; }
    const PositionVelocityEstimate& positionVelocity() const { return _posVel; }
    bool altitudeValid() const { return _altValid; }

    void resetPositionVelocity();

    // Vertical EKF update. bmpAltM is converted to local relative height using
    // a reference captured at startup; tofAltM should already be tilt-compensated
    // height above ground when the ToF sample is valid.
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
    PositionVelocityEstimate _posVel;

    float _altZ;
    float _altVz;
    float _altP00;
    float _altP01;
    float _altP10;
    float _altP11;
    float _altAccelQ;
    float _baroAltR;
    float _tofAltR;
    float _baroRefAltM;
    bool  _altValid;
    bool  _baroRefValid;
    uint32_t _altLastUpdateMs;

    float _lastBmpAltM;
    float _lastTofAltM;
    bool  _lastBmpValid;
    bool  _lastTofValid;
    uint32_t _lastAltTsMs;

    void _predictCovariance(float dt);
    void _updateScalar(int measIndex, float measurementRad, float R);
    void _updatePositionVelocity(const AHRSInput& in, float dt);
    void _predictAltitude(float dt);
    void _updateAltitudeScalar(float measurementM, float R);
    bool _magYawRad(const AHRSInput& in, float& yawRad) const;
    static void _quatFromEuler(float roll, float pitch, float yaw, AttitudeEstimate& out);
};

#endif // ATTITUDE_EKF_H
