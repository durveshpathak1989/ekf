# Test Quad Attitude EKF Library

This repository contains `AttitudeEKF`, the full attitude estimator used as the default filter in the flight controller. It estimates roll, pitch, yaw, and gyro bias using gyro prediction plus accelerometer and magnetometer correction.

## Pin Map

The EKF has no direct pins. It consumes `AHRSInput` values from the IMU driver:

| Signal | ESP32 pin | Notes |
| --- | ---: | --- |
| SPI SCK | GPIO 5 | MPU-9250/MPU-6500 clock |
| SPI MISO | GPIO 19 | MPU data to ESP32 |
| SPI MOSI | GPIO 18 | ESP32 data to MPU |
| MPU CS | GPIO 33 | Chip select passed to `MPU9250 imu(PIN_MPU_CS)` |
| MPU INT | GPIO 27 | Optional data-ready interrupt; current firmware does not require it |
| Motor FL | GPIO 25 | Front-left ESC signal |
| Motor FR | GPIO 15 | Front-right ESC signal |
| Motor RL | GPIO 14 | Rear-left ESC signal |
| Motor RR | GPIO 32 | Rear-right ESC signal |
| iBUS RX | GPIO 16 | FS-iA6B iBUS TX into ESP32 UART2 RX |
| iBUS TX | GPIO 4 | Spare UART TX; avoids GPIO17 GPS conflict |
| I2C SDA | GPIO 21 | BMP280 and VL53L4CX ToF bus |
| I2C SCL | GPIO 22 | BMP280 and VL53L4CX ToF bus |
| GPS RX | GPIO 13 | GPS TXD into ESP32 UART1 RX |
| GPS TX | GPIO 17 | Optional GPS RXD from ESP32 UART1 TX |


## Main INO Integration Example

```cpp
#include "AttitudeEKF.h"

AttitudeEKF attitudeEKF;

void setup() {
    attitudeEKF.setProcessNoise(0.0008f, 0.000001f);
    attitudeEKF.setAccelMeasurementNoise(0.060f);
    attitudeEKF.setMagMeasurementNoise(0.200f);
    attitudeEKF.setMagDeclinationDeg(0.0f);
    attitudeEKF.setMagYawOffsetDeg(0.0f);
    attitudeEKF.setMagYawSign(1.0f);
}

void updateAttitude(const MPU_SensorData& sf, float dt) {
    AHRSInput in;
    in.ax_g = sf.ax_g;     in.ay_g = sf.ay_g;     in.az_g = sf.az_g;
    in.gx_dps = sf.gx_dps; in.gy_dps = sf.gy_dps; in.gz_dps = sf.gz_dps;
    in.mx_uT = sf.mx_uT;   in.my_uT = sf.my_uT;   in.mz_uT = sf.mz_uT;
    in.magValid = imu.isMagConnected();

    AttitudeEstimate att;
    if (attitudeEKF.update(in, dt, att)) {
        // Use att.roll_deg, att.pitch_deg, att.yaw_deg in PID and telemetry.
    }
}
```


## Why These Data Types

The EKF stores its state internally in radians because trigonometric functions and covariance math are naturally radian based. Inputs remain in flight-friendly units (`g`, `deg/s`, `uT`) so the sketch can pass calibrated driver output directly. The output is degrees because PID tuning, logs, and WiFi telemetry are easier to read and tune in degrees.
