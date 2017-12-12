# cam_imu

This package provides handling for a Visual Inertial setup that comprises an iDS uEye-1226 global shutter camera and an Invensense MPU6050 IMU connected to an Arduino board. Provides also an interface to filter and integrate accelerometer data to obtain positions.

Depends on:
* tinyIMU_relay
* imu_filter_madgwick
uEye SDK must also be installed.

It serves as a complement to the article in https://riccardogiubilato.github.io/visual/odometry/2017/12/12/Visual-Inertial-Odometry-On-A-Budget.html
