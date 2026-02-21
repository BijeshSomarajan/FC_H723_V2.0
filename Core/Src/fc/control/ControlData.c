#include "ControlData.h"


CONTROL_DATA controlData;

/*

 Altitude control loop frequencies
 +------------------+-------------------------------------------------+----------------------+--------------------------------+
 | Loop | Function | Typical Frequency | Sensor Used |
 +------------------+-------------------------------------------------+----------------------+--------------------------------+
 | Outer (Position) | Commands the target vertical velocity to correct| 50–100 Hz | Barometer or LiDAR for altitude|
 | | altitude errors. | | measurement. |
 +------------------+-------------------------------------------------+----------------------+--------------------------------+
 | Inner (Velocity) | Commands motor thrust to achieve the desired | 250–400+ Hz | IMU (accelerometer) for |
 | | vertical velocity. | | vertical acceleration. |
 +------------------+-------------------------------------------------+----------------------+--------------------------------+

 ### Attitude control loop frequencies
 +------------------+-------------------------------------------------+----------------------+--------------------------------+
 | Loop | Function | Typical Frequency | Sensor Used |
 +------------------+-------------------------------------------------+----------------------+--------------------------------+
 | Outer (Angle) | Commands the target angular velocity to achieve | 50–100 Hz | Fused IMU data (accelerometer +|
 | | the desired pitch, roll, or yaw angle. | | gyroscope) for attitude angles.|
 +------------------+-------------------------------------------------+----------------------+--------------------------------+
 | Inner (Rate) | Commands motor torque to achieve the desired | 400+ Hz (often 1–8 kHz) | Gyroscope for high-frequency |
 | | angular velocity. | | angular velocity. |
 +------------------+-------------------------------------------------+----------------------+--------------------------------+

 */
