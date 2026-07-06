/**
 * ******************************************************************************
 * @file    telemetry.h
 * @brief   Generic Flight Controller Telemetry Abstraction Interface
 * ******************************************************************************
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

/* Generic high-level API functions for the main application / loop scheduler */

/**
 * @brief Dispatches current battery metrics using standard units.
 * @param voltage_v   Raw voltage in Volts (e.g., 16.8f)
 * @param current_a   Raw current draw in Amperes (e.g., 25.3f)
 * @param capacity_mah Drawn capacity in milliamp-hours
 * @param percent     Remaining capacity percentage (0 to 100)
 */
void sendBatteryTelemetry(float voltage_v, float current_a, uint32_t capacity_mah, uint8_t percent);

/**
 * @brief Dispatches GNSS location state using standard coordinate types.
 * @param lat_deg      Double precision Latitude in degrees (e.g., 12.971598)
 * @param lon_deg      Double precision Longitude in degrees (e.g., 77.594562)
 * @param speed_kmh    Ground speed in kilometers per hour
 * @param heading_deg  True ground track heading in degrees (0.0 to 359.9f)
 * @param altitude_m   Altitude above mean sea level in meters
 * @param satellites   Active locked satellite count
 */
void sendGNSSTelemetry(double lat_deg, double lon_deg, float speed_kmh, float heading_deg, float altitude_m, uint8_t satellites);

/**
 * @brief Dispatches orientation telemetry from the state estimators (EKF/IMU).
 * @param pitch_rad   Pitch angle in Radians
 * @param roll_rad    Roll angle in Radians
 * @param yaw_rad     Yaw angle in Radians
 */
void sendAttitudeTelemetry(float pitch_rad, float roll_rad, float yaw_rad);

/**
 * @brief Dispatches altimeter and vertical rate performance.
 * @param altitude_m         Relative or absolute altitude in meters
 * @param vertical_speed_ms  Climb or descent rate in meters per second (positive = up)
 */
void sendAltitudeTelemetry(float altitude_m, float vertical_speed_ms);

/**
 * @brief Dispatches the current stabilization or autonomy state machine string.
 * @param modeStr Null-terminated ASCII flight status sequence (e.g., "ANGLE", "FAILSAFE")
 */
void sendFlightModeTelemetry(const char *modeStr);

#endif /* TELEMETRY_H */
