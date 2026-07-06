#include "RCTelemetry.h"

/* Reference the CRSF backend serialization functions compiled in your single file */
extern void crsfSendBattery(uint16_t voltage_dV, uint16_t current_dA, uint32_t capacity_mAh, uint8_t remaining_pct);
extern void crsfSendGNSS(int32_t lat_scaled, int32_t lon_scaled, uint16_t speed_scaled, uint16_t heading_scaled, float altitude, uint8_t nSat);
extern void crsfSendAttitude(int16_t pitch_scaled, int16_t roll_scaled, int16_t yaw_scaled);
extern void crsfSendAltitude(float altitude, int16_t vertical_speed_cms);
extern void crsfSendFlightMode(const char *modeStr);

void sendBatteryTelemetry(float voltage_v, float current_a, uint32_t capacity_mah, uint8_t percent) {
	// CRSF expects Decivolts (V * 10) and Deciamperes (A * 10)
	uint16_t v_dV = (uint16_t) (voltage_v * 10.0f);
	uint16_t c_dA = (uint16_t) (current_a * 10.0f);
	crsfSendBattery(v_dV, c_dA, capacity_mah, percent);
}

void sendGNSSTelemetry(double lat, double lon, float speed, float heading, float altitude, uint8_t nSat) {
	// CRSF expects coordinates multiplied by 10,000,000
	int32_t latScaled = (int32_t) (lat * 10000000.0);
	int32_t lonScaled = (int32_t) (lon * 10000000.0);
	// Speed expected in 0.1 km/h increments
	uint16_t speedScaled = (uint16_t) (speed * 100.0f);
	// Heading expected in 0.01 degree increments (0 to 35999)
	uint16_t headingScaled = (uint16_t) (heading * 100.0f);
	crsfSendGNSS(latScaled, lonScaled, speedScaled, headingScaled, altitude, nSat);
}

void sendAttitudeTelemetry(float pitch, float roll, float yaw) {
	int16_t p_scaled = (int16_t) (pitch * 100.0f);
	int16_t r_scaled = (int16_t) (roll * 100.0f);
	int16_t y_scaled = (int16_t) (yaw * 100.0f);
	crsfSendAttitude(p_scaled, r_scaled, y_scaled);
}

void sendAltitudeTelemetry(float altitude, float vVel) {
	int16_t vVelScaled = (int16_t) (vVel * 100.0f);
	int16_t altitudeScaled = (int16_t) (altitude * 100.0f);
	crsfSendAltitude(altitudeScaled, vVelScaled);
}

void sendFlightModeTelemetry(const char *modeStr) {
	crsfSendFlightMode(modeStr);
}
