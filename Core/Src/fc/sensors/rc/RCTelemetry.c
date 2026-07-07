#include "RCTelemetry.h"

extern void crsfSendBattery(float voltage, float current, uint32_t capacity, uint8_t percent);
extern void crsfSendGNSS(double lat, double lon, float speed, float heading, float altitude, uint8_t nSat);
extern void crsfSendAttitude(float pitch, float roll, float yaw);
extern void crsfSendAltitude(float altitude, float verticalSpeed);
extern void crsfSendFlightMode(const char *modeStr, uint8_t length);

void sendBatteryTelemetry(float voltage, float current, uint32_t capacity, uint8_t percent) {
	crsfSendBattery(voltage, current, capacity, percent);
}

void sendGNSSTelemetry(double lat, double lon, float speed, float heading, float altitude, uint8_t nSat) {
	crsfSendGNSS(lat, lon, speed, heading, altitude, nSat);
}

void sendAttitudeTelemetry(float pitch, float roll, float yaw) {
	crsfSendAttitude(pitch, roll, yaw);
}

void sendAltitudeTelemetry(float altitude, float verticalSpeed) {
	crsfSendAltitude(altitude, verticalSpeed);
}

void sendFlightModeTelemetry(const char *modeStr, uint8_t length) {
	crsfSendFlightMode(modeStr, length);
}

