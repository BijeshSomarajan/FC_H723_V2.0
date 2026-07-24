#include "RCTelemetry.h"
#include "RCSensor.h"

#if RC_RX_TYPE== RC_RX_TYPE_CRSF
extern void crsfSendBattery(float voltage, float current, uint32_t capacity, uint8_t percent);
extern void crsfSendGNSS(double lat, double lon, float speed, float heading, float altitude, uint8_t nSat);
extern void crsfSendAttitude(float pitch, float roll, float yaw);
extern void crsfSendAltitude(float altitude, float verticalSpeed);
extern void crsfSendFlightMode(const char *modeStr, uint8_t length);
#endif

void sendBatteryTelemetry(float voltage, float current, uint32_t capacity, uint8_t percent) {
#if RC_RX_TYPE== RC_RX_TYPE_CRSF
	crsfSendBattery(voltage, current, capacity, percent);
#endif
}

void sendGNSSTelemetry(double lat, double lon, float speed, float heading, float altitude, uint8_t nSat) {
#if RC_RX_TYPE== RC_RX_TYPE_CRSF
	crsfSendGNSS(lat, lon, speed, heading, altitude, nSat);
#endif
}

void sendAttitudeTelemetry(float pitch, float roll, float yaw) {
#if RC_RX_TYPE== RC_RX_TYPE_CRSF
	crsfSendAttitude(pitch, roll, yaw);
#endif
}

void sendAltitudeTelemetry(float altitude, float verticalSpeed) {
#if RC_RX_TYPE== RC_RX_TYPE_CRSF
	crsfSendAltitude(altitude, verticalSpeed);
#endif
}

void sendFlightModeTelemetry(const char *modeStr, uint8_t length) {
#if RC_RX_TYPE== RC_RX_TYPE_CRSF
	crsfSendFlightMode(modeStr, length);
#endif
}

