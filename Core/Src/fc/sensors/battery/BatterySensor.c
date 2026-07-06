#include "BatterySensor.h"
#include "devices/INA260.h"
#include "../../logger/Logger.h"

BatteryData batteryData;
extern INA260Data ina260Data;

uint8_t initBatterySensor(void) {
	uint8_t status = 0;
	status = initINA260();
	if (!status) {
		logString("[BatterySensor] Init Failed\n");
		return 0;
	} else {
		logString("[BatterySensor] Init Success\n");
	}
	return 1;
}

uint8_t readBatterySensor(void) {
	uint8_t status = readINA260();
	if (!status) {
		return 0;
	} else {
		batteryData.voltage = ina260Data.voltage;
		batteryData.current = ina260Data.current;
	}
	return 1;
}
