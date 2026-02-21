#include "../../calibration/Calibration.h"
#include "../../logger/Logger.h"
#include "../../status/FCStatus.h"
#include "../../timers/DelayTimer.h"
#include "RCSensor.h"
#include "devices/FSIA.h"

#define RC_CALIBRATION_COUNT  100

uint8_t initRCSensor() {
	uint8_t status = initFSIA();
	if (status) {
		logString("[Rc Sensor] : FSIA > Success\n");
	} else {
		logString("[Rc Sensor] : FSIA > Failed\n");
	}
	return status;
}

uint16_t getRCFrameRate() {
	return getFSIAFrameRate();
}

uint8_t readRCSensor() {
	return readFSIA();
}

uint8_t isRCTXxActive() {
	return isFSIAActive();
}

uint16_t getRCValue(uint8_t channel) {
	return getFSIAChannelValue(channel);
}

void resetRCSensor() {

}

void calibrateRCSensor() {
	uint32_t thTotal = 0, pitchTotal = 0, rollTotal = 0, yawTotal = 0;
	for (uint16_t indx = 0; indx < RC_CALIBRATION_COUNT; indx++) {
		thTotal += getRCValue(RC_TH_CHANNEL_INDEX);
		pitchTotal += getRCValue(RC_PITCH_CHANNEL_INDEX);
		rollTotal += getRCValue(RC_ROLL_CHANNEL_INDEX);
		yawTotal += getRCValue(RC_YAW_CHANNEL_INDEX);
		delayMs(50);
	}
	setCalibrationValue(CALIB_PROP_RC_THROTTLE_OFFSET_ADDR, (uint16_t) (thTotal / RC_CALIBRATION_COUNT));
	setCalibrationValue(CALIB_PROP_RC_PITCH_OFFSET_ADDR, (uint16_t) (pitchTotal / RC_CALIBRATION_COUNT));
	setCalibrationValue(CALIB_PROP_RC_ROLL_OFFSET_ADDR, (uint16_t) (rollTotal / RC_CALIBRATION_COUNT));
	setCalibrationValue(CALIB_PROP_RC_YAW_OFFSET_ADDR, (uint16_t) (yawTotal / RC_CALIBRATION_COUNT));
	saveCalibration();
}

