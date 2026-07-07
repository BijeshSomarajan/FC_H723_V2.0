#include "../../calibration/Calibration.h"
#include "../../logger/Logger.h"
#include "../../status/FCStatus.h"
#include "../../timers/DelayTimer.h"

#include "RCSensor.h"
#include "devices/FSIA.h"
#include "devices/CRSF.h"

#define RC_CALIBRATION_COUNT  100

uint8_t initRCSensor() {

#if RC_RX_TYPE== RC_RX_TYPE_FSIA
	uint8_t status = initFSIA();
	if (status) {
		logString("[Rc Sensor] : FSIA > Success\n");
	} else {
		logString("[Rc Sensor] : FSIA > Failed\n");
	}
#else
	uint8_t status = initCRSF();
	if (status) {
		logString("[Rc Sensor] : CRSF > Success\n");
	} else {
		logString("[Rc Sensor] : CRSF > Failed\n");
	}
#endif

	return status;
}

uint16_t getRCFrameRate() {
#if RC_RX_TYPE== RC_RX_TYPE_FSIA
	return getFSIAFrameRate();
#else
	return getCRSFFrameRate();
#endif
}

uint8_t readRCSensor() {
#if RC_RX_TYPE== RC_RX_TYPE_FSIA
	return readFSIA();
#else
	return readCRSF();
#endif
}

uint8_t isRCTXxActive() {
#if RC_RX_TYPE== RC_RX_TYPE_FSIA
	return isFSIAActive();
#else
	return isCRSFActive();
#endif
}

uint16_t getRCValue(uint8_t channel) {
#if RC_RX_TYPE== RC_RX_TYPE_FSIA
	return getFSIAChannelValue(channel);
#else
	return getCRSFChannelValue(channel);
#endif
}

void setRCValue(uint8_t channel, uint16_t value) {
#if RC_RX_TYPE== RC_RX_TYPE_FSIA
	return setFSIAChannelValue(channel, value);
#else
	return setCRSFChannelValue(channel, value);
#endif
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

