#include "AttitudeManager.h"

#include <sys/_stdint.h>

#include "../../control/attitude/AttitudeControl.h"
#include "../../imu/IMU.h"
#include "../../logger/Logger.h"
#include "../../memory/Memory.h"
#include "../../sensors/attitude/noisefilter/AttitudeNoiseFilter.h"
#include "../../sensors/attitude/AttitudeSensor.h"
#include "../../sensors/rc/RCSensor.h"
#include "../../status/FCStatus.h"
#include "../../timers/DeltaTimer.h"
#include "../../timers/GPTimer.h"
#include "../../util/MathUtil.h"
#include "../../FCConfig.h"

uint8_t attitudeManagerWasInStabMode = 0;
void readAGTSensorTimerCallback(void);
void readMagSensorTimerCallback(void);
float attitudeManagerCrashThresholdG;
uint16_t attitudeManagerCrashTriggerCounter = 0;

__ATTR_ITCM_TEXT
void readAGTSensorTimerCallback() {
	readAccGyroTempSensor();
}

__ATTR_ITCM_TEXT
void readMagSensorTimerCallback() {
	readMagSensor();
}

void startAttitudeSensorsRead() {
	initGPTimer24(ATTITUDE_SENSOR_AGT_READ_FREQUENCY, readAGTSensorTimerCallback, 4);
	initGPTimer4(ATTITUDE_SENSOR_MAG_READ_FREQUENCY, readMagSensorTimerCallback, 5);
	startGPTimer4();
	startGPTimer24();
}

__ATTR_ITCM_TEXT
void alignImuRateToBoard() {
	sensorAttitudeData.pitchRate = -imuData.pitchRate;
	sensorAttitudeData.rollRate = -imuData.rollRate;
	sensorAttitudeData.yawRate = -imuData.yawRate;
}

__ATTR_ITCM_TEXT
void alignImuAnglesToBoard() {
	sensorAttitudeData.pitch = -imuData.roll;
	sensorAttitudeData.roll = -imuData.pitch;
	float temp = 180 - imuData.heading;
	if (temp < 0) {
		sensorAttitudeData.heading = temp + 360.0f;
	} else if (temp > 360) {
		sensorAttitudeData.heading = temp - 360.0f;
	} else {
		sensorAttitudeData.heading = temp;
	}
}

__ATTR_ITCM_TEXT
void doAttitudeControl(float dt) {
	if (!rcData.yawCentered) {
		fcStatusData.headingRef = sensorAttitudeData.heading;
	}
	if (fcStatusData.canFly && fcStatusData.throttlePercent > ATTITUDE_CONTROL_MIN_TH_PERCENT) {
		float expectedPitch = (float) rcData.RC_EFFECTIVE_DATA[RC_PITCH_CHANNEL_INDEX];
		float expectedRoll = -(float) rcData.RC_EFFECTIVE_DATA[RC_ROLL_CHANNEL_INDEX];
		float expectedYaw = +(float) rcData.RC_EFFECTIVE_DATA[RC_YAW_CHANNEL_INDEX];
		expectedPitch = constrainToRangeF(expectedPitch, -ATTITUDE_CONTROL_MAX_PITCH_ROLL, ATTITUDE_CONTROL_MAX_PITCH_ROLL);
		expectedRoll = constrainToRangeF(expectedRoll, -ATTITUDE_CONTROL_MAX_PITCH_ROLL, ATTITUDE_CONTROL_MAX_PITCH_ROLL);
		float rateIGain = 1.0;
		float rateDGain = 1.0f;
		//Reset the I and D gains
		if (!fcStatusData.isFlying) {
			rateIGain = 0;
			rateDGain = 0;
		}
		controlAttitudeWithGains(dt, expectedPitch, expectedRoll, expectedYaw, rateIGain, rateDGain);
	} else {
		resetAttitudeControl(1);
	}
}

__ATTR_ITCM_TEXT
void checkForCrash() {
	if (!fcStatusData.hasCrashed && fcStatusData.canFly) {
		float ax = sensorAttitudeData.axGRaw;
		float ay = sensorAttitudeData.ayGRaw;
		float az = sensorAttitudeData.azGRaw;
		float totalG = fastSqrtf(ax * ax + ay * ay + az * az);
		float impactG = fabsf(totalG - 1.0f);
		if (impactG > attitudeManagerCrashThresholdG) {
			attitudeManagerCrashTriggerCounter++;
			if (attitudeManagerCrashTriggerCounter >= ATTITUDE_SENSOR_ACC_CRASH_SAMPLES_COUNT) {
				fcStatusData.hasCrashed = 1;
			}
		} else {
			if (attitudeManagerCrashTriggerCounter > 0) {
				attitudeManagerCrashTriggerCounter--;
			}
		}
	}
}

__ATTR_ITCM_TEXT
void handleAttitudeSensorUpdates() {
	if (!fcStatusData.isConfigMode) {
		if (fcStatusData.canStabilize) {
			fcStatusData.headingHomeRef = sensorAttitudeData.heading;
			fcStatusData.headingRef = fcStatusData.headingHomeRef;
			attitudeManagerWasInStabMode = 1;
			imuSetMode(1);
		} else if (attitudeManagerWasInStabMode && fcStatusData.isStabilized) {
			imuSetMode(0);
			attitudeManagerWasInStabMode = 0;
		}

		if (loadMagSensorData()) {
			float dt = getDeltaTime(SENSOR_MAG_READ_TIMER_CHANNEL);
			updateMagSensorData(dt);
			filterMagNoise(dt);
		}

		if (loadAccGyroTempSensorData()) {
			float dt = getDeltaTime(SENSOR_AGT_READ_TIMER_CHANNEL);
			updateAGTSensorData(dt);
			updateNoiseFilterData(dt);

			checkForCrash();

			filterAGTNoise(dt);
			imuUpdateRate();
			imuAHRSUpdate(dt);
			alignImuAnglesToBoard();
			alignImuRateToBoard();

			//Handling crash land as soon as possible
			if (!fcStatusData.hasCrashed) {
				doAttitudeControl(dt);
			} else {
				resetAttitudeManager();
			}
		}

		updateNoiseFilterCoefficients();

		if (fcStatusData.hasCrashed) {
			resetAttitudeManager();
		}
	}

}

__ATTR_ITCM_TEXT
void doAttitudeManagement() {
	handleAttitudeSensorUpdates();
}

uint8_t initAttitudeManager() {
	logString("[Attitude Manager] Init > Start\n");
	uint8_t status = initAttitudeSensors();
	if (status) {
		logString("[Attitude Manager] Sensor Init > Success\n");
		initAttitudeNoiseFilter(ATTITUDE_SENSOR_AGT_READ_FREQUENCY, ATTITUDE_SENSOR_AGT_READ_FREQUENCY, ATTITUDE_SENSOR_MAG_READ_FREQUENCY, ATTITUDE_SENSOR_AGT_READ_FREQUENCY);
		startAttitudeSensorsRead();
		imuInit(0);
		initAttitudeControl();
		attitudeManagerCrashThresholdG = getMaxValidG() * ATTITUDE_SENSOR_ACC_CRASH_G_GAIN;
		logString("[Attitude Manager] All tasks > Started\n");
	} else {
		logString("[Attitude Manager] Init > Failed!\n");
	}
	return status;
}

uint8_t resetAttitudeManager() {
	resetAttitudeSensors(0);
	resetAttitudeControl(1);
	resetNoiseFilter();
	return 1;
}
