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
#include "../../managers/position/common/PositionCommon.h"
#include "../../managers/position/PositionManager.h"
#include "../../dsp/LowPassFilter.h"

extern POSITION_COMMAND_DATA positionCommandData;

uint8_t attitudeManagerWasInStabMode = 0;
void readAGTSensorTimerCallback(void);
void readMagSensorTimerCallback(void);
void attRateControlTimerCallback(void);

float attitudeManagerCrashThresholdG;
uint16_t attitudeManagerCrashTriggerCounter = 0;
float attitudeAngleControlDt = 0;

LOWPASSFILTER attitudePitchCtrlRateLPF, attitudeRollCtrlRateLPF, attitudeYawCtrlRateLPF;

__ATTR_ITCM_TEXT
void readAGTSensorTimerCallback() {
	readAccGyroTempSensor();
}

__ATTR_ITCM_TEXT
void readMagSensorTimerCallback() {
	readMagSensor();
}

void startAttitudeMgmtTimers() {
	initGPTimer24(ATTITUDE_SENSOR_AGT_READ_FREQUENCY, readAGTSensorTimerCallback, 4);
	initGPTimer4(ATTITUDE_SENSOR_MAG_READ_FREQUENCY, readMagSensorTimerCallback, 5);
	initGPTimer7(ATTITUDE_RATE_CONTROL_FREQUENCY, attRateControlTimerCallback, 4);
	startGPTimer4();
	startGPTimer24();
	startGPTimer7();
}

__ATTR_ITCM_TEXT
void alignImuRateToBoard() {
	sensorAttitudeData.pitchRate = -imuData.pitchRate;
	sensorAttitudeData.rollRate = -imuData.rollRate;
	sensorAttitudeData.yawRate = -imuData.yawRate;

	sensorAttitudeData.pitchRateRaw = -sensorAttitudeData.gxDS;
	sensorAttitudeData.rollRateRaw = -sensorAttitudeData.gyDS;
}

__ATTR_ITCM_TEXT
void alignImuAnglesToBoard() {
	sensorAttitudeData.pitch = -imuData.roll;
	sensorAttitudeData.roll = -imuData.pitch;
	float temp = 90 - imuData.heading;
	if (temp < 0) {
		sensorAttitudeData.heading = temp + 360.0f;
	} else if (temp > 360) {
		sensorAttitudeData.heading = temp - 360.0f;
	} else {
		sensorAttitudeData.heading = temp;
	}
}

__ATTR_ITCM_TEXT
void doAttitudeRateControl(float dt) {
	if (!fcStatusData.hasCrashed) {
		float ratePGain = 1.0f;
		float rateIGain = 1.0;
		float rateDGain = 1.0f;
		//Reset the I and D gains
		if (!fcStatusData.isFlying) {
			rateIGain = 0;
			rateDGain = 0;
		}
		controlAttitudeRateWithGains(dt, ratePGain, rateIGain, rateDGain);
	} else {
		resetAttitudeControl(1);
	}

}

__ATTR_ITCM_TEXT
void updateAttiudeCtrlRates(float dt) {
	sensorAttitudeData.pitchCtrlRate = lowPassFilterUpdate(&attitudePitchCtrlRateLPF, sensorAttitudeData.pitchRate, dt);
	sensorAttitudeData.rollCtrlRate = lowPassFilterUpdate(&attitudeRollCtrlRateLPF, sensorAttitudeData.rollRate, dt);
	sensorAttitudeData.yawCtrlRate = lowPassFilterUpdate(&attitudeYawCtrlRateLPF, sensorAttitudeData.yawRate, dt);
}

__ATTR_ITCM_TEXT
void attRateControlTimerCallback() {
	float dt = getDeltaTime(SENSOR_ATT_RATE_CONTROL_TIMER_CHANNEL);
	dt = constrainToRangeF(dt, ATTITUDE_RATE_CONTROL_PERIOD * 0.001f, ATTITUDE_RATE_CONTROL_PERIOD * 4.0f);
	imuUpdateRate();
	alignImuRateToBoard();
	updateAttiudeCtrlRates(dt);
	doAttitudeRateControl(dt);
}

__ATTR_ITCM_TEXT
void doAttitudeAngleControl(float dt) {
	if (!rcData.yawCentered) {
		fcStatusData.headingRef = sensorAttitudeData.heading;
	}
	if (fcStatusData.canFly && fcStatusData.throttlePercent > ATTITUDE_CONTROL_MIN_TH_PERCENT) {
		float expectedPitch = (-(float) rcData.RC_EFFECTIVE_DATA[RC_PITCH_CHANNEL_INDEX]) - positionCommandData.pitchCommand;
		float expectedRoll = (float) rcData.RC_EFFECTIVE_DATA[RC_ROLL_CHANNEL_INDEX] + positionCommandData.rollCommand;
		float expectedYaw = ((float) rcData.RC_EFFECTIVE_DATA[RC_YAW_CHANNEL_INDEX]);
		expectedPitch = constrainToRangeF(expectedPitch, -ATTITUDE_CONTROL_MAX_PITCH_ROLL, ATTITUDE_CONTROL_MAX_PITCH_ROLL);
		expectedRoll = constrainToRangeF(expectedRoll, -ATTITUDE_CONTROL_MAX_PITCH_ROLL, ATTITUDE_CONTROL_MAX_PITCH_ROLL);
		controlAttitudeAngle(dt, expectedPitch, expectedRoll, expectedYaw);
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
void doAttitudeManagement() {
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
			dt = constrainToRangeF(dt, ATTITUDE_ANGLE_CONTROL_PERIOD * 0.001f, ATTITUDE_ANGLE_CONTROL_PERIOD * 4.0f);
			updateAGTSensorData(dt);
			updateNoiseFilterData(dt);
			checkForCrash();
			filterAGTNoise(dt);

			imuAHRSUpdate(dt);
			alignImuAnglesToBoard();
			doAttitudeAngleControl(dt);
		}
		updateNoiseFilterCoefficients();
		if (fcStatusData.hasCrashed) {
			resetAttitudeManager();
		}
	}
}

uint8_t initAttitudeManager() {
	logString("[Attitude Manager] Init > Start\n");
	uint8_t status = initAttitudeSensors();
	if (status) {
		logString("[Attitude Manager] Sensor Init > Success\n");
		initAttitudeNoiseFilter(ATTITUDE_SENSOR_AGT_READ_FREQUENCY, ATTITUDE_SENSOR_AGT_READ_FREQUENCY, ATTITUDE_SENSOR_MAG_READ_FREQUENCY, ATTITUDE_SENSOR_AGT_READ_FREQUENCY);
		startAttitudeMgmtTimers();
		imuInit(0);
		initAttitudeControl();
		attitudeManagerCrashThresholdG = getMaxValidG() * ATTITUDE_SENSOR_ACC_CRASH_G_GAIN;

		lowPassFilterInit(&attitudePitchCtrlRateLPF, ATTITUDE_CTRL_RATE_LPF_FREQUENCY);
		lowPassFilterInit(&attitudeRollCtrlRateLPF, ATTITUDE_CTRL_RATE_LPF_FREQUENCY);
		lowPassFilterInit(&attitudeYawCtrlRateLPF, ATTITUDE_CTRL_RATE_LPF_FREQUENCY);

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

	lowPassFilterReset(&attitudePitchCtrlRateLPF);
	lowPassFilterReset(&attitudeRollCtrlRateLPF);
	lowPassFilterReset(&attitudeYawCtrlRateLPF);

	return 1;
}
