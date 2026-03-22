#include "PositionManager.h"

#include <math.h>
#include <sys/_stdint.h>

#include "../../control/ControlData.h"
#include "../../control/position/PositionControl.h"
#include "../../dsp/LowPassFilter.h"
#include "../../FCConfig.h"
#include "../../imu/IMU.h"
#include "../../logger/Logger.h"
#include "../../memory/Memory.h"
#include "../../sensors/attitude/AttitudeSensor.h"
#include "../../sensors/position/GNSS.h"
#include "../../sensors/rc/RCSensor.h"
#include "../../status/FCStatus.h"
#include "../../timers/DeltaTimer.h"
#include "../../timers/Scheduler.h"
#include "../../util/MathUtil.h"
#include "estimator/PositionEstimator.h"
#include "estimator/VenturiBiasEstimator.h"
#include "helpers/PositionManagerHelper.h"

POSITION_EKF positionEkf;
LOWPASSFILTER positionMgrAccXLPF, positionMgrAccYLPF, positionMgrAccZLPF;
uint8_t positionManagerWasInStabMode;
POSITION_CORDINATE_DATA positionCordinateData;
POSITION_COMMAND_DATA positionCommandData;

float positionMgrPitchStickCenteredTimer, positionMgrRollStickCenteredTimer;
float positionMgrPitchPeakStick, positionMgrRollPeakStick;

void managePositionTask(void);

uint8_t initPositionManager(void) {
	logString("[Position Manager] Init > Start\n");
	uint8_t status = initGNSS();

	if (status) {
		logString("[Position Manager] GPS Init > Success\n");
	} else {
		logString("[Position Manager] GPS Init > Failed!\n");
	}

	status = positionEKFInit(&positionEkf) && initVenturiBiasEstimator();
	if (status) {
		logString("[Position Manager] EKF Init > Success\n");

		lowPassFilterInit(&positionMgrAccXLPF, POSITION_MGR_X_ACC_LPF_FREQ);
		lowPassFilterInit(&positionMgrAccYLPF, POSITION_MGR_Y_ACC_LPF_FREQ);
		lowPassFilterInit(&positionMgrAccZLPF, POSITION_MGR_Z_ACC_LPF_FREQ);

		schedulerAddTask(managePositionTask, POSITION_MANAGEMENT_TASK_FREQUENCY, POSITION_MANAGEMENT_TASK_PRIORITY);
		logString("[Position Manager] All tasks   > Started\n");

		initPositionControl();
	} else {
		logString("[Position Manager] EKF Init > Failed\n");
	}
	return 1;
}

__ATTR_ITCM_TEXT
void upadatePositionVelocity(float vx, float vy, float vz, float dt) {
	float vel;
	// X Axis
	vel = applyDeadBandFloat(0.0f, vx, POSITION_MGR_X_VEL_DEADBAND);
	vel = constrainToRangeF(vel, -POSITION_MGR_X_VEL_MAX, POSITION_MGR_X_VEL_MAX);
	positionCordinateData.xVelocity = vel;

	// Y Axis
	vel = applyDeadBandFloat(0.0f, vy, POSITION_MGR_Y_VEL_DEADBAND);
	vel = constrainToRangeF(vel, -POSITION_MGR_Y_VEL_MAX, POSITION_MGR_Y_VEL_MAX);
	positionCordinateData.yVelocity = vel;

	// Z Axis
	vel = applyDeadBandFloat(0.0f, vz, POSITION_MGR_Z_VEL_DEADBAND);
	vel = constrainToRangeF(vel, -POSITION_MGR_Z_VEL_MAX, POSITION_MGR_Z_VEL_MAX);
	positionCordinateData.zVelocity = vel;
}

__ATTR_ITCM_TEXT
void upadatePositionAcceleration(float ax, float ay, float az, float dt) {
	float acc;
	// X Axis
	acc = applyDeadBandFloat(0.0f, ax, POSITION_MGR_X_ACC_DEADBAND);
	acc = constrainToRangeF(acc, -POSITION_MGR_X_ACC_MAX, POSITION_MGR_X_ACC_MAX);
	positionCordinateData.xAcceleration = lowPassFilterUpdate(&positionMgrAccXLPF, acc, dt);

	// Y Axis
	acc = applyDeadBandFloat(0.0f, ay - positionCordinateData.yAccelerationBias, POSITION_MGR_Y_ACC_DEADBAND);
	acc = constrainToRangeF(acc, -POSITION_MGR_Y_ACC_MAX, POSITION_MGR_Y_ACC_MAX);
	positionCordinateData.yAcceleration = lowPassFilterUpdate(&positionMgrAccYLPF, acc, dt);

	// Z Axis
	acc = applyDeadBandFloat(0.0f, az - positionCordinateData.zAccelerationBias, POSITION_MGR_Z_ACC_DEADBAND);
	acc = constrainToRangeF(acc, -POSITION_MGR_Z_ACC_MAX, POSITION_MGR_Z_ACC_MAX);
	positionCordinateData.zAcceleration = lowPassFilterUpdate(&positionMgrAccZLPF, acc, dt);
}

void manageBraking(float dt) {
	float currentPitch = (float) rcData.RC_EFFECTIVE_DATA[RC_PITCH_CHANNEL_INDEX];
	float currentRoll = (float) rcData.RC_EFFECTIVE_DATA[RC_ROLL_CHANNEL_INDEX];
	if (!rcData.pitchCentered) {
		positionMgrPitchStickCenteredTimer = 0.0f;
		positionCommandData.pitchCommand = 0.0f;
		if ((currentPitch > 0 && positionMgrPitchPeakStick < 0) || (currentPitch < 0 && positionMgrPitchPeakStick > 0)) {
			positionMgrPitchPeakStick = currentPitch;
		}
		if (fabsf(currentPitch) > fabsf(positionMgrPitchPeakStick)) {
			positionMgrPitchPeakStick = currentPitch;
		}
	} else {
		positionMgrPitchStickCenteredTimer += dt;
		if (positionMgrPitchStickCenteredTimer >= POSITION_MGR_PITCH_BRAKE_DELAY && positionMgrPitchStickCenteredTimer < (POSITION_MGR_PITCH_BRAKE_DELAY + POSITION_MGR_PITCH_BRAKE_WIDTH)) {
			float fadeTime = positionMgrPitchStickCenteredTimer - POSITION_MGR_PITCH_BRAKE_DELAY;
			float fadeWeight = fadeTime / POSITION_MGR_PITCH_BRAKE_FADE_IN;
			if (fadeWeight > 1.0f) {
				fadeWeight = 1.0f;
			}
			float pitchOut = -positionMgrPitchPeakStick * POSITION_MGR_PITCH_BRAKE_GAIN * fadeWeight;
			positionCommandData.pitchCommand = constrainToRangeF(pitchOut, -POSITION_MGR_PITCH_BRAKE_LIMIT, POSITION_MGR_PITCH_BRAKE_LIMIT);
		} else {
			positionCommandData.pitchCommand = 0.0f;
			if (positionMgrPitchStickCenteredTimer >= (POSITION_MGR_PITCH_BRAKE_DELAY + POSITION_MGR_PITCH_BRAKE_WIDTH)) {
				positionMgrPitchPeakStick = 0.0f;
			}
		}
	}

	if (!rcData.rollCentered) {
		positionMgrRollStickCenteredTimer = 0.0f;
		positionCommandData.rollCommand = 0.0f;
		if ((currentRoll > 0 && positionMgrRollPeakStick < 0) || (currentRoll < 0 && positionMgrRollPeakStick > 0)) {
			positionMgrRollPeakStick = currentRoll;
		}
		if (fabsf(currentRoll) > fabsf(positionMgrRollPeakStick)) {
			positionMgrRollPeakStick = currentRoll;
		}
	} else {
		positionMgrRollStickCenteredTimer += dt;
		if (positionMgrRollStickCenteredTimer >= POSITION_MGR_ROLL_BRAKE_DELAY && positionMgrRollStickCenteredTimer < (POSITION_MGR_ROLL_BRAKE_DELAY + POSITION_MGR_ROLL_BRAKE_WIDTH)) {
			float fadeTime = positionMgrRollStickCenteredTimer - POSITION_MGR_ROLL_BRAKE_DELAY;
			float fadeWeight = fadeTime / POSITION_MGR_ROLL_BRAKE_FADE_IN;
			if (fadeWeight > 1.0f) {
				fadeWeight = 1.0f;
			}
			float rollOut = -positionMgrRollPeakStick * POSITION_MGR_ROLL_BRAKE_GAIN * fadeWeight;
			positionCommandData.rollCommand = constrainToRangeF(rollOut, -POSITION_MGR_ROLL_BRAKE_LIMIT, POSITION_MGR_ROLL_BRAKE_LIMIT);
		} else {
			positionCommandData.rollCommand = 0.0f;
			if (positionMgrRollStickCenteredTimer >= (POSITION_MGR_ROLL_BRAKE_DELAY + POSITION_MGR_ROLL_BRAKE_WIDTH)) {
				positionMgrRollPeakStick = 0.0f;
			}
		}
	}
}

__ATTR_ITCM_TEXT
void managePositionHold(float dt) {
	//Enabled if the Pitch and Roll Sticks are centered
	if (fcStatusData.enablePositionHold) {
		controlPositionWithGains(dt, fcStatusData.positionXRef, fcStatusData.positionYRef, 1.0f, 1.0f, 1.0f, 1.0f);
		convertEarthToBodyCordinates(controlData.positionXControl, controlData.positionYControl, sensorAttitudeData.heading, &positionCommandData.pitchCommand, &positionCommandData.rollCommand);
	} else {
		fcStatusData.positionXRef = positionCordinateData.xPosition;
		fcStatusData.positionYRef = positionCordinateData.yPosition;
		positionCommandData.pitchCommand = 0.0f;
		positionCommandData.rollCommand = 0.0f;
	}
}

/**
 * Provides the positionCommand , called by attitude manager
 */
__ATTR_ITCM_TEXT
void updatePositionCommand(float dt) {
	if (fcStatusData.liftOffThrottlePercent >= fcStatusData.throttlePercent) {
		positionMgrPitchStickCenteredTimer = 0.0f;
		positionMgrRollStickCenteredTimer = 0.0f;
		positionMgrPitchPeakStick = 0.0f;
		positionMgrRollPeakStick = 0.0f;
		positionCommandData.pitchCommand = 0.0f;
		positionCommandData.rollCommand = 0.0f;
	} else if (fcStatusData.isPositionHoldModeActive && fcStatusData.isPositionHomeSet && fcStatusData.isPositionDataReliable) {
		managePositionHold(dt);
	} else {
		positionCommandData.pitchCommand = 0.0f;
		positionCommandData.rollCommand = 0.0f;
	}
}

__ATTR_ITCM_TEXT
void managePositionTask(void) {
	float dt = getDeltaTime(POSITION_MANAGER_TASK_TIMER_CHANNEL);

	if (dt <= 0.0f || dt > 0.1f) {
		dt = 0.001f; // Safety guard for dt
	}
	float axEarth = imuData.axEarthLinear * POSITION_MGR_ACC_OUTPUT_GAIN;
	float ayEarth = imuData.ayEarthLinear * POSITION_MGR_ACC_OUTPUT_GAIN;
	float azEarth = imuData.azEarthLinear * POSITION_MGR_ACC_OUTPUT_GAIN;

	// 1. Prediction (Using raw or slightly scaled earth-frame acc)
	positionEKFPredict(&positionEkf, axEarth, ayEarth, azEarth, dt);

	float *x = positionEkf.x;
	positionCordinateData.xPosition = x[0]; // X Pos
	positionCordinateData.xAccelerationBias = x[2]; // X Bias

	positionCordinateData.yPosition = x[3]; // Y Pos
	positionCordinateData.yAccelerationBias = x[5]; // Y Bias

	positionCordinateData.zPosition = x[6]; // Z Pos
	positionCordinateData.zAccelerationBias = x[8]; // Z Bias

	// 2. Filtered Acceleration
	upadatePositionAcceleration(axEarth - positionCordinateData.xAccelerationBias, ayEarth - positionCordinateData.yAccelerationBias, azEarth - positionCordinateData.zAccelerationBias, dt);
	// 3. Filtered Velocity
	upadatePositionVelocity(x[1], x[4], x[7], dt);

	//Update the position command
	updatePositionCommand(dt);

	positionCordinateData.positionProcessDt = dt;
}

void handlePositionDataUpdate(float dt) {
	if (fcStatusData.isPositionHoldModeActive) {
		updatePositionDataReliability(dt);
		if (fcStatusData.isPositionDataReliable) {
			if (!fcStatusData.isPositionHomeSet) {
				fcStatusData.positionLongHome = gnssData.longitude;
				fcStatusData.positionLatHome = gnssData.latitude;
				fcStatusData.isPositionHomeSet = 1;
			}
			convertGNSSToSICordinates(gnssData.latitude, gnssData.longitude, fcStatusData.positionLatHome, fcStatusData.positionLongHome, &positionCordinateData.xPositionRaw, &positionCordinateData.yPositionRaw);
			updatePositionManagerXYPosition(positionCordinateData.xPositionRaw, positionCordinateData.yPositionRaw, dt);
		}
	} else {
		fcStatusData.positionLongHome = 0;
		fcStatusData.positionLatHome = 0;
		fcStatusData.positionYRef = 0;
		fcStatusData.positionXRef = 0;
		fcStatusData.isPositionHomeSet = 0;
		fcStatusData.isPositionDataReliable = 0;
		updatePositionManagerXYPosition(0, 0, dt);
		positionEKFApplyXYDamping(&positionEkf, POSITION_MGR_XY_VEL_DAMP_STRENGTH);
	}
}

__ATTR_ITCM_TEXT
void doPositionManagement() {
	if (fcStatusData.canStabilize && positionManagerWasInStabMode == 0) {
		positionManagerWasInStabMode = 1;
		positionEKFSetMode(&positionEkf, 1);
	} else if (positionManagerWasInStabMode && fcStatusData.isStabilized) {
		positionManagerWasInStabMode = 0;
		positionEKFSetMode(&positionEkf, 0);
	}
	if (readGNSSData()) {
		float dt = getDeltaTime(POSITION_MANAGER_GPS_TIMER_CHANNEL);
		gnssData.updateDt = dt;
		handlePositionDataUpdate(dt);
	}
}

void resetPositionManager(void) {
	lowPassFilterReset(&positionMgrAccXLPF);
	lowPassFilterReset(&positionMgrAccYLPF);
	lowPassFilterReset(&positionMgrAccZLPF);
	positionEKFReset(&positionEkf, 0, 0, 0);
	resetVenturiBiasEstimator();
	resetPositionControl(1);
	positionManagerWasInStabMode = 0;
	positionMgrPitchStickCenteredTimer = 0;
	positionMgrRollStickCenteredTimer = 0;
}

__ATTR_ITCM_TEXT
void updatePositionManagerZPosition(float zPos, float dt) {
	positionCordinateData.positionZUpdateDt = dt;
	positionCordinateData.zPositionRaw = zPos;
	float venturiBias = updateVenturiBiasEstimate(dt);
	positionEKFUpdateZMeasureWithBias(&positionEkf, zPos, venturiBias);
}

__ATTR_ITCM_TEXT
void updatePositionManagerXYPosition(float xPos, float yPos, float dt) {
	positionCordinateData.positionXYUpdateDt = dt;
	positionEKFUpdateXYMeasure(&positionEkf, xPos, yPos);
}

