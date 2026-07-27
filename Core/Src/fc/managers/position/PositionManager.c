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
#include "estimator/PositionEstimatorHelper.h"
#include "estimator/VenturiBiasEstimator.h"
#include "helpers/PositionManagerHelper.h"
#include "helpers/PositionManagerHomeAquisitionHelper.h"

LOWPASSFILTER positionMgrAccXLPF, positionMgrAccYLPF, positionMgrAccZLPF;
LOWPASSFILTER positionMgrVelXLPF, positionMgrVelYLPF, positionMgrVelZLPF;

uint8_t positionManagerWasInStabMode;
POSITION_CORDINATE_DATA positionCordinateData;
POSITION_COMMAND_DATA positionCommandData;

uint8_t postionEKFXIndex = POS_EKF_X_AXIS * POS_EKF_AXIS_DIM;
uint8_t postionEKFYIndex = POS_EKF_Y_AXIS * POS_EKF_AXIS_DIM;
uint8_t postionEKFZIndex = POS_EKF_Z_AXIS * POS_EKF_AXIS_DIM;

float postionMgrRateDtSum, postionMgrPositionDtSum;
float positionMgrPosHoldElapseDtSum;
float positionMgrPosHoldRatePIDGain;

float positionMgrRTHVxCommand, positionMgrRTHVyCommand;
float positionMgrRTHCompleteDt = 0;
uint8_t positionMgrRTHWasActive = 0;

float positionMgrBrakeVx, positionMgrBrakeVy, positionMgrBrakeAccFFx, positionMgrBrakeAccFFy;
void managePositionTask(void);

uint8_t initPositionManager(void) {
	logString("[Position Manager] Init > Start\n");
	uint8_t status = initGNSS();
	if (status) {
		logString("[Position Manager] GPS Init > Success\n");
	} else {
		logString("[Position Manager] GPS Init > Failed!\n");
		return 0;
	}

	status = positionEKFInit(&positionEkf) && initVenturiBiasEstimator();
	if (status) {
		logString("[Position Manager] EKF Init > Success\n");

		lowPassFilterInit(&positionMgrAccXLPF, POSITION_MGR_X_EST_OUTPUT_ACC_LPF_FREQ);
		lowPassFilterInit(&positionMgrAccYLPF, POSITION_MGR_Y_EST_OUTPUT_ACC_LPF_FREQ);
		lowPassFilterInit(&positionMgrAccZLPF, POSITION_MGR_Z_EST_OUTPUT_ACC_LPF_FREQ);

		lowPassFilterInit(&positionMgrVelXLPF, POSITION_MGR_X_EST_OUTPUT_VEL_LPF_FREQ);
		lowPassFilterInit(&positionMgrVelYLPF, POSITION_MGR_Y_EST_OUTPUT_VEL_LPF_FREQ);
		lowPassFilterInit(&positionMgrVelZLPF, POSITION_MGR_Z_EST_OUTPUT_VEL_LPF_FREQ);

		schedulerAddTask(managePositionTask, POSITION_MANAGEMENT_TASK_FREQUENCY, POSITION_MANAGEMENT_TASK_PRIORITY);
		logString("[Position Manager] All tasks   > Started\n");

		initPositionControl(POSITION_MANAGEMENT_POSITION_CONTROL_FREQUENCY, POSITION_MANAGEMENT_RATE_CONTROL_FREQUENCY);
	} else {
		logString("[Position Manager] EKF Init > Failed\n");
	}
	return status;
}

__ATTR_ITCM_TEXT
void updatePositionVelocity(float vx, float vy, float vz, float dt) {
	float vel;
	// X Axis
	vel = applyDeadBandFloat(0.0f, vx, POSITION_MGR_X_EST_OUTPUT_VEL_DEADBAND);
	vel = constrainToRangeF(vel, -POSITION_MGR_X_EST_OUTPUT_VEL_MAX, POSITION_MGR_X_EST_OUTPUT_VEL_MAX);
	positionCordinateData.xVelocity = lowPassFilterUpdate(&positionMgrVelXLPF, vel, dt);

	// Y Axis
	vel = applyDeadBandFloat(0.0f, vy, POSITION_MGR_Y_EST_OUTPUT_VEL_DEADBAND);
	vel = constrainToRangeF(vel, -POSITION_MGR_Y_EST_OUTPUT_VEL_MAX, POSITION_MGR_Y_EST_OUTPUT_VEL_MAX);
	positionCordinateData.yVelocity = lowPassFilterUpdate(&positionMgrVelYLPF, vel, dt);

	// Z Axis
	vel = applyDeadBandFloat(0.0f, vz, POSITION_MGR_Z_EST_OUTPUT_VEL_DEADBAND);
	vel = constrainToRangeF(vel, -POSITION_MGR_Z_EST_OUTPUT_VEL_MAX, POSITION_MGR_Z_EST_OUTPUT_VEL_MAX);
	positionCordinateData.zVelocity = lowPassFilterUpdate(&positionMgrVelZLPF, vel, dt);
}

__ATTR_ITCM_TEXT
void updatePositionAcceleration(float ax, float ay, float az, float dt) {
	float acc;
	// X Axis
	acc = applyDeadBandFloat(0.0f, ax, POSITION_MGR_X_EST_OUTPUT_ACC_DEADBAND);
	acc = constrainToRangeF(acc, -POSITION_MGR_X_EST_OUTPUT_ACC_MAX, POSITION_MGR_X_EST_OUTPUT_ACC_MAX);
	positionCordinateData.xAcceleration = lowPassFilterUpdate(&positionMgrAccXLPF, acc, dt);

	// Y Axis
	acc = applyDeadBandFloat(0.0f, ay, POSITION_MGR_Y_EST_OUTPUT_ACC_DEADBAND);
	acc = constrainToRangeF(acc, -POSITION_MGR_Y_EST_OUTPUT_ACC_MAX, POSITION_MGR_Y_EST_OUTPUT_ACC_MAX);
	positionCordinateData.yAcceleration = lowPassFilterUpdate(&positionMgrAccYLPF, acc, dt);

	// Z Axis
	acc = applyDeadBandFloat(0.0f, az, POSITION_MGR_Z_EST_OUTPUT_ACC_DEADBAND);
	acc = constrainToRangeF(acc, -POSITION_MGR_Z_EST_OUTPUT_ACC_MAX, POSITION_MGR_Z_EST_OUTPUT_ACC_MAX);
	positionCordinateData.zAcceleration = lowPassFilterUpdate(&positionMgrAccZLPF, acc, dt);
}

__ATTR_ITCM_TEXT
void updatePositionReference() {
	if (fcStatusData.canFly && fcStatusData.isNavDataReliable && fcStatusData.isPositionHomeSet) {
		fcStatusData.positionXRef = positionCordinateData.xPosition;
		fcStatusData.positionYRef = positionCordinateData.yPosition;
	}
}

__ATTR_ITCM_TEXT
void resetPositionCommands() {
	positionCommandData.pitchCommand = 0.0f;
	positionCommandData.rollCommand = 0.0f;
	positionMgrRTHVxCommand = 0;
	positionMgrRTHVyCommand = 0;
	fcStatusData.isRTHComplete = 0;
	positionMgrRTHCompleteDt = 0;
	positionMgrPosHoldElapseDtSum = 0.0f;
	resetPositionControl(1);
}

__ATTR_ITCM_TEXT
void updatePositionRateCommand(float dt) {
	if (isNavModeActive() && fcStatusData.isPositionHomeSet) {
		if (fcStatusData.postionHoldState == POS_HOLD_STATE_SETTLING || fcStatusData.postionHoldState == POS_HOLD_STATE_BRAKING || fcStatusData.postionHoldState == POS_HOLD_STATE_LOCKED) {
			if (fcStatusData.postionHoldState == POS_HOLD_STATE_BRAKING || fcStatusData.postionHoldState == POS_HOLD_STATE_SETTLING) {
				positionMgrPosHoldRatePIDGain = POSITION_MGR_POS_HOLD_BRAKE_RATE_PI_GAIN;
			} else {
				positionMgrPosHoldRatePIDGain = 1.0f;
			}
			controlPositionRateWithGains(dt, positionMgrPosHoldRatePIDGain, positionMgrPosHoldRatePIDGain, 1.0f, positionMgrBrakeAccFFx, positionMgrBrakeAccFFy);
			float pitchCommand, rollCommand;
			convertEarthToBodyCordinates(controlData.positionXControl, controlData.positionYControl, sensorAttitudeData.heading, &pitchCommand, &rollCommand);
			positionCommandData.pitchCommand = pitchCommand;
			positionCommandData.rollCommand = rollCommand;
		} else {
			positionMgrPosHoldRatePIDGain = 1.0f;
			resetPositionCommands();
		}
	} else {
		positionMgrPosHoldRatePIDGain = 1.0f;
		resetPositionCommands();
	}
}

__ATTR_ITCM_TEXT
void resetBrakingStates() {
	positionMgrBrakeVx = 0;
	positionMgrBrakeVy = 0;
	positionMgrBrakeAccFFx = 0;
	positionMgrBrakeAccFFy = 0;
}
__ATTR_ITCM_TEXT
void doBraking(float dt) {
	positionMgrPosHoldElapseDtSum += dt;
	float step = POSITION_MGR_POS_HOLD_BRAKE_DECEL * dt; /* new define, 1.0f */
	positionMgrBrakeVx -= constrainToRangeF(positionMgrBrakeVx, -step, step);
	positionMgrBrakeVy -= constrainToRangeF(positionMgrBrakeVy, -step, step);

	/* earth-frame decel the ramp is currently commanding, sign opposes motion */
	float sgnx = (positionMgrBrakeVx > 0.0f) ? 1.0f : ((positionMgrBrakeVx < 0.0f) ? -1.0f : 0.0f);
	float sgny = (positionMgrBrakeVy > 0.0f) ? 1.0f : ((positionMgrBrakeVy < 0.0f) ? -1.0f : 0.0f);
	positionMgrBrakeAccFFx = -sgnx * POSITION_MGR_POS_HOLD_BRAKE_DECEL;
	positionMgrBrakeAccFFy = -sgny * POSITION_MGR_POS_HOLD_BRAKE_DECEL;

	float vxCmd = constrainToRangeF(positionMgrBrakeVx, -POSITION_MGR_POS_HOLD_BRAKE_MAX_VELOCITY, POSITION_MGR_POS_HOLD_BRAKE_MAX_VELOCITY);
	float vyCmd = constrainToRangeF(positionMgrBrakeVy, -POSITION_MGR_POS_HOLD_BRAKE_MAX_VELOCITY, POSITION_MGR_POS_HOLD_BRAKE_MAX_VELOCITY);
	setExpectedPositionVelocity(dt, vxCmd, vyCmd);
	// Termination conditions
	uint8_t lowGroundSpeed = (getGroundSpeed() <= POSITION_MGR_POS_HOLD_BRAKE_MAX_GROUND_SPEED);
	uint8_t timeoutReached = (positionMgrPosHoldElapseDtSum >= POSITION_MGR_POS_HOLD_BRAKE_ACTIVE_PERIOD);
	if (lowGroundSpeed || timeoutReached) {
		positionMgrPosHoldElapseDtSum = 0.0f;
		resetBrakingStates();
		resetPositionControl(1);
		fcStatusData.postionHoldState = POS_HOLD_STATE_SETTLING;
	}
}

__ATTR_ITCM_TEXT
void updateRTHVelocityCommand(float dt) {
	// Position error to home
	float dx = fcStatusData.positionXHome - positionCordinateData.xPosition;
	float dy = fcStatusData.positionYHome - positionCordinateData.yPosition;
	// Distance to home
	float distance = fastSqrtf(dx * dx + dy * dy);
	// Normalize direction vector
	float dirX = 0.0f;
	float dirY = 0.0f;
	if (distance > 0.01f) {
		float invDist = 1.0f / distance;
		dirX = dx * invDist;
		dirY = dy * invDist;
	}
	// Base cruise speed
	float targetSpeed = fminf(POSITION_MGR_RTH_CRUISE_SPEED, fastSqrtf(2.0f * POSITION_MGR_RTH_BRAKE_DECEL * distance));
	// Slow down near home
	if (distance < POSITION_MGR_RTH_NEAR_HOME_RADIUS) {
		float scale = distance / POSITION_MGR_RTH_NEAR_HOME_RADIUS;
		scale = constrainToRangeF(scale, 0.0f, 1.0f);
		targetSpeed *= scale;
	}
	// Desired velocity command
	float desiredVx = dirX * targetSpeed;
	float desiredVy = dirY * targetSpeed;
	//------------------------------------------------------------------
	// Acceleration limiting (vector magnitude based)
	//------------------------------------------------------------------
	float dvx = desiredVx - positionMgrRTHVxCommand;
	float dvy = desiredVy - positionMgrRTHVyCommand;
	float deltaMag = fastSqrtf(dvx * dvx + dvy * dvy);
	float maxDelta = POSITION_MGR_RTH_MAX_ACCEL * dt;
	if (deltaMag > maxDelta && deltaMag > 0.0001f) {
		float scale = maxDelta / deltaMag;
		dvx *= scale;
		dvy *= scale;
	}
	// Smoothed velocity target
	positionMgrRTHVxCommand += dvx;
	positionMgrRTHVyCommand += dvy;
	//------------------------------------------------------------------
	// Feed velocity target into velocity controller
	//------------------------------------------------------------------
	setExpectedPositionVelocity(dt, positionMgrRTHVxCommand, positionMgrRTHVyCommand);
}

__ATTR_ITCM_TEXT
void updateRTHCompletionStatus(float dt) {
	float dx = fcStatusData.positionXHome - positionCordinateData.xPosition;
	float dy = fcStatusData.positionYHome - positionCordinateData.yPosition;
	uint8_t lowGroundSpeed = (getGroundSpeed() <= POSITION_MGR_RTH_COMPLETE_MAX_GROUND_SPEED);
	float distance = fastSqrtf(dx * dx + dy * dy);
	if (distance <= POSITION_MGR_RTH_HOME_RADIUS) {
		if (positionMgrRTHCompleteDt < POSITION_MGR_RTH_COMPLETE_PERIOD) {
			positionMgrRTHCompleteDt += dt;
		}
	} else {
		positionMgrRTHCompleteDt = 0.0f;
	}
	uint8_t timeoutReached = (positionMgrRTHCompleteDt >= POSITION_MGR_RTH_COMPLETE_PERIOD);
	if (distance <= POSITION_MGR_RTH_HOME_RADIUS && (lowGroundSpeed || timeoutReached)) {
		fcStatusData.isRTHComplete = 1;
	} else {
		fcStatusData.isRTHComplete = 0;
	}
}

__ATTR_ITCM_TEXT
void handleRTHNavigation(float dt) {
	if (positionMgrPosHoldElapseDtSum < POSITION_MGR_RTH_SETTLING_PERIOD) {
		positionMgrPosHoldElapseDtSum += dt;
		controlPositionCordinatesWithGains(dt, fcStatusData.positionXRef, fcStatusData.positionYRef, 1.0f);
	} else {
		if (!fcStatusData.isRTHComplete) {
			updateRTHCompletionStatus(dt);
			updateRTHVelocityCommand(dt);
			updatePositionReference();
		} else {
			controlPositionCordinatesWithGains(dt, fcStatusData.positionXHome, fcStatusData.positionYHome, 1.0f);
		}
	}
}

__ATTR_ITCM_TEXT
void updatePositionCordinateCommand(float dt) {
	if (!rcData.pitchCentered || !rcData.rollCentered) {
		resetPositionCommands();
		resetBrakingStates();
		fcStatusData.postionHoldState = POS_HOLD_STATE_IDLE;
		return;
	}
	if (isNavModeActive() && fcStatusData.isPositionHomeSet) {
		switch (fcStatusData.postionHoldState) {
		case POS_HOLD_STATE_IDLE:
			resetPositionCommands();
			resetBrakingStates();
			positionMgrBrakeVx = positionCordinateData.xVelocity;
			positionMgrBrakeVy = positionCordinateData.yVelocity;
			fcStatusData.postionHoldState = POS_HOLD_STATE_BRAKING;
			break;
		case POS_HOLD_STATE_BRAKING:
			doBraking(dt);
			break;
		case POS_HOLD_STATE_SETTLING:
			positionMgrPosHoldElapseDtSum += dt;
			resetBrakingStates();
			setExpectedPositionVelocity(dt, 0.0f, 0.0f);
			if ((positionMgrPosHoldElapseDtSum >= POSITION_MGR_POS_HOLD_BRAKE_SETTLING_PERIOD && getGroundSpeed() <= 2.0f * POSITION_MGR_POS_HOLD_BRAKE_MAX_GROUND_SPEED) || positionMgrPosHoldElapseDtSum >= POSITION_MGR_POS_HOLD_SETTLING_TIMEOUT) {
				updatePositionReference();
				resetPositionControl(1);
				positionMgrPosHoldElapseDtSum = 0.0f;
				fcStatusData.postionHoldState = POS_HOLD_STATE_LOCKED;
			}
			break;
		case POS_HOLD_STATE_LOCKED:
			resetBrakingStates();
			if (fcStatusData.isNavRTHModeActive) {
				if (!positionMgrRTHWasActive) {           // rising edge: fresh RTH cycle
					positionMgrPosHoldElapseDtSum = 0.0f;
					positionMgrRTHVxCommand = 0.0f;
					positionMgrRTHVyCommand = 0.0f;
					positionMgrRTHCompleteDt = 0.0f;
					fcStatusData.isRTHComplete = 0;
				}
				positionMgrRTHWasActive = 1;
				handleRTHNavigation(dt);
			} else {
				positionMgrRTHWasActive = 0;
				controlPositionCordinatesWithGains(dt, fcStatusData.positionXRef, fcStatusData.positionYRef, 1.0f);
			}
			break;
		}
	} else {
		positionMgrPosHoldElapseDtSum = 0;
		fcStatusData.postionHoldState = POS_HOLD_STATE_IDLE;
		resetPositionCommands();
	}
}

__ATTR_ITCM_TEXT
void managePositionTask(void) {
	float dt = getDeltaTime(POSITION_MANAGER_TASK_TIMER_CHANNEL);
	dt = constrainToRangeF(dt, POSITION_MANAGEMENT_TASK_PERIOD * 0.001f, POSITION_MANAGEMENT_TASK_PERIOD * 4.0f);

	float axEarth = applyDeadBandFloat(0, imuData.axEarthLinear, POSITION_MGR_X_EST_INPUT_ACC_DEADBAND);
	float ayEarth = applyDeadBandFloat(0, imuData.ayEarthLinear, POSITION_MGR_Y_EST_INPUT_ACC_DEADBAND);
	float azEarth = applyDeadBandFloat(0, imuData.azEarthLinear, POSITION_MGR_Z_EST_INPUT_ACC_DEADBAND);

	float axEarthNED = 0;
	float ayEarthNED = 0;
	float azEarthNED = 0;

	//0. Align the IMU earth acclerations to NED.
	alignEarthAccelToNED(axEarth, ayEarth, azEarth, &axEarthNED, &ayEarthNED, &azEarthNED);

	// 1. Prediction (Using raw or slightly scaled earth-frame acc)
	positionEKFPredict(&positionEkf, axEarthNED, ayEarthNED, azEarthNED, dt);

	float *x = positionEkf.x;
	positionCordinateData.xPosition = x[postionEKFXIndex + POS_EKF_STATE_P];
	positionCordinateData.xAccelerationBias = x[postionEKFXIndex + POS_EKF_STATE_B];

	positionCordinateData.yPosition = x[postionEKFYIndex + POS_EKF_STATE_P];
	positionCordinateData.yAccelerationBias = x[postionEKFYIndex + POS_EKF_STATE_B];

	positionCordinateData.zPosition = x[postionEKFZIndex + POS_EKF_STATE_P];
	positionCordinateData.zAccelerationBias = x[postionEKFZIndex + POS_EKF_STATE_B];

	// 2. Filtered Acceleration
	updatePositionAcceleration(axEarthNED - positionCordinateData.xAccelerationBias, ayEarthNED - positionCordinateData.yAccelerationBias, azEarthNED - positionCordinateData.zAccelerationBias, dt);

	// 3. Filtered Velocity
	updatePositionVelocity(x[postionEKFXIndex + POS_EKF_STATE_V], x[postionEKFYIndex + POS_EKF_STATE_V], x[postionEKFZIndex + POS_EKF_STATE_V], dt);

	postionMgrRateDtSum += dt;
	while (postionMgrRateDtSum >= POSITION_MANAGEMENT_RATE_CONTROL_PERIOD) {
		updatePositionRateCommand(POSITION_MANAGEMENT_RATE_CONTROL_PERIOD);
		postionMgrRateDtSum -= POSITION_MANAGEMENT_RATE_CONTROL_PERIOD;
	}
	postionMgrPositionDtSum += dt;
	while (postionMgrPositionDtSum >= POSITION_MANAGEMENT_POSITION_CONTROL_PERIOD) {
		updatePositionCordinateCommand( POSITION_MANAGEMENT_POSITION_CONTROL_PERIOD);
		postionMgrPositionDtSum -= POSITION_MANAGEMENT_POSITION_CONTROL_PERIOD;
	}

	positionCordinateData.positionProcessDt = dt;
}

__ATTR_ITCM_TEXT
void loadAndProcessGNSSData() {
	if (readGNSSData()) {
		float dt = getDeltaTime(POSITION_MANAGER_GNSS_TIMER_CHANNEL);
		gnssData.updateDt = dt;
		updateGNSSDataReliability(dt);

		if (fcStatusData.isNavDataReliable) {
			uint8_t wasHomeJustSet = 0;
			if (!fcStatusData.isPositionHomeSet) {
				updateHomePositionAcquisition(dt);
				wasHomeJustSet = fcStatusData.isPositionHomeSet;
			}

			if (fcStatusData.isPositionHomeSet) {
				convertGNSSToXYCordinates(gnssData.latitude, gnssData.longitude, fcStatusData.positionLatHome, fcStatusData.positionLongHome, &positionCordinateData.xPositionRaw, &positionCordinateData.yPositionRaw);
				if (wasHomeJustSet) {
					resetPVEstimation(POS_EKF_X_AXIS, 1);
					resetPVEstimation(POS_EKF_Y_AXIS, 1);
					fcStatusData.positionXHome = positionCordinateData.xPositionRaw;
					fcStatusData.positionYHome = positionCordinateData.yPositionRaw;
				}

				// Update Velocity and Position
				updateXYVelocityGNSS(gnssData.sAcc, gnssData.velN, gnssData.velE, dt);
				updateZVelocityGNSS(gnssData.sAcc, -gnssData.velD, fcStatusData.isNavDataReliable && fcStatusData.isNavModeActive, dt); //GNSS is +ve down ( NED )
				updateXYPositionGNSS(gnssData.hAcc, positionCordinateData.xPositionRaw, positionCordinateData.yPositionRaw, dt);
				updateZPositionGNSS(gnssData.vAcc, gnssData.heightMSL - fcStatusData.positionZHome, fcStatusData.isNavDataReliable && fcStatusData.isNavModeActive, dt);
			}

		} else {
			if (!fcStatusData.isPositionHomeSet) {
				resetHomePositionAcquisition();
			}
		}
	}
}

__ATTR_ITCM_TEXT
void doPositionManagement() {
	if (fcStatusData.hasCrashed) {
		resetPositionManager();
	} else if (fcStatusData.canStabilize && !positionManagerWasInStabMode) {
		positionManagerWasInStabMode = 1;
	} else if (positionManagerWasInStabMode && fcStatusData.isStabilized) {
		positionManagerWasInStabMode = 0;
	}
	loadAndProcessGNSSData();
}

void resetPositionManager(void) {
	lowPassFilterReset(&positionMgrAccXLPF);
	lowPassFilterReset(&positionMgrAccYLPF);
	lowPassFilterReset(&positionMgrAccZLPF);

	lowPassFilterReset(&positionMgrVelXLPF);
	lowPassFilterReset(&positionMgrVelYLPF);
	lowPassFilterReset(&positionMgrVelZLPF);

	positionEKFInvalidate(&positionEkf, POS_EKF_X_AXIS);
	positionEKFInvalidate(&positionEkf, POS_EKF_Y_AXIS);
	positionEKFInvalidate(&positionEkf, POS_EKF_Z_AXIS);

	resetVenturiBiasEstimator();
	resetPositionControl(1);
	resetHomePositionAcquisition();

	positionManagerWasInStabMode = 0;
	positionCommandData.pitchCommand = 0.0f;
	positionCommandData.rollCommand = 0.0f;
	fcStatusData.isPositionHomeSet = 0;
	positionMgrPosHoldElapseDtSum = 0;
	positionMgrPosHoldRatePIDGain = 1.0f;
	fcStatusData.postionHoldState = POS_HOLD_STATE_IDLE;
	positionMgrRTHVxCommand = 0;
	positionMgrRTHVyCommand = 0;
	positionMgrRTHCompleteDt = 0;
	positionMgrRTHWasActive = 0;
}

