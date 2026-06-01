#include "PositionManager.h"

#include <sys/_stdint.h>

#include "../../control/ControlData.h"
#include "../../control/position/PositionControl.h"
#include "../../calibration/Calibration.h"
#include "../../dsp/LowPassFilter.h"
#include "../../FCConfig.h"
#include "../../imu/IMU.h"
#include "../../logger/Logger.h"
#include "../../memory/Memory.h"
#include "../../sensors/attitude/AttitudeSensor.h"
#include "../../sensors/position/GNSS.h"
#include "../../status/FCStatus.h"
#include "../../timers/DeltaTimer.h"
#include "../../timers/Scheduler.h"
#include "../../util/MathUtil.h"
#include "estimator/PositionEstimator.h"
#include "estimator/VenturiBiasEstimator.h"
#include "helpers/PositionManagerHelper.h"
#include "../../sensors/rc/RCSensor.h"

POSITION_EKF positionEkf;
LOWPASSFILTER positionMgrAccXLPF, positionMgrAccYLPF, positionMgrAccZLPF;
LOWPASSFILTER positionMgrVelXLPF, positionMgrVelYLPF, positionMgrVelZLPF;

uint8_t positionManagerWasInStabMode;
POSITION_CORDINATE_DATA positionCordinateData;
POSITION_COMMAND_DATA positionCommandData;

float postionMgrRateDtSum, postionMgrPositionDtSum;
float positionMgrPosHoldElapseDtSum;
float positionMgrPosHoldRatePIDGain;

float positionMgrRTHVxCommand, positionMgrRTHVyCommand;
float positionMgrRTHCompleteDt = 0;

// Static variables to persist sampling state across function calls
uint16_t positionMgrHomeRefSampleCount = 0;
double positionMgrHomeRefLatSum = 0.0;
double positionMgrHomeRefLongSum = 0.0;

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

		lowPassFilterInit(&positionMgrVelXLPF, POSITION_MGR_X_VEL_LPF_FREQ);
		lowPassFilterInit(&positionMgrVelYLPF, POSITION_MGR_Y_VEL_LPF_FREQ);
		lowPassFilterInit(&positionMgrVelZLPF, POSITION_MGR_Z_VEL_LPF_FREQ);

		schedulerAddTask(managePositionTask, POSITION_MANAGEMENT_TASK_FREQUENCY, POSITION_MANAGEMENT_TASK_PRIORITY);
		logString("[Position Manager] All tasks   > Started\n");

		initPositionControl(POSITION_MANAGEMENT_POSITION_CONTROL_FREQUENCY,
		POSITION_MANAGEMENT_RATE_CONTROL_FREQUENCY);
	} else {
		logString("[Position Manager] EKF Init > Failed\n");
	}
	return status;
}

__ATTR_ITCM_TEXT
void updatePositionVelocity(float vx, float vy, float vz, float dt) {
	float vel;
	// X Axis
	vel = applyDeadBandFloat(0.0f, vx, POSITION_MGR_X_VEL_DEADBAND);
	vel = constrainToRangeF(vel, -POSITION_MGR_X_VEL_MAX, POSITION_MGR_X_VEL_MAX);
	positionCordinateData.xVelocity = lowPassFilterUpdate(&positionMgrVelXLPF, vel, dt);

	// Y Axis
	vel = applyDeadBandFloat(0.0f, vy, POSITION_MGR_Y_VEL_DEADBAND);
	vel = constrainToRangeF(vel, -POSITION_MGR_Y_VEL_MAX, POSITION_MGR_Y_VEL_MAX);
	positionCordinateData.yVelocity = lowPassFilterUpdate(&positionMgrVelYLPF, vel, dt);

	// Z Axis
	vel = applyDeadBandFloat(0.0f, vz, POSITION_MGR_Z_VEL_DEADBAND);
	vel = constrainToRangeF(vel, -POSITION_MGR_Z_VEL_MAX, POSITION_MGR_Z_VEL_MAX);
	positionCordinateData.zVelocity = lowPassFilterUpdate(&positionMgrVelZLPF, vel, dt);
}

__ATTR_ITCM_TEXT
void updatePositionAcceleration(float ax, float ay, float az, float dt) {
	float acc;
	// X Axis
	acc = applyDeadBandFloat(0.0f, ax, POSITION_MGR_X_ACC_DEADBAND);
	acc = constrainToRangeF(acc, -POSITION_MGR_X_ACC_MAX, POSITION_MGR_X_ACC_MAX);
	positionCordinateData.xAcceleration = lowPassFilterUpdate(&positionMgrAccXLPF, acc, dt);

	// Y Axis
	acc = applyDeadBandFloat(0.0f, ay, POSITION_MGR_Y_ACC_DEADBAND);
	acc = constrainToRangeF(acc, -POSITION_MGR_Y_ACC_MAX, POSITION_MGR_Y_ACC_MAX);
	positionCordinateData.yAcceleration = lowPassFilterUpdate(&positionMgrAccYLPF, acc, dt);

	// Z Axis
	acc = applyDeadBandFloat(0.0f, az, POSITION_MGR_Z_ACC_DEADBAND);
	acc = constrainToRangeF(acc, -POSITION_MGR_Z_ACC_MAX, POSITION_MGR_Z_ACC_MAX);
	positionCordinateData.zAcceleration = lowPassFilterUpdate(&positionMgrAccZLPF, acc, dt);
}

__ATTR_ITCM_TEXT
void updatePositionReference() {
	if (fcStatusData.canFly && fcStatusData.isPositionDataReliable && fcStatusData.isPositionHomeSet) {
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
	resetPositionControl(1);
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
	float targetSpeed = POSITION_MGR_RTH_CRUISE_SPEED;
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
void calculateBrakeAnchorBallistic(void) {
	float vx = positionCordinateData.xVelocity;
	float vy = positionCordinateData.yVelocity;
	// Calculate total ground speed squared and magnitude
	float speedSq = (vx * vx) + (vy * vy);
	// Protection against zero-velocity noise or division-by-zero during hover
	if (speedSq < 0.001f) {
		fcStatusData.positionXRef = positionCordinateData.xPosition;
		fcStatusData.positionYRef = positionCordinateData.yPosition;
		return;
	}
	float speed = sqrtf(speedSq); // Single precise sqrtf inside ITCM
	// -------------------------------------------------
	// 1. Effective braking authority (Vector Bounded)
	// -------------------------------------------------
	float brakeGain = fmaxf(POSITION_MGR_POS_HOLD_BRAKE_STRENGTH, 0.1f);
	float effectiveDecel = POSITION_MGR_POS_HOLD_NATURAL_DECEL * brakeGain;
	effectiveDecel = fmaxf(effectiveDecel, 0.1f);
	// -------------------------------------------------
	// 2. Vector Ballistic & Lag Magnitude Calculation
	// -------------------------------------------------
	// Total scalar distance required to stop based on vector speed
	float invDenominator = 1.0f / (2.0f * effectiveDecel);
	float scalarBrakeOffset = (speedSq * invDenominator) * POSITION_MGR_POS_HOLD_BALLISTIC_SCALE;
	float scalarLagOffset = speed * POSITION_MGR_POS_HOLD_EKF_LAG_SEC;
	float totalTargetOffset = scalarBrakeOffset + scalarLagOffset;
	// -------------------------------------------------
	// 3. Safety Magnitude Clamp (Preserves Heading Angle)
	// -------------------------------------------------
	if (totalTargetOffset > POSITION_MGR_POS_HOLD_MAX_BRAKE_OFFSET) {
		totalTargetOffset = POSITION_MGR_POS_HOLD_MAX_BRAKE_OFFSET;
	}
	// -------------------------------------------------
	// 4. Directional Projection & Final Anchor Assignment
	// -------------------------------------------------
	// Compute the unit directional vectors of motion
	float dirX = vx / speed;
	float dirY = vy / speed;
	// Distribute the safe, combined magnitude back to the world coordinate components
	fcStatusData.positionXRef = positionCordinateData.xPosition + (dirX * totalTargetOffset);
	fcStatusData.positionYRef = positionCordinateData.yPosition + (dirY * totalTargetOffset);
}

__ATTR_ITCM_TEXT
void handlePositionBraking(float dt) {
	positionMgrPosHoldElapseDtSum += dt;

	// 1. Calculate progress from 0.0 to 1.0 over the active period
	float progress = positionMgrPosHoldElapseDtSum / POSITION_MGR_POS_HOLD_BRAKE_ACTIVE_PERIOD;
	progress = constrainToRangeF(progress, 0.0f, 1.0f);

	// 2. Compute progressive brake strength (Ramp up from floor to max)
	// Defines the starting baseline authority (25% of maximum strength)
	float minBrakeStrength = POSITION_MGR_POS_HOLD_BRAKE_STRENGTH * 0.25f;

	// Linearly interpolate from minBrakeStrength to max POSITION_MGR_POS_HOLD_BRAKE_STRENGTH over the duration
	float brakeStrength = minBrakeStrength + (progress * (POSITION_MGR_POS_HOLD_BRAKE_STRENGTH - minBrakeStrength));

	// Safety clamp to ensure numeric bounds match constraints perfectly
	brakeStrength = constrainToRangeF(brakeStrength, minBrakeStrength, POSITION_MGR_POS_HOLD_BRAKE_STRENGTH);

	// 3. Generate a progressive counter-velocity command based on ground speed
	float vxCmd = -positionCordinateData.xVelocity * brakeStrength;
	float vyCmd = -positionCordinateData.yVelocity * brakeStrength;

	// Clamp maximum permitted feed-forward command velocity components
	vxCmd = constrainToRangeF(vxCmd, -POSITION_MGR_POS_HOLD_BRAKE_MAX_VELOCITY, POSITION_MGR_POS_HOLD_BRAKE_MAX_VELOCITY);
	vyCmd = constrainToRangeF(vyCmd, -POSITION_MGR_POS_HOLD_BRAKE_MAX_VELOCITY, POSITION_MGR_POS_HOLD_BRAKE_MAX_VELOCITY);

	// Pipe the generated target velocity directly into the inner position controller
	setExpectedPositionVelocity(dt, vxCmd, vyCmd);

	// 4. Check termination conditions
	uint8_t lowGroundSpeed = (getGroundSpeed() <= POSITION_MGR_POS_HOLD_BRAKE_MAX_GROUND_SPEED);
	uint8_t timeoutReached = (positionMgrPosHoldElapseDtSum >= POSITION_MGR_POS_HOLD_BRAKE_ACTIVE_PERIOD);

	if (lowGroundSpeed || timeoutReached) {
		positionMgrPosHoldElapseDtSum = 0.0f;
		resetPositionControl(1);
		fcStatusData.postionHoldState = POS_HOLD_STATE_SETTLING;
	}
}

__ATTR_ITCM_TEXT
void handleRTHNavigation(float dt) {
	// Allow coordinates to stabilize briefly before beginning transition
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
void updatePositionRateCommand(float dt) {
	if ((fcStatusData.isRTHModeActive || fcStatusData.isPositionHoldModeActive) && fcStatusData.isPositionDataReliable) {
		if (fcStatusData.postionHoldState == POS_HOLD_STATE_SETTLING || fcStatusData.postionHoldState == POS_HOLD_STATE_BRAKING || fcStatusData.postionHoldState == POS_HOLD_STATE_LOCKED) {
			if (fcStatusData.postionHoldState == POS_HOLD_STATE_BRAKING || fcStatusData.postionHoldState == POS_HOLD_STATE_SETTLING) {
				positionMgrPosHoldRatePIDGain = POSITION_MGR_POS_HOLD_BRAKE_RATE_PI_GAIN;
			} else {
				positionMgrPosHoldRatePIDGain = 1.0f;
			}

			controlPositionRateWithGains(dt, positionMgrPosHoldRatePIDGain, positionMgrPosHoldRatePIDGain, 1.0f);
			float pitchCommand, rollCommand;
			float positionXControl = controlData.positionXControl;
			float positionYControl = controlData.positionYControl;

			convertEarthToBodyCordinates(positionXControl, positionYControl, sensorAttitudeData.heading, &pitchCommand, &rollCommand);

			float scaledPitchCommand = pitchCommand;
			float scaledRollCommand = rollCommand;

			float magnitudeSq = (scaledPitchCommand * scaledPitchCommand) + (scaledRollCommand * scaledRollCommand);
			float maxLimit = POSITION_MGR_MAX_POS_COMMAND;
			if (magnitudeSq > (maxLimit * maxLimit)) {
				float magnitude = fastSqrtf(magnitudeSq);
				float scale = maxLimit / magnitude;
				scaledPitchCommand *= scale;
				scaledRollCommand *= scale;
			}

			positionCommandData.pitchCommand = constrainToRangeF(scaledPitchCommand, -maxLimit, maxLimit);
			positionCommandData.rollCommand = constrainToRangeF(scaledRollCommand, -maxLimit, maxLimit);

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
void updatePositionCordinateCommand(float dt) {
	if (!rcData.pitchCentered || !rcData.rollCentered) {
		fcStatusData.postionHoldState = POS_HOLD_STATE_IDLE;
		resetPositionCommands();
		return;
	}
	switch (fcStatusData.postionHoldState) {
	case POS_HOLD_STATE_IDLE:
		positionMgrPosHoldElapseDtSum = 0.0f;
		if (fcStatusData.isPositionHoldModeActive || fcStatusData.isRTHModeActive) {
			calculateBrakeAnchorBallistic();
			fcStatusData.postionHoldState = POS_HOLD_STATE_BRAKING;
		}
		resetPositionCommands();
		break;
	case POS_HOLD_STATE_BRAKING:
		handlePositionBraking(dt);
		break;
	case POS_HOLD_STATE_SETTLING:
		positionMgrPosHoldElapseDtSum += dt;
		//updatePositionReference();
		controlPositionCordinatesWithGains(dt, fcStatusData.positionXRef, fcStatusData.positionYRef, 0.5f);
		if (positionMgrPosHoldElapseDtSum >= POSITION_MGR_POS_HOLD_BRAKE_SETTLING_PERIOD) {
			fcStatusData.postionHoldState = POS_HOLD_STATE_LOCKED;
		}
		break;
	case POS_HOLD_STATE_LOCKED:
		positionMgrPosHoldElapseDtSum = 0;
		controlPositionCordinatesWithGains(dt, fcStatusData.positionXRef, fcStatusData.positionYRef, 1.0f);
		/*
		 if (fcStatusData.isRTHModeActive) {
		 handleRTHNavigation(dt);
		 } else {
		 controlPositionCordinatesWithGains(dt, fcStatusData.positionXRef, fcStatusData.positionYRef, 1.0f);
		 }
		 */
		break;
	}
}

__ATTR_ITCM_TEXT
void managePositionTask(void) {
	float dt = getDeltaTime(POSITION_MANAGER_TASK_TIMER_CHANNEL);
	dt = constrainToRangeF(dt, POSITION_MANAGEMENT_TASK_PERIOD * 0.001f, POSITION_MANAGEMENT_TASK_PERIOD * 4.0f);

	float axEarth = applyDeadBandFloat(0, imuData.axEarthLinear, POSITION_MGR_X_ESTIMATION_ACC_DEADBAND);
	float ayEarth = applyDeadBandFloat(0, imuData.ayEarthLinear, POSITION_MGR_Y_ESTIMATION_ACC_DEADBAND);
	float azEarth = applyDeadBandFloat(0, imuData.azEarthLinear, POSITION_MGR_Z_ESTIMATION_ACC_DEADBAND);

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
	updatePositionAcceleration(axEarth - positionCordinateData.xAccelerationBias, ayEarth - positionCordinateData.yAccelerationBias, azEarth - positionCordinateData.zAccelerationBias, dt);

	// 3. Filtered Velocity
	updatePositionVelocity(x[1], x[4], x[7], dt);

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

// Define the number of samples you want to capture (e.g., 50 samples)
#define POSITION_MGR_HOME_SAMPLE_TARGET   50

__ATTR_ITCM_TEXT
void doPositionManagement() {
	if (fcStatusData.hasCrashed) {
		resetPositionManager();
	} else if (fcStatusData.canStabilize && !positionManagerWasInStabMode) {
		positionManagerWasInStabMode = 1;
		positionEKFSetMode(&positionEkf, 1);
	} else if (positionManagerWasInStabMode && fcStatusData.isStabilized) {
		positionManagerWasInStabMode = 0;
		positionEKFSetMode(&positionEkf, 0);
	}

	if (readGNSSData()) {
		float dt = getDeltaTime(POSITION_MANAGER_GPS_TIMER_CHANNEL);
		gnssData.updateDt = dt;
		updatePositionDataReliability(dt);
		if (fcStatusData.isPositionDataReliable && fcStatusData.canFly) {
			uint8_t wasHomeJustSet = 0;
			if (!fcStatusData.isPositionHomeSet) {
				if (positionMgrHomeRefSampleCount == 0) {
					positionMgrHomeRefLatSum = 0.0;
					positionMgrHomeRefLongSum = 0.0;
				}
				positionMgrHomeRefLatSum += gnssData.latitude;
				positionMgrHomeRefLongSum += gnssData.longitude;
				positionMgrHomeRefSampleCount++;
				if (positionMgrHomeRefSampleCount >= POSITION_MGR_HOME_SAMPLE_TARGET) {
					fcStatusData.positionLatHome = positionMgrHomeRefLatSum / POSITION_MGR_HOME_SAMPLE_TARGET;
					fcStatusData.positionLongHome = positionMgrHomeRefLongSum / POSITION_MGR_HOME_SAMPLE_TARGET;
					fcStatusData.isPositionHomeSet = 1;
					wasHomeJustSet = 1;
					positionMgrHomeRefSampleCount = 0;
				}
			}

			if (fcStatusData.isPositionHomeSet) {
				convertGNSSToSICordinates(gnssData.latitude, gnssData.longitude, fcStatusData.positionLatHome, fcStatusData.positionLongHome, &positionCordinateData.xPositionRaw, &positionCordinateData.yPositionRaw);
				if (wasHomeJustSet) {
					positionEKFInvalidateAxis(&positionEkf,POS_EKF_X_AXIS);
					positionEKFInvalidateAxis(&positionEkf,POS_EKF_Y_AXIS);

					fcStatusData.positionXHome = positionCordinateData.xPositionRaw;
					fcStatusData.positionYHome = positionCordinateData.yPositionRaw;
				}
				// Adaptive Velocity Measurement Noise Logic
				float dynamicRv = getEstimatedXYDynamicRV(gnssData.sAcc);
				float velN = applyDeadBandFloat(0.0f, gnssData.velN, POS_ESTIMATOR_DYNAMIC_XY_VEL_DEADBAND);
				float velE = applyDeadBandFloat(0.0f, gnssData.velE, POS_ESTIMATOR_DYNAMIC_XY_VEL_DEADBAND);
				positionEKFUpdateXYVel(&positionEkf, velN, velE, dynamicRv);

				// Calculate dynamic R for Position
				float dynamicRp = getEstimatedXYDynamicRV(gnssData.hAcc);
				positionEKFSetDymamicRP(&positionEkf, POS_EKF_X_AXIS, dynamicRp);
				positionEKFSetDymamicRP(&positionEkf, POS_EKF_Y_AXIS, dynamicRp);
				updatePositionManagerXYPosition(positionCordinateData.xPositionRaw,positionCordinateData.yPositionRaw, dt);
			}

		}
	}
}

void resetPositionManager(void) {
	lowPassFilterReset(&positionMgrAccXLPF);
	lowPassFilterReset(&positionMgrAccYLPF);
	lowPassFilterReset(&positionMgrAccZLPF);

	lowPassFilterReset(&positionMgrVelXLPF);
	lowPassFilterReset(&positionMgrVelYLPF);
	lowPassFilterReset(&positionMgrVelZLPF);

	positionEKFInvalidateAxis(&positionEkf,POS_EKF_X_AXIS);
	positionEKFInvalidateAxis(&positionEkf,POS_EKF_Y_AXIS);
	positionEKFInvalidateAxis(&positionEkf,POS_EKF_Z_AXIS);

	resetVenturiBiasEstimator();
	resetPositionControl(1);

	positionManagerWasInStabMode = 0;
	positionCommandData.pitchCommand = 0.0f;
	positionCommandData.rollCommand = 0.0f;
	fcStatusData.isPositionHomeSet = 0;
	positionMgrHomeRefSampleCount = 0;
	positionMgrPosHoldElapseDtSum = 0;
	positionMgrPosHoldRatePIDGain = 1.0f;
	fcStatusData.postionHoldState = POS_HOLD_STATE_IDLE;
	positionMgrRTHVxCommand = 0;
	positionMgrRTHVyCommand = 0;
	positionMgrRTHCompleteDt = 0;

	positionEKFResetXYVel(&positionEkf);
}

__ATTR_ITCM_TEXT
void updatePositionManagerZPosition(float zPos, float dt) {
	positionCordinateData.positionZUpdateDt = dt;
	positionCordinateData.zPositionRaw = zPos;

#if POSITION_MGR_VENTURI_ESTIMATE_ENABLED == 1
	float venturiBias = updateVenturiBiasEstimate(dt);
#else
	float venturiBias = 0.0f;
#endif

#if POSITION_MGR_Z_ENABLE_DYNAMIC_R  == 1
	float dynamicR = getEstimatedZDynamicRP(&positionEkf, zPos, venturiBias, imuData.axEarthLinear, imuData.ayEarthLinear, imuData.azEarthLinear);
	positionEKFSetDymamicRP(&positionEkf, POS_EKF_Z_AXIS, dynamicR);
#endif

	positionEKFUpdateZMeasureWithBias(&positionEkf, zPos, venturiBias);
}

__ATTR_ITCM_TEXT
void updatePositionManagerXYPosition(float xPos, float yPos, float dt) {
	positionCordinateData.positionXYUpdateDt = dt;
	positionEKFUpdateXYMeasure(&positionEkf, xPos, yPos);
}

