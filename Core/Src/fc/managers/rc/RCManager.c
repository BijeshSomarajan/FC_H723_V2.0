#include "RCManager.h"

#include "../../sensors/attitude/AttitudeSensor.h"
#include "../../calibration/Calibration.h"
#include "../../logger/Logger.h"
#include "../../sensors/rc/RCSensor.h"
#include "../../status/FCStatus.h"
#include "../../timers/Scheduler.h"
#include "../../timers/DeltaTimer.h"
#include "../../util/MathUtil.h"
#include "../../FCConfig.h"

RC_DATA rcData;

float rcActiveCheckDt = 0;
float rcStablilizationDt = 0;

float rcStickPitchGain = 0;
float rcStickRollGain = 0;
float rcStickYawGain = 0;
float rcStickThrottleGain = 0;

void updateRCDataTask(void);
void resetFCStatus();
void determineFCState(float dt);
void rcTask(void);

/************************************************************************/
/*Loads the stick middle values                                         */
/************************************************************************/
void loadRCStickMids() {
	rcData.RC_MID_DATA[RC_TH_CHANNEL_INDEX] = getCalibrationValue(CALIB_PROP_RC_THROTTLE_OFFSET_ADDR);
	rcData.RC_MID_DATA[RC_PITCH_CHANNEL_INDEX] = getCalibrationValue(CALIB_PROP_RC_PITCH_OFFSET_ADDR);
	rcData.RC_MID_DATA[RC_ROLL_CHANNEL_INDEX] = getCalibrationValue(CALIB_PROP_RC_ROLL_OFFSET_ADDR);
	rcData.RC_MID_DATA[RC_YAW_CHANNEL_INDEX] = getCalibrationValue(CALIB_PROP_RC_YAW_OFFSET_ADDR);
}

/**
 * Initializes the Radio Controller
 */
uint8_t initRCManager() {
	if (initRCSensor()) {
		logString("[RCManager] : Sensor Init > Success\n");
		loadRCStickMids();
		configureRCStickControl();
		logString("[RCManager] : Stick Controls Init > Success\n");
		schedulerAddTask(rcTask, RC_TASK_FREQUENCY, RC_TASK_PRIORITY);
		logString("[RCManager] : Task Scheduler Init > Success\n");
		return 1;
	} else {
		logString("[RCManager] : Sensor Init > Failure\n");
	}
	return 0;
}

void rcCheckActive(float dt) {
	uint8_t active = isRCTXxActive();
	if (!active) {
		if (rcActiveCheckDt >= RC_ACTIVE_CHECK_THRESHOLD_PERIOD) {
			fcStatusData.isTxOn = 0;
		} else {
			rcActiveCheckDt += dt;
		}
	} else {
		fcStatusData.isTxOn = 1;
		rcActiveCheckDt = 0;
	}
	rcData.failSafeCheckDt = dt;
}

void rcTask() {
	float dt = getDeltaTime(RC_TASK_TIMER_CHANNEL);
	rcCheckActive(dt);
}

void doRCManagement() {
	if (readRCSensor()) {
		rcData.updateDt = getDeltaTime(RC_READ_TIMER_CHANNEL);
		processRCData(rcData.updateDt);
		determineFCState(rcData.updateDt);
	}
}

/**
 * Process the RC Data
 */
void processRCData(float dt) {

	loadRCStickDelta();

	rcData.RC_DELTA_DATA[RC_FLIGHT_MODE_CHANNEL_INDEX] = getRCValue(RC_FLIGHT_MODE_CHANNEL_INDEX);
	rcData.RC_DELTA_DATA[RC_ALT_MODE_CHANNEL_INDEX] = getRCValue(RC_ALT_MODE_CHANNEL_INDEX);

	fcStatusData.canStart = canStartModel();
	fcStatusData.canArm = (fcStatusData.canStart && (!fcStatusData.canFly && !fcStatusData.isStabilized && !fcStatusData.canStabilize) ? canArmModel() : 0);
	// Capture stick centers
	rcData.pitchCentered = isPitchCentered();
	rcData.rollCentered = isRollCentered();
	rcData.yawCentered = isYawCentered();
	rcData.throttleCentered = isThrottleCentered();

	// Apply RC stick rates
	applyRCStickEffectiveness();
	// Set the FC status
	fcStatusData.enableAltitudeHold = canEnableAltHold();
	// Note order is important
	fcStatusData.isRTHModeActive = isRTHModeActive();
	fcStatusData.isGlobalPosHoldModeActive = isGlobalPosHoldModeActive();
	fcStatusData.enablePositionHold = canEnableGlobalPosHold();
	fcStatusData.enableRTH = canEnableRTH();
	fcStatusData.isTerrainAltModeActive = isTerrainAltModeActive();
	fcStatusData.isLandingModeActive = canEnableLandingMode();
	fcStatusData.isHeadLessModeActive = isHeadLessModeActive();
}

void determineFCState(float dt) {
	if (fcStatusData.hasCrashed || !fcStatusData.canStart) {
		resetFCStatus();
		if (!fcStatusData.canStart) {
			fcStatusData.hasCrashed = 0;
		}
	} else if (fcStatusData.isStabilized && fcStatusData.canFly) {
		rcStablilizationDt = 0;
		fcStatusData.canStabilize = 0;
	} else if (fcStatusData.isStabilized && !fcStatusData.canFly) {
		fcStatusData.canFly = 1;
		fcStatusData.canStabilize = 0;
		rcStablilizationDt = 0;
	} else if ((fcStatusData.canArm && !fcStatusData.isStabilized) || fcStatusData.canStabilize) {
		fcStatusData.canStabilize = 1;
		rcStablilizationDt += dt;
		if (rcStablilizationDt >= FC_STABLILIZATION_PERIOD) {
			fcStatusData.isStabilized = 1;
			fcStatusData.canStabilize = 0;
			rcStablilizationDt = 0;
		}
	}
}

void resetFCStatus() {
	fcStatusData.canArm = 0;
	fcStatusData.canFly = 0;
	fcStatusData.canStabilize = 0;
	fcStatusData.isStabilized = 0;
	rcStablilizationDt = 0;
}

void resetRCManager() {
	rcData.pitchCentered = 1;
	rcData.rollCentered = 1;
	rcData.yawCentered = 1;
	rcData.throttleCentered = 1;
	rcStablilizationDt = 0;

	rcData.RC_EFFECTIVE_DATA[RC_TH_CHANNEL_INDEX] = 0;
	rcData.RC_EFFECTIVE_DATA[RC_PITCH_CHANNEL_INDEX] = 0;
	rcData.RC_EFFECTIVE_DATA[RC_ROLL_CHANNEL_INDEX] = 0;
	rcData.RC_EFFECTIVE_DATA[RC_YAW_CHANNEL_INDEX] = 0;

	rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX] = 0;
	rcData.RC_DELTA_DATA[RC_PITCH_CHANNEL_INDEX] = 0;
	rcData.RC_DELTA_DATA[RC_ROLL_CHANNEL_INDEX] = 0;
	rcData.RC_DELTA_DATA[RC_YAW_CHANNEL_INDEX] = 0;

	rcData.RC_DELTA_DATA[RC_START_CHANNEL_INDEX] = 0;
	rcData.RC_DELTA_DATA[RC_POS_CHANNEL_INDEX] = 0;
	rcData.RC_DELTA_DATA[RC_LAND_CHANNEL_INDEX] = 0;
	rcData.RC_DELTA_DATA[RC_HEADING_CHANNEL_INDEX] = 0;
}

/*
 **
 * Configure the stick rate PIDs
 */
void configureRCStickControl() {
	rcStickThrottleGain = getScaledCalibrationValue(CALIB_PROP_RC_THROTTLE_RATE_K_ADDR);
	rcStickPitchGain = getScaledCalibrationValue(CALIB_PROP_RC_PITCH_RATE_P_ADDR);
	rcStickRollGain = getScaledCalibrationValue(CALIB_PROP_RC_ROLL_RATE_P_ADDR);
	rcStickYawGain = getScaledCalibrationValue(CALIB_PROP_RC_YAW_RATE_P_ADDR);
}

int16_t getThrottleChannelValue() {
	return rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX];
}

int16_t getPitchChannelValue() {
	return rcData.RC_DELTA_DATA[RC_PITCH_CHANNEL_INDEX];
}

int16_t getRollChannelValue() {
	return rcData.RC_DELTA_DATA[RC_ROLL_CHANNEL_INDEX];;
}

int16_t getYawChannelValue() {
	return rcData.RC_DELTA_DATA[RC_YAW_CHANNEL_INDEX];;
}

/**
 * Apply the RC stick effectiveness
 */
void applyRCStickEffectiveness() {
	rcData.RC_EFFECTIVE_DATA[RC_TH_CHANNEL_INDEX] = rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX] * rcStickThrottleGain;
	rcData.RC_EFFECTIVE_DATA[RC_PITCH_CHANNEL_INDEX] = rcData.RC_DELTA_DATA[RC_PITCH_CHANNEL_INDEX] * rcStickPitchGain;
	rcData.RC_EFFECTIVE_DATA[RC_ROLL_CHANNEL_INDEX] = rcData.RC_DELTA_DATA[RC_ROLL_CHANNEL_INDEX] * rcStickRollGain;
	rcData.RC_EFFECTIVE_DATA[RC_YAW_CHANNEL_INDEX] = rcData.RC_DELTA_DATA[RC_YAW_CHANNEL_INDEX] * rcStickYawGain;
}

/************************************************************************/
/* Get RC Stick Delta by subtracting from the stick middle values        */
/************************************************************************/
void loadRCStickDelta() {
	// Load stick deltas
	rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX] = getRCValue(RC_TH_CHANNEL_INDEX) - rcData.RC_MID_DATA[RC_TH_CHANNEL_INDEX];
	rcData.RC_DELTA_DATA[RC_PITCH_CHANNEL_INDEX] = getRCValue(RC_PITCH_CHANNEL_INDEX) - rcData.RC_MID_DATA[RC_PITCH_CHANNEL_INDEX];
	rcData.RC_DELTA_DATA[RC_ROLL_CHANNEL_INDEX] = getRCValue(RC_ROLL_CHANNEL_INDEX) - rcData.RC_MID_DATA[RC_ROLL_CHANNEL_INDEX];
	rcData.RC_DELTA_DATA[RC_YAW_CHANNEL_INDEX] = getRCValue(RC_YAW_CHANNEL_INDEX) - rcData.RC_MID_DATA[RC_YAW_CHANNEL_INDEX];
	// Aux channels
	rcData.RC_DELTA_DATA[RC_POS_CHANNEL_INDEX] = getRCValue(RC_POS_CHANNEL_INDEX);
	rcData.RC_DELTA_DATA[RC_LAND_CHANNEL_INDEX] = getRCValue(RC_LAND_CHANNEL_INDEX);
	rcData.RC_DELTA_DATA[RC_HEADING_CHANNEL_INDEX] = getRCValue(RC_HEADING_CHANNEL_INDEX);
	// Apply dead bands
	rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX] = applyStickDeadBand(rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX]);
	rcData.RC_DELTA_DATA[RC_PITCH_CHANNEL_INDEX] = applyStickDeadBand(rcData.RC_DELTA_DATA[RC_PITCH_CHANNEL_INDEX]);
	rcData.RC_DELTA_DATA[RC_ROLL_CHANNEL_INDEX] = applyStickDeadBand(rcData.RC_DELTA_DATA[RC_ROLL_CHANNEL_INDEX]);
	rcData.RC_DELTA_DATA[RC_YAW_CHANNEL_INDEX] = applyStickDeadBand(rcData.RC_DELTA_DATA[RC_YAW_CHANNEL_INDEX]);
}

/*************************************************************************/
// Checks if the throttle is stable
/*************************************************************************/
uint8_t isThrottleCentered() {
	if (applyDeadBandInt16(0, rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX], THROTTLE_CENTER_DEADBAND) != 0) {
		return 0;
	} else {
		return 1;
	}
}

/*************************************************************************/
// Checks if the yaw stick is stable
/*************************************************************************/
uint8_t isYawCentered() {
	if (applyDeadBandInt16(0, rcData.RC_DELTA_DATA[RC_YAW_CHANNEL_INDEX], YAW_CENTER_DEADBAND) != 0) {
		return 0;
	} else {
		return 1;
	}
}

/*************************************************************************/
// Checks if the Roll stick is stable
/*************************************************************************/
uint8_t isRollCentered() {
	if (applyDeadBandInt16(0, rcData.RC_DELTA_DATA[RC_ROLL_CHANNEL_INDEX], ROLL_CENTER_DEADBAND) != 0) {
		return 0;
	} else {
		return 1;
	}
}

/*************************************************************************/
// Checks if the Pitch stick is stable
/*************************************************************************/
uint8_t isPitchCentered() {
	if (applyDeadBandInt16(0, rcData.RC_DELTA_DATA[RC_PITCH_CHANNEL_INDEX], PITCH_CENTER_DEADBAND) != 0) {
		return 0;
	} else {
		return 1;
	}
}

/**
 * Checks if Position Hold mode is active
 */
uint8_t isHeadLessModeActive() {
	return (rcData.RC_DELTA_DATA[RC_HEADING_CHANNEL_INDEX] > HEADLESS_MODE_ACT_TSH);
}

uint8_t isTerrainAltModeActive() {
	return (rcData.RC_DELTA_DATA[RC_ALT_MODE_CHANNEL_INDEX] > ALT_MODE_MODE_ACT_TSH);
}
/**
 * Checks if Altitude hold can be enabled
 **/
uint8_t canEnableAltHold() {
	// If throttle stick is stable
	if (rcData.throttleCentered) {
		return 1;
	} else {
		return 0;
	}
}

/**
 * Checks if Position Hold mode is active
 */
uint8_t isGlobalPosHoldModeActive() {
	return (rcData.RC_DELTA_DATA[RC_POS_CHANNEL_INDEX] > POS_HOLD_MODE_ACT_TSH);
}

/**
 * Checks if Position hold can be enabled
 */
uint8_t canEnableGlobalPosHold() {
	//&& rcData.throttleStable
	// if (fcStatusData.isGlobalPosHoldModeActive && rcData.pitchCentered && rcData.rollCentered && rcData.yawCentered) {
	if (fcStatusData.isGlobalPosHoldModeActive && rcData.pitchCentered && rcData.rollCentered) {
		return 1;
	} else {
		return 0;
	}
}

/**
 * Checks if RTH Mode is active
 */
uint8_t isRTHModeActive() {
	return (rcData.RC_DELTA_DATA[RC_POS_CHANNEL_INDEX] > RTH_HOLD_MODE_ACT_TSH);
}

/**
 * Checks if Landing Mode is active
 */
uint8_t canEnableLandingMode() {
	return (rcData.RC_DELTA_DATA[RC_LAND_CHANNEL_INDEX] > LANDING_MODE_ACT_TSH);
}

/**
 * Checks if return to home can be enabled
 */
uint8_t canEnableRTH() {
	//&& rcData.throttleStable
	if (fcStatusData.isRTHModeActive && rcData.pitchCentered && rcData.rollCentered && rcData.yawCentered) {
		return 1;
	} else {
		return 0;
	}
}

/**
 Checks if the Model can be armed
 **/
uint8_t canStartModel() {
	if (getRCValue(RC_START_CHANNEL_INDEX) == RC_CHANNEL_MAX_VALUE) {
		return 1;
	} else {
		return 0;
	}
}

/**
 Checks if the Model can be armed, The throttle at the bottom position
 **/
uint8_t canArmModel() {
	if (applyDeadBandInt16(0, getRCValue(RC_TH_CHANNEL_INDEX) - RC_CHANNEL_MIN_VALUE, THROTTLE_CENTER_ARM_DEADBAND) == 0 && getRCValue(RC_START_CHANNEL_INDEX) == RC_CHANNEL_MAX_VALUE) {
		return 1;
	} else {
		return 0;
	}
}

/**
 Applies deadband to RC channel value;
 **/
int16_t applyStickDeadBand(int16_t rcChannelValue) {
	if (RC_CHANNEL_DEAD_BAND != 0) {
		return applyDeadBandInt16(0, rcChannelValue, RC_CHANNEL_DEAD_BAND);
	} else {
		return rcChannelValue;
	}
}
