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

static float rcActiveCheckDt = 0;
static float rcHealthCheckDt = 0;
static float rcStablilizationDt = 0;

static float rcStickPitchGain = 0;
static float rcStickRollGain = 0;
static float rcStickYawGain = 0;
static float rcStickThrottleGain = 0;

void resetFCStatus();
void determineFCState(float dt);
void rcTask(void);

/* ------------------------------------------------------------------------ */
/* Init / configuration                                                     */
/* ------------------------------------------------------------------------ */

/** Loads the stick middle values from calibration. */
void loadRCStickMids() {
	rcData.RC_MID_DATA[RC_TH_CHANNEL_INDEX] = getCalibrationValue(CALIB_PROP_RC_THROTTLE_OFFSET_ADDR);
	rcData.RC_MID_DATA[RC_PITCH_CHANNEL_INDEX] = getCalibrationValue(CALIB_PROP_RC_PITCH_OFFSET_ADDR);
	rcData.RC_MID_DATA[RC_ROLL_CHANNEL_INDEX] = getCalibrationValue(CALIB_PROP_RC_ROLL_OFFSET_ADDR);
	rcData.RC_MID_DATA[RC_YAW_CHANNEL_INDEX] = getCalibrationValue(CALIB_PROP_RC_YAW_OFFSET_ADDR);
}

/** Configures the stick rate gains. */
void configureRCStickControl() {
	rcStickThrottleGain = 1.0f;
	rcStickPitchGain = get1KXScaledCalibrationValue(CALIB_PROP_RC_PITCH_RATE_P_ADDR);
	rcStickRollGain = get1KXScaledCalibrationValue(CALIB_PROP_RC_ROLL_RATE_P_ADDR);
	rcStickYawGain = get1KXScaledCalibrationValue(CALIB_PROP_RC_YAW_RATE_P_ADDR);
}

/** Initializes the Radio Controller. */
uint8_t initRCManager() {
	if (!initRCSensor()) {
		logString("[RCManager] : Sensor Init > Failure\n");
		return 0;
	}
	logString("[RCManager] : Sensor Init > Success\n");

	loadRCStickMids();
	configureRCStickControl();
	logString("[RCManager] : Stick Controls Init > Success\n");

	schedulerAddTask(rcTask, RC_TASK_FREQUENCY, RC_TASK_PRIORITY);
	logString("[RCManager] : Task Scheduler Init > Success\n");
	return 1;
}

/* ------------------------------------------------------------------------ */
/* Link / health monitoring                                                 */
/* ------------------------------------------------------------------------ */

void rcCheckActive(float dt) {
	if (isRCTXxActive()) {
		fcStatusData.isTxOn = 1;
		rcActiveCheckDt = 0;
	} else if (rcActiveCheckDt >= RC_ACTIVE_CHECK_THRESHOLD_PERIOD) {
		fcStatusData.isTxOn = 0;
	} else {
		rcActiveCheckDt += dt;
	}
	// Just time keeping
	rcData.failSafeCheckDt = dt;
}

void rcCheckHealth(float dt) {
	if (!fcStatusData.isTxOn) {
		fcStatusData.isRCHealthy = 0;
		rcHealthCheckDt = 0;
		return;
	}
	rcHealthCheckDt += dt;
	if (rcHealthCheckDt >= RC_HEALTH_CHECK_THRESHOLD_PERIOD) {
		fcStatusData.isRCHealthy = 0;
	}
}

void rcTask() {
	float dt = getDeltaTime(RC_TASK_TIMER_CHANNEL);
	rcCheckActive(dt);
	rcCheckHealth(dt);
}

/* ------------------------------------------------------------------------ */
/* Stick processing                                                         */
/* ------------------------------------------------------------------------ */

/** Applies the RC channel dead band. */
int16_t applyStickDeadBand(int16_t rcChannelValue) {
	if (RC_CHANNEL_DEAD_BAND != 0) {
		return applyDeadBandInt16(0, rcChannelValue, RC_CHANNEL_DEAD_BAND);
	}
	return rcChannelValue;
}

/** Loads stick deltas (value minus calibrated mid) and raw aux channels. */
void loadRCStickDelta() {
	// Sticks
	rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX] = getRCValue(RC_TH_CHANNEL_INDEX) - rcData.RC_MID_DATA[RC_TH_CHANNEL_INDEX];
	rcData.RC_DELTA_DATA[RC_PITCH_CHANNEL_INDEX] = getRCValue(RC_PITCH_CHANNEL_INDEX) - rcData.RC_MID_DATA[RC_PITCH_CHANNEL_INDEX];
	rcData.RC_DELTA_DATA[RC_ROLL_CHANNEL_INDEX] = getRCValue(RC_ROLL_CHANNEL_INDEX) - rcData.RC_MID_DATA[RC_ROLL_CHANNEL_INDEX];
	rcData.RC_DELTA_DATA[RC_YAW_CHANNEL_INDEX] = getRCValue(RC_YAW_CHANNEL_INDEX) - rcData.RC_MID_DATA[RC_YAW_CHANNEL_INDEX];

	// Aux channels
	rcData.RC_DELTA_DATA[RC_NAV_CHANNEL_INDEX] = getRCValue(RC_NAV_CHANNEL_INDEX);
	rcData.RC_DELTA_DATA[RC_RTH_CHANNEL_INDEX] = getRCValue(RC_RTH_CHANNEL_INDEX);
	rcData.RC_DELTA_DATA[RC_START_CHANNEL_INDEX] = getRCValue(RC_START_CHANNEL_INDEX);
	rcData.RC_DELTA_DATA[RC_ALT_MODE_CHANNEL_INDEX] = getRCValue(RC_ALT_MODE_CHANNEL_INDEX);
	rcData.RC_DELTA_DATA[RC_TOFF_AND_LAND_CHANNEL_INDEX] = getRCValue(RC_TOFF_AND_LAND_CHANNEL_INDEX);
	rcData.RC_DELTA_DATA[RC_TOFF_AND_LAND_MODE_CHANNEL_INDEX] = getRCValue(RC_TOFF_AND_LAND_MODE_CHANNEL_INDEX);

	// Dead bands (sticks only)
	rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX] = applyStickDeadBand(rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX]);
	rcData.RC_DELTA_DATA[RC_PITCH_CHANNEL_INDEX] = applyStickDeadBand(rcData.RC_DELTA_DATA[RC_PITCH_CHANNEL_INDEX]);
	rcData.RC_DELTA_DATA[RC_ROLL_CHANNEL_INDEX] = applyStickDeadBand(rcData.RC_DELTA_DATA[RC_ROLL_CHANNEL_INDEX]);
	rcData.RC_DELTA_DATA[RC_YAW_CHANNEL_INDEX] = applyStickDeadBand(rcData.RC_DELTA_DATA[RC_YAW_CHANNEL_INDEX]);
}

/** Applies the RC stick effectiveness (gains). */
void applyRCStickEffectiveness() {
	rcData.RC_EFFECTIVE_DATA[RC_TH_CHANNEL_INDEX] = rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX] * rcStickThrottleGain;
	rcData.RC_EFFECTIVE_DATA[RC_PITCH_CHANNEL_INDEX] = rcData.RC_DELTA_DATA[RC_PITCH_CHANNEL_INDEX] * rcStickPitchGain;
	rcData.RC_EFFECTIVE_DATA[RC_ROLL_CHANNEL_INDEX] = rcData.RC_DELTA_DATA[RC_ROLL_CHANNEL_INDEX] * rcStickRollGain;
	rcData.RC_EFFECTIVE_DATA[RC_YAW_CHANNEL_INDEX] = rcData.RC_DELTA_DATA[RC_YAW_CHANNEL_INDEX] * rcStickYawGain;
}

/** Returns 1 if the stick delta is within the given center dead band. */
static inline uint8_t isStickCentered(uint8_t channelIndex, int16_t deadband) {
	return applyDeadBandInt16(0, rcData.RC_DELTA_DATA[channelIndex], deadband) == 0;
}

uint8_t checkThrottleCentered() {
	return isStickCentered(RC_TH_CHANNEL_INDEX, THROTTLE_CENTER_DEADBAND);
}

uint8_t checkYawCentered() {
	return isStickCentered(RC_YAW_CHANNEL_INDEX, YAW_CENTER_DEADBAND);
}

uint8_t checkRollCentered() {
	return isStickCentered(RC_ROLL_CHANNEL_INDEX, ROLL_CENTER_DEADBAND);
}

uint8_t checkPitchCentered() {
	return isStickCentered(RC_PITCH_CHANNEL_INDEX, PITCH_CENTER_DEADBAND);
}

/* ------------------------------------------------------------------------ */
/* Mode switch decoding                                                     */
/* ------------------------------------------------------------------------ */

/** Headless mode (shares the start channel, so HEADLESS_MODE_ACT_TSH must be above START_MODE_ACT_TSH). */
uint8_t checkHeadlessModeActivation() {
	return (rcData.RC_DELTA_DATA[RC_START_CHANNEL_INDEX] > HEADLESS_MODE_ACT_TSH);
}

uint8_t checkTerrainAltModeActivation() {
	return (rcData.RC_DELTA_DATA[RC_ALT_MODE_CHANNEL_INDEX] > TERRAIN_ALT_MODE_ACT_TSH);
}

/** Checks if the Nav (position hold) mode is active. */
uint8_t checkNavModeActivation() {
	return (rcData.RC_DELTA_DATA[RC_NAV_CHANNEL_INDEX] > NAV_MODE_ACT_TSH);
}

/** Checks if the Cruise mode is active (higher position of the nav switch). */
uint8_t checkNavCruiseModeActivation() {
	return (rcData.RC_DELTA_DATA[RC_NAV_CHANNEL_INDEX] > CRUISE_MODE_ACT_TSH);
}

/** Checks if the RTH mode is active. */
uint8_t checkNavRTHModeActivation() {
	return (rcData.RC_DELTA_DATA[RC_RTH_CHANNEL_INDEX] > RTH_MODE_ACT_TSH);
}

/** Checks if the takeoff/landing switch is engaged. */
uint8_t checkTOffLandingActivation() {
	return (rcData.RC_DELTA_DATA[RC_TOFF_AND_LAND_CHANNEL_INDEX] > LANDING_MODE_ACT_TSH);
}

/** Selects takeoff (1) or landing (0) when the takeoff/landing switch is engaged. */
uint8_t checkTOffLandingModeActivation() {
	return (rcData.RC_DELTA_DATA[RC_TOFF_AND_LAND_MODE_CHANNEL_INDEX] > LANDING_MODE_ACT_TSH);
}

/** Checks if the model can be started. */
uint8_t canStartModel() {
	return (rcData.RC_DELTA_DATA[RC_START_CHANNEL_INDEX] > START_MODE_ACT_TSH);
}

/** Checks if the model can be armed: started, with the throttle at the bottom. */
uint8_t canArmModel() {
	uint8_t throttleAtBottom = (applyDeadBandInt16(0, getRCValue(RC_TH_CHANNEL_INDEX) - RC_CHANNEL_MIN_VALUE, THROTTLE_CENTER_ARM_DEADBAND) == 0);
	return (throttleAtBottom && canStartModel());
}

/* ------------------------------------------------------------------------ */
/* Main RC processing / FC state                                            */
/* ------------------------------------------------------------------------ */

/** Updates the takeoff / landing mode flags. */
static void updateTakeOffLandingModes() {
	if (checkTOffLandingActivation()) {
		uint8_t isTakeOff = checkTOffLandingModeActivation();
		fcStatusData.isTakeOffModeActive = isTakeOff;
		fcStatusData.isLandingModeActive = !isTakeOff;
	} else if (!rcData.throttleCentered) {
		// Throttle movement cancels both
		fcStatusData.isLandingModeActive = 0;
		fcStatusData.isTakeOffModeActive = 0;
	}
}

/** Processes the RC data. */
void processRCData(float dt) {
	loadRCStickDelta();

	fcStatusData.canStart = canStartModel();
	fcStatusData.canArm = 0;
	if (fcStatusData.canStart && !fcStatusData.canFly && !fcStatusData.isStabilized && !fcStatusData.canStabilize) {
		fcStatusData.canArm = canArmModel();
	}

	// Capture stick centers
	rcData.pitchCentered = checkPitchCentered();
	rcData.rollCentered = checkRollCentered();
	rcData.yawCentered = checkYawCentered();
	rcData.throttleCentered = checkThrottleCentered();

	// Apply RC stick rates
	applyRCStickEffectiveness();

	// Set the FC mode status
	fcStatusData.isRTHModeActive = checkNavRTHModeActivation();
	fcStatusData.isNavModeActive = checkNavModeActivation();
	fcStatusData.isCruiseModeActive = checkNavCruiseModeActivation();
	fcStatusData.isHeadlessModeActive = checkHeadlessModeActivation();
	fcStatusData.isTerrainAltModeActive = checkTerrainAltModeActivation();

	updateTakeOffLandingModes();
}

void resetFCStatus() {
	fcStatusData.canArm = 0;
	fcStatusData.canFly = 0;
	fcStatusData.canStabilize = 0;
	fcStatusData.isStabilized = 0;
	rcStablilizationDt = 0;
}

void determineFCState(float dt) {
	if (fcStatusData.hasCrashed || !fcStatusData.canStart) {
		resetFCStatus();
		if (!fcStatusData.canStart) {
			fcStatusData.hasCrashed = 0;
		}
	} else if (fcStatusData.isStabilized) {
		// Stabilized: allow flight and stop stabilizing
		fcStatusData.canFly = 1;
		fcStatusData.canStabilize = 0;
		rcStablilizationDt = 0;
	} else if (fcStatusData.canArm || fcStatusData.canStabilize) {
		fcStatusData.canStabilize = 1;
		rcStablilizationDt += dt;
		if (rcStablilizationDt >= FC_STABLILIZATION_PERIOD) {
			fcStatusData.isStabilized = 1;
			fcStatusData.canStabilize = 0;
			rcStablilizationDt = 0;
		}
	}
}

void doRCManagement() {
	if (readRCSensor()) {
		rcHealthCheckDt = 0.0f;
		fcStatusData.isRCHealthy = 1;
		rcData.updateDt = getDeltaTime(RC_READ_TIMER_CHANNEL);
		processRCData(rcData.updateDt);
		determineFCState(rcData.updateDt);
	}
}

void setRCData(int32_t *data, int32_t length) {
	for (int channel = 0; channel < length; channel++) {
		setRCValue(channel, data[channel]);
	}
	processRCData(0.01f);
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
	rcData.RC_DELTA_DATA[RC_NAV_CHANNEL_INDEX] = 0;
	rcData.RC_DELTA_DATA[RC_TOFF_AND_LAND_CHANNEL_INDEX] = 0;
	rcData.RC_DELTA_DATA[RC_ALT_MODE_CHANNEL_INDEX] = 0;
}

/* ------------------------------------------------------------------------ */
/* Accessors                                                                */
/* ------------------------------------------------------------------------ */

float getRCStickPitchGain() {
	return rcStickPitchGain;
}

float getRCStickRollGain() {
	return rcStickRollGain;
}

int16_t getThrottleChannelValue() {
	return rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX];
}

int16_t getPitchChannelValue() {
	return rcData.RC_DELTA_DATA[RC_PITCH_CHANNEL_INDEX];
}

int16_t getRollChannelValue() {
	return rcData.RC_DELTA_DATA[RC_ROLL_CHANNEL_INDEX];
}

int16_t getYawChannelValue() {
	return rcData.RC_DELTA_DATA[RC_YAW_CHANNEL_INDEX];
}
