#include "PositionManager.h"

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

		initPositionControl(POSITION_MANAGEMENT_POSITION_CONTROL_FREQUENCY, POSITION_MANAGEMENT_RATE_CONTROL_FREQUENCY);
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
void upadatePositionAcceleration(float ax, float ay, float az, float dt) {
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
		if (!fcStatusData.isPositionRefSet) {
			fcStatusData.positionXRef = positionCordinateData.xPosition;
			fcStatusData.positionYRef = positionCordinateData.yPosition;
			fcStatusData.isPositionRefSet = 1;
		}
	}
}

__ATTR_ITCM_TEXT
void resetPositionCommands() {
	positionCommandData.pitchCommand = 0.0f;
	positionCommandData.rollCommand = 0.0f;
	resetPositionControl(1);
}

__ATTR_ITCM_TEXT
void updatePositionRateCommand(float dt) {
	if (fcStatusData.isPositionHoldModeActive && fcStatusData.isPositionDataReliable) {
		if ((fcStatusData.postionHoldState == POS_HOLD_STATE_BRAKING || fcStatusData.postionHoldState == POS_HOLD_STATE_LOCKED) && fcStatusData.isPositionRefSet) {
			controlPositionRateWithGains(dt, 1.0f, 1.0f, 1.0f);
			float pitchCommand, rollCommand;
			float positionXControl = controlData.positionXControl;
			float positionYControl = controlData.positionYControl;
			convertEarthToBodyCordinates(positionXControl, positionYControl, sensorAttitudeData.heading, &pitchCommand, &rollCommand);
			float magnitudeSq = (pitchCommand * pitchCommand) + (rollCommand * rollCommand);
			float maxLimit = POSITION_MGR_MAX_POS_COMMAND;
			if (magnitudeSq > (maxLimit * maxLimit)) {
				float magnitude = fastSqrtf(magnitudeSq);
				float scale = maxLimit / magnitude;
				pitchCommand *= scale;
				rollCommand *= scale;
			}
			positionCommandData.pitchCommand = constrainToRangeF(pitchCommand, -maxLimit, maxLimit);
			positionCommandData.rollCommand = constrainToRangeF(rollCommand, -maxLimit, maxLimit);
		} else {
			resetPositionCommands();
		}
	} else {
		fcStatusData.isPositionRefSet = 0;
		resetPositionCommands();
	}
}

__ATTR_ITCM_TEXT
void updatePositionCordinateCommand(float dt) {
	uint8_t isRollPitchStickActive = ((!rcData.pitchCentered) || (!rcData.rollCentered));
	if (isRollPitchStickActive) {
		fcStatusData.postionHoldState = POS_HOLD_STATE_IDLE;
		fcStatusData.isPositionRefSet = 0;
		resetPositionCommands();
		return;
	}
	switch (fcStatusData.postionHoldState) {
	case POS_HOLD_STATE_IDLE:
		// Capture hold point exactly at stick release
		fcStatusData.positionXRef = (positionCordinateData.xPositionRaw * POSITION_MGR_X_POS_OUTPUT_GAIN);
		fcStatusData.positionYRef = (positionCordinateData.yPositionRaw * POSITION_MGR_Y_POS_OUTPUT_GAIN);
		fcStatusData.isPositionRefSet = 1;
		fcStatusData.postionHoldState = POS_HOLD_STATE_BRAKING;
		positionMgrPosHoldElapseDtSum = 0.0f;
		break;
	case POS_HOLD_STATE_BRAKING:
		positionMgrPosHoldElapseDtSum += dt;
		// Time-domain braking model: Stronger braking for shorter desired stopping time
		float brakeStrength = (POSITION_MGR_POS_HOLD_BRAKE_GAIN / POSITION_MGR_POS_HOLD_ELAPSE_TIME);
		float vxCmd = (-positionCordinateData.xVelocity * brakeStrength);
		float vyCmd = (-positionCordinateData.yVelocity * brakeStrength);
		vxCmd = constrainToRangeF(vxCmd, -POSITION_MGR_POS_HOLD_MAX_VELOCITY, POSITION_MGR_POS_HOLD_MAX_VELOCITY) ;
		vyCmd = constrainToRangeF(vyCmd, -POSITION_MGR_POS_HOLD_MAX_VELOCITY, POSITION_MGR_POS_HOLD_MAX_VELOCITY) ;
		setExpectedPositionVelocity(dt, vxCmd, vyCmd);
		if (positionMgrPosHoldElapseDtSum >= POSITION_MGR_POS_HOLD_ELAPSE_TIME) {
			fcStatusData.postionHoldState = POS_HOLD_STATE_LOCKED;
			positionMgrPosHoldElapseDtSum = 0.0f;
		}
		break;
	case POS_HOLD_STATE_LOCKED:
		controlPositionCordinatesWithGains(dt, fcStatusData.positionXRef, fcStatusData.positionYRef, 1.0f);
		break;
	default:
		fcStatusData.postionHoldState = POS_HOLD_STATE_IDLE;
		break;
	}
}

__ATTR_ITCM_TEXT
void managePositionTask(void) {
	float dt = getDeltaTime(POSITION_MANAGER_TASK_TIMER_CHANNEL);
	dt = constrainToRangeF(dt, POSITION_MANAGEMENT_TASK_PERIOD * 0.001f, POSITION_MANAGEMENT_TASK_PERIOD * 4.0f);

	float axEarth = applyDeadBandFloat(0, imuData.axEarthLinear, POSITION_MGR_X_ESTIMATION_ACC_DEADBAND) * POSITION_MGR_X_ACC_OUTPUT_GAIN;
	float ayEarth = applyDeadBandFloat(0, imuData.ayEarthLinear, POSITION_MGR_Y_ESTIMATION_ACC_DEADBAND) * POSITION_MGR_Y_ACC_OUTPUT_GAIN;
	float azEarth = applyDeadBandFloat(0, imuData.azEarthLinear, POSITION_MGR_Z_ESTIMATION_ACC_DEADBAND) * POSITION_MGR_Z_ACC_OUTPUT_GAIN;

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

	postionMgrRateDtSum += dt;
	while ( postionMgrRateDtSum >= POSITION_MANAGEMENT_RATE_CONTROL_PERIOD ) {
		updatePositionRateCommand(POSITION_MANAGEMENT_RATE_CONTROL_PERIOD);
		postionMgrRateDtSum -= POSITION_MANAGEMENT_RATE_CONTROL_PERIOD;
	}

	postionMgrPositionDtSum += dt;
	while ( postionMgrPositionDtSum >= POSITION_MANAGEMENT_POSITION_CONTROL_PERIOD ) {
		updatePositionCordinateCommand(POSITION_MANAGEMENT_POSITION_CONTROL_PERIOD);
		postionMgrPositionDtSum -= POSITION_MANAGEMENT_POSITION_CONTROL_PERIOD;
	}

	positionCordinateData.positionProcessDt = dt;
}

__ATTR_ITCM_TEXT
void doPositionManagement() {
	if ((fcStatusData.canStabilize != 0) && (positionManagerWasInStabMode == 0)) {
		positionManagerWasInStabMode = 1;
		positionEKFSetMode(&positionEkf, 1);
	} else if ((positionManagerWasInStabMode != 0) && (fcStatusData.isStabilized != 0)) {
		positionManagerWasInStabMode = 0;
		positionEKFSetMode(&positionEkf, 0);
	}
	if (readGNSSData()) {
		float dt = getDeltaTime(POSITION_MANAGER_GPS_TIMER_CHANNEL);
		gnssData.updateDt = dt;
		updatePositionDataReliability(dt);
		if ((fcStatusData.isPositionDataReliable != 0) && (fcStatusData.canFly != 0)) {
			uint8_t wasHomeJustSet = 0;
			if (fcStatusData.isPositionHomeSet == 0) {
				fcStatusData.positionLongHome = gnssData.longitude;
				fcStatusData.positionLatHome = gnssData.latitude;
				fcStatusData.isPositionHomeSet = 1;
				fcStatusData.isPositionRefSet = 0;
				wasHomeJustSet = 1;
			}
			convertGNSSToSICordinates(gnssData.latitude, gnssData.longitude, fcStatusData.positionLatHome, fcStatusData.positionLongHome, &positionCordinateData.xPositionRaw, &positionCordinateData.yPositionRaw);
			if (wasHomeJustSet != 0) {
				positionEKFResetAxis(&positionEkf, POS_EKF_X_AXIS, positionCordinateData.xPositionRaw);
				positionEKFResetAxis(&positionEkf, POS_EKF_Y_AXIS, positionCordinateData.yPositionRaw);
			}

			// Adaptive Velocity Measurement Noise Logic
			float sAcc = gnssData.sAcc;
			if (sAcc < POSITION_MGR_GNSS_VEL_SACC_MIN) {
				sAcc = POSITION_MGR_GNSS_VEL_SACC_MIN;
			}
			float dynamicRv = (POSITION_MGR_XY_VEL_UPDATE_DAMP_STRENGTH + (POSITION_MGR_GNSS_VEL_SACC_SCALE * (sAcc * sAcc)));
			if (dynamicRv > POSITION_MGR_GNSS_VEL_R_MAX) {
				dynamicRv = POSITION_MGR_GNSS_VEL_R_MAX;
			}

			positionEKFUpdateXYVel(&positionEkf, gnssData.velN, gnssData.velE, dynamicRv);

			float hAcc = gnssData.hAccMts;
			if (hAcc < POSITION_MGR_GNSS_POS_HACC_MIN) {
				hAcc = POSITION_MGR_GNSS_POS_HACC_MIN;
			}

			// Calculate dynamic R for Position
			float dynamicR = (POSITION_MGR_XY_POS_DYNAMIC_R_BASE + (POSITION_MGR_GNSS_POS_HACC_SCALE * (hAcc * hAcc)));

			if (dynamicR > POSITION_MGR_GNSS_POS_R_MAX) {
				dynamicR = POSITION_MGR_GNSS_POS_R_MAX;
			}

			positionEKFSetDymamicPosR(&positionEkf, POS_EKF_X_AXIS, dynamicR);
			positionEKFSetDymamicPosR(&positionEkf, POS_EKF_Y_AXIS, dynamicR);

			updatePositionManagerXYPosition((positionCordinateData.xPositionRaw * POSITION_MGR_X_POS_OUTPUT_GAIN), (positionCordinateData.yPositionRaw * POSITION_MGR_Y_POS_OUTPUT_GAIN), dt);
		} else {
			positionEKFUpdateXYVel(&positionEkf, 0.0f, 0.0f, POSITION_MGR_XY_VEL_RESET_DAMP_STRENGTH);
			positionCommandData.pitchCommand = 0.0f;
			positionCommandData.rollCommand = 0.0f;
			fcStatusData.isPositionRefSet = 0;
			fcStatusData.isPositionHomeSet = 0;
			positionMgrPosHoldElapseDtSum = 0;
			fcStatusData.postionHoldState = POS_HOLD_STATE_IDLE;
		}
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
	positionCommandData.pitchCommand = 0.0f;
	positionCommandData.rollCommand = 0.0f;
	fcStatusData.isPositionRefSet = 0;
	fcStatusData.isPositionHomeSet = 0;
	positionMgrPosHoldElapseDtSum = 0;
	fcStatusData.postionHoldState = POS_HOLD_STATE_IDLE;
}

__ATTR_ITCM_TEXT
void updatePositionManagerZPosition(float zPos, float dt) {
	positionCordinateData.positionZUpdateDt = dt;
	positionCordinateData.zPositionRaw = zPos;

#if POSITION_MGR_VENTURI_ESTIMATE_ENABLED == 0
	float venturiBias = updateVenturiBiasEstimate(dt);
#else
	float venturiBias = 0.0f;
#endif

#if POSITION_MGR_Z_ENABLE_DYNAMIC_R  == 1
	float dynamicR = positionEKFUpdateZR(&positionEkf, zPos, venturiBias, imuData.axEarthLinear, imuData.ayEarthLinear, imuData.azEarthLinear);
	positionEKFSetDymamicPosR(&positionEkf, POS_EKF_Z_AXIS, dynamicR);
#endif

	positionEKFUpdateZMeasureWithBias(&positionEkf, zPos, venturiBias);
}

__ATTR_ITCM_TEXT
void updatePositionManagerXYPosition(float xPos, float yPos, float dt) {
	positionCordinateData.positionXYUpdateDt = dt;
	positionEKFUpdateXYMeasure(&positionEkf, xPos, yPos);
}

