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
	acc = applyDeadBandFloat(0.0f, ay, POSITION_MGR_Y_ACC_DEADBAND);
	acc = constrainToRangeF(acc, -POSITION_MGR_Y_ACC_MAX, POSITION_MGR_Y_ACC_MAX);
	positionCordinateData.yAcceleration = lowPassFilterUpdate(&positionMgrAccYLPF, acc, dt);

	// Z Axis
	acc = applyDeadBandFloat(0.0f, az, POSITION_MGR_Z_ACC_DEADBAND);
	acc = constrainToRangeF(acc, -POSITION_MGR_Z_ACC_MAX, POSITION_MGR_Z_ACC_MAX);
	positionCordinateData.zAcceleration = lowPassFilterUpdate(&positionMgrAccZLPF, acc, dt);
}

__ATTR_ITCM_TEXT
void updatePositionReference(float dt) {
	if (fcStatusData.canFly && fcStatusData.isPositionDataReliable && fcStatusData.isPositionHomeSet) {
		if (!fcStatusData.isPositionRefSet) {
			fcStatusData.positionXRef = positionCordinateData.xPosition;
			fcStatusData.positionYRef = positionCordinateData.yPosition;
			fcStatusData.isPositionRefSet = 1;
		}
	}
}

__ATTR_ITCM_TEXT
void resetPositionCommands(float dt) {
	positionCommandData.pitchCommand = 0.0f;
	positionCommandData.rollCommand = 0.0f;
}

static float posCtrlDtAccum = 0.0f;
static int positionControlTick = 0;
__ATTR_ITCM_TEXT
void updatePositionCommand(float dt) {
	if (fcStatusData.isPositionHoldModeActive) {
		updatePositionReference(dt);
		if (fcStatusData.isPositionRefSet) {
			posCtrlDtAccum += dt;
			positionControlTick++;
			if (positionControlTick >= 10) {
				controlPositionWithGains(posCtrlDtAccum, fcStatusData.positionXRef, fcStatusData.positionYRef, 1.0f, 1.0f, 1.0f, 1.0f);
				float pitchCommand, rollCommand;

				float positionXControl = controlData.positionXControl ;//- 0.02 * positionCordinateData.yVelocity;
				float positionYControl = controlData.positionYControl ;//- 0.02 * positionCordinateData.xVelocity;

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
				positionControlTick = 0;
				posCtrlDtAccum = 0.0f;
			}
		} else {
			posCtrlDtAccum = 0.0f;
			positionControlTick = 0;
			resetPositionCommands(dt);
		}
	} else {
		posCtrlDtAccum = 0.0f;
		positionControlTick = 0;
		fcStatusData.isPositionRefSet = 0;
		resetPositionCommands(dt);
	}
}

__ATTR_ITCM_TEXT
void managePositionTask(void) {
	float dt = getDeltaTime(POSITION_MANAGER_TASK_TIMER_CHANNEL);

	if (dt <= 0.0f || dt > 0.1f) {
		dt = 0.001f; // Safety guard for dt
	}

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
	//Update the position command
	updatePositionCommand(dt);

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
			float dynamicRpos = (POSITION_MGR_XY_POS_UPDATE_CONFIDENCE + (POSITION_MGR_GNSS_POS_HACC_SCALE * (hAcc * hAcc)));

			if (dynamicRpos > POSITION_MGR_GNSS_POS_R_MAX) {
				dynamicRpos = POSITION_MGR_GNSS_POS_R_MAX;
			}

			positionEKFSetDymamicPosR(&positionEkf, POS_EKF_X_AXIS, dynamicRpos);
			positionEKFSetDymamicPosR(&positionEkf, POS_EKF_Y_AXIS, dynamicRpos);

			updatePositionManagerXYPosition((positionCordinateData.xPositionRaw * POSITION_MGR_X_POS_OUTPUT_GAIN), (positionCordinateData.yPositionRaw * POSITION_MGR_Y_POS_OUTPUT_GAIN), dt);
		} else {
			positionEKFUpdateXYVel(&positionEkf, 0.0f, 0.0f, POSITION_MGR_XY_VEL_RESET_DAMP_STRENGTH);
			positionCommandData.pitchCommand = 0.0f;
			positionCommandData.rollCommand = 0.0f;
			fcStatusData.isPositionRefSet = 0;
			fcStatusData.isPositionHomeSet = 0;
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

