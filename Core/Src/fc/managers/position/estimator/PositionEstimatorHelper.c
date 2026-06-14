#include "PositionEstimatorHelper.h"
#include <string.h>
#include "../../../memory/Memory.h"
#include "../../../util/MathUtil.h"
#include "../../../imu/IMU.h"
#include "../common/PositionCommon.h"
#include "VenturiBiasEstimator.h"

float getEstimatedXYRP(float hAcc) {
	if (hAcc < POS_ESTIMATOR_DYNAMIC_XY_POS_HACC_MIN) {
		hAcc = POS_ESTIMATOR_DYNAMIC_XY_POS_HACC_MIN;
	}
	float dynamicR = (POS_ESTIMATOR_DYNAMIC_XY_RP_BASE + (POS_ESTIMATOR_DYNAMIC_XY_POS_HACC_SCALE * (hAcc * hAcc)));
	if (dynamicR > POS_ESTIMATOR_DYNAMIC_XY_RP_MAX) {
		dynamicR = POS_ESTIMATOR_DYNAMIC_XY_RP_MAX;
	}
	return dynamicR;
}

float getEstimatedZRPGNSS(float vAcc) {
	if (vAcc < POS_ESTIMATOR_DYNAMIC_Z_POS_VACC_MIN) {
		vAcc = POS_ESTIMATOR_DYNAMIC_Z_POS_VACC_MIN;
	}
	float dynamicRp = POS_ESTIMATOR_DYNAMIC_Z_GNSS_RP_BASE + (POS_ESTIMATOR_DYNAMIC_Z_POS_VACC_SCALE * (vAcc * vAcc));
	if (dynamicRp > POS_ESTIMATOR_DYNAMIC_Z_GNSS_RP_MAX) {
		dynamicRp = POS_ESTIMATOR_DYNAMIC_Z_GNSS_RP_MAX;
	}
	return dynamicRp;
}

__ATTR_ITCM_TEXT
float getEstimatedZRPSL(POSITION_EKF *ekf, float zMeas, float bias, float ax, float ay, float az) {
	float zPred = ekf->x[6];
	float residual = zMeas - (zPred + bias);
	residual = constrainToRangeF(residual, -POS_ESTIMATOR_DYNAMIC_Z_RESIDUAL_CLAMP, POS_ESTIMATOR_DYNAMIC_Z_RESIDUAL_CLAMP);

	float Pzz = ekf->P[6][6];
	if (Pzz < POS_ESTIMATOR_DYNAMIC_Z_RP_EPS) {
		Pzz = POS_ESTIMATOR_DYNAMIC_Z_RP_EPS;
	}
	float denom = Pzz + POS_EKF_Z_R_MEAS + POS_ESTIMATOR_DYNAMIC_Z_RP_SCALE_EPS;
	if (denom < 1e-3f) {
		denom = 1e-3f;
	}
	float scale = 1.0f / denom;
	float residualTerm = POS_ESTIMATOR_DYNAMIC_Z_RP_GAIN * residual * residual * scale;

	float accXY = fastSqrtf(ax * ax + ay * ay);
	float accZ = fabsf(az);

	float accXYScale = constrainToRangeF(accXY / POS_ESTIMATOR_DYNAMIC_Z_ACC_XY_THRESH, 0.0f, 1.0f);
	float accZScale = constrainToRangeF(accZ / POS_ESTIMATOR_DYNAMIC_Z_ACC_Z_THRESH, 0.0f, 1.0f);
	float motionScale = constrainToRangeF(accXYScale + accZScale, 0.0f, 1.0f);

	float baseDynamicR = POS_EKF_Z_R_MEAS + residualTerm;
	float targetDynamicR = baseDynamicR + (motionScale * (POS_ESTIMATOR_DYNAMIC_Z_RP_MAX - baseDynamicR));
	targetDynamicR = constrainToRangeF(targetDynamicR, POS_ESTIMATOR_DYNAMIC_Z_RP_MIN, POS_ESTIMATOR_DYNAMIC_Z_RP_MAX);

	// PATCH: Encapsulate within structure variable to isolate thread execution contexts
	float dynamicR = ekf->prevZR + POS_ESTIMATOR_DYNAMIC_Z_RP_ALPHA * (targetDynamicR - ekf->prevZR);
	ekf->prevZR = dynamicR;

	return dynamicR;
}

float getEstimatedXYRV(float sAcc) {
	if (sAcc < POS_ESTIMATOR_DYNAMIC_XY_VEL_SACC_MIN) {
		sAcc = POS_ESTIMATOR_DYNAMIC_XY_VEL_SACC_MIN;
	}
	float dynamicRv = (POS_ESTIMATOR_DYNAMIC_XY_RV_BASE + (POS_ESTIMATOR_DYNAMIC_XY_VEL_SACC_SCALE * (sAcc * sAcc)));
	if (dynamicRv > POS_ESTIMATOR_DYNAMIC_XY_RV_MAX) {
		dynamicRv = POS_ESTIMATOR_DYNAMIC_XY_RV_MAX;
	}
	return dynamicRv;
}

float getEstimatedZRV(float sAcc) {
	if (sAcc < POS_ESTIMATOR_DYNAMIC_Z_VEL_SACC_MIN) {
		sAcc = POS_ESTIMATOR_DYNAMIC_Z_VEL_SACC_MIN;
	}
	float dynamicRv = POS_ESTIMATOR_DYNAMIC_Z_RV_BASE + (POS_ESTIMATOR_DYNAMIC_Z_VEL_SACC_SCALE * (sAcc * sAcc));
	if (dynamicRv > POS_ESTIMATOR_DYNAMIC_Z_RV_MAX) {
		dynamicRv = POS_ESTIMATOR_DYNAMIC_Z_RV_MAX;
	}
	return dynamicRv;
}

__ATTR_ITCM_TEXT
void updateXYPosition(float hAcc, float xPos, float yPos, float dt) {
	positionCordinateData.positionXYUpdateDt = dt;
	// Calculate dynamic R for Position
	float dynamicRp = getEstimatedXYRP(hAcc);
	positionEKFUpdateRPosition(&positionEkf, POS_EKF_X_AXIS, dynamicRp);
	positionEKFUpdateRPosition(&positionEkf, POS_EKF_Y_AXIS, dynamicRp);
	positionEKFUpdateXYPosition(&positionEkf, xPos, yPos);
}

__ATTR_ITCM_TEXT
void updateZPositionSL(float zPos, float dt) {
	positionCordinateData.positionZSLUpdateDt = dt;
	positionCordinateData.zPositionRawSL = zPos;
#if POSITION_MGR_VENTURI_ESTIMATE_ENABLED == 1
	float venturiBias = updateVenturiBiasEstimate(dt);
#else
	float venturiBias = 0.0f;
#endif
#if POSITION_MGR_Z_ENABLE_DYNAMIC_R  == 1
	float dynamicRPSL = getEstimatedZRPSL(&positionEkf, zPos, venturiBias, imuData.axEarthLinear, imuData.ayEarthLinear, imuData.azEarthLinear);
	positionEKFUpdateRPosition(&positionEkf, POS_EKF_Z_AXIS, dynamicRPSL);
#endif
	positionEKFUpdateZPosition(&positionEkf, zPos, venturiBias);
}

float testGNSSRP = 0;
__ATTR_ITCM_TEXT
void updateZPositionGNSS(float vAcc, float hMSL, float dt) {
	float dynamicRp = getEstimatedZRPGNSS(vAcc);
	testGNSSRP = dynamicRp;
	positionEKFUpdateRPosition(&positionEkf, POS_EKF_Z_AXIS, dynamicRp);
	positionEKFUpdateZPosition(&positionEkf, hMSL, 0.0f);
}

__ATTR_ITCM_TEXT
void updateXYVelocity(float sAcc, float velN, float velE, float dt) {
	// Adaptive Velocity Measurement Noise Logic
	float dynamicRv = getEstimatedXYRV(sAcc);
	float velNDb = applyDeadBandFloat(0.0f, velN, POS_ESTIMATOR_DYNAMIC_XY_VEL_DEADBAND);
	float velEDb = applyDeadBandFloat(0.0f, velE, POS_ESTIMATOR_DYNAMIC_XY_VEL_DEADBAND);
	positionEKFUpdateXYVelocity(&positionEkf, velNDb, velEDb, dynamicRv);
}

float testGNSSRV = 0;
__ATTR_ITCM_TEXT
void updateZVelocity(float sAcc, float velD, float dt) {
	float dynamicRv = getEstimatedZRV(sAcc);
	testGNSSRV = dynamicRv;
	float velDDb = applyDeadBandFloat(0.0f, velD, POS_ESTIMATOR_DYNAMIC_Z_VEL_DEADBAND);
	positionEKFUpdateZVelocity(&positionEkf, velDDb, dynamicRv);
}
