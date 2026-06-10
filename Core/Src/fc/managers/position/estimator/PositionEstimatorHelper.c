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


__ATTR_ITCM_TEXT
float getEstimatedZRPTerrain(POSITION_EKF *ekf, float zTerrain, float quality) {
	const int i = POS_EKF_Z_AXIS * POS_EKF_AXIS_DIM;
	// 1. Hard safety guard against zero or negative quality values to avoid a division by zero
	if (quality < 0.01f) {
		quality = 0.01f;
	}
	/* =========================================================================
	 * DEFENSE LAYER 1: Optical Signal Quality Scaling
	 * We scale the base variance inversely with the square of the quality score.
	 * - At 100% quality (1.0): opticalR = 0.01 / 1.0  = 0.01  (Tight lock)
	 * - At 50% quality  (0.5): opticalR = 0.01 / 0.25 = 0.04  (Slightly relaxed)
	 * - At 15% quality  (0.15): opticalR = 0.01 / 0.0225 = 0.44 (Relies heavily on Baro)
	 * ========================================================================= */
	float qualityScale = 1.0f / (quality * quality);
	float opticalR = TERRAIN_BASE_R * qualityScale;

	/* =========================================================================
	 * DEFENSE LAYER 2: Kinematic Residual Scaling (Obstacle Rejection)
	 * We calculate the difference between what the TFmini sees right now vs where
	 * the EKF's accelerometer prediction says the drone should be inertially.
	 * ========================================================================= */
	float zPred = ekf->x[i + POS_EKF_STATE_P];
	float residual = zTerrain - zPred;

	// Grab current Z-axis state uncertainty from the Error Covariance Matrix (P)
	float Pzz = ekf->P[i + 0][i + 0];

	float denom = Pzz + opticalR;
	if (denom < 1e-4f) {
		denom = 1e-4f; // Safety floor
	}

	// Quadratic penalty: Small deviations are ignored; large step changes explode the variance
	float residualTerm = TERRAIN_OBSTACLE_GAIN * (residual * residual) / denom;

	// Combine both layers of defense into a unified variance output
	float finalDynamicR = opticalR + residualTerm;

	// 2. Bound the variance cleanly
	if (finalDynamicR > TERRAIN_MAX_R) {
		finalDynamicR = TERRAIN_MAX_R;
	}

	return finalDynamicR;
}

__ATTR_ITCM_TEXT
void updateXYPosition( float hAcc, float xPos, float yPos, float dt) {
	positionCordinateData.positionXYUpdateDt = dt;
	// Calculate dynamic R for Position
	float dynamicRp = getEstimatedXYRP(hAcc);
	positionEKFUpdateRPosition(&positionEkf, POS_EKF_X_AXIS, dynamicRp);
	positionEKFUpdateRPosition(&positionEkf, POS_EKF_Y_AXIS, dynamicRp);
	positionEKFUpdateXYPosition(&positionEkf, xPos, yPos);
}

__ATTR_ITCM_TEXT
void updateXYVelovity(float sAcc, float velN, float velE,float dt) {
	// Adaptive Velocity Measurement Noise Logic
	float dynamicRv = getEstimatedXYRV(sAcc);
	float velNDb = applyDeadBandFloat(0.0f, velN, POS_ESTIMATOR_DYNAMIC_XY_VEL_DEADBAND);
	float velEDb = applyDeadBandFloat(0.0f, velE, POS_ESTIMATOR_DYNAMIC_XY_VEL_DEADBAND);
	positionEKFUpdateXYVelocity(&positionEkf, velNDb, velEDb, dynamicRv);
}
float dynamicRPSLTest,venturiBiasTest;
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
	dynamicRPSLTest = dynamicRPSL;
	positionEKFUpdateRPosition(&positionEkf, POS_EKF_Z_AXIS, dynamicRPSL);
#endif
	venturiBiasTest = venturiBias;
	positionEKFUpdateZPosition(&positionEkf, zPos, venturiBias);
}

__ATTR_ITCM_TEXT
void updateZPositionTerrain( float zTerrain, float quality, float dt) {
	positionCordinateData.positionZTerrainUpdateDt = dt;
	positionCordinateData.zPositionRawTerrain = zTerrain;
	/*
	float dynamicRPTerrain = getEstimatedZRPTerrain(&positionEkf, zTerrain, quality);
	positionEKFUpdateRPosition(&positionEkf, POS_EKF_Z_AXIS, dynamicRPTerrain);
	positionEKFUpdateZPosition(&positionEkf, zTerrain, 0);
	*/
}
