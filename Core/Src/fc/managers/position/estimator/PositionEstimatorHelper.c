#include "PositionEstimatorHelper.h"
#include <string.h>
#include "../../../memory/Memory.h"
#include "../../../util/MathUtil.h"
#include "../../../imu/IMU.h"
#include "../common/PositionCommon.h"
#include "VenturiBiasEstimator.h"

const float H_BARO_WITH_BIAS[4] = { 1.0f, 0.0f, 0.0f, 1.0f };
const float H_BARO[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
const float H_BIAS[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
const float H_TERRAIN[4] = { 1, 0, 0, 0 };
const float H_GNSS[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
const float H_VEL[4] = { 0.0f, 1.0f, 0.0f, 0.0f };
const float H_POS[4] = { 1.0f, 0.0f, 0.0f, 0.0f };

/*--------------------------------------- Dynamic R Estimators ----------------------------------------------------*/
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
float testGnssRp;
float getEstimatedZRPGNSS(float vAcc) {
	if (vAcc < POS_ESTIMATOR_DYNAMIC_Z_POS_VACC_MIN) {
		vAcc = POS_ESTIMATOR_DYNAMIC_Z_POS_VACC_MIN;
	}
	float dynamicRp = POS_ESTIMATOR_DYNAMIC_Z_GNSS_RP_BASE + (POS_ESTIMATOR_DYNAMIC_Z_POS_VACC_SCALE * (vAcc * vAcc));
	if (dynamicRp > POS_ESTIMATOR_DYNAMIC_Z_GNSS_RP_MAX) {
		dynamicRp = POS_ESTIMATOR_DYNAMIC_Z_GNSS_RP_MAX;
	}
	testGnssRp = dynamicRp;
	return dynamicRp;
}

__ATTR_ITCM_TEXT
float calculateMotionScale(float ax, float ay, float az) {
	float accXY = fastSqrtf(ax * ax + ay * ay);
	float accZ = fabsf(az);

	float accXYScale = constrainToRangeF(accXY / POS_ESTIMATOR_DYNAMIC_Z_ACC_XY_THRESH, 0.0f, 1.0f);
	float accZScale = constrainToRangeF(accZ / POS_ESTIMATOR_DYNAMIC_Z_ACC_Z_THRESH, 0.0f, 1.0f);

	// Returns a consolidated 0.0 - 1.0 factor representing "Vibration/Acceleration Stress"
	return constrainToRangeF(accXYScale + accZScale, 0.0f, 1.0f);
}

__ATTR_ITCM_TEXT
float getEstimatedZRPSL(POSITION_EKF *ekf, float zMeas, float motionScale) {
	const int i = POS_EKF_Z_AXIS * POS_EKF_AXIS_DIM;

	/* ==============================================================
	 * Predicted Baro Measurement Residual
	 * ============================================================== */
	float zPred = ekf->x[i + POS_EKF_STATE_P];
	float zBias = ekf->x[i + POS_EKF_STATE_BP];
	float residual = constrainToRangeF(zMeas - (zPred + zBias), -POS_ESTIMATOR_DYNAMIC_Z_RESIDUAL_CLAMP, POS_ESTIMATOR_DYNAMIC_Z_RESIDUAL_CLAMP);

	/* ==============================================================
	 * Innovation scaling using position covariance
	 * ============================================================== */
	float Pzz = ekf->P[i + POS_EKF_STATE_P][i + POS_EKF_STATE_P];
	if (Pzz < POS_ESTIMATOR_DYNAMIC_Z_RP_EPS) {
		Pzz = POS_ESTIMATOR_DYNAMIC_Z_RP_EPS;
	}

	float denom = Pzz + POS_EKF_Z_R_MEAS + POS_ESTIMATOR_DYNAMIC_Z_RP_SCALE_EPS;
	if (denom < 1e-3f) {
		denom = 1e-3f;
	}

	float scale = 1.0f / denom;
	float residualTerm = POS_ESTIMATOR_DYNAMIC_Z_RP_GAIN * residual * residual * scale;

	/* ==============================================================
	 * Dynamic R estimation (using injected motionScale)
	 * ============================================================== */
	float baseDynamicR = POS_EKF_Z_R_MEAS + residualTerm;
	float targetDynamicR = baseDynamicR + (motionScale * (POS_ESTIMATOR_DYNAMIC_Z_RP_MAX - baseDynamicR));
	targetDynamicR = constrainToRangeF(targetDynamicR, POS_ESTIMATOR_DYNAMIC_Z_RP_MIN, POS_ESTIMATOR_DYNAMIC_Z_RP_MAX);

	/* ==============================================================
	 * LPF smoothing
	 * ============================================================== */
	float dynamicR = ekf->prevZR + POS_ESTIMATOR_DYNAMIC_Z_RP_ALPHA * (targetDynamicR - ekf->prevZR);
	ekf->prevZR = dynamicR;

	return dynamicR;
}

__ATTR_ITCM_TEXT
float getEstimatedTerrainRP(float distance, float quality, uint8_t terrainModeActive) {
	/*---------------------------------------------------------
	 * Reject invalid measurements
	 *---------------------------------------------------------*/
	if (!terrainModeActive || quality < POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_STRENGTH_MIN || distance < POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_DIST_MIN) {
		return POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_MUTED;
	}
	/*---------------------------------------------------------
	 * Distance scaling
	 *
	 * DIST_MIN -> 0.0
	 * DIST_MAX -> 1.0
	 *---------------------------------------------------------*/
	float distScale = (distance - POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_DIST_MIN) / (POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_DIST_MAX - POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_DIST_MIN);
	distScale = constrainToRangeF(distScale, 0.0f, 1.0f);
	/*---------------------------------------------------------
	 * Quality scaling
	 *
	 * quality = 1.0 -> 0.0
	 * quality = 0.0 -> 1.0
	 *---------------------------------------------------------*/
	float qualityScale = 1.0f - quality;
	/*---------------------------------------------------------
	 * Conservative: trust the worse of the two
	 *---------------------------------------------------------*/
	float scale = fmaxf(distScale, qualityScale);
	float R = POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_BASE + scale * (POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_MAX - POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_BASE);
	return constrainToRangeF(R, POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_BASE, POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_MAX);
}

__ATTR_ITCM_TEXT
float getEstimatedVenturiRP(float motionScale) {
// motionScale 0.0 (Smooth) -> R = BASE (0.1f)  => High Trust
// motionScale 1.0 (Rough)  -> R = MAX (1.0f)   => Low Trust
	float R_venturi = POS_ESTIMATOR_DYNAMIC_Z_VENTURI_RP_BASE + (motionScale * (POS_ESTIMATOR_DYNAMIC_Z_VENTURI_RP_MAX - POS_ESTIMATOR_DYNAMIC_Z_VENTURI_RP_BASE));
// Ensure we are strictly bounded within our defined tuning limits
	return constrainToRangeF(R_venturi, POS_ESTIMATOR_DYNAMIC_Z_VENTURI_RP_BASE, POS_ESTIMATOR_DYNAMIC_Z_VENTURI_RP_MAX);
}

__ATTR_ITCM_TEXT
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

/*--------------------------------------- Measurement Update Functions ----------------------------------------------------*/
__ATTR_ITCM_TEXT
void updateXYPosition(float hAcc, float xPos, float yPos, float dt) {
	positionCordinateData.positionXYUpdateDt = dt;
	float dynamicRp = getEstimatedXYRP(hAcc);
	positionEKFMeasurementUpdate(&positionEkf, POS_EKF_X_AXIS, xPos, dynamicRp, H_POS);
	positionEKFMeasurementUpdate(&positionEkf, POS_EKF_Y_AXIS, yPos, dynamicRp, H_POS);
}

float testMotionScale = 0;
float testBaroR = 0;
float testVenturiR = 0;
float testVenturiBias = 0;

__ATTR_ITCM_TEXT
void updateZPositionSL(float offset, float zPos, float dt) {
	positionCordinateData.positionZSLUpdateDt = dt;
	positionCordinateData.zPositionRawSL = zPos;
	float motionScale = calculateMotionScale(imuData.axEarthLinear, imuData.ayEarthLinear, imuData.azEarthLinear);
	testMotionScale = motionScale;
	/* ---------------- BARO ---------------- */
	float dynamicRPSL = POS_EKF_Z_R_MEAS;
#if POSITION_MGR_Z_ENABLE_DYNAMIC_R == 1
	dynamicRPSL = getEstimatedZRPSL(&positionEkf, zPos, motionScale);
	testBaroR = dynamicRPSL;
#endif
	positionEKFMeasurementUpdate(&positionEkf, POS_EKF_Z_AXIS, offset + zPos, dynamicRPSL, H_BARO_WITH_BIAS);
	/* ---------------- VENTURI ---------------- */
#if POSITION_MGR_VENTURI_ESTIMATE_ENABLED == 1
	float venturiBias = getVenturiBiasEstimate(dt);
	testVenturiBias = venturiBias;
	float venturiR = getEstimatedVenturiRP(motionScale);
	positionEKFMeasurementUpdate(&positionEkf, POS_EKF_Z_AXIS, venturiBias, venturiR, H_BIAS);
	testVenturiR = venturiR;
#endif
}

float testTerrainR = 0;
__ATTR_ITCM_TEXT
void updateZPositionTerrain(float offset, float distance, float strength, uint8_t terrainModeActive, float dt) {
	positionCordinateData.positionZTerrainUpdateDt = dt;
	positionCordinateData.zPositionRawTerrain = distance;
	float terrainR = getEstimatedTerrainRP(distance, strength, terrainModeActive);
	positionEKFMeasurementUpdate(&positionEkf, POS_EKF_Z_AXIS, offset + distance, terrainR, H_TERRAIN);
	testTerrainR = terrainR;
}

float testGNSSRP = 0;
__ATTR_ITCM_TEXT
void updateZPositionGNSS(float vAcc, float hMSL, uint8_t navigationModeActive, float dt) {
	float dynamicRp = POS_ESTIMATOR_DYNAMIC_Z_GNSS_RP_MUTED;
	if (navigationModeActive) {
		dynamicRp = getEstimatedZRPGNSS(vAcc);
	}
	testGNSSRP = dynamicRp;
	positionEKFMeasurementUpdate(&positionEkf, POS_EKF_Z_AXIS, hMSL, dynamicRp, H_GNSS);
}

__ATTR_ITCM_TEXT
void updateXYVelocity(float sAcc, float velN, float velE, float dt) {
	float dynamicRv = getEstimatedXYRV(sAcc);
	float velNDb = applyDeadBandFloat(0.0f, velN, POS_ESTIMATOR_DYNAMIC_XY_VEL_DEADBAND);
	float velEDb = applyDeadBandFloat(0.0f, velE, POS_ESTIMATOR_DYNAMIC_XY_VEL_DEADBAND);
	positionEKFMeasurementUpdate(&positionEkf, POS_EKF_X_AXIS, velNDb, dynamicRv, H_VEL);
	positionEKFMeasurementUpdate(&positionEkf, POS_EKF_Y_AXIS, velEDb, dynamicRv, H_VEL);
}

float testGNSSRV = 0;
__ATTR_ITCM_TEXT
void updateZVelocity(float sAcc, float velZ, uint8_t navigationModeActive, float dt) {
	float dynamicRv = POS_ESTIMATOR_DYNAMIC_Z_RV_MUTED;
	if (navigationModeActive) {
		dynamicRv = getEstimatedZRV(sAcc);
	}
	testGNSSRV = dynamicRv;
	float velDb = applyDeadBandFloat(0.0f, velZ, POS_ESTIMATOR_DYNAMIC_Z_VEL_DEADBAND);
	positionEKFMeasurementUpdate(&positionEkf, POS_EKF_Z_AXIS, velDb, dynamicRv, H_VEL);
}
