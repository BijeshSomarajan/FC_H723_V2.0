#include "PositionEstimatorHelper.h"
#include <string.h>
#include "../../../memory/Memory.h"
#include "../../../util/MathUtil.h"
#include "../../../imu/IMU.h"
#include "../common/PositionCommon.h"
#include "VenturiBiasEstimator.h"
#include "../../../status/FCStatus.h"

const float H_BARO_WITH_BIAS[4] = { 1.0f, 0.0f, 0.0f, 1.0f };
const float H_BARO[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
const float H_BIAS[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
const float H_TERRAIN[4] = { 1, 0, 0, 0 };
const float H_P_GNSS[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
const float H_V_GNSS[4] = { 0.0f, 1.0f, 0.0f, 0.0f };

float positionEstPrevRPSL = POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_MAX;
static float positionEstCruiseScale;

/*--------------------------------------- Utilities and inner functions ----------------------------------------------------*/

void resetPVEstimation(uint8_t axis, uint8_t keepBias) {
	positionEKFReset(&positionEkf, axis, keepBias);
	if (axis == POS_EKF_Z_AXIS) {
		positionEstPrevRPSL = POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_MAX;
	}
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
float getGroundSpeed(void) {
	float vx = positionCordinateData.xVelocity;
	float vy = positionCordinateData.yVelocity;
	return fastSqrtf((vx * vx) + (vy * vy));
}

__ATTR_ITCM_TEXT
void calculateCruiseScale(float dt) {
#if POS_ESTIMATOR_Z_CRUISE_ADAPT_ENABLED == 1
	float gs = 0.0f;
	/* No GNSS -> ground speed is dead-reckoned garbage; force hover profile ... */
	if (fcStatusData.isNavModeActive) {
		gs = getGroundSpeed();
	}
	float target = constrainToRangeF((gs - POS_ESTIMATOR_Z_CRUISE_SPEED_LO) / (POS_ESTIMATOR_Z_CRUISE_SPEED_HI - POS_ESTIMATOR_Z_CRUISE_SPEED_LO), 0.0f, 1.0f);
	float tau = (target > positionEstCruiseScale) ? POS_ESTIMATOR_Z_CRUISE_TAU_RISE : POS_ESTIMATOR_Z_CRUISE_TAU_FALL;
	positionEstCruiseScale += (dt / (tau + dt)) * (target - positionEstCruiseScale);
#else
	(void)dt;
	positionEstCruiseScale = 0.0f;
#endif
}

__ATTR_ITCM_TEXT
float getCruiseScale(void) {
	return positionEstCruiseScale;
}

/*--------------------------------------- Dynamic R Estimators ----------------------------------------------------*/
__ATTR_ITCM_TEXT
float getEstimatedXYRP(float hAcc) {
	// 1. Enforce a hardware floor to prevent under-specifying noise
	if (hAcc < POS_ESTIMATOR_DYNAMIC_XY_GNSS_HACC_MIN) {
		hAcc = POS_ESTIMATOR_DYNAMIC_XY_GNSS_HACC_MIN;
	}
	// 2. Pure variance calculation with scaling factor
	float dynamicR = POS_ESTIMATOR_DYNAMIC_XY_GNSS_HACC_SCALE * (hAcc * hAcc);
	// 3. Prevent the EKF from completely ignoring a heavily glitched GPS
	//    by enforcing your decoupled, opened-up maximum variance ceiling
	if (dynamicR > POS_ESTIMATOR_DYNAMIC_XY_GNSS_RP_MAX) {
		dynamicR = POS_ESTIMATOR_DYNAMIC_XY_GNSS_RP_MAX;
	}
	return dynamicR;
}

__ATTR_ITCM_TEXT
float getEstimatedZRPGNSS(float vAcc) {
	if (vAcc < POS_ESTIMATOR_DYNAMIC_Z_GNSS_VACC_MIN) {
		vAcc = POS_ESTIMATOR_DYNAMIC_Z_GNSS_VACC_MIN;
	}
	float dynamicRp = POS_ESTIMATOR_DYNAMIC_Z_GNSS_RP_BASE + (POS_ESTIMATOR_DYNAMIC_Z_GNSS_VACC_SCALE * (vAcc * vAcc));
	if (dynamicRp > POS_ESTIMATOR_DYNAMIC_Z_GNSS_RP_MAX) {
		dynamicRp = POS_ESTIMATOR_DYNAMIC_Z_GNSS_RP_MAX;
	}
	return dynamicRp;
}

__ATTR_ITCM_TEXT
float getEstimatedZRPSL(POSITION_EKF *ekf, float zMeas, float motionScale) {
	const int i = POS_EKF_Z_AXIS * POS_EKF_AXIS_DIM;

	/* ==============================================================
	 * Predicted Baro Measurement Residual
	 * ============================================================== */
	float zPred = ekf->x[i + POS_EKF_STATE_P];
	float zBias = ekf->x[i + POS_EKF_STATE_BP];
	float residual = constrainToRangeF(zMeas - (zPred + zBias), -POS_ESTIMATOR_DYNAMIC_Z_BARO_RESIDUAL_CLAMP, POS_ESTIMATOR_DYNAMIC_Z_BARO_RESIDUAL_CLAMP);

	/* ==============================================================
	 * Innovation scaling using position covariance
	 * ============================================================== */
	float Pzz = ekf->P[i + POS_EKF_STATE_P][i + POS_EKF_STATE_P];
	if (Pzz < POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_EPS) {
		Pzz = POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_EPS;
	}

	float denom = Pzz + POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_MIN + POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_SCALE_EPS;
	if (denom < 1e-3f) {
		denom = 1e-3f;
	}

	float scale = 1.0f / denom;
	float residualTerm = POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_GAIN * residual * residual * scale;

	/* ==============================================================
	 * Dynamic R estimation (using injected motionScale)
	 * ============================================================== */
	float baseDynamicR = POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_MIN + residualTerm;
	float targetDynamicR = baseDynamicR + (motionScale * (POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_MAX - baseDynamicR));
	targetDynamicR = constrainToRangeF(targetDynamicR, POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_MIN, POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_MAX);

	/* ==============================================================
	 * LPF smoothing
	 * ============================================================== */
	float dynamicR = positionEstPrevRPSL + POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_ALPHA * (targetDynamicR - positionEstPrevRPSL);
	positionEstPrevRPSL = dynamicR;

	return dynamicR;
}

float testR_venturi;
__ATTR_ITCM_TEXT
float getEstimatedVenturiRP(float motionScale) {
// motionScale 0.0 (Smooth) -> R = BASE (0.1f)  => High Trust
// motionScale 1.0 (Rough)  -> R = MAX (1.0f)   => Low Trust
	float R_venturi = POS_ESTIMATOR_DYNAMIC_Z_VENTURI_RP_BASE + (motionScale * (POS_ESTIMATOR_DYNAMIC_Z_VENTURI_RP_MAX - POS_ESTIMATOR_DYNAMIC_Z_VENTURI_RP_BASE));
	testR_venturi = R_venturi;
	// Ensure we are strictly bounded within our defined tuning limits
	return constrainToRangeF(R_venturi, POS_ESTIMATOR_DYNAMIC_Z_VENTURI_RP_BASE, POS_ESTIMATOR_DYNAMIC_Z_VENTURI_RP_MAX);
}

__ATTR_ITCM_TEXT
float getEstimatedXYRV(float sAcc) {
	// 1. Enforce a sensible sensor accuracy floor (meters per second)
	if (sAcc < POS_ESTIMATOR_DYNAMIC_XY_GNSS_SACC_MIN) {
		sAcc = POS_ESTIMATOR_DYNAMIC_XY_GNSS_SACC_MIN;
	}
	// 2. Pure variance calculation: (m/s)^2
	float dynamicRv = POS_ESTIMATOR_DYNAMIC_XY_GNSS_SACC_SCALE * (sAcc * sAcc);
	// 3. Cap the variance so an absolute GPS blackout doesn't cause
	//    numerical instability (NaN) in the Kalman Gain calculation
	if (dynamicRv > POS_ESTIMATOR_DYNAMIC_XY_GNSS_RV_MAX) {
		dynamicRv = POS_ESTIMATOR_DYNAMIC_XY_GNSS_RV_MAX;
	}
	return dynamicRv;
}

__ATTR_ITCM_TEXT
float getEstimatedTerrainRP(float distance, float quality, float minDistance, float maxDistance, uint8_t terrainDataValid) {
	if (!terrainDataValid) {
		return POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_MUTED;
	}
	float distScale = (distance - minDistance) / (maxDistance - minDistance);
	distScale = constrainToRangeF(distScale, 0.0f, 1.0f);
	float qualityScale = 1.0f - quality;
	/*---------------------------------------------------------
	 * Conservative: trust the worse of the two
	 *---------------------------------------------------------*/
	float scale = fmaxf(distScale, qualityScale);
	float R = POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_BASE + scale * (POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_MAX - POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_BASE);
	return constrainToRangeF(R, POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_BASE, POS_ESTIMATOR_DYNAMIC_Z_TERRAIN_RP_MAX);
}

__ATTR_ITCM_TEXT
float getEstimatedZRV(float sAcc, float cruiseScale) {
	if (sAcc < POS_ESTIMATOR_DYNAMIC_Z_GNSS_SACC_MIN) {
		sAcc = POS_ESTIMATOR_DYNAMIC_Z_GNSS_SACC_MIN;
	}

#if POS_ESTIMATOR_Z_CRUISE_ADAPT_ENABLED == 1
	float rvBase    = POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_BASE - cruiseScale * (POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_BASE - POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_BASE_CRUISE);
	float dynamicRv = rvBase + (POS_ESTIMATOR_DYNAMIC_Z_GNSS_SACC_SCALE * (sAcc * sAcc));
#else
	float dynamicRv = POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_BASE + (POS_ESTIMATOR_DYNAMIC_Z_GNSS_SACC_SCALE * (sAcc * sAcc));
#endif

	if (dynamicRv > POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_MAX) {
		dynamicRv = POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_MAX;
	}
	return dynamicRv;
}

/*--------------------------------------- Measurement Update Functions ----------------------------------------------------*/
__ATTR_ITCM_TEXT
void updateXYPositionGNSS(float hAcc, float xPos, float yPos, float dt) {
	positionCordinateData.positionXYUpdateDt = dt;
	float dynamicRp = getEstimatedXYRP(hAcc);
#if POS_ESTIMATOR_GNSS_DELAY_ENABLED == 1
	float predictionX, predictionY;
	if (positionEKFGetLaggedPrediction(&positionEkf, POS_EKF_X_AXIS, H_P_GNSS, POS_ESTIMATOR_GNSS_LATENCY_S, &predictionX)) {
		positionEKFMeasurementUpdateLagged(&positionEkf, POS_EKF_X_AXIS, xPos, dynamicRp, H_P_GNSS, predictionX);
	} else {
		positionEKFMeasurementUpdate(&positionEkf, POS_EKF_X_AXIS, xPos, dynamicRp, H_P_GNSS);
	}
	if (positionEKFGetLaggedPrediction(&positionEkf, POS_EKF_Y_AXIS, H_P_GNSS, POS_ESTIMATOR_GNSS_LATENCY_S, &predictionY)) {
		positionEKFMeasurementUpdateLagged(&positionEkf, POS_EKF_Y_AXIS, yPos, dynamicRp, H_P_GNSS, predictionY);
	} else {
		positionEKFMeasurementUpdate(&positionEkf, POS_EKF_Y_AXIS, yPos, dynamicRp, H_P_GNSS);
	}
#else
	positionEKFMeasurementUpdate(&positionEkf, POS_EKF_X_AXIS, xPos, dynamicRp, H_P_GNSS);
	positionEKFMeasurementUpdate(&positionEkf, POS_EKF_Y_AXIS, yPos, dynamicRp, H_P_GNSS);
#endif
}
float testdynamicRPSL = 0;
__ATTR_ITCM_TEXT
void updateZPositionSL(float offset, float zPos, float dt) {
	positionCordinateData.positionZSLUpdateDt = dt;
	positionCordinateData.zPositionRawSL = zPos;
	float motionScale = calculateMotionScale(imuData.axEarthLinear, imuData.ayEarthLinear, imuData.azEarthLinear);

#if POS_ESTIMATOR_Z_CRUISE_ADAPT_ENABLED == 1
	// ---------------- Update the cruise scale ----------------
	calculateCruiseScale(dt);
	motionScale = fmaxf(motionScale, getCruiseScale());   // max, not sum — don't double-count a braking cruise
#endif

// ---------------- BARO ----------------
	float dynamicRPSL = POS_ESTIMATOR_DYNAMIC_Z_BARO_RP_MIN;
#if POSITION_MGR_Z_ENABLE_DYNAMIC_R == 1
	dynamicRPSL = getEstimatedZRPSL(&positionEkf, zPos, motionScale);
	testdynamicRPSL = dynamicRPSL;
#endif

#if POSITION_MGR_VENTURI_ESTIMATE_ENABLED == 1
	float venturiBias = getVenturiBiasEstimate(dt);
	positionEKFMeasurementUpdate(&positionEkf, POS_EKF_Z_AXIS, offset + zPos + venturiBias, dynamicRPSL, H_BARO_WITH_BIAS);
#else
	positionEKFMeasurementUpdate(&positionEkf, POS_EKF_Z_AXIS, offset + zPos, dynamicRPSL, H_BARO_WITH_BIAS);
#endif

// ---------------- VENTURI ----------------
/*
#if POSITION_MGR_VENTURI_ESTIMATE_ENABLED == 1
	float venturiBias = getVenturiBiasEstimate(dt);
	float venturiR = getEstimatedVenturiRP(motionScale);
	positionEKFMeasurementUpdate(&positionEkf, POS_EKF_Z_AXIS, -venturiBias, venturiR, H_BIAS);
#endif
*/

}

__ATTR_ITCM_TEXT
void updateZPositionTerrain(float offset, float distance, float strength, float minDistance, float maxDistance, uint8_t terrainDataValid, float dt) {
	positionCordinateData.positionZTerrainUpdateDt = dt;
	positionCordinateData.zPositionRawTerrain = distance;
	float terrainR = getEstimatedTerrainRP(distance, strength, minDistance, maxDistance, terrainDataValid);
	positionEKFMeasurementUpdate(&positionEkf, POS_EKF_Z_AXIS, offset + distance, terrainR, H_TERRAIN);
}

__ATTR_ITCM_TEXT
void updateZPositionGNSS(float vAcc, float hMSL, uint8_t navigationModeActive, float dt) {
	float dynamicRp = POS_ESTIMATOR_DYNAMIC_Z_GNSS_RP_MUTED;
	if (navigationModeActive) {
		dynamicRp = getEstimatedZRPGNSS(vAcc);
	}
	positionEKFMeasurementUpdate(&positionEkf, POS_EKF_Z_AXIS, hMSL, dynamicRp, H_P_GNSS);
}

__ATTR_ITCM_TEXT
void updateXYVelocityGNSS(float sAcc, float velN, float velE, float dt) {
	float dynamicRv = getEstimatedXYRV(sAcc);
	float velNDb = applyDeadBandFloat(0.0f, velN, POS_ESTIMATOR_DYNAMIC_XY_GNSS_VEL_DEADBAND);
	float velEDb = applyDeadBandFloat(0.0f, velE, POS_ESTIMATOR_DYNAMIC_XY_GNSS_VEL_DEADBAND);
#if POS_ESTIMATOR_GNSS_DELAY_ENABLED == 1
	float predictionX, predictionY;
	if (positionEKFGetLaggedPrediction(&positionEkf, POS_EKF_X_AXIS, H_V_GNSS, POS_ESTIMATOR_GNSS_LATENCY_S, &predictionX)) {
		positionEKFMeasurementUpdateLagged(&positionEkf, POS_EKF_X_AXIS, velNDb, dynamicRv, H_V_GNSS, predictionX);
	} else {
		positionEKFMeasurementUpdate(&positionEkf, POS_EKF_X_AXIS, velNDb, dynamicRv, H_V_GNSS);
	}
	if (positionEKFGetLaggedPrediction(&positionEkf, POS_EKF_Y_AXIS, H_V_GNSS, POS_ESTIMATOR_GNSS_LATENCY_S, &predictionY)) {
		positionEKFMeasurementUpdateLagged(&positionEkf, POS_EKF_Y_AXIS, velEDb, dynamicRv, H_V_GNSS, predictionY);
	} else {
		positionEKFMeasurementUpdate(&positionEkf, POS_EKF_Y_AXIS, velEDb, dynamicRv, H_V_GNSS);
	}
#else
	positionEKFMeasurementUpdate(&positionEkf, POS_EKF_X_AXIS, velNDb, dynamicRv, H_V_GNSS);
	positionEKFMeasurementUpdate(&positionEkf, POS_EKF_Y_AXIS, velEDb, dynamicRv, H_V_GNSS);
#endif
}

__ATTR_ITCM_TEXT
void convertBodyToEarthCordinates(float xBody, float yBody, float heading, float *xEarth, float *yEarth) {
	float headingRad = convertDegToRadF(heading);
	float headingCosValue = cosApproxF(headingRad);
	float headingSinValue = sinApproxF(headingRad);
	// The transpose rotation matrix operation
	*xEarth = (xBody * headingCosValue) - (yBody * headingSinValue);
	*yEarth = (xBody * headingSinValue) + (yBody * headingCosValue);
}

__ATTR_ITCM_TEXT
void updateZVelocityGNSS(float sAcc, float velZ, uint8_t navigationModeActive, float dt) {
	float dynamicRv = POS_ESTIMATOR_DYNAMIC_Z_GNSS_RV_MUTED;
	if (navigationModeActive) {
		//dynamicRv = getEstimatedZRV(sAcc);
		dynamicRv = getEstimatedZRV(sAcc, getCruiseScale());
	}
	float velDb = applyDeadBandFloat(0.0f, velZ, POS_ESTIMATOR_DYNAMIC_Z_GNSS_VEL_DEADBAND);
	positionEKFMeasurementUpdate(&positionEkf, POS_EKF_Z_AXIS, velDb, dynamicRv, H_V_GNSS);
}

