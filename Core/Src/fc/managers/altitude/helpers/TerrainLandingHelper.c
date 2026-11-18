#include "TerrainLandingHelper.h"

#include <math.h>

#include "../../../FCConfig.h"
#include "../../../memory/Memory.h"
#include "../../../sensors/altitude/AltitudeSensor.h"
#include "../../../status/FCStatus.h"
#include "../../../util/MathUtil.h"
#include "../../position/common/PositionCommon.h"

typedef enum {
	TL_IDLE = 0, TL_ACTIVE, TL_TOUCHDOWN, TL_ABANDONED
} TerrainLandingState;

static TerrainLandingState tlState = TL_IDLE;
static float tlLastAlt = 0.0f;           // last valid height at the gear
static float tlTouchdownTimer = 0.0f;
static float tlInvalidTimer = 0.0f;
static float tlPhaseTimer = 0.0f;

void resetTerrainLandingStates(void) {
	tlState = TL_IDLE;
	tlLastAlt = 0.0f;
	tlTouchdownTimer = 0.0f;
	tlInvalidTimer = 0.0f;
	tlPhaseTimer = 0.0f;
}

uint8_t isTerrainLandingTouchdown(void) {
	return tlState == TL_TOUCHDOWN;
}

static uint8_t getTerrainAltitude(float *altitude) {
	if (fcStatusData.isTerrainSensorExist != 1 || !fcStatusData.isTerrainAltDataReliable) {
		return 0;
	}
	float h = sensorAltitudeData.altitudeTerrainFiltered;
	if (h < SENSOR_LIDAR_ALT_MIN || h > SENSOR_LIDAR_ALT_MAX) {
		return 0;
	}
	*altitude = fmaxf(h - ALT_COMMAND_TERRAIN_MOUNT_HEIGHT, 0.0f);
	return 1;
}

__ATTR_ITCM_TEXT
uint8_t handleTerrainAssistedLanding(float dt) {
	float liftOffThrottle = fcStatusData.liftOffThrottlePercent * MAX_PERMISSIBLE_THROTTLE_DELTA;
	if (tlState >= TL_TOUCHDOWN || fcStatusData.currentThrottle < liftOffThrottle || fcStatusData.altitudeHoldState != ALT_HOLD_STATE_LOCKED) {
		return 0;   // done, fallen back to baro, or the throttle ramp-down is in charge
	}

	float alt = 0.0f;
	uint8_t valid = getTerrainAltitude(&alt);
	float z = positionCordinateData.zPosition;

	if (tlState == TL_IDLE) {
		if (!valid || alt > ALT_COMMAND_TERRAIN_ENGAGE_HEIGHT) {
			return 0;   // no sensor / too high: baro stepping
		}
		// Bumpless handover: keep the ref close to the aircraft
		fcStatusData.altitudeRef = constrainToRangeF(fcStatusData.altitudeRef, z - ALT_COMMAND_TERRAIN_REF_MAX_LEAD, z + ALT_COMMAND_TERRAIN_REF_MAX_LEAD);
		tlState = TL_ACTIVE;
		tlLastAlt = alt;
		tlTouchdownTimer = 0.0f;
		tlInvalidTimer = 0.0f;
		tlPhaseTimer = 0.0f;
	}

	tlPhaseTimer += dt;
	if (valid) {
		tlLastAlt = alt;
		tlInvalidTimer = 0.0f;
	} else {
		tlInvalidTimer += dt;
	}

	// Touchdown: low valid reading, or signal lost right after a low reading
	uint8_t onGround = valid ? (alt <= ALT_COMMAND_TERRAIN_TOUCHDOWN_HEIGHT) : (tlLastAlt <= ALT_COMMAND_TERRAIN_LOW_ZONE_HEIGHT);
	tlTouchdownTimer = onGround ? (tlTouchdownTimer + dt) : 0.0f;
	if (tlTouchdownTimer >= ALT_COMMAND_TERRAIN_TOUCHDOWN_CONFIRM_TIME) {
		tlState = TL_TOUCHDOWN;
		return 0;   // handleStepAltChange sees the latch and ramps throttle down
	}

	// Sensor lost high up, or stuck in this phase too long: back to baro stepping
	if (tlInvalidTimer >= ALT_COMMAND_TERRAIN_INVALID_TOLERANCE || tlPhaseTimer >= ALT_COMMAND_TERRAIN_FINAL_TIMEOUT) {
		tlState = TL_ABANDONED;
		return 0;
	}

	if (valid) {
		float sinkRate = constrainToRangeF(alt * ALT_COMMAND_TERRAIN_SINK_RATE_GAIN, ALT_COMMAND_TERRAIN_SINK_RATE_MIN, ALT_COMMAND_TERRAIN_SINK_RATE_MAX);
		fcStatusData.altitudeRef -= sinkRate * dt;
	}
	// (short dropout: ref simply holds) never let it run away below the aircraft
	fcStatusData.altitudeRef = fmaxf(fcStatusData.altitudeRef, z - ALT_COMMAND_TERRAIN_REF_MAX_LEAD);
	return 1;
}
