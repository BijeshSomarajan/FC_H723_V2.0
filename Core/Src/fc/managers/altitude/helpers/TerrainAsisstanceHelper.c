#include "TerrainAsisstanceHelper.h"

#include <math.h>
#include <sys/_stdint.h>

#include "../../../calibration/Calibration.h"
#include "../../../FCConfig.h"
#include "../../../logger/Logger.h"
#include "../../../memory/Memory.h"
#include "../../../sensors/altitude/AltitudeSensor.h"
#include "../../../status/FCStatus.h"
#include "../../../util/MathUtil.h"
#include "../../position/common/PositionCommon.h"

typedef enum {
	TL_IDLE = 0, TL_ACTIVE, TL_TOUCHDOWN, TL_ABANDONED
} TerrainAssistState;

static TerrainAssistState tlState = TL_IDLE;
static float tlLiftOffHeight = 2.0f;     // m, takeoff height; also the landing hand-over height
static float tlLastAlt = 0.0f;           // last valid height at the gear
static float tlTouchdownTimer = 0.0f;
static float tlInvalidTimer = 0.0f;
static float tlPhaseTimer = 0.0f;
static uint8_t tlTakeoffValid = 0;       // terrain reading usable this tick (takeoff)
static float tlTakeoffAlt = 0.0f;        // height at the gear this tick (takeoff)
static float tlMountHeight = ALT_COMMAND_TERRAIN_MOUNT_HEIGHT;        // height at the gear this tick (takeoff)

void initTerrainAssistedHelper(float liftOffHeight) {
	// must stay clear of the low zone, where a lost signal means touchdown
	tlLiftOffHeight = fmaxf(liftOffHeight, ALT_COMMAND_TERRAIN_LOW_ZONE_HEIGHT * 2.0f);
	tlMountHeight = get100XScaledCalibrationValue(CALIB_PROP_DRONE_MOUNT_HEIGHT_ADDR);
	tlMountHeight = constrainToRangeF(tlMountHeight, ALT_COMMAND_TERRAIN_MOUNT_HEIGHT, tlMountHeight);
	logPrintf("[TerrainAssistedHelper]: tlLiftOffHeight:%.2f, tlMountHeight:%.2f\n", tlLiftOffHeight, tlMountHeight);
	resetTerrainLandingStates();
}

void resetTerrainLandingStates(void) {
	tlState = TL_IDLE;
	tlLastAlt = 0.0f;
	tlTouchdownTimer = 0.0f;
	tlInvalidTimer = 0.0f;
	tlPhaseTimer = 0.0f;
	tlTakeoffValid = 0;
	tlTakeoffAlt = 0.0f;
}

uint8_t isTerrainAssistanceEnabled(void) {
	return (fcStatusData.isTerrainSensorExist == 1) && fcStatusData.isTerrainAltDataReliable;
}

/**
 * Height of the landing gear above ground. Returns 0 when assistance is not
 * enabled or the reading is outside the sensor's valid range (e.g. blind zone).
 */
static uint8_t getTerrainAltitude(float *altitude) {
	if (!isTerrainAssistanceEnabled()) {
		return 0;
	}
	float h = sensorAltitudeData.altitudeTerrainFiltered;
	if (h < SENSOR_LIDAR_ALT_MIN || h > SENSOR_LIDAR_ALT_MAX) {
		return 0;
	}
	*altitude = fmaxf(h - tlMountHeight, 0.0f);
	return 1;
}

// ---------------------------------------------------------------------------
// Landing
// ---------------------------------------------------------------------------
uint8_t isTerrainLandingActive(void) {
	return tlState == TL_ACTIVE;
}

uint8_t isTerrainLandingComplete(void) {
	return tlState == TL_TOUCHDOWN;
}

__ATTR_ITCM_TEXT
void handleTerrainAssistedLanding(float dt) {
	if (tlState == TL_IDLE && !isTerrainAssistanceEnabled()) {
		return;   // no usable terrain sensor: baro landing
	}

	float liftOffThrottle = fcStatusData.liftOffThrottlePercent * MAX_PERMISSIBLE_THROTTLE_DELTA;
	if (tlState >= TL_TOUCHDOWN || fcStatusData.currentThrottle < liftOffThrottle || fcStatusData.altitudeHoldState != ALT_HOLD_STATE_LOCKED) {
		return;   // done, fallen back to baro, or the throttle ramp-down is in charge
	}

	float alt = 0.0f;
	uint8_t valid = getTerrainAltitude(&alt);
	float z = positionCordinateData.zPosition;

	if (tlState == TL_IDLE) {
		if (!valid || alt > tlLiftOffHeight) {
			return;   // no sensor / too high: baro stepping
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
		return;   // caller sees the latch and ramps throttle down
	}

	// Sensor lost high up, or stuck in this phase too long: back to baro stepping
	if (tlInvalidTimer >= ALT_COMMAND_TERRAIN_INVALID_TOLERANCE || tlPhaseTimer >= ALT_COMMAND_TERRAIN_FINAL_TIMEOUT) {
		tlState = TL_ABANDONED;
		return;
	}

	if (valid) {
		float sinkRate = constrainToRangeF(alt * ALT_COMMAND_TERRAIN_SINK_RATE_GAIN, ALT_COMMAND_TERRAIN_SINK_RATE_MIN, ALT_COMMAND_TERRAIN_SINK_RATE_MAX);
		fcStatusData.altitudeRef -= sinkRate * dt;
	}
	// (short dropout: ref simply holds) never let it run away below the aircraft
	fcStatusData.altitudeRef = fmaxf(fcStatusData.altitudeRef, z - ALT_COMMAND_TERRAIN_REF_MAX_LEAD);
}

// ---------------------------------------------------------------------------
// Takeoff
// ---------------------------------------------------------------------------
__ATTR_ITCM_TEXT
void handleTerrainAssistedTakeoff(void) {
	float alt = 0.0f;
	tlTakeoffValid = 0;
	if (!isTerrainAssistanceEnabled() || tlState == TL_ABANDONED) {
		return;   // no usable terrain sensor, or terrain rejected for this takeoff: baro
	}
	if (!getTerrainAltitude(&alt)) {
		return;   // blind zone on the ground or dropout: baro this tick
	}
	if (tlState == TL_IDLE) {
		if (alt > tlLiftOffHeight * 0.5f) {
			tlState = TL_ABANDONED;   // not launched from the ground (hand launch, platform): baro for this takeoff
			return;
		}
		tlState = TL_ACTIVE;
	}
	tlTakeoffAlt = alt;
	tlTakeoffValid = 1;
}

uint8_t isTerrainTakeoffActive(void) {
	return tlTakeoffValid;
}

uint8_t isTerrainAltitudeReached(void) {
	return tlTakeoffValid && (fabsf(tlLiftOffHeight - tlTakeoffAlt) <= ALT_COMMAND_TERRAIN_TAKEOFF_TOLERANCE);
}

float getTerrainTakeoffRef(void) {
	return positionCordinateData.zPosition + (tlLiftOffHeight - tlTakeoffAlt);
}
