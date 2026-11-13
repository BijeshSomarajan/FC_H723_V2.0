#include "blackBoxManager.h"

#include <stddef.h>
#include <stdio.h>

#include "../../control/ControlData.h"
#include "../../FCConfig.h"
#include "../../logger/Logger.h"
#include "../../memory/Memory.h"
#include "../../sensors/altitude/AltitudeSensor.h"
#include "../../sensors/attitude/AttitudeSensor.h"
#include "../../sensors/rc/RCSensor.h"
#include "../../status/FCStatus.h"
#include "../../timers/DeltaTimer.h"
#include "../position/common/PositionCommon.h"
#include "../position/estimator/PositionEstimatorHelper.h"
#include "../position/estimator/VenturiBiasEstimator.h"
#include "blackbox.h"

float blackBoxUpdateDt = 0;
float blackBoxSyncDt = 0;
float blackBoxTime = 0;
float blackboxDataTimer = 0;

uint8_t blackBoxSessionInitSuccess = 0;
uint8_t blackBoxAvailable = 0;
uint8_t needNewBlackboxSession = 1;
static uint8_t blackboxHeaderWritten = 0;

typedef enum {
	BM_SESSION_IDLE, BM_SESSION_CLOSING
} blackBoxManagerState_e;

static blackBoxManagerState_e blackBoxManagerState = BM_SESSION_IDLE;

/* A start failed while the card is faulted: do not retry (each retry runs a blocking sdcard_recover()) until the next canStart cycle. */
static uint8_t blackboxStartFailedOnce = 0;

static const char blackboxHeader[] = "ts,sThr,sPitch,sRoll,sYaw,pitch,roll,heading,rPitch,rRoll,rYaw,bAlt,tAlt,zPos,xVel,yVel,zVel,xAcc,yAcc,zAcc,gSpd,vBias,tComp,altDoB,altCon,hoverTh,controlTh\n";
static const char blackboxDataFormat[] = "%lu,%d,%d,%d,%d,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n";
static char blackboxData[256];

uint8_t initBlackBoxManager(void) {
	uint8_t status = initBlackbox();
	if (!status) {
		logString("[Blackbox Manager]>> IO >> Failed\n");
	} else {
		blackBoxAvailable = 1;
		logString("[Blackbox Manager]>> IO >> Success\n");
	}
	return status;
}

__ATTR_ITCM_TEXT
static void handleBlackboxSession(void) {
	if (!fcStatusData.canStart) {
		needNewBlackboxSession = 1;
		blackboxStartFailedOnce = 0;
		if (blackBoxSessionInitSuccess) {
			if (endBlackboxSession()) {
				blackBoxManagerState = BM_SESSION_CLOSING;
				blackBoxSessionInitSuccess = 0;
#if BLACK_BOX_MANAGER_DEBUG == 1
				logString("[Blackbox Manager]:Closing previous session\n");
#endif
			}
		}
	} else if (needNewBlackboxSession) {
		if (blackBoxSessionInitSuccess) {
			if (endBlackboxSession()) {
				blackBoxSessionInitSuccess = 0;
				blackBoxManagerState = BM_SESSION_CLOSING;
#if BLACK_BOX_MANAGER_DEBUG == 1
				logString("[Blackbox Manager]:Closing previous session\n");
#endif
			}
		} else {
			if (blackBoxManagerState == BM_SESSION_CLOSING) {
				if (isBlackboxIdle()) {
					blackBoxManagerState = BM_SESSION_IDLE;
#if BLACK_BOX_MANAGER_DEBUG == 1
					logString("[Blackbox Manager]:Previous session was closed\n");
#endif
				}
			}
			if (blackBoxManagerState == BM_SESSION_IDLE && isBlackboxIdle()) {
				if (isBlackBoxFaulted() && blackboxStartFailedOnce) {
					return;
				}
				if (startBlackboxSession()) {
					blackboxStartFailedOnce = 0;
					blackBoxSessionInitSuccess = 1;
					needNewBlackboxSession = 0;
					blackBoxTime = 0;
					blackBoxUpdateDt = 0;
					blackBoxSyncDt = 0;
					blackboxDataTimer = 0;
					blackboxHeaderWritten = 0;
#if BLACK_BOX_MANAGER_DEBUG == 1
					logString("[Blackbox Manager]:New session started\n");
#endif
				} else {
					blackboxStartFailedOnce = 1;
#if BLACK_BOX_MANAGER_DEBUG == 1
					logString("[Blackbox Manager]:New session failed\n");
#endif
				}
			}
		}
	}
}

static uint8_t writeBlackboxHeader(void) {
	uint32_t len = sizeof(blackboxHeader) - 1U;
	return recordToBlackbox(blackboxHeader, len) == len;
}

uint8_t prepareBlackboxData(float dt) {
	blackboxDataTimer += dt;
	int16_t thStick = rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX];
	int16_t pitchStick = rcData.RC_DELTA_DATA[RC_PITCH_CHANNEL_INDEX];
	int16_t rollStick = rcData.RC_DELTA_DATA[RC_ROLL_CHANNEL_INDEX];
	int16_t yawStick = rcData.RC_DELTA_DATA[RC_YAW_CHANNEL_INDEX];

	int32_t pitch = (int32_t) (sensorAttitudeData.pitch * 10.0f);
	int32_t roll = (int32_t) (sensorAttitudeData.roll * 10.0f);
	int32_t heading = (int32_t) (sensorAttitudeData.heading * 10.0f);

	int32_t pitchRate = (int32_t) (sensorAttitudeData.pitchRate * 10.0f);
	int32_t rollRate = (int32_t) (sensorAttitudeData.rollRate * 10.0f);
	int32_t headingRate = (int32_t) (sensorAttitudeData.yawRate * 10.0f);

	int32_t baroAlt = (int32_t) (sensorAltitudeData.altitudeSLScaled * 100.0f);
	int32_t terrainAlt = (int32_t) (sensorAltitudeData.altitudeTerrainFiltered * 100.0f);
	int32_t estZPos = (int32_t) (positionCordinateData.zPosition * 100.0f);

	int32_t estXVel = (int32_t) (positionCordinateData.xVelocity * 100.0f);
	int32_t estYVel = (int32_t) (positionCordinateData.yVelocity * 100.0f);
	int32_t estZVel = (int32_t) (positionCordinateData.zVelocity * 100.0f);

	int32_t estXAcc = (int32_t) (positionCordinateData.xAcceleration * 100.0f);
	int32_t estYAcc = (int32_t) (positionCordinateData.yAcceleration * 100.0f);
	int32_t estZAcc = (int32_t) (positionCordinateData.zAcceleration * 100.0f);

	int32_t grndSpd = (int32_t) (getGroundSpeed() * 100.0f);
	int32_t venturiBias = (int32_t) venturiEstimateData.venturiBias * 100.0f;
	int32_t tiltCompThDelta = (int32_t) controlData.tiltCompThDelta * 100.0f;
	int32_t altitudeDOBControl = (int32_t) controlData.altitudeDOBControl * 100.0f;
	int32_t altitudeControl = (int32_t) controlData.altitudeControl * 100.0f;
	int32_t hoverThrottle = (int32_t) fcStatusData.hoverThrottle * 10.0f;
	int32_t throttleControl = (int32_t) controlData.throttleControl * 10.0f;

	uint32_t timeMs = (uint32_t) (blackboxDataTimer * 100.0f);
	int len = snprintf(blackboxData, sizeof(blackboxData), blackboxDataFormat, (unsigned long) timeMs, thStick, pitchStick, rollStick, yawStick, (long) pitch, (long) roll, (long) heading, (long) pitchRate, (long) rollRate, (long) headingRate, (long) baroAlt, (long) terrainAlt, (long) estZPos,
			(long) estXVel, (long) estYVel, (long) estZVel, (long) estXAcc, (long) estYAcc, (long) estZAcc, (long) grndSpd, (long) venturiBias, (long) tiltCompThDelta, (long) altitudeDOBControl, (long) altitudeControl, (long) hoverThrottle, (long) throttleControl);

	if (len <= 0 || (size_t) len >= sizeof(blackboxData)) {
		return 0;
	}
	return recordToBlackbox(blackboxData, (uint32_t) len);
}

__ATTR_ITCM_TEXT
void doBlackboxManagement(void) {
	if (blackBoxAvailable) {
		processBlackbox();
		handleBlackboxSession();
		if (blackBoxSessionInitSuccess && isBlackBoxAvailable()) {
			float dt = getDeltaTime(BLACKBOX_TIMER_CHANNEL);
			blackBoxTime += dt;
			blackBoxUpdateDt += dt;
			blackBoxSyncDt += dt;
			if (!blackboxHeaderWritten) {
				blackboxHeaderWritten = writeBlackboxHeader();
			}
			if (blackboxHeaderWritten && blackBoxUpdateDt >= BLACK_BOX_RECORD_PERIOD) {
				float recordDt = blackBoxUpdateDt;
				blackBoxUpdateDt = 0;
				if (!prepareBlackboxData(recordDt)) {
#if BLACK_BOX_MANAGER_DEBUG == 1
					printBlackboxDebug();
#endif
				}
			}
			if (blackBoxSyncDt >= BLACK_BOX_SYNC_PERIOD) {
				if (syncBlackbox()) {
#if BLACK_BOX_MANAGER_DEBUG == 1
					logString("[Blackbox Manager]:Synced\n");
#endif
				}
				blackBoxSyncDt = 0;
			}
		} else {
			blackBoxTime = 0;
			blackBoxUpdateDt = 0;
			blackBoxSyncDt = 0;
#if BLACK_BOX_MANAGER_DEBUG == 1
			//logString("[Blackbox Manager]:Unavailable\n");
#endif
		}
	}
}
