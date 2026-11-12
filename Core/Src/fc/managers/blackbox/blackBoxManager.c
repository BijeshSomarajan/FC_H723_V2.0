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

static const char blackboxHeader[] = "time_ms,throttle,pitch_stick,roll_stick,yaw_stick,pitch,roll,heading,pitch_rate,roll_rate,yaw_rate,baro_alt,z_pos,x_vel,y_vel,z_vel,z_acc,grndSpd,venBias,tiltComp\n";
static const char blackboxDataFormat[] = "%lu,%d,%d,%d,%d,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n";

uint8_t initBlackBoxManager(void) {
	uint8_t status = initBlackbox();
	if (!status) {
		logString("[BM]>> IO >> Failed\n");
	} else {
		blackBoxAvailable = 1;
		logString("[BM]>> IO >> Success\n");
	}
	return status;
}

__ATTR_ITCM_TEXT
static void handleBlackboxSession(void) {
	if (!fcStatusData.canStart) {
		needNewBlackboxSession = 1;
		if (blackBoxSessionInitSuccess) {
			if (endBlackboxSession()) {
				blackBoxManagerState = BM_SESSION_CLOSING;
				blackBoxSessionInitSuccess = 0;
#if BLACK_BOX_MANAGER_DEBUG == 1
				logString("[BM]:Closing previous session\n");
#endif
			}
		}
	} else if (needNewBlackboxSession) {
		if (blackBoxSessionInitSuccess) {
			if (endBlackboxSession()) {
				blackBoxSessionInitSuccess = 0;
				blackBoxManagerState = BM_SESSION_CLOSING;
#if BLACK_BOX_MANAGER_DEBUG == 1
				logString("[BM]:Closing previous session\n");
#endif
			}
		} else {
			if (blackBoxManagerState == BM_SESSION_CLOSING) {
				if (isBlackboxIdle()) {
					blackBoxManagerState = BM_SESSION_IDLE;
#if BLACK_BOX_MANAGER_DEBUG == 1
					logString("[BM]:Previous session was closed\n");
#endif
				}
			}
			if (blackBoxManagerState == BM_SESSION_IDLE && isBlackboxIdle()) {
				if (startBlackboxSession()) {
					blackBoxSessionInitSuccess = 1;
					needNewBlackboxSession = 0;
					blackBoxTime = 0;
					blackBoxUpdateDt = 0;
					blackBoxSyncDt = 0;
					blackboxDataTimer = 0;
					blackboxHeaderWritten = 0;
#if BLACK_BOX_MANAGER_DEBUG == 1
					logString("[BM]:New session started\n");
#endif
				} else {
#if BLACK_BOX_MANAGER_DEBUG == 1
					logString("[BM]:New session failed\n");
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

	int32_t pitch = (int32_t) (sensorAttitudeData.pitch * 100.0f);
	int32_t roll = (int32_t) (sensorAttitudeData.roll * 100.0f);
	int32_t heading = (int32_t) (sensorAttitudeData.heading * 100.0f);

	int32_t pitchRate = (int32_t) (sensorAttitudeData.pitchRate * 100.0f);
	int32_t rollRate = (int32_t) (sensorAttitudeData.rollRate * 100.0f);
	int32_t headingRate = (int32_t) (sensorAttitudeData.yawRate * 100.0f);

	int32_t baroAlt = (int32_t) (sensorAltitudeData.altitudeSLScaled * 100.0f);
	int32_t estZPos = (int32_t) (positionCordinateData.zPosition * 100.0f);
	int32_t estXVel = (int32_t) (positionCordinateData.xVelocity * 100.0f);
	int32_t estYVel = (int32_t) (positionCordinateData.yVelocity * 100.0f);
	int32_t estZVel = (int32_t) (positionCordinateData.zVelocity * 100.0f);
	int32_t estZAcc = (int32_t) (positionCordinateData.zAcceleration * 100.0f);
	int32_t grndSpd = (int16_t) (getGroundSpeed() * 100.0f);
	int32_t venturiBias = venturiEstimateData.venturiBias * 100.0f;
	int32_t tiltCompThDelta = controlData.tiltCompThDelta * 100.0f;

	uint32_t timeMs = (uint32_t) (blackboxDataTimer * 1000.0f);
	static char msg[256];
	int len = snprintf(msg, sizeof(msg), blackboxDataFormat, (unsigned long) timeMs, thStick, pitchStick, rollStick, yawStick, (long) pitch, (long) roll, (long) heading, (long) pitchRate, (long) rollRate, (long) headingRate, (long) baroAlt, (long) estZPos, (long) estXVel, (long) estYVel,
			(long) estZVel, (long) estZAcc, grndSpd, venturiBias, tiltCompThDelta);

	if (len <= 0 || (size_t) len >= sizeof(msg)) {
		return 0;
	}
	return recordToBlackbox(msg, (uint32_t) len);
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
				logString("[BM]:Synced\n");
#endif
				}
				blackBoxSyncDt = 0;
			}
		} else {
			blackBoxTime = 0;
			blackBoxUpdateDt = 0;
			blackBoxSyncDt = 0;
		}
	}
}
