#include "blackBoxManager.h"

#include <stdio.h>
#include <string.h>

#include "../../FCConfig.h"
#include "../../logger/Logger.h"
#include "../../timers/DeltaTimer.h"
#include "../../imu/IMU.h"
#include "../../sensors/attitude/AttitudeSensor.h"
#include "blackbox.h"

float blackBoxUpdateDt = 0;
float blackBoxSyncDt = 0;
float blackBoxTime = 0;

uint8_t initBlackBoxManager(void) {
	uint8_t status = initBlackbox();
	if (!status) {
		logString("[BlackBox Manager] >> IO >> Failed\n");
	} else {
		logString("[BlackBox Manager] >> IO >> Success\n");
		status = startBlackboxSession();
		char msg[80];
		sprintf(msg, "BB Manager: start returned=%u\n", status);
		logString(msg);
		if (status) {
			logString("[BlackBox Manager] >> startBlackboxSession >> Success\n");
		} else {
			logString("[BlackBox Manager] >> startBlackboxSession >> Failed\n");
		}
	}
	return status;
}

static uint32_t testCount = 0;
extern IMU_DATA imuData;
void doBlackboxManagement() {
	float dt = getDeltaTime(BLACKBOX_TIMER_CHANNEL);
	processBlackbox();
	if (isBlackBoxAvailable()) {
		blackBoxTime += dt;
		blackBoxUpdateDt += dt;
		if (blackBoxUpdateDt >= BLACK_BOX_RECORD_PERIOD) {
			blackBoxUpdateDt = 0;
			static char msg[80];
			snprintf(msg, sizeof(msg), "Count:=%u,AGDt:%f,ARHSDt:%f\n", testCount,sensorAttitudeData.agtDataUpdateDt,imuData.arhsDt);
			uint32_t status = recordToBlackbox(msg, strlen(msg));
			testCount++;
			if(testCount > 100000){
				testCount  = 0;
			}
			if (!status) {
				logString("[BM]:write failed\n");
			}
		}
		blackBoxSyncDt += dt;
		if (blackBoxSyncDt >= BLACK_BOX_SYNC_PERIOD) {
			if (syncBlackbox()) {
				logPrintf("[BM]:SyncComplete,Count:%d,Dt:%f\n",testCount,blackBoxSyncDt);
			}
			blackBoxSyncDt = 0;
		}
	} else {
		blackBoxTime = 0;
		blackBoxUpdateDt = 0;
		blackBoxSyncDt = 0;
	}
}
