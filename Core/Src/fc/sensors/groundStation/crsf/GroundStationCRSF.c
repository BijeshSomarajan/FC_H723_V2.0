#include <stddef.h>
#include <sys/_stdint.h>

#include "../../../logger/Logger.h"
#include "../../rc/devices/crsf/CRSF.h"
#include "../../rc/devices/crsf/CRSFTransceiver.h"
#include "../../rc/RCSensor.h"
#include "../GroundStation.h"

MissionData missionData;
MissionCallback missionCallBackFun;

void _crsfMissionCallBack(uint8_t missionCommand) {
	switch (missionCommand) {
	case CRSF_MISSION_ACTION_START: {
		if (missionCallBackFun) {
			missionCallBackFun(MISSION_ACTION_START);
		}
		break;
	}
	case CRSF_MISSION_ACTION_ABORT: {
		if (missionCallBackFun) {
			clearMissionData();
			missionCallBackFun(MISSION_ACTION_ABORT);
		}
		break;
	}
	default:
		break;
	}
}

void _crsfMissionWPCallBack(uint8_t wpCommand, CRSFMissionWP wp) {
	switch (wpCommand) {
	case CRSF_MISSION_ACTION_CLEAR_WAYPOINTS: {
		clearMissionData();
		break;
	}
	case CRSF_MISSION_ACTION_ADD_WAYPOINT: {
		if (wp.waypointIndex >= NAV_MAX_WAYPOINTS) {
			break; // index from the radio link is out of range, drop the packet
		}
		if (wp.waypointIndex == 0) {
			clearMissionData();
			missionData.status = MISSION_STATUS_OPEN;
		}
		missionData.waypointCount++;
		MissionWPData *missionWPData = &missionData.waypoints[wp.waypointIndex];
		missionWPData->latitude = (double) wp.latitude * 1e-7;
		missionWPData->longitude = (double) wp.longitude * 1e-7;
		missionWPData->velocity = (float) wp.velocity * 1e-1;
		missionWPData->status = MISSION_STATUS_OPEN;
		missionWPData->operationCount = 0;
		missionWPData->currentOperationIndex = 0;
		for (uint8_t i = 0; i < NAV_MAX_OPERATIONS; i++) {
			if (wp.operations[i].operation != 0) {
				missionWPData->operations[i].operation = wp.operations[i].operation;
				missionWPData->operations[i].value = (float) wp.operations[i].value * 1e-1;
				missionWPData->operations[i].status = MISSION_STATUS_OPEN;
				missionWPData->operationCount++;
			}
		}
		break;
	}
	default:
		break;
	}
}

void setMissionWPData(MissionWPData missionWPData) {
	missionData.waypoints[missionData.waypointCount] = missionWPData;
	missionData.status = MISSION_STATUS_OPEN;
	missionData.waypointCount++;
}

void clearMissionData(void) {
	missionData.waypointCount = 0;
	missionData.currentWaypointIndex = 0;
	missionData.status = MISSION_STATUS_INVALID;
}

MissionData* getMissionData() {
	return &missionData;
}

uint8_t initGroundStation(MissionCallback callBackFun) {
#if  RC_RX_TYPE== RC_RX_TYPE_CRSF
	uint8_t status = initCRSF();
	if (status) {
		missionCallBackFun = callBackFun;
		registerCRSFMissionCallBack(_crsfMissionCallBack);
		registerCRSFMissionWPCallBack(_crsfMissionWPCallBack);
		clearMissionData();
		logString("[GroundStation CRSF] : Init > Success\n");
	} else {
		logString("[GroundStation CRSF] : Init > Failed\n");
	}
	return status;
#else
	logString("[GroundStation CRSF] : Init > Unavailable\n");
	return 1;
#endif
}
