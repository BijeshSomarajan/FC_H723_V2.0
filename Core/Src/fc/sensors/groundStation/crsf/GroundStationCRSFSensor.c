#include <stddef.h>
#include <sys/_stdint.h>

#include "../../../logger/Logger.h"
#include "../../rc/devices/crsf/CRSF.h"
#include "../../rc/RCSensor.h"
#include "../../rc/devices/crsf/CRSFNav.h"
#include "../GroundStationSensor.h"

GroundStationSensorMissionData groundStationSensorMissionData;
GroundStationMissionCallback groundStationMissionCb;

void _groundStationSensorCRSFMissionCb(uint8_t missionCommand) {
	switch (missionCommand) {
	case CRSF_NAV_ACTION_START_MISSION: {
		if (groundStationMissionCb) {
			groundStationMissionCb(NAV_ACTION_START_MISSION);
		}
		break;
	}
	case CRSF_NAV_ACTION_ABORT_MISSION: {
		if (groundStationMissionCb) {
			groundStationMissionCb(NAV_ACTION_ABORT_MISSION);
		}
		break;
	}
	default:
		break;
	}
}

void _groundStationSensorCRSFWPCb(uint8_t wpCommand, CRSFNavWaypointPayload_t wp) {
	switch (wpCommand) {
	case CRSF_NAV_ACTION_CLEAR_WAYPOINTS: {
		clearGroundStationSensorWPData();
		break;
	}
	case CRSF_NAV_ACTION_ADD_WAYPOINT: {
		if (wp.waypointIndex == 0U) {
			clearGroundStationSensorWPData();
		}
		groundStationSensorMissionData.waypointCount++;
		GroundStationSensorWPData groundStationSensorWPData = groundStationSensorMissionData.groundStationSensorCRSFWPs[groundStationSensorMissionData.waypointCount];
		groundStationSensorWPData.latitude = (double) wp.latitude * 1e-7;
		groundStationSensorWPData.longitude = (double) wp.longitude * 1e-7;
		groundStationSensorWPData.status = NAV_STATUS_OPEN;
		groundStationSensorWPData.operationCount = 0;
		groundStationSensorWPData.currentOperationIndex = 0;
		for (uint8_t i = 0; i < NAV_MAX_OPERATIONS; i++) {
			if (groundStationSensorWPData.operations[i].operation != 0) {
				groundStationSensorWPData.operations[i].operation = wp.operations[i].operation;
				groundStationSensorWPData.operations[i].value = (float) wp.operations[i].value * 1e-1;
				groundStationSensorWPData.operations[i].status = NAV_STATUS_OPEN;
				groundStationSensorWPData.operationCount++;
			}
		}
		break;
	}
	default:
		break;
	}
}

void clearGroundStationSensorWPData(void) {
	groundStationSensorMissionData.waypointCount = 0;
	groundStationSensorMissionData.currentWaypointIndex = 0;
	groundStationSensorMissionData.status = NAV_STATUS_INVALID;
}

GroundStationSensorWPData* getGroundStationSensorWPData(uint8_t index) {
	if (index < groundStationSensorMissionData.waypointCount) {
		return &groundStationSensorMissionData.groundStationSensorCRSFWPs[index];
	} else {
		return NULL;
	}
}

uint8_t initGroundStationSensor(GroundStationMissionCallback missionCb) {
#if  RC_RX_TYPE== RC_RX_TYPE_CRSF
	uint8_t status = initCRSF();
	if (status) {
		groundStationMissionCb = missionCb;
		registerCRSFNavMissionCB(_groundStationSensorCRSFMissionCb);
		registerCRSFNavWayPointCB(_groundStationSensorCRSFWPCb);
		resetGroundStationSensor();
		logString("[GroundStation Sensor] : CRSF > Success\n");
	} else {
		logString("[GroundStation Sensor] : CRSF > Failed\n");
	}
	return status;
#else
	logString("[GroundStation Sensor] : CRSF > Unavailable\n");
	return 1;
#endif
}
