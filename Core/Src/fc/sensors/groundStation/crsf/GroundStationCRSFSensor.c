#include <stddef.h>
#include <sys/_stdint.h>

#include "../../../logger/Logger.h"
#include "../../rc/devices/crsf/CRSF.h"
#include "../../rc/devices/crsf/CRSFNav.h"
#include "../GroundStationSensor.h"

#define GroundStationSensorCRSFWPMax 32

GroundStationSensorWPData groundStationSensorCRSFWPs[GroundStationSensorCRSFWPMax];
uint8_t groundStationSensorCRSFWPCount = 0;

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

char bufP[64];
void _groundStationSensorCRSFWPCb(uint8_t wpCommand, CRSFNavWaypointPayload_t wp) {
	switch (wpCommand) {
	case CRSF_NAV_ACTION_CLEAR_WAYPOINTS: {
		clearGroundStationSensorWPData();
		break;
	}
	case CRSF_NAV_ACTION_ADD_WAYPOINT: {
		if(wp.waypointIndex == 0){
			clearGroundStationSensorWPData();
		}
		GroundStationSensorWPData groundStationSensorWPData;
		groundStationSensorWPData.waypointIndex = groundStationSensorCRSFWPCount;
		groundStationSensorWPData.latitude = (double) wp.latitude * 1e-7;
		groundStationSensorWPData.longitude = (double) wp.longitude * 1e-7;
		setGroundStationSensorWPData(groundStationSensorWPData);
		break;
	}
	default:
		break;
	}
}

void setGroundStationSensorWPData(GroundStationSensorWPData groundStationSensorWPData) {
	if (groundStationSensorWPData.waypointIndex < GroundStationSensorCRSFWPMax) {
		groundStationSensorCRSFWPs[groundStationSensorWPData.waypointIndex] = groundStationSensorWPData;
		groundStationSensorCRSFWPCount++;
	}
}

void clearGroundStationSensorWPData(void) {
	groundStationSensorCRSFWPCount = 0;
}

uint8_t getGroundStationSensorWPDataCount() {
	return groundStationSensorCRSFWPCount;
}

GroundStationSensorWPData* getGroundStationSensorWPData(uint8_t index) {
	if (index < groundStationSensorCRSFWPCount) {
		return &groundStationSensorCRSFWPs[index];
	} else {
		return NULL;
	}
}

uint8_t initGroundStationSensor(GroundStationMissionCallback missionCb) {
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
}

void resetGroundStationSensor() {
	groundStationSensorCRSFWPCount = 0;
}
