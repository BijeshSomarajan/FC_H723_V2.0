#include <stddef.h>

#include "../../../../logger/Logger.h"
#include "../../../../util/CommonUtil.h"
#include "CRSFTransceiver.h"

CRSFMissionCallback crsfMissionCallBackFun;
CRSFMissionWPCallback crsfMissionWPCallBackFun;

void registerCRSFMissionCallBack(CRSFMissionCallback callbackFun) {
	crsfMissionCallBackFun = callbackFun;
}

void registerCRSFMissionWPCallBack(CRSFMissionWPCallback callbackFun) {
	crsfMissionWPCallBackFun = callbackFun;
}

char testBuf[128];
void manageCRSFMissionFrame(const uint8_t *payload) {
	if (payload == NULL) {
		return;
	}
	// payload[0..2] = destination, origin, sequence
	const CRSFMissionAction action = (CRSFMissionAction) payload[3];
	const uint8_t *p = payload + 4;
	switch (action) {
	case CRSF_MISSION_ACTION_ADD_WAYPOINT: {
		logString("AddWp\n");
		if (crsfMissionWPCallBackFun != NULL) {
			CRSFMissionWP wp = { 0 };
			wp.waypointIndex = readU16BE(p);
			wp.latitude = readI32BE(p + 2);
			wp.longitude = readI32BE(p + 6);
			wp.velocity = readU16BE(p + 10);
			p += 12;
			for (size_t i = 0; i < CRSF_MISSION_MAX_OPERATIONS; i++) {
				wp.operations[i].value = readI32BE(p);
				wp.operations[i].operation = p[4];
				p += 5;
			}
			crsfMissionWPCallBackFun(CRSF_MISSION_ACTION_ADD_WAYPOINT, wp);
		}
		break;
	}
	case CRSF_MISSION_ACTION_START: {
		if (crsfMissionCallBackFun) {
			crsfMissionCallBackFun(CRSF_MISSION_ACTION_START);
		}
		break;
	}
	case CRSF_MISSION_ACTION_ABORT: {
		if (crsfMissionCallBackFun) {
			crsfMissionCallBackFun(CRSF_MISSION_ACTION_ABORT);
		}
		break;
	}
	case CRSF_MISSION_ACTION_CLEAR_WAYPOINTS: {
		if (crsfMissionWPCallBackFun) {
			CRSFMissionWP dummy = { 0 };
			crsfMissionWPCallBackFun(CRSF_MISSION_ACTION_CLEAR_WAYPOINTS, dummy);
		}
		break;
	}
	default:
		break;
	}
}
