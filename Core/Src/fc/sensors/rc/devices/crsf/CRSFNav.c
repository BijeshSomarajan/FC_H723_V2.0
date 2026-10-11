#include "CRSFNav.h"

#include <stddef.h>

#include "../../../../util/CommonUtil.h"

CRSFNavMissionCallback_t crsfMissionCB;
CRSFNavWayPointCallback_t crsfWayPointCB;

void registerCRSFNavMissionCB(CRSFNavMissionCallback_t callback) {
	crsfMissionCB = callback;
}

void registerCRSFNavWayPointCB(CRSFNavWayPointCallback_t callback) {
	crsfWayPointCB = callback;
}

void manageCRSFNavFrame(const uint8_t *payload, size_t frameLength) {
	if (payload == NULL || frameLength < 4U) {
		return;
	}
	const CRSFNavAction_t action = (CRSFNavAction_t) payload[3];
	const uint8_t *p = payload + 4;
	const size_t dataLength = frameLength - 4U;
	switch (action) {
	case CRSF_NAV_ACTION_ADD_WAYPOINT: {
		const size_t expectedLength = 10U + (5U * CRSF_NAV_MAX_OPERATIONS);
		if (dataLength != expectedLength) {
			break;
		}
		if (crsfWayPointCB != NULL) {
			CRSFNavWaypointPayload_t wp = { 0 };
			wp.waypointIndex = readU16BE(p);
			wp.latitude = readI32BE(p + 2);
			wp.longitude = readI32BE(p + 6);
			p += 10;
			for (size_t i = 0; i < CRSF_NAV_MAX_OPERATIONS; i++) {
				wp.operations[i].value = readI32BE(p);
				wp.operations[i].operation = p[4];
				p += 5;
			}
			crsfWayPointCB(CRSF_NAV_ACTION_ADD_WAYPOINT, wp);
		}
		break;
	}
	case CRSF_NAV_ACTION_START_MISSION: {
		if (crsfMissionCB) {
			crsfMissionCB(CRSF_NAV_ACTION_START_MISSION);
		}
		break;
	}
	case CRSF_NAV_ACTION_ABORT_MISSION: {
		if (crsfMissionCB) {
			crsfMissionCB(CRSF_NAV_ACTION_ABORT_MISSION);
		}
		break;
	}
	case CRSF_NAV_ACTION_CLEAR_WAYPOINTS: {
		if (crsfWayPointCB) {
			CRSFNavWaypointPayload_t dummy = { 0 };
			crsfWayPointCB(CRSF_NAV_ACTION_CLEAR_WAYPOINTS, dummy);
		}
		break;
	}
	default: {
		break;
	}
	}
}
