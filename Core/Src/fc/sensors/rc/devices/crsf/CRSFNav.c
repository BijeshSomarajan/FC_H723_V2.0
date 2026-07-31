#include "CRSFNav.h"

#include "../../../../logger/Logger.h"

CRSFNavMissionCallback_t crsfMissionCB;
CRSFNavWayPointCallback_t crsfWayPointCB;

void registerCRSFNavMissionCB(CRSFNavMissionCallback_t callback) {
	crsfMissionCB = callback;
}

void registerCRSFNavWayPointCB(CRSFNavWayPointCallback_t callback) {
	crsfWayPointCB = callback;
}

char bufP1[64];
void manageCRSFNavFrame(const uint8_t *payload) {
	/*
	 0xC8,       -- Destination (Flight Controller)
	 0xEA,       -- Origin (Radio)
	 0x01        -- Sequence number
	 [Actual Payload]
	 */
	const CRSFNavCommand_t *cmd = (const CRSFNavCommand_t*) (payload + 3);
	switch (cmd->action) {
	case CRSF_NAV_ACTION_CLEAR_WAYPOINTS: {
		if (crsfWayPointCB) {
			CRSFNavWaypointPayload_t dummy = { 0 };
			crsfWayPointCB(CRSF_NAV_ACTION_CLEAR_WAYPOINTS, dummy);
		}
		break;
	}
	case CRSF_NAV_ACTION_ADD_WAYPOINT: {
		if (crsfWayPointCB) {
			const uint8_t *p = cmd->payload;
			CRSFNavWaypointPayload_t wp;
			wp.waypointIndex = (uint16_t) ((p[0] << 8) | p[1]);
			wp.latitude = (int32_t) (((uint32_t) p[2] << 24) | ((uint32_t) p[3] << 16) | ((uint32_t) p[4] << 8) | (uint32_t) p[5]);
			wp.longitude = (int32_t) (((uint32_t) p[6] << 24) | ((uint32_t) p[7] << 16) | ((uint32_t) p[8] << 8) | (uint32_t) p[9]);
			crsfWayPointCB(CRSF_NAV_ACTION_ADD_WAYPOINT, wp);
		}
		break;
	}
	default:
		break;
	}
}
