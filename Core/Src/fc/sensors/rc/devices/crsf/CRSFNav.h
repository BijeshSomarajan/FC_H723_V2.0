#ifndef SRC_FC_SENSORS_RC_DEVICES_CRSF_CRSFNAV_H_
#define SRC_FC_SENSORS_RC_DEVICES_CRSF_CRSFNAV_H_

#include <sys/_stdint.h>

#define CRSF_MSP_NAV_FRAME_TYPE  0x7C

typedef enum {
	CRSF_NAV_ACTION_CLEAR_WAYPOINTS = 1, CRSF_NAV_ACTION_ADD_WAYPOINT, CRSF_NAV_ACTION_START_MISSION, CRSF_NAV_ACTION_ABORT_MISSION
} CRSFNavAction_t;

typedef struct {
	uint8_t action;
	uint8_t payload[];
} CRSFNavCommand_t;

typedef struct {
	uint16_t waypointIndex;
	int32_t latitude;      // 1e-7 degrees
	int32_t longitude;     // 1e-7 degrees
} CRSFNavWaypointPayload_t;

typedef void (*CRSFNavMissionCallback_t)(uint8_t missionCommand);
typedef void (*CRSFNavWayPointCallback_t)(uint8_t wpCommand, CRSFNavWaypointPayload_t wp);

void registerCRSFNavMissionCB(CRSFNavMissionCallback_t callback);
void registerCRSFNavWayPointCB(CRSFNavWayPointCallback_t callback);

#endif /* SRC_FC_SENSORS_RC_DEVICES_CRSF_CRSFNAV_H_ */
