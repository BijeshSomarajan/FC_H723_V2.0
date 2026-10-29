#ifndef SRC_FC_SENSORS_GROUNDSTATION_GROUNDSTATION_H_
#define SRC_FC_SENSORS_GROUNDSTATION_GROUNDSTATION_H_
#include <sys/_stdint.h>
#define NAV_MAX_OPERATIONS 8
#define NAV_MAX_WAYPOINTS 32

typedef enum {
	MISSION_ACTION_CLEAR_WAYPOINTS = 1, MISSION_ACTION_ADD_WAYPOINT, MISSION_ACTION_START, MISSION_ACTION_ABORT
} MissionAction;

typedef enum {
	MISSION_OPERATION_TAKEOFF = 1, MISSION_OPERATION_LAND, MISSION_OPERATION_SET_ALT, MISSION_OPERATION_SET_HEADING, MISSION_OPERATION_WAIT, MISSION_OPERATION_CUSTOM
} MissionOperation;

typedef enum {
	MISSION_STATUS_INVALID = 0, MISSION_STATUS_OPEN, MISSION_STATUS_WIP, MISSION_STATUS_WIP_NAV,MISSION_STATUS_WIP_OP,MISSION_STATUS_COMPLETE
} MissionStaus;

typedef struct {
	float value;
	uint8_t operation;
	uint8_t status;
} MissionOperationData;

typedef struct {
	uint16_t waypointIndex;
	double latitude;      // 1e-7 degrees
	double longitude;     // 1e-7 degrees
	float velocity;
	uint8_t status;
	uint8_t operationCount;
	uint8_t currentOperationIndex;
	MissionOperationData operations[NAV_MAX_OPERATIONS];
} MissionWPData;

typedef struct {
	MissionWPData waypoints[NAV_MAX_WAYPOINTS];
	uint8_t status;
	uint8_t waypointCount;
	uint8_t currentWaypointIndex;
} MissionData;

typedef void (*MissionCallback)(uint8_t missionCommand);

uint8_t initGroundStation(MissionCallback missionCb);
MissionData* getMissionData(void);
void setMissionWPData(MissionWPData missionWPData);
void clearMissionData(void);

#endif /* SRC_FC_SENSORS_GROUNDSTATION_GROUNDSTATION_H_ */
