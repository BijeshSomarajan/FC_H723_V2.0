#ifndef SRC_FC_SENSORS_RC_DEVICES_CRSF_CRSFGCS_H_
#define SRC_FC_SENSORS_RC_DEVICES_CRSF_CRSFGCS_H_

#include <sys/_stdint.h>
void manageCRSFMissionFrame(const uint8_t *payload);

#define CRSF_MISSION_FRAME_TYPE  0x7C
#define CRSF_MISSION_MAX_OPERATIONS 8

typedef struct {
	uint8_t action;
	uint8_t payload[];
} CRSFMissionCommand;

typedef enum {
	CRSF_MISSION_ACTION_CLEAR_WAYPOINTS = 1,
	CRSF_MISSION_ACTION_ADD_WAYPOINT,
	CRSF_MISSION_ACTION_START,
	CRSF_MISSION_ACTION_ABORT
} CRSFMissionAction;

typedef struct {
	int32_t value;
	uint8_t operation;
} CRSFMissionOperation;

typedef struct {
	uint16_t waypointIndex;
	int32_t latitude;      // 1e-7 degrees
	int32_t longitude;     // 1e-7 degrees
	uint16_t velocity;     // 1e-1 m/s
	CRSFMissionOperation operations[CRSF_MISSION_MAX_OPERATIONS];
} CRSFMissionWP;

typedef void (*CRSFMissionCallback)(uint8_t missionCommand);
typedef void (*CRSFMissionWPCallback)(uint8_t wpCommand,CRSFMissionWP wp);

void registerCRSFMissionCallBack(CRSFMissionCallback callback);
void registerCRSFMissionWPCallBack(CRSFMissionWPCallback callback);

#endif
