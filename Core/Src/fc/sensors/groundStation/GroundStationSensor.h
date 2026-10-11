#ifndef SRC_FC_SENSORS_GROUNDSTATION_GROUNDSTATIONSENSOR_H_
#define SRC_FC_SENSORS_GROUNDSTATION_GROUNDSTATIONSENSOR_H_
#include <sys/_stdint.h>
#define NAV_MAX_OPERATIONS 8

typedef enum {
	NAV_ACTION_CLEAR_WAYPOINTS = 1, NAV_ACTION_ADD_WAYPOINT, NAV_ACTION_START_MISSION, NAV_ACTION_ABORT_MISSION
} GroundStationNavAction;

typedef enum {
	NAV_OPERATION_TAKEOFF = 1, NAV_OPERATION_LAND, NAV_OPERATION_SET_ALT, NAV_OPERATION_SET_VEL, NAV_OPERATION_SET_HEADING, NAV_OPERATION_RTH,NAV_OPERATION_HOVER, CUSTOM_OP_1, CUSTOM_OP_2
} GroundStationNavOperation;

typedef struct {
	float value;
	uint8_t operation;
	uint8_t status;
} GroundStationSensorOPData;

typedef enum {
	NAV_STATUS_OPEN = 1, NAV_STATUS_WIP, NAV_STATUS_CLOSED
} GroundStationNavStaus;


typedef struct {
	uint16_t waypointIndex;
	double latitude;      // 1e-7 degrees
	double longitude;     // 1e-7 degrees
	uint8_t status;
	GroundStationSensorOPData operations[NAV_MAX_OPERATIONS];
} GroundStationSensorWPData;

typedef void (*GroundStationMissionCallback)(uint8_t missionCommand);
void resetGroundStationSensor();
uint8_t initGroundStationSensor(GroundStationMissionCallback missionCb);
uint8_t getGroundStationSensorWPDataCount();
GroundStationSensorWPData* getGroundStationSensorWPData(uint8_t index);

void setGroundStationSensorWPData(GroundStationSensorWPData groundStationSensorWPData);
void clearGroundStationSensorWPData(void);

#endif /* SRC_FC_SENSORS_GROUNDSTATION_GROUNDSTATIONSENSOR_H_ */
