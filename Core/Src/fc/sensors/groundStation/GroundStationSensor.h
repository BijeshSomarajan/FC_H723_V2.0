#ifndef SRC_FC_SENSORS_GROUNDSTATION_GROUNDSTATIONSENSOR_H_
#define SRC_FC_SENSORS_GROUNDSTATION_GROUNDSTATIONSENSOR_H_
#include <sys/_stdint.h>

typedef enum {
	NAV_ACTION_CLEAR_WAYPOINTS = 1, ACTION_ADD_WAYPOINT, NAV_ACTION_START_MISSION, NAV_ACTION_ABORT_MISSION
} GroundStationNavAction;

typedef struct {
	uint16_t waypointIndex;
	double latitude;      // 1e-7 degrees
	double longitude;     // 1e-7 degrees
} GroundStationSensorWPData;

typedef void (*GroundStationMissionCallback)(uint8_t missionCommand);
void resetGroundStationSensor();
uint8_t initGroundStationSensor(GroundStationMissionCallback missionCb);
uint8_t getGroundStationSensorWPDataCount();
GroundStationSensorWPData* getGroundStationSensorWPData(uint8_t index);

void setGroundStationSensorWPData(GroundStationSensorWPData groundStationSensorWPData);
void clearGroundStationSensorWPData(void);

#endif /* SRC_FC_SENSORS_GROUNDSTATION_GROUNDSTATIONSENSOR_H_ */
