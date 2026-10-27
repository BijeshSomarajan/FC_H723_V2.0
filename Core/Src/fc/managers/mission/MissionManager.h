#ifndef SRC_FC_MANAGERS_MISSION_MISSIONMANAGER_H_
#define SRC_FC_MANAGERS_MISSION_MISSIONMANAGER_H_
#include <stdint.h>

#define MISSION_MGR_DATA_DUMP_ENABLED 1

#define MISSION_MGR_SPEED_MIN                    0.5f
#define MISSION_MGR_SPEED_MAX                    20.0f

#define POSITION_MISSION_BRAKE_DECEL                      0.75f  // m/s²
#define POSITION_MISSION_WP_COMPLETE_RADIUS               0.75f  //m
#define POSITION_MISSION_WP_STOP_RADIUS                   (0.5f * POSITION_MISSION_WP_COMPLETE_RADIUS)
#define POSITION_MISSION_WP_COMPLETE_RADIUS_SQ            POSITION_MISSION_WP_COMPLETE_RADIUS * POSITION_MISSION_WP_COMPLETE_RADIUS
#define POSITION_MISSION_MAX_ACCEL                        5.0f
#define POSITION_MISSION_WP_DWELL_TIMEOUT                 0.25f   //Was 1.0f
#define POSITION_MISSION_WP_COMPLETE_MAX_GROUND_SPEED     0.5f
#define POSITION_MISSION_WP_DISTANCE_LPF_ALPHA            0.65f   //LPF applied on the distance.


uint8_t initMissionManager(void);
void startRTHMission(void);
void handleMission(float dt);
void resetMissionStates(uint8_t abort);
void resetRTHStates();

#endif
