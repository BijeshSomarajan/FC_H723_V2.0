#ifndef SRC_FC_MANAGERS_POSITION_HELPERS_POSITIONMISSIONHELPER_H_
#define SRC_FC_MANAGERS_POSITION_HELPERS_POSITIONMISSIONHELPER_H_

#include "../common/PositionCommon.h"

// =============================================================================
// MISSION NAVIGATION PROFILE
// =============================================================================
/*
 * 0 - Velocity command tapers at 'NEAR_RADUIS'
 * 1 - Expected vel is set to 0 at  'CAPTURE_RADIUS' - More agressive breaking
 * 2 - Leanear ramp down.
 */
#define POSITION_MISSION_WP_PROFILE 1 //1-Works with reduced deceleration

#define POSITION_MISSION_CRUISE_SPEED_DEFAULT              0.5f   // 1.0f //Note this will be clamped by the Postion PID settings
#define POSITION_MISSION_CRUISE_SPEED_MAX                 20.0f   // 1.0f //Note this will be clamped by the Postion PID settings

#if POSITION_MISSION_WP_PROFILE == 1
#define POSITION_MISSION_BRAKE_DECEL                      0.75f    // m/s²
#else
#define POSITION_MISSION_BRAKE_DECEL                      1.5f    // m/s²
#endif

#define POSITION_MISSION_WP_NEAR_RADIUS                   1.6f
#define POSITION_MISSION_WP_CAPTURE_RADIUS                1.2f    //Was 0.8f
#define POSITION_MISSION_WP_COMPLETE_RADIUS               0.6f
#define POSITION_MISSION_MAX_ACCEL                        5.0f
#define POSITION_MISSION_WP_COMPLETE_PERIOD               0.5f   //Was 1.0f
#define POSITION_MISSION_WP_COMPLETE_MAX_GROUND_SPEED     0.4f


void initPositionMissionHelper(void);
void handleNavMission(float dt);
void resetNavMissionStates(void);
void resetNavRTHStates(void);
void resetNavMissionModeStates(void);
void resetNavWPStates(void);

#endif /* SRC_FC_MANAGERS_POSITION_HELPERS_POSITIONMISSIONHELPER_H_ */
