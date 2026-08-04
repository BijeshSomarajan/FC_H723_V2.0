#ifndef SRC_FC_MANAGERS_POSITION_HELPERS_POSITIONMISSIONHELPER_H_
#define SRC_FC_MANAGERS_POSITION_HELPERS_POSITIONMISSIONHELPER_H_

#include "../common/PositionCommon.h"

// =============================================================================
// MISSION NAVIGATION PROFILE
// =============================================================================
#define POSITION_MISSION_CRUISE_SPEED                     1.5f   // 1.0f //Note this will be clamped by the Postion PID settings
#define POSITION_MISSION_BRAKE_DECEL                      1.5f   // m/s²
#define POSITION_MISSION_WP_NEAR_RADIUS                   1.5f
#define POSITION_MISSION_WP_COMPLETE_RADIUS               0.6f
#define POSITION_MISSION_MAX_ACCEL                        5.0f
#define POSITION_MISSION_WP_SETTLING_PERIOD               0.5f
#define POSITION_MISSION_WP_COMPLETE_PERIOD               1.0f
#define POSITION_MISSION_WP_COMPLETE_MAX_GROUND_SPEED     0.4f

void handleNavMission(float dt);
void resetNavMissionStates(void);
void resetNavRTHStates(void);
void resetNavMissionModeStates(void);
void resetNavWPStates(void);

#endif /* SRC_FC_MANAGERS_POSITION_HELPERS_POSITIONMISSIONHELPER_H_ */
