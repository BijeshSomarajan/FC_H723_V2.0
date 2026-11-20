#ifndef SRC_FC_MANAGERS_MISSION_MISSIONNAVIGATIONHELPER_H_
#define SRC_FC_MANAGERS_MISSION_MISSIONNAVIGATIONHELPER_H_

#include <stdint.h>

void updateWpNavCommand(float dt);
uint8_t checkNavCompletion(float dt);
void loadNavCordinates(void);
uint8_t doWPNavigation(float dt);
void missionPositionRelock(void);
uint8_t doRTHOpNavigation(float dt);
void startRTHOpNavigation(void);
uint8_t initMissionNavigationHelper(void);

#define MISSION_COURSE_LOCK_MIN_LEG              3.0f   // m
#define MISSION_COURSE_LOCK_MIN_LEG_SQ           (MISSION_COURSE_LOCK_MIN_LEG * MISSION_COURSE_LOCK_MIN_LEG)

/* Cross-track correction (Kp comes from calibration, 0 = off) */
#define POSITION_MISSION_XTRACK_KP_MAX                    1.0f    // 1/s
#define POSITION_MISSION_XTRACK_VMAX                      1.0f    // m/s
#define POSITION_MISSION_XTRACK_MIN_LEG                   5.0f    // m
#define POSITION_MISSION_XTRACK_PREV_SNAP                 3.0f    // m


#endif
