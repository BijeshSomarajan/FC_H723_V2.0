#ifndef SRC_FC_MANAGERS_MISSION_MISSIONNAVIGATIONHELPER_H_
#define SRC_FC_MANAGERS_MISSION_MISSIONNAVIGATIONHELPER_H_

#include <stdint.h>

void updateNavCommand(float dt);
uint8_t checkNavCompletion(float dt);
void loadNavCordinates(void);
uint8_t doWPNavigation(float dt);
void missionPositionRelock(void);

#endif
