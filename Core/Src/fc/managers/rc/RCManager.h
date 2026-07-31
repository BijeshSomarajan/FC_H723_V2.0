#ifndef MODULES_RADIO_FC_RC_H_
#define MODULES_RADIO_FC_RC_H_

#include <stdio.h>
#include <stdint.h>
#define RC_TASK_FREQUENCY 200
#define RC_ACTIVE_CHECK_THRESHOLD_PERIOD  0.2f // Fail safe activated after a second

#define THROTTLE_CENTER_DEADBAND  15
#define THROTTLE_CENTER_ARM_DEADBAND  25

#define YAW_CENTER_DEADBAND  15
#define PITCH_CENTER_DEADBAND  15
#define ROLL_CENTER_DEADBAND  15

#define POS_HOLD_MODE_ACT_TSH  RC_CHANNEL_MID_VALUE * 0.9f
#define RTH_HOLD_MODE_ACT_TSH  RC_CHANNEL_MAX_VALUE * 0.9f
#define MISSION_MODE_ACT_TSH  RC_CHANNEL_MID_VALUE * 0.9f

#define LANDING_MODE_ACT_TSH  RC_CHANNEL_MID_VALUE * 0.9f

#define TERRAIN_ALT_MODE_ACT_TSH  RC_CHANNEL_MID_VALUE * 0.9f
#define TERRAIN_NAV_MODE_ACT_TSH  RC_CHANNEL_MAX_VALUE * 0.9f

uint8_t initRCManager(void);
void doRCManagement();
void resetRCManager(void);
void processRCData(float dt);
void setRCData(int32_t *data, int32_t length);

int16_t getThrottleChannelValue();
int16_t getPitchChannelValue();
int16_t getRollChannelValue();
int16_t getYawChannelValue();

uint8_t canArmModel(void);

//Internal functions
int16_t applyStickDeadBand(int16_t rcChannelValue);
void loadRCStickDelta(void);
uint8_t canStartModel(void);
uint8_t canArmModel(void);
uint8_t checkThrottleCentered(void);
uint8_t checkYawCentered(void);
uint8_t checkRollCentered(void);
uint8_t checkPitchCentered(void);
void applyRCStickEffectiveness();

void configureRCStickControl(void);

uint8_t checkNavModeActivation(void);
uint8_t checkRTHModeActivation(void);
uint8_t checkLandingModeActivation();

uint8_t checkTerrainAltModeActivation(void);
uint8_t checkTerrainNavModeActivation(void);
uint8_t checkMissionModeActivation(void);

#endif
