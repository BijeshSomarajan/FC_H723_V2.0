#ifndef MODULES_RADIO_FC_RC_H_
#define MODULES_RADIO_FC_RC_H_

#include <stdio.h>
#include <stdint.h>
#define RC_TASK_FREQUENCY 200
#define RC_ACTIVE_CHECK_THRESHOLD_PERIOD  0.2f // Fail safe activated after a second

#define THROTTLE_CENTER_DEADBAND  10
#define THROTTLE_CENTER_ARM_DEADBAND  25

#define YAW_CENTER_DEADBAND  10
#define PITCH_CENTER_DEADBAND  10
#define ROLL_CENTER_DEADBAND  10

#define POS_HOLD_MODE_ACT_TSH  RC_CHANNEL_MID_VALUE * 0.9f
#define RTH_HOLD_MODE_ACT_TSH  RC_CHANNEL_MAX_VALUE * 0.9f
#define LANDING_MODE_ACT_TSH  RC_CHANNEL_MID_VALUE * 0.9f
#define HEADLESS_MODE_ACT_TSH  RC_CHANNEL_MID_VALUE * 0.9f
#define ALT_MODE_MODE_ACT_TSH  RC_CHANNEL_MID_VALUE * 0.9f

uint8_t initRCManager(void);
void doRCManagement();
void resetRCManager(void);
void processRCData(float dt);

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
uint8_t isThrottleCentered(void);
uint8_t isYawCentered(void);
uint8_t isRollCentered(void);
uint8_t isPitchCentered(void);
void applyRCStickEffectiveness();

uint8_t canEnableAltHold(void);
void configureRCStickControl(void);
uint8_t canEnableGlobalPosHold(void);
uint8_t canEnableTerrainPosHold(void);
uint8_t canEnableRTH(void);
uint8_t isGlobalPosHoldModeActive(void);
uint8_t isRTHModeActive(void);
uint8_t isHeadLessModeActive(void);
uint8_t isTerrainAltModeActive(void);
uint8_t canEnableLandingMode();
void checkForFailSafe(float dt);

#endif
