#ifndef FC_STATUS_H_
#define FC_STATUS_H_

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

typedef struct _FC_STATUS_DATA FC_STATUS_DATA;
struct _FC_STATUS_DATA {
	//Enabled states
	uint8_t enableAltitudeHold, enablePositionHold, enableRTH;
	//FC Statues
	uint8_t hasInitialized, isTxOn, canStart, canArm, canFly, hasCrashed, canStabilize, isStabilized , isFlying;
	//FC Modes
	uint8_t isGlobalPosHoldModeActive, isRTHModeActive, isTerrainAltModeActive, isHeadLessModeActive;
	//Flag to state if landing landing mode is active
	uint8_t isLandingModeActive, isLandingModeActiveAfterRTH, isFailSafeLandingMode;
	//Flight debug status enabled
	uint8_t isDebugEnabled;
	//The position references
	double positionXRef, positionYRef;
	//The home position
	double positionXHome, positionYHome;
	uint8_t isPositionHomeSet;
	//Flight reference values
	float headingRef, headingHomeRef, headingDelta;

	float altitudeSLHome;
	float altitudeSLRef;
	float altitudeSLMax;
	float altitudeGndMax;

	//Throttle reference values
	float currentThrottle;
	float throttlePercentage;
	float liftOffThrottlePercent;

	//The config mode
	uint8_t isConfigMode;
};

extern FC_STATUS_DATA fcStatusData;

#endif
