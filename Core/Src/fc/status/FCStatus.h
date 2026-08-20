#ifndef FC_STATUS_H_
#define FC_STATUS_H_

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

typedef struct _FC_STATUS_DATA FC_STATUS_DATA;
struct _FC_STATUS_DATA {
	//FC Statues
	uint8_t hasInitialized, isTxOn, canStart, canArm, canFly, hasCrashed, canStabilize, isStabilized, isFlying, isNavDataReliable , isTerrainAltDataReliable;
	//FC Modes
	uint8_t isNavModeActive, isNavRTHModeActive,isNavMissionModeActive, isTerrainAltModeActive;
	//Flag to state if landing landing mode is active
	uint8_t isLandingModeActive, isFailSafeLandingMode;
	//Flight debug status enabled
	uint8_t isDebugEnabled;
	uint8_t isTelemetryEnabled;
	//The position references
	double positionXRef, positionYRef;
	//The mission position references
	double positionXRefMission, positionYRefMission;
	//The home position
	double positionLatHome, positionLongHome;
	float positionXHome, positionYHome;
	float positionZHome;

	uint8_t isPositionHomeSet;
	uint8_t postionHoldState;

	uint8_t isNavMissionComplete;

	//Flight reference values
	float headingRef, headingHomeRef, headingDelta;
	float altitudeSLHome;
	float altitudeRef;

	//Throttle reference values
	float currentThrottle;
	float throttlePercent;
	float throttleControlPercent;
	float liftOffThrottlePercent;
	// Learned hover throttle in throttle units.
	float hoverThrottle;

	//The config mode
	uint8_t isConfigMode;
	float batteryNomVolt;
	uint8_t batteryType; //0-Lipo , 1-lion
	uint8_t batteryAlertState ; //None=0 , Low=1, Crit=2

	//Sensor Statuses.
	uint8_t isTerrainSensorExist;
	uint8_t isGNSSSensorAvailable;
	uint8_t modelVersion;
};

extern FC_STATUS_DATA fcStatusData;

#endif
