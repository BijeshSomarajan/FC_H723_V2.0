#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <sys/_stdint.h>

typedef struct _CONTROL_DATA CONTROL_DATA;
struct _CONTROL_DATA {
	float throttleControl;
	float throttleControlBase;

	float pitchControl;
	float rollControl;
	float yawControl;

	float altitudeControl;
	float altitudeDOBControl;

	float positionXControl;
	float positionYControl;

	float previousEffectiveXControl;
	float previousEffectiveYControl;

	float attitudeControlRateDt;
	float attitudeControlAngleDt;

	float altitudeControlDt;
	float tiltCompThDelta;

	float batteryDepletionGain;

};



extern CONTROL_DATA controlData;

#endif
