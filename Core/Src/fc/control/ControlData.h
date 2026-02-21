#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <sys/_stdint.h>

typedef struct _CONTROL_DATA CONTROL_DATA;
struct _CONTROL_DATA {
	float throttleControl;
	float pitchControl;
	float rollControl;
	float yawControl;
	float altitudeControl;
	float positionPitchControl;
	float positionRollControl;
	float attitudeControlDt;
	float altitudeControlDt;
	float tiltCompThDelta;
};



extern CONTROL_DATA controlData;

#endif
