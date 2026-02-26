#ifndef _ALTITUDECONTROL_H_
#define _ALTITUDECONTROL_H_

#include <stdio.h>
#include <inttypes.h>

typedef struct _ALTITUDE_CONTROL_GAINS ALTITUDE_CONTROL_GAINS;
struct _ALTITUDE_CONTROL_GAINS {
	float masterPGain;
	float ratePGain;
	float rateIGain;
	float rateDGain;
	float accPGain;
	float accDGain;

};

uint8_t initAltitudeControl(void);
void resetAltitudeControl(uint8_t hard);
void resetAltitudeControlMaster(void);
void resetAltitudeControlRate(void);

void resetAltitudeRIControl(void);
void setAltitudeRIControl(float value);
void suspendAltitudeRIControl(void);
void resumeAltitudeRIControl(void);

float getAltitudeControlMPValue(void);
float getAltitudeControlRIValue(void);

void resetAltitudeControlMPLimits(void);
void resetAltitudeControlRILimits(void);

void applyAltitudeControlMPMinLimitToValue(float value);
void applyAltitudeControlRIMinLimitToValue(float value);

void resetAltitudeRateControl(void);
void resetAltitudeMasterControl(void);
void controlAltitude(float dt, float expectedAltitude, float currentAltitude);
void controlAltitudeWithGains(float dt, float expectedAltitude, float currentAltitude, ALTITUDE_CONTROL_GAINS altControlGains);

#define ALT_CONTROL_RATE_PID_D_LPF_FREQ 32.0f
#define ALT_CONTROL_ACC_PID_D_LPF_FREQ  32.0f

#define ALT_CONTROL_RATE_PID_I_LIMIT_RATIO 0.8f
#define ALT_CONTROL_RATE_PID_D_LIMIT_RATIO 1.0f
#define ALT_CONTROL_ACC_PID_D_LIMIT_RATIO 1.0f

#endif
