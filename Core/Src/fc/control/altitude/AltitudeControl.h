#ifndef _ALTITUDECONTROL_H_
#define _ALTITUDECONTROL_H_

#include <stdio.h>
#include <inttypes.h>

typedef struct _ALTITUDE_CONTROL_GAINS ALTITUDE_CONTROL_GAINS;
struct _ALTITUDE_CONTROL_GAINS {
	float masterPGain;
	float ratePGain;
	float rateIGain;
	float rateIBleed;
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

void applyAltitudeControlMPMinLimitToValue(float value);
void applyAltitudeControlRIMinLimitToValue(float value);

void resetAltitudeRateControl(void);
void resetAltitudeMasterControl(void);

void controlAltitudeAltWithGains(float dt, float expectedAltitude, float currentAltitude, ALTITUDE_CONTROL_GAINS altControlGains);
void controlAltitudeVelWithGains(float dt, ALTITUDE_CONTROL_GAINS altControlGains);
void controlAltitudeAccWithGains(float dt, ALTITUDE_CONTROL_GAINS altControlGains);

#define ALT_CONTROL_RATE_PID_D_LPF_FREQ 32.0f
#define ALT_CONTROL_ACC_PID_D_LPF_FREQ  32.0f

#define ALT_CONTROL_RATE_PID_I_LIMIT_RATIO 1.0f
#define ALT_CONTROL_RATE_PID_D_LIMIT_RATIO 1.0f
#define ALT_CONTROL_ACC_PID_D_LIMIT_RATIO 1.0f

#define ALT_CONTROL_VEL_FEED_FWD_ENABLED       1
#define ALT_CONTROL_VEL_FEED_FWD_GAIN          10.0f // 2.5*10 25 TU.

#define ALT_CONTROL_ACC_DISTURBANCE_EST_ENABLED  1
#define ALT_CONTROL_ACC_DISTURBANCE_TAU        0.065f //Lower values make the estimate more responsive but can introduce noise
#define ALT_CONTROL_ACC_DISTURBANCE_FF_GAIN    1.0f

#endif
