#ifndef MODULES_ATTITUDE_CONTROL_H_
#define MODULES_ATTITUDE_CONTROL_H_

#include <sys/_stdint.h>

#define ATT_CONTROL_D_RATE_LPF_FREQ 40.0f

#define ATT_CONTROL_RATE_PID_I_LIMIT_RATIO 1.0f
#define ATT_CONTROL_RATE_PID_D_LIMIT_RATIO 1.0f

uint8_t initAttitudeControl();
void resetAttitudeControl(uint8_t hard);

void controlAttitudeAngle(float dt, float expectedPitch, float expectedRoll, float expectedYaw);
void controlAttitudeRateWithGains(float dt,float ratePGain,float rateIGain, float rateDGain);

#endif
