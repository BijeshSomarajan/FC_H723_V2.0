#ifndef MODULES_ATTITUDE_CONTROL_H_
#define MODULES_ATTITUDE_CONTROL_H_

#include <sys/_stdint.h>

#define ATT_CONTROL_D_RATE_LPF_FREQ 32.0f
#define ATT_CONTROL_RATE_PID_I_LIMIT_RATIO 1.0f
#define ATT_CONTROL_RATE_PID_D_LIMIT_RATIO 1.0f


uint8_t initAttitudeControl();
void resetAttitudeControl(uint8_t hard);
void controlAttitudeWithGains(float dt, float expectedPitch, float expectedRoll, float expectedYaw, float rateIGain,float rateDGain);
void controlAttitude(float dt, float expectedPitch, float expectedRoll, float expectedYaw);

#endif
