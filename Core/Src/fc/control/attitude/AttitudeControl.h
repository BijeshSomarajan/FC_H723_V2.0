#ifndef MODULES_ATTITUDE_CONTROL_H_
#define MODULES_ATTITUDE_CONTROL_H_

#include <sys/_stdint.h>

#define ATT_CONTROL_D_RATE_LPF_FREQ 40.0f

#define ATT_CONTROL_RATE_PID_PITCH_ROLL_I_LIMIT_RATIO 0.75f
#define ATT_CONTROL_RATE_PID_YAW_I_LIMIT_RATIO 0.75f

#define ATT_CONTROL_RATE_PID_PITCH_ROLL_D_LIMIT_RATIO 0.5f
#define ATT_CONTROL_RATE_PID_YAW_D_LIMIT_RATIO 0.0f

uint8_t initAttitudeControl();
void resetAttitudeControl(uint8_t hard);

void controlAttitudeAngle(float dt, float expectedPitch, float expectedRoll, float expectedYaw);
void controlAttitudeRateWithGains(float dt,float ratePGain,float rateIGain, float rateDGain);

#endif
