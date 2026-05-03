#ifndef SRC_FC_CONTROL_POSITION_POSITIONCONTROL_H_
#define SRC_FC_CONTROL_POSITION_POSITIONCONTROL_H_

#include <sys/_stdint.h>

#define POSITION_CONTROL_D_RATE_LPF_FREQ 40.0f
#define POSITION_CONTROL_RATE_PID_I_LIMIT_RATIO 1.0f
#define POSITION_CONTROL_RATE_PID_D_LIMIT_RATIO 1.0f
#define POSITION_CONTROL_RATE_VEL_MAX 10.0f
#define POSITION_CONTROL_USE_VEL_INTERPOLATION 0


uint8_t initPositionControl(float masterControlFrequency, float rateControlFrequency);
void resetPositionControl(uint8_t hard);
void controlPositionWithGains(float dt, float expectedX, float expectedY, float masterPGain, float ratePGain, float rateIGain, float rateDGain);
void controlPositionRateWithGains(float dt, float ratePGain, float rateIGain, float rateDGain);
void controlPositionCordinatesWithGains(float dt, float expectedX, float expectedY, float masterPGain);

#endif /* SRC_FC_CONTROL_POSITION_POSITIONCONTROL_H_ */
