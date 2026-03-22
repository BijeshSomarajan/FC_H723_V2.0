#ifndef SRC_FC_CONTROL_POSITION_POSITIONCONTROL_H_
#define SRC_FC_CONTROL_POSITION_POSITIONCONTROL_H_

#include <sys/_stdint.h>

#define POSITION_CONTROL_D_RATE_LPF_FREQ 32.0f
#define POSITION_CONTROL_RATE_PID_I_LIMIT_RATIO 1.0f
#define POSITION_CONTROL_RATE_PID_D_LIMIT_RATIO 1.0f

uint8_t initPositionControl();
void resetPositionControl(uint8_t hard);
void controlPositionWithGains(float dt, float expectedX, float expectedY, float masterPGain,float ratePGain,float rateIGain,float rateDGain);

#endif /* SRC_FC_CONTROL_POSITION_POSITIONCONTROL_H_ */
