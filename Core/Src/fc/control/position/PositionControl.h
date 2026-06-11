#ifndef SRC_FC_CONTROL_POSITION_POSITIONCONTROL_H_
#define SRC_FC_CONTROL_POSITION_POSITIONCONTROL_H_

#include <sys/_stdint.h>

#define POSITION_CONTROL_D_RATE_LPF_FREQ 15.0f
#define POSITION_CONTROL_RATE_PID_I_LIMIT_RATIO 1.0f
#define POSITION_CONTROL_RATE_PID_D_LIMIT_RATIO 1.0f
#define POSITION_CONTROL_RATE_PID_I_AW_GAIN 0.4f // The antiwindup gain

/*------ FF Configurations -----*/
#define POSITION_CONTROL_VEL_FF_ENABLED 1
#define POSITION_CONTROL_VEL_FF_GAIN  0.125f
/*------ Disturbance Estimations -----*/
#define POSITION_CONTROL_DOB_ENABLED     1
#define POSITION_CONTROL_DOB_ACC_TAU     0.04f//
#define POSITION_CONTROL_DOB_ACC_GAIN    0.55f

#define POSITION_CONTROL_DOB_VEL_TAU     0.065f
#define POSITION_CONTROL_DOB_VEL_GAIN    0.45f

#define POSITION_CONTROL_DOB_ACCEL_MODEL_K 0.171f
#define POSITION_CONTROL_DOB_ACC_LIMIT    3.0f
#define POSITION_CONTROL_DOB_STATE_LIMIT  3.0f
#define POSITION_CONTROL_DOB_OUTPUT_LIMIT 3.0f

uint8_t initPositionControl(float masterControlFrequency, float rateControlFrequency);
void resetPositionControl(uint8_t hard);

void controlPositionRateWithGains(float dt, float ratePGain, float rateIGain, float rateDGain);
void setExpectedPositionVelocity(float dt, float expectedVelX, float expectedVelY);
void controlPositionCordinatesWithGains(float dt, float expectedX, float expectedY, float masterPGain);

#endif /* SRC_FC_CONTROL_POSITION_POSITIONCONTROL_H_ */
