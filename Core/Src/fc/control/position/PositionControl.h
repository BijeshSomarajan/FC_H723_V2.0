#ifndef SRC_FC_CONTROL_POSITION_POSITIONCONTROL_H_
#define SRC_FC_CONTROL_POSITION_POSITIONCONTROL_H_

#include <sys/_stdint.h>

#define POSITION_CONTROL_D_RATE_LPF_FREQ 40.0f
#define POSITION_CONTROL_RATE_PID_I_LIMIT_RATIO 1.0f
#define POSITION_CONTROL_RATE_PID_D_LIMIT_RATIO 1.0f

#define POSITION_CONTROL_NONLINEAR_BOOST_ENABLED  0
#define POSITION_CONTROL_NONLINEAR_BOOST_START 0.15f
#define POSITION_CONTROL_NONLINEAR_BOOST_GAIN     0.15f
#define POSITION_CONTROL_NONLINEAR_BOOST_DEADBAND 0.02f

/*------ FF Configurations -----*/
#define POSITION_CONTROL_VEL_FEED_FWD_ENABLED 1
#define POSITION_CONTROL_VEL_FEED_FWD_GAIN  0.1f
/*------ Disturbance Estimations -----*/
#define POSITION_CONTROL_DIST_EST_ENABLED   0
#define POSITION_CONTROL_DIST_EST_ACC_TAU   0.4f//0.5f
#define POSITION_CONTROL_DIST_EST_VEL_TAU   1.8f
#define POSITION_CONTROL_DIST_EST_ACC_GAIN  0.2f //0.15f
#define POSITION_CONTROL_DIST_EST_VEL_GAIN  0.07f//0.10f

#define POSITION_CONTROL_DIST_EST_ACC_LIMIT   5.0f
#define POSITION_CONTROL_DIST_EST_STATE_LIMIT  5.0f

//For radians
//#define POSITION_CONTROL_DIST_EST_ACCEL_MODEL_K        9.81f
//#define POSITION_CONTROL_DIST_EST_TOTAL_OUTPUT_LIMIT   0.174f
//For degrees
#define POSITION_CONTROL_DIST_EST_ACCEL_MODEL_K        0.1712f
#define POSITION_CONTROL_DIST_EST_TOTAL_OUTPUT_LIMIT   15.0f //10.0f

uint8_t initPositionControl(float masterControlFrequency, float rateControlFrequency);
void resetPositionControl(uint8_t hard);

void controlPositionRateWithGains(float dt, float ratePGain, float rateIGain, float rateDGain);
void setExpectedPositionVelocity(float dt, float expectedVelX, float expectedVelY);
void controlPositionCordinatesWithGains(float dt, float expectedX, float expectedY, float masterPGain);

#endif /* SRC_FC_CONTROL_POSITION_POSITIONCONTROL_H_ */
