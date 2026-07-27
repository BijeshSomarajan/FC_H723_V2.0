#ifndef SRC_FC_CONTROL_POSITION_POSITIONCONTROL_H_
#define SRC_FC_CONTROL_POSITION_POSITIONCONTROL_H_

#include <sys/_stdint.h>

#define POSITION_CONTROL_D_RATE_LPF_FREQ 24.0f//32.0f //was 12.0f
#define POSITION_CONTROL_RATE_PID_I_LIMIT_RATIO 1.0f
#define POSITION_CONTROL_RATE_PID_D_LIMIT_RATIO 0.5f //was 1.0f
#define POSITION_CONTROL_RATE_PID_I_ANTIWINDUP_GAIN  0.2f //0.4f // The antiwindup gain

/*------ FF Configurations -----*/
#define POSITION_CONTROL_VEL_FF_ENABLED 1
#define POSITION_CONTROL_VEL_FF_GAIN    0.15f //0.05f // was 0.0125f
/*------ Disturbance Estimations -----*/
#define POSITION_CONTROL_DOB_ENABLED     1
#define POSITION_CONTROL_DOB_ACC_TAU     0.04f
#define POSITION_CONTROL_DOB_ACC_GAIN    0.30f     // was 0.55f
// Model the attitude loop lag so the acc-DOB stops fighting its own commands
#define POSITION_CONTROL_DOB_ATT_TAU     0.15f

#define POSITION_CONTROL_DOB_VEL_TAU     0.065f
#define POSITION_CONTROL_DOB_VEL_GAIN    0.0f //Lagged rate-P   // was 0.25f

#define POSITION_CONTROL_DOB_ACCEL_MODEL_K 0.171f  //Geometric constant do not change.
#define POSITION_CONTROL_DOB_ACC_LIMIT    3.0f
#define POSITION_CONTROL_DOB_STATE_LIMIT  3.0f
#define POSITION_CONTROL_DOB_OUTPUT_LIMIT 3.0f // was 3.0f

/*------ Acceleration feedforward -----*/
#define POSITION_CONTROL_ACCEL_FF_ENABLED   1
/* Trim on the physical FF. 1.0 = feed forward exactly the tilt the DOB model
 * says produces the commanded accel (control = accel / DOB_ACCEL_MODEL_K).
 * Start at 1.0; lower only if the aircraft leads the brake (stops short). */
#define POSITION_CONTROL_ACCEL_FF_GAIN      1.0f

uint8_t initPositionControl(float masterControlFrequency, float rateControlFrequency);
void resetPositionControl(uint8_t hard);

void controlPositionRateWithGains(float dt, float ratePGain, float rateIGain, float rateDGain , float accFFX,float accFFY) ;
void setExpectedPositionVelocity(float dt, float expectedVelX, float expectedVelY);
void controlPositionCordinatesWithGains(float dt, float expectedX, float expectedY, float masterPGain);

#endif /* SRC_FC_CONTROL_POSITION_POSITIONCONTROL_H_ */
