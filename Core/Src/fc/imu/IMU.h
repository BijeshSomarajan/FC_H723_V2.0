#ifndef IMU_IMU_H_
#define IMU_IMU_H_

#include <stdint.h>
#include <math.h>

#include "../memory/Memory.h"
#include "../util/MathUtil.h"

#define MAG_INCLINATION -1.1f 
#define MAG_ANGLE_CORRECTION -10.0f 

#define IMU_FILTER_MANHONY_BF 1
#define IMU_FILTER_MADGWICK 2

#define IMU_FILTER_SELECTED IMU_FILTER_MANHONY_BF   

/**
 * Structure representing IMU data
 */
typedef struct _IMU_DATA IMU_DATA;
struct _IMU_DATA {
	//Quaternions w, x ,y ,z
	float q0, q1, q2, q3; //z
	//Rotation matrix
	float rMatrix[3][3];
	//Euler angles
	float pitch, roll, yaw;
	//Azimuth NED heading
	float heading;
	//Motion rates
	float pitchRate, rollRate, yawRate;
	//Earth frame accelerations
	float axEarth, ayEarth, azEarth;
	float arhsDt;
};

extern IMU_DATA imuData;

uint16_t getImuStabilizationCount(void);
uint8_t imuInit(float magIncl);
void imuSetMode(uint8_t stabilize);
void imuReset(uint8_t hard);
void imuAHRSUpdate(float dt);
void imuUpdateRate(void);
void imuUpdateLinearVelocity(float dt);

#endif
