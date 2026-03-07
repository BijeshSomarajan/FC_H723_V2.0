#include "../sensors/attitude/AttitudeSensor.h"
#include "../util/MathUtil.h"
#include "IMU.h"

#include "../dsp/LeakyIntegrationFlilter.h"
#include "../dsp/LowPassFilter.h"
#include "MadgwickFilter.h"
#include "MahonyFilter.h"

IMU_DATA imuData;
extern SENSOR_ATTITUDE_DATA sensorAttitudeData;

float imuMagInclination = 0;

uint16_t getImuStabilizationCount() {
	return imuFilterGetStabilizationCount();
}

/*****************************************************************************************************************/
// Initializes imuData.
/*****************************************************************************************************************/
uint8_t imuInit(float pMagInclination) {
	imuReset(1);
	imuMagInclination = pMagInclination;
	imuFilterInit(1);
	return 1;
}

void imuSetMode(uint8_t stabilize) {
	imuFilterSetMode(stabilize);
}

__ATTR_ITCM_TEXT
void imuUpdateRate() {
	// Flush the rates, Note: X and Y gyros are interchanged
	imuData.pitchRate = sensorAttitudeData.gxDSFiltered;
	imuData.rollRate = sensorAttitudeData.gyDSFiltered;
	imuData.yawRate = sensorAttitudeData.gzDSFiltered;
}

/*
 __ATTR_ITCM_TEXT
 void updateLinearMovements(float dt) {
 // Rotate body accelerations → earth frame
 float axGEarth = imuData.rMatrix[0][0] * sensorAttitudeData.axGFiltered + imuData.rMatrix[0][1] * sensorAttitudeData.ayGFiltered + imuData.rMatrix[0][2] * sensorAttitudeData.azGFiltered;
 float ayGEarth = imuData.rMatrix[1][0] * sensorAttitudeData.axGFiltered + imuData.rMatrix[1][1] * sensorAttitudeData.ayGFiltered + imuData.rMatrix[1][2] * sensorAttitudeData.azGFiltered;
 float azGEarth = imuData.rMatrix[2][0] * sensorAttitudeData.axGFiltered + imuData.rMatrix[2][1] * sensorAttitudeData.ayGFiltered + imuData.rMatrix[2][2] * sensorAttitudeData.azGFiltered;
 azGEarth -= 1.0f;
 imuData.axEarthLinear = axGEarth * GRAVITY_MSS;
 imuData.ayEarthLinear = ayGEarth * GRAVITY_MSS;
 imuData.azEarthLinear = azGEarth * GRAVITY_MSS;
 }
 */

__ATTR_ITCM_TEXT
void updateLinearMovements(float dt) {
	// 1. Existing Earth Frame Calculation
	float axGEarth = imuData.rMatrix[0][0] * sensorAttitudeData.axGFiltered + imuData.rMatrix[0][1] * sensorAttitudeData.ayGFiltered + imuData.rMatrix[0][2] * sensorAttitudeData.azGFiltered;
	float ayGEarth = imuData.rMatrix[1][0] * sensorAttitudeData.axGFiltered + imuData.rMatrix[1][1] * sensorAttitudeData.ayGFiltered + imuData.rMatrix[1][2] * sensorAttitudeData.azGFiltered;
	float azGEarth = imuData.rMatrix[2][0] * sensorAttitudeData.axGFiltered + imuData.rMatrix[2][1] * sensorAttitudeData.ayGFiltered + imuData.rMatrix[2][2] * sensorAttitudeData.azGFiltered;
	// Remove gravity (1.0G) in Earth frame
	azGEarth -= 1.0f;

	imuData.axEarthLinear = axGEarth * GRAVITY_MSS;
	imuData.ayEarthLinear = ayGEarth * GRAVITY_MSS;
	imuData.azEarthLinear = azGEarth * GRAVITY_MSS;

	// 2. Body Frame Linear Acceleration (Gravity Compensation)
	// We project the Earth gravity vector [0, 0, 1] into the Body frame.
	// In a standard Rotation Matrix, the third row represents the Earth-Z
	// components as seen by the Body axes.
	float gxBody = imuData.rMatrix[2][0]; // Gravity component on Body X
	float gyBody = imuData.rMatrix[2][1]; // Gravity component on Body Y
	float gzBody = imuData.rMatrix[2][2]; // Gravity component on Body Z

	// Subtract gravity components from raw filtered sensors to get pure linear motion
	imuData.axBodyLinear = (sensorAttitudeData.axGFiltered - gxBody) * GRAVITY_MSS;
	imuData.ayBodyLinear = (sensorAttitudeData.ayGFiltered - gyBody) * GRAVITY_MSS;
	imuData.azBodyLinear = (sensorAttitudeData.azGFiltered - gzBody) * GRAVITY_MSS;
}

/*************************************************************************/
// Does imuData fusion , returns 1 if done
/*************************************************************************/
__ATTR_ITCM_TEXT
void imuAHRSUpdate(float dt) {
	imuFilterUpdate(dt);
	imuFilterUpdateAngles();
	imuFilterUpdateHeading(imuMagInclination, MAG_ANGLE_CORRECTION);
	updateLinearMovements(dt);
	imuData.arhsDt = dt;
}

/****************************************************************************************************************/
// Resets Madgwick filter.
/****************************************************************************************************************/
void imuReset(uint8_t hard) {
	if (hard) {
		// Reset the quaternion
		imuData.q0 = 1.0f;
		imuData.q1 = 0.0f;
		imuData.q2 = 0.0f;
		imuData.q3 = 0.0f;

		// Reset the rotation matrix
		imuData.rMatrix[0][0] = 0;
		imuData.rMatrix[0][1] = 0;
		imuData.rMatrix[0][2] = 0;

		imuData.rMatrix[1][0] = 0;
		imuData.rMatrix[1][1] = 0;
		imuData.rMatrix[1][2] = 0;

		imuData.rMatrix[2][0] = 0;
		imuData.rMatrix[2][1] = 0;
		imuData.rMatrix[2][2] = 0;

		// Reset the euler angles
		imuData.pitch = 0;
		imuData.roll = 0;
		imuData.yaw = 0;
		imuData.heading = 0;

		// Reset the rates
		imuData.pitchRate = 0;
		imuData.rollRate = 0;
		imuData.yawRate = 0;

		// Reset the specific filter
		imuFilterReset();
	}

	imuData.axEarthLinear = 0;
	imuData.ayEarthLinear = 0;
	imuData.azEarthLinear = 0;

	imuData.axBodyLinear = 0;
	imuData.ayBodyLinear = 0;
	imuData.azBodyLinear = 0;

}
