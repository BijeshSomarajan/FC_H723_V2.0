#ifndef SRC_FC_MANAGERS_POSITION_ESTIMATOR_POSITIONESTIMATORHELPER_H_
#define SRC_FC_MANAGERS_POSITION_ESTIMATOR_POSITIONESTIMATORHELPER_H_
#include "PositionEstimator.h"

#define POSITION_MGR_Z_ENABLE_DYNAMIC_R                1
#define POSITION_MGR_VENTURI_ESTIMATE_ENABLED          1

void resetPVEstimation(uint8_t axis, uint8_t keepBias);
void updateXYPositionGNSS(float hAcc, float xPos, float yPos, float dt);
void updateZPositionSL(float offset, float zPos, float dt);
void updateZPositionGNSS(float vAcc, float hMSL, uint8_t navigationModeActive, float dt);
void updateZPositionTerrain(float offset, float distance, float strength, float minDistance, float maxDistance, uint8_t terrainDataValid, float dt);

void updateXYVelocityGNSS(float sAcc, float velN, float velE, float dt);
void updateXYVelocityOFlow(float flowPitchRad, float flowRollRad, float qual, float terrainAltitude, float minDistance, float maxDistance, uint8_t terrainAltValid, uint8_t terrainNavValid, float pitchRate, float rollRate, float heading, float dt);
void updateZVelocityGNSS(float sAcc, float velD, uint8_t navigationModeActive, float dt);

float getGroundSpeed(void);

#endif /* SRC_FC_MANAGERS_POSITION_ESTIMATOR_POSITIONESTIMATORHELPER_H_ */
