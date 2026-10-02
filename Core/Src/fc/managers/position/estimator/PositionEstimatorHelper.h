#ifndef SRC_FC_MANAGERS_POSITION_ESTIMATOR_POSITIONESTIMATORHELPER_H_
#define SRC_FC_MANAGERS_POSITION_ESTIMATOR_POSITIONESTIMATORHELPER_H_
#include "../../position/estimator/PositionEstimator.h"

#define POSITION_MGR_VENTURI_ESTIMATE_ENABLED         1

void resetPVEstimation(uint8_t axis, uint8_t keepBias);
void updateXYPositionGNSS(float hAcc, float xPos, float yPos, float dt);
void updateZPositionSL(float offset, float zPos, float dt);
void updateZPositionGNSS(float vAcc, float hMSL, float dt);
void updateZPositionTerrain(float offset, float distance, float strength, float minDistance, float maxDistance, float dt);
void updateXYVelocityGNSS(float sAcc, float velN, float velE, float dt);
void updateZVelocityGNSS(float sAcc, float velD, float dt);

float getGroundSpeed(void);

#endif /* SRC_FC_MANAGERS_POSITION_ESTIMATOR_POSITIONESTIMATORHELPER_H_ */
