#ifndef SRC_FC_MANAGERS_POSITION_ESTIMATOR_POSITIONESTIMATORHELPER_H_
#define SRC_FC_MANAGERS_POSITION_ESTIMATOR_POSITIONESTIMATORHELPER_H_
#include "PositionEstimator.h"

#define POSITION_MGR_Z_ENABLE_DYNAMIC_R                1
#define POSITION_MGR_VENTURI_ESTIMATE_ENABLED          1

void updateXYPosition(float hAcc, float xPos, float yPos, float dt);
void updateZPositionSL(float zPos, float dt);
void updateZPositionGNSS(float vAcc, float hMSL, float dt);

void updateXYVelocity(float sAcc, float velN, float velE,float dt);
void updateZVelocity(float sAcc, float velD,float dt);

#endif /* SRC_FC_MANAGERS_POSITION_ESTIMATOR_POSITIONESTIMATORHELPER_H_ */
