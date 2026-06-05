#ifndef SRC_FC_MANAGERS_POSITION_ESTIMATOR_POSITIONESTIMATORHELPER_H_
#define SRC_FC_MANAGERS_POSITION_ESTIMATOR_POSITIONESTIMATORHELPER_H_
#include "PositionEstimator.h"

/* Dynamic Position Scaling XY Axis*/
#define POS_ESTIMATOR_DYNAMIC_XY_POS_HACC_SCALE        2.0f//1.2f
#define POS_ESTIMATOR_DYNAMIC_XY_POS_HACC_MIN          0.1f//0.6f
#define POS_ESTIMATOR_DYNAMIC_XY_RP_BASE               POS_EKF_X_R_MEAS
#define POS_ESTIMATOR_DYNAMIC_XY_RP_MAX                (POS_EKF_X_R_MEAS * 50.0f)
/* Dynamic Velocity Scaling */
#define POS_ESTIMATOR_DYNAMIC_XY_VEL_SACC_SCALE        1.0f
#define POS_ESTIMATOR_DYNAMIC_XY_VEL_SACC_MIN          0.05f
#define POS_ESTIMATOR_DYNAMIC_XY_VEL_DEADBAND          0.0f
#define POS_ESTIMATOR_DYNAMIC_XY_RV_BASE               0.02f//0.03f
#define POS_ESTIMATOR_DYNAMIC_XY_RV_RESET              0.5f //0.15f
#define POS_ESTIMATOR_DYNAMIC_XY_RV_MAX                (POS_ESTIMATOR_DYNAMIC_XY_RV_BASE * 300.0f)

/* Dynamic scaling Z */
#define POS_ESTIMATOR_DYNAMIC_Z_RP_GAIN              0.05f
#define POS_ESTIMATOR_DYNAMIC_Z_RP_ALPHA             0.15f
#define POS_ESTIMATOR_DYNAMIC_Z_RP_MIN               POS_EKF_Z_R_MEAS
#define POS_ESTIMATOR_DYNAMIC_Z_RP_MAX               POS_EKF_Z_R_MEAS * 100.0f
#define POS_ESTIMATOR_DYNAMIC_Z_RP_EPS               0.000001f
#define POS_ESTIMATOR_DYNAMIC_Z_RP_SCALE_EPS         0.001f
#define POS_ESTIMATOR_DYNAMIC_Z_RESIDUAL_CLAMP       0.05f
#define POS_ESTIMATOR_DYNAMIC_Z_ACC_XY_THRESH        15.0f
#define POS_ESTIMATOR_DYNAMIC_Z_ACC_Z_THRESH         20.0f

#define TERRAIN_BASE_R             0.01f    // 1cm variance on perfect, flat ground
#define TERRAIN_MAX_R             500.0f   // Cap variance so matrix math doesn't overflow (weight effectively drops to 0)
#define TERRAIN_OBSTACLE_GAIN      20.0f    // Sensitivity slider: Higher values reject obstacles faster

#define POSITION_MGR_Z_ENABLE_DYNAMIC_R                1
#define POSITION_MGR_VENTURI_ESTIMATE_ENABLED          1


void updateZPositionSL(float zPos, float dt);
void updateZPositionTerrain(float zTerrain, float quality, float dt);

void updateXYPosition(float hAcc, float xPos, float yPos, float dt);
void updateXYVelovity(float sAcc, float velN, float velE,float dt);

#endif /* SRC_FC_MANAGERS_POSITION_ESTIMATOR_POSITIONESTIMATORHELPER_H_ */
