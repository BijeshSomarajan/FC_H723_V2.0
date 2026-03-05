#ifndef SRC_FC_MANAGERS_POSITION_ESTIMATOR_VENTURIBIASESTIMATOR_H_
#define SRC_FC_MANAGERS_POSITION_ESTIMATOR_VENTURIBIASESTIMATOR_H_
#include <sys/_stdint.h>

typedef struct _VENTURI_ESTIMATE_DATA VENTURI_ESTIMATE_DATA;
struct _VENTURI_ESTIMATE_DATA {
	float venturiAbsPitchAngleFiltered;
	float ventiriAbsPitchRCFiltered;


	float venturiVelocity;
    float venturiBias;
    float lateralVelocity;
    float currentThrottle;
    float venturiBiasGain;
    float venturiAccelGain;
    float venturiThrustFactor;
};
extern VENTURI_ESTIMATE_DATA venturiEstimateData;

uint8_t initVenturiBiasEstimator(void);
float updateVenturiBiasEstimate(float dt);
void resetVenturiBiasEstimator(void);

//New ones
#define VENTURI_EST_PITCH_ANGLE_LPF_FREQ  2.5f //0.4f
#define VENTURI_EST_PITCH_RC_LPF_FREQ     2.5f //0.4f
#define VENTURI_EST_BIAS_LPF_FREQ         4.0f

#define VENTURI_EST_PITCH_ANGLE_MIN    0.5f
#define VENTURI_EST_PITCH_ANGLE_MAX   30.0f
#define VENTURI_EST_PITCH_RC_MIN       0.0f
#define VENTURI_EST_PITCH_RC_MAX      60.0f

#define VENTURI_EST_SPEED_MAX         12.0f
#define VENTURI_EST_BIAS_GAIN         45.0f
#define VENTURI_EST_BIAS_VALUE_MAX    80.0f//80.0f

#define VENTURI_SMALL_ANGLE_TSH      4.0f //1.5f
#define VENTURI_SMALL_ANGLE_DEFAULT  3.5f //2.0f
#define VENTURI_K_AERO_DRAG_SCALAR   4.2f // 3.5f

#endif
