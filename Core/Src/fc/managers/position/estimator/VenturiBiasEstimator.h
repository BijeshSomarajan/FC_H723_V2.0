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
#define VENTURI_EST_PITCH_ANGLE_LPF_FREQ  0.25f
#define VENTURI_EST_PITCH_RC_LPF_FREQ     0.25f
#define VENTURI_EST_BIAS_LPF_FREQ         2.50f

#define VENTURI_EST_PITCH_ANGLE_MIN    1.0f
#define VENTURI_EST_PITCH_ANGLE_MAX   30.0f
#define VENTURI_EST_PITCH_RC_MIN       0.0f
#define VENTURI_EST_PITCH_RC_MAX      45.0f
#define VENTURI_EST_SPEED_MAX         10.0f
#define VENTURI_EST_BIAS_GAIN         55.0f
#define VENTURI_EST_BIAS_VALUE_MAX    75.0f

#endif
