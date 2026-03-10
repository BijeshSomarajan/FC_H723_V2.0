#ifndef SRC_FC_MANAGERS_POSITION_ESTIMATOR_VENTURIBIASESTIMATOR_H_
#define SRC_FC_MANAGERS_POSITION_ESTIMATOR_VENTURIBIASESTIMATOR_H_
#include <sys/_stdint.h>

typedef struct _VENTURI_ESTIMATE_DATA VENTURI_ESTIMATE_DATA;
struct _VENTURI_ESTIMATE_DATA {
	float pitchAngleAbsFiltered;
	uint8_t wasBiasFadingApplied;

	float venturiBias;
    float lateralSpeed;

    float effectiveThrottle;
    float thrustGain;

};
extern VENTURI_ESTIMATE_DATA venturiEstimateData;

uint8_t initVenturiBiasEstimator(void);
float updateVenturiBiasEstimate(float dt);
void resetVenturiBiasEstimator(void);


#define VENTURI_EST_PITCH_ANGLE_LPF_FREQ  20.0f
#define VENTURI_EST_BIAS_GAIN_LPF_FREQ    5.0f

#define VENTURI_EST_BIAS_LPF_RISE_FREQ   25.0f
#define VENTURI_EST_BIAS_LPF_FADE_FREQ   12.5f

#define VENTURI_EST_PITCH_ANGLE_MIN    1.0f
#define VENTURI_EST_PITCH_ANGLE_MAX   30.0f
#define VENTURI_EST_PITCH_ANGLE_FADING_TSH  1.0f

#define VENTURI_EST_SPEED_MAX         50.0f

#define VENTURI_EST_BIAS_GAIN_FWD     70.0f
#define VENTURI_EST_BIAS_GAIN_BWD     80.0f

#define VENTURI_EST_THRUST_GAIN_FACTOR 1.5f
#define VENTURI_EST_BIAS_VALUE_MAX     80.0f

#define VENTURI_EST_USE_PHYSICAL_MODEL 1

//Physical Model
#define VENTURI_EST_DRAG_FEEDBACK_GAIN 5.0f
//Algebraic Model
#define VENTURI_EST_PITCH_DRAG_GAIN         1.25f
#define VENTURI_EST_AERO_DRAG_FEEDBACK_GAIN 2.5f




#endif
