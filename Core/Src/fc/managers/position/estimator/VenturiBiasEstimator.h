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
};
extern VENTURI_ESTIMATE_DATA venturiEstimateData;

uint8_t initVenturiBiasEstimator(void);
float updateVenturiBiasEstimate(float dt);
void resetVenturiBiasEstimator(void);

// --------------------------------------------------
// Venturi / Dynamic Pressure Bias Estimator
// Stabilized Configuration (GPS + Baro Multirotor)
// --------------------------------------------------

// ---------------- Signal Conditioning ----------------
#define VENTURI_EST_PITCH_ANGLE_LPF_FREQ        10.0f
#define VENTURI_EST_BIAS_GAIN_LPF_FREQ          10.0f

// ---------------- Dynamic Bias Filtering ----------------
#define VENTURI_EST_BIAS_LPF_RISE_FREQ          10.0f
#define VENTURI_EST_BIAS_LPF_FADE_FREQ          5.0f  // Increased to dump phantom bias instantly on stop

// ---------------- Attitude Constraints ----------------
#define VENTURI_EST_PITCH_ANGLE_MIN             1.5f
#define VENTURI_EST_PITCH_ANGLE_MAX             30.0f
#define VENTURI_EST_PITCH_ANGLE_FADING_TSH      3.0f
#define VENTURI_EST_SPEED_MAX                   25.0f

// ---------------- Physics Model ----------------
#define VENTURI_EST_THRUST_GAIN_FACTOR          1.0f
#define VENTURI_EST_DRAG_FEEDBACK_GAIN          0.25f
#define VENTURI_EST_BRAKING_DRAG_FWD_MULT       2.5f
#define VENTURI_EST_BRAKING_DRAG_BWD_MULT       5.0f
// ---------------- Pressure Bias Scaling (CM Based) ----------------
#define VENTURI_EST_BIAS_GAIN_FWD               0.22f
#define VENTURI_EST_BIAS_GAIN_BWD               0.14f
#define VENTURI_EST_BIAS_VALUE_MAX              200.0f

#endif
