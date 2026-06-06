#ifndef SRC_FC_MANAGERS_POSITION_ESTIMATOR_VENTURIBIASESTIMATOR_H_
#define SRC_FC_MANAGERS_POSITION_ESTIMATOR_VENTURIBIASESTIMATOR_H_
#include <sys/_stdint.h>

typedef struct _VENTURI_ESTIMATE_DATA VENTURI_ESTIMATE_DATA;
struct _VENTURI_ESTIMATE_DATA {
	float pitchAngleAbsFiltered;
	float venturiBias;
    float lateralSpeed;

    float effectiveThrottle;
};
extern VENTURI_ESTIMATE_DATA venturiEstimateData;

#define VENTURI_EST_PITCH_ANGLE_MIN             0.5f   // High = Ignores slow cruise tilt | Low = Traps stationary hover drift
#define VENTURI_EST_PITCH_ANGLE_MAX             30.0f  // High = Scales deep into speed runs | Low = Safe model clipping ceiling
#define VENTURI_EST_SPEED_MAX                   25.0f  // High = Accurate high-speed tracking | Low = Limits math runaway damage
#define VENTURI_EST_ACCEL_GAIN                  1.75f  // High = Faster estimated speed buildup | Low = Heavy airframe ramp compensation
#define VENTURI_EST_DRAG_GAIN                   0.35f  // High = Low terminal speed plateau | Low = High terminal speed runway
#define VENTURI_EST_BIAS_LPF_FREQ               0.20f  // High = Real-time acceleration tracking | Low = Delayed bias entry
#define VENTURI_EST_BIAS_GAIN                   0.07f  // High = Deep altitude sag at speed | Low = Ballooning upward at speed
#define VENTURI_EST_BIAS_VALUE_MAX              100.0f // High = Maximum EKF correction space | Low = Tight structural safety clamp
#define VENTURI_EST_DAMPING_GAIN                3.5f

uint8_t initVenturiBiasEstimator(void);
float updateVenturiBiasEstimate(float dt);
void resetVenturiBiasEstimator(void);
#endif
