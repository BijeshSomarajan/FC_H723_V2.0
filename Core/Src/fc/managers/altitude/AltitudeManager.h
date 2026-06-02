#ifndef SRC_FC_MANAGERS_ALTITUDE_ALTITUDEMANAGER_H_
#define SRC_FC_MANAGERS_ALTITUDE_ALTITUDEMANAGER_H_

#include <sys/_stdint.h>

uint8_t initAltitudeManager(void);
void doAltitudeManagement(void);
void resetAltitudeManager(void);

//Baro reading frequency
#define ALTITUDE_SENSOR_BARO_READ_FREQUENCY BARO_SENSOR_READ_FREQUENCY

#define ALTITUDE_MANAGEMENT_TASK_FREQUENCY 1000
#define ALTITUDE_MANAGEMENT_TASK_PERIOD 1.0f/ALTITUDE_MANAGEMENT_TASK_FREQUENCY

#define ALTITUDE_MANAGEMENT_ACC_TASK_FREQUENCY 600
#define ALTITUDE_MANAGEMENT_ACC_TASK_PERIOD 1.0f/ALTITUDE_MANAGEMENT_ACC_TASK_FREQUENCY

#define ALTITUDE_MANAGEMENT_VEL_TASK_FREQUENCY 300
#define ALTITUDE_MANAGEMENT_VEL_TASK_PERIOD 1.0f/ALTITUDE_MANAGEMENT_VEL_TASK_FREQUENCY

#define ALTITUDE_MANAGEMENT_ALT_TASK_FREQUENCY 75
#define ALTITUDE_MANAGEMENT_ALT_TASK_PERIOD 1.0f/ALTITUDE_MANAGEMENT_ALT_TASK_FREQUENCY

//Lift Off throttle and Throttle LPF settings
#define ALT_MGR_DEFAULT_LIFTOFF_THROTTLE 300

//Max permissible throttle
#define ALT_MGR_MAX_PERMISSIBLE_THROTTLE   RC_CHANNEL_MIN_VALUE + ALT_MGR_MAX_PERMISSIBLE_THROTTLE_DELTA
#define ALT_MGR_ALT_AGGREGATION_GAIN  0.35f //meter per second

#define ALT_MGR_MAX_ALT_DELTA 2.5f //Mts
#define ALT_MGR_THROTTLE_AVERAGING_LPF_FREQUENCY 20.0f//5.0f

// Tilt compensation constants
#define ALT_MGR_TILT_COMP_ENABLED      1
#define ALT_MGR_TILT_COMP_MIN_ANGLE    1.0f
#define ALT_MGR_TILT_COMP_MAX_ANGLE    30.0f
#define ALT_MGR_TILT_COMP_TAU_RISE     0.001f
#define ALT_MGR_TILT_COMP_TAU_FADE     0.5f
#define ALT_MGR_TILT_COMP_MAX_LIMIT    160.0f
#define ALT_MGR_TILT_COMP_GAIN         1.15f

// --- Alt Control Settings ---
// This threshold defines the stick deflection beyond which the altitude control will start to attenuate. Adjust based on testing.
#define ALT_MGR_ALT_CONTROL_SETTING_LATERAL_DT_THRESHOLD 0.4f
// In terms of throttle delta per second. This is used to calculate the rate of stick movement.
#define ALT_MGR_ALT_CONTROL_SETTING_THROTTLE_RATE_MAX 10.0f
// This scaler converts the raw throttle rate into a 0.0 to 1.0 range for gain attenuation. Adjust as needed based on testing.
#define ALT_MGR_ALT_CONTROL_STICK_RATE_SCALER 0.01f
// 1.0f = PID goes to 0 at max stick. 0.7f = PID keeps 30% authority at max stick.
#define ALT_MGR_ALT_CONTROL_STICK_ATTENUATION_GAIN 1.0f
// Higher = more aggressive PID ducking on fast moves
#define ALT_MGR_THROTTLE_RATE_ATTENUATION_GAIN 0.80f

#define ALT_MGR_THROTTLE_THRESHOLD_PERIOD 0.80f

/*
 * If the drone feels "mushy" for too long after centering: Reduce MP_TAU to 0.3s. This will make the drone "snap" into hover faster.
 * If the drone "bobs" up and down when you stop a climb: Increase RI_TAU to 1.2s.
 * This further slows down the re-engagement of the vertical weight compensation (the I-term), making the hand-off even softer.
 */
#define ALT_MGR_ALT_CONTROL_SETTING_MASTER_P_GAIN  0.0f  // Light feel
#define ALT_MGR_ALT_CONTROL_SETTING_MP_TAU         0.01f // Fast lock

#define ALT_MGR_ALT_CONTROL_SETTING_RATE_I_GAIN    0.5f  // Rate I is also reset
#define ALT_MGR_ALT_CONTROL_SETTING_RI_TAU         0.5f  // Smooth handover

#define ALT_MGR_ALT_CONTROL_SETTING_RATE_P_GAIN    0.1f
#define ALT_MGR_ALT_CONTROL_SETTING_RP_TAU         0.01f

#define ALT_MGR_ALT_CONTROL_SETTING_ACC_P_GAIN     0.1f // Zero fighting
#define ALT_MGR_ALT_CONTROL_SETTING_AP_TAU         0.01f

/* Autolanding configuration */
#define ALT_MGR_ALT_LANDING_PULSE_INACTIVE_PERIOD       0.75f
#define ALT_MGR_ALT_LANDING_PULSE_ACTIVE_PERIOD         0.75f
#define ALT_MGR_ALT_LANDING_STICK_COMMAND               150

#endif
