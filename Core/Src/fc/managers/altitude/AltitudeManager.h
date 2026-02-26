#ifndef SRC_FC_MANAGERS_ALTITUDE_ALTITUDEMANAGER_H_
#define SRC_FC_MANAGERS_ALTITUDE_ALTITUDEMANAGER_H_

#include <sys/_stdint.h>

uint8_t initAltitudeManager(void);
void doAltitudeManagement(void);
void resetAltitudeManager(void);

//Baro reading frequency
#define ALTITUDE_SENSOR_BARO_READ_FREQUENCY BARO_SENSOR_READ_FREQUENCY
#define ALTITUDE_MANAGEMENT_TASK_FREQUENCY 1000

//Lift Off throttle and Throttle LPF settings
#define ALT_MGR_DEFAULT_LIFTOFF_THROTTLE 300

//Max permissible throttle
#define ALT_MGR_MAX_PERMISSIBLE_THROTTLE_DELTA 900
#define ALT_MGR_MAX_PERMISSIBLE_THROTTLE   RC_CHANNEL_MIN_VALUE + ALT_MGR_MAX_PERMISSIBLE_THROTTLE_DELTA
#define ALT_MGR_ALT_AGGREGATION_GAIN  0.5f //meter per second

#define ALT_MGR_MAX_ALT_DELTA 500 //Cms
#define ALT_MGR_THROTTLE_AVERAGING_LPF_FREQUENCY 20.0f//5.0f

//Altitude estimate Complementary filter alpha
#define ALT_MGR_TILT_TH_MIN_ANGLE 2.0f
#define ALT_MGR_TILT_TH_MAX_ANGLE 32.0f

//Tilt compensation common settings
#define ALT_MGR_TILT_TH_ADJUST_MAX_LIMIT 60
// --- Tilt Control ---
#define ALT_MGR_TILT_COMP_TH_ADJUST_GAIN 2000.0f
#define ALT_MGR_TILT_COMP_TH_ADJUST_TAU 0.1f
#define ALT_MGR_TILT_COMP_EXIT_TAU_MULTIPLIER 2.0f



// How fast the "speed" builds up
#define ALT_MGR_VENTURI_ACCEL_SCALER 10.0f
#define ALT_MGR_VENTURI_GAIN_BWD     1000 //0.03f
#define ALT_MGR_VENTURI_GAIN_FWD     500 //0.01f
#define ALT_MGR_VENTURI_TAU_RISE     0.1f
#define ALT_MGR_VENTURI_TAU_FADE     2.0f
#define ALT_MGR_VENTURI_DRAG_TAU 2.0f
#define ALT_MGR_VENTURI_BIAS_MAX 80.0f //in Cms
#define ALT_MGR_MAX_SPEED 10.0f

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

#endif
