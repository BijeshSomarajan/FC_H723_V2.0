#include <sys/_stdint.h>

#include "../FCConfig.h"
#include "IMU.h"

#if IMU_FILTER_SELECTED == IMU_FILTER_MANHONY

#include "../util/MathUtil.h"
#include <math.h>

#define MAHONY_FILTER_SPIN_RATE_LIMIT 30
#define MAHONY_FILTER_MIN_MAG_MAGNITUDE 0.000001f

/* --- Mahony Mag Norm Gate & Channel Gain ---
// Mag correction trusted only when the field magnitude matches the learned
// local reference (motor-current distortion changes the norm).
//
//   weight
//    1.0 |________
//        |        \
//        |         \
//    0.0 |          \__________
//        +---GATE_START---GATE_FULL--->  |norm deviation| (fraction of ref)
//              0.10          0.30
*/
/* Mag norm gate: mag correction fades with |norm - ref| / ref.
 *   weight 1.0 below GATE_START, linear to 0.0 at GATE_FULL.
 *   Catches current-induced norm distortion (not direction-only distortion). */
#define MAHONY_FILTER_MAG_NORM_GATE_START   0.15f   // fade starts at 15% deviation
#define MAHONY_FILTER_MAG_NORM_GATE_FULL    0.40f   // fully ignored at 40%
#define MAHONY_FILTER_MAG_NORM_GATE_INV_W   (1.0f / (MAHONY_FILTER_MAG_NORM_GATE_FULL - MAHONY_FILTER_MAG_NORM_GATE_START))
/* Mag gain as fraction of KP. Effective yaw gain = RATIO * KP.
 * 0.3 * 0.6 = 0.18 -> heading tau ~5.5 s. Higher = faster heading pull. */
#define MAHONY_FILTER_MAG_GAIN_RATIO          0.3f
// Fast heading alignment on the ground only
#define MAHONY_FILTER_STABILIZE_MAG_GAIN_RATIO 1.0f
/* Reference norm learner: slow LPF, adapts only on clean field. [s] */
#define MAHONY_FILTER_MAG_REF_LEARN_TAU       120.0f

/* --- Mahony Accel Norm Gate ---
// Accel correction trusted only when |a| ≈ 1g (gravity-dominated sample).
// Full trust inside [FULL_LO, FULL_HI], linear fade to zero at [MIN, MAX] edges.
//
//   weight
//    1.0 |         ________________
//        |        /                \
//        |       /                  \
//    0.0 |______/                    \______
//        +-----MIN--FULL_LO--FULL_HI--MAX----->  |a| (g)
//             0.85   0.90     1.10   1.35
*/
#define MAHONY_FILTER_ACC_GATE_MIN           0.85f    // hard lower bound (g)
#define MAHONY_FILTER_ACC_GATE_FULL_LO       0.90f    // full-trust band, lower edge (g)
#define MAHONY_FILTER_ACC_GATE_FULL_HI       1.10f    // full-trust band, upper edge (g)
#define MAHONY_FILTER_ACC_GATE_MAX           1.35f    // hard upper bound (g)

// Derived — do not edit directly
#define MAHONY_FILTER_ACC_GATE_MIN_SQ        (MAHONY_FILTER_ACC_GATE_MIN * MAHONY_FILTER_ACC_GATE_MIN)
#define MAHONY_FILTER_ACC_GATE_MAX_SQ        (MAHONY_FILTER_ACC_GATE_MAX * MAHONY_FILTER_ACC_GATE_MAX)
#define MAHONY_FILTER_ACC_GATE_INV_W_LO      (1.0f / (MAHONY_FILTER_ACC_GATE_FULL_LO - MAHONY_FILTER_ACC_GATE_MIN))
#define MAHONY_FILTER_ACC_GATE_INV_W_HI      (1.0f / (MAHONY_FILTER_ACC_GATE_MAX  - MAHONY_FILTER_ACC_GATE_FULL_HI))

// Lower values means more inclined to Gyroscope and less influence of accelerometer
/*
#define MAHONY_FILTER_KP  1.0f * 1.0f   //0.6 //0.4 – 0.8    for 3.2Khz
#define MAHONY_FILTER_KI  1.0f * 0.005f //0.03 //0.02 – 0.05 for 3.2Khz ( Ki ≲ Kp²/4 )
*/

#define MAHONY_FILTER_KP  1.0f * 0.6f   //0.4 – 0.8   for 3.2Khz
#define MAHONY_FILTER_KI  1.0f * 0.03f  //0.02 – 0.05 for 3.2Khz ( Ki ≲ Kp²/4 )

#define MAHONY_FILTER_STABILIZE_KP  MAHONY_FILTER_KP * 10.0f
#define MAHONY_FILTER_STABILIZE_KI  MAHONY_FILTER_KI * 10.0f

#define MAHONY_FILTER_STAB_COUNT 5000

void imuFilterUpdate(float dt);
void imuFilterSetMode(uint8_t stablize);
uint8_t imuFilterInit(uint8_t stabilize);

void imuFilterReset(void);
uint16_t imuFilterGetStabilizationCount(void);
void imuFilterUpdateAngles(void);
void imuFilterUpdateHeading(void);

#endif
