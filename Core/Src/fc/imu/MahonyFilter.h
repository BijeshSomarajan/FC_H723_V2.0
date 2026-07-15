#include "IMU.h"

#if IMU_FILTER_SELECTED == IMU_FILTER_MANHONY_BF

#include "../util/MathUtil.h"
#include <math.h>

#define MAHONY_FILTER_SPIN_RATE_LIMIT 30
#define MAHONY_FILTER_MIN_MAG_MAGNITUDE 0.000001f

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
#define MAHONY_FILTER_KP  1.0f * 0.6f   //0.4 – 0.8 for 3.2Khz
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
void imuFilterUpdateHeading(float imuHeadingBias);

#endif
