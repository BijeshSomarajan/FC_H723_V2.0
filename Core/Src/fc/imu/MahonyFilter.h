#include "IMU.h"

#if IMU_FILTER_SELECTED == IMU_FILTER_MANHONY_BF

#include "../util/MathUtil.h"
#include <math.h>

#define MAHONY_FILTER_SPIN_RATE_LIMIT 20
#define MAHONY_FILTER_MIN_ACC_MAGNITUDE 0.000001f
#define MAHONY_FILTER_MIN_MAG_MAGNITUDE 0.000001f


// Lower values means more inclined to Gyroscope and less influence of accelerometer
#define MAHONY_FILTER_KP  1.0f * 0.6f  //0.4 – 0.8 for 3.2Khz
#define MAHONY_FILTER_KI  1.0f * 0.02f  //0.02 – 0.05 for 3.2Khz
#define MAHONY_FILTER_STABILIZE_KP  MAHONY_FILTER_KP * 10.0f
#define MAHONY_FILTER_STABILIZE_KI  MAHONY_FILTER_KI * 10.0f
#define MAHONY_FILTER_STAB_COUNT 5000

void imuFilterUpdate(float dt);
void imuFilterSetMode(uint8_t stablize);
uint8_t imuFilterInit(uint8_t stabilize);
void imuFilterReset(void);
uint16_t imuFilterGetStabilizationCount(void);
void imuFilterUpdateAngles(void);
void imuFilterUpdateHeading(float magIncl, float error);

#endif
