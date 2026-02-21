#include "IMU.h"
#if IMU_FILTER_SELECTED == IMU_FILTER_MADGWICK

#include "mathUtil.h"
#include <math.h>
#include "IMU.h"

// Lower values means more inclined to Gyroscope and less influence of accelerometer
#define MADGWICK_FILTER_BETA  0.035f
#define MADGWICK_FILTER_STAB_BETA  MADGWICK_FILTER_BETA * 100

#define MADGWICK_FILTER_USE_INV_SQRT_APPROX 1
#define MADGWICK_FILTER_USE_SQRT_APPROX 1
#define MADGWICK_FILTER_USE_TRIG_APPROX 1

#define MADGWICK_FILTER_STAB_COUNT 5000

void imuFilterUpdate(float dt);
void imuFilterSetMode(uint8_t stablize);
uint8_t imuFilterInit(uint8_t stabilize);
void imuFilterReset(void);
uint16_t imuFilterGetStabilizationCount(void);
void imuFilterUpdateAngles(void);
void imuFilterUpdateHeading(float magIncl,float error);

#endif
