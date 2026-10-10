#ifndef SRC_FC_SENSORS_ATTITUDE_HELPERS_MAGCALIBRATIONHELPER_H_
#define SRC_FC_SENSORS_ATTITUDE_HELPERS_MAGCALIBRATIONHELPER_H_

#include <sys/_stdint.h>

/* ---- Tunables (units: uT unless noted) ---------------------------------- */
#define MAG_CAL_MIN_SPAN_UT       30.0f   /* min peak-to-peak on every axis       */
#define MAG_CAL_FIT_NORM_UT       50.0    /* normalizes fit features (conditioning)*/
#define MAG_CAL_MAX_FIT_RMS       0.10    /* residual of fit, ~2x relative error  */
#define MAG_CAL_FIT_MIN_RADIUS    15.0f
#define MAG_CAL_FIT_MAX_RADIUS    80.0f
#define MAG_CAL_MAX_RADIUS_RATIO  2.0f    /* largest/smallest axis radius         */
#define MAG_CAL_MAX_FIT_SHIFT_UT  25.0f   /* fit vs min/max bias disagreement     */
#define SENSOR_MAG_CALIB_SAMPLE_COUNT           5000
#define SENSOR_MAG_CALIB_SAMPLE_DELAY           10
/*
 * Running sums for a least-squares fit of an axis-aligned ellipsoid
 *
 *     A u^2 + B v^2 + C w^2 + D u + E v + F w = 1
 *
 * No sample buffer needed: 6x6 normal equations are accumulated on the fly.
 */
typedef struct {
	double s[6][6];
	double b[6];
	uint32_t n;
} MagFitAccum;


uint8_t doMagCalibration(void);

#endif /* SRC_FC_SENSORS_ATTITUDE_HELPERS_MAGCALIBRATIONHELPER_H_ */
