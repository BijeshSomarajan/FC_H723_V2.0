#ifndef SRC_FC_SENSORS_ATTITUDE_HELPERS_MAGCALIBRATIONHELPER_H_
#define SRC_FC_SENSORS_ATTITUDE_HELPERS_MAGCALIBRATIONHELPER_H_

#include <sys/_stdint.h>

/* ---- Tunables (units: uT unless noted) ---------------------------------- */
#define MAG_CAL_MIN_SPAN_UT       30.0f   /* min peak-to-peak on every axis       */
#define MAG_CAL_FIT_NORM_UT       50.0    /* normalizes fit features (conditioning)*/
#define MAG_CAL_FIT_MIN_RADIUS    15.0f   /* mean ellipsoid radius limits         */
#define MAG_CAL_FIT_MAX_RADIUS    80.0f
#define MAG_CAL_MAX_RADIUS_RATIO  2.0f    /* largest/smallest axis radius         */
#define MAG_CAL_MAX_FIT_SHIFT_UT  25.0f   /* fit bias vs min/max midpoint         */
#define MAG_CAL_MAX_SPREAD        0.03f   /* reject if |m_cal| spread > 3 %       */
#define SENSOR_MAG_CALIB_SAMPLE_COUNT           15000   /* upper limit on tries    */
#define SENSOR_MAG_CALIB_SAMPLE_DELAY           10

/* ---- Coverage control ---------------------------------------------------- */
#define MAG_CAL_BIN_START_SPAN_UT 25.0f   /* span on every axis before binning (centre must have settled) */
#define MAG_CAL_BIN_CAP           40      /* max samples kept per direction bin   */
#define MAG_CAL_MIN_BINS          24      /* of 26 direction bins ...             */
#define MAG_CAL_BIN_MIN           20      /* ... must hold at least this many (cap is MAG_CAL_BIN_CAP) */
#define MAG_CAL_MIN_SAMPLES       600     /* accepted samples needed to finish    */
#define MAG_CAL_MIN_RUN_SAMPLES   1500    /* never stop before this many reads (~ SAMPLE_DELAY+2 ms each) */

/*
 * Calibration result:  m_cal = A * (m_raw - b)     (uT, same sensor axes)
 *   b : hard-iron offset
 *   A : full 3x3 soft-iron matrix (symmetric -> does not rotate the sensor frame)
 */
typedef struct {
	float b[3];
	float A[3][3];
	float radius; /* mean field magnitude after correction, uT */
	float spread; /* approx. relative std-dev of |m_cal|       */
	float axisRatio; /* largest/smallest ellipsoid radius         */
	uint8_t binsFilled;
} MagCalResult;

/* Diagnostics of the last successful calibration (not needed at runtime). */
extern MagCalResult magCalResult;

/*
 * Runs the calibration (user tumbles the device in 3D meanwhile).
 * Returns 1 and stores the 12 parameters in deviceAttitudeData on success:
 *     biasMx/My/Mz   = b   (hard iron, uT)
 *     softIronM[3][3] = A  (soft iron)
 * On failure nothing is modified.
 */
uint8_t doMagCalibration(void);

/* Full correction: raw in uT (counts * magSensitivity) -> calibrated uT. */
void magCalApply(const float raw[3], float out[3]);

/* Restore a calibration saved earlier (b: 3 floats, A: 9 floats row-major). */
void magCalSet(const float b[3], const float A[3][3]);

#endif /* SRC_FC_SENSORS_ATTITUDE_HELPERS_MAGCALIBRATIONHELPER_H_ */
