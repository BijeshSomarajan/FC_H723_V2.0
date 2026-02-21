#ifndef SRC_FC_SENSORS_ATTITUDE_ATTITUDENOISEFILTER_H_
#define SRC_FC_SENSORS_ATTITUDE_ATTITUDENOISEFILTER_H_
#include <stdint.h>
#include "../../../dsp/FFT.h"


/* --- Gyro FFT Configurations --- */
// Enable FFT-based analysis on gyro data for noise detection and adaptive filtering
#define SENSOR_FFT_GYRO_ENABLED                 1

// Number of FFT frequency bins considered for gyro processing (limits top analysis frequency)
#define SENSOR_FFT_GYRO_FREQUENCY_N             FFT_TOP_FREQ_N

// Start applying adaptive notch from harmonics first, then fundamental if enabled
#define SENSOR_FFT_GYRO_FILTER_HARMONICS_FIRST  0   // Filtration starts from harmonics to fundamental

// Minimum frequency (Hz) allowed for gyro adaptive notch filtering
#define SENSOR_FFT_NTF_GYRO_MIN_CUTOFF_FREQUENCY  40.0f

// Maximum frequency (Hz) allowed for gyro adaptive notch filtering
#define SENSOR_FFT_NTF_GYRO_MAX_CUTOFF_FREQUENCY  600.0f

// Overall gain scaling applied to the gyro notch filter output
#define SENSOR_FFT_NTF_GYRO_GAIN                1.0f

// Maximum FFT processing budget per control loop (limits CPU usage)
#define FFT_PROCESSING_BUDGET                   64

/*
 * Higher Q: Narrow notch, precise targeting of tonal noise
 * Lower Q: Wider notch, better for smeared or broadband noise
 */

// Minimum frequency change (Hz) required before updating notch center frequency
#define SENSOR_FFT_NTF_CF_DELTA_THRESHOLD       5.0f

/* --- Accelerometer LPF Configuration --- */

// Enable standard low-pass filter on accelerometer data
#define SENSOR_LPF_STD_ACC_ENABLED              1

// Accelerometer low-pass filter cutoff frequency (Hz)
#define SENSOR_LPF_STD_ACC_FREQUENCY            20.0f


/* --- Gyroscope LPF Configuration --- */

// Enable standard low-pass filter on gyro data
#define SENSOR_LPF_STD_GYRO_ENABLED             1

// Gyroscope low-pass filter cutoff frequency (Hz)
#define SENSOR_LPF_STD_GYRO_FREQUENCY           300.0f // DO not reduce , becomes wobbly


/* --- Magnetometer & Temperature LPF Configuration --- */

// Low-pass filter cutoff frequency (Hz) for magnetometer data
#define SENSOR_LPF_MAG_FREQUENCY                10.0f

// Low-pass filter cutoff frequency (Hz) for temperature sensor data
#define SENSOR_LPF_TEMP_FREQUENCY               2.0f


// ===== Noise Update / Filtering =====
void updateNoiseFilterData(float dt);
void updateNoiseFilterCoefficients();

void filterAGTNoise(float dt);
void filterMagNoise(float dt);

// ===== Temperature Correction =====
void calculateTempCorrectionOffsets(float currentTemp);
// ===== Noise Sensor Init / Reset =====
uint8_t initAttitudeNoiseFilter(float accSampleFrequency, float gyroSampleFrequency, float magSampleFrequency, float tempSampleFrequency);

void resetNoiseFilter(void);

#endif /* SRC_FC_SENSORS_ATTITUDE_ATTITUDENOISEFILTER_H_ */
