#include "AttitudeNoiseFilter.h"
#include "AdaptiveNotchFilter.h"

#include "../../../dsp/BiQuadFilter.h"
#include "../AttitudeSensor.h"
#include "../../../dsp/LowPassFilter.h"
#include "../../../logger/Logger.h"
#include "../../../memory/Memory.h"
#include "../../../FCConfig.h"
#include "../../../timers/DeltaTimer.h"

extern SENSOR_ATTITUDE_DATA sensorAttitudeData;

FFTContext fftContextGyroX;
FFTContext fftContextGyroY;
FFTContext fftContextGyroZ;

LOWPASSFILTER noiseFilterAccXImuLpF, noiseFilterAccYImuLpF, noiseFilterAccZImuLpF;
LOWPASSFILTER noiseFilterAccXLpF, noiseFilterAccYLpF, noiseFilterAccZLpF;

LOWPASSFILTER noiseFilterGyroXLpF, noiseFilterGyroYLpF, noiseFilterGyroZLpF;
LOWPASSFILTER noiseFilterMagXLPF, noiseFilterMagYLPF, noiseFilterMagZLPF;
LOWPASSFILTER noiseFilterTempLPF;

BIQUADFILTER noiseFilterFftNtfGyroX[SENSOR_FFT_GYRO_FREQUENCY_N];
BIQUADFILTER noiseFilterFftNtfGyroY[SENSOR_FFT_GYRO_FREQUENCY_N];
BIQUADFILTER noiseFilterFftNtfGyroZ[SENSOR_FFT_GYRO_FREQUENCY_N];

AdaptiveNotchFilterState noiseFilterXAdaptive[SENSOR_FFT_GYRO_FREQUENCY_N];
AdaptiveNotchFilterState noiseFilterYAdaptive[SENSOR_FFT_GYRO_FREQUENCY_N];
AdaptiveNotchFilterState noiseFilterZAdaptive[SENSOR_FFT_GYRO_FREQUENCY_N];

void filterAccNoise(float dt);
void filterTempNoise(float dt);
void filterGyroNoise(float dt);

__ATTR_ITCM_TEXT
void updateNoiseFilterData(float dt) {
#if SENSOR_FFT_GYRO_ENABLED == 1
	updateFFT(&fftContextGyroX, sensorAttitudeData.gxDS, dt);
	updateFFT(&fftContextGyroY, sensorAttitudeData.gyDS, dt);
	updateFFT(&fftContextGyroZ, sensorAttitudeData.gzDS, dt);
#endif
	sensorAttitudeData.noiseFilterDataUpdateDt = dt;
}

__ATTR_ITCM_TEXT
void updateNoiseFilterCoefficients() {

#if SENSOR_FFT_GYRO_ENABLED == 1

	processFFT(&fftContextGyroX, FFT_PROCESSING_BUDGET);
	processFFT(&fftContextGyroY, FFT_PROCESSING_BUDGET);
	processFFT(&fftContextGyroZ, FFT_PROCESSING_BUDGET);

	if (fftContextGyroX.hasProcessUpdate) {
		fftContextGyroX.hasProcessUpdate = 0;

# if ENABLE_DT_TRACE_FOR_DEBUG == 1
		float dt = getDeltaTime(NOISE_FILTER_FFTX_TIMER_CHANNEL);
		sensorAttitudeData.noiseFilterProcessXDt = dt;
#endif

		for (int freqIndx = 0; freqIndx < SENSOR_FFT_GYRO_FREQUENCY_N; freqIndx++) {
			float topXFreq = constrainToRangeF(fftContextGyroX.topFreq[freqIndx], SENSOR_FFT_NTF_GYRO_MIN_CUTOFF_FREQUENCY, SENSOR_FFT_NTF_GYRO_MAX_CUTOFF_FREQUENCY);
			if (fabsf(noiseFilterFftNtfGyroX[freqIndx].lastCenterFreq - topXFreq) >= SENSOR_FFT_NTF_CF_DELTA_THRESHOLD) {
				processAdaptiveNotchFilter(&noiseFilterFftNtfGyroX[freqIndx], &noiseFilterXAdaptive[freqIndx], topXFreq, fftContextGyroX.topFreqBin[freqIndx], fftContextGyroX.fftMagnitudes);
			}

		}
	}

	if (fftContextGyroY.hasProcessUpdate) {
		fftContextGyroY.hasProcessUpdate = 0;

# if ENABLE_DT_TRACE_FOR_DEBUG == 1
		float dt = getDeltaTime(NOISE_FILTER_FFTY_TIMER_CHANNEL);
		sensorAttitudeData.noiseFilterProcessYDt = dt;
#endif

		for (int freqIndx = 0; freqIndx < SENSOR_FFT_GYRO_FREQUENCY_N; freqIndx++) {
			float topYFreq = constrainToRangeF(fftContextGyroY.topFreq[freqIndx], SENSOR_FFT_NTF_GYRO_MIN_CUTOFF_FREQUENCY, SENSOR_FFT_NTF_GYRO_MAX_CUTOFF_FREQUENCY);
			if (fabsf(noiseFilterFftNtfGyroY[freqIndx].lastCenterFreq - topYFreq) >= SENSOR_FFT_NTF_CF_DELTA_THRESHOLD) {
				processAdaptiveNotchFilter(&noiseFilterFftNtfGyroY[freqIndx], &noiseFilterYAdaptive[freqIndx], topYFreq, fftContextGyroY.topFreqBin[freqIndx], fftContextGyroY.fftMagnitudes);
			}

		}
	}

	if (fftContextGyroZ.hasProcessUpdate) {
		fftContextGyroZ.hasProcessUpdate = 0;

# if ENABLE_DT_TRACE_FOR_DEBUG == 1
		float dt = getDeltaTime(NOISE_FILTER_FFTZ_TIMER_CHANNEL);
		sensorAttitudeData.noiseFilterProcessZDt = dt;
#endif

		for (int freqIndx = 0; freqIndx < SENSOR_FFT_GYRO_FREQUENCY_N; freqIndx++) {
			float topZFreq = constrainToRangeF(fftContextGyroZ.topFreq[freqIndx], SENSOR_FFT_NTF_GYRO_MIN_CUTOFF_FREQUENCY, SENSOR_FFT_NTF_GYRO_MAX_CUTOFF_FREQUENCY);
			if (fabsf(noiseFilterFftNtfGyroZ[freqIndx].lastCenterFreq - topZFreq) >= SENSOR_FFT_NTF_CF_DELTA_THRESHOLD) {
				processAdaptiveNotchFilter(&noiseFilterFftNtfGyroZ[freqIndx], &noiseFilterZAdaptive[freqIndx], topZFreq, fftContextGyroZ.topFreqBin[freqIndx], fftContextGyroZ.fftMagnitudes);
			}

		}
	}

#endif

}

__ATTR_ITCM_TEXT
void filterGyroNoise(float dt) {
	float gx = sensorAttitudeData.gxDS;
	float gy = sensorAttitudeData.gyDS;
	float gz = sensorAttitudeData.gzDS;

#if SENSOR_FFT_GYRO_ENABLED == 1
	for (int freqIndx = 0; freqIndx < SENSOR_FFT_GYRO_FREQUENCY_N; freqIndx++) {
		gx = biQuadFilterUpdate(&noiseFilterFftNtfGyroX[freqIndx], gx);
		gy = biQuadFilterUpdate(&noiseFilterFftNtfGyroY[freqIndx], gy);
		gz = biQuadFilterUpdate(&noiseFilterFftNtfGyroZ[freqIndx], gz);
	}
#endif

	gx = lowPassFilterUpdate(&noiseFilterGyroXLpF, gx, dt);
	gy = lowPassFilterUpdate(&noiseFilterGyroYLpF, gy, dt);
	gz = lowPassFilterUpdate(&noiseFilterGyroZLpF, gz, dt);

	sensorAttitudeData.gxDSFiltered = gx;
	sensorAttitudeData.gyDSFiltered = gy;
	sensorAttitudeData.gzDSFiltered = gz;
}

__ATTR_ITCM_TEXT
void filterAccNoise(float dt) {
	sensorAttitudeData.axGFiltered = lowPassFilterUpdate(&noiseFilterAccXLpF, sensorAttitudeData.axG, dt);
	sensorAttitudeData.ayGFiltered = lowPassFilterUpdate(&noiseFilterAccYLpF, sensorAttitudeData.ayG, dt);
	sensorAttitudeData.azGFiltered = lowPassFilterUpdate(&noiseFilterAccZLpF, sensorAttitudeData.azG, dt);

	sensorAttitudeData.axGFilteredImu = lowPassFilterUpdate(&noiseFilterAccXImuLpF, sensorAttitudeData.axG, dt);
	sensorAttitudeData.ayGFilteredImu = lowPassFilterUpdate(&noiseFilterAccYImuLpF, sensorAttitudeData.ayG, dt);
	sensorAttitudeData.azGFilteredImu = lowPassFilterUpdate(&noiseFilterAccZImuLpF, sensorAttitudeData.azG, dt);
}

__ATTR_ITCM_TEXT
void filterTempNoise(float dt) {
	sensorAttitudeData.tempFiltered = lowPassFilterUpdate(&noiseFilterTempLPF, sensorAttitudeData.temp, dt);
	calculateTempCorrectionOffsets(sensorAttitudeData.tempFiltered);
}

__ATTR_ITCM_TEXT
void filterAGTNoise(float dt) {
	filterAccNoise(dt);
	filterGyroNoise(dt);
	filterTempNoise(dt);
	sensorAttitudeData.agtNoiseFilterationDt = dt;
}

__ATTR_ITCM_TEXT
void filterMagNoise(float dt) {
	sensorAttitudeData.mxFiltered = lowPassFilterUpdate(&noiseFilterMagXLPF, sensorAttitudeData.mx, dt);
	sensorAttitudeData.myFiltered = lowPassFilterUpdate(&noiseFilterMagYLPF, sensorAttitudeData.my, dt);
	sensorAttitudeData.mzFiltered = lowPassFilterUpdate(&noiseFilterMagZLPF, sensorAttitudeData.mz, dt);
	sensorAttitudeData.magNoiseFilterationDt = dt;
}

__ATTR_ITCM_TEXT
void calculateTempCorrectionOffsets(float currentTemp) {
#if SENSOR_TEMP_CORRECTION_ACC_ENABLED == 1 || SENSOR_TEMP_CORRECTION_GYRO_ENABLED == 1
		float tempP1 = (currentTemp - memsData.offsetTemp);
		float tempP2 = tempP1 * tempP1;
		float tempP3 = tempP2 * tempP1;
	#if SENSOR_TEMP_CORRECTION_ACC_ENABLED == 1
		memsData.accXTempOffset = memsData.accXTempCoeff[0] + memsData.accXTempCoeff[1] * tempP1 + memsData.accXTempCoeff[2] * tempP2 + memsData.accXTempCoeff[3] * tempP3;
		memsData.accYTempOffset = memsData.accYTempCoeff[0] + memsData.accYTempCoeff[1] * tempP1 + memsData.accYTempCoeff[2] * tempP2 + memsData.accYTempCoeff[3] * tempP3;
		memsData.accZTempOffset = memsData.accZTempCoeff[0] + memsData.accZTempCoeff[1] * tempP1 + memsData.accZTempCoeff[2] * tempP2 + memsData.accZTempCoeff[3] * tempP3;
	#endif
	  #if SENSOR_TEMP_CORRECTION_GYRO_ENABLED == 1
		memsData.gyroXTempOffset = memsData.gyroXTempCoeff[0] + memsData.gyroXTempCoeff[1] * tempP1 + memsData.gyroXTempCoeff[2] * tempP2 + memsData.gyroXTempCoeff[3] * tempP3;
		memsData.gyroYTempOffset = memsData.gyroYTempCoeff[0] + memsData.gyroYTempCoeff[1] * tempP1 + memsData.gyroYTempCoeff[2] * tempP2 + memsData.gyroYTempCoeff[3] * tempP3;
		memsData.gyroZTempOffset = memsData.gyroZTempCoeff[0] + memsData.gyroZTempCoeff[1] * tempP1 + memsData.gyroZTempCoeff[2] * tempP2 + memsData.gyroZTempCoeff[3] * tempP3;
	#endif
#endif
}

uint8_t initAttitudeNoiseFilter(float accSampleFrequency, float gyroSampleFrequency, float magSampleFrequency, float tempSampleFrequency) {

	lowPassFilterInit(&noiseFilterAccXLpF, SENSOR_LPF_STD_ACC_FREQUENCY);
	lowPassFilterInit(&noiseFilterAccYLpF, SENSOR_LPF_STD_ACC_FREQUENCY);
	lowPassFilterInit(&noiseFilterAccZLpF, SENSOR_LPF_STD_ACC_FREQUENCY);

	lowPassFilterInit(&noiseFilterAccXImuLpF, SENSOR_LPF_IMU_ACC_FREQUENCY);
	lowPassFilterInit(&noiseFilterAccYImuLpF, SENSOR_LPF_IMU_ACC_FREQUENCY);
	lowPassFilterInit(&noiseFilterAccZImuLpF, SENSOR_LPF_IMU_ACC_FREQUENCY);

	lowPassFilterInit(&noiseFilterGyroXLpF, SENSOR_LPF_STD_GYRO_FREQUENCY);
	lowPassFilterInit(&noiseFilterGyroYLpF, SENSOR_LPF_STD_GYRO_FREQUENCY);
	lowPassFilterInit(&noiseFilterGyroZLpF, SENSOR_LPF_STD_GYRO_FREQUENCY);

	lowPassFilterInit(&noiseFilterMagXLPF, SENSOR_LPF_MAG_FREQUENCY);
	lowPassFilterInit(&noiseFilterMagYLPF, SENSOR_LPF_MAG_FREQUENCY);
	lowPassFilterInit(&noiseFilterMagZLPF, SENSOR_LPF_MAG_FREQUENCY);

	lowPassFilterInit(&noiseFilterTempLPF, SENSOR_LPF_TEMP_FREQUENCY);

#if SENSOR_FFT_GYRO_ENABLED == 1

	initFFT(gyroSampleFrequency);

	initFFTContext(&fftContextGyroX);
	initFFTContext(&fftContextGyroY);
	initFFTContext(&fftContextGyroZ);

	for (int freqIndx = 0; freqIndx < SENSOR_FFT_GYRO_FREQUENCY_N; freqIndx++) {
		biQuadFilterInit(&noiseFilterFftNtfGyroX[freqIndx], BIQUAD_NOTCH, SENSOR_FFT_NTF_GYRO_MIN_CUTOFF_FREQUENCY, gyroSampleFrequency, ADAPTIVE_NOTCH_Q_MIN, SENSOR_FFT_NTF_GYRO_GAIN);
		biQuadFilterInit(&noiseFilterFftNtfGyroY[freqIndx], BIQUAD_NOTCH, SENSOR_FFT_NTF_GYRO_MIN_CUTOFF_FREQUENCY, gyroSampleFrequency, ADAPTIVE_NOTCH_Q_MIN, SENSOR_FFT_NTF_GYRO_GAIN);
		biQuadFilterInit(&noiseFilterFftNtfGyroZ[freqIndx], BIQUAD_NOTCH, SENSOR_FFT_NTF_GYRO_MIN_CUTOFF_FREQUENCY, gyroSampleFrequency, ADAPTIVE_NOTCH_Q_MIN, SENSOR_FFT_NTF_GYRO_GAIN);

		biQuadFilterSetCenterFreq(&noiseFilterFftNtfGyroX[freqIndx], SENSOR_FFT_NTF_GYRO_MIN_CUTOFF_FREQUENCY);
		biQuadFilterSetCenterFreq(&noiseFilterFftNtfGyroY[freqIndx], SENSOR_FFT_NTF_GYRO_MIN_CUTOFF_FREQUENCY);
		biQuadFilterSetCenterFreq(&noiseFilterFftNtfGyroZ[freqIndx], SENSOR_FFT_NTF_GYRO_MIN_CUTOFF_FREQUENCY);

		noiseFilterXAdaptive[freqIndx].smoothedQ = SENSOR_FFT_NTF_GYRO_MIN_CUTOFF_FREQUENCY;
		noiseFilterYAdaptive[freqIndx].smoothedQ = SENSOR_FFT_NTF_GYRO_MIN_CUTOFF_FREQUENCY;
		noiseFilterZAdaptive[freqIndx].smoothedQ = SENSOR_FFT_NTF_GYRO_MIN_CUTOFF_FREQUENCY;
	}

#endif
	logString("[Attitude Noise Filter ] Init > Success\n");
	return 1;
}

void resetNoiseFilter() {
	lowPassFilterReset(&noiseFilterAccXLpF);
	lowPassFilterReset(&noiseFilterAccYLpF);
	lowPassFilterReset(&noiseFilterAccZLpF);

	lowPassFilterReset(&noiseFilterAccXImuLpF);
	lowPassFilterReset(&noiseFilterAccYImuLpF);
	lowPassFilterReset(&noiseFilterAccZImuLpF);

	lowPassFilterReset(&noiseFilterGyroXLpF);
	lowPassFilterReset(&noiseFilterGyroYLpF);
	lowPassFilterReset(&noiseFilterGyroZLpF);

	lowPassFilterReset(&noiseFilterMagXLPF);
	lowPassFilterReset(&noiseFilterMagYLPF);
	lowPassFilterReset(&noiseFilterMagZLPF);

	lowPassFilterReset(&noiseFilterTempLPF);

	for (int freqIndx = 0; freqIndx < SENSOR_FFT_GYRO_FREQUENCY_N; freqIndx++) {
		biQuadFilterReset(&noiseFilterFftNtfGyroX[freqIndx]);
		biQuadFilterReset(&noiseFilterFftNtfGyroY[freqIndx]);
		biQuadFilterReset(&noiseFilterFftNtfGyroZ[freqIndx]);

		noiseFilterXAdaptive[freqIndx].smoothedQ = ADAPTIVE_NOTCH_Q_MIN;
		noiseFilterXAdaptive[freqIndx].smoothedGain = SENSOR_FFT_NTF_GYRO_GAIN;
		noiseFilterXAdaptive[freqIndx].lastAppliedFreq = 0;
		noiseFilterXAdaptive[freqIndx].lastAppliedGain = 0;
		noiseFilterXAdaptive[freqIndx].lastAppliedQ = 0;
		noiseFilterXAdaptive[freqIndx].snr = 0;
		noiseFilterXAdaptive[freqIndx].peakMag = 0;

		noiseFilterYAdaptive[freqIndx].smoothedQ = ADAPTIVE_NOTCH_Q_MIN;
		noiseFilterYAdaptive[freqIndx].smoothedGain = SENSOR_FFT_NTF_GYRO_GAIN;
		noiseFilterYAdaptive[freqIndx].lastAppliedFreq = 0;
		noiseFilterYAdaptive[freqIndx].lastAppliedGain = 0;
		noiseFilterYAdaptive[freqIndx].lastAppliedQ = 0;
		noiseFilterYAdaptive[freqIndx].snr = 0;
		noiseFilterYAdaptive[freqIndx].peakMag = 0;

		noiseFilterZAdaptive[freqIndx].smoothedQ = ADAPTIVE_NOTCH_Q_MIN;
		noiseFilterZAdaptive[freqIndx].smoothedGain = SENSOR_FFT_NTF_GYRO_GAIN;
		noiseFilterZAdaptive[freqIndx].lastAppliedFreq = 0;
		noiseFilterZAdaptive[freqIndx].lastAppliedGain = 0;
		noiseFilterZAdaptive[freqIndx].lastAppliedQ = 0;
		noiseFilterZAdaptive[freqIndx].snr = 0;
		noiseFilterZAdaptive[freqIndx].peakMag = 0;
	}

}
