static FAST_CODE void GYRO_FILTER_FUNCTION_NAME(gyroSensor_t *gyroSensor)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        GYRO_FILTER_DEBUG_SET(DEBUG_GYRO_RAW, axis, gyroSensor->gyroDev.gyroADCRaw[axis]);
        // scale gyro output to degrees per second
        float gyroADCf = gyroSensor->gyroDev.gyroADC[axis] * gyroSensor->gyroDev.scale;
        // DEBUG_GYRO_SCALED records the unfiltered, scaled gyro output
        GYRO_FILTER_DEBUG_SET(DEBUG_GYRO_SCALED, axis, lrintf(gyroADCf));

#ifdef USE_GYRO_DATA_ANALYSE
        if (isDynamicFilterActive()) {
            if (axis == X) {
                GYRO_FILTER_DEBUG_SET(DEBUG_FFT, 0, lrintf(gyroADCf));
                GYRO_FILTER_DEBUG_SET(DEBUG_FFT_FREQ, 3, lrintf(gyroADCf));
            }
        }

        float gyroDataForAnalysis = gyroADCf;
#endif

        // apply static notch filters and software lowpass filters
        gyroADCf = gyroSensor->notchFilter1ApplyFn((filter_t *)&gyroSensor->notchFilter1[axis], gyroADCf);
        gyroADCf = gyroSensor->notchFilter2ApplyFn((filter_t *)&gyroSensor->notchFilter2[axis], gyroADCf);
        gyroADCf = gyroSensor->lowpassFilterApplyFn((filter_t *)&gyroSensor->lowpassFilter[axis], gyroADCf);
        gyroADCf = gyroSensor->lowpass2FilterApplyFn((filter_t *)&gyroSensor->lowpass2Filter[axis], gyroADCf);

#ifdef USE_GYRO_DATA_ANALYSE
        if (isDynamicFilterActive()) {
            if (gyroConfig()->dyn_fft_location == DYN_FFT_AFTER_STATIC_FILTERS) {
                gyroDataForAnalysis = gyroADCf;
            }

            if (axis == X) {
                GYRO_FILTER_DEBUG_SET(DEBUG_FFT, 1, lrintf(gyroDataForAnalysis));
                GYRO_FILTER_DEBUG_SET(DEBUG_FFT_FREQ, 2, lrintf(gyroDataForAnalysis));
            }

            gyroDataAnalysePush(&gyroSensor->gyroAnalyseState, axis, gyroDataForAnalysis);
            gyroADCf = gyroSensor->notchFilterDynApplyFn((filter_t *)&gyroSensor->notchFilterDyn[axis], gyroADCf);
        }
#endif

        // DEBUG_GYRO_FILTERED records the scaled, filtered, after all software filtering has been applied.
        GYRO_FILTER_DEBUG_SET(DEBUG_GYRO_FILTERED, axis, lrintf(gyroADCf));

        gyroSensor->gyroDev.gyroADCf[axis] = gyroADCf;
    }
}
