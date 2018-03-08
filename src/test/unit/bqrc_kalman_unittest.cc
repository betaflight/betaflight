
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include "gtest/gtest.h"

extern "C" {
    #include "common/maths.h"
    #include "common/filter.h"
}

#define M_PI_FLOAT 3.14159265358979323846f
#define SAMPLE_SET_SIZE 500000
#define PASSING_CORRELATION 0.9999f


double corr(double *x, double *y, int startIndex, int n)
{
    double sumX = 0;
    double sumY = 0;
    double sumXY = 0;
    double squareSumX = 0;
    double squareSumY = 0;

    for (int i = startIndex; i <= startIndex + n; i++) {
        sumX += x[i];
        sumY += y[i];
        sumXY += x[i] * y[i];
        squareSumX += pow(x[i], 2);
        squareSumY += pow(y[i], 2);
    }
    return (n * sumXY - sumX * sumY)
                  / sqrt((n * squareSumX - sumX * sumX)
                      * (n * squareSumY - sumY * sumY));
}

double mean(double *x, int startIndex, int n)
{
    double varSum = 0;

    for (int i = startIndex; i <= startIndex + n; i++) {
        varSum += x[i];
    }

    return varSum / n;
}

double var(double *x, int startIndex, int n) {

    double varSum = 0;
    const double mu = mean(x, startIndex, n);

    for (int i = startIndex; i <= startIndex + n; i++) {
        varSum += pow(x[i] - mu,2);
    }

    return varSum / n;
}


TEST(BQRCKalmanTest, BQRCKalmanRandomCorrelationTest)
{
    // Input parameters exposed to pilot
    const uint16_t q = 400;
    const uint16_t r = 88;
    const uint8_t p = 0;
    const float rate = 32000;

    fastKalman_t fkfFilter;
    fastKalmanInit(&fkfFilter, q, r, p);

    const float dT = 1 / rate;
    const float bqrcQ = fkfFilter.q;
    const float bqrcR = fkfFilter.r;
    const float bqrcK = 2 * sqrt(bqrcQ) / (sqrt(bqrcQ + 4 * bqrcR) + sqrt(bqrcQ));
    const float cutoffFrequency = 1 / (2 * M_PI_FLOAT * 0.5f * dT * (1 / bqrcK - 2));

    biquadFilter_t bqrcFilter;
    biquadRCFIR2FilterInit(&bqrcFilter, cutoffFrequency, dT);

    double kalmanVec[SAMPLE_SET_SIZE];
    double bqrcVec[SAMPLE_SET_SIZE];
    double lastK = 0;
    int kConvergeSample = 0;

    srand(time(NULL));
    for (int i = 0; i < SAMPLE_SET_SIZE; i++) {
        const float randVal = (rand() % 20000) * 1e-1f;

        kalmanVec[i] = fastKalmanUpdate(&fkfFilter, randVal);
        bqrcVec[i] = biquadFilterApply(&bqrcFilter, randVal);

        if (fkfFilter.k > lastK) {
            lastK = fkfFilter.k;
        } else if (kConvergeSample == 0) {
            kConvergeSample = i;
        }
    }

    const int numSamples = SAMPLE_SET_SIZE - 1 - kConvergeSample;  // offset for index value
    const float correlation = corr(kalmanVec, bqrcVec, kConvergeSample, numSamples);
    const float convergeTimeMs = (kConvergeSample + 1) * dT * 1000;
    const double fkfMean = mean(kalmanVec, kConvergeSample, numSamples);
    const double bqrcMean = mean(bqrcVec, kConvergeSample, numSamples);
    const double meanDifference = abs(fkfMean - bqrcMean);
    const double fkfDispersion = var(kalmanVec, kConvergeSample, numSamples) / fkfMean;
    const double bqrcDispersion = var(bqrcVec, kConvergeSample, numSamples) / bqrcMean;
    const double dispersionDifference = abs(fkfDispersion-bqrcDispersion);

    printf("\n[:::] FKF Q : %d\n", q);
    printf("[:::] FKF R : %d\n", r);
    printf("[:::] BQRC Cutoff Frequency: %.10f\n", cutoffFrequency);
    printf("[:::] FKF K converged at sample %d (%.3fms)\n", kConvergeSample + 1, convergeTimeMs);
    printf("===== Post-transient Statistics (%d samples) =====\n", numSamples);
    printf("[:::] Correlation : %.10e\n", correlation);
    printf("[:::] Mean Difference : %.10e\n", meanDifference);
    printf("[:::] Dispersion Difference: %.10e\n\n", dispersionDifference);
    EXPECT_LT(PASSING_CORRELATION, correlation);
}

TEST(BQRCKalmanTest, BQRCKalmanFlightDataCorrelationTest)
{
    // Input parameters exposed to pilot
    const uint16_t q = 400;
    const uint16_t r = 88;
    const uint8_t p = 0;
    const float rate = 32000;

    fastKalman_t fkfFilter;
    fastKalmanInit(&fkfFilter, q, r, p);

    const float dT = 1 / rate;
    const float bqrcQ = fkfFilter.q;
    const float bqrcR = fkfFilter.r;
    const float bqrcK = 2 * sqrt(bqrcQ) / (sqrt(bqrcQ + 4 * bqrcR) + sqrt(bqrcQ));
    const float cutoffFrequency = 1 / (2 * M_PI_FLOAT * 0.5f * dT * (1 / bqrcK - 2));

    biquadFilter_t bqrcFilter;
    biquadRCFIR2FilterInit(&bqrcFilter, cutoffFrequency, dT);

    double kalmanVec[SAMPLE_SET_SIZE];
    double bqrcVec[SAMPLE_SET_SIZE];
    double lastK = 0;
    int kConvergeSample = 0;

    srand(time(NULL));
    int samplesEvaluated = 0;
    FILE *file;
    FILE *out;
    file = fopen("unit/flightDebugData.csv", "r");
    out = fopen("unit/flightDebugOut.csv", "w");

    for (int i = 0; i < SAMPLE_SET_SIZE; i++) {
        int input = 0;
        fscanf(file, "%d", &input);

        if (feof(file)) {
            break;
        }

        kalmanVec[i] = fastKalmanUpdate(&fkfFilter, input);
        bqrcVec[i] = biquadFilterApply(&bqrcFilter, input);

        if (fkfFilter.k > lastK) {
            lastK = fkfFilter.k;
        } else if (kConvergeSample == 0) {
            kConvergeSample = i;
        }

        fprintf(out, "%d,%15.10f,%15.10f,%15.10f\n", input, fkfFilter.k, kalmanVec[i], bqrcVec[i]);
        fscanf(file, " %d", &input);
        ++samplesEvaluated;
    }

    fclose(file);
    fclose(out);

    const int numSamples = samplesEvaluated - 1 - kConvergeSample;  // offset for index value
    const float correlation = corr(kalmanVec, bqrcVec, kConvergeSample, numSamples);
    const float convergeTimeMs = (kConvergeSample + 1) * dT * 1000;
    const double fkfMean = mean(kalmanVec, kConvergeSample, numSamples);
    const double bqrcMean = mean(bqrcVec, kConvergeSample, numSamples);
    const double meanDifference = abs(fkfMean - bqrcMean);
    const double fkfDispersion = var(kalmanVec, kConvergeSample, numSamples) / fkfMean;
    const double bqrcDispersion = var(bqrcVec, kConvergeSample, numSamples) / bqrcMean;
    const double dispersionDifference = abs(fkfDispersion - bqrcDispersion);
    printf("\n[:::] FKF Q : %d\n", q);
    printf("[:::] FKF R : %d\n", r);
    printf("[:::] BQRC Cutoff Frequency: %.10f\n", cutoffFrequency);
    printf("[:::] FKF K converged at sample %d (%.3fms)\n", kConvergeSample+1, convergeTimeMs);
    printf("===== Post-transient Statistics (%d samples) =====\n", numSamples);
    printf("[:::] Correlation : %.10e\n", correlation);
    printf("[:::] Mean Difference : %.10e\n", meanDifference);
    printf("[:::] Dispersion Difference: %.10e\n\n", dispersionDifference);
    EXPECT_LT(PASSING_CORRELATION, correlation);
}