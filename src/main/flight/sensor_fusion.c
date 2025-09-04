/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "sensor_fusion.h"
#include "common/maths.h"

//#if GYRO_COUNT > 1

#define EPS_REG 0.00001f

void initVarCov(varCovApprox_t *varCovApprox, float tau, float dt)
{
    for (int gyro = 0; gyro < GYRO_COUNT; gyro++) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            pt1FilterInit(&varCovApprox->inputHighpass[gyro][axis], pt1FilterGain(20.0f, dt));
            pt1FilterInit(&varCovApprox->average[gyro][axis], pt1FilterGainFromDelay(tau, dt));
            pt1FilterInit(&varCovApprox->variance[gyro][axis], pt1FilterGainFromDelay(tau, dt));
            pt1FilterInit(&varCovApprox->covariance[gyro][axis], pt1FilterGainFromDelay(tau, dt));
        }
    }
}

void initSensorFusion(sensorFusion_t *fusion, float tau, float dt)
{
    initVarCov(&fusion->varCov, tau, dt);
}

void updateVarCov(varCovApprox_t *varCovApprox, float input[GYRO_COUNT], int gyro_count, int axis)
{
    // variance equation (sum(x-xAvg)^2)/num_samples
    // variance approx equation filt((x-xFilt)^2)
    // this works because the mean and average are similar
    // the sum(error^2)/2 is just the average error^2
    // filtering the error^2 is similar to getting the average
    // this is just an approximation
    // covariance equation sum((x-xAvg)(y-yAvg))/num_samples
    // following similar logic to the variance
    // we just need x and y filtered, find their error
    // then multiply their error and filter it

    // here the input is 1 axis of each gyro, aka all the X gyro readings
    float error[GYRO_COUNT];
    for (int gyro = 0; gyro < gyro_count; gyro++) {
        // first highpass the data to remove artifacts from real movements
        float highpassed_input = input[gyro] - pt1FilterApply(&varCovApprox->inputHighpass[gyro][axis], input[gyro]);
        float mean = pt1FilterApply(&varCovApprox->average[gyro][axis], highpassed_input);
        error[gyro] = highpassed_input - mean;
        float error2 = sq(error[gyro]);
        pt1FilterApply(&varCovApprox->variance[gyro][axis], error2); // variance
    }

    int idx = 0;
    for (int i = 0; i < gyro_count; i++) {
        for (int j = i + 1; j < gyro_count; j++) {
            pt1FilterApply(&varCovApprox->covariance[idx++][axis], error[i] * error[j]);
        }
    }
}

float variance(varCovApprox_t *varCovApprox, int gyro, int axis)
{
    return varCovApprox->variance[gyro][axis].state;
}

static inline int covIndex(int i, int j, int n) {
    // i<j assumed
    return i * n - (i * (i + 1)) / 2 + (j - i - 1);
}

float covariance(varCovApprox_t *varCovApprox, int gyro1, int gyro2, int gyro_count, int axis)
{
    return varCovApprox->covariance[covIndex(gyro1, gyro2, gyro_count)][axis].state;
}

// --- Approximate "inverse variance minus covariances" fusion ---
float fuse_approx(const varCovApprox_t *vc, const float *x, int n, int axis)
{
    float sum_w = 0.0, mu = 0.0;
    for (int i = 0; i < n; i++) {
        float sigma_eff = vc->variance[i][axis].state;
        for (int j = 0; j < n; j++) {
            if (i != j) {
                int idx = (i < j) ? covIndex(i,j,n) : covIndex(j,i,n);
                sigma_eff -= vc->covariance[idx][axis].state / n;
            }
        }
        if (sigma_eff <= 0.0f) sigma_eff = EPS_REG;
        float w = sq(1.0f / sigma_eff);
        sum_w += w;
        mu += w * x[i];
    }
    return (mu / sum_w);
}

// --- Simple sensor averaging ---
float fuse_avg(const float reading[GYRO_COUNT], int gyro_count)
{
    float averaged = 0.0f;
    for (int gyro = 0; gyro < gyro_count; gyro++) {
        averaged += reading[gyro];
    }
    return averaged / gyro_count;
}

static void insertion_sort(float *dst, const float *src, int N) {
    for (int i = 0; i < N; i++) {
        float val = src[i];
        int j = i - 1;
        while (j >= 0 && dst[j] > val) {
            dst[j + 1] = dst[j];
            j--;
        }
        dst[j + 1] = val;
    }
}

float fuse_median(const float gyros[GYRO_COUNT], int gyro_count) {
    float buf[GYRO_COUNT]; // adjust max size to your needs

    insertion_sort(buf, gyros, gyro_count);

    if (gyro_count % 2 == 0) {
        return (buf[gyro_count/2 - 1] + buf[gyro_count/2]) * 0.5f;
    } else {
        return buf[gyro_count/2];
    }
}

float fuse_voting(const float *sensors, int gyro_count, int cluster_size) {
    float buf[GYRO_COUNT]; // adjust as needed

    insertion_sort(buf, sensors, gyro_count);

    // Find the tightest cluster of "cluster_size" consecutive elements
    int best_start = 0;
    float best_range = 10000.0f;
    for (int i = 0; i <= gyro_count - cluster_size; i++) {
        float range = buf[i + cluster_size - 1] - buf[i];
        if (range < best_range) {
            best_range = range;
            best_start = i;
        }
    }

    // Average the cluster
    float sum = 0.0f;
    for (int i = 0; i < cluster_size; i++) {
        sum += buf[best_start + i];
    }
    return sum / (float)cluster_size;
}

void updateSensorFusion(sensorFusion_t *fusion, float sensor[GYRO_COUNT][XYZ_AXIS_COUNT], int gyro_count, fusionType_e fuse_mode, float fused[XYZ_AXIS_COUNT])
{
    // update variance and covariance for each axis
    float axis_vals[GYRO_COUNT];
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        for (int gyro = 0; gyro < gyro_count; gyro++) {
            axis_vals[gyro] = sensor[gyro][axis];
        }
        switch(fuse_mode) {
            case AVERAGING:
                fused[axis] = fuse_avg(axis_vals, gyro_count);
                break;
            case NOISE_APPROX:
                updateVarCov(&fusion->varCov, axis_vals, gyro_count, axis);
                fused[axis] = fuse_approx(&fusion->varCov, axis_vals, gyro_count, axis);
                break;
            case MEDIAN:
                fused[axis] = fuse_median(axis_vals, gyro_count);
                break;
            case VOTING:
                fused[axis] = fuse_voting(axis_vals, gyro_count, fusion->clusterSize);
                break;
            default:
                fused[axis] = fuse_avg(axis_vals, gyro_count);
        }
    }
}

//#endif
