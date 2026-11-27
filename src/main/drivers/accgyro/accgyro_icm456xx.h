/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ICM45686_FIFO_PACKET_BYTES 16
#define ICM45686_FIFO_BATCH_CAPACITY 15
#define ICM45686_ACCEL_COUNTS_PER_G 1024.0f
#define ICM45686_GYRO_COUNTS_PER_DPS 8.192f

struct extDevice_s;
// Zero-initialize before use. Retain the context until writes complete or the
// bus is closed; a timed-out asynchronous write can still own writeBuffer.
typedef struct {
    const struct extDevice_s *dev;
    // Retain this storage until an admitted native bus write has completed.
    uint8_t writeBuffer[6];
    bool writePending;
#if ENABLE_ICM45686_FIFO
    uint64_t lastClockUs, elapsedNs;
    bool clockStarted, clockFault;
#endif
} icm456xxDevice_t;

// Shared by the firmware SPI driver and optional buffered acquisition.
bool icm456xxReset(icm456xxDevice_t *device);
bool icm456xxReadIndirect(icm456xxDevice_t *device, uint16_t reg, uint8_t *value);
bool icm456xxWriteIndirect(icm456xxDevice_t *device, uint16_t reg, uint8_t value);

#if ENABLE_ICM45686_FIFO


typedef enum {
    ICM45686_PENDING = 0,
    ICM45686_READY = 1,
    ICM45686_INVALID = -1,
    ICM45686_TRANSPORT = -2,
    ICM45686_OVERFLOW = -3,
    ICM45686_PACKET = -4,
    ICM45686_TIMELINE = -5,
    ICM45686_RESTORE = -6,
    ICM45686_CONFIGURATION = -7,
    ICM45686_CLOSED = -8,
} icm45686Result_e;

typedef struct {
    int16_t accel[3], gyro[3], temperature;
} icm45686DirectSample_t;

typedef struct {
    int16_t accel[3], gyro[3];
    uint8_t header;
    int8_t temperature;
    uint16_t deviceTick;
    uint64_t deviceElapsedUs, ordinal;
} icm45686FifoFrame_t;

typedef struct {
    uint64_t elapsedUs, ordinal;
    uint16_t previousTick;
    bool hasTick;
} icm45686FifoTimeline_t;

typedef struct {
    uint8_t savedFifo[6];
    uint8_t savedTimestamp, savedSmc, savedInterrupt;
    bool saved, active, bigEndian;
} icm45686FifoDevice_t;

typedef struct {
    uint8_t data[ICM45686_FIFO_BATCH_CAPACITY * ICM45686_FIFO_PACKET_BYTES];
    size_t count;
    uint16_t available;
    uint64_t readBeginNs, readCompleteNs;
} icm45686FifoRead_t;

typedef struct {
    icm45686FifoDevice_t device;
    icm45686Result_e fault;
    icm45686FifoTimeline_t timeline;
    uint64_t lastReadNs, lastFrameNs;
} icm45686FifoStream_t;

// The 6.4 kHz, 32 g and 4000 dps profile is shared by direct UI and FIFO capture.
bool icm45686InitDirect(icm456xxDevice_t *io, bool retryIdentification);
icm45686Result_e icm45686ReadDirect(icm456xxDevice_t *io, icm45686DirectSample_t *sample);
icm45686Result_e icm45686FifoStart(icm456xxDevice_t *io, icm45686FifoDevice_t *device);
icm45686Result_e icm45686FifoStop(icm456xxDevice_t *io, icm45686FifoDevice_t *device);
icm45686Result_e icm45686FifoFetch(icm456xxDevice_t *io, const icm45686FifoDevice_t *device,
    icm45686FifoRead_t *readout);
// Validate the complete burst before updating the stream timeline or frames.
icm45686Result_e icm45686FifoDecode(const uint8_t *data, size_t count, bool bigEndian,
    icm45686FifoTimeline_t *timeline, icm45686FifoFrame_t frames[ICM45686_FIFO_BATCH_CAPACITY]);

// Zero-initialize before first use. Exclusive stream ownership belongs to the caller. Stream reads
// validate both host and device timelines; an active stream's fault remains
// latched until stop.
icm45686Result_e icm45686FifoStreamStart(icm456xxDevice_t *io, icm45686FifoStream_t *stream);
icm45686Result_e icm45686FifoStreamStop(icm456xxDevice_t *io, icm45686FifoStream_t *stream);
icm45686Result_e icm45686FifoStreamRead(icm456xxDevice_t *io, icm45686FifoStream_t *stream,
    icm45686FifoFrame_t frames[ICM45686_FIFO_BATCH_CAPACITY], icm45686FifoRead_t *readout);

#endif // ENABLE_ICM45686_FIFO

#ifdef __cplusplus
}
#endif
