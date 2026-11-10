/*
 * This file is part of Betaflight.
 * SPDX-License-Identifier: GPL-3.0-or-later
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

typedef struct {
    void *context;
    // Return a positive value only after a complete register transaction.
    int (*read)(void *, uint8_t, uint8_t *, size_t);
    int (*write)(void *, uint8_t, const uint8_t *, size_t);
    void (*delayUs)(uint32_t);
    // Monotonic host time in nanoseconds, independent of the device timestamp.
    uint64_t (*clockNs)(void);
} icm45686Io_t;

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
bool icm45686InitDirect(const icm45686Io_t *io, bool retryIdentification);
icm45686Result_e icm45686ReadDirect(const icm45686Io_t *io, icm45686DirectSample_t *sample);
icm45686Result_e icm45686FifoStart(const icm45686Io_t *io, icm45686FifoDevice_t *device);
icm45686Result_e icm45686FifoStop(const icm45686Io_t *io, icm45686FifoDevice_t *device);
icm45686Result_e icm45686FifoFetch(const icm45686Io_t *io, const icm45686FifoDevice_t *device,
    icm45686FifoRead_t *readout);
// Validate the complete burst before updating the stream timeline or frames.
icm45686Result_e icm45686FifoDecode(const uint8_t *data, size_t count, bool bigEndian,
    icm45686FifoTimeline_t *timeline, icm45686FifoFrame_t frames[ICM45686_FIFO_BATCH_CAPACITY]);

// Zero-initialize before first use. Exclusive stream ownership belongs to the caller. Stream reads
// validate both host and device timelines; an active stream's fault remains
// latched until stop.
icm45686Result_e icm45686FifoStreamStart(const icm45686Io_t *io, icm45686FifoStream_t *stream);
icm45686Result_e icm45686FifoStreamStop(const icm45686Io_t *io, icm45686FifoStream_t *stream);
icm45686Result_e icm45686FifoStreamRead(const icm45686Io_t *io, icm45686FifoStream_t *stream,
    icm45686FifoFrame_t frames[ICM45686_FIFO_BATCH_CAPACITY], icm45686FifoRead_t *readout);

#ifdef __cplusplus
}
#endif
