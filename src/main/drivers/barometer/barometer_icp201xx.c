/*
 * This file is part of Betaflight.
 *
 * ICP201XX Barometer Driver
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "platform.h"

#if defined(USE_BARO) && (defined(USE_BARO_ICP201XX) || defined(USE_BARO_SPI_ICP201XX))

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/barometer/barometer.h"
#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "barometer_icp201xx.h"

// Register addresses
#define ICP201XX_REG_EMPTY          0x00
#define ICP201XX_REG_TRIM1_MSB      0x05
#define ICP201XX_REG_TRIM2_LSB      0x06
#define ICP201XX_REG_TRIM2_MSB      0x07
#define ICP201XX_REG_DEVICE_ID      0x0C
#define ICP201XX_REG_VERSION        0xD3
#define ICP201XX_REG_OTP_CFG1       0xAC
#define ICP201XX_REG_OTP_MR_LSB     0xAD
#define ICP201XX_REG_OTP_MR_MSB     0xAE
#define ICP201XX_REG_OTP_MRA_LSB    0xAF
#define ICP201XX_REG_OTP_MRA_MSB    0xB0
#define ICP201XX_REG_OTP_MRB_LSB    0xB1
#define ICP201XX_REG_OTP_MRB_MSB    0xB2
#define ICP201XX_REG_OTP_ADDR       0xB5
#define ICP201XX_REG_OTP_CMD        0xB6
#define ICP201XX_REG_OTP_RDATA      0xB8
#define ICP201XX_REG_OTP_STATUS     0xB9
#define ICP201XX_REG_OTP_DEBUG2     0xBC
#define ICP201XX_REG_MASTER_LOCK    0xBE
#define ICP201XX_REG_OTP_STATUS2    0xBF
#define ICP201XX_REG_MODE_SELECT    0xC0
#define ICP201XX_REG_INT_STATUS     0xC1
#define ICP201XX_REG_FIFO_CONFIG    0xC3
#define ICP201XX_REG_FIFO_FILL      0xC4
#define ICP201XX_REG_DEVICE_STATUS  0xCD
#define ICP201XX_REG_ERR            0x02
#define ICP201XX_REG_PRESS_DATA_0   0xFA

// FIFO configuration values
#define ICP201XX_FIFO_MODE_PRES_TEMP  0x00

// SPI commands
#define ICP201XX_SPI_CMD_WRITE      0x33
#define ICP201XX_SPI_CMD_READ       0x3C

// Commands
#define ICP201XX_CMD_SOFT_RESET     0x80
#define ICP201XX_OTP_CMD_READ       0x10

// Expected device ID
#define ICP201XX_DEVICE_ID          0x63

// Operation modes (bits 5-7 of MODE_SELECT)
// Mode 0: BW=6.25Hz ODR=25Hz, Mode 1: BW=30Hz ODR=120Hz
// Mode 2: BW=10Hz ODR=40Hz, Mode 3: BW=0.5Hz ODR=2Hz
#define ICP201XX_OP_MODE0           (0 << 5)  // Mode 0: 25Hz ODR
#define ICP201XX_OP_MODE1           (1 << 5)  // Mode 1: 120Hz ODR (high-speed with trend detection)
#define ICP201XX_OP_MODE2           (2 << 5)  // Mode 2: 40Hz ODR
#define ICP201XX_OP_MODE3           (3 << 5)  // Mode 3: 2Hz ODR

// Select operation mode - Mode 1 for high-speed operation with averaging
#define ICP201XX_OP_MODE            ICP201XX_OP_MODE1

// Mode select bits (following ArduPilot layout)
// Bit 0-1: FIFO readout mode (0=pres_temp)
// Bit 2: Power mode (0=normal, 1=active) - USE NORMAL!
// Bit 3: Measurement mode (1=continuous)
// Bit 4: Forced trigger (0=standby)
// Bit 5-7: Operation mode
#define ICP201XX_MODE_POWER_NORMAL      (0 << 2)  // Normal power mode
#define ICP201XX_MODE_MEAS_CONTINUOUS   (1 << 3)
#define ICP201XX_FIFO_PRES_TEMP         (0 << 0)

// Timing constants for Mode 1 (120Hz ODR = 8.3ms interval)
#define ICP201XX_RESET_DELAY_MS         50
#define ICP201XX_STARTUP_DELAY_MS       100   // Increased from 10ms to 100ms for more reliable detection
#define ICP201XX_RETRY_DELAY_MS         2000  // Wait 2 seconds before retrying detection
#define ICP201XX_MAX_DETECT_ATTEMPTS    3     // Try detection up to 3 times before giving up
#define ICP201XX_READ_INTERVAL_MS       25    // Read every 25ms to collect ~3 samples at 120Hz
#define ICP201XX_MAX_SPI_CLK_HZ         6000000
#define ICP201XX_CONVERSION_INTERVAL_US 8333  // 8.3ms = 120Hz

// Hardware timing requirement for MODE_SELECT register
// This delay is UNCONDITIONAL and always applied after MODE_SELECT writes
#define ICP201XX_MODE_SELECT_LATCH_US 200

// Trend-weighted averaging configuration
#define ICP201XX_SAMPLE_BUFFER_SIZE 8       // Hold up to 8 samples for trend analysis
#define ICP201XX_MIN_SAMPLES_FOR_TREND 3    // Minimum samples needed for trend detection
#define ICP201XX_TREND_THRESHOLD 0.5f       // Pa/ms - threshold for considering data trending

// Sample structure for trend analysis
typedef struct {
    float pressure;      // Pressure in Pa
    float temperature;   // Temperature in degrees C
    uint32_t timestamp;  // Timestamp in milliseconds
} baroSample_t;

// Circular buffer for trend analysis
static struct {
    baroSample_t samples[ICP201XX_SAMPLE_BUFFER_SIZE];
    uint8_t head;        // Next write position
    uint8_t count;       // Number of valid samples
} sampleBuffer = { .head = 0, .count = 0 };

// Store last valid pressure and temperature to return when no new data available
static int32_t lastValidPressure = 0;
static int32_t lastValidTemperature = 0;

// Rate limiting for reads
static uint32_t lastFifoReadTime = 0;
#define ICP201XX_MIN_READ_INTERVAL_MS 25  // Read every 25ms (collect ~3 samples at 120Hz)

// DMA/Segment-based SPI state management
// These buffers must be static to persist across function calls for DMA
#define ICP201XX_MAX_READ_LEN 96  // Max FIFO size: 16 samples * 6 bytes per sample
static uint8_t spiTxBuf[ICP201XX_MAX_READ_LEN + 2];     // Command + register address + Dummy bytes
static uint8_t spiRxBuf[ICP201XX_MAX_READ_LEN + 2];     // Received data (offset by 2 bytes)

// State machine for non-blocking FIFO reads
typedef enum {
    ICP201XX_STATE_IDLE,
    ICP201XX_STATE_READ_FIFO_FILL_STARTED,
    ICP201XX_STATE_READ_FIFO_DATA_STARTED,
    ICP201XX_STATE_DATA_READY
} icp201xxReadState_t;

static icp201xxReadState_t readState = ICP201XX_STATE_IDLE;
static uint8_t fifoFillBuf[1];
static uint8_t fifoDataBuf[ICP201XX_MAX_READ_LEN];  // Buffer for multiple FIFO samples
static uint8_t fifoSampleCount = 0;  // Total number of samples to read from FIFO

// Forward declarations
static bool icp201xxStartUT(baroDev_t *baro);
static bool icp201xxGetUT(baroDev_t *baro);
static bool icp201xxReadUT(baroDev_t *baro);
static bool icp201xxStartUP(baroDev_t *baro);
static bool icp201xxGetUP(baroDev_t *baro);
static bool icp201xxReadUP(baroDev_t *baro);
static void icp201xxCalculate(int32_t *pressure, int32_t *temperature);

static void icp201xxDummyRead(const extDevice_t *dev)
{
    // Wait for any pending transfer to complete
    if (busBusy(dev, NULL)) {
        spiWait(dev);
    }

    static uint8_t dummyTxBuf[3];
    static uint8_t dummyRxBuf[3];

    dummyTxBuf[0] = ICP201XX_SPI_CMD_READ;
    dummyTxBuf[1] = ICP201XX_REG_EMPTY;
    dummyTxBuf[2] = 0xFF;

    // Use segments for dummy read too
    static busSegment_t dummySegments[2];
    dummySegments[0].u.buffers.txData = dummyTxBuf;
    dummySegments[0].u.buffers.rxData = dummyRxBuf;
    dummySegments[0].len = 3;
    dummySegments[0].negateCS = true;
    dummySegments[0].callback = NULL;

    dummySegments[1].u.link.dev = NULL;
    dummySegments[1].u.link.segments = NULL;
    dummySegments[1].len = 0;
    dummySegments[1].negateCS = true;
    dummySegments[1].callback = NULL;

    spiSequence(dev, &dummySegments[0]);
    spiWait(dev);
}

// Initiates a non-blocking DMA transfer to read a register
static bool icp201xxReadRegStart(const extDevice_t *dev, uint8_t reg, uint8_t len)
{
    if (len > ICP201XX_MAX_READ_LEN) return false;

    // Check if DMA/segment transfer is in progress
    if (busBusy(dev, NULL)) {
        return false;
    }

    // Prepare command buffer
    spiTxBuf[0] = ICP201XX_SPI_CMD_READ;
    spiTxBuf[1] = reg;
    memset(&spiTxBuf[2], 0xFF, len);

    // Set up single-phase SPI read using segments
    static busSegment_t segments[2];
    segments[0].u.buffers.txData = spiTxBuf;
    segments[0].u.buffers.rxData = spiRxBuf;
    segments[0].len = len + 2;
    segments[0].negateCS = true;
    segments[0].callback = NULL;

    segments[1].u.link.dev = NULL;
    segments[1].u.link.segments = NULL;
    segments[1].len = 0;
    segments[1].negateCS = true;
    segments[1].callback = NULL;

    // Start the DMA transfer (non-blocking)
    spiSequence(dev, &segments[0]);

    return true;
}

// Attempts a read transfer. If it fails, doesn't block. If successfully started, waits for completion, and handles the dummy read requirement.
static bool icp201xxReadReg(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t len)
{
    if (!icp201xxReadRegStart(dev, reg, len)) {
        return false;
    }

    // Wait for completion
    spiWait(dev);

    // Copy received data
    memcpy(data, &spiRxBuf[2], len);

    // Perform dummy read if not reading EMPTY register
    if (reg != ICP201XX_REG_EMPTY) {
        icp201xxDummyRead(dev);
    }

    return true;
}

// A blocking operation for writing to the device
static bool icp201xxWriteReg(const extDevice_t *dev, uint8_t reg, uint8_t val)
{
    // Wait for any pending transfer
    if (busBusy(dev, NULL)) {
        spiWait(dev);
    }

    uint8_t writeTxBuf[3];
    uint8_t writeRxBuf[3];

    writeTxBuf[0] = ICP201XX_SPI_CMD_WRITE;
    writeTxBuf[1] = reg;
    writeTxBuf[2] = val;

    // Single-phase write using segments
    busSegment_t writeSegments[2];
    writeSegments[0].u.buffers.txData = writeTxBuf;
    writeSegments[0].u.buffers.rxData = writeRxBuf;
    writeSegments[0].len = 3;
    writeSegments[0].negateCS = true;
    writeSegments[0].callback = NULL;

    writeSegments[1].u.link.dev = NULL;
    writeSegments[1].u.link.segments = NULL;
    writeSegments[1].len = 0;
    writeSegments[1].negateCS = true;
    writeSegments[1].callback = NULL;

    spiSequence(dev, &writeSegments[0]);
    spiWait(dev);

    icp201xxDummyRead(dev);

    return true;
}

// Helper function for read-modify-write operations on registers
static int icp201xxModifyReg(const extDevice_t *dev, uint8_t reg, uint8_t clear_bits, uint8_t set_bits)
{
    uint8_t old;
    
    // Read current register value
    if (!icp201xxReadReg(dev, reg, &old, 1)) {
        return -1; // Error reading register
    }
    
    // Calculate new value with clear and set operations
    uint8_t new_val = (old & ~clear_bits) | set_bits;
    
    // Write the new value
    if (!icp201xxWriteReg(dev, reg, new_val)) {
        return -1; // Error writing register
    }
    
    return new_val; // Return the new value
}

static bool icp201xxSoftReset(const extDevice_t *dev)
{
    if (!icp201xxWriteReg(dev, ICP201XX_REG_MODE_SELECT, ICP201XX_CMD_SOFT_RESET)) {
        return false;
    }
    delay(ICP201XX_RESET_DELAY_MS);
    return true;
}

static bool icp201xxReadOTPData(const extDevice_t *dev, uint8_t addr, uint8_t *val)
{
    uint8_t otpStatus;
    bool otpReady = false;

    // Write the OTP address
    if (!icp201xxWriteReg(dev, ICP201XX_REG_OTP_ADDR, addr)) return false;
    if (!icp201xxWriteReg(dev, ICP201XX_REG_OTP_CMD, ICP201XX_OTP_CMD_READ)) return false;

    // Wait for OTP read completion
    for (int i = 0; i < 50; i++) {
        if (!icp201xxReadReg(dev, ICP201XX_REG_OTP_STATUS, &otpStatus, 1)) return false;
        if (otpStatus == 0) {
            otpReady = true;
            break;
        }
        delayMicroseconds(20);
    }

    // Check if OTP read timed out
    if (!otpReady) {
        return false;
    }

    if (!icp201xxReadReg(dev, ICP201XX_REG_OTP_RDATA, val, 1)) return false;
    return true;
}

static bool icp201xxBootSequence(const extDevice_t *dev)
{
    uint8_t bootupStatus, version;
    uint8_t offset, gain, hfosc;

    // Check version - B2 doesn't need boot sequence
    if (!icp201xxReadReg(dev, ICP201XX_REG_VERSION, &version, 1)) return false;

    if (version == 0xB2) {
        return true; // B2 version doesn't need boot sequence
    }

    // Check if boot already done
    if (!icp201xxReadReg(dev, ICP201XX_REG_OTP_STATUS2, &bootupStatus, 1)) return false;
    if (bootupStatus & 0x01) return true; // Already done

    // Activate OTP power domain
    if (!icp201xxWriteReg(dev, ICP201XX_REG_MODE_SELECT, 0x04)) return false;
    delay(4);

    // Unlock master register
    icp201xxWriteReg(dev, ICP201XX_REG_MASTER_LOCK, 0x1F);

    // Enable OTP and write switch
    if (icp201xxModifyReg(dev, ICP201XX_REG_OTP_CFG1, 0, 0x03) < 0) return false;
    delayMicroseconds(10);

    // Toggle OTP reset
    if (icp201xxModifyReg(dev, ICP201XX_REG_OTP_DEBUG2, 0, (1 << 7)) < 0) return false;
    delayMicroseconds(10);
    if (icp201xxModifyReg(dev, ICP201XX_REG_OTP_DEBUG2, (1 << 7), 0) < 0) return false;
    delayMicroseconds(10);

    // Program redundant read registers
    icp201xxWriteReg(dev, ICP201XX_REG_OTP_MRA_LSB, 0x04);
    icp201xxWriteReg(dev, ICP201XX_REG_OTP_MRA_MSB, 0x04);
    icp201xxWriteReg(dev, ICP201XX_REG_OTP_MRB_LSB, 0x21);
    icp201xxWriteReg(dev, ICP201XX_REG_OTP_MRB_MSB, 0x20);
    icp201xxWriteReg(dev, ICP201XX_REG_OTP_MR_LSB, 0x10);
    icp201xxWriteReg(dev, ICP201XX_REG_OTP_MR_MSB, 0x80);

    // Read OTP calibration values
    if (!icp201xxReadOTPData(dev, 0xF8, &offset)) return false;
    if (!icp201xxReadOTPData(dev, 0xF9, &gain)) return false;
    if (!icp201xxReadOTPData(dev, 0xFA, &hfosc)) return false;

    // Write OTP values to trim registers
    if (icp201xxModifyReg(dev, ICP201XX_REG_TRIM1_MSB, 0x3F, offset & 0x3F) < 0) return false;

    if (icp201xxModifyReg(dev, ICP201XX_REG_TRIM2_MSB, 0x70, (gain & 0x07) << 4) < 0) return false;

    if (icp201xxModifyReg(dev, ICP201XX_REG_TRIM2_LSB, 0x7F, hfosc & 0x7F) < 0) return false;

    delayMicroseconds(10);

    // Mark boot as complete
    if (icp201xxModifyReg(dev, ICP201XX_REG_OTP_STATUS2, 0, 0x01) < 0) return false;

    // Disable OTP
    if (icp201xxModifyReg(dev, ICP201XX_REG_OTP_CFG1, 0x03, 0) < 0) return false;

    // Lock master register
    if (!icp201xxWriteReg(dev, ICP201XX_REG_MASTER_LOCK, 0x00)) return false;

    // Return to standby mode
    if (!icp201xxWriteReg(dev, ICP201XX_REG_MODE_SELECT, 0x00)) return false;
    delay(10);

    return true;
}

static bool icp201xxFlushFifo(const extDevice_t *dev)
{
    // Flush the FIFO by setting the flush bit (0x80) in the FIFO_FILL register
    return (icp201xxModifyReg(dev, ICP201XX_REG_FIFO_FILL, 0, 0x80) >= 0);
}

static bool icp201xxStartContinuous(const extDevice_t *dev)
{
    uint8_t fifoFill;
    uint8_t fifoPackets = 0;

    // Perform soft reset to ensure clean state before configuration
    if (!icp201xxSoftReset(dev)) {
        return false;
    }

    // Use Read-Modify-Write for MODE_SELECT to preserve other bits.
    // Write standby mode first
    if (!icp201xxWriteReg(dev, ICP201XX_REG_MODE_SELECT, 0x00)) {
        return false;
    }
    delay(5);

    // Flush FIFO
    icp201xxFlushFifo(dev);
    delay(5);

    // Clear any interrupt status
    uint8_t intStatus = 0;
    if (icp201xxReadReg(dev, ICP201XX_REG_INT_STATUS, &intStatus, 1)) {
        if (intStatus != 0) {
            // Clear by writing back
            icp201xxWriteReg(dev, ICP201XX_REG_INT_STATUS, intStatus);
        }
    }

    // Now build mode register using Read-Modify-Write for each field
    // Set forced meas trigger = 0 (standby)
    if (icp201xxModifyReg(dev, ICP201XX_REG_MODE_SELECT, (1 << 4), 0) < 0) {
        return false;
    }

    // When calling MODE_SELECT, a delay is necessary for the device to stabilize.
    delayMicroseconds(ICP201XX_MODE_SELECT_LATCH_US);

    // Set power mode = 0 (normal)
    if (icp201xxModifyReg(dev, ICP201XX_REG_MODE_SELECT, (1 << 2), 0) < 0) {
        return false;
    }

    // When calling MODE_SELECT, a delay is necessary for the device to stabilize.
    delayMicroseconds(ICP201XX_MODE_SELECT_LATCH_US);

    // Set FIFO readout mode = 0 (pres+temp)
    if (icp201xxModifyReg(dev, ICP201XX_REG_MODE_SELECT, 0x03, 0) < 0) {
        return false;
    }

    // When calling MODE_SELECT, a delay is necessary for the device to stabilize.
    delayMicroseconds(ICP201XX_MODE_SELECT_LATCH_US);

    // Set measurement config (OP_MODE0 = bits 7-5 = 000)
    if (icp201xxModifyReg(dev, ICP201XX_REG_MODE_SELECT, 0xE0, ICP201XX_OP_MODE) < 0) {
        return false;
    }

    // When calling MODE_SELECT, a delay is necessary for the device to stabilize.
    delayMicroseconds(ICP201XX_MODE_SELECT_LATCH_US);

    // Finally set measurement mode = 1 (continuous) - bit 3
    if (icp201xxModifyReg(dev, ICP201XX_REG_MODE_SELECT, 0, (1 << 3)) < 0) {
        return false;
    }

    delay(10);

    // Wait for FIR filter warmup. The first few samples after mode change are invalid.
    // At 120Hz ODR (MODE1), 14 samples = 117ms.
    const uint8_t targetSamples = 14;

    for (int i = 0; i < 100; i++) {  // Max 1 second wait
        delay(10);
        if (icp201xxReadReg(dev, ICP201XX_REG_FIFO_FILL, &fifoFill, 1)) {
            fifoPackets = fifoFill & 0x1F;
            if (fifoPackets >= targetSamples) {
                break;
            }
        }
    }

    // Check if FIFO filled during warmup - if not, sensor is not working
    if (fifoPackets == 0) {
        return false;  // Sensor not producing data
    }

    // Flush warmup samples
    icp201xxFlushFifo(dev);

    return true;
}

void icp201xxBusInit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_ICP201XX
    IOInit(dev->busType_u.spi.csnPin, OWNER_BARO_CS, 0);
    IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_OUT_PP);
    IOHi(dev->busType_u.spi.csnPin);
    spiSetClkDivisor(dev, spiCalculateDivider(ICP201XX_MAX_SPI_CLK_HZ));
#else
    UNUSED(dev);
#endif
}

void icp201xxBusDeinit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_ICP201XX
    IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_IPU);
#else
    UNUSED(dev);
#endif
}

bool icp201xxDetect(baroDev_t *baro)
{
    // Real sensor detection with retry mechanism
    uint8_t deviceID;
    bool initSuccess = false;

    // Brief wait to let the sensor come up
    delay(ICP201XX_STARTUP_DELAY_MS);

    // Initialize the bus
    icp201xxBusInit(&baro->dev);

    // Register device with bus system
    busDeviceRegister(&baro->dev);

    // Try detection up to MAX_DETECT_ATTEMPTS times
    for (int attempt = 1; !initSuccess && attempt <= ICP201XX_MAX_DETECT_ATTEMPTS; attempt++) {
        // Initial startup delay (longer on first attempt)
        if (attempt > 1) {
            // Retry after additional delay
            delay(ICP201XX_RETRY_DELAY_MS);
        }

        // Read device ID
        if (!icp201xxReadReg(&baro->dev, ICP201XX_REG_DEVICE_ID, &deviceID, 1)) {
            continue;  // Try again
        }

        if (deviceID != ICP201XX_DEVICE_ID) {
            continue;  // Try again
        }

        // Boot sequence (OTP calibration) - soft reset is done inside StartContinuous
        if (!icp201xxBootSequence(&baro->dev)) {
            continue;  // Try again
        }

        // Start continuous measurement mode
        if (!icp201xxStartContinuous(&baro->dev)) {
            continue;  // Try again
        }

        // Success!
        initSuccess = true;
    }

    // Check if initialization succeeded
    if (!initSuccess) {
        icp201xxBusDeinit(&baro->dev);
        return false;
    }

    // Initialize sample buffer and last valid values
    sampleBuffer.head = 0;
    sampleBuffer.count = 0;
    lastValidPressure = 0;
    lastValidTemperature = 0;
    lastFifoReadTime = millis();

    // Setup function pointers
    baro->combined_read = true;
    baro->ut_delay = 0;
    baro->up_delay = 25000;  // 25ms (25000µs) delay - read every 25ms to collect ~3 samples at 120Hz
    baro->start_ut = icp201xxStartUT;
    baro->get_ut = icp201xxGetUT;
    baro->read_ut = icp201xxReadUT;
    baro->start_up = icp201xxStartUP;
    baro->get_up = icp201xxGetUP;
    baro->read_up = icp201xxReadUP;
    baro->calculate = icp201xxCalculate;

    return true;
}

static bool icp201xxStartUT(baroDev_t *baro)
{
    UNUSED(baro);
    return true; // Continuous mode, measurement always running
}

static bool icp201xxReadUT(baroDev_t *baro)
{
    UNUSED(baro);
    return true; // Temperature is read alongside pressure in GetUP
}

static bool icp201xxGetUT(baroDev_t *baro)
{
    UNUSED(baro);
    return true; // Temperature data retrieved in GetUP
}

static bool icp201xxStartUP(baroDev_t *baro)
{
    UNUSED(baro);
    // Sensor is in continuous mode, always sampling
    return true;
}

// Handles the completion of SPI transfers initiated by ReadUP and processes the data.
// Implements a state machine to read FIFO fill level, then read the data.
static bool icp201xxGetUP(baroDev_t *baro)
{
    // Check if bus is busy
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    // State machine for non-blocking FIFO reads
    switch (readState) {
        case ICP201XX_STATE_IDLE:
            // Do nothing, wait for readUP to start a read
            break;

        case ICP201XX_STATE_READ_FIFO_FILL_STARTED:
            // FIFO fill read completed, copy data
            fifoFillBuf[0] = spiRxBuf[2];

            // Check Empty bit (bit 6). If 0, FIFO contains data.
            if ((fifoFillBuf[0] & 0x40) == 0) {
                uint8_t fifoCount = fifoFillBuf[0] & 0x1F;
                if (fifoCount > 0) {
                    // fifoCount > 16 is reserved, but may indicate a device with larger FIFO
                    // We cap it to 16 (our maximum buffer size) to prevent device freezing
                    if (fifoCount > 16) {
                        fifoCount = 16;
                    }
                    
                    // Try reading ALL FIFO packets in one transaction (test auto-increment)
                    fifoSampleCount = fifoCount;
                    uint8_t bytesToRead = fifoCount * 6;

                    if (icp201xxReadRegStart(&baro->dev, ICP201XX_REG_PRESS_DATA_0, bytesToRead)) {
                        readState = ICP201XX_STATE_READ_FIFO_DATA_STARTED;
                    } else {
                        readState = ICP201XX_STATE_IDLE;
                    }
                } else {
                    readState = ICP201XX_STATE_IDLE;
                }
            } else {
                readState = ICP201XX_STATE_IDLE;
            }
            break;

        case ICP201XX_STATE_READ_FIFO_DATA_STARTED:
            // All FIFO data read completed in one transaction, copy to buffer
            // The SPI transaction is atomic and guarantees all data is received
            // in the order it was stored in the FIFO
            
            // Quick validation of the received data before copying
            // Only check for completely invalid patterns (all 0xFF or all 0x00)
            // This is an O(1) check that only examines the first and last byte
            const uint8_t* dataPtr = &spiRxBuf[2];
            const uint8_t dataLen = fifoSampleCount * 6;
            bool validData = true;
            
            // Check first and last byte for invalid patterns
            if (dataLen > 0) {
                const uint8_t firstByte = dataPtr[0];
                const uint8_t lastByte = dataPtr[dataLen - 1];
                
                // If all bytes are 0xFF or all bytes are 0x00, it's invalid
                if ((firstByte == 0xFF && lastByte == 0xFF) || 
                    (firstByte == 0x00 && lastByte == 0x00)) {
                    // Double-check with a slightly more thorough test only if suspicious
                    // Check 4 evenly spaced bytes across the buffer
                    const uint8_t indices[] = {0, dataLen/3, 2*dataLen/3, dataLen-1};
                    bool allSame = true;
                    for (uint8_t i = 1; i < 4; i++) {
                        if (dataPtr[indices[i]] != firstByte) {
                            allSame = false;
                            break;
                        }
                    }
                    
                    if (allSame) {
                        validData = false;
                    }
                }
            }
            
            if (validData) {
                memcpy(fifoDataBuf, dataPtr, dataLen);
                readState = ICP201XX_STATE_DATA_READY;
            } else {
                // Data validation failed, discard and reset
                fifoSampleCount = 0;
                readState = ICP201XX_STATE_IDLE;
            }
            break;

        case ICP201XX_STATE_DATA_READY:
            // Data already processed in calculate, ready for next read
            break;
    }

    return true;
}

// Initiates the barometer update process.
// Starts a non-blocking read of the FIFO fill level to determine how much data is available.
static bool icp201xxReadUP(baroDev_t *baro)
{
    // Rate limit reads
    uint32_t now = millis();
    if (cmp32(now, lastFifoReadTime) < ICP201XX_MIN_READ_INTERVAL_MS) {
        return true;  // Too soon, skip this read
    }

    // Only start a new read if idle
    if (readState != ICP201XX_STATE_IDLE && readState != ICP201XX_STATE_DATA_READY) {
        return true;  // Previous read still in progress
    }

    // Check if bus is busy
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    lastFifoReadTime = now;

    // Start non-blocking FIFO fill read
    if (icp201xxReadRegStart(&baro->dev, ICP201XX_REG_FIFO_FILL, 1)) {
        readState = ICP201XX_STATE_READ_FIFO_FILL_STARTED;
    }

    return true;
}

// Checks if a sample from the FIFO buffer is valid
// Returns true if the sample is valid, false if it contains invalid data patterns
static bool icp201xxIsValidSample(uint8_t offset)
{
    // All 0xFF = uninitialized/error
    if (fifoDataBuf[offset + 0] == 0xFF && fifoDataBuf[offset + 1] == 0xFF &&
        fifoDataBuf[offset + 2] == 0xFF && fifoDataBuf[offset + 3] == 0xFF &&
        fifoDataBuf[offset + 4] == 0xFF && fifoDataBuf[offset + 5] == 0xFF) {
        return false;
    }

    // All 0x00 = sensor not measuring
    if (fifoDataBuf[offset + 0] == 0x00 && fifoDataBuf[offset + 1] == 0x00 &&
        fifoDataBuf[offset + 2] == 0x00 && fifoDataBuf[offset + 3] == 0x00 &&
        fifoDataBuf[offset + 4] == 0x00 && fifoDataBuf[offset + 5] == 0x00) {
        return false;
    }

    return true;
}

static void icp201xxCalculate(int32_t *pressure, int32_t *temperature)
{
    // Process data if available
    if (readState == ICP201XX_STATE_DATA_READY) {
        uint32_t currentTime = millis();

        // Parse all samples from FIFO and add to trend buffer
        for (uint8_t i = 0; i < fifoSampleCount; i++) {
            uint8_t offset = i * 6;

            if (icp201xxIsValidSample(offset)) {
                // Parse pressure (20-bit signed, LSB first)
                int32_t rawPress = ((int32_t)(fifoDataBuf[offset + 2] & 0x0F) << 16) |
                                   ((int32_t)fifoDataBuf[offset + 1] << 8) |
                                   fifoDataBuf[offset + 0];
                if (rawPress & 0x080000) rawPress |= 0xFFF00000; // Sign extend

                // Parse temperature (20-bit signed, LSB first)
                int32_t rawTemp = ((int32_t)(fifoDataBuf[offset + 5] & 0x0F) << 16) |
                                  ((int32_t)fifoDataBuf[offset + 4] << 8) |
                                  fifoDataBuf[offset + 3];
                if (rawTemp & 0x080000) rawTemp |= 0xFFF00000; // Sign extend

                // Check if both are not zero
                if (rawPress != 0 || rawTemp != 0) {
                    // Convert to physical units
                    float sensorPressure = ((float)rawPress * 40000.0f / 131072.0f) + 70000.0f;
                    float sensorTemperature = ((float)rawTemp * 65.0f / 262144.0f) + 25.0f;

                    // Sanity check
                    if (sensorPressure >= 30000.0f && sensorPressure <= 110000.0f &&
                        sensorTemperature >= -40.0f && sensorTemperature <= 85.0f) {

                        // Add to circular buffer for trend analysis
                        sampleBuffer.samples[sampleBuffer.head].pressure = sensorPressure;
                        sampleBuffer.samples[sampleBuffer.head].temperature = sensorTemperature;
                        sampleBuffer.samples[sampleBuffer.head].timestamp = currentTime;

                        sampleBuffer.head = (sampleBuffer.head + 1) % ICP201XX_SAMPLE_BUFFER_SIZE;
                        if (sampleBuffer.count < ICP201XX_SAMPLE_BUFFER_SIZE) {
                            sampleBuffer.count++;
                        }
                    }
                }
            }
        }

        // Reset state for next read
        readState = ICP201XX_STATE_IDLE;
        fifoSampleCount = 0;
    }

    // Perform trend-weighted averaging if we have enough samples
    if (sampleBuffer.count >= ICP201XX_MIN_SAMPLES_FOR_TREND) {
        // Calculate statistics
        float pSum = 0.0f, tSum = 0.0f;
        float pMin = 999999.0f, pMax = 0.0f;

        uint8_t oldestIdx = (sampleBuffer.head + ICP201XX_SAMPLE_BUFFER_SIZE - sampleBuffer.count) % ICP201XX_SAMPLE_BUFFER_SIZE;
        uint8_t newestIdx = (sampleBuffer.head + ICP201XX_SAMPLE_BUFFER_SIZE - 1) % ICP201XX_SAMPLE_BUFFER_SIZE;

        for (uint8_t i = 0; i < sampleBuffer.count; i++) {
            uint8_t idx = (oldestIdx + i) % ICP201XX_SAMPLE_BUFFER_SIZE;
            float p = sampleBuffer.samples[idx].pressure;
            pSum += p;
            tSum += sampleBuffer.samples[idx].temperature;
            if (p < pMin) pMin = p;
            if (p > pMax) pMax = p;
        }

        float pMean = pSum / sampleBuffer.count;
        float tMean = tSum / sampleBuffer.count;

        // Calculate trend (slope in Pa/ms)
        float pOldest = sampleBuffer.samples[oldestIdx].pressure;
        float pNewest = sampleBuffer.samples[newestIdx].pressure;
        uint32_t tOldest = sampleBuffer.samples[oldestIdx].timestamp;
        uint32_t tNewest = sampleBuffer.samples[newestIdx].timestamp;

        float timeSpan = (float)(tNewest - tOldest);
        float slope = 0.0f;
        if (timeSpan > 0.0f) {
            slope = (pNewest - pOldest) / timeSpan;
        }

        // Apply trend-weighted averaging
        float finalPressure = pMean;

        if (slope > ICP201XX_TREND_THRESHOLD) {
            // Rising trend - bias toward higher values
            // Bias factor: 0 to 0.5 based on slope strength
            float biasFactor = slope / ICP201XX_TREND_THRESHOLD * 0.3f;
            if (biasFactor > 0.5f) biasFactor = 0.5f;
            finalPressure = pMean + (pMax - pMean) * biasFactor;
        } else if (slope < -ICP201XX_TREND_THRESHOLD) {
            // Falling trend - bias toward lower values
            float biasFactor = (-slope) / ICP201XX_TREND_THRESHOLD * 0.3f;
            if (biasFactor > 0.5f) biasFactor = 0.5f;
            finalPressure = pMean + (pMin - pMean) * biasFactor;
        }
        // else: stable, use mean

        // Store as last valid values
        lastValidPressure = (int32_t)finalPressure;  // Pa
        lastValidTemperature = (int32_t)(tMean * 100.0f);  // Centi-degrees

        // Keep only the most recent sample for continuity
        if (sampleBuffer.count > 1) {
            baroSample_t newest = sampleBuffer.samples[newestIdx];
            sampleBuffer.samples[0] = newest;
            sampleBuffer.head = 1;
            sampleBuffer.count = 1;
        }
    }

    // Always return last valid values (even when buffer is empty)
    *pressure = lastValidPressure;
    *temperature = lastValidTemperature;
}

#endif // USE_BARO_ICP201XX
