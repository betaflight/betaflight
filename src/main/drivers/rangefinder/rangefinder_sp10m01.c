#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_RANGEFINDER_SP10M01

#include "common/utils.h"

#include "io/serial.h"

#include "drivers/time.h"
#include "drivers/rangefinder/rangefinder.h"
#include "drivers/rangefinder/rangefinder_sp10m01.h"
#define SP10M01_BAUDRATE      460800
#define SP10M01_PACKET_HEADER 0x5C
#define SP10M01_PACKET_LENGTH 4
#define SP10M01_TIMEOUT_MS    200
#define SP10M01_MAX_RANGE_CM  1000

static serialPort_t *sp10m01SerialPort = NULL;
static uint8_t packetBuffer[SP10M01_PACKET_LENGTH];
static uint8_t bufferIndex;
static int32_t sp10m01Distance = RANGEFINDER_OUT_OF_RANGE;
static bool sp10m01HasNewData = false;

static void rangefinderSP10M01Init(rangefinderDev_t *dev)
{
    UNUSED(dev);
    bufferIndex = 0;
    sp10m01Distance = RANGEFINDER_OUT_OF_RANGE;
    sp10m01HasNewData = false;
}

static void rangefinderSP10M01Update(rangefinderDev_t *dev)
{
    UNUSED(dev);

    int bytes = serialRxBytesWaiting(sp10m01SerialPort);

    while (serialRxBytesWaiting(sp10m01SerialPort)) {
        uint8_t b = serialRead(sp10m01SerialPort);
        debug[1] = b;

        if (bufferIndex == 0 && b != SP10M01_PACKET_HEADER) {
            
            continue;
        }

        packetBuffer[bufferIndex++] = b;

        if (bufferIndex == SP10M01_PACKET_LENGTH) {
            uint8_t header   = packetBuffer[0];
            uint8_t dist_low = packetBuffer[1];
            uint8_t dist_high= packetBuffer[2];
            uint8_t checksum = packetBuffer[3];

            uint8_t calculated_crc = 0x5B - (header + dist_low + dist_high);

            if (calculated_crc == checksum) {
                uint16_t distance_mm = dist_low | (dist_high << 8);
                if (distance_mm >= 30 && distance_mm <= 10000) {
                    sp10m01Distance = (int32_t)(distance_mm / 10);
                } else {
                    sp10m01Distance = RANGEFINDER_OUT_OF_RANGE;
                }
                sp10m01HasNewData = true;
            }
            bufferIndex = 0;
        }
    }
}
static int32_t rangefinderSP10M01Read(rangefinderDev_t *dev)
{
    UNUSED(dev);
    if (!sp10m01HasNewData) {
        return RANGEFINDER_NO_NEW_DATA;
    }
    sp10m01HasNewData = false;
    return sp10m01Distance;
}

bool rangefinderSP10M01Detect(rangefinderDev_t *dev)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_LIDAR_TF);
    if (!portConfig) {
        return false;
    }

    sp10m01SerialPort = openSerialPort(portConfig->identifier, 
                                       FUNCTION_LIDAR_TF, 
                                       NULL, NULL, 
                                       460800,     
                                       MODE_RX,  
                                       0);        

    if (!sp10m01SerialPort) return false;

    dev->init = &rangefinderSP10M01Init;
    dev->update = &rangefinderSP10M01Update;
    dev->read = &rangefinderSP10M01Read;

    dev->delayMs = 10;
    dev->maxRangeCm = SP10M01_MAX_RANGE_CM;
    dev->detectionConeDeciDegrees = 200;
    dev->detectionConeExtendedDeciDegrees = 250;

    return true;
}
#endif

