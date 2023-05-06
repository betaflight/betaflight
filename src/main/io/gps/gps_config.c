
#include <ctype.h>
#include <string.h>
#include <math.h>
#include "gps_config.h"

#include "platform.h"

#ifdef USE_GPS

ubloxVersion_e ubloxVersion = UNDEF;

static void ubloxDetectVersion(serialPort_t *instance) {
    ubloxVersion = M8;
}

static void ubloxSendMessage(const uint8_t *data, uint8_t len, gpsData_t *gpsData) {

    switch (ubloxVersion)
    {
    case M8:
        uint8_t checksumA = 0, checksumB = 0;
        serialWrite(gpsPort, data[0]);
        serialWrite(gpsPort, data[1]);
        ubloxSendDataUpdateChecksum(&data[2], len - 2, &checksumA, &checksumB);
        serialWrite(gpsPort, checksumA);
        serialWrite(gpsPort, checksumB);

        // Save state for ACK waiting
        gpsData->ackWaitingMsgId = data[3]; //save message id for ACK
        gpsData->ackTimeoutCounter = 0;
        gpsData->ackState = UBLOX_ACK_WAITING;
        break;
    default:
        break;
    }
}

#endif