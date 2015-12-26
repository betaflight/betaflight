/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

#include "platform.h"
#include "build_config.h"
#include "debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/utils.h"

#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/sensor.h"
#include "drivers/compass.h"

#include "drivers/gps.h"
#include "drivers/gps_i2cnav.h"

#include "sensors/sensors.h"
#include "sensors/compass.h"

#include "io/serial.h"
#include "io/display.h"
#include "io/gps.h"
#include "io/gps_private.h"

#include "flight/gps_conversion.h"
#include "flight/pid.h"
#include "flight/hil.h"
#include "flight/navigation_rewrite.h"

#include "config/config.h"
#include "config/runtime_config.h"

#ifdef GPS

// GPS timeout for wrong baud rate/disconnection/etc in milliseconds (default 1500 ms)
#define GPS_TIMEOUT         (1500)
#define GPS_INIT_DELAY      (200)
#define GPS_BUS_INIT_DELAY  (500)

typedef enum {
    GPS_TYPE_NA,        // Not available
    GPS_TYPE_SERIAL,    // Serial connection (UART)
    GPS_TYPE_BUS        // Bus connection (I2C/SPI)
} gpsProviderType_e;

typedef struct {
    gpsProviderType_e   type;
    portMode_t          portMode;      // Port mode RX/TX (only for serial based)
    bool                hasCompass;    // Has a compass (NAZA)
    bool (*detect)(void);
    bool (*read)(void);
} gpsProviderDescriptor_t;

// GPS public data
gpsReceiverData_t gpsState;
gpsStatistics_t   gpsStats;
gpsSolutionData_t gpsSol;

// Map gpsBaudRate_e index to baudRate_e
baudRate_e gpsToSerialBaudRate[GPS_BAUDRATE_COUNT] = { BAUD_115200, BAUD_57600, BAUD_38400, BAUD_19200, BAUD_9600 };

static gpsProviderDescriptor_t  gpsProviders[GPS_PROVIDER_COUNT] = {
    /* NMEA GPS */
#ifdef GPS_PROTO_NMEA
    { GPS_TYPE_SERIAL, MODE_RX, false, NULL, &gpsHandleNMEA },
#else
    { GPS_TYPE_NA, 0, false,  NULL, NULL },
#endif

    /* UBLOX binary */
#ifdef GPS_PROTO_UBLOX
    { GPS_TYPE_SERIAL, MODE_RXTX, false,  NULL, gpsHandleUBLOX },
#else
    { GPS_TYPE_NA, 0, false,  NULL, NULL },
#endif

    /* MultiWii I2C-NAV module */
#ifdef GPS_PROTO_I2C_NAV
    { GPS_TYPE_BUS, 0, false, &gpsDetectI2CNAV, &gpsHandleI2CNAV },
#else
    { GPS_TYPE_NA, 0, false,  NULL, NULL },
#endif
};

void gpsSetState(gpsState_e state)
{
    gpsState.state = state;
    gpsState.lastStateSwitchMs = millis();
}

static void gpsHandleProtocol(void)
{
    bool newDataReceived = false;

    // Call protocol-specific code
    if (gpsProviders[gpsState.gpsConfig->provider].read) {
        newDataReceived = gpsProviders[gpsState.gpsConfig->provider].read();
    }

    // Received new update for solution data
    if (newDataReceived) {
        // Set GPS fix flag
        if (gpsSol.flags.fix3D)
            ENABLE_STATE(GPS_FIX);
        else
            DISABLE_STATE(GPS_FIX);

        // Emulate velNE if not available (don't set validVelNE flag to avoid confusion)
        bool validVelNE = gpsSol.flags.validVelNE;
        if (!validVelNE) {
            float gpsHeadingRad = gpsSol.groundCourse * M_PIf / 1800.0f;

            gpsSol.velNED[0] = gpsSol.groundSpeed * cos_approx(gpsHeadingRad);
            gpsSol.velNED[1] = gpsSol.groundSpeed * sin_approx(gpsHeadingRad);
            gpsSol.velNED[2] = 0;
            validVelNE = true;
        }

        // Update GPS coordinates etc
        sensorsSet(SENSOR_GPS);
        onNewGPSData(gpsSol.llh.lat, gpsSol.llh.lon, gpsSol.llh.alt, gpsSol.velNED[0], gpsSol.velNED[1], gpsSol.velNED[2], validVelNE, gpsSol.flags.validVelD, gpsSol.hdop);

        // Update timeout
        gpsState.lastLastMessageMs = gpsState.lastMessageMs;
        gpsState.lastMessageMs = millis();

        // Update statistics
        gpsStats.lastMessageDt = gpsState.lastMessageMs - gpsState.lastLastMessageMs;
    }
}

static void gpsResetSolution(void)
{
    // Clear satellites in view information, if we use I2C driver this is not used
    gpsSol.numCh = 0;
    for (int i = 0; i < GPS_SV_MAXSATS; i++){
        gpsSol.svInfo[i].chn = 0;
        gpsSol.svInfo[i].svid = 0;
        gpsSol.svInfo[i].quality = 0;
        gpsSol.svInfo[i].cno = 0;
    }

    gpsSol.flags.fix3D = 0;
    gpsSol.flags.validVelNE = 0;
    gpsSol.flags.validVelD = 0;
    gpsSol.flags.validMag = 0;
}

void gpsPreInit(gpsConfig_t *initialGpsConfig)
{
    // Make sure gpsProvider is known when gpsMagDetect is called
    gpsState.gpsConfig = initialGpsConfig;
}

void gpsInit(serialConfig_t *initialSerialConfig, gpsConfig_t *initialGpsConfig)
{
    gpsState.serialConfig = initialSerialConfig;
    gpsState.gpsConfig = initialGpsConfig;
    gpsState.baudrateIndex = 0;

    gpsStats.errors = 0;
    gpsStats.timeouts = 0;

    gpsResetSolution();

    // init gpsData structure. if we're not actually enabled, don't bother doing anything else
    gpsState.autoConfigStep = 0;
    gpsState.lastMessageMs = millis();
    gpsSetState(GPS_UNKNOWN);

    if (gpsProviders[gpsState.gpsConfig->provider].type == GPS_TYPE_BUS) {
        gpsSetState(GPS_INITIALIZING);
        return;
    }

    if (gpsProviders[gpsState.gpsConfig->provider].type == GPS_TYPE_SERIAL) {
        serialPortConfig_t * gpsPortConfig = findSerialPortConfig(FUNCTION_GPS);
        if (!gpsPortConfig) {
            featureClear(FEATURE_GPS);
        }
        else {
            while (gpsToSerialBaudRate[gpsState.baudrateIndex] != gpsPortConfig->gps_baudrateIndex) {
                gpsState.baudrateIndex++;
                if (gpsState.baudrateIndex >= GPS_BAUDRATE_COUNT) {
                    gpsState.baudrateIndex = 0;
                    break;
                }
            }

            // Start autoBaud tests at desired baud rate
            gpsState.autoBaudrateIndex = gpsState.baudrateIndex;

            portMode_t mode = gpsProviders[gpsState.gpsConfig->provider].portMode;

            // no callback - buffer will be consumed in gpsThread()
            gpsState.gpsPort = openSerialPort(gpsPortConfig->identifier, FUNCTION_GPS, NULL, gpsToSerialBaudRate[gpsState.baudrateIndex], mode, SERIAL_NOT_INVERTED);

            if (!gpsState.gpsPort) {
                featureClear(FEATURE_GPS);
            }
            else {
                gpsSetState(GPS_INITIALIZING);
                return;
            }
        }
    }
}

void gpsThread(void)
{
    // Serial-based GPS
    if ((gpsProviders[gpsState.gpsConfig->provider].type == GPS_TYPE_SERIAL) && (gpsState.gpsPort != NULL)) {
        switch (gpsState.state) {
        default:
        case GPS_INITIALIZING:
            // Handle protocol - if autobaud will switch to autoBaudrateIndex, send init string and switch to desired baud
            gpsResetSolution();
            gpsHandleProtocol();
            DISABLE_STATE(GPS_FIX);
            gpsSetState(GPS_CHANGE_BAUD);
            break;

        case GPS_CHANGE_BAUD:
            // Wait for GPS_INIT_DELAY before switching to required baud rate
            if ((millis() - gpsState.lastStateSwitchMs) >= GPS_INIT_DELAY) {
                // Switch to required serial port baud
                serialSetBaudRate(gpsState.gpsPort, baudRates[gpsToSerialBaudRate[gpsState.baudrateIndex]]);
                gpsState.autoConfigStep = 0;
                gpsState.autoConfigPosition = 0;
                gpsState.lastMessageMs = millis();
                gpsSetState(GPS_CONFIGURE);
            }
            break;

        case GPS_CONFIGURE:
        case GPS_RECEIVING_DATA:
            gpsHandleProtocol();
            if ((millis() - gpsState.lastMessageMs) > GPS_TIMEOUT) {
                // remove GPS from capability
                sensorsClear(SENSOR_GPS);
                DISABLE_STATE(GPS_FIX);
                gpsSetState(GPS_LOST_COMMUNICATION);
            }
            break;

        case GPS_LOST_COMMUNICATION:
            gpsStats.timeouts++;
            // Handle autobaud
            if (gpsState.gpsConfig->autoBaud != GPS_AUTOBAUD_OFF) {
                gpsState.autoBaudrateIndex++;
                gpsState.autoBaudrateIndex %= GPS_BAUDRATE_COUNT;
            }
            gpsSetState(GPS_INITIALIZING);
            break;
        }
    }
    // Driver-based GPS (I2C)
    else if (gpsProviders[gpsState.gpsConfig->provider].type == GPS_TYPE_BUS) {
        switch (gpsState.state) {
        default:
        case GPS_INITIALIZING:
        case GPS_CHANGE_BAUD:
            // Detect GPS unit
            if ((millis() - gpsState.lastStateSwitchMs) >= GPS_BUS_INIT_DELAY) {
                gpsResetSolution();

                if (gpsProviders[gpsState.gpsConfig->provider].detect && gpsProviders[gpsState.gpsConfig->provider].detect()) {
                    gpsState.autoConfigStep = 0;
                    gpsState.autoConfigPosition = 0;
                    gpsState.lastMessageMs = millis();
                    sensorsSet(SENSOR_GPS);
                    gpsSetState(GPS_CONFIGURE);
                }
                else {
                    sensorsClear(SENSOR_GPS);
                }
            }
            break;

        case GPS_CONFIGURE:
        case GPS_RECEIVING_DATA:
            gpsHandleProtocol();
            if (millis() - gpsState.lastMessageMs > GPS_TIMEOUT) {
                // remove GPS from capability
                gpsSetState(GPS_LOST_COMMUNICATION);
            }
            break;

        case GPS_LOST_COMMUNICATION:
            // No valid data from GPS unit, cause re-init and re-detection
            gpsStats.timeouts++;
            DISABLE_STATE(GPS_FIX);
            gpsSetState(GPS_INITIALIZING);
            break;
        }
    }
    else {
        // GPS_TYPE_NA
    }
}

void gpsEnablePassthrough(serialPort_t *gpsPassthroughPort)
{
    waitForSerialPortToFinishTransmitting(gpsState.gpsPort);
    waitForSerialPortToFinishTransmitting(gpsPassthroughPort);

    if(!(gpsState.gpsPort->mode & MODE_TX))
    serialSetMode(gpsState.gpsPort, gpsState.gpsPort->mode | MODE_TX);

    LED0_OFF;
    LED1_OFF;

    char c;
    while(1) {
        if (serialRxBytesWaiting(gpsState.gpsPort)) {
            LED0_ON;
            c = serialRead(gpsState.gpsPort);
            serialWrite(gpsPassthroughPort, c);
            LED0_OFF;
        }
        if (serialRxBytesWaiting(gpsPassthroughPort)) {
            LED1_ON;
            serialWrite(gpsState.gpsPort, serialRead(gpsPassthroughPort));
            LED1_OFF;
        }
    }
}

void updateGpsIndicator(uint32_t currentTime)
{
    static uint32_t GPSLEDTime;
    if ((int32_t)(currentTime - GPSLEDTime) >= 0 && (gpsSol.numSat>= 5)) {
        GPSLEDTime = currentTime + 150000;
        LED1_TOGGLE;
    }
}

/* Support for built-in magnetometer accessible via the native GPS protocol (i.e. NAZA) */
void gpsMagInit(void)
{
}

bool gpsMagRead(int16_t *magData)
{
    magData[X] = gpsSol.magData[0];
    magData[Y] = gpsSol.magData[1];
    magData[Z] = gpsSol.magData[2];
    return gpsSol.flags.validMag;
}

bool gpsMagDetect(mag_t *mag)
{
    if (!(feature(FEATURE_GPS) && gpsProviders[gpsState.gpsConfig->provider].hasCompass))
        return false;

    mag->init = gpsMagInit;
    mag->read = gpsMagRead;
    return true;
}

#endif