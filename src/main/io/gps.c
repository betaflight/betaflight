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
#include <math.h>

#include "platform.h"
#include "build/build_config.h"


#ifdef GPS

#include "build/debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/utils.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/compass/compass.h"
#include "drivers/light_led.h"
#include "drivers/serial.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "sensors/sensors.h"
#include "sensors/compass.h"

#include "io/serial.h"
#include "io/gps.h"
#include "io/gps_private.h"

#include "navigation/navigation.h"

#include "config/feature.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

// GPS timeout for wrong baud rate/disconnection/etc in milliseconds (default 2000 ms)
#define GPS_TIMEOUT             (2000)
#define GPS_BAUD_CHANGE_DELAY   (200)
#define GPS_INIT_DELAY          (500)
#define GPS_BUS_INIT_DELAY      (500)
#define GPS_BOOT_DELAY          (2000)

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
    { GPS_TYPE_SERIAL, MODE_RXTX, false,  NULL, &gpsHandleUBLOX },
#else
    { GPS_TYPE_NA, 0, false,  NULL, NULL },
#endif

    /* MultiWii I2C-NAV module */
#ifdef GPS_PROTO_I2C_NAV
    { GPS_TYPE_BUS, 0, false, &gpsDetectI2CNAV, &gpsHandleI2CNAV },
#else
    { GPS_TYPE_NA, 0, false,  NULL, NULL },
#endif

    /* NAZA GPS module */
#ifdef GPS_PROTO_NAZA
    { GPS_TYPE_SERIAL, MODE_RX, true,  NULL, &gpsHandleNAZA },
#else
    { GPS_TYPE_NA, 0, false,  NULL, NULL },
#endif

    /* UBLOX7PLUS binary */
#ifdef GPS_PROTO_UBLOX_NEO7PLUS
    { GPS_TYPE_SERIAL, MODE_RXTX, false,  NULL, &gpsHandleUBLOX },
#else
    { GPS_TYPE_NA, 0, false,  NULL, NULL },
#endif

    /* MTK GPS */
#ifdef GPS_PROTO_MTK
    { GPS_TYPE_SERIAL, MODE_RXTX, false, NULL, &gpsHandleMTK },
#else
    { GPS_TYPE_NA, 0, false,  NULL, NULL },
#endif

};

PG_REGISTER_WITH_RESET_TEMPLATE(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);

PG_RESET_TEMPLATE(gpsConfig_t, gpsConfig,
    .provider = GPS_UBLOX,
    .sbasMode = SBAS_NONE,
    .autoConfig = GPS_AUTOCONFIG_ON,
    .autoBaud = GPS_AUTOBAUD_ON,
    .dynModel = GPS_DYNMODEL_AIR_1G,
    .gpsMinSats = 6
);

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
        // Set GPS fix flag only if we have 3D fix
        if (gpsSol.fixType == GPS_FIX_3D && gpsSol.numSat >= gpsConfig()->gpsMinSats) {
            ENABLE_STATE(GPS_FIX);
        }
        else {
            /* When no fix available - reset flags as well */
            gpsSol.flags.validVelNE = 0;
            gpsSol.flags.validVelD = 0;
            gpsSol.flags.validEPE = 0;

            DISABLE_STATE(GPS_FIX);
        }

        // Update GPS coordinates etc
        sensorsSet(SENSOR_GPS);
        onNewGPSData();

        // Update timeout
        gpsState.lastLastMessageMs = gpsState.lastMessageMs;
        gpsState.lastMessageMs = millis();

        // Update statistics
        gpsStats.lastMessageDt = gpsState.lastMessageMs - gpsState.lastLastMessageMs;
    }
}

static void gpsResetSolution(void)
{
    gpsSol.eph = 9999;
    gpsSol.epv = 9999;

    gpsSol.fixType = GPS_NO_FIX;

    gpsSol.flags.validVelNE = 0;
    gpsSol.flags.validVelD = 0;
    gpsSol.flags.validMag = 0;
    gpsSol.flags.validEPE = 0;
}

void gpsPreInit(void)
{
    // Make sure gpsProvider is known when gpsMagDetect is called
    gpsState.gpsConfig = gpsConfig();
}

void gpsInit(void)
{
    gpsState.serialConfig = serialConfig();
    gpsState.gpsConfig = gpsConfig();
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

            portMode_t mode = gpsProviders[gpsState.gpsConfig->provider].portMode;

            // no callback - buffer will be consumed in gpsThread()
            gpsState.gpsPort = openSerialPort(gpsPortConfig->identifier, FUNCTION_GPS, NULL, baudRates[gpsToSerialBaudRate[gpsState.baudrateIndex]], mode, SERIAL_NOT_INVERTED);

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

#ifdef USE_FAKE_GPS
static void gpsFakeGPSUpdate(void)
{
    if (millis() - gpsState.lastMessageMs > 100) {
        gpsSol.fixType = GPS_FIX_3D;
        gpsSol.numSat = 6;
        gpsSol.llh.lat = 509102311;
        gpsSol.llh.lon = -15349744;
        gpsSol.llh.alt = 0;
        gpsSol.groundSpeed = 0;
        gpsSol.groundCourse = 0;
        gpsSol.velNED[X] = 0;
        gpsSol.velNED[Y] = 0;
        gpsSol.velNED[Z] = 0;
        gpsSol.flags.validVelNE = 1;
        gpsSol.flags.validVelD = 1;
        gpsSol.flags.validEPE = 1;
        gpsSol.eph = 100;
        gpsSol.epv = 100;

        ENABLE_STATE(GPS_FIX);
        sensorsSet(SENSOR_GPS);
        onNewGPSData();

        gpsState.lastLastMessageMs = gpsState.lastMessageMs;
        gpsState.lastMessageMs = millis();

        gpsSetState(GPS_RECEIVING_DATA);
    }
}
#endif

// Finish baud rate change sequence - wait for TX buffer to empty and switch to the desired port speed
void gpsFinalizeChangeBaud(void)
{
    if ((gpsProviders[gpsState.gpsConfig->provider].type == GPS_TYPE_SERIAL) && (gpsState.gpsPort != NULL)) {
        // Wait for GPS_INIT_DELAY before switching to required baud rate
        if ((millis() - gpsState.lastStateSwitchMs) >= GPS_BAUD_CHANGE_DELAY && isSerialTransmitBufferEmpty(gpsState.gpsPort)) {
            // Switch to required serial port baud
            serialSetBaudRate(gpsState.gpsPort, baudRates[gpsToSerialBaudRate[gpsState.baudrateIndex]]);
            gpsState.lastMessageMs = millis();
            gpsSetState(GPS_CHECK_VERSION);
        }
    }
}

uint16_t gpsConstrainEPE(uint32_t epe)
{
    return (epe > 99999) ? 9999 : epe; // max 99.99m error
}

uint16_t gpsConstrainHDOP(uint32_t hdop)
{
    return (hdop > 99999) ? 9999 : hdop; // max 99.99m error
}

void gpsThread(void)
{
    /* Extra delay for at least 2 seconds after booting to give GPS time to initialise */
    if (!isMPUSoftReset() && (millis() < GPS_BOOT_DELAY)) {
        sensorsClear(SENSOR_GPS);
        DISABLE_STATE(GPS_FIX);
        return;
    }

#ifdef USE_FAKE_GPS
    gpsFakeGPSUpdate();
#else

    // Serial-based GPS
    if ((gpsProviders[gpsState.gpsConfig->provider].type == GPS_TYPE_SERIAL) && (gpsState.gpsPort != NULL)) {
        switch (gpsState.state) {
        default:
        case GPS_INITIALIZING:
            if ((millis() - gpsState.lastStateSwitchMs) >= GPS_INIT_DELAY) {
                // Reset internals
                DISABLE_STATE(GPS_FIX);
                gpsSol.fixType = GPS_NO_FIX;

                gpsState.hwVersion = 0;
                gpsState.autoConfigStep = 0;
                gpsState.autoConfigPosition = 0;
                gpsState.autoBaudrateIndex = 0;

                // Reset solution
                gpsResetSolution();

                // Call protocol handler - switch to next state is done there
                gpsHandleProtocol();
            }
            break;

        case GPS_CHANGE_BAUD:
            // Call protocol handler - switch to next state is done there
            gpsHandleProtocol();
            break;

        case GPS_CHECK_VERSION:
        case GPS_CONFIGURE:
        case GPS_RECEIVING_DATA:
            gpsHandleProtocol();
            if ((millis() - gpsState.lastMessageMs) > GPS_TIMEOUT) {
                // Check for GPS timeout
                sensorsClear(SENSOR_GPS);
                DISABLE_STATE(GPS_FIX);
                gpsSol.fixType = GPS_NO_FIX;

                gpsSetState(GPS_LOST_COMMUNICATION);
            }
            break;

        case GPS_LOST_COMMUNICATION:
            gpsStats.timeouts++;
            // Handle autobaud - switch to next port baud rate
            if (gpsState.gpsConfig->autoBaud != GPS_AUTOBAUD_OFF) {
                gpsState.baudrateIndex++;
                gpsState.baudrateIndex %= GPS_BAUDRATE_COUNT;
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
            // Detect GPS unit
            if ((millis() - gpsState.lastStateSwitchMs) >= GPS_BUS_INIT_DELAY) {
                gpsResetSolution();

                if (gpsProviders[gpsState.gpsConfig->provider].detect && gpsProviders[gpsState.gpsConfig->provider].detect()) {
                    gpsState.hwVersion = 0;
                    gpsState.autoConfigStep = 0;
                    gpsState.autoConfigPosition = 0;
                    gpsState.lastMessageMs = millis();
                    sensorsSet(SENSOR_GPS);
                    gpsSetState(GPS_CHANGE_BAUD);
                }
                else {
                    sensorsClear(SENSOR_GPS);
                }
            }
            break;

        case GPS_CHANGE_BAUD:
        case GPS_CHECK_VERSION:
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
            gpsSol.fixType = GPS_NO_FIX;

            gpsSetState(GPS_INITIALIZING);
            break;
        }
    }
    else {
        // GPS_TYPE_NA
    }
#endif
}

void gpsEnablePassthrough(serialPort_t *gpsPassthroughPort)
{
    waitForSerialPortToFinishTransmitting(gpsState.gpsPort);
    waitForSerialPortToFinishTransmitting(gpsPassthroughPort);

    if (!(gpsState.gpsPort->mode & MODE_TX))
    serialSetMode(gpsState.gpsPort, gpsState.gpsPort->mode | MODE_TX);

    LED0_OFF;
    LED1_OFF;

    char c;
    while (1) {
        if (serialRxBytesWaiting(gpsState.gpsPort)) {
            LED0_ON;
            c = serialRead(gpsState.gpsPort);
            serialWrite(gpsPassthroughPort, c);
            LED0_OFF;
        }
        if (serialRxBytesWaiting(gpsPassthroughPort)) {
            LED1_ON;
            c = serialRead(gpsPassthroughPort);
            serialWrite(gpsState.gpsPort, c);
            LED1_OFF;
        }
    }
}

void updateGpsIndicator(timeUs_t currentTimeUs)
{
    static timeUs_t GPSLEDTime;
    if ((int32_t)(currentTimeUs - GPSLEDTime) >= 0 && (gpsSol.numSat>= 5)) {
        GPSLEDTime = currentTimeUs + 150000;
        LED1_TOGGLE;
    }
}

/* Support for built-in magnetometer accessible via the native GPS protocol (i.e. NAZA) */
bool gpsMagInit(magDev_t *magDev)
{
    UNUSED(magDev);
    return true;
}

bool gpsMagRead(magDev_t *magDev)
{
    magDev->magADCRaw[X] = gpsSol.magData[0];
    magDev->magADCRaw[Y] = gpsSol.magData[1];
    magDev->magADCRaw[Z] = gpsSol.magData[2];
    return gpsSol.flags.validMag;
}

bool gpsMagDetect(magDev_t *mag)
{
    if (!(feature(FEATURE_GPS) && gpsProviders[gpsState.gpsConfig->provider].hasCompass))
        return false;

    if (gpsProviders[gpsState.gpsConfig->provider].type == GPS_TYPE_SERIAL && (!findSerialPortConfig(FUNCTION_GPS))) {
        return false;
    }

    mag->init = gpsMagInit;
    mag->read = gpsMagRead;
    return true;
}

bool isGPSHealthy(void)
{
    return true;
}
#endif
