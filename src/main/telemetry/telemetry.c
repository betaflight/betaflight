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
#include <stdlib.h>

#include "platform.h"

#ifdef TELEMETRY

#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "io/serial.h"

#include "rx/rx.h"
#include "io/rc_controls.h"

#include "config/runtime_config.h"
#include "config/config.h"

#include "telemetry/telemetry.h"
#include "telemetry/frsky.h"
#include "telemetry/hott.h"
#include "telemetry/msp.h"
#include "telemetry/smartport.h"


static bool isTelemetryConfigurationValid = false; // flag used to avoid repeated configuration checks
static bool telemetryEnabled = false;
static bool telemetryPortIsShared;

static telemetryConfig_t *telemetryConfig;

void useTelemetryConfig(telemetryConfig_t *telemetryConfigToUse)
{
    telemetryConfig = telemetryConfigToUse;
}

bool isTelemetryProviderFrSky(void)
{
    return telemetryConfig->telemetry_provider == TELEMETRY_PROVIDER_FRSKY;
}

bool isTelemetryProviderHoTT(void)
{
    return telemetryConfig->telemetry_provider == TELEMETRY_PROVIDER_HOTT;
}

bool isTelemetryProviderMSP(void)
{
    return telemetryConfig->telemetry_provider == TELEMETRY_PROVIDER_MSP;
}

bool isTelemetryProviderSmartPort(void)
{
    return telemetryConfig->telemetry_provider == TELEMETRY_PROVIDER_SMARTPORT;
}

bool isTelemetryPortShared(void)
{
    return telemetryPortIsShared;
}

bool canUseTelemetryWithCurrentConfiguration(void)
{
    if (!feature(FEATURE_TELEMETRY)) {
        return false;
    }

    if (telemetryConfig->telemetry_provider != TELEMETRY_PROVIDER_SMARTPORT && !canOpenSerialPort(FUNCTION_TELEMETRY)) {
        return false;
    }

    if (telemetryConfig->telemetry_provider == TELEMETRY_PROVIDER_SMARTPORT && !canOpenSerialPort(FUNCTION_SMARTPORT_TELEMETRY)) {
        return false;
    }

    return true;
}

void telemetryInit()
{
    if (isTelemetryProviderSmartPort()) {
        telemetryPortIsShared = isSerialPortFunctionShared(FUNCTION_SMARTPORT_TELEMETRY, FUNCTION_MSP);
    } else {
        telemetryPortIsShared = isSerialPortFunctionShared(FUNCTION_TELEMETRY, FUNCTION_MSP);
    }
    isTelemetryConfigurationValid = canUseTelemetryWithCurrentConfiguration();

    if (isTelemetryProviderFrSky()) {
        initFrSkyTelemetry(telemetryConfig);
    }

    if (isTelemetryProviderHoTT()) {
        initHoTTTelemetry(telemetryConfig);
    }

    if (isTelemetryProviderMSP()) {
        initMSPTelemetry(telemetryConfig);
    }

    if (isTelemetryProviderSmartPort()) {
        initSmartPortTelemetry(telemetryConfig);
    }

    checkTelemetryState();
}

bool determineNewTelemetryEnabledState(void)
{
    bool enabled = true;

    if (telemetryPortIsShared) {
        if (telemetryConfig->telemetry_provider == TELEMETRY_PROVIDER_SMARTPORT) {
            if (isSmartPortTimedOut()) {
                enabled = false;
            }
        } else {
            if (telemetryConfig->telemetry_switch)
                enabled = IS_RC_MODE_ACTIVE(BOXTELEMETRY);
            else
                enabled = ARMING_FLAG(ARMED);
        }
    }

    return enabled;
}

bool shouldChangeTelemetryStateNow(bool newState)
{
    return newState != telemetryEnabled;
}

uint32_t getTelemetryProviderBaudRate(void)
{
    if (isTelemetryProviderFrSky()) {
        return getFrSkyTelemetryProviderBaudRate();
    }

    if (isTelemetryProviderHoTT()) {
        return getHoTTTelemetryProviderBaudRate();
    }

    if (isTelemetryProviderMSP()) {
        return getMSPTelemetryProviderBaudRate();
    }

    if (isTelemetryProviderSmartPort()) {
        return getSmartPortTelemetryProviderBaudRate();
    }

    return 0;
}

static void configureTelemetryPort(void)
{
    if (isTelemetryProviderFrSky()) {
        configureFrSkyTelemetryPort();
    }

    if (isTelemetryProviderHoTT()) {
        configureHoTTTelemetryPort();
    }

    if (isTelemetryProviderMSP()) {
        configureMSPTelemetryPort();
    }

    if (isTelemetryProviderSmartPort()) {
        configureSmartPortTelemetryPort();
    }
}


void freeTelemetryPort(void)
{
    if (isTelemetryProviderFrSky()) {
        freeFrSkyTelemetryPort();
    }

    if (isTelemetryProviderHoTT()) {
        freeHoTTTelemetryPort();
    }

    if (isTelemetryProviderMSP()) {
        freeMSPTelemetryPort();
    }

    if (isTelemetryProviderSmartPort()) {
        freeSmartPortTelemetryPort();
    }
}

void checkTelemetryState(void)
{
    if (!isTelemetryConfigurationValid) {
        return;
    }

    bool newEnabledState = determineNewTelemetryEnabledState();

    if (!shouldChangeTelemetryStateNow(newEnabledState)) {
        return;
    }

    if (newEnabledState)
        configureTelemetryPort();
    else
        freeTelemetryPort();

    telemetryEnabled = newEnabledState;
}

void handleTelemetry(void)
{
    if (!isTelemetryConfigurationValid || !determineNewTelemetryEnabledState())
        return;

    if (!telemetryEnabled) {
        return;
    }

    if (isTelemetryProviderFrSky()) {
        handleFrSkyTelemetry();
    }

    if (isTelemetryProviderHoTT()) {
        handleHoTTTelemetry();
    }

    if (isTelemetryProviderMSP()) {
        handleMSPTelemetry();
    }

    if (isTelemetryProviderSmartPort()) {
        handleSmartPortTelemetry();
    }
}

bool telemetryAllowsOtherSerial(int serialPortFunction)
{
    if (!feature(FEATURE_TELEMETRY)) {
        return true;
    }

    if (isTelemetryProviderSmartPort() && isSerialPortFunctionShared(FUNCTION_SMARTPORT_TELEMETRY, (serialPortFunction_e)serialPortFunction)) {
        return canSmartPortAllowOtherSerial();
    }

    return true;
}

#endif
