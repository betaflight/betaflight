#include "board.h"
#include "mw.h"

#include "telemetry_frsky.h"
#include "telemetry_hott.h"

bool isTelemetryProviderFrSky(void)
{
    return mcfg.telemetry_provider == TELEMETRY_PROVIDER_FRSKY;
}

bool isTelemetryProviderHoTT(void)
{
    return mcfg.telemetry_provider == TELEMETRY_PROVIDER_HOTT;
}

void initTelemetry(void)
{
    // Sanity check for softserial vs. telemetry port
    if (!feature(FEATURE_SOFTSERIAL))
        mcfg.telemetry_port = TELEMETRY_PORT_UART;

    if (mcfg.telemetry_port == TELEMETRY_PORT_SOFTSERIAL_1)
        core.telemport = &(softSerialPorts[0].port);
    else if (mcfg.telemetry_port == TELEMETRY_PORT_SOFTSERIAL_2)
        core.telemport = &(softSerialPorts[1].port);
    else
        core.telemport = core.mainport;
}

static bool telemetryEnabled = false;

bool isTelemetryEnabled(void)
{
    bool telemetryCurrentlyEnabled = true;

    if (mcfg.telemetry_port == TELEMETRY_PORT_UART) {
        if (!mcfg.telemetry_switch)
            telemetryCurrentlyEnabled = f.ARMED;
        else
            telemetryCurrentlyEnabled = rcOptions[BOXTELEMETRY];
    }

    return telemetryCurrentlyEnabled;
}

bool shouldChangeTelemetryStateNow(bool telemetryCurrentlyEnabled)
{
    return telemetryCurrentlyEnabled != telemetryEnabled;
}

void configureTelemetryPort(void) {
    if (isTelemetryProviderFrSky()) {
        configureFrSkyTelemetryPort();
    }

    if (isTelemetryProviderHoTT()) {
        configureHoTTTelemetryPort();
    }
}

void freeTelemetryPort(void) {
    if (isTelemetryProviderFrSky()) {
        freeFrSkyTelemetryPort();
    }

    if (isTelemetryProviderHoTT()) {
        freeHoTTTelemetryPort();
    }
}

void checkTelemetryState(void)
{
    bool telemetryCurrentlyEnabled = isTelemetryEnabled();

    if (!shouldChangeTelemetryStateNow(telemetryCurrentlyEnabled)) {
        return;
    }

    if (telemetryCurrentlyEnabled)
        configureTelemetryPort();
    else
        freeTelemetryPort();

    telemetryEnabled = telemetryCurrentlyEnabled;
}

void handleTelemetry(void)
{
    if (!isTelemetryEnabled())
        return;

    if (isTelemetryProviderFrSky()) {
        handleFrSkyTelemetry();
    }

    if (isTelemetryProviderHoTT()) {
        handleHoTTTelemetry();
    }
}
