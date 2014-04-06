#include "board.h"
#include "mw.h"

#include "telemetry_frsky.h"

void initTelemetry(void)
{
    // Sanity check for softserial vs. telemetry port
    if (!feature(FEATURE_SOFTSERIAL))
        mcfg.telemetry_softserial = TELEMETRY_UART;

    if (mcfg.telemetry_softserial == TELEMETRY_SOFTSERIAL_1)
        core.telemport = &(softSerialPorts[0].port);
    else if (mcfg.telemetry_softserial == TELEMETRY_SOFTSERIAL_2)
        core.telemport = &(softSerialPorts[1].port);
    else
        core.telemport = core.mainport;
}

void updateTelemetryState(void) {
    updateFrSkyTelemetryState();
}

bool isTelemetryEnabled(void)
{
    bool telemetryCurrentlyEnabled = true;

    if (mcfg.telemetry_softserial == TELEMETRY_UART) {
        if (!mcfg.telemetry_switch)
            telemetryCurrentlyEnabled = f.ARMED;
        else
            telemetryCurrentlyEnabled = rcOptions[BOXTELEMETRY];
    }

    return telemetryCurrentlyEnabled;
}

bool isFrSkyTelemetryEnabled(void)
{
    return mcfg.telemetry_provider == TELEMETRY_PROVIDER_FRSKY;
}

void sendTelemetry(void)
{
    if (!isTelemetryEnabled())
        return;

    if (isFrSkyTelemetryEnabled()) {
        sendFrSkyTelemetry();
    }
}
