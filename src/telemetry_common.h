/*
 * telemetry_common.h
 *
 *  Created on: 6 Apr 2014
 *      Author: Hydra
 */

#ifndef TELEMETRY_COMMON_H_
#define TELEMETRY_COMMON_H_

typedef enum {
    TELEMETRY_PROVIDER_FRSKY = 0,
    TELEMETRY_PROVIDER_HOTT,
    TELEMETRY_PROVIDER_MAX = TELEMETRY_PROVIDER_HOTT
} telemetryProvider_e;

typedef struct telemetryConfig_s {
    telemetryProvider_e telemetry_provider;
    uint8_t telemetry_switch;               // Use aux channel to change serial output & baudrate( MSP / Telemetry ). It disables automatic switching to Telemetry when armed.
    serialInversion_e frsky_inversion;
} telemetryConfig_t;

void checkTelemetryState(void);
void handleTelemetry(void);

void useTelemetryConfig(telemetryConfig_t *telemetryConfig);

#endif /* TELEMETRY_COMMON_H_ */
