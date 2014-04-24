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
} TelemetryProvider;

typedef enum {
    TELEMETRY_PORT_UART = 0,
    TELEMETRY_PORT_SOFTSERIAL_1, // Requires FEATURE_SOFTSERIAL
    TELEMETRY_PORT_SOFTSERIAL_2, // Requires FEATURE_SOFTSERIAL
    TELEMETRY_PORT_MAX = TELEMETRY_PORT_SOFTSERIAL_2
} TelemetryPort;

typedef struct telemetryConfig_s {
    uint8_t telemetry_provider;             // See TelemetryProvider enum.
    uint8_t telemetry_port;                 // See TelemetryPort enum.
    uint8_t telemetry_switch;               // Use aux channel to change serial output & baudrate( MSP / Telemetry ). It disables automatic switching to Telemetry when armed.
} telemetryConfig_t;

void checkTelemetryState(void);
void handleTelemetry(void);

void useTelemetryConfig(telemetryConfig_t *telemetryConfig);

#endif /* TELEMETRY_COMMON_H_ */
