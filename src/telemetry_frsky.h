/*
 * telemetry_frsky.h
 *
 *  Created on: 6 Apr 2014
 *      Author: Hydra
 */

#ifndef TELEMETRY_FRSKY_H_
#define TELEMETRY_FRSKY_H_

void handleFrSkyTelemetry(void);
void checkFrSkyTelemetryState(void);

void initFrSkyTelemetry(telemetryConfig_t *telemetryConfig);
void configureFrSkyTelemetryPort(void);
void freeFrSkyTelemetryPort(void);

uint32_t getFrSkyTelemetryProviderBaudRate(void);

#endif /* TELEMETRY_FRSKY_H_ */
