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

void configureFrSkyTelemetryPort(void);
void freeFrSkyTelemetryPort(void);

#endif /* TELEMETRY_FRSKY_H_ */
