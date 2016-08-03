/*
 * smartport.h
 *
 *  Created on: 25 October 2014
 *      Author: Frank26080115
 */

#pragma once

struct telemetryConfig_s;
void initSmartPortTelemetry(struct telemetryConfig_s *);

void handleSmartPortTelemetry(void);
void checkSmartPortTelemetryState(void);

void configureSmartPortTelemetryPort(void);
void freeSmartPortTelemetryPort(void);

bool isSmartPortTimedOut(void);

