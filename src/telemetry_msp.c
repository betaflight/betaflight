/*
 * telemetry_MSP.c
 *
 *  Created on: 22 Apr 2014
 *      Author: trey marc
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "drivers/serial_common.h"
#include "telemetry_common.h"
#include "serial_msp.h"

void initMSPTelemetry(telemetryConfig_t *initialTelemetryConfig)
{
}

void handleMSPTelemetry(void)
{
    sendMspTelemetry();
}
