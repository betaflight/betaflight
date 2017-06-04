#pragma once
#include "config/parameter_group.h"
#include "rx/rx_spi.h"

void eleresSetRcDataFromPayload(uint16_t *rcData, const uint8_t *payload);
uint8_t eleresBind(void);
uint16_t eleresRssi(void);
rx_spi_received_e eleresDataReceived(uint8_t *payload);
void eleresInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig);

typedef struct
{
    float eleresFreq;
    uint8_t  eleresTelemetryEn;
    uint8_t  eleresTelemetryPower;
    uint8_t  eleresLocEn;
    uint8_t  eleresLocPower;
    uint16_t eleresLocDelay;
    uint8_t  eleresSignature[4];
} eleresConfig_t;

PG_DECLARE(eleresConfig_t, eleresConfig);
