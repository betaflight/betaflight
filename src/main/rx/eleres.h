#pragma once
#include "config/parameter_group.h"
#include "rx/rx_spi.h"

void eLeReS_control(uint16_t *rcData, const uint8_t *payload);
uint8_t eLeReS_Bind(void);
uint8_t eleres_rssi(void);
rx_spi_received_e eLeReS_check_irq(uint8_t *payload);
void eleresInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig);

typedef struct
{
    float eleres_freq;
    uint8_t  eleres_telemetry_en;
    uint8_t  eleres_telemetry_power;
    uint8_t  eleres_loc_en;
    uint8_t  eleres_loc_power;
    uint16_t eleres_loc_delay;
    uint8_t  eleres_signature[4];
} eleresConfig_t;

PG_DECLARE(eleresConfig_t, eleresConfig);

typedef struct
{
    int32_t coord[2];
    uint32_t dist;
    int32_t dir;
    uint8_t vbat;
    uint8_t rssi;
} tFinder_info;

void Finder_enable(void);
int Finder_parse(tFinder_info *fi);

