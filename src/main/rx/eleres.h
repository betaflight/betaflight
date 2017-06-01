#pragma once
#include "config/parameter_group.h"

uint8_t eLeReS_control(void);
uint8_t eLeReS_Bind(void);
void Rfm_IRQ(void);
uint8_t eleres_rssi(void);
void eLeReS_check_irq(void);
bool eleresInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig);

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

