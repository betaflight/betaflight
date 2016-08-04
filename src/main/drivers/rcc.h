#pragma once

#include "rcc_types.h"

enum rcc_reg {
    RCC_EMPTY = 0,   // make sure that default value (0) does not enable anything
    RCC_AHB,
    RCC_APB2,
    RCC_APB1,
    RCC_AHB1,
};

#define RCC_ENCODE(reg, mask) (((reg) << 5) | LOG2_32BIT(mask))
#define RCC_AHB(periph) RCC_ENCODE(RCC_AHB, RCC_AHBENR_ ## periph ## EN)
#define RCC_APB2(periph) RCC_ENCODE(RCC_APB2, RCC_APB2ENR_ ## periph ## EN)
#define RCC_APB1(periph) RCC_ENCODE(RCC_APB1, RCC_APB1ENR_ ## periph ## EN)
#define RCC_AHB1(periph) RCC_ENCODE(RCC_AHB1, RCC_AHB1ENR_ ## periph ## EN)

void RCC_ClockCmd(rccPeriphTag_t periphTag, FunctionalState NewState);
void RCC_ResetCmd(rccPeriphTag_t periphTag, FunctionalState NewState);

