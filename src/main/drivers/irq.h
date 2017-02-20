#pragma once

#include "common/utils.h"

// check that `name` is valid handler name (corresponding IRQ number is defined) and emit handler definition
// `IRQHANDLER(TIM1_UP_IRQ)` will expand to `void TIM1_UP_IRQHandler(void)` on F1, but generate compile-time error on F3 target

#define IRQHANDLER(name) \
    extern char handler_name_check[CONCAT(name, n)*0 + 1]; \
    void CONCAT(name, Handler)(void)
