/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define USE_UART
#define USE_UART1
#define USE_UART2
#define USE_UART8
#define USE_DMA
#define USE_TIMER_AF
#define UART_TRAIT_AF_PORT 1
#define UARTHARDWARE_MAX_PINS 2
#define UART1_BIDIR_RX_PIN 7
#define UART2_BIDIR_RX_PIN 17
#define UART8_BIDIR_RX_PIN 27
#ifndef UM32_UART_BIDIR_NO_BOARD_PULLUP
#define UART1_BIDIR_PULLUP
#endif
#include "target.h"

#undef IOCFG_IN_FLOATING
#undef IOCFG_AF_PP
#undef IOCFG_IPU
#undef IOCFG_IPD
#define IOCFG_IN_FLOATING 0
#define IOCFG_AF_PP 1
#define IOCFG_IPU 32
#define IOCFG_IPD 64
#define IOCFG_AF_PP_UP 33
#define IOCFG_AF_PP_PD 65

#define UART_EX_LSR_TEMT 64
#define UART_EX_LSR_OE 2
#define UART_EX_LSR_PE 4
#define UART_EX_LSR_FE 8
#define UART_EX_LSR_RFE 128
#define UART_EX_USR_RFNE 8
#define USART_CSR_TXEMPTY 512
#define USART_CSR_RXRDY 1
#define LPTIM_CTRL_LPTEN 1
#define LPTIM_IF_OVIF 2
#define LPTIM_IE_OVIE 2
#define LPTIM_CFG_CLKSEL_1 512
#define LPTIM_CFG_DIVSEL 7168
#define LPTIM_CFG_DIVSEL_0 1024
#define LPTIM_CFG_DIVSEL_2 4096
#define LPTIM_CFG_MODE 4
#define LPTIMER1_IRQn 109
#define LPTIMER2_IRQn 110
#define __HAL_PMU_LPTIM1_CLK_ENABLE() ((void)0)
#define __HAL_PMU_LPTIM1_FORCE_RESET() ((void)0)
#define __HAL_PMU_LPTIM1_RELEASE_RESET() ((void)0)
#define __HAL_PMU_LPTIM2_CLK_ENABLE() ((void)0)
#define __HAL_PMU_LPTIM2_FORCE_RESET() ((void)0)
#define __HAL_PMU_LPTIM2_RELEASE_RESET() ((void)0)

typedef struct { volatile uint32_t LSR, USR, RBR; } UART_TypeDef;
typedef struct { volatile uint32_t CSR, RHR; } USART_TypeDef;
typedef struct { volatile uint32_t CTRL, CFG, TARGET, IF, IE; } mockLptim_t;
extern mockLptim_t mockRxLptim;
extern mockLptim_t mockLptim;
#define LPTIM1 (&mockRxLptim)
#define LPTIM2 (&mockLptim)
uint32_t HAL_RCM_GetPCLK0Freq(void);
void HAL_NVIC_SetPriority(int irq, int priority, int subpriority);
void HAL_NVIC_EnableIRQ(int irq);
