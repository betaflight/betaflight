/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#if ENABLE_DEBUG_UART

#include "drivers/debug_uart.h"

#if !defined(DEBUG_UART) || !defined(DEBUG_UART_GPIO) \
    || !defined(DEBUG_UART_PIN) || !defined(DEBUG_UART_AF)
#error "ENABLE_DEBUG_UART=1 requires DEBUG_UART, DEBUG_UART_GPIO, DEBUG_UART_PIN, DEBUG_UART_AF in config.h"
#endif

#ifndef DEBUG_UART_BAUD
#define DEBUG_UART_BAUD 115200U
#endif

// BF's SystemClock_Config and OBL both land at PCLK1 = 200 MHz after the
// PLL1 IC matrix is up; PCLK1 briefly drops to ~32 MHz during the HSI
// park inside SystemClock_Config, so bytes sent in that window will show
// as garbage on the host. Acceptable for bring-up tracing.
#define DEBUG_UART_PCLK_HZ 200000000UL
#define DEBUG_UART_BRR_VAL \
    ((DEBUG_UART_PCLK_HZ + (DEBUG_UART_BAUD / 2U)) / DEBUG_UART_BAUD)

// Compile-time-resolvable pointer-equality cascade so the compiler keeps
// only the branch matching the per-config DEBUG_UART_GPIO / DEBUG_UART.
// Using the HAL clock-enable macros keeps the RCC bit-mapping authoritative
// and avoids re-deriving the per-instance ENSR positions here.

static void enableGpioClock(GPIO_TypeDef *port)
{
    if      (port == GPIOA) { __HAL_RCC_GPIOA_CLK_ENABLE(); }
    else if (port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
    else if (port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
    else if (port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
    else if (port == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
    else if (port == GPIOF) { __HAL_RCC_GPIOF_CLK_ENABLE(); }
    else if (port == GPIOG) { __HAL_RCC_GPIOG_CLK_ENABLE(); }
    else if (port == GPIOH) { __HAL_RCC_GPIOH_CLK_ENABLE(); }
    else if (port == GPION) { __HAL_RCC_GPION_CLK_ENABLE(); }
    else if (port == GPIOO) { __HAL_RCC_GPIOO_CLK_ENABLE(); }
    else if (port == GPIOP) { __HAL_RCC_GPIOP_CLK_ENABLE(); }
    else if (port == GPIOQ) { __HAL_RCC_GPIOQ_CLK_ENABLE(); }
}

static void enableUartClock(USART_TypeDef *uart)
{
    if      (uart == USART1)  { __HAL_RCC_USART1_CLK_ENABLE();  }
    else if (uart == USART2)  { __HAL_RCC_USART2_CLK_ENABLE();  }
    else if (uart == USART3)  { __HAL_RCC_USART3_CLK_ENABLE();  }
    else if (uart == UART4)   { __HAL_RCC_UART4_CLK_ENABLE();   }
    else if (uart == UART5)   { __HAL_RCC_UART5_CLK_ENABLE();   }
    else if (uart == USART6)  { __HAL_RCC_USART6_CLK_ENABLE();  }
    else if (uart == UART7)   { __HAL_RCC_UART7_CLK_ENABLE();   }
    else if (uart == UART8)   { __HAL_RCC_UART8_CLK_ENABLE();   }
    else if (uart == UART9)   { __HAL_RCC_UART9_CLK_ENABLE();   }
    else if (uart == USART10) { __HAL_RCC_USART10_CLK_ENABLE(); }
}

void debugUartInit(void)
{
    GPIO_TypeDef * const gpio = DEBUG_UART_GPIO;
    USART_TypeDef * const uart = DEBUG_UART;
    const uint32_t pin = DEBUG_UART_PIN;

    enableGpioClock(gpio);
    enableUartClock(uart);

    uint32_t moder = gpio->MODER;
    moder &= ~(3UL << (pin * 2U));
    moder |=  (2UL << (pin * 2U));        // AF mode
    gpio->MODER = moder;

    uint32_t ospeedr = gpio->OSPEEDR;
    ospeedr &= ~(3UL << (pin * 2U));
    ospeedr |=  (2UL << (pin * 2U));      // high-speed slew
    gpio->OSPEEDR = ospeedr;

    const uint32_t afr_idx   = pin >> 3;
    const uint32_t afr_shift = (pin & 7U) * 4U;
    uint32_t afr = gpio->AFR[afr_idx];
    afr &= ~(0xFUL << afr_shift);
    afr |=  ((uint32_t)DEBUG_UART_AF << afr_shift);
    gpio->AFR[afr_idx] = afr;

    uart->CR1 = 0;
    uart->CR2 = 0;
    uart->CR3 = 0;
    uart->BRR = DEBUG_UART_BRR_VAL;
    uart->CR1 = USART_CR1_UE | USART_CR1_TE;
    __DSB();
    __ISB();
}

void debugUartPutc(char c)
{
    USART_TypeDef * const uart = DEBUG_UART;
    while (!(uart->ISR & (1UL << 7))) {}   // TXE / TXFNF
    uart->TDR = (uint8_t)c;
}

void debugUartPuts(const char *s)
{
    while (*s) {
        debugUartPutc(*s++);
    }
}

static const char hex_digits[] = "0123456789abcdef";

void debugUartPutHex8(uint8_t v)
{
    debugUartPutc(hex_digits[(v >> 4) & 0xFU]);
    debugUartPutc(hex_digits[v & 0xFU]);
}

void debugUartPutHex32(uint32_t v)
{
    for (int i = 7; i >= 0; --i) {
        debugUartPutc(hex_digits[(v >> (i * 4)) & 0xFU]);
    }
}

#endif // ENABLE_DEBUG_UART
