#ifndef PORTABLE_H
#define PORTABLE_H

#pragma once
/*
 * Use this file to map mcu specific functions
 */

#include "platform.h"

#define MHz 1000000


#if defined(STM32F7)

//    #define MCU_FLASH_PAGE_COUNT FLASH_SECTOR_TOTAL
#if defined(STM32F745xx)
    #define MCU_FLASH_NR_SECTORS 8
    #define MCU_FLASH_SECTORS {32, 32, 32, 32, 128, 256, 256, 256} // all sector sizes in kB
#elif defined(STM32F767xx)
    #define MCU_FLASH_NR_SECTORS 12
    #define MCU_FLASH_SECTORS {32, 32, 32, 32, 128, 256, 256, 256, 256, 256, 256, 256} // all sector sizes in kB
#else
#error Please select a correct device
#endif

    #define MCU_SYS_CLK     (216*MHz)
    #define MCU_APB1_CLK    (MCU_SYS_CLK/4)
    #define MCU_APB2_CLK    (MCU_SYS_CLK/2)
    
    // Configure UART 1
    #define MCU_UART1_AF                    GPIO_AF7_USART1
    #define MCU_UART1_DMA_TX_STREAM         DMA2_Stream7
    #define MCU_UART1_DMA_TX_CHANNEL        DMA_CHANNEL_4
    #define MCU_UART1_DMA_TX_STREAM_IRQn    DMA2_Stream7_IRQn
    #define MCU_UART1_DMA_TX_IRQHandler     DMA2_Stream7_IRQHandler
    #define MCU_UART1_DMA_RX_STREAM         DMA2_Stream5
    #define MCU_UART1_DMA_RX_CHANNEL        DMA_CHANNEL_4
    #define MCU_UART1_DMA_RX_STREAM_IRQn    DMA2_Stream5_IRQn
    #define MCU_UART1_DMA_RX_IRQHandler     DMA2_Stream5_IRQHandler


  // Configure UART 2
    #define MCU_UART2_AF                    GPIO_AF7_USART2
    #define MCU_UART2_DMA_TX_STREAM         DMA1_Stream6
    #define MCU_UART2_DMA_TX_CHANNEL        DMA_CHANNEL_4
    #define MCU_UART2_DMA_TX_STREAM_IRQn    DMA1_Stream6_IRQn
    #define MCU_UART2_DMA_TX_IRQHandler     DMA1_Stream6_IRQHandler
    #define MCU_UART2_DMA_RX_STREAM         DMA1_Stream5
    #define MCU_UART2_DMA_RX_CHANNEL        DMA_CHANNEL_4
    #define MCU_UART2_DMA_RX_STREAM_IRQn    DMA1_Stream5_IRQn
    #define MCU_UART2_DMA_RX_IRQHandler     DMA1_Stream5_IRQHandler

  // Configure UART 3
    #define MCU_UART3_AF                    GPIO_AF7_USART3
    #define MCU_UART3_DMA_TX_STREAM         DMA1_Stream3
    #define MCU_UART3_DMA_TX_CHANNEL        DMA_CHANNEL_4
    #define MCU_UART3_DMA_TX_STREAM_IRQn    DMA1_Stream3_IRQn
    #define MCU_UART3_DMA_TX_IRQHandler     DMA1_Stream3_IRQHandler
    #define MCU_UART3_DMA_RX_STREAM         DMA1_Stream1
    #define MCU_UART3_DMA_RX_CHANNEL        DMA_CHANNEL_4
    #define MCU_UART3_DMA_RX_STREAM_IRQn    DMA1_Stream1_IRQn
    #define MCU_UART3_DMA_RX_IRQHandler     DMA1_Stream1_IRQHandler

  // Configure UART 4
    #define MCU_UART4_AF                    GPIO_AF8_UART4
    #define MCU_UART4_DMA_TX_STREAM         DMA1_Stream4
    #define MCU_UART4_DMA_TX_CHANNEL        DMA_CHANNEL_4
    #define MCU_UART4_DMA_TX_STREAM_IRQn    DMA1_Stream4_IRQn
    #define MCU_UART4_DMA_TX_IRQHandler     DMA1_Stream4_IRQHandler
    #define MCU_UART4_DMA_RX_STREAM         DMA1_Stream2
    #define MCU_UART4_DMA_RX_CHANNEL        DMA_CHANNEL_4
    #define MCU_UART4_DMA_RX_STREAM_IRQn    DMA1_Stream2_IRQn
    #define MCU_UART4_DMA_RX_IRQHandler     DMA1_Stream2_IRQHandler
    
    /// TODO: HAL Implement definitions for other uarts
    #define MCU_UART5_AF                    GPIO_AF8_UART5
    #define MCU_UART5_DMA_TX_STREAM         DMA1_Stream7
    #define MCU_UART5_DMA_TX_CHANNEL        DMA_CHANNEL_4
    #define MCU_UART5_DMA_TX_STREAM_IRQn    DMA1_Stream7_IRQn
    #define MCU_UART5_DMA_TX_IRQHandler     DMA1_Stream7_IRQHandler
    #define MCU_UART5_DMA_RX_STREAM         DMA1_Stream0
    #define MCU_UART5_DMA_RX_CHANNEL        DMA_CHANNEL_4
    #define MCU_UART5_DMA_RX_STREAM_IRQn    DMA1_Stream0_IRQn
    #define MCU_UART5_DMA_RX_IRQHandler     DMA1_Stream0_IRQHandler

  // Configure UART 6
    #define MCU_UART6_AF                    GPIO_AF8_USART6
    #define MCU_UART6_DMA_TX_STREAM         DMA2_Stream6
    #define MCU_UART6_DMA_TX_CHANNEL        DMA_CHANNEL_5
    #define MCU_UART6_DMA_TX_STREAM_IRQn    DMA2_Stream6_IRQn
    #define MCU_UART6_DMA_TX_IRQHandler     DMA2_Stream6_IRQHandler
    #define MCU_UART6_DMA_RX_STREAM         DMA2_Stream1
    #define MCU_UART6_DMA_RX_CHANNEL        DMA_CHANNEL_5
    #define MCU_UART6_DMA_RX_STREAM_IRQn    DMA2_Stream1_IRQn
    #define MCU_UART6_DMA_RX_IRQHandler     DMA2_Stream1_IRQHandler

  // Configure UART 7
    #define MCU_UART7_AF                    GPIO_AF8_UART7
    #define MCU_UART7_DMA_TX_STREAM         DMA1_Stream1
    #define MCU_UART7_DMA_TX_CHANNEL        DMA_CHANNEL_5
    #define MCU_UART7_DMA_TX_STREAM_IRQn    DMA1_Stream1_IRQn
    #define MCU_UART7_DMA_TX_IRQHandler     DMA1_Stream1_IRQHandler
    #define MCU_UART7_DMA_RX_STREAM         DMA1_Stream3
    #define MCU_UART7_DMA_RX_CHANNEL        DMA_CHANNEL_5
    #define MCU_UART7_DMA_RX_STREAM_IRQn    DMA1_Stream3_IRQn
    #define MCU_UART7_DMA_RX_IRQHandler     DMA1_Stream3_IRQHandler

  // Configure UART 8
    #define MCU_UART8_AF                    GPIO_AF8_UART8
    #define MCU_UART8_DMA_TX_STREAM         DMA1_Stream0
    #define MCU_UART8_DMA_TX_CHANNEL        DMA_CHANNEL_5
    #define MCU_UART8_DMA_TX_STREAM_IRQn    DMA1_Stream0_IRQn
    #define MCU_UART8_DMA_TX_IRQHandler     DMA1_Stream0_IRQHandler
    #define MCU_UART8_DMA_RX_STREAM         DMA1_Stream6
    #define MCU_UART8_DMA_RX_CHANNEL        DMA_CHANNEL_5
    #define MCU_UART8_DMA_RX_STREAM_IRQn    DMA1_Stream6_IRQn
    #define MCU_UART8_DMA_RX_IRQHandler     DMA1_Stream6_IRQHandler

    // Config I2C
    #define MCU_I2C1_AF     GPIO_AF4_I2C1
    #define MCU_I2C2_AF     GPIO_AF4_I2C2
    #define MCU_I2C3_AF     GPIO_AF4_I2C3
    #define MCU_I2C4_AF     GPIO_AF4_I2C4
    
    // SPI
    #define MCU_SPI1_AF     GPIO_AF5_SPI1
    #define MCU_SPI1_CLK    MCU_APB2_CLK
    #define MCU_SPI1_SPEED  {SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_16, SPI_BAUDRATEPRESCALER_8} //0.84Mhz, 6.75Mhz, 13.5Mhz
    
    #define MCU_SPI2_AF     GPIO_AF5_SPI2
    #define MCU_SPI2_CLK    MCU_APB1_CLK
    #define MCU_SPI2_SPEED  {SPI_BAUDRATEPRESCALER_64, SPI_BAUDRATEPRESCALER_8, SPI_BAUDRATEPRESCALER_4} //0.84Mhz, 6.75Mhz, 13.5Mhz

    #define MCU_SPI3_AF     GPIO_AF6_SPI3
    #define MCU_SPI3_CLK    MCU_APB1_CLK
    #define MCU_SPI3_SPEED  {SPI_BAUDRATEPRESCALER_64, SPI_BAUDRATEPRESCALER_8, SPI_BAUDRATEPRESCALER_4} //0.84Mhz, 6.75Mhz, 13.5Mhz


    // ADC
#define MCU_ADC1_DMA_STREAM         DMA2_Stream0
#define MCU_ADC1_DMA_CHANNEL        DMA_CHANNEL_0
#define MCU_ADC1_DMA_STREAM_IRQn    DMA2_Stream0_IRQn
#define MCU_ADC1_DMA_IRQHandler     DMA2_Stream0_IRQHandler

#endif


#endif // PORTABLE_H
