/**
 * @file        apm32f4xx_dal_cfg.h
 *
 * @brief       DAL configuration file
 *
 * @version     V1.0.0
 *
 * @date        2023-07-31
 *
 * @attention
 *
 *  Copyright (C) 2023 Geehy Semiconductor
 *
 *  You may not use this file except in compliance with the
 *  GEEHY COPYRIGHT NOTICE (GEEHY SOFTWARE PACKAGE LICENSE).
 *
 *  The program is only for reference, which is distributed in the hope
 *  that it will be useful and instructional for customers to develop
 *  their software. Unless required by applicable law or agreed to in
 *  writing, the program is distributed on an "AS IS" BASIS, WITHOUT
 *  ANY WARRANTY OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the GEEHY SOFTWARE PACKAGE LICENSE for the governing permissions
 *  and limitations under the License.
 */

/* Define to prevent recursive inclusion */
#ifndef APM32F4xx_DAL_CFG_H
#define APM32F4xx_DAL_CFG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Configuration settings for log component */
#define USE_LOG_COMPONENT   0U
/* Include log header file */
#include "apm32f4xx_dal_log.h"

/* Configuration settings for assert enable */
/* #define USE_FULL_ASSERT     1U */

/* DAL module configuration */
#define DAL_MODULE_ENABLED
#define DAL_ADC_MODULE_ENABLED
// #define DAL_CAN_MODULE_ENABLED
// #define DAL_CRC_MODULE_ENABLED
// #define DAL_CRYP_MODULE_ENABLED
// #define DAL_DAC_MODULE_ENABLED
// #define DAL_DCI_MODULE_ENABLED
#define DAL_DMA_MODULE_ENABLED
// #define DAL_ETH_MODULE_ENABLED
#define DAL_FLASH_MODULE_ENABLED
// #define DAL_NAND_MODULE_ENABLED
// #define DAL_NOR_MODULE_ENABLED
// #define DAL_PCCARD_MODULE_ENABLED
// #define DAL_SRAM_MODULE_ENABLED
// #define DAL_SDRAM_MODULE_ENABLED
// #define DAL_HASH_MODULE_ENABLED
#define DAL_GPIO_MODULE_ENABLED
#define DAL_EINT_MODULE_ENABLED
#define DAL_I2C_MODULE_ENABLED
// #define DAL_SMBUS_MODULE_ENABLED
// #define DAL_I2S_MODULE_ENABLED
// #define DAL_IWDT_MODULE_ENABLED
#define DAL_PMU_MODULE_ENABLED
#define DAL_RCM_MODULE_ENABLED
// #define DAL_RNG_MODULE_ENABLED
#define DAL_RTC_MODULE_ENABLED
// #define DAL_SD_MODULE_ENABLED
#define DAL_SPI_MODULE_ENABLED
#define DAL_TMR_MODULE_ENABLED
#define DAL_UART_MODULE_ENABLED
#define DAL_USART_MODULE_ENABLED
// #define DAL_IRDA_MODULE_ENABLED
// #define DAL_SMARTCARD_MODULE_ENABLED
// #define DAL_WWDT_MODULE_ENABLED
#define DAL_CORTEX_MODULE_ENABLED
#define DAL_PCD_MODULE_ENABLED
// #define DAL_HCD_MODULE_ENABLED
// #define DAL_MMC_MODULE_ENABLED

/* Value of the external high speed oscillator in Hz */
#if !defined  (HSE_VALUE)
  #define HSE_VALUE              8000000U
#endif /* HSE_VALUE */

/* Timeout for external high speed oscillator in ms */
#if !defined  (HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT    100U
#endif /* HSE_STARTUP_TIMEOUT */

/* Value of the internal high speed oscillator in Hz */
#if !defined  (HSI_VALUE)
  #define HSI_VALUE              16000000U
#endif /* HSI_VALUE */

/* Value of the internal low speed oscillator in Hz */
#if !defined  (LSI_VALUE)
 #define LSI_VALUE               32000U
#endif /* LSI_VALUE */

/* Value of the external low speed oscillator in Hz */
#if !defined  (LSE_VALUE)
 #define LSE_VALUE               32768U
#endif /* LSE_VALUE */

/* Timeout for external low speed oscillator in ms */
#if !defined  (LSE_STARTUP_TIMEOUT)
  #define LSE_STARTUP_TIMEOUT    5000U
#endif /* LSE_STARTUP_TIMEOUT */

/* Value of the external high speed oscillator in Hz for I2S peripheral */
#if !defined  (EXTERNAL_CLOCK_VALUE)
  #define EXTERNAL_CLOCK_VALUE     12288000U
#endif /* EXTERNAL_CLOCK_VALUE */

/* System Configuration */
#define  VDD_VALUE                    3300U /*!< Value of VDD in mv */
#define  TICK_INT_PRIORITY            0x0FU /*!< tick interrupt priority */
#define  USE_RTOS                     0U
#define  PREFETCH_ENABLE              1U
#define  INSTRUCTION_CACHE_ENABLE     1U
#define  DATA_CACHE_ENABLE            1U

/* DAL peripheral register callbacks */
#define  USE_DAL_ADC_REGISTER_CALLBACKS         0U /* ADC register callback disabled       */
#define  USE_DAL_CAN_REGISTER_CALLBACKS         0U /* CAN register callback disabled       */
#define  USE_DAL_CRYP_REGISTER_CALLBACKS        0U /* CRYP register callback disabled      */
#define  USE_DAL_DAC_REGISTER_CALLBACKS         0U /* DAC register callback disabled       */
#define  USE_DAL_DCI_REGISTER_CALLBACKS         0U /* DCI register callback disabled       */
#define  USE_DAL_ETH_REGISTER_CALLBACKS         0U /* ETH register callback disabled       */
#define  USE_DAL_HASH_REGISTER_CALLBACKS        0U /* HASH register callback disabled      */
#define  USE_DAL_HCD_REGISTER_CALLBACKS         0U /* HCD register callback disabled       */
#define  USE_DAL_I2C_REGISTER_CALLBACKS         0U /* I2C register callback disabled       */
#define  USE_DAL_I2S_REGISTER_CALLBACKS         0U /* I2S register callback disabled       */
#define  USE_DAL_IRDA_REGISTER_CALLBACKS        0U /* IRDA register callback disabled      */
#define  USE_DAL_MMC_REGISTER_CALLBACKS         0U /* MMC register callback disabled       */
#define  USE_DAL_NAND_REGISTER_CALLBACKS        0U /* NAND register callback disabled      */
#define  USE_DAL_NOR_REGISTER_CALLBACKS         0U /* NOR register callback disabled       */
#define  USE_DAL_PCCARD_REGISTER_CALLBACKS      0U /* PCCARD register callback disabled    */
#define  USE_DAL_PCD_REGISTER_CALLBACKS         0U /* PCD register callback disabled       */
#define  USE_DAL_RNG_REGISTER_CALLBACKS         0U /* RNG register callback disabled       */
#define  USE_DAL_RTC_REGISTER_CALLBACKS         0U /* RTC register callback disabled       */
#define  USE_DAL_SD_REGISTER_CALLBACKS          0U /* SD register callback disabled        */
#define  USE_DAL_SMARTCARD_REGISTER_CALLBACKS   0U /* SMARTCARD register callback disabled */
#define  USE_DAL_SDRAM_REGISTER_CALLBACKS       0U /* SDRAM register callback disabled     */
#define  USE_DAL_SRAM_REGISTER_CALLBACKS        0U /* SRAM register callback disabled      */
#define  USE_DAL_SMBUS_REGISTER_CALLBACKS       0U /* SMBUS register callback disabled     */
#define  USE_DAL_SPI_REGISTER_CALLBACKS         0U /* SPI register callback disabled       */
#define  USE_DAL_TMR_REGISTER_CALLBACKS         0U /* TMR register callback disabled       */
#define  USE_DAL_UART_REGISTER_CALLBACKS        0U /* UART register callback disabled      */
#define  USE_DAL_USART_REGISTER_CALLBACKS       0U /* USART register callback disabled     */
#define  USE_DAL_WWDT_REGISTER_CALLBACKS        0U /* WWDT register callback disabled      */

/* Ethernet peripheral configuration */
/* Addr and buffer size */

/* MAC ADDRESS */
#define ETH_MAC_ADDR_0   2U
#define ETH_MAC_ADDR_1   0U
#define ETH_MAC_ADDR_2   0U
#define ETH_MAC_ADDR_3   0U
#define ETH_MAC_ADDR_4   0U
#define ETH_MAC_ADDR_5   0U

/* Ethernet driver buffers size and number */
#define ETH_BUFFER_SIZE_RX             ETH_MAX_PACKET_SIZE /* Buffer size for receive               */
#define ETH_BUFFER_SIZE_TX             ETH_MAX_PACKET_SIZE /* Buffer size for transmit              */
#define ETH_BUFFER_NUMBER_RX           4U                  /* 4 Rx buffers of size ETH_BUFFER_SIZE_RX  */
#define ETH_BUFFER_NUMBER_TX           4U                  /* 4 Tx buffers of size ETH_BUFFER_SIZE_TX  */

/* Delay and timeout */

/* PHY Reset MAX Delay */
#define EXT_PHY_RESET_MAX_DELAY         0x000000FFU
/* PHY Configuration MAX Delay */
#define EXT_PHY_CONFIG_MAX_DELAY        0x00000FFFU

#define EXT_PHY_READ_TIMEOUT            0x0000FFFFU
#define EXT_PHY_WRITE_TIMEOUT           0x0000FFFFU

/* SPI peripheral configuration */

/* SPI CRC FEATURE */
#define USE_SPI_CRC                     1U

/* Include module's header file */
#ifdef DAL_RCM_MODULE_ENABLED
  #include "apm32f4xx_dal_rcm.h"
#endif /* DAL_RCM_MODULE_ENABLED */

#ifdef DAL_GPIO_MODULE_ENABLED
  #include "apm32f4xx_dal_gpio.h"
#endif /* DAL_GPIO_MODULE_ENABLED */

#ifdef DAL_EINT_MODULE_ENABLED
  #include "apm32f4xx_dal_eint.h"
#endif /* DAL_EINT_MODULE_ENABLED */

#ifdef DAL_DMA_MODULE_ENABLED
  #include "apm32f4xx_dal_dma.h"
#endif /* DAL_DMA_MODULE_ENABLED */

#ifdef DAL_CORTEX_MODULE_ENABLED
  #include "apm32f4xx_dal_cortex.h"
#endif /* DAL_CORTEX_MODULE_ENABLED */

#ifdef DAL_ADC_MODULE_ENABLED
  #include "apm32f4xx_dal_adc.h"
#endif /* DAL_ADC_MODULE_ENABLED */

#ifdef DAL_CAN_MODULE_ENABLED
  #include "apm32f4xx_dal_can.h"
#endif /* DAL_CAN_MODULE_ENABLED */

#ifdef DAL_CRC_MODULE_ENABLED
  #include "apm32f4xx_dal_crc.h"
#endif /* DAL_CRC_MODULE_ENABLED */

#ifdef DAL_CRYP_MODULE_ENABLED
  #include "apm32f4xx_dal_cryp.h"
#endif /* DAL_CRYP_MODULE_ENABLED */

#ifdef DAL_DAC_MODULE_ENABLED
  #include "apm32f4xx_dal_dac.h"
#endif /* DAL_DAC_MODULE_ENABLED */

#ifdef DAL_DCI_MODULE_ENABLED
  #include "apm32f4xx_dal_dci.h"
#endif /* DAL_DCI_MODULE_ENABLED */

#ifdef DAL_ETH_MODULE_ENABLED
  #include "apm32f4xx_dal_eth.h"
#endif /* DAL_ETH_MODULE_ENABLED */

#ifdef DAL_FLASH_MODULE_ENABLED
  #include "apm32f4xx_dal_flash.h"
#endif /* DAL_FLASH_MODULE_ENABLED */

#ifdef DAL_HASH_MODULE_ENABLED
 #include "apm32f4xx_dal_hash.h"
#endif /* DAL_HASH_MODULE_ENABLED */

#ifdef DAL_HCD_MODULE_ENABLED
 #include "apm32f4xx_dal_hcd.h"
#endif /* DAL_HCD_MODULE_ENABLED */

#ifdef DAL_I2C_MODULE_ENABLED
 #include "apm32f4xx_dal_i2c.h"
#endif /* DAL_I2C_MODULE_ENABLED */

#ifdef DAL_I2S_MODULE_ENABLED
 #include "apm32f4xx_dal_i2s.h"
#endif /* DAL_I2S_MODULE_ENABLED */

#ifdef DAL_IRDA_MODULE_ENABLED
 #include "apm32f4xx_dal_irda.h"
#endif /* DAL_IRDA_MODULE_ENABLED */

#ifdef DAL_MMC_MODULE_ENABLED
 #include "apm32f4xx_dal_mmc.h"
#endif /* DAL_MMC_MODULE_ENABLED */

#ifdef DAL_NAND_MODULE_ENABLED
  #include "apm32f4xx_dal_nand.h"
#endif /* DAL_NAND_MODULE_ENABLED */

#ifdef DAL_NOR_MODULE_ENABLED
  #include "apm32f4xx_dal_nor.h"
#endif /* DAL_NOR_MODULE_ENABLED */

#ifdef DAL_PCCARD_MODULE_ENABLED
  #include "apm32f4xx_dal_pccard.h"
#endif /* DAL_PCCARD_MODULE_ENABLED */

#ifdef DAL_PCD_MODULE_ENABLED
 #include "apm32f4xx_dal_pcd.h"
#endif /* DAL_PCD_MODULE_ENABLED */

#ifdef DAL_PMU_MODULE_ENABLED
 #include "apm32f4xx_dal_pmu.h"
#endif /* DAL_PMU_MODULE_ENABLED */

#ifdef DAL_RNG_MODULE_ENABLED
 #include "apm32f4xx_dal_rng.h"
#endif /* DAL_RNG_MODULE_ENABLED */

#ifdef DAL_RTC_MODULE_ENABLED
 #include "apm32f4xx_dal_rtc.h"
#endif /* DAL_RTC_MODULE_ENABLED */

#ifdef DAL_SRAM_MODULE_ENABLED
  #include "apm32f4xx_dal_sram.h"
#endif /* DAL_SRAM_MODULE_ENABLED */

#ifdef DAL_SDRAM_MODULE_ENABLED
  #include "apm32f4xx_dal_sdram.h"
#endif /* DAL_SDRAM_MODULE_ENABLED */

#ifdef DAL_SMBUS_MODULE_ENABLED
 #include "apm32f4xx_dal_smbus.h"
#endif /* DAL_SMBUS_MODULE_ENABLED */

#ifdef DAL_SD_MODULE_ENABLED
 #include "apm32f4xx_dal_sd.h"
#endif /* DAL_SD_MODULE_ENABLED */

#ifdef DAL_SPI_MODULE_ENABLED
 #include "apm32f4xx_dal_spi.h"
#endif /* DAL_SPI_MODULE_ENABLED */

#ifdef DAL_SMARTCARD_MODULE_ENABLED
 #include "apm32f4xx_dal_smartcard.h"
#endif /* DAL_SMARTCARD_MODULE_ENABLED */

#ifdef DAL_TMR_MODULE_ENABLED
 #include "apm32f4xx_dal_tmr.h"
#endif /* DAL_TMR_MODULE_ENABLED */

#ifdef DAL_UART_MODULE_ENABLED
 #include "apm32f4xx_dal_uart.h"
#endif /* DAL_UART_MODULE_ENABLED */

#ifdef DAL_USART_MODULE_ENABLED
 #include "apm32f4xx_dal_usart.h"
#endif /* DAL_USART_MODULE_ENABLED */

#ifdef DAL_IWDT_MODULE_ENABLED
 #include "apm32f4xx_dal_iwdt.h"
#endif /* DAL_IWDT_MODULE_ENABLED */

#ifdef DAL_WWDT_MODULE_ENABLED
 #include "apm32f4xx_dal_wwdt.h"
#endif /* DAL_WWDT_MODULE_ENABLED */

/* Assert Component */
#if (USE_FULL_ASSERT == 1U)
    #define ASSERT_PARAM(_PARAM_)                         ((_PARAM_) ? (void)(_PARAM_) : AssertFailedHandler((uint8_t *)__FILE__, __LINE__))
    /* Declaration */
    void AssertFailedHandler(uint8_t *file, uint32_t line);
#else
    #define ASSERT_PARAM(_PARAM_)                         ((void)(_PARAM_))
#endif /* USE_FULL_ASSERT */

void DAL_ErrorHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DAL_CFG_H */
