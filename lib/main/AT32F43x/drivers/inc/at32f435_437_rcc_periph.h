/*
 * rcc_at32f43x_periph.h
 *
 *  Created on: 2022年3月19日
 *      Author: emsr
 *   仿照hal 库，为at32 各外设直接定义对应的使能、重置位
 *  offset= log2(mask)
 */

#ifndef MAIN_DRIVERS_RCC_AT32F43X_PERIPH_H_
#define MAIN_DRIVERS_RCC_AT32F43X_PERIPH_H_


/********************  Bit definition for AT32F437 CRM PERIPH MASK  ***************/
/* AHB offset  0x30 */
#define   CRM_AHB1_GPIOA_PER_MASK                  ((uint32_t)0x00000001)  /*!< gpiob periph clock */
#define   CRM_AHB1_GPIOB_PER_MASK                  ((uint32_t)0x00000002)  /*!< gpiob periph clock */
#define   CRM_AHB1_GPIOC_PER_MASK                  ((uint32_t)0x00000004)  /*!< gpioc periph clock */
#define   CRM_AHB1_GPIOD_PER_MASK                  ((uint32_t)0x00000008)  /*!< gpiod periph clock */
#define   CRM_AHB1_GPIOE_PER_MASK                  ((uint32_t)0x00000010)  /*!< gpioe periph clock */
#define   CRM_AHB1_GPIOF_PER_MASK                  ((uint32_t)0x00000020)  /*!< gpiof periph clock */
#define   CRM_AHB1_GPIOG_PER_MASK                  ((uint32_t)0x00000040)  /*!< gpiog periph clock */
#define   CRM_AHB1_GPIOH_PER_MASK                  ((uint32_t)0x00000080)  /*!< gpioh periph clock */
#define   CRM_AHB1_CRC_PER_MASK                    ((uint32_t)0x00001000)  /*!< crc periph clock */
#define   CRM_AHB1_EDMA_PER_MASK                   ((uint32_t)0x00200000) /*!< edma periph clock */
#define   CRM_AHB1_DMA1_PER_MASK                   ((uint32_t)0x00400000) /*!< dma1 periph clock */
#define   CRM_AHB1_DMA2_PER_MASK                   ((uint32_t)0x01000000) /*!< dma2 periph clock */
#define   CRM_AHB1_EMAC_PER_MASK                   ((uint32_t)0x02000000) /*!< emac periph clock */
#define   CRM_AHB1_EMACTX_PER_MASK                 ((uint32_t)0x04000000) /*!< emac tx periph clock */
#define   CRM_AHB1_EMACRX_PER_MASK                 ((uint32_t)0x08000000) /*!< emac rx periph clock */
#define   CRM_AHB1_EMACPTP_PER_MASK                ((uint32_t)0x10000000) /*!< emac ptp periph clock */
#define   CRM_AHB1_OTGFS2_PER_MASK                 ((uint32_t)0x20000000) /*!< otgfs2 periph clock */
  /* ahb periph2  offset  0x34*/
#define   CRM_AHB2_DVP_PER_MASK                    ((uint32_t)0x00000001)  /*!< dvp periph clock */
#define   CRM_AHB2_OTGFS1_PER_MASK                 ((uint32_t)0x00000080)  /*!< otgfs1 periph clock */
#define   CRM_AHB2_SDIO1_PER_MASK                  ((uint32_t)0x00008000) /*!< sdio1 periph clock */
  /* ahb periph3 offset  0x38 */
#define   CRM_AHB3_XMC_PER_MASK                    ((uint32_t)0x00000001) /*!< xmc periph clock */
#define   CRM_AHB3_QSPI1_PER_MASK                  ((uint32_t)0x00000002)  /*!< qspi1 periph clock */
#define   CRM_AHB3_QSPI2_PER_MASK                  ((uint32_t)0x00004000) /*!< qspi2 periph clock */
#define   CRM_AHB3_SDIO2_PER_MASK                  ((uint32_t)0x00008000) /*!< sdio2 periph clock */
  /* apb1 periph offset 0x40 */
#define   CRM_APB1_TMR2_PER_MASK                   ((uint32_t)0x00000001)  /*!< tmr2 periph clock */
#define   CRM_APB1_TMR3_PER_MASK                   ((uint32_t)0x00000002)  /*!< tmr3 periph clock */
#define   CRM_APB1_TMR4_PER_MASK                   ((uint32_t)0x00000004)  /*!< tmr4 periph clock */
#define   CRM_APB1_TMR5_PER_MASK                   ((uint32_t)0x00000008)  /*!< tmr5 periph clock */
#define   CRM_APB1_TMR6_PER_MASK                   ((uint32_t)0x00000010)  /*!< tmr6 periph clock */
#define   CRM_APB1_TMR7_PER_MASK                   ((uint32_t)0x00000020)  /*!< tmr7 periph clock */
#define   CRM_APB1_TMR12_PER_MASK                  ((uint32_t)0x00000040)  /*!< tmr12 periph clock */
#define   CRM_APB1_TMR13_PER_MASK                  ((uint32_t)0x00000080)  /*!< tmr13 periph clock */
#define   CRM_APB1_TMR14_PER_MASK                  ((uint32_t)0x00000100)  /*!< tmr14 periph clock */
#define   CRM_APB1_WWDT_PER_MASK                   ((uint32_t)0x00000800) /*!< wwdt periph clock */
#define   CRM_APB1_SPI2_PER_MASK                   ((uint32_t)0x00004000) /*!< spi2 periph clock */
#define   CRM_APB1_SPI3_PER_MASK                   ((uint32_t)0x00008000) /*!< spi3 periph clock */
#define   CRM_APB1_USART2_PER_MASK                 ((uint32_t)0x00020000) /*!< usart2 periph clock */
#define   CRM_APB1_USART3_PER_MASK                 ((uint32_t)0x00040000) /*!< usart3 periph clock */
#define   CRM_APB1_UART4_PER_MASK                  ((uint32_t)0x00080000) /*!< uart4 periph clock */
#define   CRM_APB1_UART5_PER_MASK                  ((uint32_t)0x00100000) /*!< uart5 periph clock */
#define   CRM_APB1_I2C1_PER_MASK                   ((uint32_t)0x00200000) /*!< i2c1 periph clock */
#define   CRM_APB1_I2C2_PER_MASK                   ((uint32_t)0x00400000) /*!< i2c2 periph clock */
#define   CRM_APB1_I2C3_PER_MASK                   ((uint32_t)0x00800000) /*!< i2c3 periph clock */
#define   CRM_APB1_CAN1_PER_MASK                   ((uint32_t)0x02000000) /*!< can1 periph clock */
#define   CRM_APB1_CAN2_PER_MASK                   ((uint32_t)0x04000000) /*!< can2 periph clock */
#define   CRM_APB1_PWC_PER_MASK                    ((uint32_t)0x10000000) /*!< pwc periph clock */
#define   CRM_APB1_DAC_PER_MASK                    ((uint32_t)0x20000000) /*!< dac periph clock */
#define   CRM_APB1_UART7_PER_MASK                  ((uint32_t)0x40000000) /*!< uart7 periph clock */
#define   CRM_APB1_UART8_PER_MASK                  ((uint32_t)0x80000000) /*!< uart8 periph clock */
  /* apb2 periph offset  0x44 */
#define   CRM_APB2_TMR1_PER_MASK                   ((uint32_t)0x00000001) /*!< tmr1 periph clock */
#define   CRM_APB2_TMR8_PER_MASK                   ((uint32_t)0x00000002)  /*!< tmr8 periph clock */
#define   CRM_APB2_USART1_PER_MASK                 ((uint32_t)0x00000010)  /*!< usart1 periph clock */
#define   CRM_APB2_USART6_PER_MASK                 ((uint32_t)0x00000020)  /*!< usart6 periph clock */
#define   CRM_APB2_ADC1_PER_MASK                   ((uint32_t)0x00000100)  /*!< adc1 periph clock */
#define   CRM_APB2_ADC2_PER_MASK                   ((uint32_t)0x00000200)  /*!< adc2 periph clock */
#define   CRM_APB2_ADC3_PER_MASK                   ((uint32_t)0x00000400) /*!< adc3 periph clock */
#define   CRM_APB2_SPI1_PER_MASK                   ((uint32_t)0x00001000) /*!< spi1 periph clock */
#define   CRM_APB2_SPI4_PER_MASK                   ((uint32_t)0x00002000) /*!< spi4 periph clock */
#define   CRM_APB2_SCFG_PER_MASK                   ((uint32_t)0x00004000) /*!< scfg periph clock */
#define   CRM_APB2_TMR9_PER_MASK                   ((uint32_t)0x00010000) /*!< tmr9 periph clock */
#define   CRM_APB2_TMR10_PER_MASK                  ((uint32_t)0x00020000) /*!< tmr10 periph clock */
#define   CRM_APB2_TMR11_PER_MASK                  ((uint32_t)0x00040000) /*!< tmr11 periph clock */
#define   CRM_APB2_TMR20_PER_MASK                  ((uint32_t)0x00100000) /*!< tmr20 periph clock */
#define   CRM_APB2_ACC_PER_MASK                    ((uint32_t)0x20000000)  /*!< acc periph clock */



#endif /* MAIN_DRIVERS_RCC_AT32F43X_PERIPH_H_ */
