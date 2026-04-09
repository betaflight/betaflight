/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : CH32H417 Device Peripheral Access Layer Header File.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/  
#ifndef __CH32H417_H
#define __CH32H417_H

#ifdef __cplusplus
 extern "C" {
#endif 

#define __MPU_PRESENT             0 /* Other CH32 devices does not provide an MPU */
#define __Vendor_SysTickConfig    0 /* Set to 1 if different SysTick Config is used */	 
	 
#ifndef HSE_VALUE
#define HSE_VALUE    ((uint32_t)25000000) /* Value of the External oscillator in Hz */
#endif

/* In the following line adjust the External High Speed oscillator (HSE) Startup Timeout value */
#define HSE_STARTUP_TIMEOUT   ((uint16_t)0x10000) /* Time out for HSE start up */

#define HSI_VALUE    ((uint32_t)25000000) /* Value of the Internal oscillator in Hz */

/* CH32H417 Standard Peripheral Library version number */
#define __CH32H417_STDPERIPH_VERSION_MAIN   (0x01) /* [15:8] main version */
#define __CH32H417_STDPERIPH_VERSION_SUB    (0x00) /* [7:0] sub version */
#define __CH32H417_STDPERIPH_VERSION        ( (__CH32H417_STDPERIPH_VERSION_MAIN << 8)\
                                             |(__CH32H417_STDPERIPH_VERSION_SUB << 0))


/* Interrupt Number Definition, according to the selected device */	 
typedef enum IRQn
{
 /******  RISC-V Processor Exceptions Numbers *******************************************************/
  NonMaskableInt_IRQn         = 2,       /* 2 Non Maskable Interrupt                             */
  HardFault_IRQn              = 3,       /* 3 HardFault Interrupt                                */
  Ecall_M_Mode_IRQn           = 5,       /* 5 Ecall M Mode Interrupt                             */
  Ecall_U_Mode_IRQn           = 8,       /* 8 Ecall U Mode Interrupt                             */
  Break_Point_IRQn            = 9,       /* 9 Break Point Interrupt                              */
  SysTick0_IRQn               = 12,      /* 12 System timer 0 Interrupt                          */
  SysTick1_IRQn               = 13,      /* 13 System timer 1 Interrupt                          */
  Software_IRQn               = 14,      /* 14 software Interrupt                                */
  IPC_CH0_IRQn                = 16,      /* 16 IPC CH0 Interrupt                                 */
  IPC_CH1_IRQn                = 17,      /* 17 IPC CH1 Interrupt                                 */
  IPC_CH2_IRQn                = 18,      /* 18 IPC CH2 Interrupt                                 */
  IPC_CH3_IRQn                = 19,      /* 19 IPC CH3 Interrupt                                 */
  HSEM_IRQn                   = 28,      /* 28 HSEM Interrupt                                    */

 /******  RISC-V specific Interrupt Numbers *********************************************************/
  WWDG_IRQn                   = 32,      /* Window WatchDog Interrupt                            */
  EXTI15_8_IRQn               = 33,      /* External Line[15:8] Interrupts                       */
  FLASH_IRQn                  = 34,      /* FLASH global Interrupt                               */
  RCC_IRQn                    = 35,      /* RCC global Interrupt                                 */
  EXTI7_0_IRQn                = 36,      /* External Line[7:0] Interrupts                        */
  SPI1_IRQn                   = 37,      /* SPI1 global Interrupt                                */
  DMA1_Channel2_IRQn          = 38,      /* DMA1 Channel 2 global Interrupt                      */
  DMA1_Channel3_IRQn          = 39,      /* DMA1 Channel 3 global Interrupt                      */
  DMA1_Channel4_IRQn          = 40,      /* DMA1 Channel 4 global Interrupt                      */
  DMA1_Channel5_IRQn          = 41,      /* DMA1 Channel 5 global Interrupt                      */
  DMA1_Channel6_IRQn          = 42,      /* DMA1 Channel 6 global Interrupt                      */
  DMA1_Channel7_IRQn          = 43,      /* DMA1 Channel 7 global Interrupt                      */
  DMA1_Channel8_IRQn          = 44,      /* DMA1 Channel 8 global Interrupt                      */
  USART2_IRQn                 = 45,      /* USART2 global Interrupt                              */
  I2C1_EV_IRQn                = 46,      /* I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn                = 47,      /* I2C1 Error Interrupt                                 */
  USART1_IRQn                 = 48,      /* USART1 global Interrupt                              */
  SPI2_IRQn                   = 49,      /* SPI2 global Interrupt                                */
  SPI3_IRQn                   = 50,      /* SPI3 global Interrupt                                */
  SPI4_IRQn                   = 51,      /* SPI4 global Interrupt                                */
  I2C2_EV_IRQn                = 52,      /* I2C2 Event Interrupt                                 */
  I2C2_ER_IRQn                = 53,      /* I2C2 Error Interrupt                                 */
  USBPD_IRQn                  = 54,      /* USBPD Interrupt                                      */
  USBPDWakeUp_IRQn            = 55,      /* USBPD WakeUp Interrupt                               */
  USBHS_IRQn                  = 56,      /* USBHS global Interrupt                               */
  DMA1_Channel1_IRQn          = 57,      /* DMA1 Channel 1 global Interrupt                      */
  CAN1_SCE_IRQn               = 58,      /* CAN1 SCE Interrupt                                   */
  CAN1_TX_IRQn                = 59,      /* CAN1 TX Interrupts                                   */
  CAN1_RX0_IRQn               = 60,      /* CAN1 RX0 Interrupts                                  */
  USBSS_IRQn                  = 62,      /* USBSS Interrupt                                      */  
  USBSS_LINK_IRQn             = 63,      /* USBSS LINK Interrupt                                 */
  USBHSWakeup_IRQn            = 64,      /* USBHS WakeUp Interrupt                               */
  USBSSWakeup_IRQn            = 65,      /* USBSS WakeUp Interrupt                               */
  RTCAlarm_IRQn               = 66,      /* RTC Alarm through EXTI Line Interrupt                */
  USBFS_IRQn                  = 67,      /* USBFS global Interrupt                               */
  USBFSWakeUp_IRQn            = 68,      /* USBFS WakeUp Interrupt                               */
  ADC1_2_IRQn                 = 69,      /* ADC1 and ADC2 global Interrupt                       */
  TIM1_BRK_IRQn               = 70,      /* TIM1 Break Interrupt                                 */
  TIM1_UP_IRQn                = 71,      /* TIM1 Update Interrupt                                */
  TIM1_TRG_COM_IRQn           = 72,      /* TIM1 Trigger and Commutation Interrupt               */
  TIM1_CC_IRQn                = 73,      /* TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn                   = 74,      /* TIM2 global Interrupt                                */
  TIM3_IRQn                   = 75,      /* TIM3 global Interrupt                                */
  TIM4_IRQn                   = 76,      /* TIM4 global Interrupt                                */
  TIM5_IRQn                   = 77,      /* TIM5 global Interrupt                                */
  I2C3_EV_IRQn                = 78,      /* I2C3 Event Interrupt                                 */
  I2C3_ER_IRQn                = 79,      /* I2C3 Error Interrupt                                 */
  I2C4_EV_IRQn                = 80,      /* I2C4 Event Interrupt                                 */
  I2C4_ER_IRQn                = 81,      /* I2C4 Error Interrupt                                 */
  QSPI1_IRQn                  = 82,      /* QSPI1 Interrupt                                      */
  SERDES_IRQn                 = 83,      /* SERDES Interrupt                                     */
  USART3_IRQn                 = 84,      /* USART3 global Interrupt                              */
  USART4_IRQn                 = 85,      /* USART4 global Interrupt                              */
  TIM8_BRK_IRQn               = 86,      /* TIM8 Break Interrupt                                 */
  TIM8_UP_IRQn                = 87,      /* TIM8 Update Interrupt                                */
  TIM8_TRG_COM_IRQn           = 88,      /* TIM8 Trigger and Commutation Interrupt               */
  TIM8_CC_IRQn                = 89,      /* TIM8 Capture Compare Interrupt                       */
  TIM9_IRQn                   = 90,      /* TIM9 global Interrupt                                */
  TIM10_IRQn                  = 91,      /* TIM10 global Interrupt                               */
  TIM11_IRQn                  = 92,      /* TIM11 global Interrupt                               */
  TIM12_IRQn                  = 93,      /* TIM12 global Interrupt                               */
  FMC_IRQn                    = 94,      /* FMC global Interrupt                                 */
  SDMMC_IRQn                  = 95,      /* SDMMC global Interrupt                               */
  LPTIM1_IRQn                 = 96,      /* LPTIM1 global Interrupt                              */
  LPTIM2_IRQn                 = 97,      /* LPTIM2 global Interrupt                              */
  USART5_IRQn                 = 98,      /* USART5 global Interrupt                              */
  USART6_IRQn                 = 99,      /* USART6 global Interrupt                              */
  TIM6_IRQn                   = 100,     /* TIM6 global Interrupt                                */
  TIM7_IRQn                   = 101,     /* TIM7 global Interrupt                                */
  DMA2_Channel1_IRQn          = 102,     /* DMA2 Channel 1 global Interrupt                      */
  DMA2_Channel2_IRQn          = 103,     /* DMA2 Channel 2 global Interrupt                      */
  DMA2_Channel3_IRQn          = 104,     /* DMA2 Channel 3 global Interrupt                      */
  DMA2_Channel4_IRQn          = 105,     /* DMA2 Channel 4 global Interrupt                      */
  DMA2_Channel5_IRQn          = 106,     /* DMA2 Channel 5 global Interrupt                      */
  DMA2_Channel6_IRQn          = 107,     /* DMA2 Channel 6 global Interrupt                      */
  DMA2_Channel7_IRQn          = 108,     /* DMA2 Channel 7 global Interrupt                      */
  DMA2_Channel8_IRQn          = 109,     /* DMA2 Channel 8 global Interrupt                      */
  ETH_IRQn                    = 110,     /* ETH global Interrupt                                 */
  ETH_WKUP_IRQn               = 111,     /* ETH WakeUp Interrupt                                 */
  CAN2_SCE_IRQn               = 112,     /* CAN2 SCE Interrupt                                   */
  CAN2_TX_IRQn                = 113,     /* CAN2 TX Interrupts                                   */
  CAN2_RX0_IRQn               = 114,     /* CAN2 RX0 Interrupts                                  */  
  USART7_IRQn                 = 116,     /* USART7 global Interrupt                              */
  USART8_IRQn                 = 117,     /* USART8 global Interrupt                              */
  I3C_EV_IRQn                 = 118,     /* I3C Event Interrupt                                  */
  I3C_ER_IRQn                 = 119,     /* I3C Error Interrupt                                  */
  DVP_IRQn                    = 120,     /* DVP global Interrupt                                 */
  ECDC_IRQn                   = 121,     /* ECDC global Interrupt                                 */
  PIOC_IRQn                   = 122,     /* PIOC global Interrupt                                */
  SAI_IRQn                    = 123,     /* SAI global Interrupt                                 */
  LTDC_IRQn                   = 124,     /* LTDC global Interrupt                                */
  GPHA_IRQn                   = 125,     /* GPHA global Interrupt                                */
  DFSDM0_IRQn                 = 127,     /* DFSDM0 global Interrupt                              */
  DFSDM1_IRQn                 = 128,     /* DFSDM1 global Interrupt                              */
  SWPMI_IRQn                  = 131,     /* SWPMI global Interrupt                               */
  QSPI2_IRQn                  = 134,     /* QSPI2 Interrupt                                      */
  SWPMI_WKUP_IRQn             = 135,     /* SWPMI WakeUp Interrupt                               */
  CAN3_SCE_IRQn               = 136,     /* CAN3 SCE Interrupt                                   */
  CAN3_TX_IRQn                = 137,     /* CAN3 TX Interrupts                                   */
  CAN3_RX0_IRQn               = 138,     /* CAN3 RX0 Interrupts                                  */
  LPTIM2_WKUP_IRQn            = 140,     /* LPTIM2 WakeUp Interrupt                              */
  LPTIM1_WKUP_IRQn            = 141,     /* LPTIM1 WakeUp Interrupt                              */
  I3C_WKUP_IRQn               = 142,     /* I3C WakeUp Interrupt                                 */
  RTC_IRQn                    = 143,     /* RTC global Interrupt                                 */
  HSADC_IRQn                  = 144,     /* HSADC global Interrupt                               */
  UHSIF_IRQn                  = 145,     /* UHSIF global Interrupt                               */
  RNG_IRQn                    = 146,     /* RNG global Interrupt                                 */
  SDIO_IRQn                   = 147,     /* SDIO global Interrupt                                */
  USART_WKUP_IRQn             = 148,     /* USART wakeup Interrupt                               */
} IRQn_Type;

#include <stdint.h>
#include "core_riscv.h"
#include "system_ch32h417.h"


/* Standard Peripheral Library old definitions (maintained for legacy purpose) */
#define HSI_Value            HSI_VALUE 
#define HSE_Value            HSE_VALUE
#define HSEStartUp_TimeOut   HSE_STARTUP_TIMEOUT

/* Analog to Digital Converter */
typedef struct
{
  __IO uint32_t STATR;
  __IO uint32_t CTLR1;
  __IO uint32_t CTLR2;
  __IO uint32_t SAMPTR1;
  __IO uint32_t SAMPTR2;
  __IO uint32_t IOFR1;
  __IO uint32_t IOFR2;
  __IO uint32_t IOFR3;
  __IO uint32_t IOFR4;
  __IO uint32_t WDHTR;
  __IO uint32_t WDLTR;
  __IO uint32_t RSQR1;
  __IO uint32_t RSQR2;
  __IO uint32_t RSQR3;
  __IO uint32_t ISQR;
  __IO uint32_t IDATAR1;
  __IO uint32_t IDATAR2;
  __IO uint32_t IDATAR3;
  __IO uint32_t IDATAR4;
  __IO uint32_t RDATAR;
  uint32_t  RESERVED0;
  __IO uint32_t AUX;
  __IO uint32_t DRV;
} ADC_TypeDef;

/* Controller Area Network TxMailBox */
typedef struct
{
  __IO uint32_t TXMIR;
  __IO uint32_t TXMDTR;
  __IO uint32_t TXMDLR;
  __IO uint32_t TXMDHR;
} CAN_TxMailBox_TypeDef;

/* Controller Area Network FIFOMailBox */ 
typedef struct
{
  __IO uint32_t RXMIR;
  __IO uint32_t RXMDTR;
  __IO uint32_t RXMDLR;
  __IO uint32_t RXMDHR;
} CAN_FIFOMailBox_TypeDef;

/* Controller Area Network */  
typedef struct
{
  __IO uint32_t CTLR;
  __IO uint32_t STATR;
  __IO uint32_t TSTATR;
  __IO uint32_t RFIFO0;
  uint32_t RESERVED0;
  __IO uint32_t INTENR;
  __IO uint32_t ERRSR;
  __IO uint32_t BTIMR;
  __IO uint32_t TTCTLR;
  __IO uint32_t TTCNT;
  __IO uint32_t TERR_CNT;
  uint32_t  RESERVED1[85];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[1];
} CAN_TypeDef;

/* CRC Calculation Unit */
typedef struct
{
  __IO uint32_t DATAR;
  __IO uint8_t  IDATAR;
  uint8_t   RESERVED0;
  uint16_t  RESERVED1;
  __IO uint32_t CTLR;
} CRC_TypeDef;

/* Digital to Analog Converter */
typedef struct
{
  __IO uint32_t CTLR;
  __IO uint32_t SWTR;
  __IO uint32_t R12BDHR1;
  __IO uint32_t L12BDHR1;
  __IO uint32_t R8BDHR1;
  __IO uint32_t R12BDHR2;
  __IO uint32_t L12BDHR2;
  __IO uint32_t R8BDHR2;
  __IO uint32_t RD12BDHR;
  __IO uint32_t LD12BDHR;
  __IO uint32_t RD8BDHR;
  __IO uint32_t DOR1;
  __IO uint32_t DOR2;
} DAC_TypeDef;

/* DMA Channel Controller */
typedef struct
{
  __IO uint32_t CFGR;
  __IO uint32_t CNTR;
  __IO uint32_t PADDR;
  __IO uint32_t MADDR;
  __IO uint32_t M1ADDR;
} DMA_Channel_TypeDef;

/* DMA Controller */
typedef struct
{
  __IO uint32_t INTFR;
  __IO uint32_t INTFCR;
} DMA_TypeDef;

/* DMA MUX Controller */
typedef struct
{
  __IO uint32_t CFGR0_3;
  __IO uint32_t CFGR4_7;
  __IO uint32_t CFGR8_11;
  __IO uint32_t CFGR12_15;
} DMAMUX_TypeDef;

typedef struct
{
  /**
    * @brief dma muxctrl register
    */
  union
  {
    __IO uint8_t MUXCTL;
    struct
    {
      __IO uint8_t reqsel               : 7; /* [6:0] */
      __IO uint8_t reserved             : 1; /* [7] */
    }muxctrl_bit;
  };
} dmamux_channel_type;

/* External Interrupt/Event Controller */
typedef struct
{
  __IO uint32_t INTENR; 
  __IO uint32_t EVENR;   
  __IO uint32_t RTENR;   
  __IO uint32_t FTENR;   
  __IO uint32_t SWIEVR;  
  __IO uint32_t INTFR;   
} EXTI_TypeDef;

/* FLASH Registers */
typedef struct
{
  __IO uint32_t ACTLR;
  __IO uint32_t KEYR;
  __IO uint32_t OBKEYR;
  __IO uint32_t STATR;
  __IO uint32_t CTLR;
  __IO uint32_t ADDR;
  uint32_t RESERVED;
  __IO uint32_t OBR;
  __IO uint32_t WPR;
  __IO uint32_t MODEKEYR;
  __IO uint32_t BOOT_MODEKEYR;
} FLASH_TypeDef;

/* Option Bytes Registers */  
typedef struct
{
  __IO uint16_t RDPR;
  __IO uint16_t USER;
  __IO uint16_t Data0;
  __IO uint16_t Data1;
  __IO uint16_t WRPR0;
  __IO uint16_t WRPR1;
  __IO uint16_t WRPR2;
  __IO uint16_t WRPR3;
} OB_TypeDef;

/* FMC Bank1 Registers */  
typedef struct
{
  __IO uint32_t BTCR[8];
} FMC_Bank1_TypeDef;  

/* FMC Bank1E Registers */
typedef struct
{
  __IO uint32_t BWTR[7];
} FMC_Bank1E_TypeDef; //0x40025504

/* FMC Bank3 Registers */
typedef struct
{
  __IO uint32_t PCR;
  __IO uint32_t SR;
  __IO uint32_t PMEM;
  __IO uint32_t PATT;
  uint32_t RESERVED0;
  __IO uint32_t ECCR;
} FMC_Bank3_TypeDef; //0x40025480

/* FMC Bank5_6 Registers */
typedef struct
{
  __IO uint32_t SDCR[2];
  __IO uint32_t SDTR[2];
  __IO uint32_t SDCMR;
  __IO uint32_t SDRTR;
  __IO uint32_t SDSR;
  uint32_t RESERVED0[9];
  __IO uint32_t MISC;
} FMC_Bank5_6_TypeDef; //0x40025540

/* General Purpose I/O */
typedef struct
{
  __IO uint32_t CFGLR;
  __IO uint32_t CFGHR;
  __IO uint32_t INDR;
  __IO uint32_t OUTDR;
  __IO uint32_t BSHR;
  __IO uint32_t BCR;
  __IO uint32_t LCKR;
  __IO uint32_t SPEED;
} GPIO_TypeDef;

/* Alternate Function I/O */
typedef struct
{
  __IO uint32_t PCFR1;
  __IO uint32_t GPIOA_AFLR;
  __IO uint32_t GPIOA_AFHR;
  __IO uint32_t GPIOB_AFLR;
  __IO uint32_t GPIOB_AFHR;
  __IO uint32_t GPIOC_AFLR;
  __IO uint32_t GPIOC_AFHR;
  __IO uint32_t GPIOD_AFLR;
  __IO uint32_t GPIOD_AFHR;
  __IO uint32_t GPIOE_AFLR;
  __IO uint32_t GPIOE_AFHR;
  __IO uint32_t GPIOF_AFLR;
  __IO uint32_t GPIOF_AFHR;
  uint32_t RESERVED0[2];
  __IO uint32_t EXTICR1;
  __IO uint32_t EXTICR2;
} AFIO_TypeDef;

/* Inter Integrated Circuit Interface */
typedef struct
{
  __IO uint16_t CTLR1;
  uint16_t RESERVED0;
  __IO uint16_t CTLR2;
  uint16_t RESERVED1;
  __IO uint16_t OADDR1;
  uint16_t RESERVED2;
  __IO uint16_t OADDR2;
  uint16_t RESERVED3;
  __IO uint16_t DATAR;
  uint16_t RESERVED4;
  __IO uint16_t STAR1;
  uint16_t RESERVED5;
  __IO uint16_t STAR2;
  uint16_t RESERVED6;
  __IO uint16_t CKCFGR;
  uint16_t RESERVED7;
  __IO uint16_t RTR;
  uint16_t RESERVED8;
} I2C_TypeDef;

typedef struct
{
  __IO uint32_t CTLR;
  __IO uint32_t CFGR;
  uint32_t RESERVED0[2];
  __IO uint32_t RDBR;
  __IO uint32_t RDWR;
  __IO uint32_t TDBR;
  __IO uint32_t TDWR;
  __IO uint32_t IBIDR;
  __IO uint32_t TGTTDR;
  uint32_t RESERVED1;
  __IO uint32_t RESET;
  __IO uint32_t STATR;
  __IO uint32_t STATER;
  uint32_t RESERVED2[2];
  __IO uint32_t RMR;
  uint32_t RESERVED3[3];
  __IO uint32_t EVR;
  __IO uint32_t INTENR;
  __IO uint32_t CEVR;
  uint32_t RESERVED4;
  __IO uint32_t DEVR0;
  __IO uint32_t DEVR1;
  __IO uint32_t DEVR2;
  __IO uint32_t DEVR3;
  __IO uint32_t DEVR4;
  uint32_t RESERVED5[7];
  __IO uint32_t MAXRLR;
  __IO uint32_t MAXWLR;
  uint32_t RESERVED6[2];
  __IO uint32_t TIMINGR0;
  __IO uint32_t TIMINGR1;
  __IO uint32_t TIMINGR2;
  uint32_t RESERVED7[5];
  __IO uint32_t BCR;
  __IO uint32_t DCR;
  __IO uint32_t GETCAPR;
  __IO uint32_t CRCAPR;
  __IO uint32_t GETMDSR;
  __IO uint32_t EPIDR;
} I3C_TypeDef;

/* Independent WatchDog */
typedef struct
{
  __IO uint32_t CTLR;
  __IO uint32_t PSCR;
  __IO uint32_t RLDR;
  __IO uint32_t STATR;
} IWDG_TypeDef;

/* Power Control */
typedef struct
{
  __IO uint32_t CTLR;
  __IO uint32_t CSR;
} PWR_TypeDef;

/* Reset and Clock Control */
typedef struct
{
  __IO uint32_t CTLR;
  __IO uint32_t CFGR0;
  __IO uint32_t PLLCFGR;
  __IO uint32_t INTR;
  __IO uint32_t HB2PRSTR;
  __IO uint32_t HB1PRSTR;
  __IO uint32_t HBPCENR;
  __IO uint32_t HB2PCENR;
  __IO uint32_t HB1PCENR;
  __IO uint32_t BDCTLR;
  __IO uint32_t RSTSCKR;
  __IO uint32_t HBPRSTR;
  __IO uint32_t CFGR2;
  __IO uint32_t PLLCFGR2;
} RCC_TypeDef;

/* Real-Time Clock */
typedef struct
{
  __IO uint16_t CTLRH;
  uint16_t RESERVED0;
  __IO uint16_t CTLRL;
  uint16_t RESERVED1;
  __IO uint16_t PSCRH;
  uint16_t RESERVED2;
  __IO uint16_t PSCRL;
  uint16_t RESERVED3;
  __IO uint16_t DIVH;
  uint16_t RESERVED4;
  __IO uint16_t DIVL;
  uint16_t RESERVED5;
  __IO uint16_t CNTH;
  uint16_t RESERVED6;
  __IO uint16_t CNTL;
  uint16_t RESERVED7;
  __IO uint16_t ALRMH;
  uint16_t RESERVED8;
  __IO uint16_t ALRML;
  uint16_t RESERVED9;
} RTC_TypeDef;

/* SDIO Registers */ 
typedef struct
{
  __IO uint32_t POWER;
  __IO uint32_t CLKCR;
  __IO uint32_t ARG;
  __IO uint32_t CMD;
  __I uint32_t RESPCMD;
  __I uint32_t RESP1;
  __I uint32_t RESP2;
  __I uint32_t RESP3;
  __I uint32_t RESP4;
  __IO uint32_t DTIMER;
  __IO uint32_t DLEN;
  __IO uint32_t DCTRL;
  __I uint32_t DCOUNT;
  __I uint32_t STA;
  __IO uint32_t ICR;
  __IO uint32_t MASK;
  uint32_t RESERVED0[2];
  __I uint32_t FIFOCNT;
  uint32_t RESERVED1[5];
  __IO uint32_t DCTRL2;
  uint32_t RESERVED2[7];
  __IO uint32_t FIFO;
} SDIO_TypeDef;

/* Serial Peripheral Interface */
typedef struct
{
  __IO uint16_t CTLR1;
  uint16_t RESERVED0;
  __IO uint16_t CTLR2;
  uint16_t RESERVED1;
  __IO uint16_t STATR;
  uint16_t RESERVED2;
  __IO uint16_t DATAR;
  uint16_t RESERVED3;
  __IO uint16_t CRCR;
  uint16_t RESERVED4;
  __IO uint16_t RCRCR;
  uint16_t RESERVED5;
  __IO uint16_t TCRCR;
  uint16_t RESERVED6;
  __IO uint16_t I2SCFGR;
  uint16_t RESERVED7;
  __IO uint16_t I2SPR;
  uint16_t RESERVED8;
  __IO uint16_t HSCR;
  uint16_t RESERVED9;
} SPI_TypeDef;

/* TIM */
typedef struct
{
  __IO uint16_t CTLR1;
  uint16_t RESERVED0;
  __IO uint16_t CTLR2;
  uint16_t RESERVED1;
  __IO uint16_t SMCFGR;
  uint16_t RESERVED2;
  __IO uint16_t DMAINTENR;
  uint16_t RESERVED3;
  __IO uint16_t INTFR;
  uint16_t RESERVED4;
  __IO uint16_t SWEVGR;
  uint16_t RESERVED5;
  __IO uint16_t CHCTLR1;
  uint16_t RESERVED6;
  __IO uint16_t CHCTLR2;
  uint16_t RESERVED7;
  __IO uint16_t CCER;
  uint16_t RESERVED8;
  union
  {
    __IO uint32_t CNT_32;  //TIM9,10,11,12
    struct
    {
        __IO uint16_t CNT;
        uint16_t RESERVED9;
    };
  };
  __IO uint16_t PSC;
  uint16_t RESERVED10;
  union
  {
    __IO uint32_t ATRLR_32;//TIM9,10,11,12
    struct
    {
        __IO uint16_t ATRLR;
        uint16_t RESERVED11;
    };
  };
  __IO uint16_t RPTCR;
  uint16_t RESERVED12;
  union
  {
    __IO uint32_t CH1CVR_32;
    struct
    {
        __IO uint16_t CH1CVR;
        uint16_t RESERVED13;
    };
  };
    union
  {
    __IO uint32_t CH2CVR_32;
    struct
    {
        __IO uint16_t CH2CVR;
        uint16_t RESERVED14;
    };
  };
  union
  {
    __IO uint32_t CH3CVR_32;
    struct
    {
        __IO uint16_t CH3CVR;
        uint16_t RESERVED15;
    };
  };
  union
  {
    __IO uint32_t CH4CVR_32;
    struct
    {
        __IO uint16_t CH4CVR;
        uint16_t RESERVED16;
    };
  };
  __IO uint16_t BDTR;
  uint16_t RESERVED17;
  __IO uint16_t DMACFGR;
  uint16_t RESERVED18;
  __IO uint16_t DMAADR;
  uint16_t RESERVED19;
  __IO uint16_t AUX;
  uint16_t RESERVED20;
} TIM_TypeDef;

/* Universal Synchronous Asynchronous Receiver Transmitter */
typedef struct
{
  __IO uint16_t STATR;
  uint16_t RESERVED0;
  __IO uint16_t DATAR;
  uint16_t RESERVED1;
  __IO uint16_t BRR;
  uint16_t RESERVED2;
  __IO uint16_t CTLR1;
  uint16_t RESERVED3;
  __IO uint16_t CTLR2;
  uint16_t RESERVED4;
  __IO uint16_t CTLR3;
  uint16_t RESERVED5;
  __IO uint16_t GPR;
  uint16_t RESERVED6;
  __IO uint16_t CTLR4;
  uint16_t RESERVED7;
} USART_TypeDef;

/* Window WatchDog */
typedef struct
{
  __IO uint32_t CTLR;
  __IO uint32_t CFGR;
  __IO uint32_t STATR;
} WWDG_TypeDef;

/* OPA Registers */
typedef struct
{
  __IO uint32_t CTLR1;
  __IO uint32_t CTLR2;
  __IO uint32_t CTLR3;
  __IO uint32_t CMP_CTLR;
  __IO uint32_t CMP_STATR;
} OPA_TypeDef;

/* RNG Registers */
typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t SR;
  __IO uint32_t DR;
} RNG_TypeDef;

/* LPTIM Registers */
typedef struct
{
  __IO uint32_t ISR;
  __IO uint32_t ICR;
  __IO uint32_t IER;
  __IO uint32_t CFGR;
  __IO uint32_t CR;
  __IO uint16_t CMP;
  uint16_t Reserved0;  
  __IO uint16_t ARR;
  uint16_t Reserved1;   
  __IO uint16_t CNT;
  uint16_t Reserved2; 
} LPTIM_TypeDef;

/* DVP Registers */
typedef struct
{
  __IO uint8_t CR0;
  __IO uint8_t CR1;
  __IO uint8_t IER;
  uint8_t Reserved0;        
  __IO uint16_t ROW_NUM;
  __IO uint16_t COL_NUM;
  __IO uint32_t DMA_BUF0;
  __IO uint32_t DMA_BUF1;
  __IO uint8_t IFR;
  __IO uint8_t STATUS;
  uint16_t Reserved1;            
  __IO uint16_t ROW_CNT;
  uint16_t Reserved2;           
  __IO uint16_t HOFFCNT;
  __IO uint16_t VST;
  __IO uint16_t CAPCNT;
  __IO uint16_t VLINE;
  __IO uint32_t DR;
} DVP_TypeDef;

/* PD Registers */
typedef struct
{
    union
    {
        __IO uint32_t USBPD_CONFIG;
        struct
        {
            __IO uint16_t CONFIG;
            __IO uint16_t BMC_CLK_CNT;
        };
    };
    union
    {
        __IO uint32_t USBPD_CONTROL;
        struct
        {
            union
            {
                __IO uint16_t R16_CONTROL;
                struct
                {
                    __IO uint8_t  CONTROL;
                    __IO uint8_t  TX_SEL;
                };
            };
            __IO uint16_t BMC_TX_SZ;
        };
    };
    union
    {
        __IO uint32_t USBPD_STATUS;
        struct
        {
            union
            {
                __IO uint16_t R16_STATUS;
                struct
                {
                    __IO uint8_t  DATA_BUF;
                    __IO uint8_t  STATUS;
                };
            };
            __IO uint16_t BMC_BYTE_CNT;
        };
    };
    union
    {
        __IO uint32_t USBPD_PORT;
        struct
        {
            __IO uint16_t PORT_CC1;
            __IO uint16_t PORT_CC2;
        };
    };
    __IO uint32_t USBPD_DMA;
} USBPD_TypeDef;

/* USBSS Deveice Registers */
typedef struct 
{
  __IO uint8_t  UEP_TX_CFG;             // 0x0
  __IO uint8_t  UEP_TX_CR;              // 0x1
  __IO uint8_t  UEP_TX_SEQ;             // 0x2
  __IO uint8_t  UEP_TX_ST;              // 0x3
  __IO uint8_t  UEP_TX_CHAIN_CR;        // 0x4
  __IO uint8_t  UEP_TX_CHAIN_ST;        // 0x5
  __IO uint16_t UEP_TX_CHAIN_LEN;       // 0x6
  __IO uint8_t  UEP_TX_CHAIN_EXP_NUMP;  // 0x8
  __IO uint8_t  UEP_TX_CHAIN_NUMP;      // 0x9
  __IO uint16_t UEP_TX_DMA_OFS;         // 0xA
  __IO uint32_t UEP_TX_DMA;             // 0xC
}USBSS_EP_TX_TypeDef;

typedef struct 
{
  __IO uint8_t  UEP_RX_CFG;             // 0x0
  __IO uint8_t  UEP_RX_CR;              // 0x1
  __IO uint8_t  UEP_RX_SEQ;             // 0x2
  __IO uint8_t  UEP_RX_ST;              // 0x3
  __IO uint8_t  UEP_RX_CHAIN_CR;        // 0x4
  __IO uint8_t  UEP_RX_CHAIN_ST;        // 0x5
  __IO uint16_t UEP_RX_CHAIN_LEN;       // 0x6
  __IO uint8_t  UEP_RX_CHAIN_MAX_NUMP;  // 0x8
  __IO uint8_t  UEP_RX_CHAIN_NUMP;      // 0x9
  __IO uint16_t UEP_RX_DMA_OFS;         // 0xA
  __IO uint32_t UEP_RX_DMA;             // 0xC
}USBSS_EP_RX_TypeDef;

typedef struct
{
    __IO uint32_t LINK_CFG;          
    __IO uint32_t LINK_CTRL;         
    __IO uint32_t LINK_INT_CTRL;     
    __IO uint32_t LINK_INT_FLAG;     
    __IO uint32_t LINK_STATUS;       
    uint8_t Reserved0[3];           
    __IO uint8_t LINK_ITP_PRE;       
    uint8_t Reserved1[5];           
    __IO uint8_t LINK_U2_INACT_TIMER;
    uint8_t Reserved2[10];          
    __IO uint8_t LINK_U1_WKUP_FILTER;
    uint8_t Reserved3[3];           
    __IO uint8_t LINK_U2_WKUP_FILTER;
    uint8_t Reserved4[3];           
    __IO uint8_t LINK_U3_WKUP_FILTER;
    uint8_t Reserved5[15];          
    __IO uint16_t LINK_ISO_DLY;      
    uint8_t Reserved6[14];          
    __IO uint16_t LINK_LPM_CR;       
    uint8_t Reserved7[2];                 
    __IO uint32_t LINK_LMP_PORT_CAP; 
    __IO uint32_t LINK_LMP_RX_DATA0; 
    __IO uint32_t LINK_LMP_RX_DATA1; 
    __IO uint32_t LINK_LMP_RX_DATA2; 
    __IO uint32_t LINK_LMP_TX_DATA0; 
    __IO uint32_t LINK_LMP_TX_DATA1; 
    __IO uint32_t LINK_LMP_TX_DATA2; 
    __IO uint32_t USB_CONTROL;
    __IO uint32_t USB_STATUS;
    __IO uint32_t USB_ITP;
    __IO uint32_t USB_ITP_ADJ;
    __IO uint16_t UEP_TX_EN;
    __IO uint16_t UEP_RX_EN;
    __IO uint32_t UEP0_TX_CTRL;
    __IO uint32_t UEP0_RX_CTRL;
    __IO uint32_t UEP0_TX_DMA;
    __IO uint32_t UEP0_RX_DMA;
    __IO uint32_t UEP0_TX_DMA_OFS;
    __IO uint32_t UEP0_RX_DMA_OFS;
    uint8_t Reserved8[36]; 
    __IO USBSS_EP_TX_TypeDef EP1_TX;
    __IO USBSS_EP_RX_TypeDef EP1_RX;
    __IO USBSS_EP_TX_TypeDef EP2_TX;
    __IO USBSS_EP_RX_TypeDef EP2_RX;
    __IO USBSS_EP_TX_TypeDef EP3_TX;
    __IO USBSS_EP_RX_TypeDef EP3_RX;
    __IO USBSS_EP_TX_TypeDef EP4_TX;
    __IO USBSS_EP_RX_TypeDef EP4_RX;
    __IO USBSS_EP_TX_TypeDef EP5_TX;
    __IO USBSS_EP_RX_TypeDef EP5_RX;
    __IO USBSS_EP_TX_TypeDef EP6_TX;
    __IO USBSS_EP_RX_TypeDef EP6_RX;
    __IO USBSS_EP_TX_TypeDef EP7_TX;
    __IO USBSS_EP_RX_TypeDef EP7_RX;
}USBSSD_TypeDef;

typedef struct
{
    __IO uint32_t LINK_CFG;          
    __IO uint32_t LINK_CTRL;         
    __IO uint32_t LINK_INT_CTRL;     
    __IO uint32_t LINK_INT_FLAG;     
    __IO uint32_t LINK_STATUS;       
    uint8_t Reserved0[3];           
    __IO uint8_t LINK_ITP_PRE;       
    uint8_t Reserved1[5];           
    __IO uint8_t LINK_U2_INACT_TIMER;
    uint8_t Reserved2[10];          
    __IO uint8_t LINK_U1_WKUP_FILTER;
    uint8_t Reserved3[3];           
    __IO uint8_t LINK_U2_WKUP_FILTER;
    uint8_t Reserved4[3];           
    __IO uint8_t LINK_U3_WKUP_FILTER;
    uint8_t Reserved5[15];          
    __IO uint16_t LINK_ISO_DLY;      
    uint8_t Reserved6[14];          
    __IO uint16_t LINK_LPM_CR;       
    uint8_t Reserved7[2];                 
    __IO uint32_t LINK_LMP_PORT_CAP; 
    __IO uint32_t LINK_LMP_RX_DATA0; 
    __IO uint32_t LINK_LMP_RX_DATA1; 
    __IO uint32_t LINK_LMP_RX_DATA2; 
    __IO uint32_t LINK_LMP_TX_DATA0; 
    __IO uint32_t LINK_LMP_TX_DATA1; 
    __IO uint32_t LINK_LMP_TX_DATA2; 

    __IO uint32_t USB_CONTROL;
    __IO uint32_t USB_STATUS;
    __IO uint32_t USB_ITP;
    __IO uint32_t USB_ITP_ADJ;
    __IO uint16_t UEP_TX_EN;
    __IO uint16_t UEP_RX_EN;
    
    __IO uint32_t UH_TX_CTRL;
    __IO uint32_t UH_RX_CTRL;
    __IO uint32_t UH_TX_DMA;
    __IO uint32_t UH_RX_DMA;
    __IO uint32_t UH_TX_DMA_OFS;
    __IO uint32_t UH_RX_DMA_OFS;
    __IO uint16_t HOST_TX_NUMP;
    __IO uint16_t HOST_RX_NUMP;
    __IO uint32_t HOST_STATUS;
    __IO uint16_t HOST_TX_FC_STATUS;
    __IO uint16_t HOST_RX_FC_STATUS;
    __IO uint32_t TP_RX_DATA0;
    __IO uint32_t TP_RX_DATA1;
    __IO uint32_t TP_RX_DATA2;
}USBSSH_TypeDef;

// /* USBHS Deveice Registers */
// typedef struct
// {
//   __IO uint8_t  CONTROL;
//   __IO uint8_t  BASE_MODE;
//   __IO uint8_t  INT_EN;
//   __IO uint8_t  DEV_AD;
//   __IO uint8_t  WAKE_CTRL;
//   __IO uint8_t  TEST_MODE;
//   __IO uint16_t LPM_DATA;

//   __IO uint8_t  INT_FG;
//   __IO uint8_t  INT_ST;
//   __IO uint8_t  MIS_ST;
//   uint8_t  RESERVED0;

//   __IO uint16_t FRAME_NO;
//   __IO uint16_t BUS;

//   __IO uint16_t UEP_TX_EN;
//   __IO uint16_t UEP_RX_EN;
//   __IO uint16_t UEP_TX_TOG_AUTO;
//   __IO uint16_t UEP_RX_TOG_AUTO;

//   __IO uint8_t  UEP_TX_BURST;
//   __IO uint8_t  UEP_TX_BURST_MODE;
//   __IO uint8_t  UEP_RX_BURST;
//   __IO uint8_t  UEP_RX_RES_MODE;

//   __IO uint32_t UEP_AF_MODE;
//   __IO uint32_t UEP0_DMA;
//   __IO uint32_t UEP1_RX_DMA;
//   __IO uint32_t UEP2_RX_DMA;
//   __IO uint32_t UEP3_RX_DMA;
//   __IO uint32_t UEP4_RX_DMA;
//   __IO uint32_t UEP5_RX_DMA;
//   __IO uint32_t UEP6_RX_DMA;
//   __IO uint32_t UEP7_RX_DMA;
//   __IO uint32_t UEP1_TX_DMA;
//   __IO uint32_t UEP2_TX_DMA;
//   __IO uint32_t UEP3_TX_DMA;
//   __IO uint32_t UEP4_TX_DMA;
//   __IO uint32_t UEP5_TX_DMA;
//   __IO uint32_t UEP6_TX_DMA;
//   __IO uint32_t UEP7_TX_DMA;
//   __IO uint32_t UEP0_MAX_LEN;
//   __IO uint32_t UEP1_MAX_LEN;
//   __IO uint32_t UEP2_MAX_LEN;
//   __IO uint32_t UEP3_MAX_LEN;
//   __IO uint32_t UEP4_MAX_LEN;
//   __IO uint32_t UEP5_MAX_LEN;
//   __IO uint32_t UEP6_MAX_LEN;
//   __IO uint32_t UEP7_MAX_LEN;

//   __IO uint16_t UEP0_RX_LEN;
//   uint16_t RESERVED1;
//   __IO uint16_t UEP1_RX_LEN;
//   __IO uint16_t UEP1_RX_SIZE;
//   __IO uint16_t UEP2_RX_LEN;
//   __IO uint16_t UEP2_RX_SIZE;
//   __IO uint16_t UEP3_RX_LEN;
//   __IO uint16_t UEP3_RX_SIZE;
//   __IO uint16_t UEP4_RX_LEN;
//   __IO uint16_t UEP4_RX_SIZE;
//   __IO uint16_t UEP5_RX_LEN;
//   __IO uint16_t UEP5_RX_SIZE;
//   __IO uint16_t UEP6_RX_LEN;
//   __IO uint16_t UEP6_RX_SIZE;
//   __IO uint16_t UEP7_RX_LEN;
//   __IO uint16_t UEP7_RX_SIZE;
//   __IO uint16_t UEP0_TX_LEN;
//   __IO uint8_t  UEP0_TX_CTRL;
//   __IO uint8_t  UEP0_RX_CTRL;

//   __IO uint16_t UEP1_TX_LEN;
//   __IO uint8_t  UEP1_TX_CTRL;
//   __IO uint8_t  UEP1_RX_CTRL;
//   __IO uint16_t UEP2_TX_LEN;
//   __IO uint8_t  UEP2_TX_CTRL;
//   __IO uint8_t  UEP2_RX_CTRL;
//   __IO uint16_t UEP3_TX_LEN;
//   __IO uint8_t  UEP3_TX_CTRL;
//   __IO uint8_t  UEP3_RX_CTRL;
//   __IO uint16_t UEP4_TX_LEN;
//   __IO uint8_t  UEP4_TX_CTRL;
//   __IO uint8_t  UEP4_RX_CTRL;
//   __IO uint16_t UEP5_TX_LEN;
//   __IO uint8_t  UEP5_TX_CTRL;
//   __IO uint8_t  UEP5_RX_CTRL;
//   __IO uint16_t UEP6_TX_LEN;
//   __IO uint8_t  UEP6_TX_CTRL;
//   __IO uint8_t  UEP6_RX_CTRL;
//   __IO uint16_t UEP7_TX_LEN;
//   __IO uint8_t  UEP7_TX_CTRL;
//   __IO uint8_t  UEP7_RX_CTRL;

//   __IO uint16_t UEP_TX_ISO;
//   __IO uint16_t UEP_RX_ISO;

//   __IO uint32_t UEP1_RX_FIFO;
//   __IO uint32_t UEP2_RX_FIFO;
//   __IO uint32_t UEP3_RX_FIFO;
//   __IO uint32_t UEP4_RX_FIFO;
//   __IO uint32_t UEP5_RX_FIFO;
//   __IO uint32_t UEP6_RX_FIFO;
//   __IO uint32_t UEP7_RX_FIFO;
//   __IO uint32_t UEP1_TX_FIFO;
//   __IO uint32_t UEP2_TX_FIFO;
//   __IO uint32_t UEP3_TX_FIFO;
//   __IO uint32_t UEP4_TX_FIFO;
//   __IO uint32_t UEP5_TX_FIFO;
//   __IO uint32_t UEP6_TX_FIFO;
//   __IO uint32_t UEP7_TX_FIFO;
// } USBHSD_TypeDef;

// /* USBHS Host Registers */
// typedef struct  __attribute__((packed))
// {
//   __IO uint8_t  CFG;
//   uint8_t  RESERVED0;
//   __IO uint8_t  INT_EN;
//   __IO uint8_t  DEV_ADDR;
//   __IO uint32_t CONTROL;

//   __IO uint8_t  INT_FLAG;
//   __IO uint8_t  INT_ST;
//   __IO uint8_t  MIS_ST;
//   uint8_t  RESERVED1;

//   __IO uint32_t LPM;
//   __IO uint32_t SPLIT;
//   __IO uint32_t FRAME;
//   __IO uint32_t TX_LEN;
//   __IO uint32_t RX_LEN;
//   __IO uint32_t RX_MAX_LEN;
//   __IO uint32_t RX_DMA;
//   __IO uint32_t TX_DMA;
//   __IO uint32_t PORT_CTRL;
//   __IO uint8_t  PORT_CFG;
//   uint8_t  RESERVED2;
//   __IO uint8_t  PORT_INT_EN;
//   __IO uint8_t  PORT_TEST_CT;

//   __IO uint16_t PORT_STATUS;
//   __IO uint8_t  PORT_STATUS_CHG;
//   uint8_t  RESERVED3[5];
//   __IO uint32_t ROOT_BC_CTRL;
// } USBHSH_TypeDef;

/* USBFS Device Registers */
typedef struct
{
   __IO uint8_t  BASE_CTRL;
   __IO uint8_t  UDEV_CTRL;
   __IO uint8_t  INT_EN;
   __IO uint8_t  DEV_ADDR;
   uint8_t  RESERVED0;
   __IO uint8_t  MIS_ST;
   __IO uint8_t  INT_FG;
   __IO uint8_t  INT_ST;
   __IO uint16_t RX_LEN;

   uint16_t RESERVED1;
   __IO uint8_t  UEP4_1_MOD;
   __IO uint8_t  UEP2_3_MOD;
   __IO uint8_t  UEP5_6_MOD;
   __IO uint8_t  UEP7_MOD;
   __IO uint32_t UEP0_DMA;
   __IO uint32_t UEP1_DMA;
   __IO uint32_t UEP2_DMA;
   __IO uint32_t UEP3_DMA;
   __IO uint32_t UEP4_DMA;
   __IO uint32_t UEP5_DMA;
   __IO uint32_t UEP6_DMA;
   __IO uint32_t UEP7_DMA;
   __IO uint8_t  UEP0_TX_LEN;
   uint8_t  RESERVED2;
   __IO uint8_t  UEP0_TX_CTRL;
   __IO uint8_t  UEP0_RX_CTRL;
   __IO uint8_t  UEP1_TX_LEN;
   uint8_t  RESERVED3;
   __IO uint8_t  UEP1_TX_CTRL;
   __IO uint8_t  UEP1_RX_CTRL;
   __IO uint8_t  UEP2_TX_LEN;
   uint8_t  RESERVED4;
   __IO uint8_t  UEP2_TX_CTRL;
   __IO uint8_t  UEP2_RX_CTRL;
   __IO uint16_t UEP3_TX_LEN;
   __IO uint8_t  UEP3_TX_CTRL;
   __IO uint8_t  UEP3_RX_CTRL;
   __IO uint8_t  UEP4_TX_LEN;
   uint8_t  RESERVED5;
   __IO uint8_t  UEP4_TX_CTRL;
   __IO uint8_t  UEP4_RX_CTRL;
   __IO uint8_t  UEP5_TX_LEN;
   uint8_t  RESERVED6;
   __IO uint8_t  UEP5_TX_CTRL;
   __IO uint8_t  UEP5_RX_CTRL;
   __IO uint8_t  UEP6_TX_LEN;
   uint8_t  RESERVED7;
   __IO uint8_t  UEP6_TX_CTRL;
   __IO uint8_t  UEP6_RX_CTRL;
   __IO uint8_t  UEP7_TX_LEN;
   uint8_t  RESERVED8;
   __IO uint8_t  UEP7_TX_CTRL;
   __IO uint8_t  UEP7_RX_CTRL;
   uint32_t RESERVED9;
   __IO uint32_t OTG_CR;
   __IO uint32_t OTG_SR;
}USBFSD_TypeDef;

/* USBFS Host Registers */
typedef struct  __attribute__((packed))
{
   __IO uint8_t   BASE_CTRL;
   __IO uint8_t   HOST_CTRL;
   __IO uint8_t   INT_EN;
   __IO uint8_t   DEV_ADDR;
   uint8_t   RESERVED0;
   __IO uint8_t   MIS_ST;
   __IO uint8_t   INT_FG;
   __IO uint8_t   INT_ST;
   __IO uint16_t  RX_LEN;

   uint16_t  RESERVED1;
   uint8_t   RESERVED2;
   __IO uint8_t   HOST_EP_MOD;
   uint16_t  RESERVED3;
   uint32_t  RESERVED4;
   uint32_t  RESERVED5;
   __IO uint32_t  HOST_RX_DMA;
   __IO uint32_t  HOST_TX_DMA;
   uint32_t  RESERVED6;
   uint32_t  RESERVED7;
   uint32_t  RESERVED8;
   uint32_t  RESERVED9;
   uint32_t  RESERVED10;
   uint16_t  RESERVED11;
   __IO uint16_t  HOST_SETUP;
   __IO uint8_t   HOST_EP_PID;
   uint8_t   RESERVED12;
   uint8_t   RESERVED13;
   __IO uint8_t   HOST_RX_CTRL;
   __IO uint16_t  HOST_TX_LEN;
   __IO uint8_t   HOST_TX_CTRL;
   uint8_t   RESERVED14;
   uint32_t  RESERVED15;
   uint32_t  RESERVED16;
   uint32_t  RESERVED17;
   uint32_t  RESERVED18;
   uint32_t  RESERVED19;
   __IO uint32_t  OTG_CR;
   __IO uint32_t  OTG_SR;
}USBFSH_TypeDef;

/* Ethernet MAC Registers */
typedef struct
{
  __IO uint32_t MACCR;
  __IO uint32_t MACFFR;
  __IO uint32_t MACHTHR;
  __IO uint32_t MACHTLR;
  __IO uint32_t MACMIIAR;
  __IO uint32_t MACMIIDR;
  __IO uint32_t MACFCR;
  __IO uint32_t MACVLANTR;
  uint32_t RESERVED0[2];
  __IO uint32_t MACRWUFFR;
  __IO uint32_t MACPMTCSR;
  uint32_t RESERVED1[2];
  __IO uint32_t MACSR;
  __IO uint32_t MACIMR;
  __IO uint32_t MACA0HR;
  __IO uint32_t MACA0LR;
  __IO uint32_t MACA1HR;
  __IO uint32_t MACA1LR;
  __IO uint32_t MACA2HR;
  __IO uint32_t MACA2LR;
  __IO uint32_t MACA3HR;
  __IO uint32_t MACA3LR;
  uint32_t RESERVED2[14];
  __IO uint32_t MACCFG0;
  uint32_t RESERVED10[25];
  __IO uint32_t MMCCR;
  __IO uint32_t MMCRIR;
  __IO uint32_t MMCTIR;
  __IO uint32_t MMCRIMR;
  __IO uint32_t MMCTIMR;
  uint32_t RESERVED3[14];
  __IO uint32_t MMCTGFSCCR;
  __IO uint32_t MMCTGFMSCCR;
  uint32_t RESERVED4[5];
  __IO uint32_t MMCTGFCR;
  uint32_t RESERVED5[10];
  __IO uint32_t MMCRFCECR;
  __IO uint32_t MMCRFAECR;
  uint32_t RESERVED6[10];
  __IO uint32_t MMCRGUFCR;
  uint32_t RESERVED7[334];
  __IO uint32_t PTPTSCR;
  __IO uint32_t PTPSSIR;
  __IO uint32_t PTPTSHR;
  __IO uint32_t PTPTSLR;
  __IO uint32_t PTPTSHUR;
  __IO uint32_t PTPTSLUR;
  __IO uint32_t PTPTSAR;
  __IO uint32_t PTPTTHR;
  __IO uint32_t PTPTTLR;
  uint32_t RESERVED8[567];
  __IO uint32_t DMABMR;
  __IO uint32_t DMATPDR;
  __IO uint32_t DMARPDR;
  __IO uint32_t DMARDLAR;
  __IO uint32_t DMATDLAR;
  __IO uint32_t DMASR;
  __IO uint32_t DMAOMR;
  __IO uint32_t DMAIER;
  __IO uint32_t DMAMFBOCR;
  uint32_t RESERVED9[9];
  __IO uint32_t DMACHTDR;
  __IO uint32_t DMACHRDR;
  __IO uint32_t DMACHTBAR;
  __IO uint32_t DMACHRBAR;
} ETH_TypeDef;

/* SDMMC Registers */
typedef struct
{
  __IO uint32_t ARGUMENT;
  __IO uint16_t CMD_SET;
  uint16_t RESERVED0;
  __IO uint32_t RESPONSE0;
  __IO uint32_t RESPONSE1;
  __IO uint32_t RESPONSE2;
  union
  {
      __IO uint32_t RESPONSE3;
      __IO uint32_t WRITE_CONT;
  };
  __IO uint16_t CONTROL;
  uint16_t RESERVED1;
  __IO uint8_t TIMEOUT;
  uint8_t RESERVED3[3];
  __IO uint32_t STATUS;
  __IO uint16_t INT_FG;
  uint16_t RESERVED4;
  __IO uint16_t INT_EN;
  uint16_t RESERVED5;
  __IO uint32_t DMA_BEG1;
  __IO uint32_t BLOCK_CFG;
  __IO uint32_t TRAN_MODE;
  __IO uint16_t CLK_DIV;
  uint16_t RESERVED6;
  __IO uint32_t DMA_BEG2;
  __IO uint32_t TUNE_DATO;
  __IO uint32_t TUNE_DATI;
  __IO uint32_t TUNE_CLK_CMD;
} SDMMC_TypeDef;

/* SAI Registers */
typedef struct
{
    __IO uint32_t CFGR1;
    __IO uint32_t CFGR2;
    __IO uint32_t FRCR;
    __IO uint32_t SLOTR;
    __IO uint32_t INTENR;
    __IO uint32_t SR;
    uint32_t RESERVED0;
    __IO uint32_t DATAR;
} SAI_Block_TypeDef;

/* QSPI Registers */
typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t DCR;
  __IO uint32_t SR;
  __IO uint32_t FCR;
  __IO uint32_t DLR;
  __IO uint32_t CCR;
  __IO uint32_t AR;
  __IO uint32_t ABR;
  __IO uint32_t DR;
  __IO uint32_t PSMKR;
  __IO uint32_t PSMAR;
  __IO uint32_t PIR;
  __IO uint32_t LPTR;
} QSPI_TypeDef;

/* SWPMI Registers */
typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t BRR;
  uint32_t RESERVED0;
  __IO uint32_t ISR;
  __IO uint32_t ICR;
  __IO uint32_t IER;
  __IO uint32_t RFL;
  __IO uint32_t TDR;
  __IO uint32_t RDR;
  __IO uint32_t OR;
} SWPMI_TypeDef;

/* ECDC Registers */
typedef struct
{
  __IO uint32_t CTRL;
  __IO uint32_t INT_FG;
  __IO uint32_t KEY_255T224;
  __IO uint32_t KEY_223T192;
  __IO uint32_t KEY_191T160;
  __IO uint32_t KEY_159T128;
  __IO uint32_t KEY_127T96;
  __IO uint32_t KEY_95T64;
  __IO uint32_t KEY_63T32;
  __IO uint32_t KEY_31T0;
  __IO uint32_t IV_127T96;
  __IO uint32_t IV_95T64;
  __IO uint32_t IV_63T32;
  __IO uint32_t IV_31T0;
  uint32_t RESERVED3[2];
  __IO uint32_t SGSD_127T96;
  __IO uint32_t SGSD_95T64;
  __IO uint32_t SGSD_63T32;
  __IO uint32_t SGSD_31T0;
  __IO uint32_t SGRT_127T96;
  __IO uint32_t SGRT_95T64;
  __IO uint32_t SGRT_63T32;
  __IO uint32_t SGRT_31T0;
  __IO uint32_t SRC_ADDR;
  __IO uint32_t DST_ADDR;
  __IO uint32_t SRAM_LEN;
} ECDC_TypeDef;

/* DFSDM filter Registers */
typedef struct
{
  __IO uint32_t CR1;
  uint32_t RESERVED0;
  __IO uint32_t CR2;
  uint32_t RESERVED1;
  __IO uint32_t ISR;
  uint32_t RESERVED2;
  __IO uint32_t ICR;
  uint32_t RESERVED3;
  __IO uint32_t JCHGR;
  uint32_t RESERVED4;
  __IO uint32_t FCR3;
  uint32_t RESERVED5;
  __IO uint32_t JDATAR;
  uint32_t RESERVED6;
  __IO uint32_t RDATAR;
  uint32_t RESERVED7;
  __IO uint32_t AWHTR;
  uint32_t RESERVED8;
  __IO uint32_t AWLTR;
  uint32_t RESERVED9;
  __IO uint32_t AWSR;
  uint32_t RESERVED10;
  __IO uint32_t AWCFR;
  uint32_t RESERVED11;
  __IO uint32_t EXMAX;
  uint32_t RESERVED12;
  __IO uint32_t EXMIN;
  uint32_t RESERVED13;
  __IO uint32_t NVTIMR;
  uint32_t RESERVED14;
} DFSDM_FLT_TypeDef;

/* DFSDM channel Registers */
typedef struct
{
  __IO uint32_t CFGR1;
  uint32_t RESERVED0;
  __IO uint32_t CFGR2;
  uint32_t RESERVED1;
  __IO uint32_t AWSCDR;
  uint32_t RESERVED2;
  __IO uint32_t WDATR;
  uint32_t RESERVED3;
  union{
    __IO uint32_t DATINR;
    struct{
       __IO int16_t DATINR0;
       __IO int16_t DATINR1;
    };
  };
} DFSDM_Channel_TypeDef;

/* LTDC Registers */
typedef struct
{
  __IO uint32_t SSCR;
  __IO uint32_t BPCR;
  __IO uint32_t AWCR;
  __IO uint32_t TWCR;
  __IO uint32_t GCR;
  __IO uint32_t SRCR;
  __IO uint32_t BCCR;
  __IO uint32_t IER;
  __IO uint32_t ISR;
  __IO uint32_t ICR;
  __IO uint32_t LIPCR;
  __IO uint32_t CPSR;
  __IO uint32_t CDSR;
} LTDC_TypeDef;

/* LTDC Layer Registers */
typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t WHPCR;
  __IO uint32_t WVPCR;
  __IO uint32_t CKCR;
  __IO uint32_t PFCR;
  __IO uint32_t CACR;
  __IO uint32_t DCCR;
  __IO uint32_t BFCR;
  __IO uint32_t CFBAR;
  __IO uint32_t CFBLR;
  __IO uint32_t CFBLNR;
  __IO uint32_t CLUTWR;
} LTDC_Layer_TypeDef;

/* GPHA Registers */
typedef struct
{
  __IO uint32_t CTLR;
  __IO uint32_t ISR;
  __IO uint32_t IFCR;
  __IO uint32_t FGMAR;
  __IO uint32_t FGOR;
  __IO uint32_t BGMAR;
  __IO uint32_t BGOR;
  __IO uint32_t FGPFCCR;
  __IO uint32_t FGCOLR;
  __IO uint32_t BGPFCCR;
  __IO uint32_t BGCOLR;
  __IO uint32_t FGCMAR;
  __IO uint32_t BGCMAR;
  __IO uint32_t OPFCCR;
  __IO uint32_t OCOLR;
  __IO uint32_t OMAR;
  __IO uint32_t OOR;
  __IO uint32_t NLR;
  __IO uint32_t LWR;
  __IO uint32_t AMTCR;
  __IO uint32_t FGCWRS;
  __IO uint32_t FGCDAT;
  __IO uint32_t BGCWRS;
  __IO uint32_t BGCDAT;
} GPHA_TypeDef;

/* HSADC Registers */
typedef struct
{
  __IO uint32_t CFGR;
  __IO uint32_t CTLR1;
  __IO uint32_t CTLR2;
  __IO uint32_t STATR;
  __IO uint32_t DATAR;
  __IO uint32_t ADDR0;
  __IO uint32_t ADDR1;
} HSADC_TypeDef;

/* SerDes Registers */
typedef struct
{
  __IO uint32_t CTRL;
  __IO uint32_t INT_EN;
  __IO uint32_t STATUS;
  __IO uint32_t RTX_CTRL;
  __IO uint32_t RX_LEN0;
  __IO uint32_t DATA0;
  __IO uint32_t DMA_0;
  __IO uint32_t RX_LEN1;
  __IO uint32_t DATA1;
  __IO uint32_t DMA_1;
} SDS_TypeDef;

/* Peripheral memory map */
#define FLASH_BASE            ((uint32_t)0x08000000) /* FLASH base address in the alias region */
#define ITCM_BASE             ((uint32_t)0x200A0000) /* ITCM base address in the alias region */
#define DTCM_BASE             ((uint32_t)0x200C0000) /* DTCM base address in the alias region */
#define SRAM_BASE             ((uint32_t)0x20100000) /* SRAM base address in the alias region */
#define PERIPH_BASE           ((uint32_t)0x40000000) /* Peripheral base address in the alias region */

#define HBPERIPH_BASE         (PERIPH_BASE)

#define TIM2_BASE             (HBPERIPH_BASE + 0x00000)
#define TIM3_BASE             (HBPERIPH_BASE + 0x00400)
#define TIM4_BASE             (HBPERIPH_BASE + 0x00800)
#define TIM5_BASE             (HBPERIPH_BASE + 0x00C00)
#define TIM6_BASE             (HBPERIPH_BASE + 0x01000)
#define TIM7_BASE             (HBPERIPH_BASE + 0x01400)
#define USART6_BASE           (HBPERIPH_BASE + 0x01800)
#define USART7_BASE           (HBPERIPH_BASE + 0x01C00)
#define USART8_BASE           (HBPERIPH_BASE + 0x02000)
#define LPTIM1_BASE           (HBPERIPH_BASE + 0x02400)
#define RTC_BASE              (HBPERIPH_BASE + 0x02800)
#define WWDG_BASE             (HBPERIPH_BASE + 0x02C00)
#define IWDG_BASE             (HBPERIPH_BASE + 0x03000)
#define LPTIM2_BASE           (HBPERIPH_BASE + 0x03400)
#define SPI2_BASE             (HBPERIPH_BASE + 0x03800)
#define SPI3_BASE             (HBPERIPH_BASE + 0x03C00)
#define SPI4_BASE             (HBPERIPH_BASE + 0x04000)
#define USART2_BASE           (HBPERIPH_BASE + 0x04400)
#define USART3_BASE           (HBPERIPH_BASE + 0x04800)
#define USART4_BASE           (HBPERIPH_BASE + 0x04C00)
#define USART5_BASE           (HBPERIPH_BASE + 0x05000)
#define I2C1_BASE             (HBPERIPH_BASE + 0x05400)
#define I2C2_BASE             (HBPERIPH_BASE + 0x05800)
#define I2C3_BASE             (HBPERIPH_BASE + 0x05C00)
#define CAN1_BASE             (HBPERIPH_BASE + 0x06400)
#define CAN2_BASE             (HBPERIPH_BASE + 0x06800)
#define PWR_BASE              (HBPERIPH_BASE + 0x07000)
#define DAC_BASE              (HBPERIPH_BASE + 0x07400)
#define CAN3_BASE             (HBPERIPH_BASE + 0x07800)
#define SWPMI_BASE            (HBPERIPH_BASE + 0x08400)

#define AFIO_BASE             (HBPERIPH_BASE + 0x10000)
#define EXTI_BASE             (HBPERIPH_BASE + 0x10400)
#define GPIOA_BASE            (HBPERIPH_BASE + 0x10800)
#define GPIOB_BASE            (HBPERIPH_BASE + 0x10C00)
#define GPIOC_BASE            (HBPERIPH_BASE + 0x11000)
#define GPIOD_BASE            (HBPERIPH_BASE + 0x11400)
#define GPIOE_BASE            (HBPERIPH_BASE + 0x11800)
#define GPIOF_BASE            (HBPERIPH_BASE + 0x11C00)
#define ADC1_BASE             (HBPERIPH_BASE + 0x12400)
#define ADC2_BASE             (HBPERIPH_BASE + 0x12800)
#define TIM1_BASE             (HBPERIPH_BASE + 0x12C00)
#define SPI1_BASE             (HBPERIPH_BASE + 0x13000)
#define TIM8_BASE             (HBPERIPH_BASE + 0x13400)
#define USART1_BASE           (HBPERIPH_BASE + 0x13800)
#define TIM12_BASE            (HBPERIPH_BASE + 0x13C00)
#define I2C4_BASE             (HBPERIPH_BASE + 0x14000)
#define I3C_BASE              (HBPERIPH_BASE + 0x14400)

#define LTDC_BASE             (HBPERIPH_BASE + 0x14800)
#define LTDC_L1_BASE          (HBPERIPH_BASE + 0x14834)
#define LTDC_L2_BASE          (HBPERIPH_BASE + 0x14864)

#define TIM9_BASE             (HBPERIPH_BASE + 0x14C00)
#define TIM10_BASE            (HBPERIPH_BASE + 0x15000)
#define TIM11_BASE            (HBPERIPH_BASE + 0x15400)

#define SAI_BASE              (HBPERIPH_BASE + 0x15800)
#define SAI_Block_A_BASE      (SAI_BASE + 0x04)
#define SAI_Block_B_BASE      (SAI_BASE + 0x24)

#define GPHA_BASE             (HBPERIPH_BASE + 0x16800)
#define ECDC_BASE             (HBPERIPH_BASE + 0x16C00)

#define DFSDM_BASE            (HBPERIPH_BASE + 0x17000)
#define DFSDM_Channel0_BASE   (HBPERIPH_BASE + 0x17000)
#define DFSDM_Channel1_BASE   (HBPERIPH_BASE + 0x17004)
#define DFSDM_FLT0_BASE       (HBPERIPH_BASE + 0x17028)
#define DFSDM_FLT1_BASE       (HBPERIPH_BASE + 0x1702C)

#define HSADC_BASE            (HBPERIPH_BASE + 0x17400)
#define OPA_BASE              (HBPERIPH_BASE + 0x17800)
#define SDIO_BASE             (HBPERIPH_BASE + 0x18000)

#define DMA1_BASE             (HBPERIPH_BASE + 0x20000)
#define DMA1_Channel1_BASE    (HBPERIPH_BASE + 0x20008)
#define DMA1_Channel2_BASE    (HBPERIPH_BASE + 0x2001C)
#define DMA1_Channel3_BASE    (HBPERIPH_BASE + 0x20030)
#define DMA1_Channel4_BASE    (HBPERIPH_BASE + 0x20044)
#define DMA1_Channel5_BASE    (HBPERIPH_BASE + 0x20058)
#define DMA1_Channel6_BASE    (HBPERIPH_BASE + 0x2006C)
#define DMA1_Channel7_BASE    (HBPERIPH_BASE + 0x20080)
#define DMA1_Channel8_BASE    (HBPERIPH_BASE + 0x20094)

#define DMA2_BASE             (HBPERIPH_BASE + 0x20400)
#define DMA2_Channel1_BASE    (HBPERIPH_BASE + 0x20408)
#define DMA2_Channel2_BASE    (HBPERIPH_BASE + 0x2041C)
#define DMA2_Channel3_BASE    (HBPERIPH_BASE + 0x20430)
#define DMA2_Channel4_BASE    (HBPERIPH_BASE + 0x20444)
#define DMA2_Channel5_BASE    (HBPERIPH_BASE + 0x20458)
#define DMA2_Channel6_BASE    (HBPERIPH_BASE + 0x2046C)
#define DMA2_Channel7_BASE    (HBPERIPH_BASE + 0x20480)
#define DMA2_Channel8_BASE    (HBPERIPH_BASE + 0x20494)

#define DMAMUX_BASE           (HBPERIPH_BASE + 0x20800)
#define RCC_BASE              (HBPERIPH_BASE + 0x21000)
#define FLASH_R_BASE          (HBPERIPH_BASE + 0x22000)
#define CRC_BASE              (HBPERIPH_BASE + 0x23000)
#define USBFS_BASE            (HBPERIPH_BASE + 0x23400)
#define RNG_BASE              (HBPERIPH_BASE + 0x23C00)
#define SDMMC_BASE            (HBPERIPH_BASE + 0x24000)
#define USBPD_BASE            (HBPERIPH_BASE + 0x24400)
#define QSPI1_BASE            (HBPERIPH_BASE + 0x24C00)
#define QSPI2_BASE            (HBPERIPH_BASE + 0x25000)
#define FMC_R_BASE            (HBPERIPH_BASE + 0x25400)
#define FMC_Bank1_R_BASE      (FMC_R_BASE + 0x0000)
#define FMC_Bank1E_R_BASE     (FMC_R_BASE + 0x0104)
#define FMC_Bank3_R_BASE      (FMC_R_BASE + 0x0080)
#define FMC_Bank5_6_R_BASE    (FMC_R_BASE + 0x0140)

#define DVP_BASE              (HBPERIPH_BASE + 0x25800)
#define PIOC_BASE             (HBPERIPH_BASE + 0x25C00)

#define SERDES_BASE           (HBPERIPH_BASE + 0x27C00)
#define SERDES1_BASE          (SERDES_BASE)
#define SERDES2_BASE          (SERDES_BASE+0x0040)

#define ETH_BASE              (HBPERIPH_BASE + 0x28000)
#define ETH_MAC_BASE          (ETH_BASE)
#define ETH_MMC_BASE          (ETH_BASE + 0x0100)
#define ETH_PTP_BASE          (ETH_BASE + 0x0700)
#define ETH_DMA_BASE          (ETH_BASE + 0x1000)

#define USBHS_BASE            (HBPERIPH_BASE + 0x30000)
#define USBHSD_BASE           (USBHS_BASE)
#define USBHSH_BASE           (USBHS_BASE + 0x100)

#define USBSS_BASE            (HBPERIPH_BASE + 0x34000)
#define UHSIF_BASE            (HBPERIPH_BASE + 0x38000)

#define OB_BASE               ((uint32_t)0x1FFFF800)

#define FLASH_CFGR0_BASE      ((uint32_t)0x4002202C)
#define SYS_CFGR0_BASE        ((uint32_t)0x5003C000)  

/* Peripheral declaration */
#define TIM2                  ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                  ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                  ((TIM_TypeDef *) TIM4_BASE)
#define TIM5                  ((TIM_TypeDef *) TIM5_BASE)
#define TIM6                  ((TIM_TypeDef *) TIM6_BASE)
#define TIM7                  ((TIM_TypeDef *) TIM7_BASE)
#define USART6                ((USART_TypeDef *) USART6_BASE)
#define USART7                ((USART_TypeDef *) USART7_BASE)
#define USART8                ((USART_TypeDef *) USART8_BASE)
#define LPTIM1                ((LPTIM_TypeDef *) LPTIM1_BASE)

#define RTC                   ((RTC_TypeDef *) RTC_BASE)
#define WWDG                  ((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                  ((IWDG_TypeDef *) IWDG_BASE)
#define LPTIM2                ((LPTIM_TypeDef *) LPTIM2_BASE)
#define SPI2                  ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                  ((SPI_TypeDef *) SPI3_BASE)
#define SPI4                  ((SPI_TypeDef *) SPI4_BASE)
#define USART2                ((USART_TypeDef *) USART2_BASE)
#define USART3                ((USART_TypeDef *) USART3_BASE)
#define USART4                ((USART_TypeDef *) USART4_BASE)
#define USART5                ((USART_TypeDef *) USART5_BASE)
#define I2C1                  ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                  ((I2C_TypeDef *) I2C2_BASE)
#define I2C3                  ((I2C_TypeDef *) I2C3_BASE)
#define CAN1                  ((CAN_TypeDef *) CAN1_BASE)
#define CAN2                  ((CAN_TypeDef *) CAN2_BASE)
#define PWR                   ((PWR_TypeDef *) PWR_BASE)
#define DAC                   ((DAC_TypeDef *) DAC_BASE)
#define CAN3                  ((CAN_TypeDef *) CAN3_BASE)
#define SWPMI                 ((SWPMI_TypeDef *) SWPMI_BASE)

#define AFIO                  ((AFIO_TypeDef *) AFIO_BASE)
#define EXTI                  ((EXTI_TypeDef *) EXTI_BASE)
#define GPIOA                 ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB                 ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC                 ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD                 ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE                 ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF                 ((GPIO_TypeDef *) GPIOF_BASE)
#define ADC1                  ((ADC_TypeDef *) ADC1_BASE)
#define ADC2                  ((ADC_TypeDef *) ADC2_BASE)
#define TKey1                 ((ADC_TypeDef *) ADC1_BASE)
#define TKey2                 ((ADC_TypeDef *) ADC2_BASE)
#define TIM1                  ((TIM_TypeDef *) TIM1_BASE)
#define SPI1                  ((SPI_TypeDef *) SPI1_BASE)
#define TIM8                  ((TIM_TypeDef *) TIM8_BASE)
#define USART1                ((USART_TypeDef *) USART1_BASE)
#define TIM12                 ((TIM_TypeDef *) TIM12_BASE)
#define I2C4                  ((I2C_TypeDef *) I2C4_BASE)
#define I3C                   ((I3C_TypeDef *) I3C_BASE)
#define LTDC                  ((LTDC_TypeDef *) LTDC_BASE)
#define LTDC_Layer1           ((LTDC_Layer_TypeDef *) LTDC_L1_BASE)
#define LTDC_Layer2           ((LTDC_Layer_TypeDef *) LTDC_L2_BASE)
#define TIM9                  ((TIM_TypeDef *) TIM9_BASE)
#define TIM10                 ((TIM_TypeDef *) TIM10_BASE)
#define TIM11                 ((TIM_TypeDef *) TIM11_BASE)
#define SAI_Block_A           ((SAI_Block_TypeDef *) SAI_Block_A_BASE)
#define SAI_Block_B           ((SAI_Block_TypeDef *) SAI_Block_B_BASE)
#define GPHA                  ((GPHA_TypeDef *) GPHA_BASE)
#define ECDC                  ((ECDC_TypeDef *) ECDC_BASE)
#define DFSDM_FLT0            ((DFSDM_FLT_TypeDef *) DFSDM_FLT0_BASE)
#define DFSDM_FLT1            ((DFSDM_FLT_TypeDef *) DFSDM_FLT1_BASE)
#define DFSDM_Channel0        ((DFSDM_Channel_TypeDef *) DFSDM_Channel0_BASE)
#define DFSDM_Channel1        ((DFSDM_Channel_TypeDef *) DFSDM_Channel1_BASE)
#define HSADC                 ((HSADC_TypeDef *) HSADC_BASE)
#define OPA                   ((OPA_TypeDef *) OPA_BASE)
#define SDIO                  ((SDIO_TypeDef *) SDIO_BASE)

#define DMA1                  ((DMA_TypeDef *) DMA1_BASE)
#define DMA2                  ((DMA_TypeDef *) DMA2_BASE)
#define DMA1_Channel1         ((DMA_Channel_TypeDef *) DMA1_Channel1_BASE)
#define DMA1_Channel2         ((DMA_Channel_TypeDef *) DMA1_Channel2_BASE)
#define DMA1_Channel3         ((DMA_Channel_TypeDef *) DMA1_Channel3_BASE)
#define DMA1_Channel4         ((DMA_Channel_TypeDef *) DMA1_Channel4_BASE)
#define DMA1_Channel5         ((DMA_Channel_TypeDef *) DMA1_Channel5_BASE)
#define DMA1_Channel6         ((DMA_Channel_TypeDef *) DMA1_Channel6_BASE)
#define DMA1_Channel7         ((DMA_Channel_TypeDef *) DMA1_Channel7_BASE)
#define DMA1_Channel8         ((DMA_Channel_TypeDef *) DMA1_Channel8_BASE)
#define DMA2_Channel1         ((DMA_Channel_TypeDef *) DMA2_Channel1_BASE)
#define DMA2_Channel2         ((DMA_Channel_TypeDef *) DMA2_Channel2_BASE)
#define DMA2_Channel3         ((DMA_Channel_TypeDef *) DMA2_Channel3_BASE)
#define DMA2_Channel4         ((DMA_Channel_TypeDef *) DMA2_Channel4_BASE)
#define DMA2_Channel5         ((DMA_Channel_TypeDef *) DMA2_Channel5_BASE)
#define DMA2_Channel6         ((DMA_Channel_TypeDef *) DMA2_Channel6_BASE)
#define DMA2_Channel7         ((DMA_Channel_TypeDef *) DMA2_Channel7_BASE)
#define DMA2_Channel8         ((DMA_Channel_TypeDef *) DMA2_Channel8_BASE)
#define DMAMUX                ((DMAMUX_TypeDef *) DMAMUX_BASE)

#define DMA1MUX_CHANNEL1_BASE            ((DMAMUX_BASE+0))
#define DMA1MUX_CHANNEL2_BASE            ((DMAMUX_BASE+1))
#define DMA1MUX_CHANNEL3_BASE            ((DMAMUX_BASE+2))
#define DMA1MUX_CHANNEL4_BASE            ((DMAMUX_BASE+3))
#define DMA1MUX_CHANNEL5_BASE            ((DMAMUX_BASE+4))
#define DMA1MUX_CHANNEL6_BASE            ((DMAMUX_BASE+5))
#define DMA1MUX_CHANNEL7_BASE            ((DMAMUX_BASE+6))
#define DMA1MUX_CHANNEL8_BASE            ((DMAMUX_BASE+7))
#define DMA2MUX_CHANNEL1_BASE            ((DMAMUX_BASE+8))
#define DMA2MUX_CHANNEL2_BASE            ((DMAMUX_BASE+9))
#define DMA2MUX_CHANNEL3_BASE            ((DMAMUX_BASE+10))
#define DMA2MUX_CHANNEL4_BASE            ((DMAMUX_BASE+11))
#define DMA2MUX_CHANNEL5_BASE            ((DMAMUX_BASE+12))
#define DMA2MUX_CHANNEL6_BASE            ((DMAMUX_BASE+13))
#define DMA2MUX_CHANNEL7_BASE            ((DMAMUX_BASE+14))
#define DMA2MUX_CHANNEL8_BASE            ((DMAMUX_BASE+15))


#define DMA1MUX_CHANNEL1            ((dmamux_channel_type *)DMA1MUX_CHANNEL1_BASE)
#define DMA1MUX_CHANNEL2            ((dmamux_channel_type *)DMA1MUX_CHANNEL2_BASE)
#define DMA1MUX_CHANNEL3            ((dmamux_channel_type *)DMA1MUX_CHANNEL3_BASE)
#define DMA1MUX_CHANNEL4            ((dmamux_channel_type *)DMA1MUX_CHANNEL4_BASE)
#define DMA1MUX_CHANNEL5            ((dmamux_channel_type *)DMA1MUX_CHANNEL5_BASE)
#define DMA1MUX_CHANNEL6            ((dmamux_channel_type *)DMA1MUX_CHANNEL6_BASE)
#define DMA1MUX_CHANNEL7            ((dmamux_channel_type *)DMA1MUX_CHANNEL7_BASE)
#define DMA1MUX_CHANNEL8            ((dmamux_channel_type *)DMA1MUX_CHANNEL8_BASE)

#define DMA2MUX_CHANNEL1            ((dmamux_channel_type *)DMA2MUX_CHANNEL1_BASE)
#define DMA2MUX_CHANNEL2            ((dmamux_channel_type *)DMA2MUX_CHANNEL2_BASE)
#define DMA2MUX_CHANNEL3            ((dmamux_channel_type *)DMA2MUX_CHANNEL3_BASE)
#define DMA2MUX_CHANNEL4            ((dmamux_channel_type *)DMA2MUX_CHANNEL4_BASE)
#define DMA2MUX_CHANNEL5            ((dmamux_channel_type *)DMA2MUX_CHANNEL5_BASE)
#define DMA2MUX_CHANNEL6            ((dmamux_channel_type *)DMA2MUX_CHANNEL6_BASE)
#define DMA2MUX_CHANNEL7            ((dmamux_channel_type *)DMA2MUX_CHANNEL7_BASE)
#define DMA2MUX_CHANNEL8            ((dmamux_channel_type *)DMA2MUX_CHANNEL8_BASE)




#define RCC                   ((RCC_TypeDef *) RCC_BASE)
#define FLASH                 ((FLASH_TypeDef *) FLASH_R_BASE)
#define CRC                   ((CRC_TypeDef *) CRC_BASE)

#define USBFSD                ((USBFSD_TypeDef *)USBFS_BASE)
#define USBFSH                ((USBFSH_TypeDef *)USBFS_BASE)
#define RNG                   ((RNG_TypeDef *) RNG_BASE)
#define SDMMC                 ((SDMMC_TypeDef *) SDMMC_BASE)
#define USBPD                 ((USBPD_TypeDef *) USBPD_BASE)
#define QSPI1                 ((QSPI_TypeDef *) QSPI1_BASE)
#define QSPI2                 ((QSPI_TypeDef *) QSPI2_BASE)
#define FMC_Bank1             ((FMC_Bank1_TypeDef *) FMC_Bank1_R_BASE)
#define FMC_Bank1E            ((FMC_Bank1E_TypeDef *) FMC_Bank1E_R_BASE)
#define FMC_Bank3             ((FMC_Bank3_TypeDef *) FMC_Bank3_R_BASE)
#define FMC_Bank5_6           ((FMC_Bank5_6_TypeDef *) FMC_Bank5_6_R_BASE)
#define DVP                   ((DVP_TypeDef *) DVP_BASE)
#define ETH                   ((ETH_TypeDef *) ETH_BASE)
#define USBHSD                ((USBHSD_TypeDef *) USBHSD_BASE)
#define USBHSH                ((USBHSH_TypeDef *) USBHSH_BASE)
#define USBSSD                ((USBSSD_TypeDef *) USBSS_BASE)
#define USBSSH                ((USBSSH_TypeDef *) USBSS_BASE)
#define SDS1                  ((SDS_TypeDef *) SERDES1_BASE)
#define SDS2                  ((SDS_TypeDef *) SERDES2_BASE)
#define UHSIF                 ((UHSIF_TypeDef *) UHSIF_BASE)
#define OB                    ((OB_TypeDef *) OB_BASE)

/******************************************************************************/
/*                         Peripheral Registers Bits Definition               */
/******************************************************************************/

/******************************************************************************/
/*                        Analog to Digital Converter                         */
/******************************************************************************/

/********************  Bit definition for ADC_STATR register  ********************/
#define  ADC_AWD                                     ((uint8_t)0x01)               /* Analog watchdog flag */
#define  ADC_EOC                                     ((uint8_t)0x02)               /* End of conversion */
#define  ADC_JEOC                                    ((uint8_t)0x04)               /* Injected channel end of conversion */
#define  ADC_JSTRT                                   ((uint8_t)0x08)               /* Injected channel Start flag */
#define  ADC_STRT                                    ((uint8_t)0x10)               /* Regular channel Start flag */

/*******************  Bit definition for ADC_CTLR1 register  ********************/
#define  ADC_AWDCH                                   ((uint32_t)0x0000001F)        /* AWDCH[4:0] bits (Analog watchdog channel select bits) */
#define  ADC_AWDCH_0                                 ((uint32_t)0x00000001)        /* Bit 0 */
#define  ADC_AWDCH_1                                 ((uint32_t)0x00000002)        /* Bit 1 */
#define  ADC_AWDCH_2                                 ((uint32_t)0x00000004)        /* Bit 2 */
#define  ADC_AWDCH_3                                 ((uint32_t)0x00000008)        /* Bit 3 */
#define  ADC_AWDCH_4                                 ((uint32_t)0x00000010)        /* Bit 4 */

#define  ADC_EOCIE                                   ((uint32_t)0x00000020)        /* Interrupt enable for EOC */
#define  ADC_AWDIE                                   ((uint32_t)0x00000040)        /* Analog Watchdog interrupt enable */
#define  ADC_JEOCIE                                  ((uint32_t)0x00000080)        /* Interrupt enable for injected channels */
#define  ADC_SCAN                                    ((uint32_t)0x00000100)        /* Scan mode */
#define  ADC_AWDSGL                                  ((uint32_t)0x00000200)        /* Enable the watchdog on a single channel in scan mode */
#define  ADC_JAUTO                                   ((uint32_t)0x00000400)        /* Automatic injected group conversion */
#define  ADC_DISCEN                                  ((uint32_t)0x00000800)        /* Discontinuous mode on regular channels */
#define  ADC_JDISCEN                                 ((uint32_t)0x00001000)        /* Discontinuous mode on injected channels */

#define  ADC_DISCNUM                                 ((uint32_t)0x0000E000)        /* DISCNUM[2:0] bits (Discontinuous mode channel count) */
#define  ADC_DISCNUM_0                               ((uint32_t)0x00002000)        /* Bit 0 */
#define  ADC_DISCNUM_1                               ((uint32_t)0x00004000)        /* Bit 1 */
#define  ADC_DISCNUM_2                               ((uint32_t)0x00008000)        /* Bit 2 */

#define  ADC_DUALMOD                                 ((uint32_t)0x000F0000)        /* DUALMOD[3:0] bits (Dual mode selection) */
#define  ADC_DUALMOD_0                               ((uint32_t)0x00010000)        /* Bit 0 */
#define  ADC_DUALMOD_1                               ((uint32_t)0x00020000)        /* Bit 1 */
#define  ADC_DUALMOD_2                               ((uint32_t)0x00040000)        /* Bit 2 */
#define  ADC_DUALMOD_3                               ((uint32_t)0x00080000)        /* Bit 3 */

#define  ADC_JAWDEN                                  ((uint32_t)0x00400000)        /* Analog watchdog enable on injected channels */
#define  ADC_AWDEN                                   ((uint32_t)0x00800000)        /* Analog watchdog enable on regular channels */
             
#define  ADC_TKENABLE                                ((uint32_t)0x01000000)        /*TKEY enable*/
#define  ADC_TKITUNE                                 ((uint32_t)0x02000000)        
#define  ADC_BUFEN                                   ((uint32_t)0x04000000)        

#define  ADC_PGA                                     ((uint32_t)0x18000000)        
#define  ADC_PGA_0                                   ((uint32_t)0x08000000)        
#define  ADC_PGA_1                                   ((uint32_t)0x10000000)

#define  ADC_ANA_RST                                 ((uint32_t)0x40000000)
#define  ADC_SW_PRE                                  ((uint32_t)0x80000000)

/*******************  Bit definition for ADC_CTLR2 register  ********************/
#define  ADC_ADON                                    ((uint32_t)0x00000001)        /* A/D Converter ON / OFF */
#define  ADC_CONT                                    ((uint32_t)0x00000002)        /* Continuous Conversion */
#define  ADC_CAL                                     ((uint32_t)0x00000004)        /* A/D Calibration */
#define  ADC_RSTCAL                                  ((uint32_t)0x00000008)        /* Reset Calibration */
#define  ADC_DMA                                     ((uint32_t)0x00000100)        /* Direct Memory access mode */
#define  ADC_ALIGN                                   ((uint32_t)0x00000800)        /* Data Alignment */

#define  ADC_JEXTSEL                                 ((uint32_t)0x00007000)        /* JEXTSEL[2:0] bits (External event select for injected group) */
#define  ADC_JEXTSEL_0                               ((uint32_t)0x00001000)        /* Bit 0 */
#define  ADC_JEXTSEL_1                               ((uint32_t)0x00002000)        /* Bit 1 */
#define  ADC_JEXTSEL_2                               ((uint32_t)0x00004000)        /* Bit 2 */

#define  ADC_JEXTTRIG                                ((uint32_t)0x00008000)        /* External Trigger Conversion mode for injected channels */

#define  ADC_EXTSEL                                  ((uint32_t)0x000E0000)        /* EXTSEL[2:0] bits (External Event Select for regular group) */
#define  ADC_EXTSEL_0                                ((uint32_t)0x00020000)        /* Bit 0 */
#define  ADC_EXTSEL_1                                ((uint32_t)0x00040000)        /* Bit 1 */
#define  ADC_EXTSEL_2                                ((uint32_t)0x00080000)        /* Bit 2 */

#define  ADC_EXTTRIG                                 ((uint32_t)0x00100000)        /* External Trigger Conversion mode for regular channels */
#define  ADC_JSWSTART                                ((uint32_t)0x00200000)        /* Start Conversion of injected channels */
#define  ADC_SWSTART                                 ((uint32_t)0x00400000)        /* Start Conversion of regular channels */
#define  ADC_TSVREFE                                 ((uint32_t)0x00800000)        /* Temperature Sensor and VREFINT Enable */

/******************  Bit definition for ADC_SAMPTR1 register  *******************/
#define  ADC_SMP10                                   ((uint32_t)0x00000007)        /* SMP10[2:0] bits (Channel 10 Sample time selection) */
#define  ADC_SMP10_0                                 ((uint32_t)0x00000001)        /* Bit 0 */
#define  ADC_SMP10_1                                 ((uint32_t)0x00000002)        /* Bit 1 */
#define  ADC_SMP10_2                                 ((uint32_t)0x00000004)        /* Bit 2 */

#define  ADC_SMP11                                   ((uint32_t)0x00000038)        /* SMP11[2:0] bits (Channel 11 Sample time selection) */
#define  ADC_SMP11_0                                 ((uint32_t)0x00000008)        /* Bit 0 */
#define  ADC_SMP11_1                                 ((uint32_t)0x00000010)        /* Bit 1 */
#define  ADC_SMP11_2                                 ((uint32_t)0x00000020)        /* Bit 2 */

#define  ADC_SMP12                                   ((uint32_t)0x000001C0)        /* SMP12[2:0] bits (Channel 12 Sample time selection) */
#define  ADC_SMP12_0                                 ((uint32_t)0x00000040)        /* Bit 0 */
#define  ADC_SMP12_1                                 ((uint32_t)0x00000080)        /* Bit 1 */
#define  ADC_SMP12_2                                 ((uint32_t)0x00000100)        /* Bit 2 */

#define  ADC_SMP13                                   ((uint32_t)0x00000E00)        /* SMP13[2:0] bits (Channel 13 Sample time selection) */
#define  ADC_SMP13_0                                 ((uint32_t)0x00000200)        /* Bit 0 */
#define  ADC_SMP13_1                                 ((uint32_t)0x00000400)        /* Bit 1 */
#define  ADC_SMP13_2                                 ((uint32_t)0x00000800)        /* Bit 2 */

#define  ADC_SMP14                                   ((uint32_t)0x00007000)        /* SMP14[2:0] bits (Channel 14 Sample time selection) */
#define  ADC_SMP14_0                                 ((uint32_t)0x00001000)        /* Bit 0 */
#define  ADC_SMP14_1                                 ((uint32_t)0x00002000)        /* Bit 1 */
#define  ADC_SMP14_2                                 ((uint32_t)0x00004000)        /* Bit 2 */

#define  ADC_SMP15                                   ((uint32_t)0x00038000)        /* SMP15[2:0] bits (Channel 15 Sample time selection) */
#define  ADC_SMP15_0                                 ((uint32_t)0x00008000)        /* Bit 0 */
#define  ADC_SMP15_1                                 ((uint32_t)0x00010000)        /* Bit 1 */
#define  ADC_SMP15_2                                 ((uint32_t)0x00020000)        /* Bit 2 */
             
#define  ADC_SMP16                                   ((uint32_t)0x001C0000)        /* SMP16[2:0] bits (Channel 16 Sample time selection) */
#define  ADC_SMP16_0                                 ((uint32_t)0x00040000)        /* Bit 0 */
#define  ADC_SMP16_1                                 ((uint32_t)0x00080000)        /* Bit 1 */
#define  ADC_SMP16_2                                 ((uint32_t)0x00100000)        /* Bit 2 */

#define  ADC_SMP17                                   ((uint32_t)0x00E00000)        /* SMP17[2:0] bits (Channel 17 Sample time selection) */
#define  ADC_SMP17_0                                 ((uint32_t)0x00200000)        /* Bit 0 */
#define  ADC_SMP17_1                                 ((uint32_t)0x00400000)        /* Bit 1 */
#define  ADC_SMP17_2                                 ((uint32_t)0x00800000)        /* Bit 2 */

/******************  Bit definition for ADC_SAMPTR2 register  *******************/
#define  ADC_SMP0                                    ((uint32_t)0x00000007)        /* SMP0[2:0] bits (Channel 0 Sample time selection) */
#define  ADC_SMP0_0                                  ((uint32_t)0x00000001)        /* Bit 0 */
#define  ADC_SMP0_1                                  ((uint32_t)0x00000002)        /* Bit 1 */
#define  ADC_SMP0_2                                  ((uint32_t)0x00000004)        /* Bit 2 */

#define  ADC_SMP1                                    ((uint32_t)0x00000038)        /* SMP1[2:0] bits (Channel 1 Sample time selection) */
#define  ADC_SMP1_0                                  ((uint32_t)0x00000008)        /* Bit 0 */
#define  ADC_SMP1_1                                  ((uint32_t)0x00000010)        /* Bit 1 */
#define  ADC_SMP1_2                                  ((uint32_t)0x00000020)        /* Bit 2 */

#define  ADC_SMP2                                    ((uint32_t)0x000001C0)        /* SMP2[2:0] bits (Channel 2 Sample time selection) */
#define  ADC_SMP2_0                                  ((uint32_t)0x00000040)        /* Bit 0 */
#define  ADC_SMP2_1                                  ((uint32_t)0x00000080)        /* Bit 1 */
#define  ADC_SMP2_2                                  ((uint32_t)0x00000100)        /* Bit 2 */

#define  ADC_SMP3                                    ((uint32_t)0x00000E00)        /* SMP3[2:0] bits (Channel 3 Sample time selection) */
#define  ADC_SMP3_0                                  ((uint32_t)0x00000200)        /* Bit 0 */
#define  ADC_SMP3_1                                  ((uint32_t)0x00000400)        /* Bit 1 */
#define  ADC_SMP3_2                                  ((uint32_t)0x00000800)        /* Bit 2 */

#define  ADC_SMP4                                    ((uint32_t)0x00007000)        /* SMP4[2:0] bits (Channel 4 Sample time selection) */
#define  ADC_SMP4_0                                  ((uint32_t)0x00001000)        /* Bit 0 */
#define  ADC_SMP4_1                                  ((uint32_t)0x00002000)        /* Bit 1 */
#define  ADC_SMP4_2                                  ((uint32_t)0x00004000)        /* Bit 2 */

#define  ADC_SMP5                                    ((uint32_t)0x00038000)        /* SMP5[2:0] bits (Channel 5 Sample time selection) */
#define  ADC_SMP5_0                                  ((uint32_t)0x00008000)        /* Bit 0 */
#define  ADC_SMP5_1                                  ((uint32_t)0x00010000)        /* Bit 1 */
#define  ADC_SMP5_2                                  ((uint32_t)0x00020000)        /* Bit 2 */

#define  ADC_SMP6                                    ((uint32_t)0x001C0000)        /* SMP6[2:0] bits (Channel 6 Sample time selection) */
#define  ADC_SMP6_0                                  ((uint32_t)0x00040000)        /* Bit 0 */
#define  ADC_SMP6_1                                 ((uint32_t)0x00080000)        /* Bit 1 */
#define  ADC_SMP6_2                                  ((uint32_t)0x00100000)        /* Bit 2 */

#define  ADC_SMP7                                    ((uint32_t)0x00E00000)        /* SMP7[2:0] bits (Channel 7 Sample time selection) */
#define  ADC_SMP7_0                                  ((uint32_t)0x00200000)        /* Bit 0 */
#define  ADC_SMP7_1                                  ((uint32_t)0x00400000)        /* Bit 1 */
#define  ADC_SMP7_2                                  ((uint32_t)0x00800000)        /* Bit 2 */

#define  ADC_SMP8                                    ((uint32_t)0x07000000)        /* SMP8[2:0] bits (Channel 8 Sample time selection) */
#define  ADC_SMP8_0                                  ((uint32_t)0x01000000)        /* Bit 0 */
#define  ADC_SMP8_1                                  ((uint32_t)0x02000000)        /* Bit 1 */
#define  ADC_SMP8_2                                  ((uint32_t)0x04000000)        /* Bit 2 */

#define  ADC_SMP9                                    ((uint32_t)0x38000000)        /* SMP9[2:0] bits (Channel 9 Sample time selection) */
#define  ADC_SMP9_0                                  ((uint32_t)0x08000000)        /* Bit 0 */
#define  ADC_SMP9_1                                  ((uint32_t)0x10000000)        /* Bit 1 */
#define  ADC_SMP9_2                                  ((uint32_t)0x20000000)        /* Bit 2 */

/******************  Bit definition for ADC_IOFR1 register  *******************/
#define  ADC_JOFFSET1                                ((uint16_t)0x0FFF)            /* Data offset for injected channel 1 */

/******************  Bit definition for ADC_IOFR2 register  *******************/
#define  ADC_JOFFSET2                                ((uint16_t)0x0FFF)            /* Data offset for injected channel 2 */

/******************  Bit definition for ADC_IOFR3 register  *******************/
#define  ADC_JOFFSET3                                ((uint16_t)0x0FFF)            /* Data offset for injected channel 3 */

/******************  Bit definition for ADC_IOFR4 register  *******************/
#define  ADC_JOFFSET4                                ((uint16_t)0x0FFF)            /* Data offset for injected channel 4 */

/*******************  Bit definition for ADC_WDHTR register  ********************/
#define  ADC_HT                                     ((uint16_t)0x0FFF)            /* Analog watchdog high threshold */

/*******************  Bit definition for ADC_WDLTR register  ********************/
#define  ADC_LT                                      ((uint16_t)0x0FFF)            /* Analog watchdog low threshold */

/*******************  Bit definition for ADC_RSQR1 register  *******************/
#define  ADC_SQ13                                    ((uint32_t)0x0000001F)        /* SQ13[4:0] bits (13th conversion in regular sequence) */
#define  ADC_SQ13_0                                  ((uint32_t)0x00000001)        /* Bit 0 */
#define  ADC_SQ13_1                                  ((uint32_t)0x00000002)        /* Bit 1 */
#define  ADC_SQ13_2                                  ((uint32_t)0x00000004)        /* Bit 2 */
#define  ADC_SQ13_3                                  ((uint32_t)0x00000008)        /* Bit 3 */
#define  ADC_SQ13_4                                  ((uint32_t)0x00000010)        /* Bit 4 */

#define  ADC_SQ14                                    ((uint32_t)0x000003E0)        /* SQ14[4:0] bits (14th conversion in regular sequence) */
#define  ADC_SQ14_0                                  ((uint32_t)0x00000020)        /* Bit 0 */
#define  ADC_SQ14_1                                  ((uint32_t)0x00000040)        /* Bit 1 */
#define  ADC_SQ14_2                                  ((uint32_t)0x00000080)        /* Bit 2 */
#define  ADC_SQ14_3                                  ((uint32_t)0x00000100)        /* Bit 3 */
#define  ADC_SQ14_4                                  ((uint32_t)0x00000200)        /* Bit 4 */

#define  ADC_SQ15                                    ((uint32_t)0x00007C00)        /* SQ15[4:0] bits (15th conversion in regular sequence) */
#define  ADC_SQ15_0                                  ((uint32_t)0x00000400)        /* Bit 0 */
#define  ADC_SQ15_1                                  ((uint32_t)0x00000800)        /* Bit 1 */
#define  ADC_SQ15_2                                  ((uint32_t)0x00001000)        /* Bit 2 */
#define  ADC_SQ15_3                                  ((uint32_t)0x00002000)        /* Bit 3 */
#define  ADC_SQ15_4                                  ((uint32_t)0x00004000)        /* Bit 4 */

#define  ADC_SQ16                                    ((uint32_t)0x000F8000)        /* SQ16[4:0] bits (16th conversion in regular sequence) */
#define  ADC_SQ16_0                                  ((uint32_t)0x00008000)        /* Bit 0 */
#define  ADC_SQ16_1                                  ((uint32_t)0x00010000)        /* Bit 1 */
#define  ADC_SQ16_2                                  ((uint32_t)0x00020000)        /* Bit 2 */
#define  ADC_SQ16_3                                  ((uint32_t)0x00040000)        /* Bit 3 */
#define  ADC_SQ16_4                                  ((uint32_t)0x00080000)        /* Bit 4 */

#define  ADC_L                                       ((uint32_t)0x00F00000)        /* L[3:0] bits (Regular channel sequence length) */
#define  ADC_L_0                                     ((uint32_t)0x00100000)        /* Bit 0 */
#define  ADC_L_1                                     ((uint32_t)0x00200000)        /* Bit 1 */
#define  ADC_L_2                                     ((uint32_t)0x00400000)        /* Bit 2 */
#define  ADC_L_3                                     ((uint32_t)0x00800000)        /* Bit 3 */

/*******************  Bit definition for ADC_RSQR2 register  *******************/
#define  ADC_SQ7                                     ((uint32_t)0x0000001F)        /* SQ7[4:0] bits (7th conversion in regular sequence) */
#define  ADC_SQ7_0                                   ((uint32_t)0x00000001)        /* Bit 0 */
#define  ADC_SQ7_1                                   ((uint32_t)0x00000002)        /* Bit 1 */
#define  ADC_SQ7_2                                   ((uint32_t)0x00000004)        /* Bit 2 */
#define  ADC_SQ7_3                                   ((uint32_t)0x00000008)        /* Bit 3 */
#define  ADC_SQ7_4                                   ((uint32_t)0x00000010)        /* Bit 4 */

#define  ADC_SQ8                                     ((uint32_t)0x000003E0)        /* SQ8[4:0] bits (8th conversion in regular sequence) */
#define  ADC_SQ8_0                                   ((uint32_t)0x00000020)        /* Bit 0 */
#define  ADC_SQ8_1                                   ((uint32_t)0x00000040)        /* Bit 1 */
#define  ADC_SQ8_2                                   ((uint32_t)0x00000080)        /* Bit 2 */
#define  ADC_SQ8_3                                   ((uint32_t)0x00000100)        /* Bit 3 */
#define  ADC_SQ8_4                                   ((uint32_t)0x00000200)        /* Bit 4 */

#define  ADC_SQ9                                     ((uint32_t)0x00007C00)        /* SQ9[4:0] bits (9th conversion in regular sequence) */
#define  ADC_SQ9_0                                   ((uint32_t)0x00000400)        /* Bit 0 */
#define  ADC_SQ9_1                                   ((uint32_t)0x00000800)        /* Bit 1 */
#define  ADC_SQ9_2                                   ((uint32_t)0x00001000)        /* Bit 2 */
#define  ADC_SQ9_3                                   ((uint32_t)0x00002000)        /* Bit 3 */
#define  ADC_SQ9_4                                   ((uint32_t)0x00004000)        /* Bit 4 */

#define  ADC_SQ10                                    ((uint32_t)0x000F8000)        /* SQ10[4:0] bits (10th conversion in regular sequence) */
#define  ADC_SQ10_0                                  ((uint32_t)0x00008000)        /* Bit 0 */
#define  ADC_SQ10_1                                  ((uint32_t)0x00010000)        /* Bit 1 */
#define  ADC_SQ10_2                                  ((uint32_t)0x00020000)        /* Bit 2 */
#define  ADC_SQ10_3                                  ((uint32_t)0x00040000)        /* Bit 3 */
#define  ADC_SQ10_4                                  ((uint32_t)0x00080000)        /* Bit 4 */

#define  ADC_SQ11                                    ((uint32_t)0x01F00000)        /* SQ11[4:0] bits (11th conversion in regular sequence) */
#define  ADC_SQ11_0                                  ((uint32_t)0x00100000)        /* Bit 0 */
#define  ADC_SQ11_1                                  ((uint32_t)0x00200000)        /* Bit 1 */
#define  ADC_SQ11_2                                  ((uint32_t)0x00400000)        /* Bit 2 */
#define  ADC_SQ11_3                                  ((uint32_t)0x00800000)        /* Bit 3 */
#define  ADC_SQ11_4                                  ((uint32_t)0x01000000)        /* Bit 4 */

#define  ADC_SQ12                                    ((uint32_t)0x3E000000)        /* SQ12[4:0] bits (12th conversion in regular sequence) */
#define  ADC_SQ12_0                                  ((uint32_t)0x02000000)        /* Bit 0 */
#define  ADC_SQ12_1                                  ((uint32_t)0x04000000)        /* Bit 1 */
#define  ADC_SQ12_2                                  ((uint32_t)0x08000000)        /* Bit 2 */
#define  ADC_SQ12_3                                  ((uint32_t)0x10000000)        /* Bit 3 */
#define  ADC_SQ12_4                                  ((uint32_t)0x20000000)        /* Bit 4 */

/*******************  Bit definition for ADC_RSQR3 register  *******************/
#define  ADC_SQ1                                     ((uint32_t)0x0000001F)        /* SQ1[4:0] bits (1st conversion in regular sequence) */
#define  ADC_SQ1_0                                   ((uint32_t)0x00000001)        /* Bit 0 */
#define  ADC_SQ1_1                                   ((uint32_t)0x00000002)        /* Bit 1 */
#define  ADC_SQ1_2                                   ((uint32_t)0x00000004)        /* Bit 2 */
#define  ADC_SQ1_3                                   ((uint32_t)0x00000008)        /* Bit 3 */
#define  ADC_SQ1_4                                   ((uint32_t)0x00000010)        /* Bit 4 */

#define  ADC_SQ2                                     ((uint32_t)0x000003E0)        /* SQ2[4:0] bits (2nd conversion in regular sequence) */
#define  ADC_SQ2_0                                   ((uint32_t)0x00000020)        /* Bit 0 */
#define  ADC_SQ2_1                                   ((uint32_t)0x00000040)        /* Bit 1 */
#define  ADC_SQ2_2                                   ((uint32_t)0x00000080)        /* Bit 2 */
#define  ADC_SQ2_3                                   ((uint32_t)0x00000100)        /* Bit 3 */
#define  ADC_SQ2_4                                   ((uint32_t)0x00000200)        /* Bit 4 */

#define  ADC_SQ3                                     ((uint32_t)0x00007C00)        /* SQ3[4:0] bits (3rd conversion in regular sequence) */
#define  ADC_SQ3_0                                   ((uint32_t)0x00000400)        /* Bit 0 */
#define  ADC_SQ3_1                                   ((uint32_t)0x00000800)        /* Bit 1 */
#define  ADC_SQ3_2                                   ((uint32_t)0x00001000)        /* Bit 2 */
#define  ADC_SQ3_3                                   ((uint32_t)0x00002000)        /* Bit 3 */
#define  ADC_SQ3_4                                   ((uint32_t)0x00004000)        /* Bit 4 */

#define  ADC_SQ4                                     ((uint32_t)0x000F8000)        /* SQ4[4:0] bits (4th conversion in regular sequence) */
#define  ADC_SQ4_0                                   ((uint32_t)0x00008000)        /* Bit 0 */
#define  ADC_SQ4_1                                   ((uint32_t)0x00010000)        /* Bit 1 */
#define  ADC_SQ4_2                                   ((uint32_t)0x00020000)        /* Bit 2 */
#define  ADC_SQ4_3                                   ((uint32_t)0x00040000)        /* Bit 3 */
#define  ADC_SQ4_4                                   ((uint32_t)0x00080000)        /* Bit 4 */

#define  ADC_SQ5                                     ((uint32_t)0x01F00000)        /* SQ5[4:0] bits (5th conversion in regular sequence) */
#define  ADC_SQ5_0                                   ((uint32_t)0x00100000)        /* Bit 0 */
#define  ADC_SQ5_1                                   ((uint32_t)0x00200000)        /* Bit 1 */
#define  ADC_SQ5_2                                   ((uint32_t)0x00400000)        /* Bit 2 */
#define  ADC_SQ5_3                                   ((uint32_t)0x00800000)        /* Bit 3 */
#define  ADC_SQ5_4                                   ((uint32_t)0x01000000)        /* Bit 4 */

#define  ADC_SQ6                                     ((uint32_t)0x3E000000)        /* SQ6[4:0] bits (6th conversion in regular sequence) */
#define  ADC_SQ6_0                                   ((uint32_t)0x02000000)        /* Bit 0 */
#define  ADC_SQ6_1                                   ((uint32_t)0x04000000)        /* Bit 1 */
#define  ADC_SQ6_2                                   ((uint32_t)0x08000000)        /* Bit 2 */
#define  ADC_SQ6_3                                   ((uint32_t)0x10000000)        /* Bit 3 */
#define  ADC_SQ6_4                                   ((uint32_t)0x20000000)        /* Bit 4 */

/*******************  Bit definition for ADC_ISQR register  *******************/
#define  ADC_JSQ1                                    ((uint32_t)0x0000001F)        /* JSQ1[4:0] bits (1st conversion in injected sequence) */  
#define  ADC_JSQ1_0                                  ((uint32_t)0x00000001)        /* Bit 0 */
#define  ADC_JSQ1_1                                  ((uint32_t)0x00000002)        /* Bit 1 */
#define  ADC_JSQ1_2                                  ((uint32_t)0x00000004)        /* Bit 2 */
#define  ADC_JSQ1_3                                  ((uint32_t)0x00000008)        /* Bit 3 */
#define  ADC_JSQ1_4                                  ((uint32_t)0x00000010)        /* Bit 4 */

#define  ADC_JSQ2                                    ((uint32_t)0x000003E0)        /* JSQ2[4:0] bits (2nd conversion in injected sequence) */
#define  ADC_JSQ2_0                                  ((uint32_t)0x00000020)        /* Bit 0 */
#define  ADC_JSQ2_1                                  ((uint32_t)0x00000040)        /* Bit 1 */
#define  ADC_JSQ2_2                                  ((uint32_t)0x00000080)        /* Bit 2 */
#define  ADC_JSQ2_3                                  ((uint32_t)0x00000100)        /* Bit 3 */
#define  ADC_JSQ2_4                                  ((uint32_t)0x00000200)        /* Bit 4 */

#define  ADC_JSQ3                                    ((uint32_t)0x00007C00)        /* JSQ3[4:0] bits (3rd conversion in injected sequence) */
#define  ADC_JSQ3_0                                  ((uint32_t)0x00000400)        /* Bit 0 */
#define  ADC_JSQ3_1                                  ((uint32_t)0x00000800)        /* Bit 1 */
#define  ADC_JSQ3_2                                  ((uint32_t)0x00001000)        /* Bit 2 */
#define  ADC_JSQ3_3                                  ((uint32_t)0x00002000)        /* Bit 3 */
#define  ADC_JSQ3_4                                  ((uint32_t)0x00004000)        /* Bit 4 */

#define  ADC_JSQ4                                    ((uint32_t)0x000F8000)        /* JSQ4[4:0] bits (4th conversion in injected sequence) */
#define  ADC_JSQ4_0                                  ((uint32_t)0x00008000)        /* Bit 0 */
#define  ADC_JSQ4_1                                  ((uint32_t)0x00010000)        /* Bit 1 */
#define  ADC_JSQ4_2                                  ((uint32_t)0x00020000)        /* Bit 2 */
#define  ADC_JSQ4_3                                  ((uint32_t)0x00040000)        /* Bit 3 */
#define  ADC_JSQ4_4                                  ((uint32_t)0x00080000)        /* Bit 4 */

#define  ADC_JL                                      ((uint32_t)0x00300000)        /* JL[1:0] bits (Injected Sequence length) */
#define  ADC_JL_0                                    ((uint32_t)0x00100000)        /* Bit 0 */
#define  ADC_JL_1                                    ((uint32_t)0x00200000)        /* Bit 1 */

/*******************  Bit definition for ADC_IDATAR1 register  *******************/
#define  ADC_IDATAR1_JDATA                           ((uint16_t)0xFFFF)            /* Injected data */

/*******************  Bit definition for ADC_IDATAR2 register  *******************/
#define  ADC_IDATAR2_JDATA                           ((uint16_t)0xFFFF)            /* Injected data */

/*******************  Bit definition for ADC_IDATAR3 register  *******************/
#define  ADC_IDATAR3_JDATA                           ((uint16_t)0xFFFF)            /* Injected data */

/*******************  Bit definition for ADC_IDATAR4 register  *******************/
#define  ADC_IDATAR4_JDATA                           ((uint16_t)0xFFFF)            /* Injected data */

/********************  Bit definition for ADC_RDATAR register  ********************/
#define  ADC_RDATAR_DATA                             ((uint32_t)0x0000FFFF)        /* Regular data */
#define  ADC_RDATAR_ADC2DATA                         ((uint32_t)0xFFFF0000)        /* ADC2 data */

/********************  Bit definition for ADC_AUX register  ********************/
#define  ADC_SMP_SEL_0                               ((uint32_t)0x00000001)        /* channel_0 */
#define  ADC_SMP_SEL_1                               ((uint32_t)0x00000002)        /* channel_1 */
#define  ADC_SMP_SEL_2                               ((uint32_t)0x00000004)        /* channel_2 */
#define  ADC_SMP_SEL_3                               ((uint32_t)0x00000008)        /* channel_3 */
#define  ADC_SMP_SEL_4                               ((uint32_t)0x00000010)        /* channel_4 */
#define  ADC_SMP_SEL_5                               ((uint32_t)0x00000020)        /* channel_5 */
#define  ADC_SMP_SEL_6                               ((uint32_t)0x00000040)        /* channel_6 */
#define  ADC_SMP_SEL_7                               ((uint32_t)0x00000080)        /* channel_7 */
#define  ADC_SMP_SEL_8                               ((uint32_t)0x00000100)        /* channel_8 */
#define  ADC_SMP_SEL_9                               ((uint32_t)0x00000200)        /* channel_9 */
#define  ADC_SMP_SEL_10                              ((uint32_t)0x00000400)        /* channel_10 */
#define  ADC_SMP_SEL_11                              ((uint32_t)0x00000800)        /* channel_11 */
#define  ADC_SMP_SEL_12                              ((uint32_t)0x00001000)        /* channel_12 */
#define  ADC_SMP_SEL_13                              ((uint32_t)0x00002000)        /* channel_13 */
#define  ADC_SMP_SEL_14                              ((uint32_t)0x00004000)        /* channel_14 */
#define  ADC_SMP_SEL_15                              ((uint32_t)0x00008000)        /* channel_15 */
#define  ADC_SMP_SEL_16                              ((uint32_t)0x00010000)        /* channel_16 */
#define  ADC_SMP_SEL_17                              ((uint32_t)0x00020000)        /* channel_17 */

#define  ADC_TO_DFSDM                                ((uint32_t)0x80000000)

/********************  Bit definition for ADC_DRV register  ********************/
#define  ADC_DRV_TKEY_OUTEN                          ((uint32_t)0x0000FFFF)        /* Touchkey enables multi-channel shielding of each channel */
#define  ADC_DRV_TKEY_EN                             ((uint32_t)0x00010000)        /* Touchkey Multi Channel Shielding Enable */

/******************************************************************************/
/*                       High Speed Analog to Digital Converter                         */
/******************************************************************************/

/********************  Bit definition for HSADC_CFGR register  ********************/
#define  HSADC_EN                                    ((uint32_t)0x00000001)
#define  HSADC_DMAEN                                 ((uint32_t)0x00000002)

#define  HSADC_CHSEL                                 ((uint32_t)0x0000001C)
#define  HSADC_CHSEL_0                               ((uint32_t)0x00000004)
#define  HSADC_CHSEL_1                               ((uint32_t)0x00000008)
#define  HSADC_CHSEL_2                               ((uint32_t)0x00000010)

#define  HSADC_SETUP                                 ((uint32_t)0x00000060)
#define  HSADC_SETUP_0                               ((uint32_t)0x00000020)
#define  HSADC_SETUP_1                               ((uint32_t)0x00000040)

#define  HSADC_WIDTH                                 ((uint32_t)0x00000080)

#define  HSADC_CLKDIV                                ((uint32_t)0x00003F00)
#define  HSADC_CLKDIV_0                              ((uint32_t)0x00000100)
#define  HSADC_CLKDIV_1                              ((uint32_t)0x00000200)
#define  HSADC_CLKDIV_2                              ((uint32_t)0x00000400)
#define  HSADC_CLKDIV_3                              ((uint32_t)0x00000800)
#define  HSADC_CLKDIV_4                              ((uint32_t)0x00001000)
#define  HSADC_CLKDIV_5                              ((uint32_t)0x00002000)

#define  HSADC_PPMODE                                ((uint32_t)0x00004000)
#define  HSADC_BURST_EN                              ((uint32_t)0x00008000)

#define  HSADC_DMA_LEN                               ((uint32_t)0xFFFF0000)

/********************  Bit definition for HSADC_CTLR1 register  ********************/
#define  HSADC_START                                 ((uint32_t)0x00000001)
#define  HSADC_BURSTEND                              ((uint32_t)0x00000002)
#define  HSADC_EOCIE                                 ((uint32_t)0x00000100)
#define  HSADC_DMAIE                                 ((uint32_t)0x00000200)
#define  HSADC_BURSTIE                               ((uint32_t)0x00000400)

/********************  Bit definition for HSADC_CTLR2 register  ********************/
#define  HSADC_BURST_LEN                             ((uint32_t)0x0000FFFF)
#define  HSADC_BURST_DMA_LEN                         ((uint32_t)0xFFFF0000)

/********************  Bit definition for HSADC_STATR register  ********************/
#define  HSADC_EOCIF                                 ((uint32_t)0x00000001)
#define  HSADC_DMAIF                                 ((uint32_t)0x00000002)
#define  HSADC_BURSTIF                               ((uint32_t)0x00000004)
#define  HSADC_RXNE                                  ((uint32_t)0x00000008)
#define  HSADC_PP_ADDR                               ((uint32_t)0x00000010)

#define  HSADC_FIFO_RDY                              ((uint32_t)0x00000100)
#define  HSADC_FIFO_FULL                             ((uint32_t)0x00000200)
#define  HSADC_FIFO_OV                               ((uint32_t)0x00000400)

#define  HSADC_FIFO_CNT                              ((uint32_t)0x00003800)

/********************  Bit definition for HSADC_DATAR register  ********************/
#define  HSADC_DR                                    ((uint32_t)0x000003FF)

/********************  Bit definition for HSADC_ADDR0 register  ********************/
#define  HSADC_DMA_ADDR0                             ((uint32_t)0xFFFFFFFF)

/********************  Bit definition for HSADC_ADDR1 register  ********************/
#define  HSADC_DMA_ADDR1                             ((uint32_t)0xFFFFFFFF)

/******************************************************************************/
/*                         Controller Area Network                            */
/******************************************************************************/

/*******************  Bit definition for CAN_CTLR register  ********************/
#define  CAN_CTLR_INRQ                               ((uint16_t)0x0001)            /* Initialization Request */
#define  CAN_CTLR_SLEEP                              ((uint16_t)0x0002)            /* Sleep Mode Request */
#define  CAN_CTLR_TXFP                               ((uint16_t)0x0004)            /* Transmit FIFO Priority */
#define  CAN_CTLR_RFLM                               ((uint16_t)0x0008)            /* Receive FIFO Locked Mode */
#define  CAN_CTLR_NART                               ((uint16_t)0x0010)            /* No Automatic Retransmission */
#define  CAN_CTLR_AWUM                               ((uint16_t)0x0020)            /* Automatic Wakeup Mode */
#define  CAN_CTLR_ABOM                              ((uint16_t)0x0040)            /* Automatic Bus-Off Management */
#define  CAN_CTLR_TTCM                               ((uint16_t)0x0080)            /* Time Triggered Communication Mode */
#define  CAN_CTLR_RESET                              ((uint16_t)0x8000)            /* CAN software master reset */
#define  CAN_CTLR_DBF                                ((uint32_t)0x10000)
#define  CAN_CTLR_CFGCANM                            ((uint32_t)0x20000)
/*******************  Bit definition for CAN_STATR register  ********************/
#define  CAN_STATR_INAK                              ((uint16_t)0x0001)            /* Initialization Acknowledge */
#define  CAN_STATR_SLAK                              ((uint16_t)0x0002)            /* Sleep Acknowledge */
#define  CAN_STATR_ERRI                              ((uint16_t)0x0004)            /* Error Interrupt */
#define  CAN_STATR_WKUI                              ((uint16_t)0x0008)            /* Wakeup Interrupt */
#define  CAN_STATR_SLAKI                             ((uint16_t)0x0010)            /* Sleep Acknowledge Interrupt */
#define  CAN_STATR_TXM                               ((uint16_t)0x0100)            /* Transmit Mode */
#define  CAN_STATR_RXM                               ((uint16_t)0x0200)            /* Receive Mode */
#define  CAN_STATR_SAMP                              ((uint16_t)0x0400)            /* Last Sample Point */
#define  CAN_STATR_RX                                ((uint16_t)0x0800)            /* CAN Rx Signal */

/*******************  Bit definition for CAN_TSTATR register  ********************/
#define  CAN_TSTATR_RQCP0                            ((uint32_t)0x00000001)        /* Request Completed Mailbox0 */
#define  CAN_TSTATR_TXOK0                            ((uint32_t)0x00000002)        /* Transmission OK of Mailbox0 */
#define  CAN_TSTATR_ALST0                            ((uint32_t)0x00000004)        /* Arbitration Lost for Mailbox0 */
#define  CAN_TSTATR_TERR0                            ((uint32_t)0x00000008)        /* Transmission Error of Mailbox0 */
#define  CAN_TSTATR_ABRQ0                            ((uint32_t)0x00000080)        /* Abort Request for Mailbox0 */
#define  CAN_TSTATR_RQCP1                            ((uint32_t)0x00000100)        /* Request Completed Mailbox1 */
#define  CAN_TSTATR_TXOK1                            ((uint32_t)0x00000200)        /* Transmission OK of Mailbox1 */
#define  CAN_TSTATR_ALST1                            ((uint32_t)0x00000400)        /* Arbitration Lost for Mailbox1 */
#define  CAN_TSTATR_TERR1                            ((uint32_t)0x00000800)        /* Transmission Error of Mailbox1 */
#define  CAN_TSTATR_ABRQ1                            ((uint32_t)0x00008000)        /* Abort Request for Mailbox 1 */
#define  CAN_TSTATR_RQCP2                            ((uint32_t)0x00010000)        /* Request Completed Mailbox2 */
#define  CAN_TSTATR_TXOK2                            ((uint32_t)0x00020000)        /* Transmission OK of Mailbox 2 */
#define  CAN_TSTATR_ALST2                            ((uint32_t)0x00040000)        /* Arbitration Lost for mailbox 2 */
#define  CAN_TSTATR_TERR2                            ((uint32_t)0x00080000)        /* Transmission Error of Mailbox 2 */
#define  CAN_TSTATR_ABRQ2                            ((uint32_t)0x00800000)        /* Abort Request for Mailbox 2 */
#define  CAN_TSTATR_CODE                             ((uint32_t)0x03000000)        /* Mailbox Code */

#define  CAN_TSTATR_TME                              ((uint32_t)0x1C000000)        /* TME[2:0] bits */
#define  CAN_TSTATR_TME0                             ((uint32_t)0x04000000)        /* Transmit Mailbox 0 Empty */
#define  CAN_TSTATR_TME1                             ((uint32_t)0x08000000)        /* Transmit Mailbox 1 Empty */
#define  CAN_TSTATR_TME2                             ((uint32_t)0x10000000)        /* Transmit Mailbox 2 Empty */

#define  CAN_TSTATR_LOW                              ((uint32_t)0xE0000000)        /* LOW[2:0] bits */
#define  CAN_TSTATR_LOW0                             ((uint32_t)0x20000000)        /* Lowest Priority Flag for Mailbox 0 */
#define  CAN_TSTATR_LOW1                             ((uint32_t)0x40000000)        /* Lowest Priority Flag for Mailbox 1 */
#define  CAN_TSTATR_LOW2                             ((uint32_t)0x80000000)        /* Lowest Priority Flag for Mailbox 2 */

/*******************  Bit definition for CAN_RFIFO0 register  *******************/
#define  CAN_RFIFO0_FMP0                             ((uint8_t)0x03)               /* FIFO 0 Message Pending */
#define  CAN_RFIFO0_FULL0                            ((uint8_t)0x08)               /* FIFO 0 Full */
#define  CAN_RFIFO0_FOVR0                            ((uint8_t)0x10)               /* FIFO 0 Overrun */
#define  CAN_RFIFO0_RFOM0                            ((uint8_t)0x20)               /* Release FIFO 0 Output Mailbox */

/********************  Bit definition for CAN_INTENR register  *******************/
#define  CAN_INTENR_TMEIE                            ((uint32_t)0x00000001)        /* Transmit Mailbox Empty Interrupt Enable */
#define  CAN_INTENR_FMPIE0                           ((uint32_t)0x00000002)        /* FIFO Message Pending Interrupt Enable */
#define  CAN_INTENR_FFIE0                            ((uint32_t)0x00000004)        /* FIFO Full Interrupt Enable */
#define  CAN_INTENR_FOVIE0                           ((uint32_t)0x00000008)        /* FIFO Overrun Interrupt Enable */
#define  CAN_INTENR_EWGIE                            ((uint32_t)0x00000100)        /* Error Warning Interrupt Enable */
#define  CAN_INTENR_EPVIE                            ((uint32_t)0x00000200)        /* Error Passive Interrupt Enable */
#define  CAN_INTENR_BOFIE                            ((uint32_t)0x00000400)        /* Bus-Off Interrupt Enable */
#define  CAN_INTENR_LECIE                            ((uint32_t)0x00000800)        /* Last Error Code Interrupt Enable */
#define  CAN_INTENR_ERRIE                            ((uint32_t)0x00008000)        /* Error Interrupt Enable */
#define  CAN_INTENR_WKUIE                            ((uint32_t)0x00010000)        /* Wakeup Interrupt Enable */
#define  CAN_INTENR_SLKIE                            ((uint32_t)0x00020000)        /* Sleep Interrupt Enable */

/********************  Bit definition for CAN_ERRSR register  *******************/
#define  CAN_ERRSR_EWGF                              ((uint32_t)0x00000001)        /* Error Warning Flag */
#define  CAN_ERRSR_EPVF                              ((uint32_t)0x00000002)        /* Error Passive Flag */
#define  CAN_ERRSR_BOFF                              ((uint32_t)0x00000004)        /* Bus-Off Flag */

#define  CAN_ERRSR_LEC                               ((uint32_t)0x00000070)        /* LEC[2:0] bits (Last Error Code) */
#define  CAN_ERRSR_LEC_0                             ((uint32_t)0x00000010)        /* Bit 0 */
#define  CAN_ERRSR_LEC_1                             ((uint32_t)0x00000020)        /* Bit 1 */
#define  CAN_ERRSR_LEC_2                             ((uint32_t)0x00000040)        /* Bit 2 */

#define  CAN_ERRSR_TEC                               ((uint32_t)0x00FF0000)        /* Least significant byte of the 9-bit Transmit Error Counter */
#define  CAN_ERRSR_REC                               ((uint32_t)0xFF000000)        /* Receive Error Counter */

/********************  Bit definition for CAN_BTIMR register  *******************/
#define  CAN_BTIMR_BRP                               ((uint32_t)0x000003FF)
#define  CAN_BTIMR_BTR_TS1_T                         ((uint32_t)0x0000F000)
#define  CAN_BTIMR_TS1                               ((uint32_t)0x000F0000)
#define  CAN_BTIMR_TS2                               ((uint32_t)0x00F00000)
#define  CAN_BTIMR_SJW                               ((uint32_t)0x0F000000)

#define  CAN_BTIMR_LBKM                              ((uint32_t)0x40000000)
#define  CAN_BTIMR_SILM                              ((uint32_t)0x80000000)

/*******************  Bit definition for CAN_TTCTLR register  ********************/
#define  CAN_TTCTLR_TIMCMV                           ((uint32_t)0x0000FFFF)
#define  CAN_TTCTLR_TIMRST                           ((uint32_t)0x00010000)
#define  CAN_TTCTLR_MODE                             ((uint32_t)0x00020000)

/*******************  Bit definition for CAN_TTCNT register  ********************/
#define  CAN_TIMCNT                                  ((uint32_t)0x0000FFFF)

/*******************  Bit definition for CAN_TERR_CNT register  ********************/
#define  CAN_TERR_CNT                                ((uint32_t)0x000001FF)

/******************  Bit definition for CAN_TXMI0R register  ********************/
#define CAN_TXMI0R_TXRQ                              ((uint32_t)0x00000001) /* Transmit Mailbox Request */
#define CAN_TXMI0R_RTR                               ((uint32_t)0x00000002) /* Remote Transmission Request */
#define CAN_TXMI0R_IDE                               ((uint32_t)0x00000004) /* Identifier Extension */
#define CAN_TXMI0R_EXID                              ((uint32_t)0x001FFFF8) /* Extended Identifier */
#define CAN_TXMI0R_STID                              ((uint32_t)0xFFE00000) /* Standard Identifier or Extended Identifier */

/******************  Bit definition for CAN_TXMDT0R register  *******************/
#define CAN_TXMDT0R_DLC                              ((uint32_t)0x0000000F) /* Data Length Code */
#define CAN_TXMDT0R_TGT                              ((uint32_t)0x00000100) /* Transmit Global Time */
#define CAN_TXMDT0R_TIME                             ((uint32_t)0xFFFF0000) /* Message Time Stamp */

/******************  Bit definition for CAN_TXMDL0R register  *******************/
#define CAN_TXMDL0R_DATA0                            ((uint32_t)0x000000FF) /* Data byte 0 */
#define CAN_TXMDL0R_DATA1                            ((uint32_t)0x0000FF00) /* Data byte 1 */
#define CAN_TXMDL0R_DATA2                            ((uint32_t)0x00FF0000) /* Data byte 2 */
#define CAN_TXMDL0R_DATA3                            ((uint32_t)0xFF000000) /* Data byte 3 */

/******************  Bit definition for CAN_TXMDH0R register  *******************/
#define CAN_TXMDH0R_DATA4                            ((uint32_t)0x000000FF) /* Data byte 4 */
#define CAN_TXMDH0R_DATA5                            ((uint32_t)0x0000FF00) /* Data byte 5 */
#define CAN_TXMDH0R_DATA6                            ((uint32_t)0x00FF0000) /* Data byte 6 */
#define CAN_TXMDH0R_DATA7                            ((uint32_t)0xFF000000) /* Data byte 7 */

/*******************  Bit definition for CAN_TXMI1R register  *******************/
#define CAN_TXMI1R_TXRQ                              ((uint32_t)0x00000001) /* Transmit Mailbox Request */
#define CAN_TXMI1R_RTR                               ((uint32_t)0x00000002) /* Remote Transmission Request */
#define CAN_TXMI1R_IDE                               ((uint32_t)0x00000004) /* Identifier Extension */
#define CAN_TXMI1R_EXID                              ((uint32_t)0x001FFFF8) /* Extended Identifier */
#define CAN_TXMI1R_STID                              ((uint32_t)0xFFE00000) /* Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TXMDT1R register  ******************/
#define CAN_TXMDT1R_DLC                              ((uint32_t)0x0000000F) /* Data Length Code */
#define CAN_TXMDT1R_TGT                              ((uint32_t)0x00000100) /* Transmit Global Time */
#define CAN_TXMDT1R_TIME                             ((uint32_t)0xFFFF0000) /* Message Time Stamp */

/*******************  Bit definition for CAN_TXMDL1R register  ******************/
#define CAN_TXMDL1R_DATA0                            ((uint32_t)0x000000FF) /* Data byte 0 */
#define CAN_TXMDL1R_DATA1                            ((uint32_t)0x0000FF00) /* Data byte 1 */
#define CAN_TXMDL1R_DATA2                            ((uint32_t)0x00FF0000) /* Data byte 2 */
#define CAN_TXMDL1R_DATA3                            ((uint32_t)0xFF000000) /* Data byte 3 */

/*******************  Bit definition for CAN_TXMDH1R register  ******************/
#define CAN_TXMDH1R_DATA4                            ((uint32_t)0x000000FF) /* Data byte 4 */
#define CAN_TXMDH1R_DATA5                            ((uint32_t)0x0000FF00) /* Data byte 5 */
#define CAN_TXMDH1R_DATA6                            ((uint32_t)0x00FF0000) /* Data byte 6 */
#define CAN_TXMDH1R_DATA7                            ((uint32_t)0xFF000000) /* Data byte 7 */

/*******************  Bit definition for CAN_TXMI2R register  *******************/
#define CAN_TXMI2R_TXRQ                              ((uint32_t)0x00000001) /* Transmit Mailbox Request */
#define CAN_TXMI2R_RTR                               ((uint32_t)0x00000002) /* Remote Transmission Request */
#define CAN_TXMI2R_IDE                               ((uint32_t)0x00000004) /* Identifier Extension */
#define CAN_TXMI2R_EXID                              ((uint32_t)0x001FFFF8) /* Extended identifier */
#define CAN_TXMI2R_STID                              ((uint32_t)0xFFE00000) /* Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TXMDT2R register  ******************/
#define CAN_TXMDT2R_DLC                              ((uint32_t)0x0000000F) /* Data Length Code */
#define CAN_TXMDT2R_TGT                              ((uint32_t)0x00000100) /* Transmit Global Time */
#define CAN_TXMDT2R_TIME                             ((uint32_t)0xFFFF0000) /* Message Time Stamp */

/*******************  Bit definition for CAN_TXMDL2R register  ******************/
#define CAN_TXMDL2R_DATA0                            ((uint32_t)0x000000FF) /* Data byte 0 */
#define CAN_TXMDL2R_DATA1                            ((uint32_t)0x0000FF00) /* Data byte 1 */
#define CAN_TXMDL2R_DATA2                            ((uint32_t)0x00FF0000) /* Data byte 2 */
#define CAN_TXMDL2R_DATA3                            ((uint32_t)0xFF000000) /* Data byte 3 */

/*******************  Bit definition for CAN_TXMDH2R register  ******************/
#define CAN_TXMDH2R_DATA4                            ((uint32_t)0x000000FF) /* Data byte 4 */
#define CAN_TXMDH2R_DATA5                            ((uint32_t)0x0000FF00) /* Data byte 5 */
#define CAN_TXMDH2R_DATA6                            ((uint32_t)0x00FF0000) /* Data byte 6 */
#define CAN_TXMDH2R_DATA7                            ((uint32_t)0xFF000000) /* Data byte 7 */

/*******************  Bit definition for CAN_RXMI0R register  *******************/
#define CAN_RXMIOR_FDF                               ((uint32_t)0x00000001)
#define CAN_RXMI0R_RTR                               ((uint32_t)0x00000002) /* Remote Transmission Request */
#define CAN_RXMI0R_IDE                               ((uint32_t)0x00000004) /* Identifier Extension */
#define CAN_RXMI0R_EXID                              ((uint32_t)0x001FFFF8) /* Extended Identifier */
#define CAN_RXMI0R_STID                              ((uint32_t)0xFFE00000) /* Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RXMDT0R register  ******************/
#define CAN_RXMDT0R_DLC                              ((uint32_t)0x0000000F) /* Data Length Code */
#define CAN_RXMDT0R_BRS                              ((uint32_t)0x00000010)
#define CAN_RXMDT0R_ESI                              ((uint32_t)0x00000020)
#define CAN_RXMDH0R_RES                              ((uint32_t)0x00000100)
#define CAN_RXMDT0R_TIME                             ((uint32_t)0xFFFF0000) /* Message Time Stamp */

/*******************  Bit definition for CAN_RXMDL0R register  ******************/
#define CAN_RXMDL0R_DATA0                            ((uint32_t)0x000000FF) /* Data byte 0 */
#define CAN_RXMDL0R_DATA1                            ((uint32_t)0x0000FF00) /* Data byte 1 */
#define CAN_RXMDL0R_DATA2                            ((uint32_t)0x00FF0000) /* Data byte 2 */
#define CAN_RXMDL0R_DATA3                            ((uint32_t)0xFF000000) /* Data byte 3 */

/*******************  Bit definition for CAN_RXMDH0R register  ******************/
#define CAN_RXMDH0R_DATA4                            ((uint32_t)0x000000FF) /* Data byte 4 */
#define CAN_RXMDH0R_DATA5                            ((uint32_t)0x0000FF00) /* Data byte 5 */
#define CAN_RXMDH0R_DATA6                            ((uint32_t)0x00FF0000) /* Data byte 6 */
#define CAN_RXMDH0R_DATA7                            ((uint32_t)0xFF000000) /* Data byte 7 */

/******************************************************************************/
/*                          CRC Calculation Unit                              */
/******************************************************************************/

/*******************  Bit definition for CRC_DATAR register  *********************/
#define  CRC_DATAR_DR                                ((uint32_t)0xFFFFFFFF) /* Data register bits */

/*******************  Bit definition for CRC_IDATAR register  ********************/
#define  CRC_IDR_IDATAR                              ((uint8_t)0xFF)        /* General-purpose 8-bit data register bits */

/********************  Bit definition for CRC_CTLR register  ********************/
#define  CRC_CTLR_RESET                              ((uint8_t)0x01)        /* RESET bit */

/******************************************************************************/
/*                      Digital to Analog Converter                           */
/******************************************************************************/

/********************  Bit definition for DAC_CTLR register  ********************/
#define  DAC_EN1                                     ((uint32_t)0x00000001)        /* DAC channel1 enable */
#define  DAC_BOFF1                                   ((uint32_t)0x00000002)        /* DAC channel1 output buffer disable */
#define  DAC_TEN1                                    ((uint32_t)0x00000004)        /* DAC channel1 Trigger enable */

#define  DAC_TSEL1                                   ((uint32_t)0x00000038)        /* TSEL1[2:0] (DAC channel1 Trigger selection) */
#define  DAC_TSEL1_0                                 ((uint32_t)0x00000008)        /* Bit 0 */
#define  DAC_TSEL1_1                                 ((uint32_t)0x00000010)        /* Bit 1 */
#define  DAC_TSEL1_2                                 ((uint32_t)0x00000020)        /* Bit 2 */

#define  DAC_WAVE1                                   ((uint32_t)0x000000C0)        /* WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) */
#define  DAC_WAVE1_0                                 ((uint32_t)0x00000040)        /* Bit 0 */
#define  DAC_WAVE1_1                                 ((uint32_t)0x00000080)        /* Bit 1 */

#define  DAC_MAMP1                                   ((uint32_t)0x00000F00)        /* MAMP1[3:0] (DAC channel1 Mask/Amplitude selector) */
#define  DAC_MAMP1_0                                 ((uint32_t)0x00000100)        /* Bit 0 */
#define  DAC_MAMP1_1                                 ((uint32_t)0x00000200)        /* Bit 1 */
#define  DAC_MAMP1_2                                 ((uint32_t)0x00000400)        /* Bit 2 */
#define  DAC_MAMP1_3                                 ((uint32_t)0x00000800)        /* Bit 3 */

#define  DAC_DMAEN1                                  ((uint32_t)0x00001000)        /* DAC channel1 DMA enable */
#define  DAC_EN2                                     ((uint32_t)0x00010000)        /* DAC channel2 enable */
#define  DAC_BOFF2                                   ((uint32_t)0x00020000)        /* DAC channel2 output buffer disable */
#define  DAC_TEN2                                    ((uint32_t)0x00040000)        /* DAC channel2 Trigger enable */

#define  DAC_TSEL2                                   ((uint32_t)0x00380000)        /* TSEL2[2:0] (DAC channel2 Trigger selection) */
#define  DAC_TSEL2_0                                 ((uint32_t)0x00080000)        /* Bit 0 */
#define  DAC_TSEL2_1                                 ((uint32_t)0x00100000)        /* Bit 1 */
#define  DAC_TSEL2_2                                 ((uint32_t)0x00200000)        /* Bit 2 */

#define  DAC_WAVE2                                   ((uint32_t)0x00C00000)        /* WAVE2[1:0] (DAC channel2 noise/triangle wave generation enable) */
#define  DAC_WAVE2_0                                 ((uint32_t)0x00400000)        /* Bit 0 */
#define  DAC_WAVE2_1                                 ((uint32_t)0x00800000)        /* Bit 1 */

#define  DAC_MAMP2                                   ((uint32_t)0x0F000000)        /* MAMP2[3:0] (DAC channel2 Mask/Amplitude selector) */
#define  DAC_MAMP2_0                                 ((uint32_t)0x01000000)        /* Bit 0 */
#define  DAC_MAMP2_1                                 ((uint32_t)0x02000000)        /* Bit 1 */
#define  DAC_MAMP2_2                                 ((uint32_t)0x04000000)        /* Bit 2 */
#define  DAC_MAMP2_3                                 ((uint32_t)0x08000000)        /* Bit 3 */

#define  DAC_DMAEN2                                  ((uint32_t)0x10000000)        /* DAC channel2 DMA enabled */

/*****************  Bit definition for DAC_SWTR register  ******************/
#define  DAC_SWTRIG1                                 ((uint8_t)0x01)               /* DAC channel1 software trigger */
#define  DAC_SWTRIG2                                 ((uint8_t)0x02)               /* DAC channel2 software trigger */

/*****************  Bit definition for DAC_R12BDHR1 register  ******************/
#define  DAC_DHR12R1                                 ((uint16_t)0x0FFF)            /* DAC channel1 12-bit Right aligned data */

/*****************  Bit definition for DAC_L12BDHR1 register  ******************/
#define  DAC_DHR12L1                                 ((uint16_t)0xFFF0)            /* DAC channel1 12-bit Left aligned data */

/******************  Bit definition for DAC_R8BDHR1 register  ******************/
#define  DAC_DHR8R1                                  ((uint8_t)0xFF)               /* DAC channel1 8-bit Right aligned data */

/*****************  Bit definition for DAC_R12BDHR2 register  ******************/
#define  DAC_DHR12R2                                 ((uint16_t)0x0FFF)            /* DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_L12BDHR2 register  ******************/
#define  DAC_DHR12L2                                 ((uint16_t)0xFFF0)            /* DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_R8BDHR2 register  ******************/
#define  DAC_DHR8R2                                  ((uint8_t)0xFF)               /* DAC channel2 8-bit Right aligned data */

/*****************  Bit definition for DAC_RD12BDHR register  ******************/
#define  DAC_RD12BDHR_DACC1DHR                       ((uint32_t)0x00000FFF)        /* DAC channel1 12-bit Right aligned data */
#define  DAC_RD12BDHR_DACC2DHR                       ((uint32_t)0x0FFF0000)        /* DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_LD12BDHR register  ******************/
#define  DAC_LD12BDHR_DACC1DHR                       ((uint32_t)0x0000FFF0)        /* DAC channel1 12-bit Left aligned data */
#define  DAC_LD12BDHR_DACC2DHR                       ((uint32_t)0xFFF00000)        /* DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_RD8BDHR register  ******************/
#define  DAC_RD8BDHR_DACC1DHR                        ((uint16_t)0x00FF)            /* DAC channel1 8-bit Right aligned data */
#define  DAC_RD8BDHR_DACC2DHR                        ((uint16_t)0xFF00)            /* DAC channel2 8-bit Right aligned data */

/*******************  Bit definition for DAC_DOR1 register  *******************/
#define  DAC_DACC1DOR                                ((uint16_t)0x0FFF)            /* DAC channel1 data output */

/*******************  Bit definition for DAC_DOR2 register  *******************/
#define  DAC_DACC2DOR                                ((uint16_t)0x0FFF)            /* DAC channel2 data output */

/******************************************************************************/
/*                             DMA Controller                                 */
/******************************************************************************/

/*******************  Bit definition for DMA_INTFR register  ********************/
#define  DMA_GIF1                                    ((uint32_t)0x00000001)        /* Channel 1 Global interrupt flag */
#define  DMA_TCIF1                                   ((uint32_t)0x00000002)        /* Channel 1 Transfer Complete flag */
#define  DMA_HTIF1                                   ((uint32_t)0x00000004)        /* Channel 1 Half Transfer flag */
#define  DMA_TEIF1                                   ((uint32_t)0x00000008)        /* Channel 1 Transfer Error flag */
#define  DMA_GIF2                                    ((uint32_t)0x00000010)        /* Channel 2 Global interrupt flag */
#define  DMA_TCIF2                                   ((uint32_t)0x00000020)        /* Channel 2 Transfer Complete flag */
#define  DMA_HTIF2                                   ((uint32_t)0x00000040)        /* Channel 2 Half Transfer flag */
#define  DMA_TEIF2                                   ((uint32_t)0x00000080)        /* Channel 2 Transfer Error flag */
#define  DMA_GIF3                                    ((uint32_t)0x00000100)        /* Channel 3 Global interrupt flag */
#define  DMA_TCIF3                                   ((uint32_t)0x00000200)        /* Channel 3 Transfer Complete flag */
#define  DMA_HTIF3                                   ((uint32_t)0x00000400)        /* Channel 3 Half Transfer flag */
#define  DMA_TEIF3                                   ((uint32_t)0x00000800)        /* Channel 3 Transfer Error flag */
#define  DMA_GIF4                                    ((uint32_t)0x00001000)        /* Channel 4 Global interrupt flag */
#define  DMA_TCIF4                                   ((uint32_t)0x00002000)        /* Channel 4 Transfer Complete flag */
#define  DMA_HTIF4                                   ((uint32_t)0x00004000)        /* Channel 4 Half Transfer flag */
#define  DMA_TEIF4                                   ((uint32_t)0x00008000)        /* Channel 4 Transfer Error flag */
#define  DMA_GIF5                                    ((uint32_t)0x00010000)        /* Channel 5 Global interrupt flag */
#define  DMA_TCIF5                                   ((uint32_t)0x00020000)        /* Channel 5 Transfer Complete flag */
#define  DMA_HTIF5                                   ((uint32_t)0x00040000)        /* Channel 5 Half Transfer flag */
#define  DMA_TEIF5                                   ((uint32_t)0x00080000)        /* Channel 5 Transfer Error flag */
#define  DMA_GIF6                                    ((uint32_t)0x00100000)        /* Channel 6 Global interrupt flag */
#define  DMA_TCIF6                                   ((uint32_t)0x00200000)        /* Channel 6 Transfer Complete flag */
#define  DMA_HTIF6                                   ((uint32_t)0x00400000)        /* Channel 6 Half Transfer flag */
#define  DMA_TEIF6                                   ((uint32_t)0x00800000)        /* Channel 6 Transfer Error flag */
#define  DMA_GIF7                                    ((uint32_t)0x01000000)        /* Channel 7 Global interrupt flag */
#define  DMA_TCIF7                                   ((uint32_t)0x02000000)        /* Channel 7 Transfer Complete flag */
#define  DMA_HTIF7                                   ((uint32_t)0x04000000)        /* Channel 7 Half Transfer flag */
#define  DMA_TEIF7                                   ((uint32_t)0x08000000)        /* Channel 7 Transfer Error flag */
#define  DMA_GIF8                                    ((uint32_t)0x10000000)        /* Channel 8 Global interrupt flag */
#define  DMA_TCIF8                                   ((uint32_t)0x20000000)        /* Channel 8 Transfer Complete flag */
#define  DMA_HTIF8                                   ((uint32_t)0x40000000)        /* Channel 8 Half Transfer flag */
#define  DMA_TEIF8                                   ((uint32_t)0x80000000)        /* Channel 8 Transfer Error flag */

/*******************  Bit definition for DMA_INTFCR register  *******************/
#define  DMA_CGIF1                                   ((uint32_t)0x00000001)        /* Channel 1 Global interrupt clear */
#define  DMA_CTCIF1                                  ((uint32_t)0x00000002)        /* Channel 1 Transfer Complete clear */
#define  DMA_CHTIF1                                  ((uint32_t)0x00000004)        /* Channel 1 Half Transfer clear */
#define  DMA_CTEIF1                                  ((uint32_t)0x00000008)        /* Channel 1 Transfer Error clear */
#define  DMA_CGIF2                                   ((uint32_t)0x00000010)        /* Channel 2 Global interrupt clear */
#define  DMA_CTCIF2                                  ((uint32_t)0x00000020)        /* Channel 2 Transfer Complete clear */
#define  DMA_CHTIF2                                  ((uint32_t)0x00000040)        /* Channel 2 Half Transfer clear */
#define  DMA_CTEIF2                                  ((uint32_t)0x00000080)        /* Channel 2 Transfer Error clear */
#define  DMA_CGIF3                                   ((uint32_t)0x00000100)        /* Channel 3 Global interrupt clear */
#define  DMA_CTCIF3                                  ((uint32_t)0x00000200)        /* Channel 3 Transfer Complete clear */
#define  DMA_CHTIF3                                  ((uint32_t)0x00000400)        /* Channel 3 Half Transfer clear */
#define  DMA_CTEIF3                                  ((uint32_t)0x00000800)        /* Channel 3 Transfer Error clear */
#define  DMA_CGIF4                                   ((uint32_t)0x00001000)        /* Channel 4 Global interrupt clear */
#define  DMA_CTCIF4                                  ((uint32_t)0x00002000)        /* Channel 4 Transfer Complete clear */
#define  DMA_CHTIF4                                  ((uint32_t)0x00004000)        /* Channel 4 Half Transfer clear */
#define  DMA_CTEIF4                                  ((uint32_t)0x00008000)        /* Channel 4 Transfer Error clear */
#define  DMA_CGIF5                                   ((uint32_t)0x00010000)        /* Channel 5 Global interrupt clear */
#define  DMA_CTCIF5                                  ((uint32_t)0x00020000)        /* Channel 5 Transfer Complete clear */
#define  DMA_CHTIF5                                  ((uint32_t)0x00040000)        /* Channel 5 Half Transfer clear */
#define  DMA_CTEIF5                                  ((uint32_t)0x00080000)        /* Channel 5 Transfer Error clear */
#define  DMA_CGIF6                                   ((uint32_t)0x00100000)        /* Channel 6 Global interrupt clear */
#define  DMA_CTCIF6                                  ((uint32_t)0x00200000)        /* Channel 6 Transfer Complete clear */
#define  DMA_CHTIF6                                  ((uint32_t)0x00400000)        /* Channel 6 Half Transfer clear */
#define  DMA_CTEIF6                                  ((uint32_t)0x00800000)        /* Channel 6 Transfer Error clear */
#define  DMA_CGIF7                                   ((uint32_t)0x01000000)        /* Channel 7 Global interrupt clear */
#define  DMA_CTCIF7                                  ((uint32_t)0x02000000)        /* Channel 7 Transfer Complete clear */
#define  DMA_CHTIF7                                  ((uint32_t)0x04000000)        /* Channel 7 Half Transfer clear */
#define  DMA_CTEIF7                                  ((uint32_t)0x08000000)        /* Channel 7 Transfer Error clear */
#define  DMA_CGIF8                                   ((uint32_t)0x10000000)        /* Channel 8 Global interrupt clear */
#define  DMA_CTCIF8                                  ((uint32_t)0x20000000)        /* Channel 8 Transfer Complete clear */
#define  DMA_CHTIF8                                  ((uint32_t)0x40000000)        /* Channel 8 Half Transfer clear */
#define  DMA_CTEIF8                                  ((uint32_t)0x80000000)        /* Channel 8 Transfer Error clear */

/*******************  Bit definition for DMA_CFGR1 register  *******************/
#define  DMA_CFGR1_EN                                ((uint16_t)0x0001)            /* Channel enable*/
#define  DMA_CFGR1_TCIE                              ((uint16_t)0x0002)            /* Transfer complete interrupt enable */
#define  DMA_CFGR1_HTIE                              ((uint16_t)0x0004)            /* Half Transfer interrupt enable */
#define  DMA_CFGR1_TEIE                              ((uint16_t)0x0008)            /* Transfer error interrupt enable */
#define  DMA_CFGR1_DIR                               ((uint16_t)0x0010)            /* Data transfer direction */
#define  DMA_CFGR1_CIRC                              ((uint16_t)0x0020)            /* Circular mode */
#define  DMA_CFGR1_PINC                              ((uint16_t)0x0040)            /* Peripheral increment mode */
#define  DMA_CFGR1_MINC                              ((uint16_t)0x0080)            /* Memory increment mode */

#define  DMA_CFGR1_PSIZE                             ((uint16_t)0x0300)            /* PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CFGR1_PSIZE_0                           ((uint16_t)0x0100)            /* Bit 0 */
#define  DMA_CFGR1_PSIZE_1                           ((uint16_t)0x0200)            /* Bit 1 */
         
#define  DMA_CFGR1_MSIZE                             ((uint16_t)0x0C00)            /* MSIZE[1:0] bits (Memory size) */
#define  DMA_CFGR1_MSIZE_0                           ((uint16_t)0x0400)            /* Bit 0 */
#define  DMA_CFGR1_MSIZE_1                           ((uint16_t)0x0800)            /* Bit 1 */
         
#define  DMA_CFGR1_PL                                ((uint16_t)0x3000)            /* PL[1:0] bits(Channel Priority level) */
#define  DMA_CFGR1_PL_0                              ((uint16_t)0x1000)            /* Bit 0 */
#define  DMA_CFGR1_PL_1                              ((uint16_t)0x2000)            /* Bit 1 */
         
#define  DMA_CFGR1_MEM2MEM                           ((uint16_t)0x4000)            /* Memory to memory mode */

/*******************  Bit definition for DMA_CFGR2 register  *******************/
#define  DMA_CFGR2_EN                                ((uint16_t)0x0001)            /* Channel enable */
#define  DMA_CFGR2_TCIE                              ((uint16_t)0x0002)            /* Transfer complete interrupt enable */
#define  DMA_CFGR2_HTIE                              ((uint16_t)0x0004)            /* Half Transfer interrupt enable */
#define  DMA_CFGR2_TEIE                              ((uint16_t)0x0008)            /* Transfer error interrupt enable */
#define  DMA_CFGR2_DIR                               ((uint16_t)0x0010)            /* Data transfer direction */
#define  DMA_CFGR2_CIRC                              ((uint16_t)0x0020)            /* Circular mode */
#define  DMA_CFGR2_PINC                              ((uint16_t)0x0040)            /* Peripheral increment mode */
#define  DMA_CFGR2_MINC                              ((uint16_t)0x0080)            /* Memory increment mode */
         
#define  DMA_CFGR2_PSIZE                             ((uint16_t)0x0300)            /* PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CFGR2_PSIZE_0                           ((uint16_t)0x0100)            /* Bit 0 */
#define  DMA_CFGR2_PSIZE_1                           ((uint16_t)0x0200)            /* Bit 1 */
         
#define  DMA_CFGR2_MSIZE                             ((uint16_t)0x0C00)            /* MSIZE[1:0] bits (Memory size) */
#define  DMA_CFGR2_MSIZE_0                           ((uint16_t)0x0400)            /* Bit 0 */
#define  DMA_CFGR2_MSIZE_1                           ((uint16_t)0x0800)            /* Bit 1 */
         
#define  DMA_CFGR2_PL                                ((uint16_t)0x3000)            /* PL[1:0] bits (Channel Priority level) */
#define  DMA_CFGR2_PL_0                              ((uint16_t)0x1000)            /* Bit 0 */
#define  DMA_CFGR2_PL_1                              ((uint16_t)0x2000)            /* Bit 1 */
         
#define  DMA_CFGR2_MEM2MEM                           ((uint16_t)0x4000)            /* Memory to memory mode */

/*******************  Bit definition for DMA_CFGR3 register  *******************/
#define  DMA_CFGR3_EN                                ((uint16_t)0x0001)            /* Channel enable */
#define  DMA_CFGR3_TCIE                              ((uint16_t)0x0002)            /* Transfer complete interrupt enable */
#define  DMA_CFGR3_HTIE                              ((uint16_t)0x0004)            /* Half Transfer interrupt enable */
#define  DMA_CFGR3_TEIE                              ((uint16_t)0x0008)            /* Transfer error interrupt enable */
#define  DMA_CFGR3_DIR                               ((uint16_t)0x0010)            /* Data transfer direction */
#define  DMA_CFGR3_CIRC                              ((uint16_t)0x0020)            /* Circular mode */
#define  DMA_CFGR3_PINC                              ((uint16_t)0x0040)            /* Peripheral increment mode */
#define  DMA_CFGR3_MINC                              ((uint16_t)0x0080)            /* Memory increment mode */
        
#define  DMA_CFGR3_PSIZE                             ((uint16_t)0x0300)            /* PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CFGR3_PSIZE_0                           ((uint16_t)0x0100)            /* Bit 0 */
#define  DMA_CFGR3_PSIZE_1                           ((uint16_t)0x0200)            /* Bit 1 */
        
#define  DMA_CFGR3_MSIZE                             ((uint16_t)0x0C00)            /* MSIZE[1:0] bits (Memory size) */
#define  DMA_CFGR3_MSIZE_0                           ((uint16_t)0x0400)            /* Bit 0 */
#define  DMA_CFGR3_MSIZE_1                           ((uint16_t)0x0800)            /* Bit 1 */
        
#define  DMA_CFGR3_PL                                ((uint16_t)0x3000)            /* PL[1:0] bits (Channel Priority level) */
#define  DMA_CFGR3_PL_0                              ((uint16_t)0x1000)            /* Bit 0 */
#define  DMA_CFGR3_PL_1                              ((uint16_t)0x2000)            /* Bit 1 */
        
#define  DMA_CFGR3_MEM2MEM                           ((uint16_t)0x4000)            /* Memory to memory mode */

/*******************  Bit definition for DMA_CFGR4 register  *******************/
#define  DMA_CFGR4_EN                                ((uint16_t)0x0001)            /* Channel enable */
#define  DMA_CFGR4_TCIE                              ((uint16_t)0x0002)            /* Transfer complete interrupt enable */
#define  DMA_CFGR4_HTIE                              ((uint16_t)0x0004)            /* Half Transfer interrupt enable */
#define  DMA_CFGR4_TEIE                              ((uint16_t)0x0008)            /* Transfer error interrupt enable */
#define  DMA_CFGR4_DIR                               ((uint16_t)0x0010)            /* Data transfer direction */
#define  DMA_CFGR4_CIRC                              ((uint16_t)0x0020)            /* Circular mode */
#define  DMA_CFGR4_PINC                              ((uint16_t)0x0040)            /* Peripheral increment mode */
#define  DMA_CFGR4_MINC                              ((uint16_t)0x0080)            /* Memory increment mode */
        
#define  DMA_CFGR4_PSIZE                             ((uint16_t)0x0300)            /* PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CFGR4_PSIZE_0                           ((uint16_t)0x0100)            /* Bit 0 */
#define  DMA_CFGR4_PSIZE_1                           ((uint16_t)0x0200)            /* Bit 1 */
                      
#define  DMA_CFGR4_MSIZE                             ((uint16_t)0x0C00)            /* MSIZE[1:0] bits (Memory size) */
#define  DMA_CFGR4_MSIZE_0                           ((uint16_t)0x0400)            /* Bit 0 */
#define  DMA_CFGR4_MSIZE_1                           ((uint16_t)0x0800)            /* Bit 1 */
                      
#define  DMA_CFGR4_PL                                ((uint16_t)0x3000)            /* PL[1:0] bits (Channel Priority level) */
#define  DMA_CFGR4_PL_0                              ((uint16_t)0x1000)            /* Bit 0 */
#define  DMA_CFGR4_PL_1                              ((uint16_t)0x2000)            /* Bit 1 */
                      
#define  DMA_CFGR4_MEM2MEM                           ((uint16_t)0x4000)            /* Memory to memory mode */

/******************  Bit definition for DMA_CFGR5 register  *******************/
#define  DMA_CFGR5_EN                                ((uint16_t)0x0001)            /* Channel enable */
#define  DMA_CFGR5_TCIE                              ((uint16_t)0x0002)            /* Transfer complete interrupt enable */
#define  DMA_CCFGR_HTIE                              ((uint16_t)0x0004)            /* Half Transfer interrupt enable */
#define  DMA_CFGR5_TEIE                              ((uint16_t)0x0008)            /* Transfer error interrupt enable */
#define  DMA_CFGR5_DIR                               ((uint16_t)0x0010)            /* Data transfer direction */
#define  DMA_CFGR5_CIRC                              ((uint16_t)0x0020)            /* Circular mode */
#define  DMA_CFGR5_PINC                              ((uint16_t)0x0040)            /* Peripheral increment mode */
#define  DMA_CFGR5_MINC                              ((uint16_t)0x0080)            /* Memory increment mode */
                      
#define  DMA_CFGR5_PSIZE                             ((uint16_t)0x0300)            /* PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CFGR5_PSIZE_0                           ((uint16_t)0x0100)            /* Bit 0 */
#define  DMA_CFGR5_PSIZE_1                           ((uint16_t)0x0200)            /* Bit 1 */
                      
#define  DMA_CFGR5_MSIZE                             ((uint16_t)0x0C00)            /* MSIZE[1:0] bits (Memory size) */
#define  DMA_CFGR5_MSIZE_0                           ((uint16_t)0x0400)            /* Bit 0 */
#define  DMA_CFGR5_MSIZE_1                           ((uint16_t)0x0800)            /* Bit 1 */
                      
#define  DMA_CFGR5_PL                                ((uint16_t)0x3000)            /* PL[1:0] bits (Channel Priority level) */
#define  DMA_CFGR5_PL_0                              ((uint16_t)0x1000)            /* Bit 0 */
#define  DMA_CFGR5_PL_1                              ((uint16_t)0x2000)            /* Bit 1 */
                      
#define  DMA_CFGR5_MEM2MEM                           ((uint16_t)0x4000)            /* Memory to memory mode enable */

/*******************  Bit definition for DMA_CFGR6 register  *******************/
#define  DMA_CFGR6_EN                                ((uint16_t)0x0001)            /* Channel enable */
#define  DMA_CFGR6_TCIE                              ((uint16_t)0x0002)            /* Transfer complete interrupt enable */
#define  DMA_CFGR6_HTIE                              ((uint16_t)0x0004)            /* Half Transfer interrupt enable */
#define  DMA_CFGR6_TEIE                              ((uint16_t)0x0008)            /* Transfer error interrupt enable */
#define  DMA_CFGR6_DIR                               ((uint16_t)0x0010)            /* Data transfer direction */
#define  DMA_CFGR6_CIRC                              ((uint16_t)0x0020)            /* Circular mode */
#define  DMA_CFGR6_PINC                              ((uint16_t)0x0040)            /* Peripheral increment mode */
#define  DMA_CFGR6_MINC                              ((uint16_t)0x0080)            /* Memory increment mode */
        
#define  DMA_CFGR6_PSIZE                             ((uint16_t)0x0300)            /* PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CFGR6_PSIZE_0                           ((uint16_t)0x0100)            /* Bit 0 */
#define  DMA_CFGR6_PSIZE_1                           ((uint16_t)0x0200)            /* Bit 1 */
        
#define  DMA_CFGR6_MSIZE                             ((uint16_t)0x0C00)            /* MSIZE[1:0] bits (Memory size) */
#define  DMA_CFGR6_MSIZE_0                           ((uint16_t)0x0400)            /* Bit 0 */
#define  DMA_CFGR6_MSIZE_1                           ((uint16_t)0x0800)            /* Bit 1 */
        
#define  DMA_CFGR6_PL                                ((uint16_t)0x3000)            /* PL[1:0] bits (Channel Priority level) */
#define  DMA_CFGR6_PL_0                              ((uint16_t)0x1000)            /* Bit 0 */
#define  DMA_CFGR6_PL_1                              ((uint16_t)0x2000)            /* Bit 1 */
                      
#define  DMA_CFGR6_MEM2MEM                           ((uint16_t)0x4000)            /* Memory to memory mode */

/*******************  Bit definition for DMA_CFGR7 register  *******************/
#define  DMA_CFGR7_EN                                ((uint16_t)0x0001)            /* Channel enable */
#define  DMA_CFGR7_TCIE                              ((uint16_t)0x0002)            /* Transfer complete interrupt enable */
#define  DMA_CFGR7_HTIE                              ((uint16_t)0x0004)            /* Half Transfer interrupt enable */
#define  DMA_CFGR7_TEIE                              ((uint16_t)0x0008)            /* Transfer error interrupt enable */
#define  DMA_CFGR7_DIR                               ((uint16_t)0x0010)            /* Data transfer direction */
#define  DMA_CFGR7_CIRC                              ((uint16_t)0x0020)            /* Circular mode */
#define  DMA_CFGR7_PINC                              ((uint16_t)0x0040)            /* Peripheral increment mode */
#define  DMA_CFGR7_MINC                              ((uint16_t)0x0080)            /* Memory increment mode */
                      
#define  DMA_CFGR7_PSIZE                             ((uint16_t)0x0300)            /* PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CFGR7_PSIZE_0                           ((uint16_t)0x0100)            /* Bit 0 */
#define  DMA_CFGR7_PSIZE_1                           ((uint16_t)0x0200)            /* Bit 1 */
                      
#define  DMA_CFGR7_MSIZE                             ((uint16_t)0x0C00)            /* MSIZE[1:0] bits (Memory size) */
#define  DMA_CFGR7_MSIZE_0                           ((uint16_t)0x0400)            /* Bit 0 */
#define  DMA_CFGR7_MSIZE_1                           ((uint16_t)0x0800)            /* Bit 1 */
                      
#define  DMA_CFGR7_PL                                ((uint16_t)0x3000)            /* PL[1:0] bits (Channel Priority level) */
#define  DMA_CFGR7_PL_0                              ((uint16_t)0x1000)            /* Bit 0 */
#define  DMA_CFGR7_PL_1                              ((uint16_t)0x2000)            /* Bit 1 */
        
#define  DMA_CFGR7_MEM2MEM                           ((uint16_t)0x4000)            /* Memory to memory mode enable */

/*******************  Bit definition for DMA_CFGR8 register  *******************/
#define  DMA_CFGR8_EN                                ((uint16_t)0x0001)            /* Channel enable */
#define  DMA_CFGR8_TCIE                              ((uint16_t)0x0002)            /* Transfer complete interrupt enable */
#define  DMA_CFGR8_HTIE                              ((uint16_t)0x0004)            /* Half Transfer interrupt enable */
#define  DMA_CFGR8_TEIE                              ((uint16_t)0x0008)            /* Transfer error interrupt enable */
#define  DMA_CFGR8_DIR                               ((uint16_t)0x0010)            /* Data transfer direction */
#define  DMA_CFGR8_CIRC                              ((uint16_t)0x0020)            /* Circular mode */
#define  DMA_CFGR8_PINC                              ((uint16_t)0x0040)            /* Peripheral increment mode */
#define  DMA_CFGR8_MINC                              ((uint16_t)0x0080)            /* Memory increment mode */
                      
#define  DMA_CFGR8_PSIZE                             ((uint16_t)0x0300)            /* PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CFGR8_PSIZE_0                           ((uint16_t)0x0100)            /* Bit 0 */
#define  DMA_CFGR8_PSIZE_1                           ((uint16_t)0x0200)            /* Bit 1 */
                      
#define  DMA_CFGR8_MSIZE                             ((uint16_t)0x0C00)            /* MSIZE[1:0] bits (Memory size) */
#define  DMA_CFGR8_MSIZE_0                           ((uint16_t)0x0400)            /* Bit 0 */
#define  DMA_CFGR8_MSIZE_1                           ((uint16_t)0x0800)            /* Bit 1 */
                      
#define  DMA_CFGR8_PL                                ((uint16_t)0x3000)            /* PL[1:0] bits (Channel Priority level) */
#define  DMA_CFGR8_PL_0                              ((uint16_t)0x1000)            /* Bit 0 */
#define  DMA_CFGR8_PL_1                              ((uint16_t)0x2000)            /* Bit 1 */
                      
#define  DMA_CFGR8_MEM2MEM                           ((uint16_t)0x4000)            /* Memory to memory mode enable */

/******************  Bit definition for DMA_CNTR1 register  ******************/
#define  DMA_CNTR1_NDT                               ((uint16_t)0xFFFF)            /* Number of data to Transfer */

/******************  Bit definition for DMA_CNTR2 register  ******************/
#define  DMA_CNTR2_NDT                               ((uint16_t)0xFFFF)            /* Number of data to Transfer */

/******************  Bit definition for DMA_CNTR3 register  ******************/
#define  DMA_CNTR3_NDT                               ((uint16_t)0xFFFF)            /* Number of data to Transfer */

/******************  Bit definition for DMA_CNTR4 register  ******************/
#define  DMA_CNTR4_NDT                               ((uint16_t)0xFFFF)            /* Number of data to Transfer */

/******************  Bit definition for DMA_CNTR5 register  ******************/
#define  DMA_CNTR5_NDT                               ((uint16_t)0xFFFF)            /* Number of data to Transfer */

/******************  Bit definition for DMA_CNTR6 register  ******************/
#define  DMA_CNTR6_NDT                               ((uint16_t)0xFFFF)            /* Number of data to Transfer */

/******************  Bit definition for DMA_CNTR7 register  ******************/
#define  DMA_CNTR7_NDT                               ((uint16_t)0xFFFF)            /* Number of data to Transfer */

/******************  Bit definition for DMA_CNTR8 register  ******************/
#define  DMA_CNTR8_NDT                               ((uint16_t)0xFFFF)            /* Number of data to Transfer */

/******************  Bit definition for DMA_PADDR1 register  *******************/
#define  DMA_PADDR1_PA                               ((uint32_t)0xFFFFFFFF)        /* Peripheral Address */

/******************  Bit definition for DMA_PADDR2 register  *******************/
#define  DMA_PADDR2_PA                               ((uint32_t)0xFFFFFFFF)        /* Peripheral Address */

/******************  Bit definition for DMA_PADDR3 register  *******************/
#define  DMA_PADDR3_PA                               ((uint32_t)0xFFFFFFFF)        /* Peripheral Address */

/******************  Bit definition for DMA_PADDR4 register  *******************/
#define  DMA_PADDR4_PA                               ((uint32_t)0xFFFFFFFF)        /* Peripheral Address */

/******************  Bit definition for DMA_PADDR5 register  *******************/
#define  DMA_PADDR5_PA                               ((uint32_t)0xFFFFFFFF)        /* Peripheral Address */

/******************  Bit definition for DMA_PADDR6 register  *******************/
#define  DMA_PADDR6_PA                               ((uint32_t)0xFFFFFFFF)        /* Peripheral Address */

/******************  Bit definition for DMA_PADDR7 register  *******************/
#define  DMA_PADDR7_PA                               ((uint32_t)0xFFFFFFFF)        /* Peripheral Address */

/******************  Bit definition for DMA_PADDR8 register  *******************/
#define  DMA_PADDR8_PA                               ((uint32_t)0xFFFFFFFF)        /* Peripheral Address */

/******************  Bit definition for DMA_MADDR1 register  *******************/
#define  DMA_MADDR1_MA                               ((uint32_t)0xFFFFFFFF)        /* Memory Address */

/******************  Bit definition for DMA_MADDR2 register  *******************/
#define  DMA_MADDR2_MA                               ((uint32_t)0xFFFFFFFF)        /* Memory Address */

/******************  Bit definition for DMA_MADDR3 register  *******************/
#define  DMA_MADDR3_MA                               ((uint32_t)0xFFFFFFFF)        /* Memory Address */

/******************  Bit definition for DMA_MADDR4 register  *******************/
#define  DMA_MADDR4_MA                               ((uint32_t)0xFFFFFFFF)        /* Memory Address */

/******************  Bit definition for DMA_MADDR5 register  *******************/
#define  DMA_MADDR5_MA                               ((uint32_t)0xFFFFFFFF)        /* Memory Address */

/******************  Bit definition for DMA_MADDR6 register  *******************/
#define  DMA_MADDR6_MA                               ((uint32_t)0xFFFFFFFF)        /* Memory Address */

/******************  Bit definition for DMA_MADDR7 register  *******************/
#define  DMA_MADDR7_MA                               ((uint32_t)0xFFFFFFFF)        /* Memory Address */

/******************  Bit definition for DMA_MADDR8 register  *******************/
#define  DMA_MADDR8_MA                               ((uint32_t)0xFFFFFFFF)        /* Memory Address */

/******************  Bit definition for DMA_M1ADDR1 register  *******************/
#define  DMA_M1ADDR1_M1A                             ((uint32_t)0xFFFFFFFF)        /* Memory Address */

/******************  Bit definition for DMA_M1ADDR2 register  *******************/
#define  DMA_M1ADDR2_M1A                             ((uint32_t)0xFFFFFFFF)        /* Memory Address */

/******************  Bit definition for DMA_M1ADDR3 register  *******************/
#define  DMA_M1ADDR3_M1A                             ((uint32_t)0xFFFFFFFF)        /* Memory Address */

/******************  Bit definition for DMA_M1ADDR4 register  *******************/
#define  DMA_M1ADDR4_M1A                             ((uint32_t)0xFFFFFFFF)        /* Memory Address */

/******************  Bit definition for DMA_M1ADDR5 register  *******************/
#define  DMA_M1ADDR5_M1A                             ((uint32_t)0xFFFFFFFF)        /* Memory Address */

/******************  Bit definition for DMA_M1ADDR6 register  *******************/
#define  DMA_M1ADDR6_M1A                             ((uint32_t)0xFFFFFFFF)        /* Memory Address */

/******************  Bit definition for DMA_M1ADDR7 register  *******************/
#define  DMA_M1ADDR7_M1A                             ((uint32_t)0xFFFFFFFF)        /* Memory Address */

/******************  Bit definition for DMA_M1ADDR8 register  *******************/
#define  DMA_M1ADDR8_M1A                             ((uint32_t)0xFFFFFFFF)        /* Memory Address */

/******************  Bit definition for DMA_MUX0_3_CFGR register  *******************/
#define  DMA_MUX_CH0                                 ((uint32_t)0x0000007F)
#define  DMA_MUX_CH1                                 ((uint32_t)0x00007F00)
#define  DMA_MUX_CH2                                 ((uint32_t)0x007F0000)
#define  DMA_MUX_CH3                                 ((uint32_t)0x7F000000)

/******************  Bit definition for DMA_MUX4_7_CFGR register  *******************/
#define  DMA_MUX_CH4                                 ((uint32_t)0x0000007F)
#define  DMA_MUX_CH5                                 ((uint32_t)0x00007F00)
#define  DMA_MUX_CH6                                 ((uint32_t)0x007F0000)
#define  DMA_MUX_CH7                                 ((uint32_t)0x7F000000)

/******************  Bit definition for DMA_MUX8_11_CFGR register  *******************/
#define  DMA_MUX_CH8                                 ((uint32_t)0x0000007F)
#define  DMA_MUX_CH9                                 ((uint32_t)0x00007F00)
#define  DMA_MUX_CH10                                ((uint32_t)0x007F0000)
#define  DMA_MUX_CH11                                ((uint32_t)0x7F000000)

/******************  Bit definition for DMA_MUX12_15_CFGR register  *******************/
#define  DMA_MUX_CH12                                ((uint32_t)0x0000007F)
#define  DMA_MUX_CH13                                ((uint32_t)0x00007F00)
#define  DMA_MUX_CH14                                ((uint32_t)0x007F0000)
#define  DMA_MUX_CH15                                ((uint32_t)0x7F000000)

/******************************************************************************/
/*                    External Interrupt/Event Controller                     */
/******************************************************************************/

/*******************  Bit definition for EXTI_INTENR register  *******************/
#define  EXTI_INTENR_MR0                             ((uint32_t)0x00000001)        /* Interrupt Mask on line 0 */
#define  EXTI_INTENR_MR1                             ((uint32_t)0x00000002)        /* Interrupt Mask on line 1 */
#define  EXTI_INTENR_MR2                             ((uint32_t)0x00000004)        /* Interrupt Mask on line 2 */
#define  EXTI_INTENR_MR3                             ((uint32_t)0x00000008)        /* Interrupt Mask on line 3 */
#define  EXTI_INTENR_MR4                             ((uint32_t)0x00000010)        /* Interrupt Mask on line 4 */
#define  EXTI_INTENR_MR5                             ((uint32_t)0x00000020)        /* Interrupt Mask on line 5 */
#define  EXTI_INTENR_MR6                             ((uint32_t)0x00000040)        /* Interrupt Mask on line 6 */
#define  EXTI_INTENR_MR7                             ((uint32_t)0x00000080)        /* Interrupt Mask on line 7 */
#define  EXTI_INTENR_MR8                             ((uint32_t)0x00000100)        /* Interrupt Mask on line 8 */
#define  EXTI_INTENR_MR9                             ((uint32_t)0x00000200)        /* Interrupt Mask on line 9 */
#define  EXTI_INTENR_MR10                            ((uint32_t)0x00000400)        /* Interrupt Mask on line 10 */
#define  EXTI_INTENR_MR11                            ((uint32_t)0x00000800)        /* Interrupt Mask on line 11 */
#define  EXTI_INTENR_MR12                            ((uint32_t)0x00001000)        /* Interrupt Mask on line 12 */
#define  EXTI_INTENR_MR13                            ((uint32_t)0x00002000)        /* Interrupt Mask on line 13 */
#define  EXTI_INTENR_MR14                            ((uint32_t)0x00004000)        /* Interrupt Mask on line 14 */
#define  EXTI_INTENR_MR15                            ((uint32_t)0x00008000)        /* Interrupt Mask on line 15 */
#define  EXTI_INTENR_MR16                            ((uint32_t)0x00010000)        /* Interrupt Mask on line 16 */
#define  EXTI_INTENR_MR17                            ((uint32_t)0x00020000)        /* Interrupt Mask on line 17 */
#define  EXTI_INTENR_MR18                            ((uint32_t)0x00040000)        /* Interrupt Mask on line 18 */
#define  EXTI_INTENR_MR19                            ((uint32_t)0x00080000)        /* Interrupt Mask on line 19 */
#define  EXTI_INTENR_MR20                            ((uint32_t)0x00100000)        /* Interrupt Mask on line 20 */
#define  EXTI_INTENR_MR21                            ((uint32_t)0x00200000)        /* Interrupt Mask on line 21 */
#define  EXTI_INTENR_MR22                            ((uint32_t)0x00400000)        /* Interrupt Mask on line 22 */
#define  EXTI_INTENR_MR23                            ((uint32_t)0x00800000)        /* Interrupt Mask on line 23 */
#define  EXTI_INTENR_MR24                            ((uint32_t)0x01000000)        /* Interrupt Mask on line 24 */
#define  EXTI_INTENR_MR25                            ((uint32_t)0x02000000)        /* Interrupt Mask on line 25 */

/*******************  Bit definition for EXTI_EVENR register  *******************/
#define  EXTI_EVENR_MR0                              ((uint32_t)0x00000001)        /* Event Mask on line 0 */
#define  EXTI_EVENR_MR1                              ((uint32_t)0x00000002)        /* Event Mask on line 1 */
#define  EXTI_EVENR_MR2                              ((uint32_t)0x00000004)        /* Event Mask on line 2 */
#define  EXTI_EVENR_MR3                              ((uint32_t)0x00000008)        /* Event Mask on line 3 */
#define  EXTI_EVENR_MR4                              ((uint32_t)0x00000010)        /* Event Mask on line 4 */
#define  EXTI_EVENR_MR5                              ((uint32_t)0x00000020)        /* Event Mask on line 5 */
#define  EXTI_EVENR_MR6                              ((uint32_t)0x00000040)        /* Event Mask on line 6 */
#define  EXTI_EVENR_MR7                              ((uint32_t)0x00000080)        /* Event Mask on line 7 */
#define  EXTI_EVENR_MR8                              ((uint32_t)0x00000100)        /* Event Mask on line 8 */
#define  EXTI_EVENR_MR9                              ((uint32_t)0x00000200)        /* Event Mask on line 9 */
#define  EXTI_EVENR_MR10                             ((uint32_t)0x00000400)        /* Event Mask on line 10 */
#define  EXTI_EVENR_MR11                             ((uint32_t)0x00000800)        /* Event Mask on line 11 */
#define  EXTI_EVENR_MR12                             ((uint32_t)0x00001000)        /* Event Mask on line 12 */
#define  EXTI_EVENR_MR13                             ((uint32_t)0x00002000)        /* Event Mask on line 13 */
#define  EXTI_EVENR_MR14                             ((uint32_t)0x00004000)        /* Event Mask on line 14 */
#define  EXTI_EVENR_MR15                             ((uint32_t)0x00008000)        /* Event Mask on line 15 */
#define  EXTI_EVENR_MR16                             ((uint32_t)0x00010000)        /* Event Mask on line 16 */
#define  EXTI_EVENR_MR17                             ((uint32_t)0x00020000)        /* Event Mask on line 17 */
#define  EXTI_EVENR_MR18                             ((uint32_t)0x00040000)        /* Event Mask on line 18 */
#define  EXTI_EVENR_MR19                             ((uint32_t)0x00080000)        /* Event Mask on line 19 */
#define  EXTI_EVENR_MR20                             ((uint32_t)0x00100000)        /* Event Mask on line 20 */
#define  EXTI_EVENR_MR21                             ((uint32_t)0x00200000)        /* Event Mask on line 21 */
#define  EXTI_EVENR_MR22                             ((uint32_t)0x00400000)        /* Event Mask on line 22 */
#define  EXTI_EVENR_MR23                             ((uint32_t)0x00800000)        /* Event Mask on line 23 */
#define  EXTI_EVENR_MR24                             ((uint32_t)0x01000000)        /* Event Mask on line 24 */
#define  EXTI_EVENR_MR25                             ((uint32_t)0x02000000)        /* Event Mask on line 25 */

/******************  Bit definition for EXTI_RTENR register  *******************/
#define  EXTI_RTENR_TR0                              ((uint32_t)0x00000001)        /* Rising trigger event configuration bit of line 0 */
#define  EXTI_RTENR_TR1                              ((uint32_t)0x00000002)        /* Rising trigger event configuration bit of line 1 */
#define  EXTI_RTENR_TR2                              ((uint32_t)0x00000004)        /* Rising trigger event configuration bit of line 2 */
#define  EXTI_RTENR_TR3                              ((uint32_t)0x00000008)        /* Rising trigger event configuration bit of line 3 */
#define  EXTI_RTENR_TR4                              ((uint32_t)0x00000010)        /* Rising trigger event configuration bit of line 4 */
#define  EXTI_RTENR_TR5                              ((uint32_t)0x00000020)        /* Rising trigger event configuration bit of line 5 */
#define  EXTI_RTENR_TR6                              ((uint32_t)0x00000040)        /* Rising trigger event configuration bit of line 6 */
#define  EXTI_RTENR_TR7                              ((uint32_t)0x00000080)        /* Rising trigger event configuration bit of line 7 */
#define  EXTI_RTENR_TR8                              ((uint32_t)0x00000100)        /* Rising trigger event configuration bit of line 8 */
#define  EXTI_RTENR_TR9                              ((uint32_t)0x00000200)        /* Rising trigger event configuration bit of line 9 */
#define  EXTI_RTENR_TR10                             ((uint32_t)0x00000400)        /* Rising trigger event configuration bit of line 10 */
#define  EXTI_RTENR_TR11                             ((uint32_t)0x00000800)        /* Rising trigger event configuration bit of line 11 */
#define  EXTI_RTENR_TR12                             ((uint32_t)0x00001000)        /* Rising trigger event configuration bit of line 12 */
#define  EXTI_RTENR_TR13                             ((uint32_t)0x00002000)        /* Rising trigger event configuration bit of line 13 */
#define  EXTI_RTENR_TR14                             ((uint32_t)0x00004000)        /* Rising trigger event configuration bit of line 14 */
#define  EXTI_RTENR_TR15                             ((uint32_t)0x00008000)        /* Rising trigger event configuration bit of line 15 */
#define  EXTI_RTENR_TR16                             ((uint32_t)0x00010000)        /* Rising trigger event configuration bit of line 16 */
#define  EXTI_RTENR_TR17                             ((uint32_t)0x00020000)        /* Rising trigger event configuration bit of line 17 */
#define  EXTI_RTENR_TR18                             ((uint32_t)0x00040000)        /* Rising trigger event configuration bit of line 18 */
#define  EXTI_RTENR_TR19                             ((uint32_t)0x00080000)        /* Rising trigger event configuration bit of line 19 */
#define  EXTI_RTENR_TR20                             ((uint32_t)0x00100000)        /* Rising trigger event configuration bit of line 20 */
#define  EXTI_RTENR_TR21                             ((uint32_t)0x00200000)        /* Rising trigger event configuration bit of line 21 */
#define  EXTI_RTENR_TR22                             ((uint32_t)0x00400000)        /* Rising trigger event configuration bit of line 22 */
#define  EXTI_RTENR_TR23                             ((uint32_t)0x00800000)        /* Rising trigger event configuration bit of line 23 */
#define  EXTI_RTENR_TR24                             ((uint32_t)0x01000000)        /* Rising trigger event configuration bit of line 24 */
#define  EXTI_RTENR_TR25                             ((uint32_t)0x02000000)        /* Rising trigger event configuration bit of line 25 */

/******************  Bit definition for EXTI_FTENR register  *******************/
#define  EXTI_FTENR_TR0                              ((uint32_t)0x00000001)        /* Falling trigger event configuration bit of line 0 */
#define  EXTI_FTENR_TR1                              ((uint32_t)0x00000002)        /* Falling trigger event configuration bit of line 1 */
#define  EXTI_FTENR_TR2                              ((uint32_t)0x00000004)        /* Falling trigger event configuration bit of line 2 */
#define  EXTI_FTENR_TR3                              ((uint32_t)0x00000008)        /* Falling trigger event configuration bit of line 3 */
#define  EXTI_FTENR_TR4                              ((uint32_t)0x00000010)        /* Falling trigger event configuration bit of line 4 */
#define  EXTI_FTENR_TR5                              ((uint32_t)0x00000020)        /* Falling trigger event configuration bit of line 5 */
#define  EXTI_FTENR_TR6                              ((uint32_t)0x00000040)        /* Falling trigger event configuration bit of line 6 */
#define  EXTI_FTENR_TR7                              ((uint32_t)0x00000080)        /* Falling trigger event configuration bit of line 7 */
#define  EXTI_FTENR_TR8                              ((uint32_t)0x00000100)        /* Falling trigger event configuration bit of line 8 */
#define  EXTI_FTENR_TR9                              ((uint32_t)0x00000200)        /* Falling trigger event configuration bit of line 9 */
#define  EXTI_FTENR_TR10                             ((uint32_t)0x00000400)        /* Falling trigger event configuration bit of line 10 */
#define  EXTI_FTENR_TR11                             ((uint32_t)0x00000800)        /* Falling trigger event configuration bit of line 11 */
#define  EXTI_FTENR_TR12                             ((uint32_t)0x00001000)        /* Falling trigger event configuration bit of line 12 */
#define  EXTI_FTENR_TR13                             ((uint32_t)0x00002000)        /* Falling trigger event configuration bit of line 13 */
#define  EXTI_FTENR_TR14                             ((uint32_t)0x00004000)        /* Falling trigger event configuration bit of line 14 */
#define  EXTI_FTENR_TR15                             ((uint32_t)0x00008000)        /* Falling trigger event configuration bit of line 15 */
#define  EXTI_FTENR_TR16                             ((uint32_t)0x00010000)        /* Falling trigger event configuration bit of line 16 */
#define  EXTI_FTENR_TR17                             ((uint32_t)0x00020000)        /* Falling trigger event configuration bit of line 17 */
#define  EXTI_FTENR_TR18                             ((uint32_t)0x00040000)        /* Falling trigger event configuration bit of line 18 */
#define  EXTI_FTENR_TR19                             ((uint32_t)0x00080000)        /* Falling trigger event configuration bit of line 19 */
#define  EXTI_FTENR_TR20                             ((uint32_t)0x00100000)        /* Falling trigger event configuration bit of line 20 */
#define  EXTI_FTENR_TR21                             ((uint32_t)0x00200000)        /* Falling trigger event configuration bit of line 21 */
#define  EXTI_FTENR_TR22                             ((uint32_t)0x00400000)        /* Falling trigger event configuration bit of line 22 */
#define  EXTI_FTENR_TR23                             ((uint32_t)0x00800000)        /* Falling trigger event configuration bit of line 23 */
#define  EXTI_FTENR_TR24                             ((uint32_t)0x01000000)        /* Falling trigger event configuration bit of line 24 */
#define  EXTI_FTENR_TR25                             ((uint32_t)0x02000000)        /* Falling trigger event configuration bit of line 25 */

/******************  Bit definition for EXTI_SWIEVR register  ******************/
#define  EXTI_SWIEVR_SWIEVR0                         ((uint32_t)0x00000001)        /* Software Interrupt on line 0 */
#define  EXTI_SWIEVR_SWIEVR1                         ((uint32_t)0x00000002)        /* Software Interrupt on line 1 */
#define  EXTI_SWIEVR_SWIEVR2                         ((uint32_t)0x00000004)        /* Software Interrupt on line 2 */
#define  EXTI_SWIEVR_SWIEVR3                         ((uint32_t)0x00000008)        /* Software Interrupt on line 3 */
#define  EXTI_SWIEVR_SWIEVR4                         ((uint32_t)0x00000010)        /* Software Interrupt on line 4 */
#define  EXTI_SWIEVR_SWIEVR5                         ((uint32_t)0x00000020)        /* Software Interrupt on line 5 */
#define  EXTI_SWIEVR_SWIEVR6                         ((uint32_t)0x00000040)        /* Software Interrupt on line 6 */
#define  EXTI_SWIEVR_SWIEVR7                         ((uint32_t)0x00000080)        /* Software Interrupt on line 7 */
#define  EXTI_SWIEVR_SWIEVR8                         ((uint32_t)0x00000100)        /* Software Interrupt on line 8 */
#define  EXTI_SWIEVR_SWIEVR9                         ((uint32_t)0x00000200)        /* Software Interrupt on line 9 */
#define  EXTI_SWIEVR_SWIEVR10                        ((uint32_t)0x00000400)        /* Software Interrupt on line 10 */
#define  EXTI_SWIEVR_SWIEVR11                        ((uint32_t)0x00000800)        /* Software Interrupt on line 11 */
#define  EXTI_SWIEVR_SWIEVR12                        ((uint32_t)0x00001000)        /* Software Interrupt on line 12 */
#define  EXTI_SWIEVR_SWIEVR13                        ((uint32_t)0x00002000)        /* Software Interrupt on line 13 */
#define  EXTI_SWIEVR_SWIEVR14                        ((uint32_t)0x00004000)        /* Software Interrupt on line 14 */
#define  EXTI_SWIEVR_SWIEVR15                        ((uint32_t)0x00008000)        /* Software Interrupt on line 15 */
#define  EXTI_SWIEVR_SWIEVR16                        ((uint32_t)0x00010000)        /* Software Interrupt on line 16 */
#define  EXTI_SWIEVR_SWIEVR17                        ((uint32_t)0x00020000)        /* Software Interrupt on line 17 */
#define  EXTI_SWIEVR_SWIEVR18                        ((uint32_t)0x00040000)        /* Software Interrupt on line 18 */
#define  EXTI_SWIEVR_SWIEVR19                        ((uint32_t)0x00080000)        /* Software Interrupt on line 19 */
#define  EXTI_SWIEVR_SWIEVR20                        ((uint32_t)0x00100000)        /* Software Interrupt on line 20 */
#define  EXTI_SWIEVR_SWIEVR21                        ((uint32_t)0x00200000)        /* Software Interrupt on line 21 */
#define  EXTI_SWIEVR_SWIEVR22                        ((uint32_t)0x00400000)        /* Software Interrupt on line 22 */
#define  EXTI_SWIEVR_SWIEVR23                        ((uint32_t)0x00800000)        /* Software Interrupt on line 23 */
#define  EXTI_SWIEVR_SWIEVR24                        ((uint32_t)0x01000000)        /* Software Interrupt on line 24 */
#define  EXTI_SWIEVR_SWIEVR25                        ((uint32_t)0x02000000)        /* Software Interrupt on line 25 */

/*******************  Bit definition for EXTI_INTFR register  ********************/
#define  EXTI_INTF_INTF0                             ((uint32_t)0x00000001)        /* Pending bit for line 0 */
#define  EXTI_INTF_INTF1                             ((uint32_t)0x00000002)        /* Pending bit for line 1 */
#define  EXTI_INTF_INTF2                             ((uint32_t)0x00000004)        /* Pending bit for line 2 */
#define  EXTI_INTF_INTF3                             ((uint32_t)0x00000008)        /* Pending bit for line 3 */
#define  EXTI_INTF_INTF4                             ((uint32_t)0x00000010)        /* Pending bit for line 4 */
#define  EXTI_INTF_INTF5                             ((uint32_t)0x00000020)        /* Pending bit for line 5 */
#define  EXTI_INTF_INTF6                             ((uint32_t)0x00000040)        /* Pending bit for line 6 */
#define  EXTI_INTF_INTF7                             ((uint32_t)0x00000080)        /* Pending bit for line 7 */
#define  EXTI_INTF_INTF8                             ((uint32_t)0x00000100)        /* Pending bit for line 8 */
#define  EXTI_INTF_INTF9                             ((uint32_t)0x00000200)        /* Pending bit for line 9 */
#define  EXTI_INTF_INTF10                            ((uint32_t)0x00000400)        /* Pending bit for line 10 */
#define  EXTI_INTF_INTF11                            ((uint32_t)0x00000800)        /* Pending bit for line 11 */
#define  EXTI_INTF_INTF12                            ((uint32_t)0x00001000)        /* Pending bit for line 12 */
#define  EXTI_INTF_INTF13                            ((uint32_t)0x00002000)        /* Pending bit for line 13 */
#define  EXTI_INTF_INTF14                            ((uint32_t)0x00004000)        /* Pending bit for line 14 */
#define  EXTI_INTF_INTF15                            ((uint32_t)0x00008000)        /* Pending bit for line 15 */
#define  EXTI_INTF_INTF16                            ((uint32_t)0x00010000)        /* Pending bit for line 16 */
#define  EXTI_INTF_INTF17                            ((uint32_t)0x00020000)        /* Pending bit for line 17 */
#define  EXTI_INTF_INTF18                            ((uint32_t)0x00040000)        /* Pending bit for line 18 */
#define  EXTI_INTF_INTF19                            ((uint32_t)0x00080000)        /* Pending bit for line 19 */
#define  EXTI_INTF_INTF20                            ((uint32_t)0x00100000)        /* Pending bit for line 20 */
#define  EXTI_INTF_INTF21                            ((uint32_t)0x00200000)        /* Pending bit for line 21 */
#define  EXTI_INTF_INTF22                            ((uint32_t)0x00400000)        /* Pending bit for line 22 */
#define  EXTI_INTF_INTF23                            ((uint32_t)0x00800000)        /* Pending bit for line 23 */
#define  EXTI_INTF_INTF24                            ((uint32_t)0x01000000)        /* Pending bit for line 24 */
#define  EXTI_INTF_INTF25                            ((uint32_t)0x02000000)        /* Pending bit for line 25 */

/******************************************************************************/
/*                      FLASH and Option Bytes Registers                      */
/******************************************************************************/

/*******************  Bit definition for FLASH_ACTLR register  ******************/

/******************  Bit definition for FLASH_ACTLR register  ******************/
#define  FLASH_ACTLR_SCK_CFG                         ((uint32_t)0x00000003)
#define  FLASH_ACTLR_SCK_CFG_0                       ((uint32_t)0x00000001)
#define  FLASH_ACTLR_SCK_CFG_1                       ((uint32_t)0x00000002)

#define  FLASH_ACTLR_LATENCY_HCLK_DIV1               ((uint32_t)0x00000000)
#define  FLASH_ACTLR_LATENCY_HCLK_DIV2               ((uint32_t)0x00000001)
#define  FLASH_ACTLR_LATENCY_HCLK_DIV4               ((uint32_t)0x00000002)
#define  FLASH_ACTLR_LATENCY_HCLK_DIV8               ((uint32_t)0x00000003)

#define  FLASH_ACTLR_ENHANCE_STATUS                  ((uint32_t)0x00000040)
#define  FLASH_ACTLR_EHMOD                           ((uint32_t)0x00000080)
      
#define  FLASH_ACTLR_LP                              ((uint32_t)0x00000100)
#define  FLASH_ACTLR_READY                           ((uint32_t)0x00004000)
#define  FLASH_ACTLR_ST                              ((uint32_t)0x00008000)

/******************  Bit definition for FLASH_KEYR register  ******************/
#define  FLASH_KEYR_FKEYR                            ((uint32_t)0xFFFFFFFF)        /* FPEC Key */
#define  FLASH_KEYR_KEY1                             ((uint32_t)0x45670123)
#define  FLASH_KEYR_KEY2                             ((uint32_t)0xCDEF89AB)

/*****************  Bit definition for FLASH_OBKEYR register  ****************/
#define  FLASH_OBKEYR_OBKEYR                         ((uint32_t)0xFFFFFFFF)        /* Option Byte Key */

/******************  Bit definition for FLASH_STATR register  *******************/
#define  FLASH_STATR_BSY                             ((uint8_t)0x01)               /* Busy */
#define  FLASH_STATR_WRBSY                           ((uint8_t)0x02)               
#define  FLASH_STATR_WRPRTERR                        ((uint8_t)0x10)               /* Write Protection Error */
#define  FLASH_STATR_EOP                             ((uint8_t)0x20)               /* End of operation */

#define  FLASH_STATR_BOOT_AVA                        ((uint16_t)0x1000)
#define  FLASH_STATR_BOOT_STATUS                     ((uint16_t)0x2000)
#define  FLASH_STATR_BOOT_MODE                       ((uint16_t)0x4000)
#define  FLASH_STATR_BOOT_LOCK                       ((uint16_t)0x8000)

/*******************  Bit definition for FLASH_CTLR register  *******************/
#define  FLASH_CTLR_PG                               ((uint32_t)0x00000001)        /* Programming */
#define  FLASH_CTLR_PER                              ((uint32_t)0x00000002)        /* Sector Erase 4K */
#define  FLASH_CTLR_OPTPG                            ((uint32_t)0x00000010)        /* Option Byte Programming */
#define  FLASH_CTLR_OPTER                            ((uint32_t)0x00000020)        /* Option Byte Erase */
#define  FLASH_CTLR_STRT                             ((uint32_t)0x00000040)        /* Start */
#define  FLASH_CTLR_LOCK                             ((uint32_t)0x00000080)        /* Lock */
#define  FLASH_CTLR_OPTWRE                           ((uint32_t)0x00000200)        /* Option Bytes Write Enable */
#define  FLASH_CTLR_ERRIE                            ((uint32_t)0x00000400)        /* Error Interrupt Enable */
#define  FLASH_CTLR_EOPIE                            ((uint32_t)0x00001000)        /* End of operation interrupt enable */
#define  FLASH_CTLR_FAST_LOCK                        ((uint32_t)0x00008000)        /* Fast Lock */
#define  FLASH_CTLR_PAGE_PG                          ((uint32_t)0x00010000)        /* Page Programming 256Byte */
#define  FLASH_CTLR_PAGE_BER32                       ((uint32_t)0x00040000)        /* Block Erase 32K */
#define  FLASH_CTLR_PG_STRT                          ((uint32_t)0x00200000)        /* Page Programming Start */
#define  FLASH_CTLR_RSENACT                          ((uint32_t)0x00400000)

/*******************  Bit definition for FLASH_ADDR register  *******************/
#define  FLASH_ADDR_FAR                              ((uint32_t)0xFFFFFFFF)        /* Flash Address */

/******************  Bit definition for FLASH_OBR register  *******************/
#define  FLASH_OBR_OPTERR                            ((uint16_t)0x0001)            /* Option Byte Error */
#define  FLASH_OBR_RDPRT                             ((uint16_t)0x0002)            /* Read protection */

#define  FLASH_OBR_USER                              ((uint16_t)0x0304)            /* User Option Bytes */
#define  FLASH_OBR_WDG_SW                            ((uint16_t)0x0004)            /* WDG_SW */
#define  FLASH_OBR_FIX_11                            ((uint16_t)0x0300)            /* nRST_STOP */

#define  FLASH_OBR_DATA0                             ((uint32_t)0x0003FC00)
#define  FLASH_OBR_DATA1                             ((uint32_t)0x03FC0000)

/******************  Bit definition for FLASH_WPR register  ******************/
#define  FLASH_WPR_WRP                               ((uint32_t)0xFFFFFFFF)        /* Write Protect */

/******************  Bit definition for FLASH_MODEKEYR register  ******************/
#define  FLASH_MODEKEYR_KEY1                         ((uint32_t)0x45670123)        
#define  FLASH_MODEKEYR_KEY2                         ((uint32_t)0xCDEF89AB)        

/******************  Bit definition for FLASH_RDPR register  *******************/
#define  FLASH_RDPR_RDPR                             ((uint32_t)0x000000FF)        /* Read protection option byte */
#define  FLASH_RDPR_nRDPR                            ((uint32_t)0x0000FF00)        /* Read protection complemented option byte */

/******************  Bit definition for FLASH_USER register  ******************/
#define  FLASH_USER_USER                             ((uint32_t)0x00FF0000)        /* User option byte */
#define  FLASH_USER_nUSER                            ((uint32_t)0xFF000000)        /* User complemented option byte */

/******************  Bit definition for FLASH_Data0 register  *****************/
#define  FLASH_Data0_Data0                           ((uint32_t)0x000000FF)        /* User data storage option byte */
#define  FLASH_Data0_nData0                          ((uint32_t)0x0000FF00)        /* User data storage complemented option byte */

/******************  Bit definition for FLASH_Data1 register  *****************/
#define  FLASH_Data1_Data1                           ((uint32_t)0x00FF0000)        /* User data storage option byte */
#define  FLASH_Data1_nData1                          ((uint32_t)0xFF000000)        /* User data storage complemented option byte */

/******************  Bit definition for FLASH_WRPR0 register  ******************/
#define  FLASH_WRPR0_WRPR0                           ((uint32_t)0x000000FF)        /* Flash memory write protection option bytes */
#define  FLASH_WRPR0_nWRPR0                          ((uint32_t)0x0000FF00)        /* Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRPR1 register  ******************/
#define  FLASH_WRPR1_WRPR1                           ((uint32_t)0x00FF0000)        /* Flash memory write protection option bytes */
#define  FLASH_WRPR1_nWRPR1                          ((uint32_t)0xFF000000)        /* Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRPR2 register  ******************/
#define  FLASH_WRPR2_WRPR2                           ((uint32_t)0x000000FF)        /* Flash memory write protection option bytes */
#define  FLASH_WRPR2_nWRPR2                          ((uint32_t)0x0000FF00)        /* Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRPR3 register  ******************/
#define  FLASH_WRPR3_WRPR3                           ((uint32_t)0x00FF0000)        /* Flash memory write protection option bytes */
#define  FLASH_WRPR3_nWRPR3                          ((uint32_t)0xFF000000)        /* Flash memory write protection complemented option bytes */

/******************************************************************************/
/*                General Purpose and Alternate Function I/O                  */
/******************************************************************************/

/*******************  Bit definition for GPIO_CFGLR register  *******************/
#define  GPIO_CFGLR_MODE                             ((uint32_t)0x33333333)        /* Port x mode bits */
      
#define  GPIO_CFGLR_MODE0                            ((uint32_t)0x00000003)        /* MODE0[1:0] bits (Port x mode bits, pin 0) */
#define  GPIO_CFGLR_MODE0_0                          ((uint32_t)0x00000001)        /* Bit 0 */
#define  GPIO_CFGLR_MODE0_1                          ((uint32_t)0x00000002)        /* Bit 1 */

#define  GPIO_CFGLR_MODE1                            ((uint32_t)0x00000030)        /* MODE1[1:0] bits (Port x mode bits, pin 1) */
#define  GPIO_CFGLR_MODE1_0                          ((uint32_t)0x00000010)        /* Bit 0 */
#define  GPIO_CFGLR_MODE1_1                          ((uint32_t)0x00000020)        /* Bit 1 */

#define  GPIO_CFGLR_MODE2                            ((uint32_t)0x00000300)        /* MODE2[1:0] bits (Port x mode bits, pin 2) */
#define  GPIO_CFGLR_MODE2_0                          ((uint32_t)0x00000100)        /* Bit 0 */
#define  GPIO_CFGLR_MODE2_1                          ((uint32_t)0x00000200)        /* Bit 1 */

#define  GPIO_CFGLR_MODE3                            ((uint32_t)0x00003000)        /* MODE3[1:0] bits (Port x mode bits, pin 3) */
#define  GPIO_CFGLR_MODE3_0                          ((uint32_t)0x00001000)        /* Bit 0 */
#define  GPIO_CFGLR_MODE3_1                          ((uint32_t)0x00002000)        /* Bit 1 */

#define  GPIO_CFGLR_MODE4                            ((uint32_t)0x00030000)        /* MODE4[1:0] bits (Port x mode bits, pin 4) */
#define  GPIO_CFGLR_MODE4_0                          ((uint32_t)0x00010000)        /* Bit 0 */
#define  GPIO_CFGLR_MODE4_1                          ((uint32_t)0x00020000)        /* Bit 1 */
      
#define  GPIO_CFGLR_MODE5                            ((uint32_t)0x00300000)        /* MODE5[1:0] bits (Port x mode bits, pin 5) */
#define  GPIO_CFGLR_MODE5_0                          ((uint32_t)0x00100000)        /* Bit 0 */
#define  GPIO_CFGLR_MODE5_1                          ((uint32_t)0x00200000)        /* Bit 1 */
      
#define  GPIO_CFGLR_MODE6                            ((uint32_t)0x03000000)        /* MODE6[1:0] bits (Port x mode bits, pin 6) */
#define  GPIO_CFGLR_MODE6_0                          ((uint32_t)0x01000000)        /* Bit 0 */
#define  GPIO_CFGLR_MODE6_1                          ((uint32_t)0x02000000)        /* Bit 1 */

#define  GPIO_CFGLR_MODE7                            ((uint32_t)0x30000000)        /* MODE7[1:0] bits (Port x mode bits, pin 7) */
#define  GPIO_CFGLR_MODE7_0                          ((uint32_t)0x10000000)        /* Bit 0 */
#define  GPIO_CFGLR_MODE7_1                          ((uint32_t)0x20000000)        /* Bit 1 */

#define  GPIO_CFGLR_CNF                              ((uint32_t)0xCCCCCCCC)        /* Port x configuration bits */

#define  GPIO_CFGLR_CNF0                             ((uint32_t)0x0000000C)        /* CNF0[1:0] bits (Port x configuration bits, pin 0) */
#define  GPIO_CFGLR_CNF0_0                           ((uint32_t)0x00000004)        /* Bit 0 */
#define  GPIO_CFGLR_CNF0_1                           ((uint32_t)0x00000008)        /* Bit 1 */
      
#define  GPIO_CFGLR_CNF1                             ((uint32_t)0x000000C0)        /* CNF1[1:0] bits (Port x configuration bits, pin 1) */
#define  GPIO_CFGLR_CNF1_0                           ((uint32_t)0x00000040)        /* Bit 0 */
#define  GPIO_CFGLR_CNF1_1                           ((uint32_t)0x00000080)        /* Bit 1 */

#define  GPIO_CFGLR_CNF2                             ((uint32_t)0x00000C00)        /* CNF2[1:0] bits (Port x configuration bits, pin 2) */
#define  GPIO_CFGLR_CNF2_0                           ((uint32_t)0x00000400)        /* Bit 0 */
#define  GPIO_CFGLR_CNF2_1                           ((uint32_t)0x00000800)        /* Bit 1 */

#define  GPIO_CFGLR_CNF3                             ((uint32_t)0x0000C000)        /* CNF3[1:0] bits (Port x configuration bits, pin 3) */
#define  GPIO_CFGLR_CNF3_0                           ((uint32_t)0x00004000)        /* Bit 0 */
#define  GPIO_CFGLR_CNF3_1                           ((uint32_t)0x00008000)        /* Bit 1 */

#define  GPIO_CFGLR_CNF4                             ((uint32_t)0x000C0000)        /* CNF4[1:0] bits (Port x configuration bits, pin 4) */
#define  GPIO_CFGLR_CNF4_0                           ((uint32_t)0x00040000)        /* Bit 0 */
#define  GPIO_CFGLR_CNF4_1                           ((uint32_t)0x00080000)        /* Bit 1 */

#define  GPIO_CFGLR_CNF5                             ((uint32_t)0x00C00000)        /* CNF5[1:0] bits (Port x configuration bits, pin 5) */
#define  GPIO_CFGLR_CNF5_0                           ((uint32_t)0x00400000)        /* Bit 0 */
#define  GPIO_CFGLR_CNF5_1                           ((uint32_t)0x00800000)        /* Bit 1 */

#define  GPIO_CFGLR_CNF6                             ((uint32_t)0x0C000000)        /* CNF6[1:0] bits (Port x configuration bits, pin 6) */
#define  GPIO_CFGLR_CNF6_0                           ((uint32_t)0x04000000)        /* Bit 0 */
#define  GPIO_CFGLR_CNF6_1                           ((uint32_t)0x08000000)        /* Bit 1 */

#define  GPIO_CFGLR_CNF7                             ((uint32_t)0xC0000000)        /* CNF7[1:0] bits (Port x configuration bits, pin 7) */
#define  GPIO_CFGLR_CNF7_0                           ((uint32_t)0x40000000)        /* Bit 0 */
#define  GPIO_CFGLR_CNF7_1                           ((uint32_t)0x80000000)        /* Bit 1 */

/*******************  Bit definition for GPIO_CFGHR register  *******************/
#define  GPIO_CFGHR_MODE                             ((uint32_t)0x33333333)        /* Port x mode bits */

#define  GPIO_CFGHR_MODE8                            ((uint32_t)0x00000003)        /* MODE8[1:0] bits (Port x mode bits, pin 8) */
#define  GPIO_CFGHR_MODE8_0                          ((uint32_t)0x00000001)        /* Bit 0 */
#define  GPIO_CFGHR_MODE8_1                          ((uint32_t)0x00000002)        /* Bit 1 */
      
#define  GPIO_CFGHR_MODE9                            ((uint32_t)0x00000030)        /* MODE9[1:0] bits (Port x mode bits, pin 9) */
#define  GPIO_CFGHR_MODE9_0                          ((uint32_t)0x00000010)        /* Bit 0 */
#define  GPIO_CFGHR_MODE9_1                          ((uint32_t)0x00000020)        /* Bit 1 */

#define  GPIO_CFGHR_MODE10                           ((uint32_t)0x00000300)        /* MODE10[1:0] bits (Port x mode bits, pin 10) */
#define  GPIO_CFGHR_MODE10_0                         ((uint32_t)0x00000100)        /* Bit 0 */
#define  GPIO_CFGHR_MODE10_1                         ((uint32_t)0x00000200)        /* Bit 1 */

#define  GPIO_CFGHR_MODE11                           ((uint32_t)0x00003000)        /* MODE11[1:0] bits (Port x mode bits, pin 11) */
#define  GPIO_CFGHR_MODE11_0                         ((uint32_t)0x00001000)        /* Bit 0 */
#define  GPIO_CFGHR_MODE11_1                         ((uint32_t)0x00002000)        /* Bit 1 */

#define  GPIO_CFGHR_MODE12                           ((uint32_t)0x00030000)        /* MODE12[1:0] bits (Port x mode bits, pin 12) */
#define  GPIO_CFGHR_MODE12_0                         ((uint32_t)0x00010000)        /* Bit 0 */
#define  GPIO_CFGHR_MODE12_1                         ((uint32_t)0x00020000)        /* Bit 1 */
      
#define  GPIO_CFGHR_MODE13                           ((uint32_t)0x00300000)        /* MODE13[1:0] bits (Port x mode bits, pin 13) */
#define  GPIO_CFGHR_MODE13_0                         ((uint32_t)0x00100000)        /* Bit 0 */
#define  GPIO_CFGHR_MODE13_1                         ((uint32_t)0x00200000)        /* Bit 1 */

#define  GPIO_CFGHR_MODE14                           ((uint32_t)0x03000000)        /* MODE14[1:0] bits (Port x mode bits, pin 14) */
#define  GPIO_CFGHR_MODE14_0                         ((uint32_t)0x01000000)        /* Bit 0 */
#define  GPIO_CFGHR_MODE14_1                         ((uint32_t)0x02000000)        /* Bit 1 */

#define  GPIO_CFGHR_MODE15                           ((uint32_t)0x30000000)        /* MODE15[1:0] bits (Port x mode bits, pin 15) */
#define  GPIO_CFGHR_MODE15_0                         ((uint32_t)0x10000000)        /* Bit 0 */
#define  GPIO_CFGHR_MODE15_1                         ((uint32_t)0x20000000)        /* Bit 1 */

#define  GPIO_CFGHR_CNF                              ((uint32_t)0xCCCCCCCC)        /* Port x configuration bits */

#define  GPIO_CFGHR_CNF8                             ((uint32_t)0x0000000C)        /* CNF8[1:0] bits (Port x configuration bits, pin 8) */
#define  GPIO_CFGHR_CNF8_0                           ((uint32_t)0x00000004)        /* Bit 0 */
#define  GPIO_CFGHR_CNF8_1                           ((uint32_t)0x00000008)        /* Bit 1 */

#define  GPIO_CFGHR_CNF9                             ((uint32_t)0x000000C0)        /* CNF9[1:0] bits (Port x configuration bits, pin 9) */
#define  GPIO_CFGHR_CNF9_0                           ((uint32_t)0x00000040)        /* Bit 0 */
#define  GPIO_CFGHR_CNF9_1                           ((uint32_t)0x00000080)        /* Bit 1 */

#define  GPIO_CFGHR_CNF10                            ((uint32_t)0x00000C00)        /* CNF10[1:0] bits (Port x configuration bits, pin 10) */
#define  GPIO_CFGHR_CNF10_0                          ((uint32_t)0x00000400)        /* Bit 0 */
#define  GPIO_CFGHR_CNF10_1                          ((uint32_t)0x00000800)        /* Bit 1 */
            
#define  GPIO_CFGHR_CNF11                            ((uint32_t)0x0000C000)        /* CNF11[1:0] bits (Port x configuration bits, pin 11) */
#define  GPIO_CFGHR_CNF11_0                          ((uint32_t)0x00004000)        /* Bit 0 */
#define  GPIO_CFGHR_CNF11_1                          ((uint32_t)0x00008000)        /* Bit 1 */

#define  GPIO_CFGHR_CNF12                            ((uint32_t)0x000C0000)        /* CNF12[1:0] bits (Port x configuration bits, pin 12) */
#define  GPIO_CFGHR_CNF12_0                          ((uint32_t)0x00040000)        /* Bit 0 */
#define  GPIO_CFGHR_CNF12_1                          ((uint32_t)0x00080000)        /* Bit 1 */

#define  GPIO_CFGHR_CNF13                            ((uint32_t)0x00C00000)        /* CNF13[1:0] bits (Port x configuration bits, pin 13) */
#define  GPIO_CFGHR_CNF13_0                          ((uint32_t)0x00400000)        /* Bit 0 */
#define  GPIO_CFGHR_CNF13_1                          ((uint32_t)0x00800000)        /* Bit 1 */
      
#define  GPIO_CFGHR_CNF14                            ((uint32_t)0x0C000000)        /* CNF14[1:0] bits (Port x configuration bits, pin 14) */
#define  GPIO_CFGHR_CNF14_0                          ((uint32_t)0x04000000)        /* Bit 0 */
#define  GPIO_CFGHR_CNF14_1                          ((uint32_t)0x08000000)        /* Bit 1 */

#define  GPIO_CFGHR_CNF15                            ((uint32_t)0xC0000000)        /* CNF15[1:0] bits (Port x configuration bits, pin 15) */
#define  GPIO_CFGHR_CNF15_0                          ((uint32_t)0x40000000)        /* Bit 0 */
#define  GPIO_CFGHR_CNF15_1                          ((uint32_t)0x80000000)        /* Bit 1 */

/*******************  Bit definition for GPIO_INDR register  *******************/
#define GPIO_INDR_IDR0                               ((uint16_t)0x0001)            /* Port input data, bit 0 */
#define GPIO_INDR_IDR1                               ((uint16_t)0x0002)            /* Port input data, bit 1 */
#define GPIO_INDR_IDR2                               ((uint16_t)0x0004)            /* Port input data, bit 2 */
#define GPIO_INDR_IDR3                               ((uint16_t)0x0008)            /* Port input data, bit 3 */
#define GPIO_INDR_IDR4                               ((uint16_t)0x0010)            /* Port input data, bit 4 */
#define GPIO_INDR_IDR5                               ((uint16_t)0x0020)            /* Port input data, bit 5 */
#define GPIO_INDR_IDR6                               ((uint16_t)0x0040)            /* Port input data, bit 6 */
#define GPIO_INDR_IDR7                               ((uint16_t)0x0080)            /* Port input data, bit 7 */
#define GPIO_INDR_IDR8                               ((uint16_t)0x0100)            /* Port input data, bit 8 */
#define GPIO_INDR_IDR9                               ((uint16_t)0x0200)            /* Port input data, bit 9 */
#define GPIO_INDR_IDR10                              ((uint16_t)0x0400)            /* Port input data, bit 10 */
#define GPIO_INDR_IDR11                              ((uint16_t)0x0800)            /* Port input data, bit 11 */
#define GPIO_INDR_IDR12                              ((uint16_t)0x1000)            /* Port input data, bit 12 */
#define GPIO_INDR_IDR13                              ((uint16_t)0x2000)            /* Port input data, bit 13 */
#define GPIO_INDR_IDR14                              ((uint16_t)0x4000)            /* Port input data, bit 14 */
#define GPIO_INDR_IDR15                              ((uint16_t)0x8000)            /* Port input data, bit 15 */

/*******************  Bit definition for GPIO_OUTDR register  *******************/
#define GPIO_OUTDR_ODR0                              ((uint16_t)0x0001)            /* Port output data, bit 0 */
#define GPIO_OUTDR_ODR1                              ((uint16_t)0x0002)            /* Port output data, bit 1 */
#define GPIO_OUTDR_ODR2                              ((uint16_t)0x0004)            /* Port output data, bit 2 */
#define GPIO_OUTDR_ODR3                              ((uint16_t)0x0008)            /* Port output data, bit 3 */
#define GPIO_OUTDR_ODR4                              ((uint16_t)0x0010)            /* Port output data, bit 4 */
#define GPIO_OUTDR_ODR5                              ((uint16_t)0x0020)            /* Port output data, bit 5 */
#define GPIO_OUTDR_ODR6                              ((uint16_t)0x0040)            /* Port output data, bit 6 */
#define GPIO_OUTDR_ODR7                              ((uint16_t)0x0080)            /* Port output data, bit 7 */
#define GPIO_OUTDR_ODR8                              ((uint16_t)0x0100)            /* Port output data, bit 8 */
#define GPIO_OUTDR_ODR9                              ((uint16_t)0x0200)            /* Port output data, bit 9 */
#define GPIO_OUTDR_ODR10                             ((uint16_t)0x0400)            /* Port output data, bit 10 */
#define GPIO_OUTDR_ODR11                             ((uint16_t)0x0800)            /* Port output data, bit 11 */
#define GPIO_OUTDR_ODR12                             ((uint16_t)0x1000)            /* Port output data, bit 12 */
#define GPIO_OUTDR_ODR13                             ((uint16_t)0x2000)            /* Port output data, bit 13 */
#define GPIO_OUTDR_ODR14                             ((uint16_t)0x4000)            /* Port output data, bit 14 */
#define GPIO_OUTDR_ODR15                             ((uint16_t)0x8000)            /* Port output data, bit 15 */

/******************  Bit definition for GPIO_BSHR register  *******************/
#define GPIO_BSHR_BS0                                ((uint32_t)0x00000001)        /* Port x Set bit 0 */
#define GPIO_BSHR_BS1                                ((uint32_t)0x00000002)        /* Port x Set bit 1 */
#define GPIO_BSHR_BS2                                ((uint32_t)0x00000004)        /* Port x Set bit 2 */
#define GPIO_BSHR_BS3                                ((uint32_t)0x00000008)        /* Port x Set bit 3 */
#define GPIO_BSHR_BS4                                ((uint32_t)0x00000010)        /* Port x Set bit 4 */
#define GPIO_BSHR_BS5                                ((uint32_t)0x00000020)        /* Port x Set bit 5 */
#define GPIO_BSHR_BS6                                ((uint32_t)0x00000040)        /* Port x Set bit 6 */
#define GPIO_BSHR_BS7                                ((uint32_t)0x00000080)        /* Port x Set bit 7 */
#define GPIO_BSHR_BS8                                ((uint32_t)0x00000100)        /* Port x Set bit 8 */
#define GPIO_BSHR_BS9                                ((uint32_t)0x00000200)        /* Port x Set bit 9 */
#define GPIO_BSHR_BS10                               ((uint32_t)0x00000400)        /* Port x Set bit 10 */
#define GPIO_BSHR_BS11                               ((uint32_t)0x00000800)        /* Port x Set bit 11 */
#define GPIO_BSHR_BS12                               ((uint32_t)0x00001000)        /* Port x Set bit 12 */
#define GPIO_BSHR_BS13                               ((uint32_t)0x00002000)        /* Port x Set bit 13 */
#define GPIO_BSHR_BS14                               ((uint32_t)0x00004000)        /* Port x Set bit 14 */
#define GPIO_BSHR_BS15                               ((uint32_t)0x00008000)        /* Port x Set bit 15 */

#define GPIO_BSHR_BR0                                ((uint32_t)0x00010000)        /* Port x Reset bit 0 */
#define GPIO_BSHR_BR1                                ((uint32_t)0x00020000)        /* Port x Reset bit 1 */
#define GPIO_BSHR_BR2                                ((uint32_t)0x00040000)        /* Port x Reset bit 2 */
#define GPIO_BSHR_BR3                                ((uint32_t)0x00080000)        /* Port x Reset bit 3 */
#define GPIO_BSHR_BR4                                ((uint32_t)0x00100000)        /* Port x Reset bit 4 */
#define GPIO_BSHR_BR5                                ((uint32_t)0x00200000)        /* Port x Reset bit 5 */
#define GPIO_BSHR_BR6                                ((uint32_t)0x00400000)        /* Port x Reset bit 6 */
#define GPIO_BSHR_BR7                                ((uint32_t)0x00800000)        /* Port x Reset bit 7 */
#define GPIO_BSHR_BR8                                ((uint32_t)0x01000000)        /* Port x Reset bit 8 */
#define GPIO_BSHR_BR9                                ((uint32_t)0x02000000)        /* Port x Reset bit 9 */
#define GPIO_BSHR_BR10                               ((uint32_t)0x04000000)        /* Port x Reset bit 10 */
#define GPIO_BSHR_BR11                               ((uint32_t)0x08000000)        /* Port x Reset bit 11 */
#define GPIO_BSHR_BR12                               ((uint32_t)0x10000000)        /* Port x Reset bit 12 */
#define GPIO_BSHR_BR13                               ((uint32_t)0x20000000)        /* Port x Reset bit 13 */
#define GPIO_BSHR_BR14                               ((uint32_t)0x40000000)        /* Port x Reset bit 14 */
#define GPIO_BSHR_BR15                               ((uint32_t)0x80000000)        /* Port x Reset bit 15 */

/*******************  Bit definition for GPIO_BCR register  *******************/
#define GPIO_BCR_BR0                                 ((uint16_t)0x0001)            /* Port x Reset bit 0 */
#define GPIO_BCR_BR1                                 ((uint16_t)0x0002)            /* Port x Reset bit 1 */
#define GPIO_BCR_BR2                                 ((uint16_t)0x0004)            /* Port x Reset bit 2 */
#define GPIO_BCR_BR3                                 ((uint16_t)0x0008)            /* Port x Reset bit 3 */
#define GPIO_BCR_BR4                                 ((uint16_t)0x0010)            /* Port x Reset bit 4 */
#define GPIO_BCR_BR5                                 ((uint16_t)0x0020)            /* Port x Reset bit 5 */
#define GPIO_BCR_BR6                                 ((uint16_t)0x0040)            /* Port x Reset bit 6 */
#define GPIO_BCR_BR7                                 ((uint16_t)0x0080)            /* Port x Reset bit 7 */
#define GPIO_BCR_BR8                                 ((uint16_t)0x0100)            /* Port x Reset bit 8 */
#define GPIO_BCR_BR9                                 ((uint16_t)0x0200)            /* Port x Reset bit 9 */
#define GPIO_BCR_BR10                                ((uint16_t)0x0400)            /* Port x Reset bit 10 */
#define GPIO_BCR_BR11                                ((uint16_t)0x0800)            /* Port x Reset bit 11 */
#define GPIO_BCR_BR12                                ((uint16_t)0x1000)            /* Port x Reset bit 12 */
#define GPIO_BCR_BR13                                ((uint16_t)0x2000)            /* Port x Reset bit 13 */
#define GPIO_BCR_BR14                                ((uint16_t)0x4000)            /* Port x Reset bit 14 */
#define GPIO_BCR_BR15                                ((uint16_t)0x8000)            /* Port x Reset bit 15 */

/******************  Bit definition for GPIO_LCKR register  *******************/
#define GPIO_LCK0                                    ((uint32_t)0x00000001)        /* Port x Lock bit 0 */
#define GPIO_LCK1                                    ((uint32_t)0x00000002)        /* Port x Lock bit 1 */
#define GPIO_LCK2                                    ((uint32_t)0x00000004)        /* Port x Lock bit 2 */
#define GPIO_LCK3                                    ((uint32_t)0x00000008)        /* Port x Lock bit 3 */
#define GPIO_LCK4                                    ((uint32_t)0x00000010)        /* Port x Lock bit 4 */
#define GPIO_LCK5                                    ((uint32_t)0x00000020)        /* Port x Lock bit 5 */
#define GPIO_LCK6                                    ((uint32_t)0x00000040)        /* Port x Lock bit 6 */
#define GPIO_LCK7                                    ((uint32_t)0x00000080)        /* Port x Lock bit 7 */
#define GPIO_LCK8                                    ((uint32_t)0x00000100)        /* Port x Lock bit 8 */
#define GPIO_LCK9                                    ((uint32_t)0x00000200)        /* Port x Lock bit 9 */
#define GPIO_LCK10                                   ((uint32_t)0x00000400)        /* Port x Lock bit 10 */
#define GPIO_LCK11                                   ((uint32_t)0x00000800)        /* Port x Lock bit 11 */
#define GPIO_LCK12                                   ((uint32_t)0x00001000)        /* Port x Lock bit 12 */
#define GPIO_LCK13                                   ((uint32_t)0x00002000)        /* Port x Lock bit 13 */
#define GPIO_LCK14                                   ((uint32_t)0x00004000)        /* Port x Lock bit 14 */
#define GPIO_LCK15                                   ((uint32_t)0x00008000)        /* Port x Lock bit 15 */
#define GPIO_LCKK                                    ((uint32_t)0x00010000)        /* Lock key */

/******************  Bit definition for AFIO_PCFR1 register  ******************/
#define AFIO_PCFR1_PD0_1_REMAP                       ((uint32_t)0x00000001)
#define AFIO_PCFR1_ADC1_ETRGINJ_REMAP                ((uint32_t)0x00000002)
#define AFIO_PCFR1_ADC1_ETRGREG_REMAP                ((uint32_t)0x00000004)
#define AFIO_PCFR1_ADC2_ETRGINJ_REMAP                ((uint32_t)0x00000008)
#define AFIO_PCFR1_ADC2_ETRGREG_REMAP                ((uint32_t)0x00000010)

#define AFIO_PCFR1_UHSIF_CLK_REMAP                   ((uint32_t)0x000000C0)
#define AFIO_PCFR1_UHSIF_CLK_REMAP_0                 ((uint32_t)0x00000040)
#define AFIO_PCFR1_UHSIF_CLK_REMAP_1                 ((uint32_t)0x00000080)

#define AFIO_PCFR1_UHSIF_PORT_REMAP                  ((uint32_t)0x00000300)
#define AFIO_PCFR1_UHSIF_PORT_REMAP_0                ((uint32_t)0x00000100)
#define AFIO_PCFR1_UHSIF_PORT_REMAP_1                ((uint32_t)0x00000200)

#define AFIO_PCFR1_SDMMC_REMAP                       ((uint32_t)0x00000C00)
#define AFIO_PCFR1_SDMMC_REMAP_0                     ((uint32_t)0x00000400)
#define AFIO_PCFR1_SDMMC_REMAP_1                     ((uint32_t)0x00000800)

#define AFIO_PCFR1_TIM2ITR1_REMAP                    ((uint32_t)0x00001000)

#define AFIO_PCFR1_VIO18_IO_HSLV                     ((uint32_t)0x00010000)
#define AFIO_PCFR1_VIO33_IO_HSLV                     ((uint32_t)0x00020000)
#define AFIO_PCFR1_VDD33_IO_HSLV                     ((uint32_t)0x00040000)

#define AFIO_PCFR1_USBPD_CC_HVT                      ((uint32_t)0x00100000)

#define AFIO_PCFR1_SWJ_CFG                           ((uint32_t)0x07000000)        /* SWJ_CFG[2:0] bits (Serial Wire JTAG configuration) */
#define AFIO_PCFR1_SWJ_CFG_0                         ((uint32_t)0x01000000)        /* Bit 0 */
#define AFIO_PCFR1_SWJ_CFG_1                         ((uint32_t)0x02000000)        /* Bit 1 */
#define AFIO_PCFR1_SWJ_CFG_2                         ((uint32_t)0x04000000)        /* Bit 2 */

/******************  Bit definition for AFIO_AFLR register  *******************/
#define AFIO_AFLR_AFR0                               ((uint32_t)0x0000000F)
#define AFIO_AFLR_AFR0_0                             ((uint32_t)0x00000001)
#define AFIO_AFLR_AFR0_1                             ((uint32_t)0x00000002)
#define AFIO_AFLR_AFR0_2                             ((uint32_t)0x00000004)
#define AFIO_AFLR_AFR0_3                             ((uint32_t)0x00000008)

#define AFIO_AFLR_AFR1                               ((uint32_t)0x000000F0)
#define AFIO_AFLR_AFR1_0                             ((uint32_t)0x00000010)
#define AFIO_AFLR_AFR1_1                             ((uint32_t)0x00000020)
#define AFIO_AFLR_AFR1_2                             ((uint32_t)0x00000040)
#define AFIO_AFLR_AFR1_3                             ((uint32_t)0x00000080)

#define AFIO_AFLR_AFR2                               ((uint32_t)0x00000F00)
#define AFIO_AFLR_AFR2_0                             ((uint32_t)0x00000100)
#define AFIO_AFLR_AFR2_1                             ((uint32_t)0x00000200)
#define AFIO_AFLR_AFR2_2                             ((uint32_t)0x00000400)
#define AFIO_AFLR_AFR2_3                             ((uint32_t)0x00000800)

#define AFIO_AFLR_AFR3                               ((uint32_t)0x0000F000)
#define AFIO_AFLR_AFR3_0                             ((uint32_t)0x00001000)
#define AFIO_AFLR_AFR3_1                             ((uint32_t)0x00002000)
#define AFIO_AFLR_AFR3_2                             ((uint32_t)0x00004000)
#define AFIO_AFLR_AFR3_3                             ((uint32_t)0x00008000)
      
#define AFIO_AFLR_AFR4                               ((uint32_t)0x000F0000)
#define AFIO_AFLR_AFR4_0                             ((uint32_t)0x00010000)
#define AFIO_AFLR_AFR4_1                             ((uint32_t)0x00020000)
#define AFIO_AFLR_AFR4_2                             ((uint32_t)0x00040000)
#define AFIO_AFLR_AFR4_3                             ((uint32_t)0x00080000)

#define AFIO_AFLR_AFR5                               ((uint32_t)0x00F00000)
#define AFIO_AFLR_AFR5_0                             ((uint32_t)0x00100000)
#define AFIO_AFLR_AFR5_1                             ((uint32_t)0x00200000)
#define AFIO_AFLR_AFR5_2                             ((uint32_t)0x00400000)
#define AFIO_AFLR_AFR5_3                             ((uint32_t)0x00800000)

#define AFIO_AFLR_AFR6                               ((uint32_t)0x0F000000)
#define AFIO_AFLR_AFR6_0                             ((uint32_t)0x01000000)
#define AFIO_AFLR_AFR6_1                             ((uint32_t)0x02000000)
#define AFIO_AFLR_AFR6_2                             ((uint32_t)0x04000000)
#define AFIO_AFLR_AFR6_3                             ((uint32_t)0x08000000)

#define AFIO_AFLR_AFR7                               ((uint32_t)0xF0000000)
#define AFIO_AFLR_AFR7_0                             ((uint32_t)0x10000000)
#define AFIO_AFLR_AFR7_1                             ((uint32_t)0x20000000)
#define AFIO_AFLR_AFR7_2                             ((uint32_t)0x40000000)
#define AFIO_AFLR_AFR7_3                             ((uint32_t)0x80000000)

/******************  Bit definition for AFIO_AFHR register  *******************/
#define AFIO_AFHR_AFR8                               ((uint32_t)0x0000000F)
#define AFIO_AFHR_AFR8_0                             ((uint32_t)0x00000001)
#define AFIO_AFHR_AFR8_1                             ((uint32_t)0x00000002)
#define AFIO_AFHR_AFR8_2                             ((uint32_t)0x00000004)
#define AFIO_AFHR_AFR8_3                             ((uint32_t)0x00000008)

#define AFIO_AFHR_AFR9                               ((uint32_t)0x000000F0)
#define AFIO_AFHR_AFR9_0                             ((uint32_t)0x00000010)
#define AFIO_AFHR_AFR9_1                             ((uint32_t)0x00000020)
#define AFIO_AFHR_AFR9_2                             ((uint32_t)0x00000040)
#define AFIO_AFHR_AFR9_3                             ((uint32_t)0x00000080)

#define AFIO_AFHR_AFR10                              ((uint32_t)0x00000F00)
#define AFIO_AFHR_AFR10_0                            ((uint32_t)0x00000100)
#define AFIO_AFHR_AFR10_1                            ((uint32_t)0x00000200)
#define AFIO_AFHR_AFR10_2                            ((uint32_t)0x00000400)
#define AFIO_AFHR_AFR10_3                            ((uint32_t)0x00000800)

#define AFIO_AFHR_AFR11                              ((uint32_t)0x0000F000)
#define AFIO_AFHR_AFR11_0                            ((uint32_t)0x00001000)
#define AFIO_AFHR_AFR11_1                            ((uint32_t)0x00002000)
#define AFIO_AFHR_AFR11_2                            ((uint32_t)0x00004000)
#define AFIO_AFHR_AFR11_3                            ((uint32_t)0x00008000)

#define AFIO_AFHR_AFR12                              ((uint32_t)0x000F0000)
#define AFIO_AFHR_AFR12_0                            ((uint32_t)0x00010000)
#define AFIO_AFHR_AFR12_1                            ((uint32_t)0x00020000)
#define AFIO_AFHR_AFR12_2                            ((uint32_t)0x00040000)
#define AFIO_AFHR_AFR12_3                            ((uint32_t)0x00080000)

#define AFIO_AFHR_AFR13                              ((uint32_t)0x00F00000)
#define AFIO_AFHR_AFR13_0                            ((uint32_t)0x00100000)
#define AFIO_AFHR_AFR13_1                            ((uint32_t)0x00200000)
#define AFIO_AFHR_AFR13_2                            ((uint32_t)0x00400000)
#define AFIO_AFHR_AFR13_3                            ((uint32_t)0x00800000)

#define AFIO_AFHR_AFR14                              ((uint32_t)0x0F000000)
#define AFIO_AFHR_AFR14_0                            ((uint32_t)0x01000000)
#define AFIO_AFHR_AFR14_1                            ((uint32_t)0x02000000)
#define AFIO_AFHR_AFR14_2                            ((uint32_t)0x04000000)
#define AFIO_AFHR_AFR14_3                            ((uint32_t)0x08000000)
      
#define AFIO_AFHR_AFR15                              ((uint32_t)0xF0000000)
#define AFIO_AFHR_AFR15_0                            ((uint32_t)0x10000000)
#define AFIO_AFHR_AFR15_1                            ((uint32_t)0x20000000)
#define AFIO_AFHR_AFR15_2                            ((uint32_t)0x40000000)
#define AFIO_AFHR_AFR15_3                            ((uint32_t)0x80000000)

/******************  Bit definition for AFIO_EXTICR1 register  *******************/
#define AFIO_EXTICR1_EXTI0                           ((uint32_t)0x0000000F)        /* EXTI 0 configuration */
#define AFIO_EXTICR1_EXTI1                           ((uint32_t)0x000000F0)        /* EXTI 1 configuration */
#define AFIO_EXTICR1_EXTI2                           ((uint32_t)0x00000F00)        /* EXTI 2 configuration */
#define AFIO_EXTICR1_EXTI3                           ((uint32_t)0x0000F000)        /* EXTI 3 configuration */
#define AFIO_EXTICR1_EXTI4                           ((uint32_t)0x000F0000)        /* EXTI 4 configuration */
#define AFIO_EXTICR1_EXTI5                           ((uint32_t)0x00F00000)        /* EXTI 5 configuration */
#define AFIO_EXTICR1_EXTI6                           ((uint32_t)0x0F000000)        /* EXTI 6 configuration */
#define AFIO_EXTICR1_EXTI7                           ((uint32_t)0xF0000000)        /* EXTI 7 configuration */

#define AFIO_EXTICR1_EXTI0_PA                        ((uint32_t)0x00000000)            /* PA[0] pin */
#define AFIO_EXTICR1_EXTI0_PB                        ((uint32_t)0x00000001)            /* PB[0] pin */
#define AFIO_EXTICR1_EXTI0_PC                        ((uint32_t)0x00000002)            /* PC[0] pin */
#define AFIO_EXTICR1_EXTI0_PD                        ((uint32_t)0x00000003)            /* PD[0] pin */
#define AFIO_EXTICR1_EXTI0_PE                        ((uint32_t)0x00000004)            /* PE[0] pin */
#define AFIO_EXTICR1_EXTI0_PF                        ((uint32_t)0x00000005)            /* PF[0] pin */
#define AFIO_EXTICR1_EXTI0_CMPOUT                    ((uint32_t)0x00000006)            /* CMP OUT */

#define AFIO_EXTICR1_EXTI1_PA                        ((uint32_t)0x00000000)            /* PA[1] pin */
#define AFIO_EXTICR1_EXTI1_PB                        ((uint32_t)0x00000010)            /* PB[1] pin */
#define AFIO_EXTICR1_EXTI1_PC                        ((uint32_t)0x00000020)            /* PC[1] pin */
#define AFIO_EXTICR1_EXTI1_PD                        ((uint32_t)0x00000030)            /* PD[1] pin */
#define AFIO_EXTICR1_EXTI1_PE                        ((uint32_t)0x00000040)            /* PE[1] pin */
#define AFIO_EXTICR1_EXTI1_PF                        ((uint32_t)0x00000050)            /* PF[1] pin */
#define AFIO_EXTICR1_EXTI1_CMPOUT                    ((uint32_t)0x00000060)            /* CMP OUT */

#define AFIO_EXTICR1_EXTI2_PA                        ((uint32_t)0x00000000)            /* PA[2] pin */
#define AFIO_EXTICR1_EXTI2_PB                        ((uint32_t)0x00000100)            /* PB[2] pin */
#define AFIO_EXTICR1_EXTI2_PC                        ((uint32_t)0x00000200)            /* PC[2] pin */
#define AFIO_EXTICR1_EXTI2_PD                        ((uint32_t)0x00000300)            /* PD[2] pin */
#define AFIO_EXTICR1_EXTI2_PE                        ((uint32_t)0x00000400)            /* PE[2] pin */
#define AFIO_EXTICR1_EXTI2_PF                        ((uint32_t)0x00000500)            /* PF[2] pin */
#define AFIO_EXTICR1_EXTI2_CMPOUT                    ((uint32_t)0x00000600)            /* CMP OUT */

#define AFIO_EXTICR1_EXTI3_PA                        ((uint32_t)0x00000000)            /* PA[3] pin */
#define AFIO_EXTICR1_EXTI3_PB                        ((uint32_t)0x00001000)            /* PB[3] pin */
#define AFIO_EXTICR1_EXTI3_PC                        ((uint32_t)0x00002000)            /* PC[3] pin */
#define AFIO_EXTICR1_EXTI3_PD                        ((uint32_t)0x00003000)            /* PD[3] pin */
#define AFIO_EXTICR1_EXTI3_PE                        ((uint32_t)0x00004000)            /* PE[3] pin */
#define AFIO_EXTICR1_EXTI3_PF                        ((uint32_t)0x00005000)            /* PF[3] pin */
#define AFIO_EXTICR1_EXTI3_CMPOUT                    ((uint32_t)0x00006000)            /* CMP OUT */

#define AFIO_EXTICR1_EXTI4_PA                        ((uint32_t)0x00000000)            /* PA[4] pin */
#define AFIO_EXTICR1_EXTI4_PB                        ((uint32_t)0x00010000)            /* PB[4] pin */
#define AFIO_EXTICR1_EXTI4_PC                        ((uint32_t)0x00020000)            /* PC[4] pin */
#define AFIO_EXTICR1_EXTI4_PD                        ((uint32_t)0x00030000)            /* PD[4] pin */
#define AFIO_EXTICR1_EXTI4_PE                        ((uint32_t)0x00040000)            /* PE[4] pin */
#define AFIO_EXTICR1_EXTI4_PF                        ((uint32_t)0x00050000)            /* PF[4] pin */
#define AFIO_EXTICR1_EXTI4_CMPOUT                    ((uint32_t)0x00060000)            /* CMP OUT */

#define AFIO_EXTICR1_EXTI5_PA                        ((uint32_t)0x00000000)            /* PA[5] pin */
#define AFIO_EXTICR1_EXTI5_PB                        ((uint32_t)0x00100000)            /* PB[5] pin */
#define AFIO_EXTICR1_EXTI5_PC                        ((uint32_t)0x00200000)            /* PC[5] pin */
#define AFIO_EXTICR1_EXTI5_PD                        ((uint32_t)0x00300000)            /* PD[5] pin */
#define AFIO_EXTICR1_EXTI5_PE                        ((uint32_t)0x00400000)            /* PE[5] pin */
#define AFIO_EXTICR1_EXTI5_PF                        ((uint32_t)0x00500000)            /* PF[5] pin */
#define AFIO_EXTICR1_EXTI5_CMPOUT                    ((uint32_t)0x00600000)            /* CMP OUT */

#define AFIO_EXTICR1_EXTI6_PA                        ((uint32_t)0x00000000)            /* PA[6] pin */
#define AFIO_EXTICR1_EXTI6_PB                        ((uint32_t)0x01000000)            /* PB[6] pin */
#define AFIO_EXTICR1_EXTI6_PC                        ((uint32_t)0x02000000)            /* PC[6] pin */
#define AFIO_EXTICR1_EXTI6_PD                        ((uint32_t)0x03000000)            /* PD[6] pin */
#define AFIO_EXTICR1_EXTI6_PE                        ((uint32_t)0x04000000)            /* PE[6] pin */
#define AFIO_EXTICR1_EXTI6_PF                        ((uint32_t)0x05000000)            /* PF[6] pin */
#define AFIO_EXTICR1_EXTI6_CMPOUT                    ((uint32_t)0x06000000)            /* CMP OUT */

#define AFIO_EXTICR1_EXTI7_PA                        ((uint32_t)0x00000000)            /* PA[7] pin */
#define AFIO_EXTICR1_EXTI7_PB                        ((uint32_t)0x10000000)            /* PB[7] pin */
#define AFIO_EXTICR1_EXTI7_PC                        ((uint32_t)0x20000000)            /* PC[7] pin */
#define AFIO_EXTICR1_EXTI7_PD                        ((uint32_t)0x30000000)            /* PD[7] pin */
#define AFIO_EXTICR1_EXTI7_PE                        ((uint32_t)0x40000000)            /* PE[7] pin */
#define AFIO_EXTICR1_EXTI7_PF                        ((uint32_t)0x50000000)            /* PF[7] pin */
#define AFIO_EXTICR1_EXTI7_CMPOUT                    ((uint32_t)0x60000000)            /* CMP OUT */

/******************  Bit definition for AFIO_EXTICR2 register  *******************/
#define AFIO_EXTICR2_EXTI8                           ((uint32_t)0x0000000F)        /* EXTI 0 configuration */
#define AFIO_EXTICR2_EXTI9                           ((uint32_t)0x000000F0)        /* EXTI 1 configuration */
#define AFIO_EXTICR2_EXTI10                          ((uint32_t)0x00000F00)        /* EXTI 2 configuration */
#define AFIO_EXTICR2_EXTI11                          ((uint32_t)0x0000F000)        /* EXTI 3 configuration */
#define AFIO_EXTICR2_EXTI12                          ((uint32_t)0x000F0000)        /* EXTI 4 configuration */
#define AFIO_EXTICR2_EXTI13                          ((uint32_t)0x00F00000)        /* EXTI 5 configuration */
#define AFIO_EXTICR2_EXTI14                          ((uint32_t)0x0F000000)        /* EXTI 6 configuration */
#define AFIO_EXTICR2_EXTI15                          ((uint32_t)0xF0000000)        /* EXTI 7 configuration */

#define AFIO_EXTICR2_EXTI8_PA                        ((uint32_t)0x00000000)            /* PA[0] pin */
#define AFIO_EXTICR2_EXTI8_PB                        ((uint32_t)0x00000001)            /* PB[0] pin */
#define AFIO_EXTICR2_EXTI8_PC                        ((uint32_t)0x00000002)            /* PC[0] pin */
#define AFIO_EXTICR2_EXTI8_PD                        ((uint32_t)0x00000003)            /* PD[0] pin */
#define AFIO_EXTICR2_EXTI8_PE                        ((uint32_t)0x00000004)            /* PE[0] pin */
#define AFIO_EXTICR2_EXTI8_PF                        ((uint32_t)0x00000005)            /* PF[0] pin */
#define AFIO_EXTICR2_EXTI8_CMPOUT                    ((uint32_t)0x00000006)            /* CMP OUT */

#define AFIO_EXTICR2_EXTI9_PA                        ((uint32_t)0x00000000)            /* PA[1] pin */
#define AFIO_EXTICR2_EXTI9_PB                        ((uint32_t)0x00000010)            /* PB[1] pin */
#define AFIO_EXTICR2_EXTI9_PC                        ((uint32_t)0x00000020)            /* PC[1] pin */
#define AFIO_EXTICR2_EXTI9_PD                        ((uint32_t)0x00000030)            /* PD[1] pin */
#define AFIO_EXTICR2_EXTI9_PE                        ((uint32_t)0x00000040)            /* PE[1] pin */
#define AFIO_EXTICR2_EXTI9_PF                        ((uint32_t)0x00000050)            /* PF[1] pin */
#define AFIO_EXTICR2_EXTI9_CMPOUT                    ((uint32_t)0x00000060)            /* CMP OUT */

#define AFIO_EXTICR2_EXTI10_PA                       ((uint32_t)0x00000000)            /* PA[2] pin */
#define AFIO_EXTICR2_EXTI10_PB                       ((uint32_t)0x00000100)            /* PB[2] pin */
#define AFIO_EXTICR2_EXTI10_PC                       ((uint32_t)0x00000200)            /* PC[2] pin */
#define AFIO_EXTICR2_EXTI10_PD                       ((uint32_t)0x00000300)            /* PD[2] pin */
#define AFIO_EXTICR2_EXTI10_PE                       ((uint32_t)0x00000400)            /* PE[2] pin */
#define AFIO_EXTICR2_EXTI10_PF                       ((uint32_t)0x00000500)            /* PF[2] pin */
#define AFIO_EXTICR2_EXTI10_CMPOUT                   ((uint32_t)0x00000600)            /* CMP OUT */

#define AFIO_EXTICR2_EXTI11_PA                       ((uint32_t)0x00000000)            /* PA[3] pin */
#define AFIO_EXTICR2_EXTI11_PB                       ((uint32_t)0x00001000)            /* PB[3] pin */
#define AFIO_EXTICR2_EXTI11_PC                       ((uint32_t)0x00002000)            /* PC[3] pin */
#define AFIO_EXTICR2_EXTI11_PD                       ((uint32_t)0x00003000)            /* PD[3] pin */
#define AFIO_EXTICR2_EXTI11_PE                       ((uint32_t)0x00004000)            /* PE[3] pin */
#define AFIO_EXTICR2_EXTI11_PF                       ((uint32_t)0x00005000)            /* PF[3] pin */
#define AFIO_EXTICR2_EXTI11_CMPOUT                   ((uint32_t)0x00006000)            /* CMP OUT */

#define AFIO_EXTICR2_EXTI12_PA                       ((uint32_t)0x00000000)            /* PA[4] pin */
#define AFIO_EXTICR2_EXTI12_PB                       ((uint32_t)0x00010000)            /* PB[4] pin */
#define AFIO_EXTICR2_EXTI12_PC                       ((uint32_t)0x00020000)            /* PC[4] pin */
#define AFIO_EXTICR2_EXTI12_PD                       ((uint32_t)0x00030000)            /* PD[4] pin */
#define AFIO_EXTICR2_EXTI12_PE                       ((uint32_t)0x00040000)            /* PE[4] pin */
#define AFIO_EXTICR2_EXTI12_PF                       ((uint32_t)0x00050000)            /* PF[4] pin */
#define AFIO_EXTICR2_EXTI12_CMPOUT                   ((uint32_t)0x00060000)            /* CMP OUT */

#define AFIO_EXTICR2_EXTI13_PA                       ((uint32_t)0x00000000)            /* PA[5] pin */
#define AFIO_EXTICR2_EXTI13_PB                       ((uint32_t)0x00100000)            /* PB[5] pin */
#define AFIO_EXTICR2_EXTI13_PC                       ((uint32_t)0x00200000)            /* PC[5] pin */
#define AFIO_EXTICR2_EXTI13_PD                       ((uint32_t)0x00300000)            /* PD[5] pin */
#define AFIO_EXTICR2_EXTI13_PE                       ((uint32_t)0x00400000)            /* PE[5] pin */
#define AFIO_EXTICR2_EXTI13_PF                       ((uint32_t)0x00500000)            /* PF[5] pin */
#define AFIO_EXTICR2_EXTI13_CMPOUT                   ((uint32_t)0x00600000)            /* CMP OUT */

#define AFIO_EXTICR2_EXTI14_PA                       ((uint32_t)0x00000000)            /* PA[6] pin */
#define AFIO_EXTICR2_EXTI14_PB                       ((uint32_t)0x01000000)            /* PB[6] pin */
#define AFIO_EXTICR2_EXTI14_PC                       ((uint32_t)0x02000000)            /* PC[6] pin */
#define AFIO_EXTICR2_EXTI14_PD                       ((uint32_t)0x03000000)            /* PD[6] pin */
#define AFIO_EXTICR2_EXTI14_PE                       ((uint32_t)0x04000000)            /* PE[6] pin */
#define AFIO_EXTICR2_EXTI14_PF                       ((uint32_t)0x05000000)            /* PF[6] pin */
#define AFIO_EXTICR2_EXTI14_CMPOUT                   ((uint32_t)0x06000000)            /* CMP OUT */

#define AFIO_EXTICR2_EXTI15_PA                       ((uint32_t)0x00000000)            /* PA[7] pin */
#define AFIO_EXTICR2_EXTI15_PB                       ((uint32_t)0x10000000)            /* PB[7] pin */
#define AFIO_EXTICR2_EXTI15_PC                       ((uint32_t)0x20000000)            /* PC[7] pin */
#define AFIO_EXTICR2_EXTI15_PD                       ((uint32_t)0x30000000)            /* PD[7] pin */
#define AFIO_EXTICR2_EXTI15_PE                       ((uint32_t)0x40000000)            /* PE[7] pin */
#define AFIO_EXTICR2_EXTI15_PF                       ((uint32_t)0x50000000)            /* PF[7] pin */
#define AFIO_EXTICR2_EXTI15_CMPOUT                   ((uint32_t)0x60000000)            /* CMP OUT */

/******************************************************************************/
/*                           Independent WATCHDOG                             */
/******************************************************************************/

/*******************  Bit definition for IWDG_CTLR register  ********************/
#define  IWDG_KEY                                    ((uint16_t)0xFFFF)            /* Key value (write only, read 0000h) */

/*******************  Bit definition for IWDG_PSCR register  ********************/
#define  IWDG_PR                                     ((uint8_t)0x07)               /* PR[2:0] (Prescaler divider) */
#define  IWDG_PR_0                                   ((uint8_t)0x01)               /* Bit 0 */
#define  IWDG_PR_1                                   ((uint8_t)0x02)               /* Bit 1 */
#define  IWDG_PR_2                                   ((uint8_t)0x04)               /* Bit 2 */

/*******************  Bit definition for IWDG_RLDR register  *******************/
#define  IWDG_RL                                    ((uint16_t)0x0FFF)            /* Watchdog counter reload value */

/*******************  Bit definition for IWDG_STATR register  ********************/
#define  IWDG_PVU                                    ((uint8_t)0x01)               /* Watchdog prescaler value update */
#define  IWDG_RVU                                    ((uint8_t)0x02)               /* Watchdog counter reload value update */

/******************************************************************************/
/*                      Inter-integrated Circuit Interface                    */
/******************************************************************************/

/*******************  Bit definition for I2C_CTLR1 register  ********************/
#define  I2C_CTLR1_PE                                ((uint16_t)0x0001)            /* Peripheral Enable */
#define  I2C_CTLR1_SMBUS                             ((uint16_t)0x0002)            /* SMBus Mode */
#define  I2C_CTLR1_SMBTYPE                           ((uint16_t)0x0008)            /* SMBus Type */
#define  I2C_CTLR1_ENARP                             ((uint16_t)0x0010)            /* ARP Enable */
#define  I2C_CTLR1_ENPEC                             ((uint16_t)0x0020)            /* PEC Enable */
#define  I2C_CTLR1_ENGC                              ((uint16_t)0x0040)            /* General Call Enable */
#define  I2C_CTLR1_NOSTRETCH                         ((uint16_t)0x0080)            /* Clock Stretching Disable (Slave mode) */
#define  I2C_CTLR1_START                             ((uint16_t)0x0100)            /* Start Generation */
#define  I2C_CTLR1_STOP                              ((uint16_t)0x0200)            /* Stop Generation */
#define  I2C_CTLR1_ACK                               ((uint16_t)0x0400)            /* Acknowledge Enable */
#define  I2C_CTLR1_POS                               ((uint16_t)0x0800)            /* Acknowledge/PEC Position (for data reception) */
#define  I2C_CTLR1_PEC                               ((uint16_t)0x1000)            /* Packet Error Checking */
#define  I2C_CTLR1_ALERT                             ((uint16_t)0x2000)            /* SMBus Alert */
#define  I2C_CTLR1_SWRST                             ((uint16_t)0x8000)            /* Software Reset */

/*******************  Bit definition for I2C_CTLR2 register  ********************/
#define  I2C_CTLR2_FREQ                              ((uint16_t)0x003F)            /* FREQ[5:0] bits (Peripheral Clock Frequency) */
#define  I2C_CTLR2_FREQ_0                            ((uint16_t)0x0001)            /* Bit 0 */
#define  I2C_CTLR2_FREQ_1                            ((uint16_t)0x0002)            /* Bit 1 */
#define  I2C_CTLR2_FREQ_2                            ((uint16_t)0x0004)            /* Bit 2 */
#define  I2C_CTLR2_FREQ_3                            ((uint16_t)0x0008)            /* Bit 3 */
#define  I2C_CTLR2_FREQ_4                            ((uint16_t)0x0010)            /* Bit 4 */
#define  I2C_CTLR2_FREQ_5                            ((uint16_t)0x0020)            /* Bit 5 */

#define  I2C_CTLR2_ITERREN                           ((uint16_t)0x0100)            /* Error Interrupt Enable */
#define  I2C_CTLR2_ITEVTEN                           ((uint16_t)0x0200)            /* Event Interrupt Enable */
#define  I2C_CTLR2_ITBUFEN                           ((uint16_t)0x0400)            /* Buffer Interrupt Enable */
#define  I2C_CTLR2_DMAEN                             ((uint16_t)0x0800)            /* DMA Requests Enable */
#define  I2C_CTLR2_LAST                              ((uint16_t)0x1000)            /* DMA Last Transfer */

/*******************  Bit definition for I2C_OADDR1 register  *******************/
#define  I2C_OADDR1_ADD0                             ((uint16_t)0x0001)
#define  I2C_OADDR1_ADD1_7                           ((uint16_t)0x00FE)            /* Interface Address */
#define  I2C_OADDR1_ADD8_9                           ((uint16_t)0x0300)            /* Interface Address */
      
#define  I2C_OADDR1_ADD0                             ((uint16_t)0x0001)            /* Bit 0 */
#define  I2C_OADDR1_ADD1                             ((uint16_t)0x0002)            /* Bit 1 */
#define  I2C_OADDR1_ADD2                             ((uint16_t)0x0004)            /* Bit 2 */
#define  I2C_OADDR1_ADD3                             ((uint16_t)0x0008)            /* Bit 3 */
#define  I2C_OADDR1_ADD4                             ((uint16_t)0x0010)            /* Bit 4 */
#define  I2C_OADDR1_ADD5                             ((uint16_t)0x0020)            /* Bit 5 */
#define  I2C_OADDR1_ADD6                             ((uint16_t)0x0040)            /* Bit 6 */
#define  I2C_OADDR1_ADD7                             ((uint16_t)0x0080)            /* Bit 7 */
#define  I2C_OADDR1_ADD8                             ((uint16_t)0x0100)            /* Bit 8 */
#define  I2C_OADDR1_ADD9                             ((uint16_t)0x0200)            /* Bit 9 */

#define  I2C_OADDR1_ADDMODE                          ((uint16_t)0x8000)            /* Addressing Mode (Slave mode) */

/*******************  Bit definition for I2C_OADDR2 register  *******************/
#define  I2C_OADDR2_ENDUAL                           ((uint8_t)0x01)               /* Dual addressing mode enable */
#define  I2C_OADDR2_ADD2                             ((uint8_t)0xFE)               /* Interface address */

/********************  Bit definition for I2C_DATAR register  ********************/
#define  I2C_DR_DATAR                                ((uint8_t)0xFF)               /* 8-bit Data Register */

/*******************  Bit definition for I2C_STAR1 register  ********************/
#define  I2C_STAR1_SB                                ((uint16_t)0x0001)            /* Start Bit (Master mode) */
#define  I2C_STAR1_ADDR                              ((uint16_t)0x0002)            /* Address sent (master mode)/matched (slave mode) */
#define  I2C_STAR1_BTF                               ((uint16_t)0x0004)            /* Byte Transfer Finished */
#define  I2C_STAR1_ADD10                             ((uint16_t)0x0008)            /* 10-bit header sent (Master mode) */
#define  I2C_STAR1_STOPF                             ((uint16_t)0x0010)            /* Stop detection (Slave mode) */
#define  I2C_STAR1_RXNE                              ((uint16_t)0x0040)            /* Data Register not Empty (receivers) */
#define  I2C_STAR1_TXE                               ((uint16_t)0x0080)            /* Data Register Empty (transmitters) */
#define  I2C_STAR1_BERR                              ((uint16_t)0x0100)            /* Bus Error */
#define  I2C_STAR1_ARLO                              ((uint16_t)0x0200)            /* Arbitration Lost (master mode) */
#define  I2C_STAR1_AF                                ((uint16_t)0x0400)            /* Acknowledge Failure */
#define  I2C_STAR1_OVR                               ((uint16_t)0x0800)            /* Overrun/Underrun */
#define  I2C_STAR1_PECERR                            ((uint16_t)0x1000)            /* PEC Error in reception */
#define  I2C_STAR1_TIMEOUT                           ((uint16_t)0x4000)            /* Timeout or Tlow Error */
#define  I2C_STAR1_SMBALERT                          ((uint16_t)0x8000)            /* SMBus Alert */

/*******************  Bit definition for I2C_STAR2 register  ********************/
#define  I2C_STAR2_MSL                               ((uint16_t)0x0001)            /* Master/Slave */
#define  I2C_STAR2_BUSY                              ((uint16_t)0x0002)            /* Bus Busy */
#define  I2C_STAR2_TRA                               ((uint16_t)0x0004)            /* Transmitter/Receiver */
#define  I2C_STAR2_GENCALL                           ((uint16_t)0x0010)            /* General Call Address (Slave mode) */
#define  I2C_STAR2_SMBDEFAULT                        ((uint16_t)0x0020)            /* SMBus Device Default Address (Slave mode) */
#define  I2C_STAR2_SMBHOST                           ((uint16_t)0x0040)            /* SMBus Host Header (Slave mode) */
#define  I2C_STAR2_DUALF                             ((uint16_t)0x0080)            /* Dual Flag (Slave mode) */
#define  I2C_STAR2_PEC                               ((uint16_t)0xFF00)            /* Packet Error Checking Register */

/*******************  Bit definition for I2C_CKCFGR register  ********************/
#define  I2C_CKCFGR_CCR                              ((uint16_t)0x0FFF)            /* Clock Control Register in Fast/Standard mode (Master mode) */
#define  I2C_CKCFGR_DUTY                             ((uint16_t)0x4000)            /* Fast Mode Duty Cycle */
#define  I2C_CKCFGR_FS                               ((uint16_t)0x8000)            /* I2C Master Mode Selection */

/******************  Bit definition for I2C_RTR register  *******************/
#define  I2C_RTR_TRISE                               ((uint8_t)0x3F)               /* Maximum Rise Time in Fast/Standard mode (Master mode) */

/******************************************************************************/
/*                 Improved Inter-integrated Circuit Interface              */
/******************************************************************************/

/*******************  Bit definition for I3C_CTLR register  ********************/
#define  I3C_CTLR_DCNT                               ((uint32_t)0x0000FFFF)
#define  I3C_CTLR_RNW                                ((uint32_t)0x00010000)
#define  I3C_CTLR_ADD                                ((uint32_t)0x00FE0000)

#define  I3C_CTLR_CCC                                ((uint32_t)0x00FF0000)

#define  I3C_CTLR_MTYPE                              ((uint32_t)0x78000000)
#define  I3C_CTLR_MTYPE_0                            ((uint32_t)0x08000000)
#define  I3C_CTLR_MTYPE_1                            ((uint32_t)0x10000000)
#define  I3C_CTLR_MTYPE_2                            ((uint32_t)0x20000000)
#define  I3C_CTLR_MTYPE_3                            ((uint32_t)0x40000000)

#define  I3C_CTLR_MEND                               ((uint32_t)0x80000000)

/*******************  Bit definition for I3C_CFGR register  ********************/
#define  I3C_CFGR_EN                                 ((uint32_t)0x00000001)
#define  I3C_CFGR_CRINIT                             ((uint32_t)0x00000002)
#define  I3C_CFGR_NOARBH                             ((uint32_t)0x00000004)
#define  I3C_CFGR_RSTPTRN                            ((uint32_t)0x00000008)
#define  I3C_CFGR_EXITPTRN                           ((uint32_t)0x00000010)

#define  I3C_CFGR_HJACK                              ((uint32_t)0x00000080)
#define  I3C_CFGR_RXDMAEN                            ((uint32_t)0x00000100)
#define  I3C_CFGR_RXFLUSH                            ((uint32_t)0x00000200)
#define  I3C_CFGR_RXTHRES                            ((uint32_t)0x00000400)
      
#define  I3C_CFGR_TXMAEN                             ((uint32_t)0x00001000)
#define  I3C_CFGR_TXFLUSH                            ((uint32_t)0x00002000)
#define  I3C_CFGR_TXTHRES                            ((uint32_t)0x00004000)

#define  I3C_CFGR_SDMAEN                             ((uint32_t)0x00010000)
#define  I3C_CFGR_SFLUSH                             ((uint32_t)0x00020000)
#define  I3C_CFGR_SMODE                              ((uint32_t)0x00040000)
#define  I3C_CFGR_TMODE                              ((uint32_t)0x00080000)
#define  I3C_CFGR_CDMAEN                             ((uint32_t)0x00010000)
#define  I3C_CFGR_CFLUSH                             ((uint32_t)0x00020000)

#define  I3C_CFGR_TSFSET                             ((uint32_t)0x40000000)

/*******************  Bit definition for I3C_RDR register  ********************/
#define  I3C_RDR_RDB0                                ((uint32_t)0x000000FF)

/*******************  Bit definition for I3C_RDWR register  ********************/
#define  I3C_RDWR_RDB0                               ((uint32_t)0x000000FF)
#define  I3C_RDWR_RDB1                               ((uint32_t)0x0000FF00)
#define  I3C_RDWR_RDB2                               ((uint32_t)0x00FF0000)
#define  I3C_RDWR_RDB3                               ((uint32_t)0xFF000000)

/*******************  Bit definition for I3C_TDR register  ********************/
#define  I3C_TDR_TDB0                                ((uint32_t)0x000000FF)

/*******************  Bit definition for I3C_TDWR register  ********************/
#define  I3C_TDWR_TDB0                               ((uint32_t)0x000000FF)
#define  I3C_TDWR_TDB1                               ((uint32_t)0x0000FF00)
#define  I3C_TDWR_TDB2                               ((uint32_t)0x00FF0000)
#define  I3C_TDWR_TDB3                               ((uint32_t)0xFF000000)

/*******************  Bit definition for I3C_IBIDR register  ********************/
#define  I3C_IBIDR_IBIDB0                            ((uint32_t)0x000000FF)
#define  I3C_IBIDR_IBIDB1                            ((uint32_t)0x0000FF00)
#define  I3C_IBIDR_IBIDB2                            ((uint32_t)0x00FF0000)
#define  I3C_IBIDR_IBIDB3                            ((uint32_t)0xFF000000)

/*******************  Bit definition for I3C_TGTTDR register  ********************/
#define  I3C_TGTTDR_TGTTDCNT                         ((uint32_t)0x0000FFFF)
#define  I3C_TGTTDR_PRELOAD                          ((uint32_t)0x00010000)

/*******************  Bit definition for I3C_RESET register  ********************/
#define  I3C_RESET_HST_SIE_RST                       ((uint32_t)0x01000000)
#define  I3C_RESET_TGT_SIE_RST                       ((uint32_t)0x02000000)

/*******************  Bit definition for I3C_STATR register  ********************/
#define  I3C_STATR_XDCNT                             ((uint32_t)0x0000FFFF)
#define  I3C_STATR_ABT                               ((uint32_t)0x00020000)
#define  I3C_STATR_DIR                               ((uint32_t)0x00040000)

#define  I3C_STATR_MID                               ((uint32_t)0xFF000000)

/*******************  Bit definition for I3C_STATER register  ********************/
#define  I3C_STATER_CODERR                           ((uint32_t)0x0000000F)
#define  I3C_STATER_PERR                             ((uint32_t)0x00000010)
#define  I3C_STATER_STALL                            ((uint32_t)0x00000020)
#define  I3C_STATER_DOVR                             ((uint32_t)0x00000040)
#define  I3C_STATER_COVR                             ((uint32_t)0x00000080)
#define  I3C_STATER_ANACK                            ((uint32_t)0x00000100)
#define  I3C_STATER_DNACK                            ((uint32_t)0x00000200)
#define  I3C_STATER_DERR                             ((uint32_t)0x00000400)

/*******************  Bit definition for I3C_RMR register  ********************/
#define  I3C_RMR_IBIRDCNT                            ((uint32_t)0x00000007)
#define  I3C_RMR_RCODE                               ((uint32_t)0x0000FF00)
#define  I3C_RMR_RADD                                ((uint32_t)0x00FE0000)

/*******************  Bit definition for I3C_EVR register  ********************/
#define  I3C_EVR_CFEF                                ((uint32_t)0x00000001)
#define  I3C_EVR_TXFEF                               ((uint32_t)0x00000002)
#define  I3C_EVR_CFNFF                               ((uint32_t)0x00000004)
#define  I3C_EVR_SFNEF                               ((uint32_t)0x00000008)
#define  I3C_EVR_TXFNFF                              ((uint32_t)0x00000010)
#define  I3C_EVR_RXFNEF                              ((uint32_t)0x00000020)
#define  I3C_EVR_TXLASTF                             ((uint32_t)0x00000040)
#define  I3C_EVR_RXLASTF                             ((uint32_t)0x00000080)

#define  I3C_EVR_FCF                                 ((uint32_t)0x00000200)
#define  I3C_EVR_RXTGTENDF                           ((uint32_t)0x00000400)
#define  I3C_EVR_ERRF                                ((uint32_t)0x00000800)

#define  I3C_EVR_IBIF                                ((uint32_t)0x00008000)
#define  I3C_EVR_IBIENDF                             ((uint32_t)0x00010000)
#define  I3C_EVR_CRF                                 ((uint32_t)0x00020000)
#define  I3C_EVR_CRUPDF                              ((uint32_t)0x00040000)
#define  I3C_EVR_HJF                                 ((uint32_t)0x00080000)
#define  I3C_EVR_WKPF                                ((uint32_t)0x00200000)
#define  I3C_EVR_GETF                                ((uint32_t)0x00400000)
#define  I3C_EVR_STAF                                ((uint32_t)0x00800000)
#define  I3C_EVR_DAUPDF                              ((uint32_t)0x01000000)
#define  I3C_EVR_MWLUPDF                             ((uint32_t)0x02000000)
#define  I3C_EVR_MRLUPDF                             ((uint32_t)0x04000000)
#define  I3C_EVR_RSTF                                ((uint32_t)0x08000000)
#define  I3C_EVR_ASUPDF                              ((uint32_t)0x10000000)
#define  I3C_EVR_INTUPDF                             ((uint32_t)0x20000000)
#define  I3C_EVR_DEFF                                ((uint32_t)0x40000000)
#define  I3C_EVR_GRPF                                ((uint32_t)0x80000000)

/*****************  Bit definition for I3C_INTENR register  ******************/
#define  I3C_INTENR_CFNFIE                           ((uint32_t)0x00000004)
#define  I3C_INTENR_SFNEIE                           ((uint32_t)0x00000008)
#define  I3C_INTENR_TXFNEIE                          ((uint32_t)0x00000010)
#define  I3C_INTENR_RXFNEIE                          ((uint32_t)0x00000020)

#define  I3C_INTENR_FCIE                             ((uint32_t)0x00000200)
#define  I3C_INTENR_RXTGTENDIE                       ((uint32_t)0x00000400)
#define  I3C_INTENR_ERRIE                            ((uint32_t)0x00000800)

#define  I3C_INTENR_IBIIE                            ((uint32_t)0x00008000)
#define  I3C_INTENR_IBIENDIE                         ((uint32_t)0x00010000)
#define  I3C_INTENR_CRIE                             ((uint32_t)0x00020000)
#define  I3C_INTENR_CRUPDIE                          ((uint32_t)0x00040000)
#define  I3C_INTENR_HJIE                             ((uint32_t)0x00080000)

#define  I3C_INTENR_WKPIE                            ((uint32_t)0x00200000)
#define  I3C_INTENR_GETIE                            ((uint32_t)0x00400000)
#define  I3C_INTENR_STAIE                            ((uint32_t)0x00800000)
#define  I3C_INTENR_DAUPDIE                          ((uint32_t)0x01000000)
#define  I3C_INTENR_MWLUPDIE                         ((uint32_t)0x02000000)
#define  I3C_INTENR_MRLUPDIE                         ((uint32_t)0x04000000)
#define  I3C_INTENR_RSTIE                            ((uint32_t)0x08000000)
#define  I3C_INTENR_ASUPDIE                          ((uint32_t)0x10000000)
#define  I3C_INTENR_INTUPDIE                         ((uint32_t)0x20000000)
#define  I3C_INTENR_DEFIE                            ((uint32_t)0x40000000)
#define  I3C_INTENR_GRPIE                            ((uint32_t)0x80000000)

/*****************  Bit definition for I3C_CEVR register  ******************/
#define  I3C_CEVR_CFCR                               ((uint32_t)0x00000200)
#define  I3C_CEVR_CRXTGTENDF                         ((uint32_t)0x00000400)
#define  I3C_CEVR_CERRF                              ((uint32_t)0x00000800)

#define  I3C_CEVR_CIBIF                              ((uint32_t)0x00008000)
#define  I3C_CEVR_CIBIENDF                           ((uint32_t)0x00010000)
#define  I3C_CEVR_CCRF                               ((uint32_t)0x00020000)
#define  I3C_CEVR_CCRUPDF                            ((uint32_t)0x00040000)
#define  I3C_CEVR_CHJF                               ((uint32_t)0x00080000)

#define  I3C_CEVR_CWKPF                              ((uint32_t)0x00200000)
#define  I3C_CEVR_CGETF                              ((uint32_t)0x00400000)
#define  I3C_CEVR_CSTAF                              ((uint32_t)0x00800000)
#define  I3C_CEVR_CDAUPDF                            ((uint32_t)0x01000000)
#define  I3C_CEVR_CMWLUPDF                           ((uint32_t)0x02000000)
#define  I3C_CEVR_CMRLUPDF                           ((uint32_t)0x04000000)
#define  I3C_CEVR_CRSTF                              ((uint32_t)0x08000000)
#define  I3C_CEVR_CASUPDF                            ((uint32_t)0x10000000)
#define  I3C_CEVR_CINTUPDF                           ((uint32_t)0x20000000)
#define  I3C_CEVR_CDEFF                              ((uint32_t)0x40000000)
#define  I3C_CEVR_CGRPF                              ((uint32_t)0x80000000)

/*****************  Bit definition for I3C_DEVR0 register  ******************/
#define  I3C_DEVR0_DAVAL                             ((uint32_t)0x00000001)
#define  I3C_DEVR0_DA                                ((uint32_t)0x000000FE)

#define  I3C_DEVR0_IBIEN                             ((uint32_t)0x00010000)
#define  I3C_DEVR0_CREN                              ((uint32_t)0x00020000)

#define  I3C_DEVR0_HJEN                              ((uint32_t)0x00080000)

#define  I3C_DEVR0_AS                                ((uint32_t)0x00300000)
#define  I3C_DEVR0_AS_0                              ((uint32_t)0x00100000)
#define  I3C_DEVR0_AS_1                              ((uint32_t)0x00200000)

#define  I3C_DEVR0_RSTACT                            ((uint32_t)0x00C00000)
#define  I3C_DEVR0_RSTACT_0                          ((uint32_t)0x00400000)
#define  I3C_DEVR0_RSTACT_1                          ((uint32_t)0x00800000)

#define  I3C_DEVR0_RSTVAL                            ((uint32_t)0x01000000)

/*****************  Bit definition for I3C_DEVR1 register  ******************/
#define  I3C_DEVR1_DA                                ((uint32_t)0x000000FE)

#define  I3C_DEVR1_IBIACK                            ((uint32_t)0x00010000)
#define  I3C_DEVR1_CRACK                             ((uint32_t)0x00020000)
#define  I3C_DEVR1_IBIDEN                            ((uint32_t)0x00040000)
#define  I3C_DEVR1_SUSP                              ((uint32_t)0x00080000)

#define  I3C_DEVR1_DIS                               ((uint32_t)0x80000000)

/*****************  Bit definition for I3C_DEVR2 register  ******************/
#define  I3C_DEVR2_DA                                ((uint32_t)0x000000FE)

#define  I3C_DEVR2_IBIACK                            ((uint32_t)0x00010000)
#define  I3C_DEVR2_CRACK                             ((uint32_t)0x00020000)
#define  I3C_DEVR2_IBIDEN                            ((uint32_t)0x00040000)
#define  I3C_DEVR2_SUSP                              ((uint32_t)0x00080000)

#define  I3C_DEVR2_DIS                               ((uint32_t)0x80000000)

/*****************  Bit definition for I3C_DEVR3 register  ******************/
#define  I3C_DEVR3_DA                                ((uint32_t)0x000000FE)

#define  I3C_DEVR3_IBIACK                            ((uint32_t)0x00010000)
#define  I3C_DEVR3_CRACK                             ((uint32_t)0x00020000)
#define  I3C_DEVR3_IBIDEN                            ((uint32_t)0x00040000)
#define  I3C_DEVR3_SUSP                              ((uint32_t)0x00080000)

#define  I3C_DEVR3_DIS                               ((uint32_t)0x80000000)

/*****************  Bit definition for I3C_DEVR4 register  ******************/
#define  I3C_DEVR4_DA                                ((uint32_t)0x000000FE)

#define  I3C_DEVR4_IBIACK                            ((uint32_t)0x00010000)
#define  I3C_DEVR4_CRACK                             ((uint32_t)0x00020000)
#define  I3C_DEVR4_IBIDEN                            ((uint32_t)0x00040000)
#define  I3C_DEVR4_SUSP                              ((uint32_t)0x00080000)

#define  I3C_DEVR4_DIS                               ((uint32_t)0x80000000)

/*****************  Bit definition for I3C_MAXRLR register  ******************/
#define  I3C_MAXRLR_MRL                              ((uint32_t)0x0000FFFF)
      
#define  I3C_MAXRLR_IBIP                             ((uint32_t)0x00070000)
#define  I3C_MAXRLR_IBIP_0                           ((uint32_t)0x00010000)
#define  I3C_MAXRLR_IBIP_1                           ((uint32_t)0x00020000)
#define  I3C_MAXRLR_IBIP_2                           ((uint32_t)0x00040000)

/*****************  Bit definition for I3C_MAXWLR register  ******************/
#define  I3C_MAXWLR_MWL                              ((uint32_t)0x0000FFFF)

/*****************  Bit definition for I3C_TIMINGR0 register  ******************/
#define  I3C_TIMINGR0_SCLL_PP                        ((uint32_t)0x000000FF)
#define  I3C_TIMINGR0_SCLH_I3C                       ((uint32_t)0x0000FF00)
#define  I3C_TIMINGR0_SCLL_OD                        ((uint32_t)0x00FF0000)
#define  I3C_TIMINGR0_SCLH_I2C                       ((uint32_t)0xFF000000)

/*****************  Bit definition for I3C_TIMINGR1 register  ******************/
#define  I3C_TIMINGR1_AVAL                           ((uint32_t)0x000000FF)
#define  I3C_TIMINGR1_ASNCR                          ((uint32_t)0x00000300)

#define  I3C_TIMINGR1_FREE                           ((uint32_t)0x007F0000)
#define  I3C_TIMINGR1_SDA_HD                         ((uint32_t)0x10000000)

/*****************  Bit definition for I3C_TIMINGR2 register  ******************/
#define  I3C_TIMINGR2_STALLT                         ((uint32_t)0x00000001)
#define  I3C_TIMINGR2_STALLD                         ((uint32_t)0x00000002)
#define  I3C_TIMINGR2_STALLC                         ((uint32_t)0x00000004)
#define  I3C_TIMINGR2_STALLA                         ((uint32_t)0x00000008)

#define  I3C_TIMINGR2_STALL                          ((uint32_t)0x0000FF00)

/*****************  Bit definition for I3C_DCR register  ******************/
#define  I3C_DCR                                     ((uint32_t)0x000000FF)

/*****************  Bit definition for I3C_GETCAPR register  ******************/
#define  I3C_GETCAPR_CAPPEND                         ((uint32_t)0x00004000)

/*****************  Bit definition for I3C_CRCAPR register  ******************/
#define  I3C_CRCAPR_CAPDHOFF                         ((uint32_t)0x00000008)
#define  I3C_CRCAPR_CAPGRP                           ((uint32_t)0x00000200)

/*****************  Bit definition for I3C_GETMDSR register  ******************/
#define  I3C_GETMDSR_HOFFAS                          ((uint32_t)0x00000003)
#define  I3C_GETMDSR_HOFFAS_0                        ((uint32_t)0x00000001)
#define  I3C_GETMDSR_HOFFAS_1                        ((uint32_t)0x00000002)

#define  I3C_GETMDSR_FMT                             ((uint32_t)0x00000300)
#define  I3C_GETMDSR_FMT_0                           ((uint32_t)0x00000100)
#define  I3C_GETMDSR_FMT_1                           ((uint32_t)0x00000200)

#define  I3C_GETMDSR_RDTURN                          ((uint32_t)0x00FF0000)
#define  I3C_GETMDSR_TSCO                            ((uint32_t)0x01000000)

/*****************  Bit definition for I3C_EPIDR register  ******************/
#define  I3C_EPIDR_MIPIID                            ((uint32_t)0x0000F000)
#define  I3C_EPIDR_IDTSEL                            ((uint32_t)0x00010000)
#define  I3C_EPIDR_MIPIMID                           ((uint32_t)0xFFFE0000)

/******************************************************************************/
/*                              LOW POWER TIM                                 */
/******************************************************************************/
/*******************  Bit definition for LPTIM_ISR register  *******************/
#define LPTIM_ISR_CMPM                               ((uint32_t)0x00000001)
#define LPTIM_ISR_ARRM                               ((uint32_t)0x00000002)
#define LPTIM_ISR_EXTTRIG                            ((uint32_t)0x00000004)
#define LPTIM_ISR_CMPOK                              ((uint32_t)0x00000008)
#define LPTIM_ISR_ARROK                              ((uint32_t)0000000010)
#define LPTIM_ISR_UP                                 ((uint32_t)0x00000020)
#define LPTIM_ISR_DOWN                               ((uint32_t)0x00000040)
#define LPTIM_ISR_DIRSYNC                            ((uint32_t)0x00000080)

/*******************  Bit definition for LPTIM_ICR register  *******************/
#define LPTIM_ICR_CMPMCF                             ((uint32_t)0x00000001)
#define LPTIM_ICR_ARRMCF                             ((uint32_t)0x00000002)
#define LPTIM_ICR_EXTTRIGCF                          ((uint32_t)0x00000004)
#define LPTIM_ICR_CMPOKCF                            ((uint32_t)0x00000008)
#define LPTIM_ICR_ARROKCF                            ((uint32_t)0x00000010)
#define LPTIM_ICR_UPCF                               ((uint32_t)0x00000020)
#define LPTIM_ICR_DOWNCF                             ((uint32_t)0x00000040)

/*******************  Bit definition for LPTIM_IER register  *******************/
#define LPTIM_IER_CMPMIE                             ((uint32_t)0x00000001)
#define LPTIM_IER_ARRMIE                             ((uint32_t)0x00000002)
#define LPTIM_IER_EXTTRIGIE                          ((uint32_t)0x00000004)
#define LPTIM_IER_CMPOKIE                            ((uint32_t)0x00000008)
#define LPTIM_IER_ARROKIE                            ((uint32_t)0x00000010)
#define LPTIM_IER_UPIE                               ((uint32_t)0x00000020)
#define LPTIM_IER_DOWNIE                             ((uint32_t)0x00000040)

/*******************  Bit definition for LPTIM_CFGR register  *******************/
#define LPTIM_CFGR_CKSEL                             ((uint32_t)0x00000001)
#define LPTIM_CFGR_CKPOL                             ((uint32_t)0x00000006)
#define LPTIM_CFGR_CKFLT                             ((uint32_t)0x00000018)
#define LPTIM_CFGR_TRGFLT                            ((uint32_t)0x000000C0)
#define LPTIM_CFGR_PRESC                             ((uint32_t)0x00000E00)
#define LPTIM_CFGR_TRIGSEL                           ((uint32_t)0x00006000)
#define LPTIM_CFGR_TRIGEN                            ((uint32_t)0x00060000)
#define LPTIM_CFGR_TIMOUT                            ((uint32_t)0x00080000)
#define LPTIM_CFGR_WAVE                              ((uint32_t)0x00100000)
#define LPTIM_CFGR_WAVPOL                            ((uint32_t)0x00200000)
#define LPTIM_CFGR_PRELOAD                           ((uint32_t)0x00400000)
#define LPTIM_CFGR_CONTMODE                          ((uint32_t)0x00800000)
#define LPTIM_CFGR_ENC                               ((uint32_t)0x01000000)
#define LPTIM_CFGR_CLKSEL                            ((uint32_t)0x06000000)
#define LPTIM_CFGR_FORCEPWM                          ((uint32_t)0x08000000)

/*******************  Bit definition for LPTIM_CR register  *******************/
#define LPTIM_CR_ENABLE                              ((uint32_t)0x00000001)
#define LPTIM_CR_SNGSTRT                             ((uint32_t)0x00000002)
#define LPTIM_CR_CNTSTRT                             ((uint32_t)0x00000004)
#define LPTIM_CR_OUTEN                               ((uint32_t)0x00000008)
#define LPTIM_CR_DIR_EXTEN                           ((uint32_t)0x00000010)

/*******************  Bit definition for LPTIM_CMP register  *******************/
#define LPTIM_CMP                                    ((uint32_t)0x0000FFFF)

/*******************  Bit definition for LPTIM_ARR register  *******************/
#define LPTIM_ARR                                    ((uint32_t)0x0000FFFF)

/*******************  Bit definition for LPTIM_CNT register  *******************/
#define LPTIM_COUNT                                  ((uint32_t)0x0000FFFF)

/******************************************************************************/
/*                             Power Control                                  */
/******************************************************************************/

/********************  Bit definition for PWR_CTLR register  ********************/
#define  PWR_CTLR_LPDS                               ((uint16_t)0x0001)     /* Low-Power Deepsleep */
#define  PWR_CTLR_PVDE                               ((uint16_t)0x0010)     /* Power Voltage Detector Enable */

#define  PWR_CTLR_PLS                                ((uint16_t)0x00E0)     /* PLS[2:0] bits (PVD Level Selection) */
#define  PWR_CTLR_PLS_0                              ((uint16_t)0x0020)     /* Bit 0 */
#define  PWR_CTLR_PLS_1                              ((uint16_t)0x0040)     /* Bit 1 */
#define  PWR_CTLR_PLS_2                              ((uint16_t)0x0080)     /* Bit 2 */

#define  PWR_CTLR_PLS_MODE0                          ((uint16_t)0x0000)     
#define  PWR_CTLR_PLS_MODE1                          ((uint16_t)0x0020)     
#define  PWR_CTLR_PLS_MODE2                          ((uint16_t)0x0040)     
#define  PWR_CTLR_PLS_MODE3                          ((uint16_t)0x0060)     
#define  PWR_CTLR_PLS_MODE4                          ((uint16_t)0x0080)     
#define  PWR_CTLR_PLS_MODE5                          ((uint16_t)0x00A0)     
#define  PWR_CTLR_PLS_MODE6                          ((uint16_t)0x00C0)     
#define  PWR_CTLR_PLS_MODE7                          ((uint16_t)0x00E0)     

#define  PWR_CTLR_DBP                                ((uint16_t)0x0100)     /* Disable Backup Domain write protection */
#define  PWR_CTLR_VIO_SWCR                           ((uint16_t)0x0200)

#define  PWR_CTLR_VSEL_VIO18                         ((uint16_t)0x1C00)
#define  PWR_CTLR_VSEL_VIO18_0                       ((uint16_t)0x0400)
#define  PWR_CTLR_VSEL_VIO18_1                       ((uint16_t)0x0800)
#define  PWR_CTLR_VSEL_VIO18_2                       ((uint16_t)0x1000)

#define  PWR_CTLR_VSEL_VIO18_MODE0                   ((uint16_t)0x0000)
#define  PWR_CTLR_VSEL_VIO18_MODE1                   ((uint16_t)0x0400)
#define  PWR_CTLR_VSEL_VIO18_MODE2                   ((uint16_t)0x0800)
#define  PWR_CTLR_VSEL_VIO18_MODE3                   ((uint16_t)0x0C00)
#define  PWR_CTLR_VSEL_VIO18_MODE4                   ((uint16_t)0x1000)
#define  PWR_CTLR_VSEL_VIO18_MODE5                   ((uint16_t)0x1400)
#define  PWR_CTLR_VSEL_VIO18_MODE6                   ((uint16_t)0x1800)
#define  PWR_CTLR_VSEL_VIO18_MODE7                   ((uint16_t)0x1C00)

/*******************  Bit definition for PWR_CSR register  ********************/
#define  PWR_CSR_PVDO                                ((uint16_t)0x0001)

#define  PWR_CSR_VIO18_SR                            ((uint16_t)0x0300)
#define  PWR_CSR_VIO18_SR_0                          ((uint16_t)0x0100)
#define  PWR_CSR_VIO18_SR_1                          ((uint16_t)0x0200)

/******************************************************************************/
/*                         Reset and Clock Control                            */
/******************************************************************************/

/********************  Bit definition for RCC_CTLR register  ********************/
#define  RCC_HSION                                   ((uint32_t)0x00000001)        /* Internal High Speed clock enable */
#define  RCC_HSIRDY                                  ((uint32_t)0x00000002)        /* Internal High Speed clock ready flag */
#define  RCC_HSITRIM                                 ((uint32_t)0x000000F8)        /* Internal High Speed clock trimming */
#define  RCC_HSICAL                                  ((uint32_t)0x0000FF00)        /* Internal High Speed clock Calibration */
#define  RCC_HSEON                                   ((uint32_t)0x00010000)        /* External High Speed clock enable */
#define  RCC_HSERDY                                  ((uint32_t)0x00020000)        /* External High Speed clock ready flag */
#define  RCC_HSEBYP                                  ((uint32_t)0x00040000)        /* External High Speed clock Bypass */
#define  RCC_CSSON                                   ((uint32_t)0x00080000)        /* Clock Security System enable */
#define  RCC_USBHS_PLLON                             ((uint32_t)0x00100000)
#define  RCC_USBHS_PLLRDY                            ((uint32_t)0x00200000)
#define  RCC_USBSS_PLLON                             ((uint32_t)0x00400000)
#define  RCC_USBSS_PLLRDY                            ((uint32_t)0x00800000)
#define  RCC_PLLON                                   ((uint32_t)0x01000000)        /* PLL enable */
#define  RCC_PLLRDY                                  ((uint32_t)0x02000000)        /* PLL clock ready flag */
#define  RCC_ETH_PLLON                               ((uint32_t)0x04000000)
#define  RCC_ETH_PLLRDY                              ((uint32_t)0x08000000)
#define  RCC_SERDES_PLLON                            ((uint32_t)0x10000000)
#define  RCC_SERDES_PLLRDY                           ((uint32_t)0x20000000)
#define  RCC_CSS_HSE_DIS                             ((uint32_t)0x80000000)
            
/*******************  Bit definition for RCC_CFGR0 register  *******************/
#define  RCC_SW                                      ((uint32_t)0x00000003)        /* SW[1:0] bits (System clock Switch) */
#define  RCC_SW_0                                    ((uint32_t)0x00000001)        /* Bit 0 */
#define  RCC_SW_1                                    ((uint32_t)0x00000002)        /* Bit 1 */

#define  RCC_SW_HSI                                  ((uint32_t)0x00000000)        /* HSI selected as system clock */
#define  RCC_SW_HSE                                  ((uint32_t)0x00000001)        /* HSE selected as system clock */
#define  RCC_SW_PLL                                  ((uint32_t)0x00000002)        /* PLL selected as system clock */

#define  RCC_SWS                                     ((uint32_t)0x0000000C)        /* SWS[1:0] bits (System Clock Switch Status) */
#define  RCC_SWS_0                                   ((uint32_t)0x00000004)        /* Bit 0 */
#define  RCC_SWS_1                                   ((uint32_t)0x00000008)        /* Bit 1 */

#define  RCC_SWS_HSI                                 ((uint32_t)0x00000000)        /* HSI oscillator used as system clock */
#define  RCC_SWS_HSE                                 ((uint32_t)0x00000004)        /* HSE oscillator used as system clock */
#define  RCC_SWS_PLL                                 ((uint32_t)0x00000008)        /* PLL used as system clock */

#define  RCC_HPRE                                    ((uint32_t)0x000000F0)        /* HPRE[3:0] bits (AHB prescaler) */
#define  RCC_HPRE_0                                  ((uint32_t)0x00000010)        /* Bit 0 */
#define  RCC_HPRE_1                                  ((uint32_t)0x00000020)        /* Bit 1 */
#define  RCC_HPRE_2                                  ((uint32_t)0x00000040)        /* Bit 2 */
#define  RCC_HPRE_3                                  ((uint32_t)0x00000080)        /* Bit 3 */

#define  RCC_HPRE_DIV1                               ((uint32_t)0x00000000)        /* SYSCLK not divided */
#define  RCC_HPRE_DIV2                               ((uint32_t)0x00000080)        /* SYSCLK divided by 2 */
#define  RCC_HPRE_DIV4                               ((uint32_t)0x00000090)        /* SYSCLK divided by 4 */
#define  RCC_HPRE_DIV8                               ((uint32_t)0x000000A0)        /* SYSCLK divided by 8 */
#define  RCC_HPRE_DIV16                              ((uint32_t)0x000000B0)        /* SYSCLK divided by 16 */
#define  RCC_HPRE_DIV64                              ((uint32_t)0x000000C0)        /* SYSCLK divided by 64 */
#define  RCC_HPRE_DIV128                             ((uint32_t)0x000000D0)        /* SYSCLK divided by 128 */
#define  RCC_HPRE_DIV256                             ((uint32_t)0x000000E0)        /* SYSCLK divided by 256 */
#define  RCC_HPRE_DIV512                             ((uint32_t)0x000000F0)        /* SYSCLK divided by 512 */

#define  RCC_PPRE1                                   ((uint32_t)0x00000700)        /* PRE1[2:0] bits (APB1 prescaler) */
#define  RCC_PPRE1_0                                 ((uint32_t)0x00000100)        /* Bit 0 */
#define  RCC_PPRE1_1                                 ((uint32_t)0x00000200)        /* Bit 1 */
#define  RCC_PPRE1_2                                 ((uint32_t)0x00000400)        /* Bit 2 */

#define  RCC_PPRE1_DIV1                              ((uint32_t)0x00000000)        /* HCLK not divided */
#define  RCC_PPRE1_DIV2                              ((uint32_t)0x00000500)        /* HCLK divided by 2 */
#define  RCC_PPRE1_DIV4                              ((uint32_t)0x00000600)        /* HCLK divided by 4 */
#define  RCC_PPRE1_DIV8                              ((uint32_t)0x00000700)        /* HCLK divided by 8 */

#define  RCC_PPRE2                                   ((uint32_t)0x00003800)        /* PRE2[2:0] bits (APB2 prescaler) */
#define  RCC_PPRE2_0                                 ((uint32_t)0x00000800)        /* Bit 0 */
#define  RCC_PPRE2_1                                 ((uint32_t)0x00001000)        /* Bit 1 */
#define  RCC_PPRE2_2                                 ((uint32_t)0x00002000)        /* Bit 2 */

#define  RCC_PPRE2_TIM_DIV1                          ((uint32_t)0x00000000)        /* HCLK divided by 1 for TIM1,8,9,10*/
#define  RCC_PPRE2_TIM_DIV2                          ((uint32_t)0x00002800)        /* HCLK divided by 2 for TIM1,8,9,10*/
#define  RCC_PPRE2_TIM_DIV4                          ((uint32_t)0x00003000)        /* HCLK divided by 4 for TIM1,8,9,10*/
#define  RCC_PPRE2_TIM_DIV8                          ((uint32_t)0x00003800)        /* HCLK divided by 8 for TIM1,8,9,10*/

#define  RCC_PPRE2_ADCL_DIV1                         ((uint32_t)0x00000000)        /* HCLK divided by 1 for ADC*/
#define  RCC_PPRE2_ADCL_DIV2                         ((uint32_t)0x00002000)        /* HCLK divided by 2 for ADC*/
#define  RCC_PPRE2_ADCL_DIV4                         ((uint32_t)0x00002800)        /* HCLK divided by 4 for ADC*/
#define  RCC_PPRE2_ADCL_DIV8                         ((uint32_t)0x00003000)        /* HCLK divided by 8 for ADC*/
#define  RCC_PPRE2_ADCL_DIV16                        ((uint32_t)0x00003800)        /* HCLK divided by 16 for ADC*/

#define  RCC_ADCPRE                                  ((uint32_t)0x0000C000)        /* ADCPRE[1:0] bits (ADC prescaler) */
#define  RCC_ADCPRE_0                                ((uint32_t)0x00004000)        /* Bit 0 */
#define  RCC_ADCPRE_1                                ((uint32_t)0x00008000)        /* Bit 1 */

#define  RCC_ADCPRE_ADCH_DIV2                        ((uint32_t)0x00000000)        /* HCLK divided by 2 for ADC*/
#define  RCC_ADCPRE_ADCH_DIV4                        ((uint32_t)0x00001000)        /* HCLK divided by 4 for ADC*/
#define  RCC_ADCPRE_ADCH_DIV6                        ((uint32_t)0x00002000)        /* HCLK divided by 6 for ADC*/
#define  RCC_ADCPRE_ADCH_DIV8                        ((uint32_t)0x00003000)        /* HCLK divided by 8 for ADC*/

#define  RCC_ADCPRE_DIV5                             ((uint32_t)0x00000000)        /* USBHS PLL divided by 5 for ADC */
#define  RCC_ADCPRE_DIV6                             ((uint32_t)0x00000800)        /* USBHS PLL divided by 6 for ADC */
#define  RCC_ADCPRE_DIV7                             ((uint32_t)0x00001000)        /* USBHS PLL divided by 7 for ADC */
#define  RCC_ADCPRE_DIV8                             ((uint32_t)0x00001800)        /* USBHS PLL divided by 8 for ADC */
#define  RCC_ADCPRE_DIV9                             ((uint32_t)0x00002000)        /* USBHS PLL divided by 9 for ADC */
#define  RCC_ADCPRE_DIV10                            ((uint32_t)0x00002800)        /* USBHS PLL divided by 10 for ADC */
#define  RCC_ADCPRE_DIV11                            ((uint32_t)0x00003000)        /* USBHS PLL divided by 11 for ADC */
#define  RCC_ADCPRE_DIV12                            ((uint32_t)0x00003800)        /* USBHS PLL divided by 12 for ADC */
#define  RCC_ADCPRE_DIV13                            ((uint32_t)0x00004000)        /* USBHS PLL divided by 13 for ADC */
#define  RCC_ADCPRE_DIV14                            ((uint32_t)0x00004800)        /* USBHS PLL divided by 14 for ADC */
#define  RCC_ADCPRE_DIV15                            ((uint32_t)0x00005000)        /* USBHS PLL divided by 15 for ADC */
#define  RCC_ADCPRE_DIV16                            ((uint32_t)0x00005800)        /* USBHS PLL divided by 16 for ADC */
#define  RCC_ADCPRE_DIV17                            ((uint32_t)0x00006000)        /* USBHS PLL divided by 17 for ADC */
#define  RCC_ADCPRE_DIV18                            ((uint32_t)0x00006800)        /* USBHS PLL divided by 18 for ADC */
#define  RCC_ADCPRE_DIV19                            ((uint32_t)0x00007000)        /* USBHS PLL divided by 19 for ADC */
#define  RCC_ADCPRE_DIV20                            ((uint32_t)0x00007800)        /* USBHS PLL divided by 20 for ADC */
#define  RCC_ADCPRE_DIV21                            ((uint32_t)0x00008000)        /* USBHS PLL divided by 21 for ADC */
#define  RCC_ADCPRE_DIV22                            ((uint32_t)0x00008800)        /* USBHS PLL divided by 22 for ADC */
#define  RCC_ADCPRE_DIV23                            ((uint32_t)0x00009000)        /* USBHS PLL divided by 23 for ADC */
#define  RCC_ADCPRE_DIV24                            ((uint32_t)0x00009800)        /* USBHS PLL divided by 24 for ADC */
#define  RCC_ADCPRE_DIV25                            ((uint32_t)0x0000A000)        /* USBHS PLL divided by 25 for ADC */
#define  RCC_ADCPRE_DIV26                            ((uint32_t)0x0000A800)        /* USBHS PLL divided by 26 for ADC */
#define  RCC_ADCPRE_DIV27                            ((uint32_t)0x0000B000)        /* USBHS PLL divided by 27 for ADC */
#define  RCC_ADCPRE_DIV28                            ((uint32_t)0x0000B800)        /* USBHS PLL divided by 28 for ADC */
#define  RCC_ADCPRE_DIV29                            ((uint32_t)0x0000C000)        /* USBHS PLL divided by 29 for ADC */
#define  RCC_ADCPRE_DIV30                            ((uint32_t)0x0000C800)        /* USBHS PLL divided by 30 for ADC */
#define  RCC_ADCPRE_DIV31                            ((uint32_t)0x0000D000)        /* USBHS PLL divided by 31 for ADC */
#define  RCC_ADCPRE_DIV32                            ((uint32_t)0x0000D800)        /* USBHS PLL divided by 32 for ADC */
#define  RCC_ADCPRE_DIV33                            ((uint32_t)0x0000E000)        /* USBHS PLL divided by 33 for ADC */
#define  RCC_ADCPRE_DIV34                            ((uint32_t)0x0000E800)        /* USBHS PLL divided by 34 for ADC */
#define  RCC_ADCPRE_DIV35                            ((uint32_t)0x0000F000)        /* USBHS PLL divided by 35 for ADC */
#define  RCC_ADCPRE_DIV36                            ((uint32_t)0x0000F800)        /* USBHS PLL divided by 36 for ADC */

#define  RCC_FPRE                                    ((uint32_t)0x00030000)
#define  RCC_FPRE_0                                  ((uint32_t)0x00010000)
#define  RCC_FPRE_1                                  ((uint32_t)0x00020000)

#define  RCC_FPRE_DIV1                               ((uint32_t)0x00000000)
#define  RCC_FPRE_DIV2                               ((uint32_t)0x00010000)
#define  RCC_FPRE_DIV4                               ((uint32_t)0x00020000)

#define  RCC_RGMIION                                 ((uint32_t)0x00200000)

#define  RCC_PIPEON                                  ((uint32_t)0x00400000)
#define  RCC_UTMION                                  ((uint32_t)0x00800000)

#define  RCC_CFGR0_MCO                               ((uint32_t)0x0F000000)
#define  RCC_MCO_0                                   ((uint32_t)0x01000000)
#define  RCC_MCO_1                                   ((uint32_t)0x02000000)
#define  RCC_MCO_2                                   ((uint32_t)0x04000000)
#define  RCC_MCO_3                                   ((uint32_t)0x08000000)

#define  RCC_CFGR0_MCO_NOCLOCK                       ((uint32_t)0x00000000)
#define  RCC_CFGR0_MCO_SYSCLK                        ((uint32_t)0x04000000)
#define  RCC_CFGR0_MCO_HSI                           ((uint32_t)0x05000000)
#define  RCC_CFGR0_MCO_HSE                           ((uint32_t)0x06000000)
#define  RCC_CFGR0_MCO_PLL_DIV2                      ((uint32_t)0x07000000)
#define  RCC_CFGR0_MCO_UTMI                          ((uint32_t)0x08000000)
#define  RCC_CFGR0_MCO_USBSS_PLL_DIV2                ((uint32_t)0x09000000)
#define  RCC_CFGR0_MCO_ETH_PLL_DIV8                  ((uint32_t)0x0A000000)
#define  RCC_CFGR0_MCO_SERDES_DIV16                  ((uint32_t)0x0B000000)

#define  RCC_ADC_DUTY_SEL                            ((uint32_t)0x40000000)
#define  RCC_ADCSRC                                  ((uint32_t)0x80000000)

/*******************  Bit definition for RCC_PLLCFGR register  *******************/
#define  RCC_PLLMUL                                  ((uint32_t)0x0000001F)
#define  RCC_PLLMUL_0                                ((uint32_t)0x00000001)
#define  RCC_PLLMUL_1                                ((uint32_t)0x00000002)
#define  RCC_PLLMUL_2                                ((uint32_t)0x00000004)
#define  RCC_PLLMUL_3                                ((uint32_t)0x00000008)
#define  RCC_PLLMUL_4                                ((uint32_t)0x00000010)

#define  RCC_PLLMUL4                                 ((uint32_t)0x00000000)
#define  RCC_PLLMUL6                                 ((uint32_t)0x00000001)
#define  RCC_PLLMUL7                                 ((uint32_t)0x00000002)
#define  RCC_PLLMUL8                                 ((uint32_t)0x00000003)
#define  RCC_PLLMUL8_5                               ((uint32_t)0x00000004)
#define  RCC_PLLMUL9                                 ((uint32_t)0x00000005)
#define  RCC_PLLMUL9_5                               ((uint32_t)0x00000006)
#define  RCC_PLLMUL10                                ((uint32_t)0x00000007)
#define  RCC_PLLMUL10_5                              ((uint32_t)0x00000008)
#define  RCC_PLLMUL11                                ((uint32_t)0x00000009)
#define  RCC_PLLMUL11_5                              ((uint32_t)0x0000000A)
#define  RCC_PLLMUL12                                ((uint32_t)0x0000000B)
#define  RCC_PLLMUL12_5                              ((uint32_t)0x0000000C)
#define  RCC_PLLMUL13                                ((uint32_t)0x0000000D)
#define  RCC_PLLMUL14                                ((uint32_t)0x0000000E)
#define  RCC_PLLMUL15                                ((uint32_t)0x0000000F)
#define  RCC_PLLMUL16                                ((uint32_t)0x00000010)
#define  RCC_PLLMUL17                                ((uint32_t)0x00000011)
#define  RCC_PLLMUL18                                ((uint32_t)0x00000012)
#define  RCC_PLLMUL19                                ((uint32_t)0x00000013)
#define  RCC_PLLMUL20                                ((uint32_t)0x00000014)
#define  RCC_PLLMUL22                                ((uint32_t)0x00000015)
#define  RCC_PLLMUL24                                ((uint32_t)0x00000016)
#define  RCC_PLLMUL26                                ((uint32_t)0x00000017)
#define  RCC_PLLMUL28                                ((uint32_t)0x00000018)
#define  RCC_PLLMUL30                                ((uint32_t)0x00000019)
#define  RCC_PLLMUL32                                ((uint32_t)0x0000001A)
#define  RCC_PLLMUL34                                ((uint32_t)0x0000001B)
#define  RCC_PLLMUL36                                ((uint32_t)0x0000001C)
#define  RCC_PLLMUL38                                ((uint32_t)0x0000001D)
#define  RCC_PLLMUL40                                ((uint32_t)0x0000001E)
#define  RCC_PLLMUL59                                ((uint32_t)0x0000001F)

#define  RCC_PLLSRC                                  ((uint32_t)0x000000E0)
#define  RCC_PLLSRC_0                                ((uint32_t)0x00000020)
#define  RCC_PLLSRC_2                                ((uint32_t)0x00000040)
#define  RCC_PLLSRC_4                                ((uint32_t)0x00000080)

#define  RCC_PLLSRC_HSI                              ((uint32_t)0x00000000)
#define  RCC_PLLSRC_HSE                              ((uint32_t)0x00000020)
#define  RCC_PLLSRC_USBHS_PLL                        ((uint32_t)0x00000080)
#define  RCC_PLLSRC_ETH_PLL                          ((uint32_t)0x000000A0)
#define  RCC_PLLSRC_USBSS_PLL                        ((uint32_t)0x000000C0)
#define  RCC_PLLSRC_SERDES_PLL                       ((uint32_t)0x000000E0)

#define  RCC_PLL_SRC_DIV                             ((uint32_t)0x00003F00)
#define  RCC_PLL_SRC_DIV_0                           ((uint32_t)0x00000100)
#define  RCC_PLL_SRC_DIV_1                           ((uint32_t)0x00000200)
#define  RCC_PLL_SRC_DIV_2                           ((uint32_t)0x00000400)
#define  RCC_PLL_SRC_DIV_3                           ((uint32_t)0x00000800)
#define  RCC_PLL_SRC_DIV_4                           ((uint32_t)0x00001000)
#define  RCC_PLL_SRC_DIV_5                           ((uint32_t)0x00002000)

#define  RCC_PLL_SRC_DIV1                            ((uint32_t)0x00000000)
#define  RCC_PLL_SRC_DIV2                            ((uint32_t)0x00000100)
#define  RCC_PLL_SRC_DIV3                            ((uint32_t)0x00000200)
#define  RCC_PLL_SRC_DIV4                            ((uint32_t)0x00000300)
#define  RCC_PLL_SRC_DIV5                            ((uint32_t)0x00000400)
#define  RCC_PLL_SRC_DIV6                            ((uint32_t)0x00000500)
#define  RCC_PLL_SRC_DIV7                            ((uint32_t)0x00000600)
#define  RCC_PLL_SRC_DIV8                            ((uint32_t)0x00000700)
#define  RCC_PLL_SRC_DIV9                            ((uint32_t)0x00000800)
#define  RCC_PLL_SRC_DIV10                           ((uint32_t)0x00000900)
#define  RCC_PLL_SRC_DIV11                           ((uint32_t)0x00000A00)
#define  RCC_PLL_SRC_DIV12                           ((uint32_t)0x00000B00)
#define  RCC_PLL_SRC_DIV13                           ((uint32_t)0x00000C00)
#define  RCC_PLL_SRC_DIV14                           ((uint32_t)0x00000D00)
#define  RCC_PLL_SRC_DIV15                           ((uint32_t)0x00000E00)
#define  RCC_PLL_SRC_DIV16                           ((uint32_t)0x00000F00)
#define  RCC_PLL_SRC_DIV17                           ((uint32_t)0x00001000)
#define  RCC_PLL_SRC_DIV18                           ((uint32_t)0x00001100)
#define  RCC_PLL_SRC_DIV19                           ((uint32_t)0x00001200)
#define  RCC_PLL_SRC_DIV20                           ((uint32_t)0x00001300)
#define  RCC_PLL_SRC_DIV21                           ((uint32_t)0x00001400)
#define  RCC_PLL_SRC_DIV22                           ((uint32_t)0x00001500)
#define  RCC_PLL_SRC_DIV23                           ((uint32_t)0x00001600)
#define  RCC_PLL_SRC_DIV24                           ((uint32_t)0x00001700)
#define  RCC_PLL_SRC_DIV25                           ((uint32_t)0x00001800)
#define  RCC_PLL_SRC_DIV26                           ((uint32_t)0x00001900)
#define  RCC_PLL_SRC_DIV27                           ((uint32_t)0x00001A00)
#define  RCC_PLL_SRC_DIV28                           ((uint32_t)0x00001B00)
#define  RCC_PLL_SRC_DIV29                           ((uint32_t)0x00001C00)
#define  RCC_PLL_SRC_DIV30                           ((uint32_t)0x00001D00)
#define  RCC_PLL_SRC_DIV31                           ((uint32_t)0x00001E00)
#define  RCC_PLL_SRC_DIV32                           ((uint32_t)0x00001F00)
#define  RCC_PLL_SRC_DIV33                           ((uint32_t)0x00002000)
#define  RCC_PLL_SRC_DIV34                           ((uint32_t)0x00002100)
#define  RCC_PLL_SRC_DIV35                           ((uint32_t)0x00002200)
#define  RCC_PLL_SRC_DIV36                           ((uint32_t)0x00002300)
#define  RCC_PLL_SRC_DIV37                           ((uint32_t)0x00002400)
#define  RCC_PLL_SRC_DIV38                           ((uint32_t)0x00002500)
#define  RCC_PLL_SRC_DIV39                           ((uint32_t)0x00002600)
#define  RCC_PLL_SRC_DIV40                           ((uint32_t)0x00002700)
#define  RCC_PLL_SRC_DIV41                           ((uint32_t)0x00002800)
#define  RCC_PLL_SRC_DIV42                           ((uint32_t)0x00002900)
#define  RCC_PLL_SRC_DIV43                           ((uint32_t)0x00002A00)
#define  RCC_PLL_SRC_DIV44                           ((uint32_t)0x00002B00)
#define  RCC_PLL_SRC_DIV45                           ((uint32_t)0x00002C00)
#define  RCC_PLL_SRC_DIV46                           ((uint32_t)0x00002D00)
#define  RCC_PLL_SRC_DIV47                           ((uint32_t)0x00002E00)
#define  RCC_PLL_SRC_DIV48                           ((uint32_t)0x00002F00)
#define  RCC_PLL_SRC_DIV49                           ((uint32_t)0x00003000)
#define  RCC_PLL_SRC_DIV50                           ((uint32_t)0x00003100)
#define  RCC_PLL_SRC_DIV51                           ((uint32_t)0x00003200)
#define  RCC_PLL_SRC_DIV52                           ((uint32_t)0x00003300)
#define  RCC_PLL_SRC_DIV53                           ((uint32_t)0x00003400)
#define  RCC_PLL_SRC_DIV54                           ((uint32_t)0x00003500)
#define  RCC_PLL_SRC_DIV55                           ((uint32_t)0x00003600)
#define  RCC_PLL_SRC_DIV56                           ((uint32_t)0x00003700)
#define  RCC_PLL_SRC_DIV57                           ((uint32_t)0x00003800)
#define  RCC_PLL_SRC_DIV58                           ((uint32_t)0x00003900)
#define  RCC_PLL_SRC_DIV59                           ((uint32_t)0x00003A00)
#define  RCC_PLL_SRC_DIV60                           ((uint32_t)0x00003B00)
#define  RCC_PLL_SRC_DIV61                           ((uint32_t)0x00003C00)
#define  RCC_PLL_SRC_DIV62                           ((uint32_t)0x00003D00)
#define  RCC_PLL_SRC_DIV63                           ((uint32_t)0x00003E00)
#define  RCC_PLL_SRC_DIV64                           ((uint32_t)0x00003F00)

#define  RCC_SYSPLL_SEL                              ((uint32_t)0x70000000)
#define  RCC_SYSPLL_SEL_0                            ((uint32_t)0x10000000)
#define  RCC_SYSPLL_SEL_1                            ((uint32_t)0x20000000)
#define  RCC_SYSPLL_SEL_2                            ((uint32_t)0x40000000)

#define  RCC_SYSPLL_PLL                              ((uint32_t)0x00000000)
#define  RCC_SYSPLL_USBHS                            ((uint32_t)0x40000000)
#define  RCC_SYSPLL_ETH                              ((uint32_t)0x50000000)
#define  RCC_SYSPLL_SERDES                           ((uint32_t)0x60000000)
#define  RCC_SYSPLL_USBSS                            ((uint32_t)0x70000000)

#define  RCC_SYSPLL_GATE                             ((uint32_t)0x80000000)

/*******************  Bit definition for RCC_INTR register  ********************/
#define  RCC_LSIRDYF                                 ((uint32_t)0x00000001)        /* LSI Ready Interrupt flag */
#define  RCC_LSERDYF                                 ((uint32_t)0x00000002)        /* LSE Ready Interrupt flag */
#define  RCC_HSIRDYF                                 ((uint32_t)0x00000004)        /* HSI Ready Interrupt flag */
#define  RCC_HSERDYF                                 ((uint32_t)0x00000008)        /* HSE Ready Interrupt flag */
#define  RCC_PLLRDYF                                 ((uint32_t)0x00000010)        /* PLL Ready Interrupt flag */
#define  RCC_ETHPLLRDYF                              ((uint32_t)0x00000020)
#define  RCC_SERDESPLLRDYF                           ((uint32_t)0x00000040)
#define  RCC_CSSF                                    ((uint32_t)0x00000080)        /* Clock Security System Interrupt flag */
#define  RCC_LSIRDYIE                                ((uint32_t)0x00000100)        /* LSI Ready Interrupt Enable */
#define  RCC_LSERDYIE                                ((uint32_t)0x00000200)        /* LSE Ready Interrupt Enable */
#define  RCC_HSIRDYIE                                ((uint32_t)0x00000400)        /* HSI Ready Interrupt Enable */
#define  RCC_HSERDYIE                                ((uint32_t)0x00000800)        /* HSE Ready Interrupt Enable */
#define  RCC_PLLRDYIE                                ((uint32_t)0x00001000)        /* PLL Ready Interrupt Enable */
#define  RCC_ETHPLLRDYIE                             ((uint32_t)0x00002000)
#define  RCC_SERDESPLLRDYIE                          ((uint32_t)0x00004000)
#define  RCC_LSIRDYC                                 ((uint32_t)0x00010000)        /* LSI Ready Interrupt Clear */
#define  RCC_LSERDYC                                 ((uint32_t)0x00020000)        /* LSE Ready Interrupt Clear */
#define  RCC_HSIRDYC                                 ((uint32_t)0x00040000)        /* HSI Ready Interrupt Clear */
#define  RCC_HSERDYC                                 ((uint32_t)0x00080000)        /* HSE Ready Interrupt Clear */
#define  RCC_PLLRDYC                                 ((uint32_t)0x00100000)        /* PLL Ready Interrupt Clear */
#define  RCC_ETHPLLRDYC                              ((uint32_t)0x00200000)
#define  RCC_SERDESPLLRDYC                           ((uint32_t)0x00400000)
#define  RCC_CSSC                                    ((uint32_t)0x00800000)        /* Clock Security System Interrupt Clear */

/*****************  Bit definition for RCC_HB2PRSTR register  *****************/
#define  RCC_AFIORST                                 ((uint32_t)0x00000001)        /* Alternate Function I/O reset */
#define  RCC_HSADCRST                                ((uint32_t)0x00000002)
#define  RCC_IOPARST                                 ((uint32_t)0x00000004)        /* I/O port A reset */
#define  RCC_IOPBRST                                 ((uint32_t)0x00000008)        /* I/O port B reset */
#define  RCC_IOPCRST                                 ((uint32_t)0x00000010)        /* I/O port C reset */
#define  RCC_IOPDRST                                 ((uint32_t)0x00000020)        /* I/O port D reset */
#define  RCC_IOPERST                                 ((uint32_t)0x00000040)
#define  RCC_IOPFRST                                 ((uint32_t)0x00000080)

#define  RCC_ADC1RST                                 ((uint32_t)0x00000200)        /* ADC 1 interface reset */
#define  RCC_ADC2RST                                 ((uint32_t)0x00000400)        /* ADC 2 interface reset */
#define  RCC_TIM1RST                                 ((uint32_t)0x00000800)        /* TIM1 Timer reset */
#define  RCC_SPI1RST                                 ((uint32_t)0x00001000)        /* SPI 1 reset */
#define  RCC_TIM8RST                                 ((uint32_t)0x00002000)
#define  RCC_USART1RST                               ((uint32_t)0x00004000)        /* USART1 reset */
#define  RCC_I2C4RST                                 ((uint32_t)0x00008000)
#define  RCC_SAIRST                                  ((uint32_t)0x00010000)
#define  RCC_SDIORST                                 ((uint32_t)0x00040000)
#define  RCC_TIM9RST                                 ((uint32_t)0x00080000)
#define  RCC_TIM10RST                                ((uint32_t)0x00100000)
#define  RCC_TIM11RST                                ((uint32_t)0x00200000)
#define  RCC_TIM12RST                                ((uint32_t)0x00400000)
#define  RCC_OPCMRST                                 ((uint32_t)0x00800000)
#define  RCC_DFSDMRST                                ((uint32_t)0x02000000)
#define  RCC_ECDCRST                                 ((uint32_t)0x04000000)
#define  RCC_GPHARST                                 ((uint32_t)0x08000000)
#define  RCC_LTDCRST                                 ((uint32_t)0x40000000)
#define  RCC_I3CRST                                  ((uint32_t)0x80000000)

/*****************  Bit definition for RCC_HB1PRSTR register  *****************/
#define  RCC_TIM2RST                                 ((uint32_t)0x00000001)        /* Timer 2 reset */
#define  RCC_TIM3RST                                 ((uint32_t)0x00000002)        /* Timer 3 reset */
#define  RCC_TIM4RST                                 ((uint32_t)0x00000004)        /* Timer 4 reset */
#define  RCC_TIM5RST                                 ((uint32_t)0x00000008)        /* Timer 5 reset */
#define  RCC_TIM6RST                                 ((uint32_t)0x00000010)        /* Timer 6 reset */
#define  RCC_TIM7RST                                 ((uint32_t)0x00000020)        /* Timer 7 reset */
#define  RCC_USART6RST                               ((uint32_t)0x00000040)        /* USART 2 reset */
#define  RCC_USART7RST                               ((uint32_t)0x00000080)        /* USART 2 reset */
#define  RCC_USART8RST                               ((uint32_t)0x00000100)        /* USART 2 reset */
#define  RCC_LPTIM1RST                               ((uint32_t)0x00000200)
#define  RCC_LPTIM2RST                               ((uint32_t)0x00000400)
#define  RCC_WWDGRST                                 ((uint32_t)0x00000800)        /* Window Watchdog reset */
#define  RCC_QSPI1RST                                ((uint32_t)0x00001000)
#define  RCC_QSPI2RST                                ((uint32_t)0x00002000)
#define  RCC_SPI2RST                                 ((uint32_t)0x00004000)        /* SPI 2 reset */
#define  RCC_SPI3RST                                 ((uint32_t)0x00008000)        /* SPI 3 reset */
#define  RCC_SPI4RST                                 ((uint32_t)0x00010000)
#define  RCC_USART2RST                               ((uint32_t)0x00020000)        /* USART 2 reset */
#define  RCC_USART3RST                               ((uint32_t)0x00040000)        /* USART 3 reset */
#define  RCC_USART4RST                               ((uint32_t)0x00080000)        /* USART 4 reset */
#define  RCC_USART5RST                               ((uint32_t)0x00100000)        /* USART 5 reset */
#define  RCC_I2C1RST                                 ((uint32_t)0x00200000)        /* I2C 1 reset */
#define  RCC_I2C2RST                                 ((uint32_t)0x00400000)        /* I2C 2 reset */ 
#define  RCC_CAN3RST                                 ((uint32_t)0x01000000)
#define  RCC_CAN1RST                                 ((uint32_t)0x02000000)        /* CAN1 reset */
#define  RCC_CAN2RST                                 ((uint32_t)0x04000000)        /* CAN2 reset */
#define  RCC_BKPRST                                  ((uint32_t)0x08000000)        /* Backup interface reset */
#define  RCC_PWRRST                                  ((uint32_t)0x10000000)        /* Power interface reset */
#define  RCC_DACRST                                  ((uint32_t)0x20000000)        /* DAC reset */
#define  RCC_I2C3RST                                 ((uint32_t)0x40000000)
#define  RCC_SWPMIRST                                ((uint32_t)0x80000000)

/******************  Bit definition for RCC_HBPCENR register  ******************/
#define  RCC_DMA1EN                                  ((uint16_t)0x0001)            /* DMA1 clock enable */
#define  RCC_DMA2EN                                  ((uint16_t)0x0002)
#define  RCC_CRCEN                                   ((uint16_t)0x0040)            /* CRC clock enable */
#define  RCC_FMCEN                                   ((uint16_t)0x0100)
#define  RCC_RNGEN                                   ((uint16_t)0x0200)
#define  RCC_SDMMCEN                                 ((uint16_t)0x0400)
#define  RCC_USBHSEN                                 ((uint16_t)0x0800)
#define  RCC_USBSSEN                                 ((uint16_t)0x1000)
#define  RCC_DVPEN                                   ((uint16_t)0x2000)
#define  RCC_ETHEN                                   ((uint16_t)0x4000)

#define  RCC_USBOTGEN                                ((uint32_t)0x00020000)
#define  RCC_UHSIFEN                                 ((uint32_t)0x00040000)
#define  RCC_USBPDEN                                 ((uint32_t)0x00080000)
#define  RCC_SERDESEN                                ((uint32_t)0x00100000)
#define  RCC_PIOCEN                                  ((uint32_t)0x00400000)

/******************  Bit definition for RCC_HB2PCENR register  *****************/
#define  RCC_AFIOEN                                  ((uint32_t)0x00000001)         /* Alternate Function I/O clock enable */
#define  RCC_HSADCEN                                 ((uint32_t)0x00000002)
#define  RCC_IOPAEN                                  ((uint32_t)0x00000004)         /* I/O port A clock enable */
#define  RCC_IOPBEN                                  ((uint32_t)0x00000008)         /* I/O port B clock enable */
#define  RCC_IOPCEN                                  ((uint32_t)0x00000010)         /* I/O port C clock enable */
#define  RCC_IOPDEN                                  ((uint32_t)0x00000020)         /* I/O port D clock enable */
#define  RCC_IOPEEN                                  ((uint32_t)0x00000040)
#define  RCC_IOPFEN                                  ((uint32_t)0x00000080)

#define  RCC_ADC1EN                                  ((uint32_t)0x00000200)         /* ADC 1 interface clock enable */
#define  RCC_ADC2EN                                  ((uint32_t)0x00000400)         /* ADC 2 interface clock enable */
#define  RCC_TIM1EN                                  ((uint32_t)0x00000800)         /* TIM1 Timer clock enable */
#define  RCC_SPI1EN                                  ((uint32_t)0x00001000)         /* SPI 1 clock enable */
#define  RCC_TIM8EN                                  ((uint32_t)0x00002000)
#define  RCC_USART1EN                                ((uint32_t)0x00004000)         /* USART1 clock enable */
#define  RCC_I2C4EN                                  ((uint32_t)0x00008000)

#define  RCC_SAIEN                                   ((uint32_t)0x00010000)
#define  RCC_SDIOEN                                  ((uint32_t)0x00040000)
#define  RCC_TIM9EN                                  ((uint32_t)0x00080000)
#define  RCC_TIM10EN                                 ((uint32_t)0x00100000)
#define  RCC_TIM11EN                                 ((uint32_t)0x00200000)
#define  RCC_TIM12EN                                 ((uint32_t)0x00400000)
#define  RCC_OPCMEN                                  ((uint32_t)0x00800000)

#define  RCC_DFSDMEN                                 ((uint32_t)0x02000000)
#define  RCC_ECDCEN                                  ((uint32_t)0x04000000)
#define  RCC_GPHAEN                                  ((uint32_t)0x08000000)
#define  RCC_LTDCEN                                  ((uint32_t)0x40000000)
#define  RCC_I3CEN                                   ((uint32_t)0x80000000)

/*****************  Bit definition for RCC_HB1PCENR register  ******************/
#define  RCC_TIM2EN                                  ((uint32_t)0x00000001)        /* Timer 2 clock enabled*/
#define  RCC_TIM3EN                                  ((uint32_t)0x00000002)        /* Timer 3 clock enable */
#define  RCC_TIM4EN                                  ((uint32_t)0x00000004)
#define  RCC_TIM5EN                                  ((uint32_t)0x00000008)
#define  RCC_TIM6EN                                  ((uint32_t)0x00000010)
#define  RCC_TIM7EN                                  ((uint32_t)0x00000020)
#define  RCC_USART6EN                                ((uint32_t)0x00000040)
#define  RCC_USART7EN                                ((uint32_t)0x00000080)
#define  RCC_USART8EN                                ((uint32_t)0x00000100)
#define  RCC_LPTIM1EN                                ((uint32_t)0x00000200)
#define  RCC_LPTIM2EN                                ((uint32_t)0x00000400)
#define  RCC_WWDGEN                                  ((uint32_t)0x00000800)        /* Window Watchdog clock enable */

#define  RCC_QSPI1EN                                 ((uint32_t)0x00001000)
#define  RCC_QSPI2EN                                 ((uint32_t)0x00002000)
#define  RCC_SPI2EN                                  ((uint32_t)0x00004000)
#define  RCC_SPI3EN                                  ((uint32_t)0x00008000)
#define  RCC_SPI4EN                                  ((uint32_t)0x00010000)

#define  RCC_USART2EN                                ((uint32_t)0x00020000)        /* USART 2 clock enable */
#define  RCC_USART3EN                                ((uint32_t)0x00040000)
#define  RCC_USART4EN                                ((uint32_t)0x00080000)
#define  RCC_USART5EN                                ((uint32_t)0x00100000)
#define  RCC_I2C1EN                                  ((uint32_t)0x00200000)        /* I2C 1 clock enable */
#define  RCC_I2C2EN                                  ((uint32_t)0x00400000)
#define  RCC_CAN3EN                                  ((uint32_t)0x01000000)        /* USB Device clock enable */
#define  RCC_CAN1EN                                  ((uint32_t)0x02000000)
#define  RCC_CAN2EN                                  ((uint32_t)0x04000000)
#define  RCC_BKPEN                                   ((uint32_t)0x08000000)        /* Backup interface clock enable */
#define  RCC_PWREN                                   ((uint32_t)0x10000000)        /* Power interface clock enable */
#define  RCC_DACEN                                   ((uint32_t)0x20000000)
#define  RCC_I2C3EN                                  ((uint32_t)0x40000000)
#define  RCC_SWPMIEN                                 ((uint32_t)0x80000000)

/*******************  Bit definition for RCC_BDCTLR register  *******************/
#define  RCC_LSEON                                   ((uint32_t)0x00000001)        /* External Low Speed oscillator enable */
#define  RCC_LSERDY                                  ((uint32_t)0x00000002)        /* External Low Speed oscillator Ready */
#define  RCC_LSEBYP                                  ((uint32_t)0x00000004)        /* External Low Speed oscillator Bypass */
#define  RCC_CCO                                     ((uint32_t)0x00000008)
#define  RCC_ASOE                                    ((uint32_t)0x00000010)
#define  RCC_ASOS                                    ((uint32_t)0x00000020)

#define  RCC_RTCSEL                                  ((uint32_t)0x000000C0)        /* RTCSEL[1:0] bits (RTC clock source selection) */
#define  RCC_RTCSEL_0                                ((uint32_t)0x00000040)        /* Bit 0 */
#define  RCC_RTCSEL_1                                ((uint32_t)0x00000080)        /* Bit 1 */

#define  RCC_RTCSEL_NOCLOCK                          ((uint32_t)0x00000000)        /* No clock */
#define  RCC_RTCSEL_LSE                              ((uint32_t)0x00000040)        /* LSE oscillator clock used as RTC clock */
#define  RCC_RTCSEL_LSI                              ((uint32_t)0x00000080)        /* LSI oscillator clock used as RTC clock */
#define  RCC_RTCSEL_HSE                              ((uint32_t)0x000000C0)

#define  RCC_RTCEN                                   ((uint32_t)0x00000100)        /* RTC clock enable */
#define  RCC_RTCCAL                                  ((uint32_t)0x0000FE00)
#define  RCC_BDRST                                   ((uint32_t)0x00010000)        /* Backup domain software reset  */

/*******************  Bit definition for RCC_RSTSCKR register  ********************/  
#define  RCC_LSION                                   ((uint32_t)0x00000001)        /* Internal Low Speed oscillator enable */
#define  RCC_LSIRDY                                  ((uint32_t)0x00000002)        /* Internal Low Speed oscillator Ready */
#define  RCC_RMVF                                    ((uint32_t)0x01000000)        /* Remove reset flag */
#define  RCC_PINRSTF                                 ((uint32_t)0x04000000)        /* PIN reset flag */
#define  RCC_PORRSTF                                 ((uint32_t)0x08000000)        /* POR/PDR reset flag */
#define  RCC_SFTRSTF                                 ((uint32_t)0x10000000)        /* Software Reset flag */
#define  RCC_IWDGRSTF                                ((uint32_t)0x20000000)        /* Independent Watchdog reset flag */
#define  RCC_WWDGRSTF                                ((uint32_t)0x40000000)        /* Window watchdog reset flag */
#define  RCC_LOCKUPRSTF                              ((uint32_t)0x80000000)        /* Low-Power reset flag */

/*******************  Bit definition for RCC_HBRSTR register  ********************/
#define  RCC_DMA1RST                                 ((uint32_t)0x00000001)
#define  RCC_DMA2RST                                 ((uint32_t)0x00000002)
#define  RCC_FMCRST                                  ((uint32_t)0x00000100)
#define  RCC_RNGRST                                  ((uint32_t)0x00000200)
#define  RCC_SDMMCRST                                ((uint32_t)0x00000400)
#define  RCC_USBHSRST                                ((uint32_t)0x00000800)

#define  RCC_USBSSRST                                ((uint32_t)0x00001000)
#define  RCC_DVPRST                                  ((uint32_t)0x00002000)        
#define  RCC_ETHRST                                  ((uint32_t)0x00004000)

#define  RCC_USBOTGRST                               ((uint32_t)0x00020000)
#define  RCC_UHSIFRST                                ((uint32_t)0x00040000)
#define  RCC_USBPDRST                                ((uint32_t)0x00080000)
#define  RCC_SERDESRST                               ((uint32_t)0x00100000)
#define  RCC_PIOCRST                                 ((uint32_t)0x00400000)

/*******************  Bit definition for RCC_CFGR2 register  ********************/  
#define  RCC_UHSIFDIV                                ((uint32_t)0x0000003F)
#define  RCC_UHSIFDIV_0                              ((uint32_t)0x00000001)
#define  RCC_UHSIFDIV_1                              ((uint32_t)0x00000002)
#define  RCC_UHSIFDIV_2                              ((uint32_t)0x00000004)
#define  RCC_UHSIFDIV_3                              ((uint32_t)0x00000008)
#define  RCC_UHSIFDIV_4                              ((uint32_t)0x00000010)
#define  RCC_UHSIFDIV_5                              ((uint32_t)0x00000020)

#define  RCC_UHSIFDIV_DIV1                           ((uint32_t)0x00000000)
#define  RCC_UHSIFDIV_DIV2                           ((uint32_t)0x00000001)
#define  RCC_UHSIFDIV_DIV3                           ((uint32_t)0x00000002)
#define  RCC_UHSIFDIV_DIV4                           ((uint32_t)0x00000003)
#define  RCC_UHSIFDIV_DIV5                           ((uint32_t)0x00000004)
#define  RCC_UHSIFDIV_DIV6                           ((uint32_t)0x00000005)
#define  RCC_UHSIFDIV_DIV7                           ((uint32_t)0x00000006)
#define  RCC_UHSIFDIV_DIV8                           ((uint32_t)0x00000007)
#define  RCC_UHSIFDIV_DIV9                           ((uint32_t)0x00000008)
#define  RCC_UHSIFDIV_DIV10                          ((uint32_t)0x00000009)
#define  RCC_UHSIFDIV_DIV11                          ((uint32_t)0x0000000A)
#define  RCC_UHSIFDIV_DIV12                          ((uint32_t)0x0000000B)
#define  RCC_UHSIFDIV_DIV13                          ((uint32_t)0x0000000C)
#define  RCC_UHSIFDIV_DIV14                          ((uint32_t)0x0000000D)
#define  RCC_UHSIFDIV_DIV15                          ((uint32_t)0x0000000E)
#define  RCC_UHSIFDIV_DIV16                          ((uint32_t)0x0000000F)
#define  RCC_UHSIFDIV_DIV17                          ((uint32_t)0x00000010)
#define  RCC_UHSIFDIV_DIV18                          ((uint32_t)0x00000011)
#define  RCC_UHSIFDIV_DIV19                          ((uint32_t)0x00000012)
#define  RCC_UHSIFDIV_DIV20                          ((uint32_t)0x00000013)
#define  RCC_UHSIFDIV_DIV21                          ((uint32_t)0x00000014)
#define  RCC_UHSIFDIV_DIV22                          ((uint32_t)0x00000015)
#define  RCC_UHSIFDIV_DIV23                          ((uint32_t)0x00000016)
#define  RCC_UHSIFDIV_DIV24                          ((uint32_t)0x00000017)
#define  RCC_UHSIFDIV_DIV25                          ((uint32_t)0x00000018)
#define  RCC_UHSIFDIV_DIV26                          ((uint32_t)0x00000019)
#define  RCC_UHSIFDIV_DIV27                          ((uint32_t)0x0000001A)
#define  RCC_UHSIFDIV_DIV28                          ((uint32_t)0x0000001B)
#define  RCC_UHSIFDIV_DIV29                          ((uint32_t)0x0000001C)
#define  RCC_UHSIFDIV_DIV30                          ((uint32_t)0x0000001D)
#define  RCC_UHSIFDIV_DIV31                          ((uint32_t)0x0000001E)
#define  RCC_UHSIFDIV_DIV32                          ((uint32_t)0x0000001F)
#define  RCC_UHSIFDIV_DIV33                          ((uint32_t)0x00000020)
#define  RCC_UHSIFDIV_DIV34                          ((uint32_t)0x00000021)
#define  RCC_UHSIFDIV_DIV35                          ((uint32_t)0x00000022)
#define  RCC_UHSIFDIV_DIV36                          ((uint32_t)0x00000023)
#define  RCC_UHSIFDIV_DIV37                          ((uint32_t)0x00000024)
#define  RCC_UHSIFDIV_DIV38                          ((uint32_t)0x00000025)
#define  RCC_UHSIFDIV_DIV39                          ((uint32_t)0x00000026)
#define  RCC_UHSIFDIV_DIV40                          ((uint32_t)0x00000027)
#define  RCC_UHSIFDIV_DIV41                          ((uint32_t)0x00000028)
#define  RCC_UHSIFDIV_DIV42                          ((uint32_t)0x00000029)
#define  RCC_UHSIFDIV_DIV43                          ((uint32_t)0x0000002A)
#define  RCC_UHSIFDIV_DIV44                          ((uint32_t)0x0000002B)
#define  RCC_UHSIFDIV_DIV45                          ((uint32_t)0x0000002C)
#define  RCC_UHSIFDIV_DIV46                          ((uint32_t)0x0000002D)
#define  RCC_UHSIFDIV_DIV47                          ((uint32_t)0x0000002E)
#define  RCC_UHSIFDIV_DIV48                          ((uint32_t)0x0000002F)
#define  RCC_UHSIFDIV_DIV49                          ((uint32_t)0x00000030)
#define  RCC_UHSIFDIV_DIV50                          ((uint32_t)0x00000031)
#define  RCC_UHSIFDIV_DIV51                          ((uint32_t)0x00000032)
#define  RCC_UHSIFDIV_DIV52                          ((uint32_t)0x00000033)
#define  RCC_UHSIFDIV_DIV53                          ((uint32_t)0x00000034)
#define  RCC_UHSIFDIV_DIV54                          ((uint32_t)0x00000035)
#define  RCC_UHSIFDIV_DIV55                          ((uint32_t)0x00000036)
#define  RCC_UHSIFDIV_DIV56                          ((uint32_t)0x00000037)
#define  RCC_UHSIFDIV_DIV57                          ((uint32_t)0x00000038)
#define  RCC_UHSIFDIV_DIV58                          ((uint32_t)0x00000039)
#define  RCC_UHSIFDIV_DIV59                          ((uint32_t)0x0000003A)
#define  RCC_UHSIFDIV_DIV60                          ((uint32_t)0x0000003B)
#define  RCC_UHSIFDIV_DIV61                          ((uint32_t)0x0000003C)
#define  RCC_UHSIFDIV_DIV62                          ((uint32_t)0x0000003D)
#define  RCC_UHSIFDIV_DIV63                          ((uint32_t)0x0000003E)
#define  RCC_UHSIFDIV_DIV64                          ((uint32_t)0x0000003F)

#define  RCC_UHSIFSRC                                ((uint32_t)0x000000C0)
#define  RCC_UHSIFSRC_0                              ((uint32_t)0x00000040)
#define  RCC_UHSIFSRC_1                              ((uint32_t)0x00000080)

#define  RCC_UHSIFSRC_SYSCLK                         ((uint32_t)0x00000000)
#define  RCC_UHSIFSRC_PLLCLK                         ((uint32_t)0x00000040)
#define  RCC_UHSIFSRC_USBHSPLL                       ((uint32_t)0x00000080)
#define  RCC_UHSIFSRC_ETHPLL                         ((uint32_t)0x000000C0)

#define  RCC_LTDCDIV                                 ((uint32_t)0x00003F00)
#define  RCC_LTDCDIV_0                               ((uint32_t)0x00000100)
#define  RCC_LTDCDIV_1                               ((uint32_t)0x00000200)
#define  RCC_LTDCDIV_2                               ((uint32_t)0x00000400)
#define  RCC_LTDCDIV_3                               ((uint32_t)0x00000800)
#define  RCC_LTDCDIV_4                               ((uint32_t)0x00001000)
#define  RCC_LTDCDIV_5                               ((uint32_t)0x00002000)

#define  RCC_LTDCDIV_DIV1                            ((uint32_t)0x00000000)
#define  RCC_LTDCDIV_DIV2                            ((uint32_t)0x00000100)
#define  RCC_LTDCDIV_DIV3                            ((uint32_t)0x00000200)
#define  RCC_LTDCDIV_DIV4                            ((uint32_t)0x00000300)
#define  RCC_LTDCDIV_DIV5                            ((uint32_t)0x00000400)
#define  RCC_LTDCDIV_DIV6                            ((uint32_t)0x00000500)
#define  RCC_LTDCDIV_DIV7                            ((uint32_t)0x00000600)
#define  RCC_LTDCDIV_DIV8                            ((uint32_t)0x00000700)
#define  RCC_LTDCDIV_DIV9                            ((uint32_t)0x00000800)
#define  RCC_LTDCDIV_DIV10                           ((uint32_t)0x00000900)
#define  RCC_LTDCDIV_DIV11                           ((uint32_t)0x00000A00)
#define  RCC_LTDCDIV_DIV12                           ((uint32_t)0x00000B00)
#define  RCC_LTDCDIV_DIV13                           ((uint32_t)0x00000C00)
#define  RCC_LTDCDIV_DIV14                           ((uint32_t)0x00000D00)
#define  RCC_LTDCDIV_DIV15                           ((uint32_t)0x00000E00)
#define  RCC_LTDCDIV_DIV16                           ((uint32_t)0x00000F00)
#define  RCC_LTDCDIV_DIV17                           ((uint32_t)0x00001000)
#define  RCC_LTDCDIV_DIV18                           ((uint32_t)0x00001100)
#define  RCC_LTDCDIV_DIV19                           ((uint32_t)0x00001200)
#define  RCC_LTDCDIV_DIV20                           ((uint32_t)0x00001300)
#define  RCC_LTDCDIV_DIV21                           ((uint32_t)0x00001400)
#define  RCC_LTDCDIV_DIV22                           ((uint32_t)0x00001500)
#define  RCC_LTDCDIV_DIV23                           ((uint32_t)0x00001600)
#define  RCC_LTDCDIV_DIV24                           ((uint32_t)0x00001700)
#define  RCC_LTDCDIV_DIV25                           ((uint32_t)0x00001800)
#define  RCC_LTDCDIV_DIV26                           ((uint32_t)0x00001900)
#define  RCC_LTDCDIV_DIV27                           ((uint32_t)0x00001A00)
#define  RCC_LTDCDIV_DIV28                           ((uint32_t)0x00001B00)
#define  RCC_LTDCDIV_DIV29                           ((uint32_t)0x00001C00)
#define  RCC_LTDCDIV_DIV30                           ((uint32_t)0x00001D00)
#define  RCC_LTDCDIV_DIV31                           ((uint32_t)0x00001E00)
#define  RCC_LTDCDIV_DIV32                           ((uint32_t)0x00001F00)
#define  RCC_LTDCDIV_DIV33                           ((uint32_t)0x00002000)
#define  RCC_LTDCDIV_DIV34                           ((uint32_t)0x00002100)
#define  RCC_LTDCDIV_DIV35                           ((uint32_t)0x00002200)
#define  RCC_LTDCDIV_DIV36                           ((uint32_t)0x00002300)
#define  RCC_LTDCDIV_DIV37                           ((uint32_t)0x00002400)
#define  RCC_LTDCDIV_DIV38                           ((uint32_t)0x00002500)
#define  RCC_LTDCDIV_DIV39                           ((uint32_t)0x00002600)
#define  RCC_LTDCDIV_DIV40                           ((uint32_t)0x00002700)
#define  RCC_LTDCDIV_DIV41                           ((uint32_t)0x00002800)
#define  RCC_LTDCDIV_DIV42                           ((uint32_t)0x00002900)
#define  RCC_LTDCDIV_DIV43                           ((uint32_t)0x00002A00)
#define  RCC_LTDCDIV_DIV44                           ((uint32_t)0x00002B00)
#define  RCC_LTDCDIV_DIV45                           ((uint32_t)0x00002C00)
#define  RCC_LTDCDIV_DIV46                           ((uint32_t)0x00002D00)
#define  RCC_LTDCDIV_DIV47                           ((uint32_t)0x00002E00)
#define  RCC_LTDCDIV_DIV48                           ((uint32_t)0x00002F00)
#define  RCC_LTDCDIV_DIV49                           ((uint32_t)0x00003000)
#define  RCC_LTDCDIV_DIV50                           ((uint32_t)0x00003100)
#define  RCC_LTDCDIV_DIV51                           ((uint32_t)0x00003200)
#define  RCC_LTDCDIV_DIV52                           ((uint32_t)0x00003300)
#define  RCC_LTDCDIV_DIV53                           ((uint32_t)0x00003400)
#define  RCC_LTDCDIV_DIV54                           ((uint32_t)0x00003500)
#define  RCC_LTDCDIV_DIV55                           ((uint32_t)0x00003600)
#define  RCC_LTDCDIV_DIV56                           ((uint32_t)0x00003700)
#define  RCC_LTDCDIV_DIV57                           ((uint32_t)0x00003800)
#define  RCC_LTDCDIV_DIV58                           ((uint32_t)0x00003900)
#define  RCC_LTDCDIV_DIV59                           ((uint32_t)0x00003A00)
#define  RCC_LTDCDIV_DIV60                           ((uint32_t)0x00003B00)
#define  RCC_LTDCDIV_DIV61                           ((uint32_t)0x00003C00)
#define  RCC_LTDCDIV_DIV62                           ((uint32_t)0x00003D00)
#define  RCC_LTDCDIV_DIV63                           ((uint32_t)0x00003E00)
#define  RCC_LTDCDIV_DIV64                           ((uint32_t)0x00003F00)

#define  RCC_LTDCSRC                                 ((uint32_t)0x0000C000)
#define  RCC_LTDCSRC_0                               ((uint32_t)0x00004000)
#define  RCC_LTDCSRC_1                               ((uint32_t)0x00008000)

#define  RCC_LTDCSRC_PLLCLK                          ((uint32_t)0x00000000)
#define  RCC_LTDCSRC_SERDESPLL                       ((uint32_t)0x00004000)
#define  RCC_LTDCSRC_ETHPLL                          ((uint32_t)0x00008000)
#define  RCC_LTDCSRC_USBHSPLL                        ((uint32_t)0x0000C000)

#define  RCC_USBFSDIV                                ((uint32_t)0x000F0000)
#define  RCC_USBFSDIV_0                              ((uint32_t)0x00010000)
#define  RCC_USBFSDIV_1                              ((uint32_t)0x00020000)
#define  RCC_USBFSDIV_2                              ((uint32_t)0x00040000)
#define  RCC_USBFSDIV_3                              ((uint32_t)0x00080000)

#define  RCC_USBFSDIV_DIV1                           ((uint32_t)0x00000000)
#define  RCC_USBFSDIV_DIV2                           ((uint32_t)0x00010000)
#define  RCC_USBFSDIV_DIV3                           ((uint32_t)0x00020000)
#define  RCC_USBFSDIV_DIV4                           ((uint32_t)0x00030000)
#define  RCC_USBFSDIV_DIV5                           ((uint32_t)0x00040000)
#define  RCC_USBFSDIV_DIV6                           ((uint32_t)0x00050000)
#define  RCC_USBFSDIV_DIV8                           ((uint32_t)0x00060000)
#define  RCC_USBFSDIV_DIV10                          ((uint32_t)0x00070000)
#define  RCC_USBFSDIV_DIV1_5                         ((uint32_t)0x00080000)
#define  RCC_USBFSDIV_DIV2_5                         ((uint32_t)0x00090000)
#define  RCC_USBFSDIV_DIV3_5                         ((uint32_t)0x000A0000)
#define  RCC_USBFSDIV_DIV4_5                         ((uint32_t)0x000B0000)
#define  RCC_USBFSDIV_DIV5_5                         ((uint32_t)0x000C0000)
#define  RCC_USBFSDIV_DIV6_5                         ((uint32_t)0x000D0000)
#define  RCC_USBFSDIV_DIV7_5                         ((uint32_t)0x000E0000)
#define  RCC_USBFSDIV_DIV9_5                         ((uint32_t)0x000F0000)

#define  RCC_USBFSSRC                                ((uint32_t)0x00100000)

#define  RCC_USBFSSRC_PLLCLK                         ((uint32_t)0x00000000)
#define  RCC_USBFSSRC_USBHSPLL                       ((uint32_t)0x00100000)

#define  RCC_RNGSRC                                  ((uint32_t)0x00800000)

#define  RCC_RNGSRC_SYSCLK                           ((uint32_t)0x00000000)
#define  RCC_RNGSRC_PLLCLK                           ((uint32_t)0x00800000)

#define  RCC_I2S2SRC                                 ((uint32_t)0x01000000)

#define  RCC_I2S2SRC_SYSCLK                          ((uint32_t)0x00000000)
#define  RCC_I2S2SRC_PLLCLK                          ((uint32_t)0x01000000)

#define  RCC_I2S3SRC                                 ((uint32_t)0x02000000)

#define  RCC_I2S3SRC_SYSCLK                          ((uint32_t)0x00000000)
#define  RCC_I2S3SRC_PLLCLK                          ((uint32_t)0x02000000)

#define  RCC_HSADCSRC                                ((uint32_t)0x30000000)
#define  RCC_HSADCSRC_0                              ((uint32_t)0x10000000)
#define  RCC_HSADCSRC_1                              ((uint32_t)0x20000000)

#define  RCC_HSADCSRC_SYSCLK                         ((uint32_t)0x00000000)
#define  RCC_HSADCSRC_PLLCLK                         ((uint32_t)0x10000000)
#define  RCC_HSADCSRC_USBHSPLL                       ((uint32_t)0x20000000)
#define  RCC_HSADCSRC_ETHPLL                         ((uint32_t)0x30000000)

#define  RCC_ETH1GSRC                                ((uint32_t)0xC0000000)
#define  RCC_ETH1GSRC_0                              ((uint32_t)0x40000000)
#define  RCC_ETH1GSRC_1                              ((uint32_t)0x80000000)

#define  RCC_ETH1GSRC_PLLCLK                         ((uint32_t)0x00000000)
#define  RCC_ETH1GSRC_USBSSPLL                       ((uint32_t)0x40000000)
#define  RCC_ETH1GSRC_ETHPLL_DIV4                    ((uint32_t)0x80000000)
#define  RCC_ETH1GSRC_SERDESPLL_DIV8                 ((uint32_t)0xC0000000)

/*******************  Bit definition for RCC_PLLCFGR2 register  ********************/
#define  RCC_USBHSPLLSRC                             ((uint32_t)0x00000003)
#define  RCC_USBHSPLLSRC_0                           ((uint32_t)0x00000001)
#define  RCC_USBHSPLLSRC_1                           ((uint32_t)0x00000002)

#define  RCC_USBHSPLLSRC_HSE                         ((uint32_t)0x00000000)
#define  RCC_USBHSPLLSRC_HSI                         ((uint32_t)0x00000001)
#define  RCC_USBHSPLLSRC_ETHCLK_20M                  ((uint32_t)0x00000002)
#define  RCC_USBHSPLLSRC_PLLCLK                      ((uint32_t)0x00000003)

#define  RCC_USBHSPLL_REFSEL                         ((uint32_t)0x0000000C)
#define  RCC_USBHSPLL_REFSEL_0                       ((uint32_t)0x00000004)
#define  RCC_USBHSPLL_REFSEL_1                       ((uint32_t)0x00000008)

#define  RCC_USBHSPLL_REFSEL_25MHZ                   ((uint32_t)0x00000000)
#define  RCC_USBHSPLL_REFSEL_20MHZ                   ((uint32_t)0x00000004)
#define  RCC_USBHSPLL_REFSEL_24MHZ                   ((uint32_t)0x00000008)
#define  RCC_USBHSPLL_REFSEL_32MHZ                   ((uint32_t)0x0000000C)

#define  RCC_USBSSPLL_REFSEL                         ((uint32_t)0x00000070)
#define  RCC_USBSSPLL_REFSEL_0                       ((uint32_t)0x00000010)
#define  RCC_USBSSPLL_REFSEL_1                       ((uint32_t)0x00000020)
#define  RCC_USBSSPLL_REFSEL_2                       ((uint32_t)0x00000030)

#define  RCC_USBSSPLL_REFSEL_20MHz                   ((uint32_t)0x00000000)
#define  RCC_USBSSPLL_REFSEL_24MHz                   ((uint32_t)0x00000010)
#define  RCC_USBSSPLL_REFSEL_25MHz                   ((uint32_t)0x00000020)
#define  RCC_USBSSPLL_REFSEL_30MHz                   ((uint32_t)0x00000030)
#define  RCC_USBSSPLL_REFSEL_32MHz                   ((uint32_t)0x00000040)
#define  RCC_USBSSPLL_REFSEL_40MHz                   ((uint32_t)0x00000050)
#define  RCC_USBSSPLL_REFSEL_60MHz                   ((uint32_t)0x00000060)
#define  RCC_USBSSPLL_REFSEL_80MHz                   ((uint32_t)0x00000070)

#define  RCC_USBHSPLL_IN_DIV                         ((uint32_t)0x00001F00)
#define  RCC_USBHSPLL_IN_DIV_0                       ((uint32_t)0x00000100)
#define  RCC_USBHSPLL_IN_DIV_1                       ((uint32_t)0x00000200)
#define  RCC_USBHSPLL_IN_DIV_2                       ((uint32_t)0x00000400)
#define  RCC_USBHSPLL_IN_DIV_3                       ((uint32_t)0x00000800)
#define  RCC_USBHSPLL_IN_DIV_4                       ((uint32_t)0x00001000)

#define  RCC_USBHSPLL_IN_DIV1                        ((uint32_t)0x00000000)
#define  RCC_USBHSPLL_IN_DIV2                        ((uint32_t)0x00000100)
#define  RCC_USBHSPLL_IN_DIV3                        ((uint32_t)0x00000200)
#define  RCC_USBHSPLL_IN_DIV4                        ((uint32_t)0x00000300)
#define  RCC_USBHSPLL_IN_DIV5                        ((uint32_t)0x00000400)
#define  RCC_USBHSPLL_IN_DIV6                        ((uint32_t)0x00000500)
#define  RCC_USBHSPLL_IN_DIV7                        ((uint32_t)0x00000600)
#define  RCC_USBHSPLL_IN_DIV8                        ((uint32_t)0x00000700)
#define  RCC_USBHSPLL_IN_DIV9                        ((uint32_t)0x00000800)
#define  RCC_USBHSPLL_IN_DIV10                       ((uint32_t)0x00000900)
#define  RCC_USBHSPLL_IN_DIV11                       ((uint32_t)0x00000A00)
#define  RCC_USBHSPLL_IN_DIV12                       ((uint32_t)0x00000B00)
#define  RCC_USBHSPLL_IN_DIV13                       ((uint32_t)0x00000C00)
#define  RCC_USBHSPLL_IN_DIV14                       ((uint32_t)0x00000D00)
#define  RCC_USBHSPLL_IN_DIV15                       ((uint32_t)0x00000E00)
#define  RCC_USBHSPLL_IN_DIV16                       ((uint32_t)0x00000F00)
#define  RCC_USBHSPLL_IN_DIV17                       ((uint32_t)0x00001000)
#define  RCC_USBHSPLL_IN_DIV18                       ((uint32_t)0x00001100)
#define  RCC_USBHSPLL_IN_DIV19                       ((uint32_t)0x00001200)
#define  RCC_USBHSPLL_IN_DIV20                       ((uint32_t)0x00001300)
#define  RCC_USBHSPLL_IN_DIV21                       ((uint32_t)0x00001400)
#define  RCC_USBHSPLL_IN_DIV22                       ((uint32_t)0x00001500)
#define  RCC_USBHSPLL_IN_DIV23                       ((uint32_t)0x00001600)
#define  RCC_USBHSPLL_IN_DIV24                       ((uint32_t)0x00001700)
#define  RCC_USBHSPLL_IN_DIV25                       ((uint32_t)0x00001800)
#define  RCC_USBHSPLL_IN_DIV26                       ((uint32_t)0x00001900)
#define  RCC_USBHSPLL_IN_DIV27                       ((uint32_t)0x00001A00)
#define  RCC_USBHSPLL_IN_DIV28                       ((uint32_t)0x00001B00)
#define  RCC_USBHSPLL_IN_DIV29                       ((uint32_t)0x00001C00)
#define  RCC_USBHSPLL_IN_DIV30                       ((uint32_t)0x00001D00)
#define  RCC_USBHSPLL_IN_DIV31                       ((uint32_t)0x00001E00)
#define  RCC_USBHSPLL_IN_DIV32                       ((uint32_t)0x00001F00)

#define  RCC_SERDESPLL_MUL                           ((uint32_t)0x000F0000)
#define  RCC_SERDESPLL_MUL_0                         ((uint32_t)0x00010000)
#define  RCC_SERDESPLL_MUL_1                         ((uint32_t)0x00020000)
#define  RCC_SERDESPLL_MUL_2                         ((uint32_t)0x00040000)
#define  RCC_SERDESPLL_MUL_3                         ((uint32_t)0x00080000)

#define  RCC_SERDESPLL_MUL25                         ((uint32_t)0x00000000)
#define  RCC_SERDESPLL_MUL28                         ((uint32_t)0x00010000)
#define  RCC_SERDESPLL_MUL30                         ((uint32_t)0x00020000)
#define  RCC_SERDESPLL_MUL32                         ((uint32_t)0x00030000)
#define  RCC_SERDESPLL_MUL35                         ((uint32_t)0x00040000)
#define  RCC_SERDESPLL_MUL38                         ((uint32_t)0x00050000)
#define  RCC_SERDESPLL_MUL40                         ((uint32_t)0x00060000)
#define  RCC_SERDESPLL_MUL45                         ((uint32_t)0x00070000)
#define  RCC_SERDESPLL_MUL50                         ((uint32_t)0x00080000)
#define  RCC_SERDESPLL_MUL56                         ((uint32_t)0x00090000)
#define  RCC_SERDESPLL_MUL60                         ((uint32_t)0x000A0000)
#define  RCC_SERDESPLL_MUL64                         ((uint32_t)0x000B0000)
#define  RCC_SERDESPLL_MUL70                         ((uint32_t)0x000C0000)
#define  RCC_SERDESPLL_MUL76                         ((uint32_t)0x000D0000)
#define  RCC_SERDESPLL_MUL80                         ((uint32_t)0x000E0000)
#define  RCC_SERDESPLL_MUL90                         ((uint32_t)0x000F0000)

/******************************************************************************/
/*                                    RNG                                     */
/******************************************************************************/
/********************  Bit definition for RNG_CR register  *******************/
#define  RNG_CR_RNGEN                                ((uint32_t)0x00000004)
#define  RNG_CR_IE                                   ((uint32_t)0x00000008)

/********************  Bit definition for RNG_SR register  *******************/
#define  RNG_SR_DRDY                                 ((uint32_t)0x00000001)
#define  RNG_SR_CECS                                 ((uint32_t)0x00000002)
#define  RNG_SR_SECS                                 ((uint32_t)0x00000004)
#define  RNG_SR_CEIS                                 ((uint32_t)0x00000020)
#define  RNG_SR_SEIS                                 ((uint32_t)0x00000040)

/******************************************************************************/
/*                             Real-Time Clock                                */
/******************************************************************************/

/*******************  Bit definition for RTC_CTLRH register  ********************/
#define  RTC_CTLRH_SECIE                             ((uint8_t)0x01)               /* Second Interrupt Enable */
#define  RTC_CTLRH_ALRIE                             ((uint8_t)0x02)               /* Alarm Interrupt Enable */
#define  RTC_CTLRH_OWIE                              ((uint8_t)0x04)               /* OverfloW Interrupt Enable */

/*******************  Bit definition for RTC_CTLRL register  ********************/
#define  RTC_CTLRL_SECF                              ((uint8_t)0x01)               /* Second Flag */
#define  RTC_CTLRL_ALRF                              ((uint8_t)0x02)               /* Alarm Flag */
#define  RTC_CTLRL_OWF                               ((uint8_t)0x04)               /* OverfloW Flag */
#define  RTC_CTLRL_RSF                               ((uint8_t)0x08)               /* Registers Synchronized Flag */
#define  RTC_CTLRL_CNF                               ((uint8_t)0x10)               /* Configuration Flag */
#define  RTC_CTLRL_RTOFF                             ((uint8_t)0x20)               /* RTC operation OFF */

/*******************  Bit definition for RTC_PSCRH register  *******************/
#define  RTC_PSCH_PRL                                ((uint16_t)0x000F)            /* RTC Prescaler Reload Value High */

/*******************  Bit definition for RTC_PSCRL register  *******************/
#define  RTC_PSCL_PRL                                ((uint16_t)0xFFFF)            /* RTC Prescaler Reload Value Low */

/*******************  Bit definition for RTC_DIVH register  *******************/
#define  RTC_DIVH_RTC_DIV                            ((uint16_t)0x000F)            /* RTC Clock Divider High */

/*******************  Bit definition for RTC_DIVL register  *******************/
#define  RTC_DIVL_RTC_DIV                            ((uint16_t)0xFFFF)            /* RTC Clock Divider Low */

/*******************  Bit definition for RTC_CNTH register  *******************/
#define  RTC_CNTH_RTC_CNT                            ((uint16_t)0xFFFF)            /* RTC Counter High */

/*******************  Bit definition for RTC_CNTL register  *******************/
#define  RTC_CNTL_RTC_CNT                            ((uint16_t)0xFFFF)            /* RTC Counter Low */

/*******************  Bit definition for RTC_ALRMH register  *******************/
#define  RTC_ALRMH_RTC_ALRM                          ((uint16_t)0xFFFF)            /* RTC Alarm High */

/*******************  Bit definition for RTC_ALRML register  *******************/
#define  RTC_ALRML_RTC_ALRM                          ((uint16_t)0xFFFF)            /* RTC Alarm Low */

/******************************************************************************/
/*                        Serial Peripheral Interface                         */
/******************************************************************************/

/*******************  Bit definition for SPI_CTLR1 register  ********************/
#define  SPI_CTLR1_CPHA                              ((uint16_t)0x0001)            /* Clock Phase */
#define  SPI_CTLR1_CPOL                              ((uint16_t)0x0002)            /* Clock Polarity */
#define  SPI_CTLR1_MSTR                              ((uint16_t)0x0004)            /* Master Selection */

#define  SPI_CTLR1_BR                                ((uint16_t)0x0038)            /* BR[2:0] bits (Baud Rate Control) */
#define  SPI_CTLR1_BR_0                              ((uint16_t)0x0008)            /* Bit 0 */
#define  SPI_CTLR1_BR_1                              ((uint16_t)0x0010)            /* Bit 1 */
#define  SPI_CTLR1_BR_2                              ((uint16_t)0x0020)            /* Bit 2 */

#define  SPI_CTLR1_SPE                               ((uint16_t)0x0040)            /* SPI Enable */
#define  SPI_CTLR1_LSBFIRST                          ((uint16_t)0x0080)            /* Frame Format */
#define  SPI_CTLR1_SSI                               ((uint16_t)0x0100)            /* Internal slave select */
#define  SPI_CTLR1_SSM                               ((uint16_t)0x0200)            /* Software slave management */
#define  SPI_CTLR1_RXONLY                            ((uint16_t)0x0400)            /* Receive only */
#define  SPI_CTLR1_DFF                               ((uint16_t)0x0800)            /* Data Frame Format */
#define  SPI_CTLR1_CRCNEXT                           ((uint16_t)0x1000)            /* Transmit CRC next */
#define  SPI_CTLR1_CRCEN                             ((uint16_t)0x2000)            /* Hardware CRC calculation enable */
#define  SPI_CTLR1_BIDIOE                            ((uint16_t)0x4000)            /* Output enable in bidirectional mode */
#define  SPI_CTLR1_BIDIMODE                          ((uint16_t)0x8000)            /* Bidirectional data mode enable */

/*******************  Bit definition for SPI_CTLR2 register  ********************/
#define  SPI_CTLR2_RXDMAEN                           ((uint8_t)0x01)               /* Rx Buffer DMA Enable */
#define  SPI_CTLR2_TXDMAEN                           ((uint8_t)0x02)               /* Tx Buffer DMA Enable */
#define  SPI_CTLR2_SSOE                              ((uint8_t)0x04)               /* SS Output Enable */
#define  SPI_CTLR2_ERRIE                             ((uint8_t)0x20)               /* Error Interrupt Enable */
#define  SPI_CTLR2_RXNEIE                            ((uint8_t)0x40)               /* RX buffer Not Empty Interrupt Enable */
#define  SPI_CTLR2_TXEIE                             ((uint8_t)0x80)               /* Tx buffer Empty Interrupt Enable */

/********************  Bit definition for SPI_STATR register  ********************/
#define  SPI_STATR_RXNE                              ((uint8_t)0x01)               /* Receive buffer Not Empty */
#define  SPI_STATR_TXE                               ((uint8_t)0x02)               /* Transmit buffer Empty */
#define  SPI_STATR_CHSIDE                            ((uint8_t)0x04)               /* Channel side */
#define  SPI_STATR_UDR                               ((uint8_t)0x08)               /* Underrun flag */
#define  SPI_STATR_CRCERR                            ((uint8_t)0x10)               /* CRC Error flag */
#define  SPI_STATR_MODF                              ((uint8_t)0x20)               /* Mode fault */
#define  SPI_STATR_OVR                               ((uint8_t)0x40)               /* Overrun flag */
#define  SPI_STATR_BSY                               ((uint8_t)0x80)               /* Busy flag */

/********************  Bit definition for SPI_DATAR register  ********************/
#define  SPI_DATAR_DR                                ((uint16_t)0xFFFF)            /* Data Register */

/*******************  Bit definition for SPI_CRCR register  ******************/
#define  SPI_CRCR_CRCPOLY                            ((uint16_t)0xFFFF)            /* CRC polynomial register */

/******************  Bit definition for SPI_RCRCR register  ******************/
#define  SPI_RCRCR_RXCRC                             ((uint16_t)0xFFFF)            /* Rx CRC Register */

/******************  Bit definition for SPI_TCRCR register  ******************/
#define  SPI_TCRCR_TXCRC                             ((uint16_t)0xFFFF)            /* Tx CRC Register */

/******************  Bit definition for SPI_I2SCFGR register  *****************/
#define  SPI_I2SCFGR_CHLEN                           ((uint16_t)0x0001)            /* Channel length (number of bits per audio channel) */

#define  SPI_I2SCFGR_DATLEN                          ((uint16_t)0x0006)            /* DATLEN[1:0] bits (Data length to be transferred) */
#define  SPI_I2SCFGR_DATLEN_0                        ((uint16_t)0x0002)            /* Bit 0 */
#define  SPI_I2SCFGR_DATLEN_1                        ((uint16_t)0x0004)            /* Bit 1 */

#define  SPI_I2SCFGR_CKPOL                           ((uint16_t)0x0008)            /* steady state clock polarity */

#define  SPI_I2SCFGR_I2SSTD                          ((uint16_t)0x0030)            /* I2SSTD[1:0] bits (I2S standard selection) */
#define  SPI_I2SCFGR_I2SSTD_0                        ((uint16_t)0x0010)            /* Bit 0 */
#define  SPI_I2SCFGR_I2SSTD_1                        ((uint16_t)0x0020)            /* Bit 1 */

#define  SPI_I2SCFGR_PCMSYNC                         ((uint16_t)0x0080)            /* PCM frame synchronization */

#define  SPI_I2SCFGR_I2SCFG                          ((uint16_t)0x0300)            /* I2SCFG[1:0] bits (I2S configuration mode) */
#define  SPI_I2SCFGR_I2SCFG_0                        ((uint16_t)0x0100)            /* Bit 0 */
#define  SPI_I2SCFGR_I2SCFG_1                        ((uint16_t)0x0200)            /* Bit 1 */

#define  SPI_I2SCFGR_I2SE                            ((uint16_t)0x0400)            /* I2S Enable */
#define  SPI_I2SCFGR_I2SMOD                          ((uint16_t)0x0800)            /* I2S mode selection */

/******************  Bit definition for SPI_I2SPR register  *******************/
#define  SPI_I2SPR_I2SDIV                            ((uint16_t)0x00FF)            /* I2S Linear prescaler */
#define  SPI_I2SPR_ODD                               ((uint16_t)0x0100)            /* Odd factor for the prescaler */
#define  SPI_I2SPR_MCKOE                             ((uint16_t)0x0200)            /* Master Clock Output Enable */

/******************  Bit definition for SPI_HSCR register  *******************/
#define  SPI_HSCR_HSRXEN                             ((uint16_t)0x0001)            
#define  SPI_HSCR_HSRXEN2                            ((uint16_t)0x0004)            

/******************************************************************************/
/*                                    TIM                                     */
/******************************************************************************/

/*******************  Bit definition for TIM_CTLR1 register  ********************/
#define  TIM_CEN                                     ((uint16_t)0x0001)            /* Counter enable */
#define  TIM_UDIS                                    ((uint16_t)0x0002)            /* Update disable */
#define  TIM_URS                                     ((uint16_t)0x0004)            /* Update request source */
#define  TIM_OPM                                     ((uint16_t)0x0008)            /* One pulse mode */
#define  TIM_DIR                                     ((uint16_t)0x0010)            /* Direction */

#define  TIM_CMS                                     ((uint16_t)0x0060)            /* CMS[1:0] bits (Center-aligned mode selection) */
#define  TIM_CMS_0                                   ((uint16_t)0x0020)            /* Bit 0 */
#define  TIM_CMS_1                                   ((uint16_t)0x0040)            /* Bit 1 */

#define  TIM_ARPE                                    ((uint16_t)0x0080)            /* Auto-reload preload enable */

#define  TIM_CTLR1_CKD                               ((uint16_t)0x0300)            /* CKD[1:0] bits (clock division) */
#define  TIM_CKD_0                                   ((uint16_t)0x0100)            /* Bit 0 */
#define  TIM_CKD_1                                   ((uint16_t)0x0200)            /* Bit 1 */

#define  TIM_CAPOV                                   ((uint16_t)0x4000)
#define  TIM_CAPLVL                                  ((uint16_t)0x8000)

/*******************  Bit definition for TIM_CTLR2 register  ********************/
#define  TIM_CCPC                                    ((uint16_t)0x0001)            /* Capture/Compare Preloaded Control */
#define  TIM_CCUS                                    ((uint16_t)0x0004)            /* Capture/Compare Control Update Selection */
#define  TIM_CCDS                                    ((uint16_t)0x0008)            /* Capture/Compare DMA Selection */

#define  TIM_MMS                                     ((uint16_t)0x0070)            /* MMS[2:0] bits (Master Mode Selection) */
#define  TIM_MMS_0                                   ((uint16_t)0x0010)            /* Bit 0 */
#define  TIM_MMS_1                                   ((uint16_t)0x0020)            /* Bit 1 */
#define  TIM_MMS_2                                   ((uint16_t)0x0040)            /* Bit 2 */

#define  TIM_TI1S                                    ((uint16_t)0x0080)            /* TI1 Selection */
#define  TIM_OIS1                                    ((uint16_t)0x0100)            /* Output Idle state 1 (OC1 output) */
#define  TIM_OIS1N                                   ((uint16_t)0x0200)            /* Output Idle state 1 (OC1N output) */
#define  TIM_OIS2                                    ((uint16_t)0x0400)            /* Output Idle state 2 (OC2 output) */
#define  TIM_OIS2N                                   ((uint16_t)0x0800)            /* Output Idle state 2 (OC2N output) */
#define  TIM_OIS3                                    ((uint16_t)0x1000)            /* Output Idle state 3 (OC3 output) */
#define  TIM_OIS3N                                   ((uint16_t)0x2000)            /* Output Idle state 3 (OC3N output) */
#define  TIM_OIS4                                    ((uint16_t)0x4000)            /* Output Idle state 4 (OC4 output) */

/*******************  Bit definition for TIM_SMCFGR register  *******************/
#define  TIM_SMS                                     ((uint16_t)0x0007)            /* SMS[2:0] bits (Slave mode selection) */
#define  TIM_SMS_0                                   ((uint16_t)0x0001)            /* Bit 0 */
#define  TIM_SMS_1                                   ((uint16_t)0x0002)            /* Bit 1 */
#define  TIM_SMS_2                                   ((uint16_t)0x0004)            /* Bit 2 */

#define  TIM_TS                                      ((uint16_t)0x0070)            /* TS[2:0] bits (Trigger selection) */
#define  TIM_TS_0                                    ((uint16_t)0x0010)            /* Bit 0 */
#define  TIM_TS_1                                    ((uint16_t)0x0020)            /* Bit 1 */
#define  TIM_TS_2                                    ((uint16_t)0x0040)            /* Bit 2 */

#define  TIM_MSM                                     ((uint16_t)0x0080)            /* Master/slave mode */
 
#define  TIM_ETF                                     ((uint16_t)0x0F00)            /* ETF[3:0] bits (External trigger filter) */
#define  TIM_ETF_0                                   ((uint16_t)0x0100)            /* Bit 0 */
#define  TIM_ETF_1                                   ((uint16_t)0x0200)            /* Bit 1 */
#define  TIM_ETF_2                                   ((uint16_t)0x0400)            /* Bit 2 */
#define  TIM_ETF_3                                   ((uint16_t)0x0800)            /* Bit 3 */

#define  TIM_ETPS                                    ((uint16_t)0x3000)            /* ETPS[1:0] bits (External trigger prescaler) */
#define  TIM_ETPS_0                                  ((uint16_t)0x1000)            /* Bit 0 */
#define  TIM_ETPS_1                                  ((uint16_t)0x2000)            /* Bit 1 */
 
#define  TIM_ECE                                     ((uint16_t)0x4000)            /* External clock enable */
#define  TIM_ETP                                     ((uint16_t)0x8000)            /* External trigger polarity */

/*******************  Bit definition for TIM_DMAINTENR register  *******************/
#define  TIM_UIE                                     ((uint16_t)0x0001)            /* Update interrupt enable */
#define  TIM_CC1IE                                   ((uint16_t)0x0002)            /* Capture/Compare 1 interrupt enable */
#define  TIM_CC2IE                                   ((uint16_t)0x0004)            /* Capture/Compare 2 interrupt enable */
#define  TIM_CC3IE                                   ((uint16_t)0x0008)            /* Capture/Compare 3 interrupt enable */
#define  TIM_CC4IE                                   ((uint16_t)0x0010)            /* Capture/Compare 4 interrupt enable */
#define  TIM_COMIE                                   ((uint16_t)0x0020)            /* COM interrupt enable */
#define  TIM_TIE                                     ((uint16_t)0x0040)            /* Trigger interrupt enable */
#define  TIM_BIE                                     ((uint16_t)0x0080)            /* Break interrupt enable */
#define  TIM_UDE                                     ((uint16_t)0x0100)            /* Update DMA request enable */
#define  TIM_CC1DE                                   ((uint16_t)0x0200)            /* Capture/Compare 1 DMA request enable */
#define  TIM_CC2DE                                   ((uint16_t)0x0400)            /* Capture/Compare 2 DMA request enable */
#define  TIM_CC3DE                                   ((uint16_t)0x0800)            /* Capture/Compare 3 DMA request enable */
#define  TIM_CC4DE                                   ((uint16_t)0x1000)            /* Capture/Compare 4 DMA request enable */
#define  TIM_COMDE                                   ((uint16_t)0x2000)            /* COM DMA request enable */
#define  TIM_TDE                                     ((uint16_t)0x4000)            /* Trigger DMA request enable */

/********************  Bit definition for TIM_INTFR register  ********************/
#define  TIM_UIF                                     ((uint16_t)0x0001)            /* Update interrupt Flag */
#define  TIM_CC1IF                                   ((uint16_t)0x0002)            /* Capture/Compare 1 interrupt Flag */
#define  TIM_CC2IF                                   ((uint16_t)0x0004)            /* Capture/Compare 2 interrupt Flag */
#define  TIM_CC3IF                                   ((uint16_t)0x0008)            /* Capture/Compare 3 interrupt Flag */
#define  TIM_CC4IF                                   ((uint16_t)0x0010)            /* Capture/Compare 4 interrupt Flag */
#define  TIM_COMIF                                   ((uint16_t)0x0020)            /* COM interrupt Flag */
#define  TIM_TIF                                     ((uint16_t)0x0040)            /* Trigger interrupt Flag */
#define  TIM_BIF                                     ((uint16_t)0x0080)            /* Break interrupt Flag */
#define  TIM_CC1OF                                   ((uint16_t)0x0200)            /* Capture/Compare 1 Overcapture Flag */
#define  TIM_CC2OF                                   ((uint16_t)0x0400)            /* Capture/Compare 2 Overcapture Flag */
#define  TIM_CC3OF                                   ((uint16_t)0x0800)            /* Capture/Compare 3 Overcapture Flag */
#define  TIM_CC4OF                                   ((uint16_t)0x1000)            /* Capture/Compare 4 Overcapture Flag */

/*******************  Bit definition for TIM_SWEVGR register  ********************/
#define  TIM_UG                                      ((uint8_t)0x01)               /* Update Generation */
#define  TIM_CC1G                                    ((uint8_t)0x02)               /* Capture/Compare 1 Generation */
#define  TIM_CC2G                                    ((uint8_t)0x04)               /* Capture/Compare 2 Generation */
#define  TIM_CC3G                                    ((uint8_t)0x08)               /* Capture/Compare 3 Generation */
#define  TIM_CC4G                                    ((uint8_t)0x10)               /* Capture/Compare 4 Generation */
#define  TIM_COMG                                    ((uint8_t)0x20)               /* Capture/Compare Control Update Generation */
#define  TIM_TG                                      ((uint8_t)0x40)               /* Trigger Generation */
#define  TIM_BG                                      ((uint8_t)0x80)               /* Break Generation */

/******************  Bit definition for TIM_CHCTLR1 register  *******************/
#define  TIM_CC1S                                    ((uint16_t)0x0003)            /* CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define  TIM_CC1S_0                                  ((uint16_t)0x0001)            /* Bit 0 */
#define  TIM_CC1S_1                                  ((uint16_t)0x0002)            /* Bit 1 */

#define  TIM_OC1FE                                   ((uint16_t)0x0004)            /* Output Compare 1 Fast enable */
#define  TIM_OC1PE                                   ((uint16_t)0x0008)            /* Output Compare 1 Preload enable */

#define  TIM_OC1M                                    ((uint16_t)0x0070)            /* OC1M[2:0] bits (Output Compare 1 Mode) */
#define  TIM_OC1M_0                                  ((uint16_t)0x0010)            /* Bit 0 */
#define  TIM_OC1M_1                                  ((uint16_t)0x0020)            /* Bit 1 */
#define  TIM_OC1M_2                                  ((uint16_t)0x0040)            /* Bit 2 */

#define  TIM_OC1CE                                   ((uint16_t)0x0080)            /* Output Compare 1Clear Enable */

#define  TIM_CC2S                                    ((uint16_t)0x0300)            /* CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define  TIM_CC2S_0                                  ((uint16_t)0x0100)            /* Bit 0 */
#define  TIM_CC2S_1                                  ((uint16_t)0x0200)            /* Bit 1 */

#define  TIM_OC2FE                                   ((uint16_t)0x0400)            /* Output Compare 2 Fast enable */
#define  TIM_OC2PE                                   ((uint16_t)0x0800)            /* Output Compare 2 Preload enable */

#define  TIM_OC2M                                    ((uint16_t)0x7000)            /* OC2M[2:0] bits (Output Compare 2 Mode) */
#define  TIM_OC2M_0                                  ((uint16_t)0x1000)            /* Bit 0 */
#define  TIM_OC2M_1                                  ((uint16_t)0x2000)            /* Bit 1 */
#define  TIM_OC2M_2                                  ((uint16_t)0x4000)            /* Bit 2 */

#define  TIM_OC2CE                                   ((uint16_t)0x8000)            /* Output Compare 2 Clear Enable */


#define  TIM_IC1PSC                                  ((uint16_t)0x000C)            /* IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define  TIM_IC1PSC_0                                ((uint16_t)0x0004)            /* Bit 0 */
#define  TIM_IC1PSC_1                                ((uint16_t)0x0008)            /* Bit 1 */

#define  TIM_IC1F                                    ((uint16_t)0x00F0)            /* IC1F[3:0] bits (Input Capture 1 Filter) */
#define  TIM_IC1F_0                                  ((uint16_t)0x0010)            /* Bit 0 */
#define  TIM_IC1F_1                                  ((uint16_t)0x0020)            /* Bit 1 */
#define  TIM_IC1F_2                                  ((uint16_t)0x0040)            /* Bit 2 */
#define  TIM_IC1F_3                                  ((uint16_t)0x0080)            /* Bit 3 */

#define  TIM_IC2PSC                                  ((uint16_t)0x0C00)            /* IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define  TIM_IC2PSC_0                                ((uint16_t)0x0400)            /* Bit 0 */
#define  TIM_IC2PSC_1                                ((uint16_t)0x0800)            /* Bit 1 */

#define  TIM_IC2F                                    ((uint16_t)0xF000)            /* IC2F[3:0] bits (Input Capture 2 Filter) */
#define  TIM_IC2F_0                                  ((uint16_t)0x1000)            /* Bit 0 */
#define  TIM_IC2F_1                                  ((uint16_t)0x2000)            /* Bit 1 */
#define  TIM_IC2F_2                                  ((uint16_t)0x4000)            /* Bit 2 */
#define  TIM_IC2F_3                                  ((uint16_t)0x8000)            /* Bit 3 */

/******************  Bit definition for TIM_CHCTLR2 register  *******************/
#define  TIM_CC3S                                    ((uint16_t)0x0003)            /* CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define  TIM_CC3S_0                                  ((uint16_t)0x0001)            /* Bit 0 */
#define  TIM_CC3S_1                                  ((uint16_t)0x0002)            /* Bit 1 */

#define  TIM_OC3FE                                   ((uint16_t)0x0004)            /* Output Compare 3 Fast enable */
#define  TIM_OC3PE                                   ((uint16_t)0x0008)            /* Output Compare 3 Preload enable */

#define  TIM_OC3M                                    ((uint16_t)0x0070)            /* OC3M[2:0] bits (Output Compare 3 Mode) */
#define  TIM_OC3M_0                                  ((uint16_t)0x0010)            /* Bit 0 */
#define  TIM_OC3M_1                                  ((uint16_t)0x0020)            /* Bit 1 */
#define  TIM_OC3M_2                                  ((uint16_t)0x0040)            /* Bit 2 */

#define  TIM_OC3CE                                   ((uint16_t)0x0080)            /* Output Compare 3 Clear Enable */

#define  TIM_CC4S                                    ((uint16_t)0x0300)            /* CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define  TIM_CC4S_0                                  ((uint16_t)0x0100)            /* Bit 0 */
#define  TIM_CC4S_1                                  ((uint16_t)0x0200)            /* Bit 1 */

#define  TIM_OC4FE                                   ((uint16_t)0x0400)            /* Output Compare 4 Fast enable */
#define  TIM_OC4PE                                   ((uint16_t)0x0800)            /* Output Compare 4 Preload enable */

#define  TIM_OC4M                                    ((uint16_t)0x7000)            /* OC4M[2:0] bits (Output Compare 4 Mode) */
#define  TIM_OC4M_0                                  ((uint16_t)0x1000)            /* Bit 0 */
#define  TIM_OC4M_1                                  ((uint16_t)0x2000)            /* Bit 1 */
#define  TIM_OC4M_2                                  ((uint16_t)0x4000)            /* Bit 2 */

#define  TIM_OC4CE                                   ((uint16_t)0x8000)            /* Output Compare 4 Clear Enable */


#define  TIM_IC3PSC                                  ((uint16_t)0x000C)            /* IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define  TIM_IC3PSC_0                                ((uint16_t)0x0004)            /* Bit 0 */
#define  TIM_IC3PSC_1                                ((uint16_t)0x0008)            /* Bit 1 */

#define  TIM_IC3F                                    ((uint16_t)0x00F0)            /* IC3F[3:0] bits (Input Capture 3 Filter) */
#define  TIM_IC3F_0                                  ((uint16_t)0x0010)            /* Bit 0 */
#define  TIM_IC3F_1                                  ((uint16_t)0x0020)            /* Bit 1 */
#define  TIM_IC3F_2                                  ((uint16_t)0x0040)            /* Bit 2 */
#define  TIM_IC3F_3                                  ((uint16_t)0x0080)            /* Bit 3 */

#define  TIM_IC4PSC                                  ((uint16_t)0x0C00)            /* IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define  TIM_IC4PSC_0                                ((uint16_t)0x0400)            /* Bit 0 */
#define  TIM_IC4PSC_1                                ((uint16_t)0x0800)            /* Bit 1 */

#define  TIM_IC4F                                    ((uint16_t)0xF000)            /* IC4F[3:0] bits (Input Capture 4 Filter) */
#define  TIM_IC4F_0                                  ((uint16_t)0x1000)            /* Bit 0 */
#define  TIM_IC4F_1                                  ((uint16_t)0x2000)            /* Bit 1 */
#define  TIM_IC4F_2                                  ((uint16_t)0x4000)            /* Bit 2 */
#define  TIM_IC4F_3                                  ((uint16_t)0x8000)            /* Bit 3 */

/*******************  Bit definition for TIM_CCER register  *******************/
#define  TIM_CC1E                                    ((uint16_t)0x0001)            /* Capture/Compare 1 output enable */
#define  TIM_CC1P                                    ((uint16_t)0x0002)            /* Capture/Compare 1 output Polarity */
#define  TIM_CC1NE                                   ((uint16_t)0x0004)            /* Capture/Compare 1 Complementary output enable */
#define  TIM_CC1NP                                   ((uint16_t)0x0008)            /* Capture/Compare 1 Complementary output Polarity */
#define  TIM_CC2E                                    ((uint16_t)0x0010)            /* Capture/Compare 2 output enable */
#define  TIM_CC2P                                    ((uint16_t)0x0020)            /* Capture/Compare 2 output Polarity */
#define  TIM_CC2NE                                   ((uint16_t)0x0040)            /* Capture/Compare 2 Complementary output enable */
#define  TIM_CC2NP                                   ((uint16_t)0x0080)            /* Capture/Compare 2 Complementary output Polarity */
#define  TIM_CC3E                                    ((uint16_t)0x0100)            /* Capture/Compare 3 output enable */
#define  TIM_CC3P                                    ((uint16_t)0x0200)            /* Capture/Compare 3 output Polarity */
#define  TIM_CC3NE                                   ((uint16_t)0x0400)            /* Capture/Compare 3 Complementary output enable */
#define  TIM_CC3NP                                   ((uint16_t)0x0800)            /* Capture/Compare 3 Complementary output Polarity */
#define  TIM_CC4E                                    ((uint16_t)0x1000)            /* Capture/Compare 4 output enable */
#define  TIM_CC4P                                    ((uint16_t)0x2000)            /* Capture/Compare 4 output Polarity */

/*******************  Bit definition for TIM_CNT register  ********************/
#define  TIM_CNT                                     ((uint16_t)0xFFFF)            /* Counter Value */

/*******************  Bit definition for TIM_PSC register  ********************/
#define  TIM_PSC                                     ((uint16_t)0xFFFF)            /* Prescaler Value */

/*******************  Bit definition for TIM_ATRLR register  ********************/
#define  TIM_ARR                                     ((uint16_t)0xFFFF)            /* actual auto-reload Value */

/*******************  Bit definition for TIM_RPTCR register  ********************/
#define  TIM_REP                                     ((uint8_t)0xFF)               /* Repetition Counter Value */

/*******************  Bit definition for TIM_CH1CVR register  *******************/
#define  TIM_CCR1                                    ((uint16_t)0xFFFF)            /* Capture/Compare 1 Value */
#define  TIM_LEVEL1                                  ((uint32_t)0x00010000)

/*******************  Bit definition for TIM_CH2CVR register  *******************/
#define  TIM_CCR2                                    ((uint16_t)0xFFFF)            /* Capture/Compare 2 Value */
#define  TIM_LEVEL2                                  ((uint32_t)0x00010000)

/*******************  Bit definition for TIM_CH3CVR register  *******************/
#define  TIM_CCR3                                    ((uint16_t)0xFFFF)            /* Capture/Compare 3 Value */
#define  TIM_LEVEL3                                  ((uint32_t)0x00010000)

/*******************  Bit definition for TIM_CH4CVR register  *******************/
#define  TIM_CCR4                                    ((uint16_t)0xFFFF)            /* Capture/Compare 4 Value */
#define  TIM_LEVEL4                                  ((uint32_t)0x00010000)

/*******************  Bit definition for TIM_BDTR register  *******************/
#define  TIM_DTG                                     ((uint16_t)0x00FF)            /* DTG[0:7] bits (Dead-Time Generator set-up) */
#define  TIM_DTG_0                                   ((uint16_t)0x0001)            /* Bit 0 */
#define  TIM_DTG_1                                   ((uint16_t)0x0002)            /* Bit 1 */
#define  TIM_DTG_2                                   ((uint16_t)0x0004)            /* Bit 2 */
#define  TIM_DTG_3                                   ((uint16_t)0x0008)            /* Bit 3 */
#define  TIM_DTG_4                                   ((uint16_t)0x0010)            /* Bit 4 */
#define  TIM_DTG_5                                   ((uint16_t)0x0020)            /* Bit 5 */
#define  TIM_DTG_6                                   ((uint16_t)0x0040)            /* Bit 6 */
#define  TIM_DTG_7                                   ((uint16_t)0x0080)            /* Bit 7 */

#define  TIM_LOCK                                    ((uint16_t)0x0300)            /* LOCK[1:0] bits (Lock Configuration) */
#define  TIM_LOCK_0                                  ((uint16_t)0x0100)            /* Bit 0 */
#define  TIM_LOCK_1                                  ((uint16_t)0x0200)            /* Bit 1 */
 
#define  TIM_OSSI                                    ((uint16_t)0x0400)            /* Off-State Selection for Idle mode */
#define  TIM_OSSR                                    ((uint16_t)0x0800)            /* Off-State Selection for Run mode */
#define  TIM_BKE                                     ((uint16_t)0x1000)            /* Break enable */
#define  TIM_BKP                                     ((uint16_t)0x2000)            /* Break Polarity */
#define  TIM_AOE                                     ((uint16_t)0x4000)            /* Automatic Output enable */
#define  TIM_MOE                                     ((uint16_t)0x8000)            /* Main Output enable */

/*******************  Bit definition for TIM_DMACFGR register  ********************/
#define  TIM_DBA                                     ((uint16_t)0x001F)            /* DBA[4:0] bits (DMA Base Address) */
#define  TIM_DBA_0                                   ((uint16_t)0x0001)            /* Bit 0 */
#define  TIM_DBA_1                                   ((uint16_t)0x0002)            /* Bit 1 */
#define  TIM_DBA_2                                   ((uint16_t)0x0004)            /* Bit 2 */
#define  TIM_DBA_3                                   ((uint16_t)0x0008)            /* Bit 3 */
#define  TIM_DBA_4                                   ((uint16_t)0x0010)            /* Bit 4 */

#define  TIM_DBL                                     ((uint16_t)0x1F00)            /* DBL[4:0] bits (DMA Burst Length) */
#define  TIM_DBL_0                                   ((uint16_t)0x0100)            /* Bit 0 */
#define  TIM_DBL_1                                   ((uint16_t)0x0200)            /* Bit 1 */
#define  TIM_DBL_2                                   ((uint16_t)0x0400)            /* Bit 2 */
#define  TIM_DBL_3                                   ((uint16_t)0x0800)            /* Bit 3 */
#define  TIM_DBL_4                                   ((uint16_t)0x1000)            /* Bit 4 */

/*******************  Bit definition for TIM_DMAADR register  *******************/
#define  TIM_DMAR_DMAB                               ((uint16_t)0xFFFF)            /* DMA register for burst accesses */

/*******************  Bit definition for TIM_AUX register  *******************/
#define  TIM_AUX_CAPCH2_ED                           ((uint16_t)0x0001)            
#define  TIM_AUX_CAPCH3_ED                           ((uint16_t)0x0002)            
#define  TIM_AUX_CAPCH4_ED                           ((uint16_t)0x0004)            
#define  TIM_AUX_CAPCH234_ED                         ((uint16_t)0x0007)

#define  TIM_AUX_BK_SEL                              ((uint16_t)0x0038)
#define  TIM_AUX_BK_SEL_0                            ((uint16_t)0x0008)
#define  TIM_AUX_BK_SEL_1                            ((uint16_t)0x0010)
#define  TIM_AUX_BK_SEL_2                            ((uint16_t)0x0020)

#define  TIM_AUX_DT_MODE                             ((uint16_t)0x0040)
#define  TIM_AUX_DTN_MODE                            ((uint16_t)0x0080)

#define  TIM_AUX_DT_VLU2                             ((uint16_t)0xFF00)

/******************************************************************************/
/*         Universal Synchronous Asynchronous Receiver Transmitter            */
/******************************************************************************/

/*******************  Bit definition for USART_STATR register  *******************/
#define  USART_STATR_PE                              ((uint16_t)0x0001)            /* Parity Error */
#define  USART_STATR_FE                              ((uint16_t)0x0002)            /* Framing Error */
#define  USART_STATR_NE                              ((uint16_t)0x0004)            /* Noise Error Flag */
#define  USART_STATR_ORE                             ((uint16_t)0x0008)            /* OverRun Error */
#define  USART_STATR_IDLE                            ((uint16_t)0x0010)            /* IDLE line detected */
#define  USART_STATR_RXNE                            ((uint16_t)0x0020)            /* Read Data Register Not Empty */
#define  USART_STATR_TC                              ((uint16_t)0x0040)            /* Transmission Complete */
#define  USART_STATR_TXE                             ((uint16_t)0x0080)            /* Transmit Data Register Empty */
#define  USART_STATR_LBD                             ((uint16_t)0x0100)            /* LIN Break Detection Flag */
#define  USART_STATR_CTS                             ((uint16_t)0x0200)            /* CTS Flag */
#define  USART_STATR_RX_BUSY                         ((uint16_t)0x0400)
#define  USART_STATR_MS_ERR                          ((uint16_t)0x0800)
#define  USART_STATR_USART_WKUP                      ((uint16_t)0x8000)

/*******************  Bit definition for USART_DATAR register  *******************/
#define  USART_DATAR_DR                              ((uint16_t)0x01FF)            /* Data value */

/******************  Bit definition for USART_BRR register  *******************/
#define  USART_BRR_DIV_Fraction                      ((uint16_t)0x000F)            /* Fraction of USARTDIV */
#define  USART_BRR_DIV_Mantissa                      ((uint16_t)0xFFF0)            /* Mantissa of USARTDIV */

/******************  Bit definition for USART_CTLR1 register  *******************/
#define  USART_CTLR1_SBK                             ((uint16_t)0x0001)            /* Send Break */
#define  USART_CTLR1_RWU                             ((uint16_t)0x0002)            /* Receiver wakeup */
#define  USART_CTLR1_RE                              ((uint16_t)0x0004)            /* Receiver Enable */
#define  USART_CTLR1_TE                              ((uint16_t)0x0008)            /* Transmitter Enable */
#define  USART_CTLR1_IDLEIE                          ((uint16_t)0x0010)            /* IDLE Interrupt Enable */
#define  USART_CTLR1_RXNEIE                          ((uint16_t)0x0020)            /* RXNE Interrupt Enable */
#define  USART_CTLR1_TCIE                            ((uint16_t)0x0040)            /* Transmission Complete Interrupt Enable */
#define  USART_CTLR1_TXEIE                           ((uint16_t)0x0080)            /* PE Interrupt Enable */
#define  USART_CTLR1_PEIE                            ((uint16_t)0x0100)            /* PE Interrupt Enable */
#define  USART_CTLR1_PS                              ((uint16_t)0x0200)            /* Parity Selection */
#define  USART_CTLR1_PCE                             ((uint16_t)0x0400)            /* Parity Control Enable */
#define  USART_CTLR1_WAKE                            ((uint16_t)0x0800)            /* Wakeup method */
#define  USART_CTLR1_M                               ((uint16_t)0x1000)            /* Word length */
#define  USART_CTLR1_UE                              ((uint16_t)0x2000)            /* USART Enable */

#define  USART_CTLR1_M_EXT                           ((uint16_t)0xC000)
#define  USART_CTLR1_M_EXT_0                         ((uint16_t)0x4000)            
#define  USART_CTLR1_M_EXT_1                         ((uint16_t)0x8000) 

#define  USART_CTLR1_M_EXT5                          ((uint16_t)0xC000) 
#define  USART_CTLR1_M_EXT6                          ((uint16_t)0x8000) 
#define  USART_CTLR1_M_EXT7                          ((uint16_t)0x4000) 

/******************  Bit definition for USART_CTLR2 register  *******************/
#define  USART_CTLR2_ADD                             ((uint16_t)0x000F)            /* Address of the USART node */
#define  USART_CTLR2_LBDL                            ((uint16_t)0x0020)            /* LIN Break Detection Length */
#define  USART_CTLR2_LBDIE                           ((uint16_t)0x0040)            /* LIN Break Detection Interrupt Enable */
#define  USART_CTLR2_LBCL                            ((uint16_t)0x0100)            /* Last Bit Clock pulse */
#define  USART_CTLR2_CPHA                            ((uint16_t)0x0200)            /* Clock Phase */
#define  USART_CTLR2_CPOL                            ((uint16_t)0x0400)            /* Clock Polarity */
#define  USART_CTLR2_CLKEN                           ((uint16_t)0x0800)            /* Clock Enable */

#define  USART_CTLR2_STOP                            ((uint16_t)0x3000)            /* STOP[1:0] bits (STOP bits) */
#define  USART_CTLR2_STOP_0                          ((uint16_t)0x1000)            /* Bit 0 */
#define  USART_CTLR2_STOP_1                          ((uint16_t)0x2000)            /* Bit 1 */

#define  USART_CTLR2_LINEN                           ((uint16_t)0x4000)            /* LIN mode enable */

/******************  Bit definition for USART_CTLR3 register  *******************/
#define  USART_CTLR3_EIE                             ((uint16_t)0x0001)            /* Error Interrupt Enable */
#define  USART_CTLR3_IREN                            ((uint16_t)0x0002)            /* IrDA mode Enable */
#define  USART_CTLR3_IRLP                            ((uint16_t)0x0004)            /* IrDA Low-Power */
#define  USART_CTLR3_HDSEL                           ((uint16_t)0x0008)            /* Half-Duplex Selection */
#define  USART_CTLR3_NACK                            ((uint16_t)0x0010)            /* Smartcard NACK enable */
#define  USART_CTLR3_SCEN                            ((uint16_t)0x0020)            /* Smartcard mode enable */
#define  USART_CTLR3_DMAR                            ((uint16_t)0x0040)            /* DMA Enable Receiver */
#define  USART_CTLR3_DMAT                            ((uint16_t)0x0080)            /* DMA Enable Transmitter */
#define  USART_CTLR3_RTSE                            ((uint16_t)0x0100)            /* RTS Enable */
#define  USART_CTLR3_CTSE                            ((uint16_t)0x0200)            /* CTS Enable */
#define  USART_CTLR3_CTSIE                           ((uint16_t)0x0400)            /* CTS Interrupt Enable */
#define  USART_CTLR3_LPWKUP_EN                       ((uint16_t)0x0800)
#define  USART_CTLR3_LPWKUP_CK_SRC                   ((uint16_t)0x1000)
#define  USART_CTLR3_LPWKUP_DLY_CFG                  ((uint16_t)0xE000)

/******************  Bit definition for USART_GPR register  ******************/
#define  USART_GPR_PSC                               ((uint16_t)0x00FF)            /* PSC[7:0] bits (Prescaler value) */
#define  USART_GPR_PSC_0                             ((uint16_t)0x0001)            /* Bit 0 */
#define  USART_GPR_PSC_1                             ((uint16_t)0x0002)            /* Bit 1 */
#define  USART_GPR_PSC_2                             ((uint16_t)0x0004)            /* Bit 2 */
#define  USART_GPR_PSC_3                             ((uint16_t)0x0008)            /* Bit 3 */
#define  USART_GPR_PSC_4                             ((uint16_t)0x0010)            /* Bit 4 */
#define  USART_GPR_PSC_5                             ((uint16_t)0x0020)            /* Bit 5 */
#define  USART_GPR_PSC_6                             ((uint16_t)0x0040)            /* Bit 6 */
#define  USART_GPR_PSC_7                             ((uint16_t)0x0080)            /* Bit 7 */

#define  USART_GPR_GT                                ((uint16_t)0xFF00)            /* Guard time value */

/******************  Bit definition for USART_CTLR4 register  ******************/
#define  USART_CTLR4_MS_ERRIE                        ((uint16_t)0x0002)            
#define  USART_CTLR4_CHECK_SEL                       ((uint16_t)0x000C)            
#define  USART_CTLR4_CHECK_MARKENABLE                ((uint16_t)0x0008)
#define  USART_CTLR4_CHECK_APACEENABLE               ((uint16_t)0x000C)

/******************************************************************************/
/*                            OPA                                 */
/******************************************************************************/

/*******************  Bit definition for OPA_CTLR1 register  ********************/
#define  OPA_CTLR1_EN1                               ((uint16_t)0x0001)

#define  OPA_CTLR1_MODE1                             ((uint16_t)0x0006)
#define  OPA_CTLR1_MODE1_0                           ((uint16_t)0x0002)
#define  OPA_CTLR1_MODE1_1                           ((uint16_t)0x0004)

#define  OPA_CTLR1_PSEL1                             ((uint16_t)0x0008)

#define  OPA_CTLR1_NSEL1                             ((uint16_t)0x0070)
#define  OPA_CTLR1_NSEL1_0                           ((uint16_t)0x0010)
#define  OPA_CTLR1_NSEL1_1                           ((uint16_t)0x0020)
#define  OPA_CTLR1_NSEL1_2                           ((uint16_t)0x0030)

#define  OPA_CTLR1_FBEN1                             ((uint16_t)0x0100)
#define  OPA_CTLR1_PGADIF1                           ((uint16_t)0x0200)
#define  OPA_CTLR1_HS1                               ((uint16_t)0x0400)

/*******************  Bit definition for OPA_CTLR2 register  ********************/
#define  OPA_CTLR2_EN2                               ((uint16_t)0x0001)

#define  OPA_CTLR2_MODE2                             ((uint16_t)0x0006)
#define  OPA_CTLR2_MODE2_0                           ((uint16_t)0x0002)
#define  OPA_CTLR2_MODE2_1                           ((uint16_t)0x0004)

#define  OPA_CTLR2_PSEL2                             ((uint16_t)0x0008)

#define  OPA_CTLR2_NSEL2                             ((uint16_t)0x0070)
#define  OPA_CTLR2_NSEL2_0                           ((uint16_t)0x0010)
#define  OPA_CTLR2_NSEL2_1                           ((uint16_t)0x0020)
#define  OPA_CTLR2_NSEL2_2                           ((uint16_t)0x0030)

#define  OPA_CTLR2_FBEN2                             ((uint16_t)0x0100)
#define  OPA_CTLR2_PGADIF2                           ((uint16_t)0x0200)
#define  OPA_CTLR2_HS2                               ((uint16_t)0x0400)

/*******************  Bit definition for OPA_CTLR3 register  ********************/
#define  OPA_CTLR3_EN3                               ((uint16_t)0x0001)

#define  OPA_CTLR3_MODE3                             ((uint16_t)0x0006)
#define  OPA_CTLR3_MODE3_0                           ((uint16_t)0x0002)
#define  OPA_CTLR3_MODE3_1                           ((uint16_t)0x0004)

#define  OPA_CTLR3_PSEL3                             ((uint16_t)0x0008)

#define  OPA_CTLR3_NSEL3                             ((uint16_t)0x0070)
#define  OPA_CTLR3_NSEL3_0                           ((uint16_t)0x0010)
#define  OPA_CTLR3_NSEL3_1                           ((uint16_t)0x0020)
#define  OPA_CTLR3_NSEL3_2                           ((uint16_t)0x0030)

#define  OPA_CTLR3_FBEN3                             ((uint16_t)0x0100)
#define  OPA_CTLR3_PGADIF3                           ((uint16_t)0x0200)
#define  OPA_CTLR3_HS3                               ((uint16_t)0x0400)

/*******************  Bit definition for CMP_CTLR register  ********************/
#define  OPA_CMP_CTLR_PSEL                           ((uint32_t)0x00000003)
#define  OPA_CMP_CTLR_PSEL_0                         ((uint32_t)0x00000001)
#define  OPA_CMP_CTLR_PSEL_1                         ((uint32_t)0x00000002)

#define  OPA_CMP_CTLR_NSEL                           ((uint32_t)0x0000000C)
#define  OPA_CMP_CTLR_NSEL_0                         ((uint32_t)0x00000004)
#define  OPA_CMP_CTLR_NSEL_1                         ((uint32_t)0x00000008)

#define  OPA_CMP_CTLR_MODE                           ((uint32_t)0x000000F0)
#define  OPA_CMP_CTLR_MODE_0                         ((uint32_t)0x00000010)
#define  OPA_CMP_CTLR_MODE_1                         ((uint32_t)0x00000020)
#define  OPA_CMP_CTLR_MODE_2                         ((uint32_t)0x00000040)
#define  OPA_CMP_CTLR_MODE_3                         ((uint32_t)0x00000080)

#define  OPA_CMP_CTLR_EN                             ((uint32_t)0x00000100)

#define  OPA_CMP_CTLR_HYPSEL                         ((uint32_t)0x00000600)
#define  OPA_CMP_CTLR_HYPSEL_0                       ((uint32_t)0x00000200)
#define  OPA_CMP_CTLR_HYPSEL_1                       ((uint32_t)0x00000400)

#define  OPA_CMP_CTLR_VREF                           ((uint32_t)0x00001800)
#define  OPA_CMP_CTLR_VREF_0                         ((uint32_t)0x00000800)
#define  OPA_CMP_CTLR_VREF_1                         ((uint32_t)0x00001000)

#define  OPA_CMP_CTLR_FILT_EN                        ((uint32_t)0x00002000)

#define  OPA_CMP_CTLR_FILT_CFG                       ((uint32_t)0x01FF0000)
#define  OPA_CMP_CTLR_FILT_BASE                      ((uint32_t)0x70000000)

/*******************  Bit definition for CMP_STATR register  ********************/
#define  OPA_CMP_STATR_OUTFILT                       ((uint8_t)0x01)

/******************************************************************************/
/*                            Window WATCHDOG                                 */
/******************************************************************************/

/*******************  Bit definition for WWDG_CTLR register  ********************/
#define  WWDG_CTLR_T                                 ((uint8_t)0x7F)               /* T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define  WWDG_CTLR_T0                                ((uint8_t)0x01)               /* Bit 0 */
#define  WWDG_CTLR_T1                                ((uint8_t)0x02)               /* Bit 1 */
#define  WWDG_CTLR_T2                                ((uint8_t)0x04)               /* Bit 2 */
#define  WWDG_CTLR_T3                                ((uint8_t)0x08)               /* Bit 3 */
#define  WWDG_CTLR_T4                                ((uint8_t)0x10)               /* Bit 4 */
#define  WWDG_CTLR_T5                                ((uint8_t)0x20)               /* Bit 5 */
#define  WWDG_CTLR_T6                                ((uint8_t)0x40)               /* Bit 6 */

#define  WWDG_CTLR_WDGA                              ((uint8_t)0x80)               /* Activation bit */

/*******************  Bit definition for WWDG_CFGR register  *******************/
#define  WWDG_CFGR_W                                 ((uint16_t)0x007F)            /* W[6:0] bits (7-bit window value) */
#define  WWDG_CFGR_W0                                ((uint16_t)0x0001)            /* Bit 0 */
#define  WWDG_CFGR_W1                                ((uint16_t)0x0002)            /* Bit 1 */
#define  WWDG_CFGR_W2                                ((uint16_t)0x0004)            /* Bit 2 */
#define  WWDG_CFGR_W3                                ((uint16_t)0x0008)            /* Bit 3 */
#define  WWDG_CFGR_W4                                ((uint16_t)0x0010)            /* Bit 4 */
#define  WWDG_CFGR_W5                                ((uint16_t)0x0020)            /* Bit 5 */
#define  WWDG_CFGR_W6                                ((uint16_t)0x0040)            /* Bit 6 */

#define  WWDG_CFGR_WDGTB                             ((uint16_t)0x0180)            /* WDGTB[1:0] bits (Timer Base) */
#define  WWDG_CFGR_WDGTB0                            ((uint16_t)0x0080)            /* Bit 0 */
#define  WWDG_CFGR_WDGTB1                            ((uint16_t)0x0100)            /* Bit 1 */
 
#define  WWDG_CFGR_EWI                               ((uint16_t)0x0200)            /* Early Wakeup Interrupt */

/*******************  Bit definition for WWDG_STATR register  ********************/
#define  WWDG_STATR_EWIF                             ((uint8_t)0x01)               /* Early Wakeup Interrupt Flag */

/******************************************************************************/
/*                                  DVP                                       */
/******************************************************************************/

/*******************  Bit definition for DVP_CR0 register  ********************/
#define RB_DVP_ENABLE			                           0x01	 // RW, DVP enable
#define RB_DVP_V_POLAR			                         0x02	 // RW, DVP VSYNC polarity control: 1 = invert, 0 = not invert
#define	RB_DVP_H_POLAR			                         0x04	 // RW, DVP HSYNC polarity control: 1 = invert, 0 = not invert
#define	RB_DVP_P_POLAR			                         0x08	 // RW, DVP PCLK polarity control: 1 = invert, 0 = not invert
#define RB_DVP_MSK_DAT_MOD		                       0x30					
#define 	RB_DVP_D8_MOD			                         0x00	 // RW, DVP 8bits data mode
#define		RB_DVP_D10_MOD			                       0x10	 // RW, DVP 10bits data mode
#define		RB_DVP_D12_MOD			                       0x20	 // RW, DVP 12bits data mode
#define	RB_DVP_JPEG				                           0x40	 // RW, DVP JPEG mode

/*******************  Bit definition for DVP_CR1 register  ********************/
#define RB_DVP_DMA_EN			                           0x01	 // RW, DVP dma enable
#define RB_DVP_ALL_CLR			                         0x02	 // RW, DVP all clear, high action
#define	RB_DVP_RCV_CLR			                         0x04	 // RW, DVP receive logic clear, high action
#define RB_DVP_BUF_TOG			                         0x08	 // RW, DVP bug toggle by software, write 1 to toggle, ignored writing 0
#define RB_DVP_CM				                             0x10	 // RW, DVP capture mode
#define	RB_DVP_CROP				                           0x20	 // RW, DVP Crop feature enable
#define RB_DVP_FCRC				                           0xC0	 // RW, DVP frame capture rate control: 
#define		DVP_RATE_100P		                           0x00	 // 00 = every frame captured (100%) 
#define		DVP_RATE_50P		                           0x40	 // 01 = every alternate frame captured (50%)
#define		DVP_RATE_25P		                           0x80	 // 10 = one frame in four frame captured (25%)

/*******************  Bit definition for DVP_IER register  ********************/
#define	RB_DVP_IE_STR_FRM		                         0x01	 // RW, DVP frame start interrupt enable
#define	RB_DVP_IE_ROW_DONE		                       0x02	 // RW, DVP row received done interrupt enable
#define RB_DVP_IE_FRM_DONE		                       0x04	 // RW, DVP frame received done interrupt enable
#define	RB_DVP_IE_FIFO_OV		                         0x08	 // RW, DVP receive fifo overflow interrupt enable	
#define RB_DVP_IE_STP_FRM		                         0x10	 // RW, DVP frame stop interrupt enable				

/*******************  Bit definition for DVP_ROW_NUM register  ********************/
#define RB_DVP_ROW_NUM                               ((uint16_t)0xFFFF)

/*******************  Bit definition for DVP_COL_NUM register  ********************/
#define RB_DVP_COL_NUM                               ((uint16_t)0xFFFF)

/*******************  Bit definition for DVP_DMA_BUF0 register  ********************/
#define RB_DVP_DMA_BUF0                              ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for DVP_DMA_BUF1 register  ********************/
#define RB_DVP_DMA_BUF1                              ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for DVP_IFR register  ********************/
#define RB_DVP_IF_STR_FRM	                           0x01	 // RW1, interrupt flag for DVP frame start
#define RB_DVP_IF_ROW_DONE		                       0x02	 // RW1, interrupt flag for DVP row receive done
#define RB_DVP_IF_FRM_DONE		                       0x04	 // RW1, interrupt flag for DVP frame receive done
#define RB_DVP_IF_FIFO_OV		                         0x08	 // RW1, interrupt flag for DVP receive fifo overflow
#define RB_DVP_IF_STP_FRM		                         0x10	 // RW1, interrupt flag for DVP frame stop

/*******************  Bit definition for DVP_STATUS register  ********************/
#define RB_DVP_FIFO_RDY			                         0x01	 // RO, DVP receive fifo ready
#define RB_DVP_FIFO_FULL		                         0x02	 // RO, DVP receive fifo full
#define RB_DVP_FIFO_OV			                         0x04	 // RO, DVP receive fifo overflow
#define RB_DVP_MSK_FIFO_CNT		                       0x70	 // RO, DVP receive fifo count

/*******************  Bit definition for DVP_ROW_CNT register  ********************/
#define RB_DVP_ROW_CNT			                         ((uint16_t)0xFF)					

/*******************  Bit definition for DVP_HOFFCNT register  ********************/
#define RB_DVP_HOFFCNT			                         ((uint16_t)0xFF)					

/*******************  Bit definition for DVP_VST register  ********************/
#define RB_DVP_VST     			                         ((uint16_t)0xFF)				

/*******************  Bit definition for DVP_CAPCNT register  ********************/
#define RB_DVP_CAPCNT     	                         ((uint16_t)0xFF)

/*******************  Bit definition for DVP_VLINE register  ********************/
#define RB_DVP_VLINE                                 ((uint16_t)0xFF)

/*******************  Bit definition for DVP_DR register  ********************/
#define RB_DVP_DR         	                         ((uint16_t)0xFF)

/******************************************************************************/
/*                                  TKEY                                       */
/******************************************************************************/

/*******************  Bit definition for TKEY_CHARGE1 register  *******************/
#define  TKEY_CHARGE1_TKCG10                         ((uint32_t)0x0007)            
#define  TKEY_CHARGE1_TKCG10_1C5                     ((uint32_t)0x0000)
#define  TKEY_CHARGE1_TKCG10_7C5                     ((uint32_t)0x0001)
#define  TKEY_CHARGE1_TKCG10_13C5                    ((uint32_t)0x0002)
#define  TKEY_CHARGE1_TKCG10_28C5                    ((uint32_t)0x0003)
#define  TKEY_CHARGE1_TKCG10_41C5                    ((uint32_t)0x0004)
#define  TKEY_CHARGE1_TKCG10_55C5                    ((uint32_t)0x0005)
#define  TKEY_CHARGE1_TKCG10_71C5                    ((uint32_t)0x0006)
#define  TKEY_CHARGE1_TKCG10_239C5                   ((uint32_t)0x0007)

#define  TKEY_CHARGE1_TKCG11                         ((uint32_t)0x0038)            
#define  TKEY_CHARGE1_TKCG11_1C5                     ((uint32_t)0x0000)
#define  TKEY_CHARGE1_TKCG11_7C5                     ((uint32_t)0x0008)
#define  TKEY_CHARGE1_TKCG11_13C5                    ((uint32_t)0x0010)
#define  TKEY_CHARGE1_TKCG11_28C5                    ((uint32_t)0x0018)
#define  TKEY_CHARGE1_TKCG11_41C5                    ((uint32_t)0x0020)
#define  TKEY_CHARGE1_TKCG11_55C5                    ((uint32_t)0x0028)
#define  TKEY_CHARGE1_TKCG11_71C5                    ((uint32_t)0x0030)
#define  TKEY_CHARGE1_TKCG11_239C5                   ((uint32_t)0x0038)

#define  TKEY_CHARGE1_TKCG12                         ((uint32_t)0x01C0)            
#define  TKEY_CHARGE1_TKCG12_1C5                     ((uint32_t)0x0000)
#define  TKEY_CHARGE1_TKCG12_7C5                     ((uint32_t)0x0040)
#define  TKEY_CHARGE1_TKCG12_13C5                    ((uint32_t)0x0080)
#define  TKEY_CHARGE1_TKCG12_28C5                    ((uint32_t)0x00C0)
#define  TKEY_CHARGE1_TKCG12_41C5                    ((uint32_t)0x0100)
#define  TKEY_CHARGE1_TKCG12_55C5                    ((uint32_t)0x0140)
#define  TKEY_CHARGE1_TKCG12_71C5                    ((uint32_t)0x0180)
#define  TKEY_CHARGE1_TKCG12_239C5                   ((uint32_t)0x01C0)

#define  TKEY_CHARGE1_TKCG13                         ((uint32_t)0x0E00)            
#define  TKEY_CHARGE1_TKCG13_1C5                     ((uint32_t)0x0000)
#define  TKEY_CHARGE1_TKCG13_7C5                     ((uint32_t)0x0200)
#define  TKEY_CHARGE1_TKCG13_13C5                    ((uint32_t)0x0400)
#define  TKEY_CHARGE1_TKCG13_28C5                    ((uint32_t)0x0600)
#define  TKEY_CHARGE1_TKCG13_41C5                    ((uint32_t)0x0800)
#define  TKEY_CHARGE1_TKCG13_55C5                    ((uint32_t)0x0A00)
#define  TKEY_CHARGE1_TKCG13_71C5                    ((uint32_t)0x0C00)
#define  TKEY_CHARGE1_TKCG13_239C5                   ((uint32_t)0x0E00)

#define  TKEY_CHARGE1_TKCG14                         ((uint32_t)0x7000)            
#define  TKEY_CHARGE1_TKCG15                         ((uint32_t)0x38000)            

/******************************************************************************/
/*                                  SDMMC                                       */
/******************************************************************************/
/*******************  Bit definition for EMMC_ARGUMENT register  *******************/
#define  EMMC_ARGUMENT                               ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for EMMC_CMD_SET register  *******************/
#define  EMMC_CMDIDX_MASK                            ((uint16_t)0x003F)
#define  EMMC_RPTY_MASK                              ((uint16_t)0x0300)
#define  EMMC_CKCRC                                  ((uint16_t)0x0400)
#define  EMMC_CKIDX                                  ((uint16_t)0x0800)

/*******************  Bit definition for EMMC_RESPONSE0 register  *******************/
#define  EMMC_RESPONSE0                              ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for EMMC_RESPONSE1 register  *******************/
#define  EMMC_RESPONSE1                              ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for EMMC_RESPONSE2 register  *******************/
#define  EMMC_RESPONSE2                              ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for EMMC_RESPONSE3 register  *******************/
#define  EMMC_RESPONSE3                              ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for EMMC_WRITE_CONT register  *******************/
#define  EMMC_WRITE_CONT                             ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for EMMC_CONTROL register  *******************/
#define  EMMC_LW_MASK                                ((uint16_t)0x0003)
#define  EMMC_LW_MASK_0                              ((uint16_t)0x0001)
#define  EMMC_LW_MASK_1                              ((uint16_t)0x0002)

#define  EMMC_ALL_CLR                                ((uint16_t)0x0004)
#define  EMMC_DMAEN                                  ((uint16_t)0x0008)
#define  EMMC_RST_LGC                                ((uint16_t)0x0010)
#define  EMMC_NEGSMP                                 ((uint16_t)0x0020)
#define  EMMC_SLV_MODE                               ((uint16_t)0x0100)
#define  EMMC_SLV_FORCE_ERR                          ((uint16_t)000200)

/*******************  Bit definition for EMMC_TIMEOUT register  *******************/
#define  EMMC_TOCNT_MASK                             ((uint8_t)0x0F)

/*******************  Bit definition for EMMC_STATUS register  *******************/
#define  EMMC_MASK_BLOCK_NUM                         ((uint16_t)0xFFFF)
#define  EMMC_CMDSTA                                 ((uint32_t)0x00010000)
#define  EMMC_DAT0STA                                ((uint32_t)0x00020000)

/*******************  Bit definition for EMMC_INT_FG register  *******************/
#define  EMMC_IF_RE_TMOUT                            ((uint16_t)0x0001)
#define  EMMC_IF_RECRC_WR                            ((uint16_t)0x0002)
#define  EMMC_IF_REIDX_ER                            ((uint16_t)0x0004)
#define  EMMC_IF_CMDDONE                             ((uint16_t)0x0008)
#define  EMMC_IF_DATTMO                              ((uint16_t)0x0010)
#define  EMMC_IF_TRANERR                             ((uint16_t)0x0020)
#define  EMMC_IF_TRANDONE                            ((uint16_t)0x0040)
#define  EMMC_IF_BKGAP                               ((uint16_t)0x0080)
#define  EMMC_IF_FIFO_OV                             ((uint16_t)0x0100)
#define  EMMC_IF_SDIOINT                             ((uint16_t)0x0200)
#define  EMMC_SIF_SLV_BUF_RELEAS                     ((uint16_t)0x0400)

/*******************  Bit definition for EMMC_INT_EN register  *******************/
#define  EMMC_IE_RE_TMOUT                            ((uint16_t)0x0001)
#define  EMMC_IE_RECRC_WR                            ((uint16_t)0x0002)
#define  EMMC_IE_REIDX_ER                            ((uint16_t)0x0004)
#define  EMMC_IE_CMDDONE                             ((uint16_t)0x0008)
#define  EMMC_IE_DATTMO                              ((uint16_t)0x0010)
#define  EMMC_IE_TRANERR                             ((uint16_t)0x0020)
#define  EMMC_IE_TRANDONE                            ((uint16_t)0x0040)
#define  EMMC_IE_FIFO_OV                             ((uint16_t)0x0080)
#define  EMMC_IE_SDIOINT                             ((uint16_t)0x0100)

/*******************  Bit definition for EMMC_DMA_BEG1 register  *******************/
#define  EMMC_DMAAD1_MASK                            ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for EMMC_BLOCK_CFG register  *******************/
#define  EMMC_BKNUM_MASK                             ((uint32_t)0x0000FFFF)
#define  EMMC_BKSIZE_MASK                            ((uint32_t)0x0FFF0000)

/*******************  Bit definition for EMMC_TRAN_MODE register  *******************/
#define  EMMC_DMA_DIR                                ((uint32_t)0x00000001)
#define  EMMC_GAP_STOP                               ((uint32_t)0x00000002)
#define  EMMC_MODE_BOOT                              ((uint32_t)0x00000004)
#define  EMMC_AUTOGAPSTOP                            ((uint32_t)0x00000010)

#define  EMMC_DMATN_CNT                              ((uint32_t)0x00007F00)

#define  EMMC_DULEDMA_EN                             ((uint32_t)0x00010000)
#define  EMMC_DDR_MODE                               ((uint32_t)0x00020000)
#define  EMMC_CARE_NEG                               ((uint32_t)0x00040000)

#define  EMMC_SW                                     ((uint32_t)0x00180000)
#define  EMMC_SW_0                                   ((uint32_t)0x00080000)
#define  EMMC_SW_1                                   ((uint32_t)0x00100000)

/*******************  Bit definition for EMMC_CLK_DIV register  *******************/
#define  EMMC_DIV_MASK                               ((uint32_t)0x0000000F)
#define  EMMC_CLKOE                                  ((uint32_t)0x00000100)
#define  EMMC_CLKMode                                ((uint32_t)0x00000200)
#define  EMMC_PHASEINV                               ((uint32_t)0x00000400)

/*******************  Bit definition for EMMC_DMA_BEG2 register  *******************/
#define  EMMC_DMAAD2_MASK                            ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for EMMC_TUNE_DATO register  *******************/
#define  EMMC_TUNNE_DAT0_O                           ((uint32_t)0x0000000F)
#define  EMMC_TUNNE_DAT1_O                           ((uint32_t)0x000000F0)
#define  EMMC_TUNNE_DAT2_O                           ((uint32_t)0x00000F00)
#define  EMMC_TUNNE_DAT3_O                           ((uint32_t)0x0000F000)
#define  EMMC_TUNNE_DAT4_O                           ((uint32_t)0x000F0000)
#define  EMMC_TUNNE_DAT5_O                           ((uint32_t)0x00F00000)
#define  EMMC_TUNNE_DAT6_O                           ((uint32_t)0x0F000000)
#define  EMMC_TUNNE_DAT7_O                           ((uint32_t)0xF0000000)

/*******************  Bit definition for EMMC_TUNE_DATI register  *******************/
#define  EMMC_TUNNE_DAT0_I                           ((uint32_t)0x0000000F)
#define  EMMC_TUNNE_DAT1_I                           ((uint32_t)0x000000F0)
#define  EMMC_TUNNE_DAT2_I                           ((uint32_t)0x00000F00)
#define  EMMC_TUNNE_DAT3_I                           ((uint32_t)0x0000F000)
#define  EMMC_TUNNE_DAT4_I                           ((uint32_t)0x000F0000)
#define  EMMC_TUNNE_DAT5_I                           ((uint32_t)0x00F00000)
#define  EMMC_TUNNE_DAT6_I                           ((uint32_t)0x0F000000)
#define  EMMC_TUNNE_DAT7_I                           ((uint32_t)0xF0000000)

/*******************  Bit definition for EMMC_TUNE_CLK_CMD register  *******************/
#define  EMMC_TUNNE_CLK_O                            ((uint32_t)0x0000000F)
#define  EMMC_TUNNE_CLK_I                            ((uint32_t)0x000000F0)
#define  EMMC_TUNNE_CMD_O                            ((uint32_t)0x000F0000)
#define  EMMC_TUNNE_CMD_I                            ((uint32_t)0x00F00000)

/******************************************************************************/
/*                                  SAI                                       */
/******************************************************************************/
/*******************  Bit definition for SAI_xCFGR1 register  *******************/
#define  SAI_CFGR1_MODE                              ((uint32_t)0x00000003)
#define  SAI_CFGR1_MODE_0                            ((uint32_t)0x00000001)
#define  SAI_CFGR1_MODE_1                            ((uint32_t)0x00000002)

#define  SAI_CFGR1_PRTCFG                            ((uint32_t)0x0000000C)
#define  SAI_CFGR1_PRTCFG_0                          ((uint32_t)0x00000004)
#define  SAI_CFGR1_PRTCFG_1                          ((uint32_t)0x00000008)

#define  SAI_CFGR1_DS                                ((uint32_t)0x000000E0)
#define  SAI_CFGR1_DS_0                              ((uint32_t)0x00000020)
#define  SAI_CFGR1_DS_1                              ((uint32_t)0x00000040)
#define  SAI_CFGR1_DS_2                              ((uint32_t)0x00000080)

#define  SAI_CFGR1_LSBFIRST                          ((uint32_t)0x00000100)
#define  SAI_CFGR1_CKSTR                             ((uint32_t)0x00000200)

#define  SAI_CFGR1_SYNCEN                            ((uint32_t)0x00000C00)
#define  SAI_CFGR1_SYNCEN_0                          ((uint32_t)0x00000400)
#define  SAI_CFGR1_SYNCEN_1                          ((uint32_t)0x00000800)
#define  SAI_CFGR1_MONO                              ((uint32_t)0x00001000)
#define  SAI_CFGR1_OUTDRIV                           ((uint32_t)0x00002000)
#define  SAI_CFGR1_EN                                ((uint32_t)0x00010000)
#define  SAI_CFGR1_DMAEN                             ((uint32_t)0x00020000)
#define  SAI_CFGR1_NODIV                             ((uint32_t)0x00080000)
#define  SAI_CFGR1_MCKDIV                            ((uint32_t)0x03F00000)
#define  SAI_CFGR1_OSR                               ((uint32_t)0x04000000)

/*******************  Bit definition for SAI_xCFGR2 register  *******************/
#define  SAI_CFGR2_FTH                               ((uint32_t)0x00000007)
#define  SAI_CFGR2_FTH_0                             ((uint32_t)0x00000001)
#define  SAI_CFGR2_FTH_1                             ((uint32_t)0x00000002)
#define  SAI_CFGR2_FTH_2                             ((uint32_t)0x00000004)

#define  SAI_CFGR2_FFLUSH                            ((uint32_t)0x00000008)
#define  SAI_CFGR2_TRIS                              ((uint32_t)0x00000010)
#define  SAI_CFGR2_MUTE                              ((uint32_t)0x00000020)
#define  SAI_CFGR2_MUTEVAL                           ((uint32_t)0x00000040)

#define  SAI_CFGR2_MUTECNT                           ((uint32_t)0x00001F80)

#define  SAI_CFGR2_CPL                               ((uint32_t)0x00002000)
#define  SAI_CFGR2_COMP                              ((uint32_t)0x0000C000)

/*******************  Bit definition for SAI_xFRCR register  *******************/
#define  SAI_FRCR_FRL                                ((uint32_t)0x000000FF)

#define  SAI_FRCR_FSALL                              ((uint32_t)0x00007F00)
#define  SAI_FRCR_FSDEF                              ((uint32_t)0x00010000)
#define  SAI_FRCR_FSPOL                              ((uint32_t)0x00020000)
#define  SAI_FRCR_FSOFF                              ((uint32_t)0x00040000)

/*******************  Bit definition for SAI_xSLOTR register  *******************/
#define  SAI_SLOTR_FBOFF                             ((uint32_t)0x0000001F)
#define  SAI_SLOTR_SLOTSZ                            ((uint32_t)0x000000C0)
#define  SAI_SLOTR_NBSLOT                            ((uint32_t)0x00000F00)
#define  SAI_SLOTR_SLOTEN                            ((uint32_t)0xFFFF0000)

/*******************  Bit definition for SAI_xINTENR register  *******************/
#define  SAI_INTENR_OVRUDRIE                         ((uint32_t)0x00000001)
#define  SAI_INTENR_MUTEDETIE                        ((uint32_t)0x00000002)
#define  SAI_INTENR_WCKCFGIE                         ((uint32_t)0x00000004)
#define  SAI_INTENR_FREQIE                           ((uint32_t)0x00000008)
#define  SAI_INTENR_CNRDYIE                          ((uint32_t)0x00000010)
#define  SAI_INTENR_AFSDETIE                         ((uint32_t)0x00000020)
#define  SAI_INTENR_LFSDETIE                         ((uint32_t)0x00000040)

/*********************  Bit definition for SAI_xSR register  *********************/
#define  SAI_SR_OVRUDR                               ((uint32_t)0x00000001)
#define  SAI_SR_MUTEDET                              ((uint32_t)0x00000002)
#define  SAI_SR_WCKCFG                               ((uint32_t)0x00000004)
#define  SAI_SR_FREQ                                 ((uint32_t)0x00000008)
#define  SAI_SR_CNRDY                                ((uint32_t)0x00000010)
#define  SAI_SR_AFSDET                               ((uint32_t)0x00000020)
#define  SAI_SR_LFSDET                               ((uint32_t)0x00000040)

#define  SAI_SR_FLTH                                 ((uint32_t)0x00070000)
#define  SAI_SR_FLTH_0                               ((uint32_t)0x00010000)
#define  SAI_SR_FLTH_1                               ((uint32_t)0x00020000)
#define  SAI_SR_FLTH_2                               ((uint32_t)0x00030000)

/*********************  Bit definition for SAI_xDATAR register  *********************/
#define  SAI_DATAR_DR                                ((uint32_t)0xFFFFFFFF)

/******************************************************************************/
/*                                 SERDES                                     */
/******************************************************************************/
/*********************  Bit definition for SERDES_CTRL register  *********************/
#define  SDS_CLR_ALL                                 ((uint32_t)0x00000001)
#define  SDS_RESET_LINK                              ((uint32_t)0x00000002)
#define  SDS_RESET_PHY                               ((uint32_t)0x00000004)
#define  SDS_INT_BUSY_EN                             ((uint32_t)0x00000008)
#define  SDS_RX_POLARITY                             ((uint32_t)0x00000010)
#define  SDS_RX_EN                                   ((uint32_t)0x00000020)
#define  SDS_TX_EN                                   ((uint32_t)0x00000040)
#define  SDS_DMA_EN                                  ((uint32_t)0x00000080)
#define  SDS_PLL_FACTOR                              ((uint32_t)0x00001F00)
#define  SDS_PLL_PWR_UP                              ((uint32_t)0x00002000)
#define  SDS_RX_PWR_UP                               ((uint32_t)0x00004000)
#define  SDS_TX_PWR_UP                               ((uint32_t)0x00008000)
#define  SDS_PHY_PWR_UP                              ((uint32_t)0x00010000)
#define  SDS_CONT_EN                                 ((uint32_t)0x00020000)
#define  SDS_ALIGN_EN                                ((uint32_t)0x00040000)



/*********************  Bit definition for SERDES_INT_EN register  *********************/
#define  SDS_PHYRDY_IE                               ((uint32_t)0x00000001)
#define  SDS_RECV_ERR_IE                             ((uint32_t)0x00000002)
#define  SDS_TRAN_DONE_IE                            ((uint32_t)0x00000002)
#define  SDS_RECV_DONE_IE                            ((uint32_t)0x00000004)
#define  SDS_FIFO_OV_IE                              ((uint32_t)0x00000008)
#define  SDS_COMINIT_IE                              ((uint32_t)0x00000020)

/*********************  Bit definition for SERDES_INT_FG register  *********************/
#define  SDS_PHYRDY_IF                               ((uint32_t)0x00000001)
#define  SDS_RECV_ERR_IF                             ((uint32_t)0x00000002)
#define  SDS_TRAN_DONE_IF                            ((uint32_t)0x00000002)
#define  SDS_RECV_DONE_IF                            ((uint32_t)0x00000004)
#define  SDS_FIFO_OV_IF                              ((uint32_t)0x00000008)
#define  SDS_COMINIT_IF                              ((uint32_t)0x00000020)
#define  SDS_PHYRDY                                  ((uint32_t)0x00010000)
#define  SDS_RX_SEQ_MATCH                            ((uint32_t)0x00020000)
#define  SDS_RECV_CRC_OK                             ((uint32_t)0x00040000)
#define  SDS_PLL_LOCK                                ((uint32_t)0x00080000)
#define  SDS_LINK_FREE                               ((uint32_t)0x00100000)
#define  SDS_R_FIFO_RDY                              ((uint32_t)0x00200000)
#define  SDS_RX_SEQ_NUM                              ((uint32_t)0x0F000000)
#define  SDS_TX_SEQ_NUM                              ((uint32_t)0xF0000000)

/*********************  Bit definition for SERDES_RTX_CTRL register  *********************/
#define  SDS_SERDES_TX_LEN                           ((uint32_t)0x0000FFFF)
#define  SDS_LINK_INIT                               ((uint32_t)0x00010000)
#define  SDS_TX_VLD                                  ((uint32_t)0x00020000)
#define  SDS_BUF_MODE                                ((uint32_t)0x00040000)

/*********************  Bit definition for SERDES_RX_LEN0 register  *********************/
#define  SDS_SERDES_RX_LEN0                          ((uint32_t)0x0000FFFF)

/*********************  Bit definition for SERDES_DATA0 register  *********************/
#define  SDS_SERDES_DATA0                            ((uint32_t)0xFFFFFFFF)

/*********************  Bit definition for SERDES_DMA0 register  *********************/
#define  SDS_SERDES_DMA0                             ((uint32_t)0xFFFFFFFF)

/*********************  Bit definition for SERDES_RX_LEN1 register  *********************/
#define  SDS_SERDES_RX_LEN1                          ((uint32_t)0x0000FFFF)

/*********************  Bit definition for SERDES_DATA1 register  *********************/
#define  SDS_SERDES_DATA1                            ((uint32_t)0xFFFFFFFF)

/*********************  Bit definition for SERDES_DMA1 register  *********************/
#define  SDS_SERDES_DMA1                             ((uint32_t)0xFFFFFFFF)


/******************************************************************************/
/*                                 SWPMI                                      */
/******************************************************************************/
/*******************  Bit definition for SWPMI_CR register  *******************/
#define  SWPMI_RXDMA                                 ((uint32_t)0x00000001)
#define  SWPMI_TXDMA                                 ((uint32_t)0x00000002)
#define  SWPMI_RXMODE                                ((uint32_t)0x00000004)
#define  SWPMI_TXMODE                                ((uint32_t)0x00000008)
#define  SWPMI_LPBK                                  ((uint32_t)0x00000010)
#define  SWPMI_SWPACT                                ((uint32_t)0x00000020)
#define  SWPMI_DEACT                                 ((uint32_t)0x00000400)
#define  SWPMI_SWPTEN                                ((uint32_t)0x00000800)

/*******************  Bit definition for SWPMI_BRR register  *******************/
#define  SWPMI_BR                                    ((uint32_t)0x000000FF)

/*******************  Bit definition for SWPMI_ISR register  *******************/
#define  SWPMI_RXBFF                                 ((uint32_t)0x00000001)
#define  SWPMI_TXBEF                                 ((uint32_t)0x00000002)
#define  SWPMI_RXBERF                                ((uint32_t)0x00000004)
#define  SWPMI_RXOVRF                                ((uint32_t)0x00000008)
#define  SWPMI_TXUNRF                                ((uint32_t)0x00000010)
#define  SWPMI_RXNE                                  ((uint32_t)0x00000020)
#define  SWPMI_TXE                                   ((uint32_t)0x00000040)
#define  SWPMI_TCF                                   ((uint32_t)0x00000080)
#define  SWPMI_SRF                                   ((uint32_t)0x00000100)
#define  SWPMI_SUSP                                  ((uint32_t)0x00000200)
#define  SWPMI_DEACTF                                ((uint32_t)0x00000400)
#define  SWPMI_RDYF                                  ((uint32_t)0x00000800)

/*******************  Bit definition for SWPMI_ICR register  *******************/
#define  SWPMI_CRXBFF                                ((uint32_t)0x00000001)
#define  SWPMI_CTXBEF                                ((uint32_t)0x00000002)
#define  SWPMI_CRXBERF                               ((uint32_t)0x00000004)
#define  SWPMI_CRXOVRF                               ((uint32_t)0x00000008)
#define  SWPMI_CTXUNRF                               ((uint32_t)0x00000010)
#define  SWPMI_CTCF                                  ((uint32_t)0x00000080)
#define  SWPMI_CSRF                                  ((uint32_t)0x00000100)
#define  SWPMI_CRDYF                                 ((uint32_t)0x00000800)

/*******************  Bit definition for SWPMI_IER register  *******************/
#define  SWPMI_RXBFIE                                ((uint32_t)0x00000001)
#define  SWPMI_TXBEIE                                ((uint32_t)0x00000002)
#define  SWPMI_RXBERIE                               ((uint32_t)0x00000004)
#define  SWPMI_RXOVRIE                               ((uint32_t)0x00000008)
#define  SWPMI_TXUNRIE                               ((uint32_t)0x00000010)
#define  SWPMI_RIE                                   ((uint32_t)0x00000020)
#define  SWPMI_TIE                                   ((uint32_t)0x00000040)
#define  SWPMI_TCIE                                  ((uint32_t)0x00000080)
#define  SWPMI_SRIE                                  ((uint32_t)0x00000100)
#define  SWPMI_RDYIE                                 ((uint32_t)0x00000800)

/*******************  Bit definition for SWPMI_RFL register  *******************/
#define  SWPMI_RFL                                   ((uint32_t)0x0000001F)

/*******************  Bit definition for SWPMI_TDR register  *******************/
#define  SWPMI_TD                                    ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for SWPMI_OR register  *******************/
#define  SWPMI_SWP_TBYP                              ((uint32_t)0x00000001)

#define  SWPMI_SWP_ISEL                              ((uint32_t)0x0000000C)
#define  SWPMI_SWP_ISEL_0                            ((uint32_t)0x00000004)
#define  SWPMI_SWP_ISEL_1                            ((uint32_t)0x00000008)

/******************************************************************************/
/*                                 FMC                                      */
/******************************************************************************/
/*******************  Bit definition for FMC_BCR1 register  *******************/
#define  FMC_BCR1_MBKEN                              ((uint32_t)0x00000001)
#define  FMC_BCR1_MUXEN                              ((uint32_t)0x00000002)

#define  FMC_BCR1_MTYP                               ((uint32_t)0x0000000C)
#define  FMC_BCR1_MTYP_0                             ((uint32_t)0x00000004)
#define  FMC_BCR1_MTYP_1                             ((uint32_t)0x00000008)

#define  FMC_BCR1_MWID                               ((uint32_t)0x00000030)
#define  FMC_BCR1_MWID_0                             ((uint32_t)0x00000010)
#define  FMC_BCR1_MWID_1                             ((uint32_t)0x00000020)

#define  FMC_BCR1_MFACCEN                            ((uint32_t)0x00000040)
#define  FMC_BCR1_BURSTEN                            ((uint32_t)0x00000100)
#define  FMC_BCR1_WAITPOL                            ((uint32_t)0x00000200)
#define  FMC_BCR1_WAITCFG                            ((uint32_t)0x00000800)

#define  FMC_BCR1_WREN                               ((uint32_t)0x00001000)
#define  FMC_BCR1_WAITEN                             ((uint32_t)0x00002000)
#define  FMC_BCR1_EXTMOD                             ((uint32_t)0x00004000)
#define  FMC_BCR1_ASYNCWAIT                          ((uint32_t)0x00008000)

#define  FMC_BCR1_CPSIZE                             ((uint32_t)0x00070000)
#define  FMC_BCR1_CPSIZE_0                           ((uint32_t)0x00010000)
#define  FMC_BCR1_CPSIZE_1                           ((uint32_t)0x00020000)
#define  FMC_BCR1_CPSIZE_2                           ((uint32_t)0x00040000)

#define  FMC_BCR1_CBURSTRW                           ((uint32_t)0x00080000)

#define  FMC_BCR1_BMP                                ((uint32_t)0x03000000)
#define  FMC_BCR1_BMP_0                              ((uint32_t)0x01000000)
#define  FMC_BCR1_BMP_1                              ((uint32_t)0x02000000)

#define  FMC_BCR1_FMCEN                              ((uint32_t)0x80000000)

/*******************  Bit definition for FMC_BCR2 register  *******************/
#define  FMC_BCR2_MBKEN                              ((uint32_t)0x00000001)
#define  FMC_BCR2_MUXEN                              ((uint32_t)0x00000002)

#define  FMC_BCR2_MTYP                               ((uint32_t)0x0000000C)
#define  FMC_BCR2_MTYP_0                             ((uint32_t)0x00000004)
#define  FMC_BCR2_MTYP_1                             ((uint32_t)0x00000008)

#define  FMC_BCR2_MWID                               ((uint32_t)0x00000030)
#define  FMC_BCR2_MWID_0                             ((uint32_t)0x00000010)
#define  FMC_BCR2_MWID_1                             ((uint32_t)0x00000020)

#define  FMC_BCR2_FACCEN                             ((uint32_t)0x00000040)
#define  FMC_BCR2_BURSTEN                            ((uint32_t)0x00000100)
#define  FMC_BCR2_WAITPOL                            ((uint32_t)0x00000200)
#define  FMC_BCR2_WAITCFG                            ((uint32_t)0x00000800)
#define  FMC_BCR2_WREN                               ((uint32_t)0x00001000)
#define  FMC_BCR2_WAITEN                             ((uint32_t)0x00002000)
#define  FMC_BCR2_EXTMOD                             ((uint32_t)0x00004000)
#define  FMC_BCR2_ASYNCWAIT                          ((uint32_t)0x00008000)

#define  FMC_BCR2_CPSZIE                             ((uint32_t)0x00070000)
#define  FMC_BCR2_CPSZIE_0                           ((uint32_t)0x00010000)
#define  FMC_BCR2_CPSZIE_1                           ((uint32_t)0x00020000)
#define  FMC_BCR2_CPSZIE_2                           ((uint32_t)0x00040000)

#define  FMC_BCR2_CBURSTRW                           ((uint32_t)0x00080000)
#define  FMC_BCR2_BMP                                ((uint32_t)0x03000000)
#define  FMC_BCR2_BMP_0                              ((uint32_t)0x01000000)
#define  FMC_BCR2_BMP_1                              ((uint32_t)0x02000000)

#define  FMC_BCR2_FMCEN                              ((uint32_t)0x80000000)

/*******************  Bit definition for FMC_BCR3 register  *******************/
#define  FMC_BCR3_MBKEN                              ((uint32_t)0x00000001)
#define  FMC_BCR3_MUXEN                              ((uint32_t)0x00000002)

#define  FMC_BCR3_MTYP                               ((uint32_t)0x0000000C)
#define  FMC_BCR3_MTYP_0                             ((uint32_t)0x00000004)
#define  FMC_BCR3_MTYP_1                             ((uint32_t)0x00000008)

#define  FMC_BCR3_MWID                               ((uint32_t)0x00000030)
#define  FMC_BCR3_MWID_0                             ((uint32_t)0x00000010)
#define  FMC_BCR3_MWID_1                             ((uint32_t)0x00000020)

#define  FMC_BCR3_FACCEN                             ((uint32_t)0x00000040)
#define  FMC_BCR3_BURSTEN                            ((uint32_t)0x00000100)
#define  FMC_BCR3_WAITPOL                            ((uint32_t)0x00000200)
#define  FMC_BCR3_WAITCFG                            ((uint32_t)0x00000800)
#define  FMC_BCR3_WREN                               ((uint32_t)0x00001000)
#define  FMC_BCR3_WAITEN                             ((uint32_t)0x00002000)
#define  FMC_BCR3_EXTMOD                             ((uint32_t)0x00004000)
#define  FMC_BCR3_ASYNCWAIT                          ((uint32_t)0x00008000)

#define  FMC_BCR3_CPSZIE                             ((uint32_t)0x00070000)
#define  FMC_BCR3_CPSZIE_0                           ((uint32_t)0x00010000)
#define  FMC_BCR3_CPSZIE_1                           ((uint32_t)0x00020000)
#define  FMC_BCR3_CPSZIE_2                           ((uint32_t)0x00040000)

#define  FMC_BCR3_CBURSTRW                           ((uint32_t)0x00080000)
#define  FMC_BCR3_BMP                                ((uint32_t)0x03000000)
#define  FMC_BCR3_BMP_0                              ((uint32_t)0x01000000)
#define  FMC_BCR3_BMP_1                              ((uint32_t)0x02000000)

#define  FMC_BCR3_FMCEN                              ((uint32_t)0x80000000)

/*******************  Bit definition for FMC_BCR4 register  *******************/
#define  FMC_BCR4_MBKEN                              ((uint32_t)0x00000001)
#define  FMC_BCR4_MUXEN                              ((uint32_t)0x00000002)

#define  FMC_BCR4_MTYP                               ((uint32_t)0x0000000C)
#define  FMC_BCR4_MTYP_0                             ((uint32_t)0x00000004)
#define  FMC_BCR4_MTYP_1                             ((uint32_t)0x00000008)

#define  FMC_BCR4_MWID                               ((uint32_t)0x00000030)
#define  FMC_BCR4_MWID_0                             ((uint32_t)0x00000010)
#define  FMC_BCR4_MWID_1                             ((uint32_t)0x00000020)

#define  FMC_BCR4_FACCEN                             ((uint32_t)0x00000040)
#define  FMC_BCR4_BURSTEN                            ((uint32_t)0x00000100)
#define  FMC_BCR4_WAITPOL                            ((uint32_t)0x00000200)
#define  FMC_BCR4_WAITCFG                            ((uint32_t)0x00000800)
#define  FMC_BCR4_WREN                               ((uint32_t)0x00001000)
#define  FMC_BCR4_WAITEN                             ((uint32_t)0x00002000)
#define  FMC_BCR4_EXTMOD                             ((uint32_t)0x00004000)
#define  FMC_BCR4_ASYNCWAIT                          ((uint32_t)0x00008000)

#define  FMC_BCR4_CPSZIE                             ((uint32_t)0x00070000)
#define  FMC_BCR4_CPSZIE_0                           ((uint32_t)0x00010000)
#define  FMC_BCR4_CPSZIE_1                           ((uint32_t)0x00020000)
#define  FMC_BCR4_CPSZIE_2                           ((uint32_t)0x00040000)

#define  FMC_BCR4_CBURSTRW                           ((uint32_t)0x00080000)
#define  FMC_BCR4_BMP                                ((uint32_t)0x03000000)
#define  FMC_BCR4_BMP_0                              ((uint32_t)0x01000000)
#define  FMC_BCR4_BMP_1                              ((uint32_t)0x02000000)

#define  FMC_BCR4_FMCEN                              ((uint32_t)0x80000000)

/*******************  Bit definition for FMC_BTR1 register  *******************/
#define  FMC_BTR1_ADDSET                             ((uint32_t)0x0000000F)
#define  FMC_BTR1_ADDSET_0                           ((uint32_t)0x00000001)
#define  FMC_BTR1_ADDSET_1                           ((uint32_t)0x00000002)
#define  FMC_BTR1_ADDSET_2                           ((uint32_t)0x00000004)
#define  FMC_BTR1_ADDSET_3                           ((uint32_t)0x00000008)

#define  FMC_BTR1_ADDHLD                             ((uint32_t)0x000000F0)
#define  FMC_BTR1_ADDHLD_0                           ((uint32_t)0x00000010)
#define  FMC_BTR1_ADDHLD_1                           ((uint32_t)0x00000020)
#define  FMC_BTR1_ADDHLD_2                           ((uint32_t)0x00000040)
#define  FMC_BTR1_ADDHLD_3                           ((uint32_t)0x00000080)

#define  FMC_BTR1_DATAST                             ((uint32_t)0x0000FF00)
#define  FMC_BTR1_DATAST_0                           ((uint32_t)0x00000100)
#define  FMC_BTR1_DATAST_1                           ((uint32_t)0x00000200)
#define  FMC_BTR1_DATAST_2                           ((uint32_t)0x00000400)
#define  FMC_BTR1_DATAST_3                           ((uint32_t)0x00000800)
#define  FMC_BTR1_DATAST_4                           ((uint32_t)0x00001000)
#define  FMC_BTR1_DATAST_5                           ((uint32_t)0x00002000)
#define  FMC_BTR1_DATAST_6                           ((uint32_t)0x00004000)
#define  FMC_BTR1_DATAST_7                           ((uint32_t)0x00008000)

#define  FMC_BTR1_BUSTURN                            ((uint32_t)0x000F0000)
#define  FMC_BTR1_BUSTURN_0                          ((uint32_t)0x00010000)
#define  FMC_BTR1_BUSTURN_1                          ((uint32_t)0x00020000)
#define  FMC_BTR1_BUSTURN_2                          ((uint32_t)0x00040000)
#define  FMC_BTR1_BUSTURN_3                          ((uint32_t)0x00080000)

#define  FMC_BTR1_CLKDIV                             ((uint32_t)0x00F00000)
#define  FMC_BTR1_CLKDIV_0                           ((uint32_t)0x00100000)
#define  FMC_BTR1_CLKDIV_1                           ((uint32_t)0x00200000)
#define  FMC_BTR1_CLKDIV_2                           ((uint32_t)0x00400000)
#define  FMC_BTR1_CLKDIV_3                           ((uint32_t)0x00800000)

#define  FMC_BTR1_DATLAT                             ((uint32_t)0x0F000000)
#define  FMC_BTR1_DATLAT_0                           ((uint32_t)0x01000000)
#define  FMC_BTR1_DATLAT_1                           ((uint32_t)0x02000000)
#define  FMC_BTR1_DATLAT_2                           ((uint32_t)0x04000000)
#define  FMC_BTR1_DATLAT_3                           ((uint32_t)0x08000000)

#define  FMC_BTR1_ACCMOD                             ((uint32_t)0x30000000)
#define  FMC_BTR1_ACCMOD_0                           ((uint32_t)0x10000000)
#define  FMC_BTR1_ACCMOD_1                           ((uint32_t)0x20000000)

/*******************  Bit definition for FMC_BTR2 register  *******************/
#define  FMC_BTR2_ADDSET                             ((uint32_t)0x0000000F)
#define  FMC_BTR2_ADDSET_0                           ((uint32_t)0x00000001)
#define  FMC_BTR2_ADDSET_1                           ((uint32_t)0x00000002)
#define  FMC_BTR2_ADDSET_2                           ((uint32_t)0x00000004)
#define  FMC_BTR2_ADDSET_3                           ((uint32_t)0x00000008)

#define  FMC_BTR2_ADDHLD                             ((uint32_t)0x000000F0)
#define  FMC_BTR2_ADDHLD_0                           ((uint32_t)0x00000010)
#define  FMC_BTR2_ADDHLD_1                           ((uint32_t)0x00000020)
#define  FMC_BTR2_ADDHLD_2                           ((uint32_t)0x00000040)
#define  FMC_BTR2_ADDHLD_3                           ((uint32_t)0x00000080)

#define  FMC_BTR2_DATAST                             ((uint32_t)0x0000FF00)
#define  FMC_BTR2_DATAST_0                           ((uint32_t)0x00000100)
#define  FMC_BTR2_DATAST_1                           ((uint32_t)0x00000200)
#define  FMC_BTR2_DATAST_2                           ((uint32_t)0x00000400)
#define  FMC_BTR2_DATAST_3                           ((uint32_t)0x00000800)
#define  FMC_BTR2_DATAST_4                           ((uint32_t)0x00001000)
#define  FMC_BTR2_DATAST_5                           ((uint32_t)0x00002000)
#define  FMC_BTR2_DATAST_6                           ((uint32_t)0x00004000)
#define  FMC_BTR2_DATAST_7                           ((uint32_t)0x00008000)

#define  FMC_BTR2_BUSTURN                            ((uint32_t)0x000F0000)
#define  FMC_BTR2_BUSTURN_0                          ((uint32_t)0x00010000)
#define  FMC_BTR2_BUSTURN_1                          ((uint32_t)0x00020000)
#define  FMC_BTR2_BUSTURN_2                          ((uint32_t)0x00040000)
#define  FMC_BTR2_BUSTURN_3                          ((uint32_t)0x00080000)

#define  FMC_BTR2_CLKDIV                             ((uint32_t)0x00F00000)
#define  FMC_BTR2_CLKDIV_0                           ((uint32_t)0x00100000)
#define  FMC_BTR2_CLKDIV_1                           ((uint32_t)0x00200000)
#define  FMC_BTR2_CLKDIV_2                           ((uint32_t)0x00400000)
#define  FMC_BTR2_CLKDIV_3                           ((uint32_t)0x00800000)

#define  FMC_BTR2_DATLAT                             ((uint32_t)0x0F000000)
#define  FMC_BTR2_DATLAT_0                           ((uint32_t)0x01000000)
#define  FMC_BTR2_DATLAT_1                           ((uint32_t)0x02000000)
#define  FMC_BTR2_DATLAT_2                           ((uint32_t)0x04000000)
#define  FMC_BTR2_DATLAT_3                           ((uint32_t)0x08000000)

#define  FMC_BTR2_ACCMOD                             ((uint32_t)0x30000000)
#define  FMC_BTR2_ACCMOD_0                           ((uint32_t)0x10000000)
#define  FMC_BTR2_ACCMOD_1                           ((uint32_t)0x20000000)

/*******************  Bit definition for FMC_BTR3 register  *******************/
#define  FMC_BTR3_ADDSET                             ((uint32_t)0x0000000F)
#define  FMC_BTR3_ADDSET_0                           ((uint32_t)0x00000001)
#define  FMC_BTR3_ADDSET_1                           ((uint32_t)0x00000002)
#define  FMC_BTR3_ADDSET_2                           ((uint32_t)0x00000004)
#define  FMC_BTR3_ADDSET_3                           ((uint32_t)0x00000008)

#define  FMC_BTR3_ADDHLD                             ((uint32_t)0x000000F0)
#define  FMC_BTR3_ADDHLD_0                           ((uint32_t)0x00000010)
#define  FMC_BTR3_ADDHLD_1                           ((uint32_t)0x00000020)
#define  FMC_BTR3_ADDHLD_2                           ((uint32_t)0x00000040)
#define  FMC_BTR3_ADDHLD_3                           ((uint32_t)0x00000080)

#define  FMC_BTR3_DATAST                             ((uint32_t)0x0000FF00)
#define  FMC_BTR3_DATAST_0                           ((uint32_t)0x00000100)
#define  FMC_BTR3_DATAST_1                           ((uint32_t)0x00000200)
#define  FMC_BTR3_DATAST_2                           ((uint32_t)0x00000400)
#define  FMC_BTR3_DATAST_3                           ((uint32_t)0x00000800)
#define  FMC_BTR3_DATAST_4                           ((uint32_t)0x00001000)
#define  FMC_BTR3_DATAST_5                           ((uint32_t)0x00002000)
#define  FMC_BTR3_DATAST_6                           ((uint32_t)0x00004000)
#define  FMC_BTR3_DATAST_7                           ((uint32_t)0x00008000)

#define  FMC_BTR3_BUSTURN                            ((uint32_t)0x000F0000)
#define  FMC_BTR3_BUSTURN_0                          ((uint32_t)0x00010000)
#define  FMC_BTR3_BUSTURN_1                          ((uint32_t)0x00020000)
#define  FMC_BTR3_BUSTURN_2                          ((uint32_t)0x00040000)
#define  FMC_BTR3_BUSTURN_3                          ((uint32_t)0x00080000)

#define  FMC_BTR3_CLKDIV                             ((uint32_t)0x00F00000)
#define  FMC_BTR3_CLKDIV_0                           ((uint32_t)0x00100000)
#define  FMC_BTR3_CLKDIV_1                           ((uint32_t)0x00200000)
#define  FMC_BTR3_CLKDIV_2                           ((uint32_t)0x00400000)
#define  FMC_BTR3_CLKDIV_3                           ((uint32_t)0x00800000)

#define  FMC_BTR3_DATLAT                             ((uint32_t)0x0F000000)
#define  FMC_BTR3_DATLAT_0                           ((uint32_t)0x01000000)
#define  FMC_BTR3_DATLAT_1                           ((uint32_t)0x02000000)
#define  FMC_BTR3_DATLAT_2                           ((uint32_t)0x04000000)
#define  FMC_BTR3_DATLAT_3                           ((uint32_t)0x08000000)

#define  FMC_BTR3_ACCMOD                             ((uint32_t)0x30000000)
#define  FMC_BTR3_ACCMOD_0                           ((uint32_t)0x10000000)
#define  FMC_BTR3_ACCMOD_1                           ((uint32_t)0x20000000)

/*******************  Bit definition for FMC_BTR4 register  *******************/
#define  FMC_BTR4_ADDSET                             ((uint32_t)0x0000000F)
#define  FMC_BTR4_ADDSET_0                           ((uint32_t)0x00000001)
#define  FMC_BTR4_ADDSET_1                           ((uint32_t)0x00000002)
#define  FMC_BTR4_ADDSET_2                           ((uint32_t)0x00000004)
#define  FMC_BTR4_ADDSET_3                           ((uint32_t)0x00000008)

#define  FMC_BTR4_ADDHLD                             ((uint32_t)0x000000F0)
#define  FMC_BTR4_ADDHLD_0                           ((uint32_t)0x00000010)
#define  FMC_BTR4_ADDHLD_1                           ((uint32_t)0x00000020)
#define  FMC_BTR4_ADDHLD_2                           ((uint32_t)0x00000040)
#define  FMC_BTR4_ADDHLD_3                           ((uint32_t)0x00000080)

#define  FMC_BTR4_DATAST                             ((uint32_t)0x0000FF00)
#define  FMC_BTR4_DATAST_0                           ((uint32_t)0x00000100)
#define  FMC_BTR4_DATAST_1                           ((uint32_t)0x00000200)
#define  FMC_BTR4_DATAST_2                           ((uint32_t)0x00000400)
#define  FMC_BTR4_DATAST_3                           ((uint32_t)0x00000800)
#define  FMC_BTR4_DATAST_4                           ((uint32_t)0x00001000)
#define  FMC_BTR4_DATAST_5                           ((uint32_t)0x00002000)
#define  FMC_BTR4_DATAST_6                           ((uint32_t)0x00004000)
#define  FMC_BTR4_DATAST_7                           ((uint32_t)0x00008000)

#define  FMC_BTR4_BUSTURN                            ((uint32_t)0x000F0000)
#define  FMC_BTR4_BUSTURN_0                          ((uint32_t)0x00010000)
#define  FMC_BTR4_BUSTURN_1                          ((uint32_t)0x00020000)
#define  FMC_BTR4_BUSTURN_2                          ((uint32_t)0x00040000)
#define  FMC_BTR4_BUSTURN_3                          ((uint32_t)0x00080000)

#define  FMC_BTR4_CLKDIV                             ((uint32_t)0x00F00000)
#define  FMC_BTR4_CLKDIV_0                           ((uint32_t)0x00100000)
#define  FMC_BTR4_CLKDIV_1                           ((uint32_t)0x00200000)
#define  FMC_BTR4_CLKDIV_2                           ((uint32_t)0x00400000)
#define  FMC_BTR4_CLKDIV_3                           ((uint32_t)0x00800000)

#define  FMC_BTR4_DATLAT                             ((uint32_t)0x0F000000)
#define  FMC_BTR4_DATLAT_0                           ((uint32_t)0x01000000)
#define  FMC_BTR4_DATLAT_1                           ((uint32_t)0x02000000)
#define  FMC_BTR4_DATLAT_2                           ((uint32_t)0x04000000)
#define  FMC_BTR4_DATLAT_3                           ((uint32_t)0x08000000)

#define  FMC_BTR4_ACCMOD                             ((uint32_t)0x30000000)
#define  FMC_BTR4_ACCMOD_0                           ((uint32_t)0x10000000)
#define  FMC_BTR4_ACCMOD_1                           ((uint32_t)0x20000000)

/*******************  Bit definition for FMC_BWTR1 register  *******************/
#define  FMC_BWTR1_ADDSET                            ((uint32_t)0x0000000F)
#define  FMC_BWTR1_ADDSET_0                          ((uint32_t)0x00000001)
#define  FMC_BWTR1_ADDSET_1                          ((uint32_t)0x00000002)
#define  FMC_BWTR1_ADDSET_2                          ((uint32_t)0x00000004)
#define  FMC_BWTR1_ADDSET_3                          ((uint32_t)0x00000008)

#define  FMC_BWTR1_ADDHLD                            ((uint32_t)0x000000F0)
#define  FMC_BWTR1_ADDHLD_0                          ((uint32_t)0x00000010)
#define  FMC_BWTR1_ADDHLD_1                          ((uint32_t)0x00000020)
#define  FMC_BWTR1_ADDHLD_2                          ((uint32_t)0x00000040)
#define  FMC_BWTR1_ADDHLD_3                          ((uint32_t)0x00000080)

#define  FMC_BWTR1_DATAST                            ((uint32_t)0x0000FF00)
#define  FMC_BWTR1_DATAST_0                          ((uint32_t)0x00000100)
#define  FMC_BWTR1_DATAST_1                          ((uint32_t)0x00000200)
#define  FMC_BWTR1_DATAST_2                          ((uint32_t)0x00000400)
#define  FMC_BWTR1_DATAST_3                          ((uint32_t)0x00000800)
#define  FMC_BWTR1_DATAST_4                          ((uint32_t)0x00001000)
#define  FMC_BWTR1_DATAST_5                          ((uint32_t)0x00002000)
#define  FMC_BWTR1_DATAST_6                          ((uint32_t)0x00004000)
#define  FMC_BWTR1_DATAST_7                          ((uint32_t)0x00008000)

#define  FMC_BWTR1_BUSTURN                           ((uint32_t)0x000F0000)
#define  FMC_BWTR1_BUSTURN_0                         ((uint32_t)0x00010000)
#define  FMC_BWTR1_BUSTURN_1                         ((uint32_t)0x00020000)
#define  FMC_BWTR1_BUSTURN_2                         ((uint32_t)0x00040000)
#define  FMC_BWTR1_BUSTURN_3                         ((uint32_t)0x00080000)

#define  FMC_BWTR1_ACCMOD                            ((uint32_t)0x30000000)
#define  FMC_BWTR1_ACCMOD_0                          ((uint32_t)0x10000000)
#define  FMC_BWTR1_ACCMOD_1                          ((uint32_t)0x20000000)

/*******************  Bit definition for FMC_BWTR2 register  *******************/
#define  FMC_BWTR2_ADDSET                            ((uint32_t)0x0000000F)
#define  FMC_BWTR2_ADDSET_0                          ((uint32_t)0x00000001)
#define  FMC_BWTR2_ADDSET_1                          ((uint32_t)0x00000002)
#define  FMC_BWTR2_ADDSET_2                          ((uint32_t)0x00000004)
#define  FMC_BWTR2_ADDSET_3                          ((uint32_t)0x00000008)

#define  FMC_BWTR2_ADDHLD                            ((uint32_t)0x000000F0)
#define  FMC_BWTR2_ADDHLD_0                          ((uint32_t)0x00000010)
#define  FMC_BWTR2_ADDHLD_1                          ((uint32_t)0x00000020)
#define  FMC_BWTR2_ADDHLD_2                          ((uint32_t)0x00000040)
#define  FMC_BWTR2_ADDHLD_3                          ((uint32_t)0x00000080)

#define  FMC_BWTR2_DATAST                            ((uint32_t)0x0000FF00)
#define  FMC_BWTR2_DATAST_0                          ((uint32_t)0x00000100)
#define  FMC_BWTR2_DATAST_1                          ((uint32_t)0x00000200)
#define  FMC_BWTR2_DATAST_2                          ((uint32_t)0x00000400)
#define  FMC_BWTR2_DATAST_3                          ((uint32_t)0x00000800)
#define  FMC_BWTR2_DATAST_4                          ((uint32_t)0x00001000)
#define  FMC_BWTR2_DATAST_5                          ((uint32_t)0x00002000)
#define  FMC_BWTR2_DATAST_6                          ((uint32_t)0x00004000)
#define  FMC_BWTR2_DATAST_7                          ((uint32_t)0x00008000)

#define  FMC_BWTR2_BUSTURN                           ((uint32_t)0x000F0000)
#define  FMC_BWTR2_BUSTURN_0                         ((uint32_t)0x00010000)
#define  FMC_BWTR2_BUSTURN_1                         ((uint32_t)0x00020000)
#define  FMC_BWTR2_BUSTURN_2                         ((uint32_t)0x00040000)
#define  FMC_BWTR2_BUSTURN_3                         ((uint32_t)0x00080000)

#define  FMC_BWTR2_ACCMOD                            ((uint32_t)0x30000000)
#define  FMC_BWTR2_ACCMOD_0                          ((uint32_t)0x10000000)
#define  FMC_BWTR2_ACCMOD_1                          ((uint32_t)0x20000000)

/*******************  Bit definition for FMC_BWTR3 register  *******************/
#define  FMC_BWTR3_ADDSET                            ((uint32_t)0x0000000F)
#define  FMC_BWTR3_ADDSET_0                          ((uint32_t)0x00000001)
#define  FMC_BWTR3_ADDSET_1                          ((uint32_t)0x00000002)
#define  FMC_BWTR3_ADDSET_2                          ((uint32_t)0x00000004)
#define  FMC_BWTR3_ADDSET_3                          ((uint32_t)0x00000008)

#define  FMC_BWTR3_ADDHLD                            ((uint32_t)0x000000F0)
#define  FMC_BWTR3_ADDHLD_0                          ((uint32_t)0x00000010)
#define  FMC_BWTR3_ADDHLD_1                          ((uint32_t)0x00000020)
#define  FMC_BWTR3_ADDHLD_2                          ((uint32_t)0x00000040)
#define  FMC_BWTR3_ADDHLD_3                          ((uint32_t)0x00000080)

#define  FMC_BWTR3_DATAST                            ((uint32_t)0x0000FF00)
#define  FMC_BWTR3_DATAST_0                          ((uint32_t)0x00000100)
#define  FMC_BWTR3_DATAST_1                          ((uint32_t)0x00000200)
#define  FMC_BWTR3_DATAST_2                          ((uint32_t)0x00000400)
#define  FMC_BWTR3_DATAST_3                          ((uint32_t)0x00000800)
#define  FMC_BWTR3_DATAST_4                          ((uint32_t)0x00001000)
#define  FMC_BWTR3_DATAST_5                          ((uint32_t)0x00002000)
#define  FMC_BWTR3_DATAST_6                          ((uint32_t)0x00004000)
#define  FMC_BWTR3_DATAST_7                          ((uint32_t)0x00008000)

#define  FMC_BWTR3_BUSTURN                           ((uint32_t)0x000F0000)
#define  FMC_BWTR3_BUSTURN_0                         ((uint32_t)0x00010000)
#define  FMC_BWTR3_BUSTURN_1                         ((uint32_t)0x00020000)
#define  FMC_BWTR3_BUSTURN_2                         ((uint32_t)0x00040000)
#define  FMC_BWTR3_BUSTURN_3                         ((uint32_t)0x00080000)

#define  FMC_BWTR3_ACCMOD                            ((uint32_t)0x30000000)
#define  FMC_BWTR3_ACCMOD_0                          ((uint32_t)0x10000000)
#define  FMC_BWTR3_ACCMOD_1                          ((uint32_t)0x20000000)

/*******************  Bit definition for FMC_BWTR4 register  *******************/
#define  FMC_BWTR4_ADDSET                            ((uint32_t)0x0000000F)
#define  FMC_BWTR4_ADDSET_0                          ((uint32_t)0x00000001)
#define  FMC_BWTR4_ADDSET_1                          ((uint32_t)0x00000002)
#define  FMC_BWTR4_ADDSET_2                          ((uint32_t)0x00000004)
#define  FMC_BWTR4_ADDSET_3                          ((uint32_t)0x00000008)

#define  FMC_BWTR4_ADDHLD                            ((uint32_t)0x000000F0)
#define  FMC_BWTR4_ADDHLD_0                          ((uint32_t)0x00000010)
#define  FMC_BWTR4_ADDHLD_1                          ((uint32_t)0x00000020)
#define  FMC_BWTR4_ADDHLD_2                          ((uint32_t)0x00000040)
#define  FMC_BWTR4_ADDHLD_3                          ((uint32_t)0x00000080)

#define  FMC_BWTR4_DATAST                            ((uint32_t)0x0000FF00)
#define  FMC_BWTR4_DATAST_0                          ((uint32_t)0x00000100)
#define  FMC_BWTR4_DATAST_1                          ((uint32_t)0x00000200)
#define  FMC_BWTR4_DATAST_2                          ((uint32_t)0x00000400)
#define  FMC_BWTR4_DATAST_3                          ((uint32_t)0x00000800)
#define  FMC_BWTR4_DATAST_4                          ((uint32_t)0x00001000)
#define  FMC_BWTR4_DATAST_5                          ((uint32_t)0x00002000)
#define  FMC_BWTR4_DATAST_6                          ((uint32_t)0x00004000)
#define  FMC_BWTR4_DATAST_7                          ((uint32_t)0x00008000)

#define  FMC_BWTR4_BUSTURN                           ((uint32_t)0x000F0000)
#define  FMC_BWTR4_BUSTURN_0                         ((uint32_t)0x00010000)
#define  FMC_BWTR4_BUSTURN_1                         ((uint32_t)0x00020000)
#define  FMC_BWTR4_BUSTURN_2                         ((uint32_t)0x00040000)
#define  FMC_BWTR4_BUSTURN_3                         ((uint32_t)0x00080000)

#define  FMC_BWTR4_ACCMOD                            ((uint32_t)0x30000000)
#define  FMC_BWTR4_ACCMOD_0                          ((uint32_t)0x10000000)
#define  FMC_BWTR4_ACCMOD_1                          ((uint32_t)0x20000000)

/*******************  Bit definition for FMC_PCR register  *******************/
#define  FMC_PCR_PWAITEN                             ((uint32_t)0x00000020)
#define  FMC_PCR_PBKEN                               ((uint32_t)0x00000040)
#define  FMC_PCR_PTYP                                ((uint32_t)0x00000080)
#define  FMC_PCR_PWID                                ((uint32_t)0x00000300)
#define  FMC_PCR_PWID_0                              ((uint32_t)0x00000100)
#define  FMC_PCR_PWID_1                              ((uint32_t)0x00000200)

#define  FMC_PCR_ECCEN                               ((uint32_t)0x00000400)

#define  FMC_PCR_TCLR                                ((uint32_t)0x00001E00)
#define  FMC_PCR_TCLR_0                              ((uint32_t)0x00000200)
#define  FMC_PCR_TCLR_1                              ((uint32_t)0x00000400)
#define  FMC_PCR_TCLR_2                              ((uint32_t)0x00000800)
#define  FMC_PCR_TCLR_3                              ((uint32_t)0x00001000)

#define  FMC_PCR_TAR                                 ((uint32_t)0x0001E000)
#define  FMC_PCR_TAR_0                               ((uint32_t)0x00002000)
#define  FMC_PCR_TAR_1                               ((uint32_t)0x00004000)
#define  FMC_PCR_TAR_2                               ((uint32_t)0x00008000)
#define  FMC_PCR_TAR_3                               ((uint32_t)0x00010000)

#define  FMC_PCR_ECCPS                               ((uint32_t)0x000E0000)
#define  FMC_PCR_ECCPS_0                             ((uint32_t)0x00020000)
#define  FMC_PCR_ECCPS_1                             ((uint32_t)0x00040000)
#define  FMC_PCR_ECCPS_2                             ((uint32_t)0x00080000)

/*******************  Bit definition for FMC_SR register  *******************/
#define  FMC_SR_IRS                                  ((uint32_t)0x00000001)
#define  FMC_SR_ILS                                  ((uint32_t)0x00000002)
#define  FMC_SR_IFS                                  ((uint32_t)0x00000004)
#define  FMC_SR_IREN                                 ((uint32_t)0x00000008)
#define  FMC_SR_ILEN                                 ((uint32_t)0x00000010)
#define  FMC_SR_IFEN                                 ((uint32_t)0x00000020)
#define  FMC_SR_FEMPT                                ((uint32_t)0x00000040)

/*******************  Bit definition for FMC_PMEM register  *******************/
#define  FMC_PMEM_MEMSET                             ((uint32_t)0x000000FF)
#define  FMC_PMEM_MEMWAIT                            ((uint32_t)0x0000FF00)
#define  FMC_PMEM_MEMHOLD                            ((uint32_t)0x00FF0000)
#define  FMC_PMEM_MEMHIZ                             ((uint32_t)0xFF000000)

/*******************  Bit definition for FMC_PATT register  *******************/
#define  FMC_PATT_ATTSET                             ((uint32_t)0x000000FF)
#define  FMC_PATT_ATTWAIT                            ((uint32_t)0x0000FF00)
#define  FMC_PATT_ATTHOLD                            ((uint32_t)0x00FF0000)
#define  FMC_PATT_ATTHIZ                             ((uint32_t)0xFF000000)

/*******************  Bit definition for FMC_ECCR register  *******************/
#define  FMC_ECCR_ECC                                ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for FMC_SDCR1 register  *******************/
#define  FMC_SDCR1_NC                                ((uint32_t)0x00000003)
#define  FMC_SDCR1_NC_0                              ((uint32_t)0x00000001)
#define  FMC_SDCR1_NC_1                              ((uint32_t)0x00000002)

#define  FMC_SDCR1_NR                                ((uint32_t)0x0000000C)
#define  FMC_SDCR1_NR_0                              ((uint32_t)0x00000004)
#define  FMC_SDCR1_NR_1                              ((uint32_t)0x00000008)

#define  FMC_SDCR1_MWID                              ((uint32_t)0x00000030)
#define  FMC_SDCR1_MWID_0                            ((uint32_t)0x00000010)
#define  FMC_SDCR1_MWID_1                            ((uint32_t)0x00000020)

#define  FMC_SDCR1_NB                                ((uint32_t)0x00000040)
#define  FMC_SDCR1_CAS                               ((uint32_t)0x00000180)
#define  FMC_SDCR1_CAS_0                             ((uint32_t)0x00000080)
#define  FMC_SDCR1_CAS_1                             ((uint32_t)0x00000100)

#define  FMC_SDCR1_WP                                ((uint32_t)0x00000200)
#define  FMC_SDCR1_SDCLK                             ((uint32_t)0x00000C00)
#define  FMC_SDCR1_SDCLK_0                           ((uint32_t)0x00000400)
#define  FMC_SDCR1_SDCLK_1                           ((uint32_t)0x00000800)

#define  FMC_SDCR1_RBURST                            ((uint32_t)0x00001000)

#define  FMC_SDCR1_RPIPE                             ((uint32_t)0x00006000)
#define  FMC_SDCR1_RPIPE_0                           ((uint32_t)0x00002000)
#define  FMC_SDCR1_RPIPE_1                           ((uint32_t)0x00004000)

/*******************  Bit definition for FMC_SDCR2 register  *******************/
#define  FMC_SDCR2_NC                                ((uint32_t)0x00000003)
#define  FMC_SDCR2_NC_0                              ((uint32_t)0x00000001)
#define  FMC_SDCR2_NC_1                              ((uint32_t)0x00000002)

#define  FMC_SDCR2_NR                                ((uint32_t)0x0000000C)
#define  FMC_SDCR2_NR_0                              ((uint32_t)0x00000004)
#define  FMC_SDCR2_NR_1                              ((uint32_t)0x00000008)

#define  FMC_SDCR2_MWID                              ((uint32_t)0x00000030)
#define  FMC_SDCR2_MWID_0                            ((uint32_t)0x00000010)
#define  FMC_SDCR2_MWID_1                            ((uint32_t)0x00000020)

#define  FMC_SDCR2_NB                                ((uint32_t)0x00000040)
#define  FMC_SDCR2_CAS                               ((uint32_t)0x00000180)
#define  FMC_SDCR2_CAS_0                             ((uint32_t)0x00000080)
#define  FMC_SDCR2_CAS_1                             ((uint32_t)0x00000100)

#define  FMC_SDCR2_WP                                ((uint32_t)0x00000200)
#define  FMC_SDCR2_SDCLK                             ((uint32_t)0x00000C00)
#define  FMC_SDCR2_SDCLK_0                           ((uint32_t)0x00000400)
#define  FMC_SDCR2_SDCLK_1                           ((uint32_t)0x00000800)

#define  FMC_SDCR2_RBURST                            ((uint32_t)0x00001000)

#define  FMC_SDCR2_RPIPE                             ((uint32_t)0x00006000)
#define  FMC_SDCR2_RPIPE_0                           ((uint32_t)0x00002000)
#define  FMC_SDCR2_RPIPE_1                           ((uint32_t)0x00004000)

/*******************  Bit definition for FMC_SDTR1 register  *******************/
#define  FMC_SDTR1_TMRD                              ((uint32_t)0x0000000F)
#define  FMC_SDTR1_TMRD_0                            ((uint32_t)0x00000001)
#define  FMC_SDTR1_TMRD_1                            ((uint32_t)0x00000002)
#define  FMC_SDTR1_TMRD_2                            ((uint32_t)0x00000004)
#define  FMC_SDTR1_TMRD_3                            ((uint32_t)0x00000008)

#define  FMC_SDTR1_TXSR                              ((uint32_t)0x000000F0)
#define  FMC_SDTR1_TXSR_0                            ((uint32_t)0x00000010)
#define  FMC_SDTR1_TXSR_2                            ((uint32_t)0x00000020)
#define  FMC_SDTR1_TXSR_4                            ((uint32_t)0x00000040)
#define  FMC_SDTR1_TXSR_8                            ((uint32_t)0x00000080)

#define  FMC_SDTR1_TRAS                              ((uint32_t)0x00000F00)
#define  FMC_SDTR1_TRAS_0                            ((uint32_t)0x00000100)
#define  FMC_SDTR1_TRAS_1                            ((uint32_t)0x00000200)
#define  FMC_SDTR1_TRAS_2                            ((uint32_t)0x00000400)
#define  FMC_SDTR1_TRAS_3                            ((uint32_t)0x00000800)

#define  FMC_SDTR1_TRC                               ((uint32_t)0x0000F000)
#define  FMC_SDTR1_TRC_0                             ((uint32_t)0x00001000)
#define  FMC_SDTR1_TRC_1                             ((uint32_t)0x00002000)
#define  FMC_SDTR1_TRC_2                             ((uint32_t)0x00004000)
#define  FMC_SDTR1_TRC_3                             ((uint32_t)0x00008000)

#define  FMC_SDTR1_TWR                               ((uint32_t)0x000F0000)
#define  FMC_SDTR1_TWR_0                             ((uint32_t)0x00010000)
#define  FMC_SDTR1_TWR_1                             ((uint32_t)0x00020000)
#define  FMC_SDTR1_TWR_2                             ((uint32_t)0x00040000)
#define  FMC_SDTR1_TWR_3                             ((uint32_t)0x00080000)

#define  FMC_SDTR1_TRP                               ((uint32_t)0x00F00000)
#define  FMC_SDTR1_TRP_0                             ((uint32_t)0x00100000)
#define  FMC_SDTR1_TRP_2                             ((uint32_t)0x00200000)
#define  FMC_SDTR1_TRP_3                             ((uint32_t)0x00400000)
#define  FMC_SDTR1_TRP_4                             ((uint32_t)0x00800000)

#define  FMC_SDTR1_TRCD                              ((uint32_t)0x0F000000)
#define  FMC_SDTR1_TRCD_0                            ((uint32_t)0x01000000)
#define  FMC_SDTR1_TRCD_1                            ((uint32_t)0x02000000)
#define  FMC_SDTR1_TRCD_2                            ((uint32_t)0x04000000)
#define  FMC_SDTR1_TRCD_3                            ((uint32_t)0x08000000)

/*******************  Bit definition for FMC_SDTR2 register  *******************/
#define  FMC_SDTR2_TMRD                              ((uint32_t)0x0000000F)
#define  FMC_SDTR2_TMRD_0                            ((uint32_t)0x00000001)
#define  FMC_SDTR2_TMRD_1                            ((uint32_t)0x00000002)
#define  FMC_SDTR2_TMRD_2                            ((uint32_t)0x00000004)
#define  FMC_SDTR2_TMRD_3                            ((uint32_t)0x00000008)

#define  FMC_SDTR2_TXSR                              ((uint32_t)0x000000F0)
#define  FMC_SDTR2_TXSR_0                            ((uint32_t)0x00000010)
#define  FMC_SDTR2_TXSR_2                            ((uint32_t)0x00000020)
#define  FMC_SDTR2_TXSR_4                            ((uint32_t)0x00000040)
#define  FMC_SDTR2_TXSR_8                            ((uint32_t)0x00000080)

#define  FMC_SDTR2_TRAS                              ((uint32_t)0x00000F00)
#define  FMC_SDTR2_TRAS_0                            ((uint32_t)0x00000100)
#define  FMC_SDTR2_TRAS_1                            ((uint32_t)0x00000200)
#define  FMC_SDTR2_TRAS_2                            ((uint32_t)0x00000400)
#define  FMC_SDTR2_TRAS_3                            ((uint32_t)0x00000800)

#define  FMC_SDTR2_TRC                               ((uint32_t)0x0000F000)
#define  FMC_SDTR2_TRC_0                             ((uint32_t)0x00001000)
#define  FMC_SDTR2_TRC_1                             ((uint32_t)0x00002000)
#define  FMC_SDTR2_TRC_2                             ((uint32_t)0x00004000)
#define  FMC_SDTR2_TRC_3                             ((uint32_t)0x00008000)

#define  FMC_SDTR2_TWR                               ((uint32_t)0x000F0000)
#define  FMC_SDTR2_TWR_0                             ((uint32_t)0x00010000)
#define  FMC_SDTR2_TWR_1                             ((uint32_t)0x00020000)
#define  FMC_SDTR2_TWR_2                             ((uint32_t)0x00040000)
#define  FMC_SDTR2_TWR_3                             ((uint32_t)0x00080000)

#define  FMC_SDTR2_TRP                               ((uint32_t)0x00F00000)
#define  FMC_SDTR2_TRP_0                             ((uint32_t)0x00100000)
#define  FMC_SDTR2_TRP_2                             ((uint32_t)0x00200000)
#define  FMC_SDTR2_TRP_3                             ((uint32_t)0x00400000)
#define  FMC_SDTR2_TRP_4                             ((uint32_t)0x00800000)

#define  FMC_SDTR2_TRCD                              ((uint32_t)0x0F000000)
#define  FMC_SDTR2_TRCD_0                            ((uint32_t)0x01000000)
#define  FMC_SDTR2_TRCD_1                            ((uint32_t)0x02000000)
#define  FMC_SDTR2_TRCD_2                            ((uint32_t)0x04000000)
#define  FMC_SDTR2_TRCD_3                            ((uint32_t)0x08000000)

/*******************  Bit definition for FMC_SDCMR register  *******************/
#define  FMC_SDCMR_MODE                              ((uint32_t)0x00000007)
#define  FMC_SDCMR_MODE_0                            ((uint32_t)0x00000001)
#define  FMC_SDCMR_MODE_1                            ((uint32_t)0x00000002)
#define  FMC_SDCMR_MODE_2                            ((uint32_t)0x00000004)

#define  FMC_SDCMR_CTB2                              ((uint32_t)0x00000008)
#define  FMC_SDCMR_CTB1                              ((uint32_t)0x00000010)

#define  FMC_SDCMR_NRFS                              ((uint32_t)0x000001E0)
#define  FMC_SDCMR_NRFS_0                            ((uint32_t)0x00000020)
#define  FMC_SDCMR_NRFS_1                            ((uint32_t)0x00000040)
#define  FMC_SDCMR_NRFS_2                            ((uint32_t)0x00000080)
#define  FMC_SDCMR_NRFS_3                            ((uint32_t)0x00000100)

#define  FMC_SDCMR_MRD                               ((uint32_t)0x003FFE00)

/*******************  Bit definition for FMC_SDRTR register  *******************/
#define  FMC_SDRTR_CRE                               ((uint32_t)0x00000001)
#define  FMC_SDRTR_COUNT                             ((uint32_t)0x00003FFE)
#define  FMC_SDRTR_REIE                              ((uint32_t)0x00004000)

/*******************  Bit definition for FMC_SDSR register  *******************/
#define  FMC_SDSR_RE                                 ((uint32_t)0x00000001)

#define  FMC_SDSR_MODES1                             ((uint32_t)0x00000006)
#define  FMC_SDSR_MODES1_0                           ((uint32_t)0x00000002)
#define  FMC_SDSR_MODES1_1                           ((uint32_t)0x00000004)

#define  FMC_SDSR_MODES2                             ((uint32_t)0x00000018)
#define  FMC_SDSR_MODES2_0                           ((uint32_t)0x00000008)
#define  FMC_SDSR_MODES2_1                           ((uint32_t)0x00000010)

#define  FMC_SDSR_BUSY                               ((uint32_t)0x00000020)

/*******************  Bit definition for FMC_MISC register  *******************/
#define  FMC_MISC_NRFS_CNT                           ((uint32_t)0x0000000F)
#define  FMC_MISC_NRFS_CNT_0                         ((uint32_t)0x00000001)
#define  FMC_MISC_NRFS_CNT_1                         ((uint32_t)0x00000002)
#define  FMC_MISC_NRFS_CNT_2                         ((uint32_t)0x00000004)
#define  FMC_MISC_NRFS_CNT_3                         ((uint32_t)0x00000008)

#define  FMC_MISC_Phase_Sel                          ((uint32_t)0x000000F0)
#define  FMC_MISC_Phase_Sel_0                        ((uint32_t)0x00000010)
#define  FMC_MISC_Phase_Sel_1                        ((uint32_t)0x00000020)
#define  FMC_MISC_Phase_Sel_2                        ((uint32_t)0x00000040)
#define  FMC_MISC_Phase_Sel_3                        ((uint32_t)0x00000080)

#define  FMC_MISC_Enhance_read_mode                  ((uint32_t)0x00001000)
#define  FMC_MISC_En_Bank1                           ((uint32_t)0x00010000)
#define  FMC_MISC_En_Bank2                           ((uint32_t)0x00020000)

/******************************************************************************/
/*                                 ECDC                                      */
/******************************************************************************/
/*******************  Bit definition for ECDC_CTRL register  *******************/
#define  ECDC_KEYEX_EN                               ((uint32_t)0x00000001)
#define  ECDC_NORMAL_EN                              ((uint32_t)0x00000002)
#define  ECDC_MODE_SEL                               ((uint32_t)0x00000008)

#define  ECDC_CLKDIV_MASK                            ((uint32_t)0x00000070)

#define  ECDC_WRSRAM_EN                              ((uint32_t)0x00000080)
#define  ECDC_ALGRM_MOD                              ((uint32_t)0x00000100)
#define  ECDC_CIPHER_MOD                             ((uint32_t)0x00000200)

#define  ECDC_KLEN_MASK                              ((uint32_t)0x00000C00)

#define  ECDC_DAT_MOD                                ((uint32_t)0x00002000)

#define  ECDC_IE_EKDONE                              ((uint32_t)0x00010000)
#define  ECDC_IE_SINGLE                              ((uint32_t)0x00020000)
#define  ECDC_IE_WRSRAM                              ((uint32_t)0x00040000)

#define  ECDC_CLOCK_SELECT                           ((uint32_t)0x01000000)
#define  ECDC_ECDC_SM4_CLOCK_EN                      ((uint32_t)0x02000000)

/*******************  Bit definition for ECDC_INT_FG register  *******************/
#define  ECDC_IF_EKDONE                              ((uint32_t)0x00010000)
#define  ECDC_IF_SINGLE                              ((uint32_t)0x00020000)
#define  ECDC_IF_WRSRAM                              ((uint32_t)0x00040000)

/*******************  Bit definition for ECDC_KEY register  *******************/
#define  ECDC_KEY_255T224                            ((uint32_t)0xFFFFFFFF)
#define  ECDC_KEY_223T192                            ((uint32_t)0xFFFFFFFF)
#define  ECDC_KEY_191T160                            ((uint32_t)0xFFFFFFFF)
#define  ECDC_KEY_159T128                            ((uint32_t)0xFFFFFFFF)
#define  ECDC_KEY_127T96                             ((uint32_t)0xFFFFFFFF)
#define  ECDC_KEY_95T64                              ((uint32_t)0xFFFFFFFF)
#define  ECDC_KEY_63T32                              ((uint32_t)0xFFFFFFFF)
#define  ECDC_KEY_31T0                               ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for ECDC_IV register  *******************/
#define  ECDC_IV_127T96                              ((uint32_t)0xFFFFFFFF)
#define  ECDC_IV_95T64                               ((uint32_t)0xFFFFFFFF)
#define  ECDC_IV_63T32                               ((uint32_t)0xFFFFFFFF)
#define  ECDC_IV_31T0                                ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for ECDC_SGSD register  *******************/
#define  ECDC_SGSD_127T96                            ((uint32_t)0xFFFFFFFF)
#define  ECDC_SGSD_95T64                             ((uint32_t)0xFFFFFFFF)
#define  ECDC_SGSD_63T32                             ((uint32_t)0xFFFFFFFF)
#define  ECDC_SGSD_31T0                              ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for ECDC_SGRT register  *******************/
#define  ECDC_SGRT_127T96                            ((uint32_t)0xFFFFFFFF)
#define  ECDC_SGRT_95T64                             ((uint32_t)0xFFFFFFFF)
#define  ECDC_SGRT_63T32                             ((uint32_t)0xFFFFFFFF)
#define  ECDC_SGRT_31T0                              ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for ECDC_SRC_ADDR register  *******************/
#define  ECDC_SRAM_SRC_ADDR                          ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for ECDC_DST_ADDR register  *******************/
#define  ECDC_SRAM_DST_ADDR                          ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for ECDC_SRAM_LEN register  *******************/
#define  ECDC_SRAM_LEN                               ((uint32_t)0xFFFFFFFF)

/******************************************************************************/
/*                                 DFSDM                                      */
/******************************************************************************/
/*******************  Bit definition for DFSDM_CFGR1 register  *******************/
#define  DFSDM_CFGR1_SITP                            ((uint32_t)0x00000003)
#define  DFSDM_CFGR1_SITP_0                          ((uint32_t)0x00000001)
#define  DFSDM_CFGR1_SITP_1                          ((uint32_t)0x00000002)

#define  DFSDM_CFGR1_SPICKSEL                        ((uint32_t)0x0000000C)
#define  DFSDM_CFGR1_SPICKSEL_0                      ((uint32_t)0x00000004)
#define  DFSDM_CFGR1_SPICKSEL_1                      ((uint32_t)0x00000008)

#define  DFSDM_CFGR1_SCDEN                           ((uint32_t)0x00000020)
#define  DFSDM_CFGR1_CKABEN                          ((uint32_t)0x00000040)
#define  DFSDM_CFGR1_CHEN                            ((uint32_t)0x00000080)
#define  DFSDM_CFGR1_CHINSEL                         ((uint32_t)0x00000100)

#define  DFSDM_CFGR1_DATMPX                          ((uint32_t)0x00003000)
#define  DFSDM_CFGR1_DATMPX_0                        ((uint32_t)0x00001000)
#define  DFSDM_CFGR1_DATMPX_1                        ((uint32_t)0x00002000)

#define  DFSDM_CFGR1_DATPACK                         ((uint32_t)0x0000C000)
#define  DFSDM_CFGR1_DATPACK_0                       ((uint32_t)0x00004000)
#define  DFSDM_CFGR1_DATPACK_1                       ((uint32_t)0x00008000)

#define  DFSDM_CFGR1_CKOUTDIV                        ((uint32_t)0x00FF0000)

#define  DFSDM_CFGR1_CKOUTSRC                        ((uint32_t)0x40000000)
#define  DFSDM_CFGR1_DFSDMEN                         ((uint32_t)0x80000000)

/*******************  Bit definition for DFSDM_CFGR2 register  *******************/
#define  DFSDM_CFGR2_DTRBS                           ((uint32_t)0x000000F8)
#define  DFSDM_CFGR2_OFFSET                          ((uint32_t)0xFFFFFF00)

/*******************  Bit definition for DFSDM_AWSCDR register  *******************/
#define  DFSDM_AWSCDR_SCDT                           ((uint32_t)0x000000FF)

#define  DFSDM_AWSCDR_BKSCD                          ((uint32_t)0x00003000)
#define  DFSDM_AWSCDR_BKSCD_0                        ((uint32_t)0x00001000)
#define  DFSDM_AWSCDR_BKSCD_1                        ((uint32_t)0x00002000)

#define  DFSDM_AWSCDR_AWFOSR                         ((uint32_t)0x001F0000)
#define  DFSDM_AWSCDR_AWFOSR_0                       ((uint32_t)0x00010000)
#define  DFSDM_AWSCDR_AWFOSR_1                       ((uint32_t)0x00020000)
#define  DFSDM_AWSCDR_AWFOSR_2                       ((uint32_t)0x00040000)
#define  DFSDM_AWSCDR_AWFOSR_3                       ((uint32_t)0x00080000)
#define  DFSDM_AWSCDR_AWFOSR_4                       ((uint32_t)0x00100000)

#define  DFSDM_AWSCDR_AWFORD                         ((uint32_t)0x00C00000)
#define  DFSDM_AWSCDR_AWFORD_0                       ((uint32_t)0x00400000)
#define  DFSDM_AWSCDR_AWFORD_1                       ((uint32_t)0x00800000)

/*******************  Bit definition for DFSDM_WDATR register  *******************/
#define  DFSDM_WDATR                                 ((uint16_t)0xFFFF)

/*******************  Bit definition for DFSDM_DATINR register  *******************/
#define  DFSDM_DATINR_INDAT0                         ((uint32_t)0x0000FFFF)
#define  DFSDM_DATINR_INDAT1                         ((uint32_t)0xFFFF0000)

/*******************  Bit definition for DFSDM_FLTCR1 register  *******************/
#define  DFSDM_FLTCR1_DFEN                           ((uint32_t)0x00000001)
#define  DFSDM_FLTCR1_JSWSTART                       ((uint32_t)0x00000002)
#define  DFSDM_FLTCR1_JSYNC                          ((uint32_t)0x00000008)
#define  DFSDM_FLTCR1_JSCAN                          ((uint32_t)0x00000010)
#define  DFSDM_FLTCR1_JDMAEN                         ((uint32_t)0x00000020)

#define  DFSDM_FLTCR1_JEXTSEL                        ((uint32_t)0x00000F00)
#define  DFSDM_FLTCR1_JEXTSEL_0                      ((uint32_t)0x00000100)
#define  DFSDM_FLTCR1_JEXTSEL_1                      ((uint32_t)0x00000200)
#define  DFSDM_FLTCR1_JEXTSEL_2                      ((uint32_t)0x00000400)
#define  DFSDM_FLTCR1_JEXTSEL_3                      ((uint32_t)0x00000800)

#define  DFSDM_FLTCR1_JEXTEN                         ((uint32_t)0x00006000)
#define  DFSDM_FLTCR1_JEXTEN_0                       ((uint32_t)0x00002000)
#define  DFSDM_FLTCR1_JEXTEN_1                       ((uint32_t)0x00004000)

#define  DFSDM_FLTCR1_RSWSTART                       ((uint32_t)0x00020000)
#define  DFSDM_FLTCR1_RCONT                          ((uint32_t)0x00040000)
#define  DFSDM_FLTCR1_RSYNC                          ((uint32_t)0x00080000)

#define  DFSDM_FLTCR1_RDMAEN                         ((uint32_t)0x00200000)

#define  DFSDM_FLTCR1_RCH                            ((uint32_t)0x01000000)

#define  DFSDM_FLTCR1_FAST                           ((uint32_t)0x20000000)
#define  DFSDM_FLTCR1_AWFSEL                         ((uint32_t)0x40000000)

/*******************  Bit definition for DFSDM_FLTCR2 register  *******************/
#define  DFSDM_FLTCR2_JEOCIE                         ((uint32_t)0x00000001)
#define  DFSDM_FLTCR2_REOCIE                         ((uint32_t)0x00000002)
#define  DFSDM_FLTCR2_JOVRIE                         ((uint32_t)0x00000004)
#define  DFSDM_FLTCR2_ROVRIE                         ((uint32_t)0x00000008)
#define  DFSDM_FLTCR2_AWDIE                          ((uint32_t)0x00000010)
#define  DFSDM_FLTCR2_SCDIE                          ((uint32_t)0x00000020)
#define  DFSDM_FLTCR2_CKABIE                         ((uint32_t)0x00000040)

#define  DFSDM_FLTCR2_EXCH                           ((uint32_t)0x00000300)
#define  DFSDM_FLTCR2_EXCH_0                         ((uint32_t)0x00000100)
#define  DFSDM_FLTCR2_EXCH_1                         ((uint32_t)0x00000200)

#define  DFSDM_FLTCR2_AWDCH                          ((uint32_t)0x00030000)
#define  DFSDM_FLTCR2_AWDCH_0                        ((uint32_t)0x00010000)
#define  DFSDM_FLTCR2_AWDCH_1                        ((uint32_t)0x00020000)

/*******************  Bit definition for DFSDM_FLTISR register  *******************/
#define  DFSDM_FLTISR_JEOCF                          ((uint32_t)0x00000001)
#define  DFSDM_FLTISR_REOCF                          ((uint32_t)0x00000002)
#define  DFSDM_FLTISR_JOVRF                          ((uint32_t)0x00000004)
#define  DFSDM_FLTISR_ROVRF                          ((uint32_t)0x00000008)
#define  DFSDM_FLTISR_AWDF                           ((uint32_t)0x00000010)

#define  DFSDM_FLTISR_JCIP                           ((uint32_t)0x00002000)
#define  DFSDM_FLTISR_RCIP                           ((uint32_t)0x00004000)

#define  DFSDM_FLTISR_CKABF                          ((uint32_t)0x00030000)
#define  DFSDM_FLTISR_CKABF_0                        ((uint32_t)0x00010000)
#define  DFSDM_FLTISR_CKABF_1                        ((uint32_t)0x00020000)

#define  DFSDM_FLTISR_SCDF                           ((uint32_t)0x03000000)
#define  DFSDM_FLTISR_SCDF_0                         ((uint32_t)0x01000000)
#define  DFSDM_FLTISR_SCDF_1                         ((uint32_t)0x02000000)

/*******************  Bit definition for DFSDM_FLTICR register  *******************/
#define  DFSDM_FLTICR_CLRJOVRF                       ((uint32_t)0x00000004)
#define  DFSDM_FLTICR_CLRROVRF                       ((uint32_t)0x00000008)

#define  DFSDM_FLTICR_CLRCKABF                       ((uint32_t)0x00030000)
#define  DFSDM_FLTICR_CLRCKABF_0                     ((uint32_t)0x00010000)
#define  DFSDM_FLTICR_CLRCKABF_1                     ((uint32_t)0x00020000)

#define  DFSDM_FLTICR_CLRSCDF                        ((uint32_t)0x03000000)
#define  DFSDM_FLTICR_CLRSCDF_0                      ((uint32_t)0x01000000)
#define  DFSDM_FLTICR_CLRSCDF_1                      ((uint32_t)0x02000000)

/*******************  Bit definition for DFSDM_FLTJCHGR register  *******************/
#define  DFSDM_FLTJCHGR_JCHG                         ((uint32_t)0x00000003)

/*******************  Bit definition for DFSDM_FLTFCR3 register  *******************/
#define  DFSDM_FLTFCR3_IOSR                          ((uint32_t)0x000000FF)
#define  DFSDM_FLTFCR3_IOSR_0                        ((uint32_t)0x00000001)
#define  DFSDM_FLTFCR3_IOSR_1                        ((uint32_t)0x00000002)
#define  DFSDM_FLTFCR3_IOSR_2                        ((uint32_t)0x00000004)
#define  DFSDM_FLTFCR3_IOSR_3                        ((uint32_t)0x00000008)
#define  DFSDM_FLTFCR3_IOSR_4                        ((uint32_t)0x00000010)
#define  DFSDM_FLTFCR3_IOSR_5                        ((uint32_t)0x00000020)
#define  DFSDM_FLTFCR3_IOSR_6                        ((uint32_t)0x00000040)
#define  DFSDM_FLTFCR3_IOSR_7                        ((uint32_t)0x00000080)


#define  DFSDM_FLTFCR3_FOSR                          ((uint32_t)0x03FF0000)
#define  DFSDM_FLTFCR3_FOSR_0                        ((uint32_t)0x00010000)
#define  DFSDM_FLTFCR3_FOSR_1                        ((uint32_t)0x00020000)
#define  DFSDM_FLTFCR3_FOSR_2                        ((uint32_t)0x00040000)
#define  DFSDM_FLTFCR3_FOSR_3                        ((uint32_t)0x00080000)
#define  DFSDM_FLTFCR3_FOSR_4                        ((uint32_t)0x00100000)
#define  DFSDM_FLTFCR3_FOSR_5                        ((uint32_t)0x00200000)
#define  DFSDM_FLTFCR3_FOSR_6                        ((uint32_t)0x00400000)
#define  DFSDM_FLTFCR3_FOSR_7                        ((uint32_t)0x00800000)
#define  DFSDM_FLTFCR3_FOSR_8                        ((uint32_t)0x01000000)
#define  DFSDM_FLTFCR3_FOSR_9                        ((uint32_t)0x03FF0000)

#define  DFSDM_FLTFCR3_FORD                          ((uint32_t)0xE0000000)
#define  DFSDM_FLTFCR3_FORD_0                        ((uint32_t)0x20000000)
#define  DFSDM_FLTFCR3_FORD_1                        ((uint32_t)0x40000000)
#define  DFSDM_FLTFCR3_FORD_2                        ((uint32_t)0x80000000)

/*******************  Bit definition for DFSDM_FLTJDATAR register  *******************/
#define  DFSDM_FLTJDATAR_JDATACH                     ((uint32_t)0x00000001)
#define  DFSDM_FLTJDATAR_JDATA                       ((uint32_t)0xFFFFFF00)

/*******************  Bit definition for DFSDM_FLTRDATAR register  *******************/
#define  DFSDM_FLTRDATAR_RDATACH                     ((uint32_t)0x00000001)
#define  DFSDM_FLTRDATAR_RPEND                       ((uint32_t)0x00000010)
#define  DFSDM_FLTRDATAR_RDATA                       ((uint32_t)0xFFFFFF00)

/*******************  Bit definition for DFSDM_FLTAWHTR register  *******************/
#define  DFSDM_FLTAWHTR_BKAWH                        ((uint32_t)0x0000000F)
#define  DFSDM_FLTAWHTR_BKAWH_0                      ((uint32_t)0x00000001)
#define  DFSDM_FLTAWHTR_BKAWH_1                      ((uint32_t)0x00000002)
#define  DFSDM_FLTAWHTR_BKAWH_2                      ((uint32_t)0x00000004)
#define  DFSDM_FLTAWHTR_BKAWH_3                      ((uint32_t)0x00000008)

#define  DFSDM_FLTAWHTR_AWHT                         ((uint32_t)0xFFFFFF00)

/*******************  Bit definition for DFSDM_FLTAWLTR register  *******************/
#define  DFSDM_FLTAWLTR_BKAWL                        ((uint32_t)0x0000000F)
#define  DFSDM_FLTAWLTR_BKAWL_0                      ((uint32_t)0x00000001)
#define  DFSDM_FLTAWLTR_BKAWL_1                      ((uint32_t)0x00000002)
#define  DFSDM_FLTAWLTR_BKAWL_2                      ((uint32_t)0x00000004)
#define  DFSDM_FLTAWLTR_BKAWL_3                      ((uint32_t)0x00000008)

#define  DFSDM_FLTAWLTR_AWLT                         ((uint32_t)0xFFFFFF00)

/*******************  Bit definition for DFSDM_FLTAWSR register  *******************/
#define  DFSDM_FLTAWSR_AWLTF                         ((uint32_t)0x00000003)
#define  DFSDM_FLTAWSR_AWHTF                         ((uint32_t)0x00000300)

/*******************  Bit definition for DFSDM_FLTAWCFR register  *******************/
#define  DFSDM_FLTAWCFR_CLRAWLTF                     ((uint32_t)0x00000003)
#define  DFSDM_FLTAWCFR_CLRAWLTF_0                   ((uint32_t)0x00000001)
#define  DFSDM_FLTAWCFR_CLRAWLTF_1                   ((uint32_t)0x00000002)

#define  DFSDM_FLTAWCFR_CLRAWHTF                     ((uint32_t)0x00000300)
#define  DFSDM_FLTAWCFR_CLRAWHTF_0                   ((uint32_t)0x00000100)
#define  DFSDM_FLTAWCFR_CLRAWHTF_1                   ((uint32_t)0x00000200)

/*******************  Bit definition for DFSDM_FLTEXMAX register  *******************/
#define  DFSDM_FLTEXMAX_EXMAXCH                      ((uint32_t)0x00000001)
#define  DFSDM_FLTEXMAX_EXMAX                        ((uint32_t)0xFFFFFF00)

/*******************  Bit definition for DFSDM_FLTEXMIN register  *******************/
#define  DFSDM_FLTEXMIN_EXMINCH                      ((uint32_t)0x00000001)
#define  DFSDM_FLTEXMIN_EXMIN                        ((uint32_t)0xFFFFFF00)

/*******************  Bit definition for DFSDM_FLTCNVTIMR register  *******************/
#define  DFSDM_FLTCNVTIMR_CNVCNT                     ((uint32_t)0xFFFFFF00)

/******************************************************************************/
/*                                 LTDC                                      */
/******************************************************************************/
/*******************  Bit definition for LTDC_SSCR register  *******************/
#define  LTDC_SSCR_VSH                               ((uint32_t)0x000007FF)
#define  LTDC_SSCR_HSW                               ((uint32_t)0x0FFF0000)

/*******************  Bit definition for LTDC_BPCR register  *******************/
#define  LTDC_BPCR_AVBP                              ((uint32_t)0x000007FF)
#define  LTDC_BPCR_AHBP                              ((uint32_t)0x0FFF0000)

/*******************  Bit definition for LTDC_AWCR register  *******************/
#define  LTDC_AWCR_AAH                               ((uint32_t)0x000007FF)
#define  LTDC_AWCR_AAW                               ((uint32_t)0x0FFF0000)

/*******************  Bit definition for LTDC_TWCR register  *******************/
#define  LTDC_TWCR_TOTALH                            ((uint32_t)0x000007FF)
#define  LTDC_TWCR_TOTALW                            ((uint32_t)0x0FFF0000)

/*******************  Bit definition for LTDC_GCR register  *******************/
#define  LTDC_GCR_LTDCEN                             ((uint32_t)0x00000001)

#define  LTDC_GCR_DBW                                ((uint32_t)0x00000070)
#define  LTDC_GCR_DBW_0                              ((uint32_t)0x00000010)
#define  LTDC_GCR_DBW_1                              ((uint32_t)0x00000020)
#define  LTDC_GCR_DBW_2                              ((uint32_t)0x00000040)

#define  LTDC_GCR_DGW                                ((uint32_t)0x00000700)
#define  LTDC_GCR_DGW_0                              ((uint32_t)0x00000100)
#define  LTDC_GCR_DGW_1                              ((uint32_t)0x00000200)
#define  LTDC_GCR_DGW_2                              ((uint32_t)0x00000400)

#define  LTDC_GCR_DRW                                ((uint32_t)0x00007000)
#define  LTDC_GCR_DRW_0                              ((uint32_t)0x00001000)
#define  LTDC_GCR_DRW_1                              ((uint32_t)0x00002000)
#define  LTDC_GCR_DRW_2                              ((uint32_t)0x00004000)

#define  LTDC_GCR_DEN                                ((uint32_t)0x00010000)

#define  LTDC_GCR_PCPOL                              ((uint32_t)0x10000000)
#define  LTDC_GCR_DEPOL                              ((uint32_t)0x20000000)
#define  LTDC_GCR_VSPOL                              ((uint32_t)0x40000000)
#define  LTDC_GCR_HSPOL                              ((uint32_t)0x80000000)

/*******************  Bit definition for LTDC_SRCR register  *******************/
#define  LTDC_SRCR_IMR                               ((uint32_t)0x00000001)
#define  LTDC_SRCR_VBR                               ((uint32_t)0x00000002)

/*******************  Bit definition for LTDC_BCCR register  *******************/
#define  LTDC_BCCR_BCBLUE                            ((uint32_t)0x000000FF)
#define  LTDC_BCCR_BCGREEN                           ((uint32_t)0x0000FF00)
#define  LTDC_BCCR_BCRED                             ((uint32_t)0x00FF0000)

/*******************  Bit definition for LTDC_IER register  *******************/
#define  LTDC_IER_LIE                                ((uint32_t)0x00000001)
#define  LTDC_IER_FUIE                               ((uint32_t)0x00000002)

#define  LTDC_IER_RRIE                               ((uint32_t)0x00000008)

/*******************  Bit definition for LTDC_ISR register  *******************/
#define  LTDC_ISR_LIF                                ((uint32_t)0x00000001)
#define  LTDC_ISR_FUIF                               ((uint32_t)0x00000002)

#define  LTDC_ISR_RRIF                               ((uint32_t)0x00000008)

/*******************  Bit definition for LTDC_ICR register  *******************/
#define  LTDC_ICR_CLIF                               ((uint32_t)0x00000001)
#define  LTDC_ICR_CFUIF                              ((uint32_t)0x00000002)

#define  LTDC_ICR_CRRIF                              ((uint32_t)0x00000008)

/*******************  Bit definition for LTDC_LIPCR register  *******************/
#define  LTDC_LIPCR_LIPOS                            ((uint32_t)0x000007FF)

/*******************  Bit definition for LTDC_CPSR register  *******************/
#define  LTDC_CPSR_CYPOS                             ((uint32_t)0x0000FFFF)
#define  LTDC_CPSR_CXPOS                             ((uint32_t)0xFFFF0000)

/*******************  Bit definition for LTDC_CDSR register  *******************/
#define  LTDC_CDSR_VDES                              ((uint32_t)0x00000001)
#define  LTDC_CDSR_HDES                              ((uint32_t)0x00000002)
#define  LTDC_CDSR_VSYNCS                            ((uint32_t)0x00000004)
#define  LTDC_CDSR_HSYNCS                            ((uint32_t)0x00000008)

/*******************  Bit definition for LTDC_CR register  *******************/
#define  LTDC_CR_LEN                                 ((uint32_t)0x00000001)
#define  LTDC_CR_COLKEN                              ((uint32_t)0x00000002)

#define  LTDC_CR_CLUTEN                              ((uint32_t)0x00000010)

/*******************  Bit definition for LTDC_WHPCR register  *******************/
#define  LTDC_WHPCR_WHSTPOS                          ((uint32_t)0x00000FFF)
#define  LTDC_WHPCR_WHSPPOS                          ((uint32_t)0x0FFF0000)

/*******************  Bit definition for LTDC_WVPCR register  *******************/
#define  LTDC_WVPCR_WVSTPOS                          ((uint32_t)0x000007FF)
#define  LTDC_WVPCR_WVSPPOS                          ((uint32_t)0x07FF0000)

/*******************  Bit definition for LTDC_CKCR register  *******************/
#define  LTDC_CKCR_CKBLUE                            ((uint32_t)0x000000FF)
#define  LTDC_CKCR_CKGREEN                           ((uint32_t)0x0000FF00)
#define  LTDC_CKCR_CKRED                             ((uint32_t)0x00FF0000)

/*******************  Bit definition for LTDC_PFCR register  *******************/
#define  LTDC_PFCR_PF                                ((uint32_t)0x00000007)
#define  LTDC_PFCR_PF_0                              ((uint32_t)0x00000001)
#define  LTDC_PFCR_PF_1                              ((uint32_t)0x00000002)
#define  LTDC_PFCR_PF_2                              ((uint32_t)0x00000004)

#define  LTDC_PFCR_PF_ARGB8888                       ((uint32_t)0x00000000)
#define  LTDC_PFCR_PF_RGB888                         ((uint32_t)0x00000001)
#define  LTDC_PFCR_PF_RGB565                         ((uint32_t)0x00000002)
#define  LTDC_PFCR_PF_ARGB1555                       ((uint32_t)0x00000003)
#define  LTDC_PFCR_PF_ARGB4444                       ((uint32_t)0x00000004)
#define  LTDC_PFCR_PF_L8                             ((uint32_t)0x00000005)
#define  LTDC_PFCR_PF_AL44                           ((uint32_t)0x00000006)
#define  LTDC_PFCR_PF_AL88                           ((uint32_t)0x00000007)

/*******************  Bit definition for LTDC_CACR register  *******************/
#define  LTDC_CACR_CONSTA                            ((uint32_t)0x000000FF)

/*******************  Bit definition for LTDC_DCCR register  *******************/
#define  LTDC_DCCR_DCBLUE                            ((uint32_t)0x000000FF)
#define  LTDC_DCCR_DCGREEN                           ((uint32_t)0x0000FF00)
#define  LTDC_DCCR_DCRED                             ((uint32_t)0x00FF0000)
#define  LTDC_DCCR_DCALPHA                           ((uint32_t)0xFF000000)

/*******************  Bit definition for LTDC_BFCR register  *******************/
#define  LTDC_BFCR_BF2                               ((uint32_t)0x00000007)
#define  LTDC_BFCR_BF2_0                             ((uint32_t)0x00000001)
#define  LTDC_BFCR_BF2_1                             ((uint32_t)0x00000002)
#define  LTDC_BFCR_BF2_2                             ((uint32_t)0x00000004)

#define  LTDC_BFCR_BF1                               ((uint32_t)0x00000700)
#define  LTDC_BFCR_BF1_0                             ((uint32_t)0x00000100)
#define  LTDC_BFCR_BF1_1                             ((uint32_t)0x00000200)
#define  LTDC_BFCR_BF1_2                             ((uint32_t)0x00000400)

/*******************  Bit definition for LTDC_CFBAR register  *******************/
#define  LTDC_CFBAR_CFBADD                           ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for LTDC_CFBLR register  *******************/
#define  LTDC_CFBLR_CFBLL                            ((uint32_t)0x00001FFF)
#define  LTDC_CFBLR_CFBP                             ((uint32_t)0x1FFF0000)

/*******************  Bit definition for LTDC_CFBLNR register  *******************/
#define  LTDC_CFBLNR_CFBLNBR                         ((uint32_t)0x000007FF)

/*******************  Bit definition for LTDC_CLUTWR register  *******************/
#define  LTDC_CLUTWR_BLUE                            ((uint32_t)0x000000FF)
#define  LTDC_CLUTWR_GREEN                           ((uint32_t)0x0000FF00)
#define  LTDC_CLUTWR_RED                             ((uint32_t)0x00FF0000)
#define  LTDC_CLUTWR_CLUTADD                         ((uint32_t)0xFF000000)

/******************************************************************************/
/*                                 GPHA                                      */
/******************************************************************************/
/*******************  Bit definition for GPHA_CTLR register  *******************/
#define  GPHA_CTLR_START                             ((uint32_t)0x00000001)
#define  GPHA_CTLR_SUSP                              ((uint32_t)0x00000002)
#define  GPHA_CTLR_ABORT                             ((uint32_t)0x00000004)

#define  GPHA_CTLR_TCIE                              ((uint32_t)0x00000200)
#define  GPHA_CTLR_TWIE                              ((uint32_t)0x00000400)
#define  GPHA_CTLR_CAEIE                             ((uint32_t)0x00000800)
#define  GPHA_CTLR_CTCIE                             ((uint32_t)0x00001000)
#define  GPHA_CTLR_CEIE                              ((uint32_t)0x00002000)

#define  GPHA_CTLR_MODE                              ((uint32_t)0x00070000)
#define  GPHA_CTLR_MODE_0                            ((uint32_t)0x00010000)
#define  GPHA_CTLR_MODE_1                            ((uint32_t)0x00020000)
#define  GPHA_CTLR_MODE_2                            ((uint32_t)0x00040000)

/*******************  Bit definition for GPHA_ISR register  *******************/
#define  GPHA_ISR_TCIF                               ((uint32_t)0x00000002)
#define  GPHA_ISR_TWIF                               ((uint32_t)0x00000004)
#define  GPHA_ISR_CAEIF                              ((uint32_t)0x00000008)
#define  GPHA_ISR_CTCIF                              ((uint32_t)0x00000010)
#define  GPHA_ISR_CEIF                               ((uint32_t)0x00000020)

/*******************  Bit definition for GPHA_IFCR register  *******************/
#define  GPHA_IFCR_CTCIF                             ((uint32_t)0x00000002)
#define  GPHA_IFCR_CTWIF                             ((uint32_t)0x00000004)
#define  GPHA_IFCR_CAECIF                            ((uint32_t)0x00000008)
#define  GPHA_IFCR_CCTCIF                            ((uint32_t)0x00000010)
#define  GPHA_IFCR_CCEIF                             ((uint32_t)0x00000020)

/*******************  Bit definition for GPHA_FGMAR register  *******************/
#define  GPHA_FGMAR_MA                               ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for GPHA_FGOR register  *******************/
#define  GPHA_FGOR_LO                                ((uint32_t)0x00003FFF)

/*******************  Bit definition for GPHA_BGMAR register  *******************/
#define  GPHA_BGMAR_MA                               ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for GPHA_BGOR register  *******************/
#define  GPHA_BGOR_LO                                ((uint32_t)0x00003FFF)

/*******************  Bit definition for GPHA_FGPFCCR register  *******************/
#define  GPHA_FGPFCCR_CM                             ((uint32_t)0x0000000F)
#define  GPHA_FGPFCCR_CM_0                           ((uint32_t)0x00000001)
#define  GPHA_FGPFCCR_CM_1                           ((uint32_t)0x00000002)
#define  GPHA_FGPFCCR_CM_2                           ((uint32_t)0x00000004)
#define  GPHA_FGPFCCR_CM_3                           ((uint32_t)0x00000008)

#define  GPHA_FGPFCCR_CM_ARGB8888                    ((uint32_t)0x00000000)
#define  GPHA_FGPFCCR_CM_RGB888                      ((uint32_t)0x00000001)
#define  GPHA_FGPFCCR_CM_RGB565                      ((uint32_t)0x00000002)
#define  GPHA_FGPFCCR_CM_ARGB1555                    ((uint32_t)0x00000003)
#define  GPHA_FGPFCCR_CM_ARGB4444                    ((uint32_t)0x00000004)
#define  GPHA_FGPFCCR_CM_L8                          ((uint32_t)0x00000005)
#define  GPHA_FGPFCCR_CM_AL44                        ((uint32_t)0x00000006)
#define  GPHA_FGPFCCR_CM_AL88                        ((uint32_t)0x00000007)
#define  GPHA_FGPFCCR_CM_L4                          ((uint32_t)0x00000008)
#define  GPHA_FGPFCCR_CM_A8                          ((uint32_t)0x00000009)
#define  GPHA_FGPFCCR_CM_A4                          ((uint32_t)0x0000000A)
#define  GPHA_FGPFCCR_CM_YcbCr                       ((uint32_t)0x0000000B)

#define  GPHA_FGPFCCR_CCM                            ((uint32_t)0x00000010)
#define  GPHA_FGPFCCR_START                          ((uint32_t)0x00000020)

#define  GPHA_FGPFCCR_CS                             ((uint32_t)0x0000FF00)

#define  GPHA_FGPFCCR_AM                             ((uint32_t)0x00030000)
#define  GPHA_FGPFCCR_AM_0                           ((uint32_t)0x00010000)
#define  GPHA_FGPFCCR_AM_1                           ((uint32_t)0x00020000)

#define  GPHA_FGPFCCR_CSS                            ((uint32_t)0x000C0000)
#define  GPHA_FGPFCCR_CSS_0                          ((uint32_t)0x00040000)
#define  GPHA_FGPFCCR_CSS_1                          ((uint32_t)0x00080000)

#define  GPHA_FGPFCCR_AI                             ((uint32_t)0x00100000)
#define  GPHA_FGPFCCR_RBS                            ((uint32_t)0x00200000)

#define  GPHA_FGPFCCR_ALPHA                          ((uint32_t)0xFF000000)

/*******************  Bit definition for GPHA_FGCOLR register  *******************/
#define  GPHA_FGCOLR_BLUE                            ((uint32_t)0x000000FF)
#define  GPHA_FGCOLR_GREEN                           ((uint32_t)0x0000FF00)
#define  GPHA_FGCOLR_RED                             ((uint32_t)0x00FF0000)

/*******************  Bit definition for GPHA_BGPFCCR register  *******************/
#define  GPHA_BGPFCCR_CM                             ((uint32_t)0x0000000F)
#define  GPHA_BGPFCCR_CM_0                           ((uint32_t)0x00000001)
#define  GPHA_BGPFCCR_CM_1                           ((uint32_t)0x00000002)
#define  GPHA_BGPFCCR_CM_2                           ((uint32_t)0x00000004)
#define  GPHA_BGPFCCR_CM_3                           ((uint32_t)0x00000008)

#define  GPHA_BGPFCCR_CM_ARGB8888                    ((uint32_t)0x00000000)
#define  GPHA_BGPFCCR_CM_RGB888                      ((uint32_t)0x00000001)
#define  GPHA_BGPFCCR_CM_RGB565                      ((uint32_t)0x00000002)
#define  GPHA_BGPFCCR_CM_ARGB1555                    ((uint32_t)0x00000003)
#define  GPHA_BGPFCCR_CM_ARGB4444                    ((uint32_t)0x00000004)
#define  GPHA_BGPFCCR_CM_L8                          ((uint32_t)0x00000005)
#define  GPHA_BGPFCCR_CM_AL44                        ((uint32_t)0x00000006)
#define  GPHA_BGPFCCR_CM_AL88                        ((uint32_t)0x00000007)
#define  GPHA_BGPFCCR_CM_L4                          ((uint32_t)0x00000008)
#define  GPHA_BGPFCCR_CM_A8                          ((uint32_t)0x00000009)
#define  GPHA_BGPFCCR_CM_A4                          ((uint32_t)0x0000000A)

#define  GPHA_BGPFCCR_CCM                            ((uint32_t)0x00000010)
#define  GPHA_BGPFCCR_START                          ((uint32_t)0x00000020)

#define  GPHA_BGPFCCR_CS                             ((uint32_t)0x0000FF00)

#define  GPHA_BGPFCCR_AM                             ((uint32_t)0x00030000)
#define  GPHA_BGPFCCR_AM_0                           ((uint32_t)0x00010000)
#define  GPHA_BGPFCCR_AM_1                           ((uint32_t)0x00020000)

#define  GPHA_BGPFCCR_AI                             ((uint32_t)0x00100000)
#define  GPHA_BGPFCCR_RBS                            ((uint32_t)0x00200000)

#define  GPHA_BGPFCCR_ALPHA                          ((uint32_t)0xFF000000)

/*******************  Bit definition for GPHA_BGCOLR register  *******************/
#define  GPHA_BGCOLR_BLUE                            ((uint32_t)0x000000FF)
#define  GPHA_BGCOLR_GREEN                           ((uint32_t)0x0000FF00)
#define  GPHA_BGCOLR_RED                             ((uint32_t)0x00FF0000)

/*******************  Bit definition for GPHA_FGCMAR register  *******************/
#define  GPHA_FGCMAR_MA                              ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for GPHA_BGCMAR register  *******************/
#define  GPHA_BGCMAR_MA                              ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for GPHA_OPFCCR register  *******************/
#define  GPHA_OPFCCR_CM                              ((uint32_t)0x00000007)
#define  GPHA_OPFCCR_CM_0                            ((uint32_t)0x00000001)
#define  GPHA_OPFCCR_CM_1                            ((uint32_t)0x00000002)
#define  GPHA_OPFCCR_CM_2                            ((uint32_t)0x00000004)

#define  GPHA_OPFCCR_CM_ARGB8888                     ((uint32_t)0x00000000)
#define  GPHA_OPFCCR_CM_RGB888                       ((uint32_t)0x00000001)
#define  GPHA_OPFCCR_CM_RGB565                       ((uint32_t)0x00000002)
#define  GPHA_OPFCCR_CM_ARGB1555                     ((uint32_t)0x00000003)
#define  GPHA_OPFCCR_CM_ARGB4444                     ((uint32_t)0x00000004)

#define  GPHA_OPFCCR_AI                              ((uint32_t)0x00100000)
#define  GPHA_OPFCCR_RBS                             ((uint32_t)0x00200000)

/*******************  Bit definition for GPHA_OCOLR register  *******************/
#define  GPHA_OCOLR_BLUE                             ((uint32_t)0x000000FF)
#define  GPHA_OCOLR_GREEN                            ((uint32_t)0x0000FF00)
#define  GPHA_OCOLR_RED                              ((uint32_t)0x00FF0000)
#define  GPHA_OCOLR_ALPHA                            ((uint32_t)0xFF000000)

/*******************  Bit definition for GPHA_OMAR register  *******************/
#define  GPHA_OMAR_MA                                ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for GPHA_OOR register  *******************/
#define  GPHA_OOR_LO                                 ((uint32_t)0x00003FFF)

/*******************  Bit definition for GPHA_NLR register  *******************/
#define  GPHA_NLR_NL                                 ((uint32_t)0x0000FFFF)
#define  GPHA_NLR_PL                                 ((uint32_t)0x3FFF0000)

/*******************  Bit definition for GPHA_LWR register  *******************/
#define  GPHA_LWR_LW                                 ((uint32_t)0x0000FFFF)

/*******************  Bit definition for GPHA_AMTCR register  *******************/
#define  GPHA_AMTCR_EN                               ((uint32_t)0x00000001)
#define  GPHA_AMTCR_DT                               ((uint32_t)0x0000FF00)

/*******************  Bit definition for GPHA_FGCWRS register  *******************/
#define  GPHA_FGCWRS_FG_CLUT_INDEX                   ((uint32_t)0x000000FF)
#define  GPHA_FGCWRS_FG_CLUT_EN                      ((uint32_t)0x00000100)

/*******************  Bit definition for GPHA_FGCDAT register  *******************/
#define  GPHA_FGCDAT_FG_CLUT_DATA                    ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for GPHA_BGCWRS register  *******************/
#define  GPHA_BGCWRS_BG_CLUT_INDEX                   ((uint32_t)0x000000FF)
#define  GPHA_BGCWRS_BG_CLUT_EN                      ((uint32_t)0x00000100)

/*******************  Bit definition for GPHA_BGCDAT register  *******************/
#define  GPHA_BGCDAT_BG_CLUT_DATA                    ((uint32_t)0xFFFFFFFF)

/******************************************************************************/
/*                                 QSPI                                      */
/******************************************************************************/
/*******************  Bit definition for QSPI_CR register  *******************/
#define  QSPI_CR_EN                                  ((uint32_t)0x00000001)
#define  QSPI_CR_ABORT                               ((uint32_t)0x00000002)
#define  QSPI_CR_DMAEN                               ((uint32_t)0x00000004)
#define  QSPI_CR_TCEN                                ((uint32_t)0x00000008)

#define  QSPI_CR_START                               ((uint32_t)0x00000020)
#define  QSPI_CR_DFM                                 ((uint32_t)0x00000040)
#define  QSPI_CR_FSEL                                ((uint32_t)0x00000080)

#define  QSPI_CR_FTHRES                              ((uint32_t)0x00001F00)
#define  QSPI_CR_FTHRES_0                            ((uint32_t)0x00000100)
#define  QSPI_CR_FTHRES_1                            ((uint32_t)0x00000200)
#define  QSPI_CR_FTHRES_2                            ((uint32_t)0x00000400)
#define  QSPI_CR_FTHRES_3                            ((uint32_t)0x00000800)
#define  QSPI_CR_FTHRES_4                            ((uint32_t)0x00001000)

#define  QSPI_CR_TEIE                                ((uint32_t)0x00010000)
#define  QSPI_CR_TCIE                                ((uint32_t)0x00020000)
#define  QSPI_CR_FTIE                                ((uint32_t)0x00040000)
#define  QSPI_CR_SMIE                                ((uint32_t)0x00080000)
#define  QSPI_CR_TOIE                                ((uint32_t)0x00100000)
#define  QSPI_CR_APMS                                ((uint32_t)0x00400000)
#define  QSPI_CR_PMM                                 ((uint32_t)0x00800000)

#define  QSPI_CR_PRESCALER                           ((uint32_t)0xFF000000)
#define  QSPI_CR_PRESCALER_0                         ((uint32_t)0x01000000)
#define  QSPI_CR_PRESCALER_1                         ((uint32_t)0x02000000)
#define  QSPI_CR_PRESCALER_2                         ((uint32_t)0x04000000)
#define  QSPI_CR_PRESCALER_3                         ((uint32_t)0x08000000)
#define  QSPI_CR_PRESCALER_4                         ((uint32_t)0x10000000)
#define  QSPI_CR_PRESCALER_5                         ((uint32_t)0x20000000)
#define  QSPI_CR_PRESCALER_6                         ((uint32_t)0x40000000)
#define  QSPI_CR_PRESCALER_7                         ((uint32_t)0x80000000)

/*******************  Bit definition for QSPI_DCR register  *******************/
#define  QSPI_DCR_CKMODE                             ((uint32_t)0x00000001)

#define  QSPI_DCR_CSHT                               ((uint32_t)0x00000700)
#define  QSPI_DCR_CSHT_0                             ((uint32_t)0x00000100)
#define  QSPI_DCR_CSHT_1                             ((uint32_t)0x00000200)
#define  QSPI_DCR_CSHT_2                             ((uint32_t)0x00000400)

#define  QSPI_DCR_FSIZE                              ((uint32_t)0x001F0000)
#define  QSPI_DCR_FSIZE_0                            ((uint32_t)0x00010000)
#define  QSPI_DCR_FSIZE_1                            ((uint32_t)0x00020000)
#define  QSPI_DCR_FSIZE_2                            ((uint32_t)0x00040000)
#define  QSPI_DCR_FSIZE_3                            ((uint32_t)0x00080000)
#define  QSPI_DCR_FSIZE_4                            ((uint32_t)0x00100000)

/*******************  Bit definition for QSPI_SR register  *******************/
#define  QSPI_SR_TEF                                 ((uint32_t)0x00000001)
#define  QSPI_SR_TCF                                 ((uint32_t)0x00000002)
#define  QSPI_SR_FTF                                 ((uint32_t)0x00000004)
#define  QSPI_SR_SMF                                 ((uint32_t)0x00000008)
#define  QSPI_SR_TOF                                 ((uint32_t)0x00000010)
#define  QSPI_SR_BUSY                                ((uint32_t)0x00000020)

#define  QSPI_SR_FLEVEL                              ((uint32_t)0x00003F00)
#define  QSPI_SR_FLEVEL_0                            ((uint32_t)0x00000100)
#define  QSPI_SR_FLEVEL_1                            ((uint32_t)0x00000200)
#define  QSPI_SR_FLEVEL_2                            ((uint32_t)0x00000400)
#define  QSPI_SR_FLEVEL_3                            ((uint32_t)0x00000800)
#define  QSPI_SR_FLEVEL_4                            ((uint32_t)0x00001000)
#define  QSPI_SR_FLEVEL_5                            ((uint32_t)0x00002000)

/*******************  Bit definition for QSPI_FCR register  *******************/
#define  QSPI_FCR_CTEF                               ((uint32_t)0x00000001)
#define  QSPI_FCR_CTCF                               ((uint32_t)0x00000002)
#define  QSPI_FCR_CSMF                               ((uint32_t)0x00000008)
#define  QSPI_FCR_CTOF                               ((uint32_t)0x00000010)

/*******************  Bit definition for QSPI_DLR register  *******************/
#define  QSPI_DLR_DL                                 ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for QSPI_CCR register  *******************/
#define  QSPI_CCR_INSTRUCTION                        ((uint32_t)0x000000FF)
#define  QSPI_CCR_INSTRUCTION_0                      ((uint32_t)0x00000001)
#define  QSPI_CCR_INSTRUCTION_1                      ((uint32_t)0x00000002)
#define  QSPI_CCR_INSTRUCTION_2                      ((uint32_t)0x00000004)
#define  QSPI_CCR_INSTRUCTION_3                      ((uint32_t)0x00000008)
#define  QSPI_CCR_INSTRUCTION_4                      ((uint32_t)0x00000010)
#define  QSPI_CCR_INSTRUCTION_5                      ((uint32_t)0x00000020)
#define  QSPI_CCR_INSTRUCTION_6                      ((uint32_t)0x00000040)
#define  QSPI_CCR_INSTRUCTION_7                      ((uint32_t)0x00000080)

#define  QSPI_CCR_IMODE                              ((uint32_t)0x00000300)
#define  QSPI_CCR_IMODE_0                            ((uint32_t)0x00000100)
#define  QSPI_CCR_IMODE_1                            ((uint32_t)0x00000200)

#define  QSPI_CCR_ADMODE                             ((uint32_t)0x00000C00)
#define  QSPI_CCR_ADMODE_0                           ((uint32_t)0x00000400)
#define  QSPI_CCR_ADMODE_1                           ((uint32_t)0x00000800)

#define  QSPI_CCR_ADSIZE                             ((uint32_t)0x00003000)
#define  QSPI_CCR_ADSIZE_0                           ((uint32_t)0x00001000)
#define  QSPI_CCR_ADSIZE_1                           ((uint32_t)0x00002000)

#define  QSPI_CCR_ABMODE                             ((uint32_t)0x0000C000)
#define  QSPI_CCR_ABMODE_0                           ((uint32_t)0x00004000)
#define  QSPI_CCR_ABMODE_1                           ((uint32_t)0x00008000)

#define  QSPI_CCR_ABSIZE                             ((uint32_t)0x00030000)
#define  QSPI_CCR_ABSIZE_0                           ((uint32_t)0x00010000)
#define  QSPI_CCR_ABSIZE_1                           ((uint32_t)0x00020000)

#define  QSPI_CCR_DCYC                               ((uint32_t)0x007C0000)
#define  QSPI_CCR_DCYC_0                             ((uint32_t)0x00040000)
#define  QSPI_CCR_DCYC_1                             ((uint32_t)0x00080000)
#define  QSPI_CCR_DCYC_2                             ((uint32_t)0x00100000)
#define  QSPI_CCR_DCYC_3                             ((uint32_t)0x00200000)
#define  QSPI_CCR_DCYC_4                             ((uint32_t)0x00400000)

#define  QSPI_CCR_DMODE                              ((uint32_t)0x03000000)
#define  QSPI_CCR_DMODE_0                            ((uint32_t)0x01000000)
#define  QSPI_CCR_DMODE_1                            ((uint32_t)0x02000000)

#define  QSPI_CCR_FMODE                              ((uint32_t)0x0C000000)
#define  QSPI_CCR_FMODE_0                            ((uint32_t)0x04000000)
#define  QSPI_CCR_FMODE_1                            ((uint32_t)0x08000000)

#define  QSPI_CCR_SIOO                               ((uint32_t)0x10000000)

/*******************  Bit definition for QSPI_AR register  *******************/
#define  QSPI_AR_ADDR                                ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for QSPI_ABR register  *******************/
#define  QSPI_ABR_ALTERNATE                          ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for QSPI_DR register  *******************/
#define  QSPI_DR_DATA                                ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for QSPI_PSMKR register  *******************/
#define  QSPI_PSMKR_MASK                             ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for QSPI_PSMAR register  *******************/
#define  QSPI_PSMAR_MATCH                            ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for QSPI_PIR register  *******************/
#define  QSPI_PIR_INTERVAL                           ((uint16_t)0xFFFF)

/*******************  Bit definition for QSPI_LPIR register  *******************/
#define  QSPI_LPIR_TIMEOUT                           ((uint16_t)0xFFFF)



#include "ch32h417_conf.h"


#ifdef __cplusplus
}
#endif

#endif 




