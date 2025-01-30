/**
  *
  * @file    apm32f4xx_ddl_spi.c
  * @brief   SPI DDL module driver.
  *
  * @attention
  *
  * Redistribution and use in source and binary forms, with or without modification, 
  * are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors
  *    may be used to endorse or promote products derived from this software without
  *    specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
  * OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  * The original code has been modified by Geehy Semiconductor.
  *
  * Copyright (c) 2016 STMicroelectronics.
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  */
#if defined(USE_FULL_DDL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_ddl_spi.h"
#include "apm32f4xx_ddl_bus.h"
#include "apm32f4xx_ddl_rcm.h"

#ifdef  USE_FULL_ASSERT
#include "apm32_assert.h"
#else
#define ASSERT_PARAM(_PARAM_) ((void)(_PARAM_))
#endif /* USE_FULL_ASSERT */

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined (SPI1) || defined (SPI2) || defined (SPI3) || defined (SPI4) || defined (SPI5) || defined(SPI6)

/** @addtogroup SPI_DDL
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/
/** @defgroup SPI_DDL_Private_Constants SPI Private Constants
  * @{
  */
/* SPI registers Masks */
#define SPI_CTRL1_CLEAR_MASK                 (SPI_CTRL1_CPHA    | SPI_CTRL1_CPOL     | SPI_CTRL1_MSMCFG   | \
                                            SPI_CTRL1_BRSEL      | SPI_CTRL1_LSBSEL | SPI_CTRL1_ISSEL    | \
                                            SPI_CTRL1_SSEN     | SPI_CTRL1_RXOMEN   | SPI_CTRL1_DFLSEL    | \
                                            SPI_CTRL1_CRCNXT | SPI_CTRL1_CRCEN    | SPI_CTRL1_BMOEN | \
                                            SPI_CTRL1_BMEN)
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup SPI_DDL_Private_Macros SPI Private Macros
  * @{
  */
#define IS_DDL_SPI_TRANSFER_DIRECTION(__VALUE__) (((__VALUE__) == DDL_SPI_FULL_DUPLEX)       \
                                                 || ((__VALUE__) == DDL_SPI_SIMPLEX_RX)     \
                                                 || ((__VALUE__) == DDL_SPI_HALF_DUPLEX_RX) \
                                                 || ((__VALUE__) == DDL_SPI_HALF_DUPLEX_TX))

#define IS_DDL_SPI_MODE(__VALUE__) (((__VALUE__) == DDL_SPI_MODE_MASTER) \
                                   || ((__VALUE__) == DDL_SPI_MODE_SLAVE))

#define IS_DDL_SPI_DATAWIDTH(__VALUE__) (((__VALUE__) == DDL_SPI_DATAWIDTH_8BIT)  \
                                        || ((__VALUE__) == DDL_SPI_DATAWIDTH_16BIT))

#define IS_DDL_SPI_POLARITY(__VALUE__) (((__VALUE__) == DDL_SPI_POLARITY_LOW) \
                                       || ((__VALUE__) == DDL_SPI_POLARITY_HIGH))

#define IS_DDL_SPI_PHASE(__VALUE__) (((__VALUE__) == DDL_SPI_PHASE_1EDGE) \
                                    || ((__VALUE__) == DDL_SPI_PHASE_2EDGE))

#define IS_DDL_SPI_NSS(__VALUE__) (((__VALUE__) == DDL_SPI_NSS_SOFT)          \
                                  || ((__VALUE__) == DDL_SPI_NSS_HARD_INPUT) \
                                  || ((__VALUE__) == DDL_SPI_NSS_HARD_OUTPUT))

#define IS_DDL_SPI_BAUDRATE(__VALUE__) (((__VALUE__) == DDL_SPI_BAUDRATEPRESCALER_DIV2)      \
                                       || ((__VALUE__) == DDL_SPI_BAUDRATEPRESCALER_DIV4)   \
                                       || ((__VALUE__) == DDL_SPI_BAUDRATEPRESCALER_DIV8)   \
                                       || ((__VALUE__) == DDL_SPI_BAUDRATEPRESCALER_DIV16)  \
                                       || ((__VALUE__) == DDL_SPI_BAUDRATEPRESCALER_DIV32)  \
                                       || ((__VALUE__) == DDL_SPI_BAUDRATEPRESCALER_DIV64)  \
                                       || ((__VALUE__) == DDL_SPI_BAUDRATEPRESCALER_DIV128) \
                                       || ((__VALUE__) == DDL_SPI_BAUDRATEPRESCALER_DIV256))

#define IS_DDL_SPI_BITORDER(__VALUE__) (((__VALUE__) == DDL_SPI_LSB_FIRST) \
                                       || ((__VALUE__) == DDL_SPI_MSB_FIRST))

#define IS_DDL_SPI_CRCCALCULATION(__VALUE__) (((__VALUE__) == DDL_SPI_CRCCALCULATION_ENABLE) \
                                             || ((__VALUE__) == DDL_SPI_CRCCALCULATION_DISABLE))

#define IS_DDL_SPI_CRC_POLYNOMIAL(__VALUE__) ((__VALUE__) >= 0x1U)

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup SPI_DDL_Exported_Functions
  * @{
  */

/** @addtogroup SPI_DDL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize the SPI registers to their default reset values.
  * @param  SPIx SPI Instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: SPI registers are de-initialized
  *          - ERROR: SPI registers are not de-initialized
  */
ErrorStatus DDL_SPI_DeInit(SPI_TypeDef *SPIx)
{
  ErrorStatus status = ERROR;

  /* Check the parameters */
  ASSERT_PARAM(IS_SPI_ALL_INSTANCE(SPIx));

#if defined(SPI1)
  if (SPIx == SPI1)
  {
    /* Force reset of SPI clock */
    DDL_APB2_GRP1_ForceReset(DDL_APB2_GRP1_PERIPH_SPI1);

    /* Release reset of SPI clock */
    DDL_APB2_GRP1_ReleaseReset(DDL_APB2_GRP1_PERIPH_SPI1);

    status = SUCCESS;
  }
#endif /* SPI1 */
#if defined(SPI2)
  if (SPIx == SPI2)
  {
    /* Force reset of SPI clock */
    DDL_APB1_GRP1_ForceReset(DDL_APB1_GRP1_PERIPH_SPI2);

    /* Release reset of SPI clock */
    DDL_APB1_GRP1_ReleaseReset(DDL_APB1_GRP1_PERIPH_SPI2);

    status = SUCCESS;
  }
#endif /* SPI2 */
#if defined(SPI3)
  if (SPIx == SPI3)
  {
    /* Force reset of SPI clock */
    DDL_APB1_GRP1_ForceReset(DDL_APB1_GRP1_PERIPH_SPI3);

    /* Release reset of SPI clock */
    DDL_APB1_GRP1_ReleaseReset(DDL_APB1_GRP1_PERIPH_SPI3);

    status = SUCCESS;
  }
#endif /* SPI3 */
#if defined(SPI4)
  if (SPIx == SPI4)
  {
    /* Force reset of SPI clock */
    DDL_APB2_GRP1_ForceReset(DDL_APB2_GRP1_PERIPH_SPI4);

    /* Release reset of SPI clock */
    DDL_APB2_GRP1_ReleaseReset(DDL_APB2_GRP1_PERIPH_SPI4);

    status = SUCCESS;
  }
#endif /* SPI4 */
#if defined(SPI5)
  if (SPIx == SPI5)
  {
    /* Force reset of SPI clock */
    DDL_APB2_GRP1_ForceReset(DDL_APB2_GRP1_PERIPH_SPI5);

    /* Release reset of SPI clock */
    DDL_APB2_GRP1_ReleaseReset(DDL_APB2_GRP1_PERIPH_SPI5);

    status = SUCCESS;
  }
#endif /* SPI5 */
#if defined(SPI6)
  if (SPIx == SPI6)
  {
    /* Force reset of SPI clock */
    DDL_APB2_GRP1_ForceReset(DDL_APB2_GRP1_PERIPH_SPI6);

    /* Release reset of SPI clock */
    DDL_APB2_GRP1_ReleaseReset(DDL_APB2_GRP1_PERIPH_SPI6);

    status = SUCCESS;
  }
#endif /* SPI6 */

  return status;
}

/**
  * @brief  Initialize the SPI registers according to the specified parameters in SPI_InitStruct.
  * @note   As some bits in SPI configuration registers can only be written when the SPI is disabled (SPI_CTRL1_SPIEN bit =0),
  *         SPI peripheral should be in disabled state prior calling this function. Otherwise, ERROR result will be returned.
  * @param  SPIx SPI Instance
  * @param  SPI_InitStruct pointer to a @ref DDL_SPI_InitTypeDef structure
  * @retval An ErrorStatus enumeration value. (Return always SUCCESS)
  */
ErrorStatus DDL_SPI_Init(SPI_TypeDef *SPIx, DDL_SPI_InitTypeDef *SPI_InitStruct)
{
  ErrorStatus status = ERROR;

  /* Check the SPI Instance SPIx*/
  ASSERT_PARAM(IS_SPI_ALL_INSTANCE(SPIx));

  /* Check the SPI parameters from SPI_InitStruct*/
  ASSERT_PARAM(IS_DDL_SPI_TRANSFER_DIRECTION(SPI_InitStruct->TransferDirection));
  ASSERT_PARAM(IS_DDL_SPI_MODE(SPI_InitStruct->Mode));
  ASSERT_PARAM(IS_DDL_SPI_DATAWIDTH(SPI_InitStruct->DataWidth));
  ASSERT_PARAM(IS_DDL_SPI_POLARITY(SPI_InitStruct->ClockPolarity));
  ASSERT_PARAM(IS_DDL_SPI_PHASE(SPI_InitStruct->ClockPhase));
  ASSERT_PARAM(IS_DDL_SPI_NSS(SPI_InitStruct->NSS));
  ASSERT_PARAM(IS_DDL_SPI_BAUDRATE(SPI_InitStruct->BaudRate));
  ASSERT_PARAM(IS_DDL_SPI_BITORDER(SPI_InitStruct->BitOrder));
  ASSERT_PARAM(IS_DDL_SPI_CRCCALCULATION(SPI_InitStruct->CRCCalculation));

  if (DDL_SPI_IsEnabled(SPIx) == 0x00000000U)
  {
    /*---------------------------- SPIx CTRL1 Configuration ------------------------
     * Configure SPIx CTRL1 with parameters:
     * - TransferDirection:  SPI_CTRL1_BMEN, SPI_CTRL1_BMOEN and SPI_CTRL1_RXOMEN bits
     * - Master/Slave Mode:  SPI_CTRL1_MSMCFG bit
     * - DataWidth:          SPI_CTRL1_DFLSEL bit
     * - ClockPolarity:      SPI_CTRL1_CPOL bit
     * - ClockPhase:         SPI_CTRL1_CPHA bit
     * - NSS management:     SPI_CTRL1_SSEN bit
     * - BaudRate prescaler: SPI_CTRL1_BRSEL[2:0] bits
     * - BitOrder:           SPI_CTRL1_LSBSEL bit
     * - CRCCalculation:     SPI_CTRL1_CRCEN bit
     */
    MODIFY_REG(SPIx->CTRL1,
               SPI_CTRL1_CLEAR_MASK,
               SPI_InitStruct->TransferDirection | SPI_InitStruct->Mode | SPI_InitStruct->DataWidth |
               SPI_InitStruct->ClockPolarity | SPI_InitStruct->ClockPhase |
               SPI_InitStruct->NSS | SPI_InitStruct->BaudRate |
               SPI_InitStruct->BitOrder | SPI_InitStruct->CRCCalculation);

    /*---------------------------- SPIx CTRL2 Configuration ------------------------
     * Configure SPIx CTRL2 with parameters:
     * - NSS management:     SSOE bit
     */
    MODIFY_REG(SPIx->CTRL2, SPI_CTRL2_SSOEN, (SPI_InitStruct->NSS >> 16U));

    /*---------------------------- SPIx CRCPOLY Configuration ----------------------
     * Configure SPIx CRCPR with parameters:
     * - CRCPoly:            CRCPOLY[15:0] bits
     */
    if (SPI_InitStruct->CRCCalculation == DDL_SPI_CRCCALCULATION_ENABLE)
    {
      ASSERT_PARAM(IS_DDL_SPI_CRC_POLYNOMIAL(SPI_InitStruct->CRCPoly));
      DDL_SPI_SetCRCPolynomial(SPIx, SPI_InitStruct->CRCPoly);
    }
    status = SUCCESS;
  }

  /* Activate the SPI mode (Reset I2SMOD bit in I2SCFGR register) */
  CLEAR_BIT(SPIx->I2SCFG, SPI_I2SCFG_MODESEL);
  return status;
}

/**
  * @brief  Set each @ref DDL_SPI_InitTypeDef field to default value.
  * @param  SPI_InitStruct pointer to a @ref DDL_SPI_InitTypeDef structure
  * whose fields will be set to default values.
  * @retval None
  */
void DDL_SPI_StructInit(DDL_SPI_InitTypeDef *SPI_InitStruct)
{
  /* Set SPI_InitStruct fields to default values */
  SPI_InitStruct->TransferDirection = DDL_SPI_FULL_DUPLEX;
  SPI_InitStruct->Mode              = DDL_SPI_MODE_SLAVE;
  SPI_InitStruct->DataWidth         = DDL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct->ClockPolarity     = DDL_SPI_POLARITY_LOW;
  SPI_InitStruct->ClockPhase        = DDL_SPI_PHASE_1EDGE;
  SPI_InitStruct->NSS               = DDL_SPI_NSS_HARD_INPUT;
  SPI_InitStruct->BaudRate          = DDL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct->BitOrder          = DDL_SPI_MSB_FIRST;
  SPI_InitStruct->CRCCalculation    = DDL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct->CRCPoly           = 7U;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup I2S_DDL
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup I2S_DDL_Private_Constants I2S Private Constants
  * @{
  */
/* I2S registers Masks */
#define I2S_I2SCFG_CLEAR_MASK             (SPI_I2SCFG_CHLEN   | SPI_I2SCFG_DATALEN | \
                                            SPI_I2SCFG_CPOL   | SPI_I2SCFG_I2SSSEL | \
                                            SPI_I2SCFG_I2SMOD  | SPI_I2SCFG_MODESEL )

#define I2S_I2SPSC_CLEAR_MASK               0x0002U
/**
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/** @defgroup I2S_DDL_Private_Macros I2S Private Macros
  * @{
  */

#define IS_DDL_I2S_DATAFORMAT(__VALUE__)  (((__VALUE__) == DDL_I2S_DATAFORMAT_16B)             \
                                          || ((__VALUE__) == DDL_I2S_DATAFORMAT_16B_EXTENDED) \
                                          || ((__VALUE__) == DDL_I2S_DATAFORMAT_24B)          \
                                          || ((__VALUE__) == DDL_I2S_DATAFORMAT_32B))

#define IS_DDL_I2S_CPOL(__VALUE__)        (((__VALUE__) == DDL_I2S_POLARITY_LOW)  \
                                          || ((__VALUE__) == DDL_I2S_POLARITY_HIGH))

#define IS_DDL_I2S_STANDARD(__VALUE__)    (((__VALUE__) == DDL_I2S_STANDARD_PHILIPS)      \
                                          || ((__VALUE__) == DDL_I2S_STANDARD_MSB)       \
                                          || ((__VALUE__) == DDL_I2S_STANDARD_LSB)       \
                                          || ((__VALUE__) == DDL_I2S_STANDARD_PCM_SHORT) \
                                          || ((__VALUE__) == DDL_I2S_STANDARD_PCM_LONG))

#define IS_DDL_I2S_MODE(__VALUE__)        (((__VALUE__) == DDL_I2S_MODE_SLAVE_TX)     \
                                          || ((__VALUE__) == DDL_I2S_MODE_SLAVE_RX)  \
                                          || ((__VALUE__) == DDL_I2S_MODE_MASTER_TX) \
                                          || ((__VALUE__) == DDL_I2S_MODE_MASTER_RX))

#define IS_DDL_I2S_MCLK_OUTPUT(__VALUE__) (((__VALUE__) == DDL_I2S_MCLK_OUTPUT_ENABLE) \
                                          || ((__VALUE__) == DDL_I2S_MCLK_OUTPUT_DISABLE))

#define IS_DDL_I2S_AUDIO_FREQ(__VALUE__) ((((__VALUE__) >= DDL_I2S_AUDIOFREQ_8K)       \
                                          && ((__VALUE__) <= DDL_I2S_AUDIOFREQ_192K)) \
                                         || ((__VALUE__) == DDL_I2S_AUDIOFREQ_DEFAULT))

#define IS_DDL_I2S_PRESCALER_LINEAR(__VALUE__)  ((__VALUE__) >= 0x2U)

#define IS_DDL_I2S_PRESCALER_PARITY(__VALUE__) (((__VALUE__) == DDL_I2S_PRESCALER_PARITY_EVEN) \
                                               || ((__VALUE__) == DDL_I2S_PRESCALER_PARITY_ODD))
/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup I2S_DDL_Exported_Functions
  * @{
  */

/** @addtogroup I2S_DDL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize the SPI/I2S registers to their default reset values.
  * @param  SPIx SPI Instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: SPI registers are de-initialized
  *          - ERROR: SPI registers are not de-initialized
  */
ErrorStatus DDL_I2S_DeInit(SPI_TypeDef *SPIx)
{
  return DDL_SPI_DeInit(SPIx);
}

/**
  * @brief  Initializes the SPI/I2S registers according to the specified parameters in I2S_InitStruct.
  * @note   As some bits in SPI configuration registers can only be written when the SPI is disabled (SPI_CTRL1_SPIEN bit =0),
  *         SPI peripheral should be in disabled state prior calling this function. Otherwise, ERROR result will be returned.
  * @param  SPIx SPI Instance
  * @param  I2S_InitStruct pointer to a @ref DDL_I2S_InitTypeDef structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: SPI registers are Initialized
  *          - ERROR: SPI registers are not Initialized
  */
ErrorStatus DDL_I2S_Init(SPI_TypeDef *SPIx, DDL_I2S_InitTypeDef *I2S_InitStruct)
{
  uint32_t i2sdiv = 2U;
  uint32_t i2sodd = 0U;
  uint32_t packetlength = 1U;
  uint32_t tmp;
  uint32_t sourceclock;
  ErrorStatus status = ERROR;

  /* Check the I2S parameters */
  ASSERT_PARAM(IS_I2S_ALL_INSTANCE(SPIx));
  ASSERT_PARAM(IS_DDL_I2S_MODE(I2S_InitStruct->Mode));
  ASSERT_PARAM(IS_DDL_I2S_STANDARD(I2S_InitStruct->Standard));
  ASSERT_PARAM(IS_DDL_I2S_DATAFORMAT(I2S_InitStruct->DataFormat));
  ASSERT_PARAM(IS_DDL_I2S_MCLK_OUTPUT(I2S_InitStruct->MCLKOutput));
  ASSERT_PARAM(IS_DDL_I2S_AUDIO_FREQ(I2S_InitStruct->AudioFreq));
  ASSERT_PARAM(IS_DDL_I2S_CPOL(I2S_InitStruct->ClockPolarity));

  if (DDL_I2S_IsEnabled(SPIx) == 0x00000000U)
  {
    /*---------------------------- SPIx I2SCFG Configuration --------------------
     * Configure SPIx I2SCFG with parameters:
     * - Mode:          SPI_I2SCFG_I2SMOD[1:0] bit
     * - Standard:      SPI_I2SCFG_I2SSSEL[1:0] and SPI_I2SCFG_PFSSEL bits
     * - DataFormat:    SPI_I2SCFG_CHLEN and SPI_I2SCFG_DATALEN bits
     * - ClockPolarity: SPI_I2SCFG_CPOL bit
     */

    /* Write to SPIx I2SCFG */
    MODIFY_REG(SPIx->I2SCFG,
               I2S_I2SCFG_CLEAR_MASK,
               I2S_InitStruct->Mode | I2S_InitStruct->Standard |
               I2S_InitStruct->DataFormat | I2S_InitStruct->ClockPolarity |
               SPI_I2SCFG_MODESEL);

    /*---------------------------- SPIx I2SPSC Configuration ----------------------
     * Configure SPIx I2SPSC with parameters:
     * - MCLKOutput:    SPI_I2SPSC_MCOEN bit
     * - AudioFreq:     SPI_I2SPSC_I2SPSC[7:0] and SPI_I2SPSC_ODDPS bits
     */

    /* If the requested audio frequency is not the default, compute the prescaler (i2sodd, i2sdiv)
     * else, default values are used:  i2sodd = 0U, i2sdiv = 2U.
     */
    if (I2S_InitStruct->AudioFreq != DDL_I2S_AUDIOFREQ_DEFAULT)
    {
      /* Check the frame length (For the Prescaler computing)
       * Default value: DDL_I2S_DATAFORMAT_16B (packetlength = 1U).
       */
      if (I2S_InitStruct->DataFormat != DDL_I2S_DATAFORMAT_16B)
      {
        /* Packet length is 32 bits */
        packetlength = 2U;
      }

      /* If an external I2S clock has to be used, the specific define should be set
      in the project configuration or in the apm32f4xx_ll_rcc.h file */
      /* Get the I2S source clock value */
      sourceclock = DDL_RCM_GetI2SClockFreq(DDL_RCM_I2S1_CLKSOURCE);

      /* Compute the Real divider depending on the MCLK output state with a floating point */
      if (I2S_InitStruct->MCLKOutput == DDL_I2S_MCLK_OUTPUT_ENABLE)
      {
        /* MCLK output is enabled */
        tmp = (((((sourceclock / 256U) * 10U) / I2S_InitStruct->AudioFreq)) + 5U);
      }
      else
      {
        /* MCLK output is disabled */
        tmp = (((((sourceclock / (32U * packetlength)) * 10U) / I2S_InitStruct->AudioFreq)) + 5U);
      }

      /* Remove the floating point */
      tmp = tmp / 10U;

      /* Check the parity of the divider */
      i2sodd = (tmp & (uint16_t)0x0001U);

      /* Compute the i2sdiv prescaler */
      i2sdiv = ((tmp - i2sodd) / 2U);

      /* Get the Mask for the Odd bit (SPI_I2SPSC[8]) register */
      i2sodd = (i2sodd << 8U);
    }

    /* Test if the divider is 1 or 0 or greater than 0xFF */
    if ((i2sdiv < 2U) || (i2sdiv > 0xFFU))
    {
      /* Set the default values */
      i2sdiv = 2U;
      i2sodd = 0U;
    }

    /* Write to SPIx I2SPSC register the computed value */
    WRITE_REG(SPIx->I2SPSC, i2sdiv | i2sodd | I2S_InitStruct->MCLKOutput);

    status = SUCCESS;
  }
  return status;
}

/**
  * @brief  Set each @ref DDL_I2S_InitTypeDef field to default value.
  * @param  I2S_InitStruct pointer to a @ref DDL_I2S_InitTypeDef structure
  *         whose fields will be set to default values.
  * @retval None
  */
void DDL_I2S_StructInit(DDL_I2S_InitTypeDef *I2S_InitStruct)
{
  /*--------------- Reset I2S init structure parameters values -----------------*/
  I2S_InitStruct->Mode              = DDL_I2S_MODE_SLAVE_TX;
  I2S_InitStruct->Standard          = DDL_I2S_STANDARD_PHILIPS;
  I2S_InitStruct->DataFormat        = DDL_I2S_DATAFORMAT_16B;
  I2S_InitStruct->MCLKOutput        = DDL_I2S_MCLK_OUTPUT_DISABLE;
  I2S_InitStruct->AudioFreq         = DDL_I2S_AUDIOFREQ_DEFAULT;
  I2S_InitStruct->ClockPolarity     = DDL_I2S_POLARITY_LOW;
}

/**
  * @brief  Set linear and parity prescaler.
  * @note   To calculate value of PrescalerLinear(I2SDIV[7:0] bits) and PrescalerParity(ODD bit)\n
  *         Check Audio frequency table and formulas inside Reference Manual (SPI/I2S).
  * @param  SPIx SPI Instance
  * @param  PrescalerLinear value Min_Data=0x02 and Max_Data=0xFF.
  * @param  PrescalerParity This parameter can be one of the following values:
  *         @arg @ref DDL_I2S_PRESCALER_PARITY_EVEN
  *         @arg @ref DDL_I2S_PRESCALER_PARITY_ODD
  * @retval None
  */
void DDL_I2S_ConfigPrescaler(SPI_TypeDef *SPIx, uint32_t PrescalerLinear, uint32_t PrescalerParity)
{
  /* Check the I2S parameters */
  ASSERT_PARAM(IS_I2S_ALL_INSTANCE(SPIx));
  ASSERT_PARAM(IS_DDL_I2S_PRESCALER_LINEAR(PrescalerLinear));
  ASSERT_PARAM(IS_DDL_I2S_PRESCALER_PARITY(PrescalerParity));

  /* Write to SPIx I2SPSC */
  MODIFY_REG(SPIx->I2SPSC, SPI_I2SPSC_I2SPSC | SPI_I2SPSC_ODDPS, PrescalerLinear | (PrescalerParity << 8U));
}

#if defined (SPI_I2S_FULLDUPLEX_SUPPORT)
/**
  * @brief  Configures the full duplex mode for the I2Sx peripheral using its extension
  *         I2Sxext according to the specified parameters in the I2S_InitStruct.
  * @note   The structure pointed by I2S_InitStruct parameter should be the same
  *         used for the master I2S peripheral. In this case, if the master is
  *         configured as transmitter, the slave will be receiver and vice versa.
  *         Or you can force a different mode by modifying the field I2S_Mode to the
  *         value I2S_SlaveRx or I2S_SlaveTx independently of the master configuration.
  * @param  I2Sxext SPI Instance
  * @param  I2S_InitStruct pointer to a @ref DDL_I2S_InitTypeDef structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: I2Sxext registers are Initialized
  *          - ERROR: I2Sxext registers are not Initialized
  */
ErrorStatus  DDL_I2S_InitFullDuplex(SPI_TypeDef *I2Sxext, DDL_I2S_InitTypeDef *I2S_InitStruct)
{
  uint32_t mode = 0U;
  ErrorStatus status = ERROR;

  /* Check the I2S parameters */
  ASSERT_PARAM(IS_I2S_EXT_ALL_INSTANCE(I2Sxext));
  ASSERT_PARAM(IS_DDL_I2S_MODE(I2S_InitStruct->Mode));
  ASSERT_PARAM(IS_DDL_I2S_STANDARD(I2S_InitStruct->Standard));
  ASSERT_PARAM(IS_DDL_I2S_DATAFORMAT(I2S_InitStruct->DataFormat));
  ASSERT_PARAM(IS_DDL_I2S_CPOL(I2S_InitStruct->ClockPolarity));

  if (DDL_I2S_IsEnabled(I2Sxext) == 0x00000000U)
  {
    /*---------------------------- SPIx I2SCFG Configuration --------------------
     * Configure SPIx I2SCFG with parameters:
     * - Mode:          SPI_I2SCFG_I2SMOD[1:0] bit
     * - Standard:      SPI_I2SCFG_I2SSSEL[1:0] and SPI_I2SCFG_PFSSEL bits
     * - DataFormat:    SPI_I2SCFG_CHLEN and SPI_I2SCFG_DATALEN bits
     * - ClockPolarity: SPI_I2SCFG_CPOL bit
     */

    /* Reset I2SPSC registers */
    WRITE_REG(I2Sxext->I2SPSC, I2S_I2SPSC_CLEAR_MASK);

    /* Get the mode to be configured for the extended I2S */
    if ((I2S_InitStruct->Mode == DDL_I2S_MODE_MASTER_TX) || (I2S_InitStruct->Mode == DDL_I2S_MODE_SLAVE_TX))
    {
      mode = DDL_I2S_MODE_SLAVE_RX;
    }
    else
    {
      if ((I2S_InitStruct->Mode == DDL_I2S_MODE_MASTER_RX) || (I2S_InitStruct->Mode == DDL_I2S_MODE_SLAVE_RX))
      {
        mode = DDL_I2S_MODE_SLAVE_TX;
      }
    }

    /* Write to SPIx I2SCFG */
    MODIFY_REG(I2Sxext->I2SCFG,
               I2S_I2SCFG_CLEAR_MASK,
               I2S_InitStruct->Standard |
               I2S_InitStruct->DataFormat | I2S_InitStruct->ClockPolarity |
               SPI_I2SCFG_MODESEL | mode);

    status = SUCCESS;
  }
  return status;
}
#endif /* SPI_I2S_FULLDUPLEX_SUPPORT */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* defined (SPI1) || defined (SPI2) || defined (SPI3) || defined (SPI4) || defined (SPI5) || defined(SPI6) */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */

