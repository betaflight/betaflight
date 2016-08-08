/**
  ******************************************************************************
  * @file    stm32f4xx_dfsdm.c
  * @author  MCD Application Team
  * @version V1.7.1
  * @date    20-May-2016
  * @brief   This file provides firmware functions to manage the following
  *          functionalities of Digital Filter for Sigma Delta modulator 
  *          (DFSDM) peripheral:
  *           + Initialization functions.
  *           + Configuration functions.
  *           + Interrupts and flags management functions.
  *
  *  @verbatim
  *
================================================================================
                   ##### How to use this driver #####
================================================================================
 [..]

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_dfsdm.h"
#include "stm32f4xx_rcc.h"

/** @addtogroup STM32F4xx_StdPeriph_Driver
  * @{
  */

/** @defgroup DFSDM
  * @brief DFSDM driver modules
  * @{
  */
#if defined(STM32F412xG)

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/

#define CHCFGR_INIT_CLEAR_MASK               (uint32_t) 0xFFFE0F10

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup DFSDM_Private_Functions
  * @{
  */

/** @defgroup DFSDM_Group1 Initialization functions
 *  @brief   Initialization functions
 *
@verbatim
 ===============================================================================
                             Initialization functions
 ===============================================================================
  This section provides functions allowing to:
   - Deinitialize  the DFSDM
   - Initialize DFSDM serial channels transceiver
   - Initialize DFSDM filter

@endverbatim
  * @{
  */

/**
  * @brief  Deinitializes the DFSDM peripheral registers to their default reset values.
  * @param  None.
  * @retval None.
  *
  */
void DFSDM_DeInit(void)
{
  /* Enable LPTx reset state */
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_DFSDM, ENABLE);
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_DFSDM, DISABLE);
}

/**
  * @brief  Initializes the DFSDM serial channels transceiver  according to the specified
  *         parameters in the DFSDM_TransceiverInit.
  * @param  DFSDM_Channelx: where x can be a value from 0 to 7 to select the DFSDM Channel.
  * @param  DFSDM_TransceiverInitStruct: pointer to a DFSDM_TransceiverInitTypeDef structure
  *         that contains the configuration information for the specified channel.
  * @retval None
  * @note   It is mandatory to disable the selected channel to use this function.
  */
void DFSDM_TransceiverInit(DFSDM_Channel_TypeDef* DFSDM_Channelx, DFSDM_TransceiverInitTypeDef* DFSDM_TransceiverInitStruct)
{
    uint32_t tmpreg1 = 0;
    uint32_t tmpreg2 = 0;

    /* Check the parameters */
    assert_param(IS_DFSDM_ALL_CHANNEL(DFSDM_Channelx));
    assert_param(IS_DFSDM_INTERFACE(DFSDM_TransceiverInitStruct->DFSDM_Interface));    
    assert_param(IS_DFSDM_Input_MODE(DFSDM_TransceiverInitStruct->DFSDM_Input));
    assert_param(IS_DFSDM_Redirection_STATE(DFSDM_TransceiverInitStruct->DFSDM_Redirection));
    assert_param(IS_DFSDM_PACK_MODE(DFSDM_TransceiverInitStruct->DFSDM_PackingMode));    
    assert_param(IS_DFSDM_CLOCK(DFSDM_TransceiverInitStruct->DFSDM_Clock));
    assert_param(IS_DFSDM_DATA_RIGHT_BIT_SHIFT(DFSDM_TransceiverInitStruct->DFSDM_DataRightShift));
    assert_param(IS_DFSDM_OFFSET(DFSDM_TransceiverInitStruct->DFSDM_Offset));
    assert_param(IS_DFSDM_CLK_DETECTOR_STATE(DFSDM_TransceiverInitStruct->DFSDM_CLKAbsenceDetector));
    assert_param(IS_DFSDM_SC_DETECTOR_STATE(DFSDM_TransceiverInitStruct->DFSDM_ShortCircuitDetector));

    /* Get the DFSDM Channelx CHCFGR1 value */
    tmpreg1 = DFSDM_Channelx->CHCFGR1;

    /* Clear SITP, CKABEN, SCDEN and SPICKSEL bits */
    tmpreg1 &= CHCFGR_INIT_CLEAR_MASK;

    /* Set or Reset SITP bits according to DFSDM_Interface value */
    /* Set or Reset SPICKSEL bits according to DFSDM_Clock value */
    /* Set or Reset DATMPX bits according to DFSDM_InputMode value */
    /* Set or Reset CHINSEL bits according to DFSDM_Redirection value */
    /* Set or Reset DATPACK bits according to DFSDM_PackingMode value */
    /* Set or Reset CKABEN bit according to DFSDM_CLKAbsenceDetector value */
    /* Set or Reset SCDEN bit according to DFSDM_ShortCircuitDetector value */
    tmpreg1 |= (DFSDM_TransceiverInitStruct->DFSDM_Interface |
                DFSDM_TransceiverInitStruct->DFSDM_Clock |
                DFSDM_TransceiverInitStruct->DFSDM_Input |
                DFSDM_TransceiverInitStruct->DFSDM_Redirection |
                DFSDM_TransceiverInitStruct->DFSDM_PackingMode |
                DFSDM_TransceiverInitStruct->DFSDM_CLKAbsenceDetector |
                DFSDM_TransceiverInitStruct->DFSDM_ShortCircuitDetector);

    /* Write to DFSDM Channelx CHCFGR1R */
    DFSDM_Channelx->CHCFGR1 = tmpreg1;

    /* Get the DFSDM Channelx CHCFGR2 value */
    tmpreg2 = DFSDM_Channelx->CHCFGR2;

    /* Clear DTRBS and OFFSET bits */
    tmpreg2 &= ~(DFSDM_CHCFGR2_DTRBS | DFSDM_CHCFGR2_OFFSET);

    /* Set or Reset DTRBS bits according to DFSDM_DataRightShift value */
    /* Set or Reset OFFSET bits according to DFSDM_Offset value */
    tmpreg2 |= (((DFSDM_TransceiverInitStruct->DFSDM_DataRightShift) <<3 ) |
                ((DFSDM_TransceiverInitStruct->DFSDM_Offset) <<8 ));

    /* Write to DFSDM Channelx CHCFGR1R */
    DFSDM_Channelx->CHCFGR2 = tmpreg2;
}

/**
  * @brief  Fills each DFSDM_TransceiverInitStruct member with its default value.
  * @param  DFSDM_TransceiverInitStruct : pointer to a DFSDM_TransceiverInitTypeDef structure
  *         which will be initialized.
  * @retval None
  */
void DFSDM_TransceiverStructInit(DFSDM_TransceiverInitTypeDef* DFSDM_TransceiverInitStruct)
{
    /* SPI with rising edge to strobe data is selected as default serial interface */
    DFSDM_TransceiverInitStruct->DFSDM_Interface = DFSDM_Interface_SPI_FallingEdge;

    /* Clock coming from internal DFSDM_CKOUT output is selected as default serial clock */
    DFSDM_TransceiverInitStruct->DFSDM_Clock = DFSDM_Clock_Internal;

    /* No data right bit-shift is selected as default data right bit-shift */
    DFSDM_TransceiverInitStruct->DFSDM_DataRightShift = 0x0;

    /* No offset is selected as default offset */
    DFSDM_TransceiverInitStruct->DFSDM_Offset = 0x0;

    /* Clock Absence Detector is Enabled as default state */
    DFSDM_TransceiverInitStruct->DFSDM_CLKAbsenceDetector = DFSDM_CLKAbsenceDetector_Enable;
}

/**
  * @brief  Initializes the DFSDMx Filter according to the specified
  *         parameters in the DFSDM_FilterInitStruct.
  * @param  DFSDMx: where x can be a value from 0 to 1 to select the DFSDM Module.
  * @param  DFSDM_FilterInitStruct: pointer to a DFSDM_FilterInitTypeDef structure
  *         that contains the configuration information for the specified filter.
  * @retval None
  *
  * @note   It is mandatory to disable the selected filter to use this function.
  */
void DFSDM_FilterInit(DFSDM_TypeDef* DFSDMx, DFSDM_FilterInitTypeDef* DFSDM_FilterInitStruct)
{
    uint32_t tmpreg1 = 0;

    /* Check the parameters */
    assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));
    assert_param(IS_DFSDM_SINC_ORDER(DFSDM_FilterInitStruct->DFSDM_SincOrder));
    assert_param(IS_DFSDM_SINC_OVRSMPL_RATIO(DFSDM_FilterInitStruct->DFSDM_FilterOversamplingRatio));
    assert_param(IS_DFSDM_INTG_OVRSMPL_RATIO(DFSDM_FilterInitStruct->DFSDM_IntegratorOversamplingRatio));

    /* Get the DFSDMx FCR value */
    tmpreg1 = DFSDMx->FLTFCR;

    /* Clear FORD, FOSR and IOSR bits */
    tmpreg1 &= ~(DFSDM_FLTFCR_FORD | DFSDM_FLTFCR_FOSR | DFSDM_FLTFCR_IOSR);

    /* Set or Reset FORD bits according to DFSDM_SincOrder value */
    /* Set or Reset FOSR bits according to DFSDM_FilterOversamplingRatio value */
    /* Set or Reset IOSR bits according to DFSDM_IntegratorOversamplingRatio value */
    tmpreg1 |= (DFSDM_FilterInitStruct->DFSDM_SincOrder |
               ((DFSDM_FilterInitStruct->DFSDM_FilterOversamplingRatio -1) << 16) |
               (DFSDM_FilterInitStruct->DFSDM_IntegratorOversamplingRatio -1));

    /* Write to DFSDMx FCR */
    DFSDMx->FLTFCR = tmpreg1;
}

/**
  * @brief  Fills each DFSDM_FilterInitStruct member with its default value.
  * @param  DFSDM_FilterInitStruct: pointer to a DFSDM_FilterInitTypeDef structure
  *         which will be initialized.
  * @retval None
  */
void DFSDM_FilterStructInit(DFSDM_FilterInitTypeDef* DFSDM_FilterInitStruct)
{
    /* Order = 3 is selected as default sinc order */
    DFSDM_FilterInitStruct->DFSDM_SincOrder = DFSDM_SincOrder_Sinc3;

    /* Ratio = 64 is selected as default oversampling ratio */
    DFSDM_FilterInitStruct->DFSDM_FilterOversamplingRatio  = 64 ;

    /* Ratio = 4 is selected as default integrator oversampling ratio */
    DFSDM_FilterInitStruct->DFSDM_IntegratorOversamplingRatio = 4;
}

/**
  * @}
  */

/** @defgroup DFSDM_Group2 Configuration functions
 *  @brief   Configuration functions
 *
@verbatim
 ===============================================================================
                       Configuration functions
 ===============================================================================
    This section provides functions allowing to configure DFSDM:
    - Enable/Disable (DFSDM peripheral, Channel, Filter)
    - Configure Clock output 
    - Configure Injected/Regular channels for Conversion
    - Configure  short circuit detector
    - Configure Analog watchdog filter

@endverbatim
  * @{
  */

/**
  * @brief  Enables or disables the DFSDM peripheral.
  * @param  NewState: new state of the DFSDM interface.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DFSDM_Cmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Set the ENABLE bit */
    DFSDM1_Channel0 -> CHCFGR1 |= DFSDM_CHCFGR1_DFSDMEN;
  }
  else
  {
    /* Reset the ENABLE bit */
    DFSDM1_Channel0 -> CHCFGR1 &= ~(DFSDM_CHCFGR1_DFSDMEN);
  }
}

/**
  * @brief  Enables or disables the specified DFSDM serial channelx.
  * @param  DFSDM_Channelx: where x can be a value from 0 to 7 to select the DFSDM Channel.
  * @param  NewState: new state of the DFSDM serial channelx .
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DFSDM_ChannelCmd(DFSDM_Channel_TypeDef* DFSDM_Channelx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_CHANNEL(DFSDM_Channelx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Set the ENABLE bit */
    DFSDM_Channelx->CHCFGR1 |= DFSDM_CHCFGR1_CHEN;
  }
  else
  {
    /* Reset the ENABLE bit */
    DFSDM_Channelx->CHCFGR1 &= ~(DFSDM_CHCFGR1_CHEN);
  }
}

/**
  * @brief  Enables or disables the specified DFSDMx Filter.
  * @param  DFSDMx: where x can be 0, 1 to select the DFSDM module.
  * @param  NewState: new state of the selected DFSDM module.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DFSDM_FilterCmd(DFSDM_TypeDef* DFSDMx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Set the ENABLE bit */
    DFSDMx->FLTCR1 |= DFSDM_FLTCR1_DFEN;
  }
  else
  {
    /* Reset the ENABLE bit */
    DFSDMx->FLTCR1 &= ~(DFSDM_FLTCR1_DFEN);
  }
}

/**
  * @brief  Configures the Output serial clock divider.
  * @param  DFSDM_ClkOutDivision: Defines the divider for the output serial clock
  *         This parameter can be a value between 1 and 256.
  * @retval None
  * @note   The output serial clock is stopped if the divider =1.
  *         By default the serial output clock is stopped.
  */
void DFSDM_ConfigClkOutputDivider(uint32_t DFSDM_ClkOutDivision)
{
    uint32_t tmpreg1 = 0;

    /* Check the parameters */
    assert_param(IS_DFSDM_CLOCK_OUT_DIVIDER(DFSDM_ClkOutDivision));

    /* Get the DFSDM_Channel0 CHCFGR1 value */
    tmpreg1 = DFSDM1_Channel0 -> CHCFGR1;

    /* Clear the CKOUTDIV bits */
    tmpreg1 &= (uint32_t)(~DFSDM_CHCFGR1_CKOUTDIV);

    /* Set or Reset the CKOUTDIV bits */
    tmpreg1 |= (uint32_t)((DFSDM_ClkOutDivision - 1) << 16);

    /* Write to DFSDM Channel0 CHCFGR1 */
    DFSDM1_Channel0 -> CHCFGR1 = tmpreg1;
}

/**
  * @brief  Configures the Output serial clock source.
  * @param  DFSDM_ClkOutSource: Defines the divider for the output serial clock
  *         This parameter can be a value of:
  *            @arg DFSDM_ClkOutSource_SysClock
  *            @arg DFSDM_ClkOutSource_AudioClock   
  * @retval None
  */
void DFSDM_ConfigClkOutputSource(uint32_t DFSDM_ClkOutSource)
{
    uint32_t tmpreg1 = 0;

    /* Check the parameters */
    assert_param(IS_DFSDM_CLOCK_OUT_SOURCE(DFSDM_ClkOutSource));

    /* Get the DFSDM_Channel0 CHCFGR1 value */
    tmpreg1 = DFSDM1_Channel0 -> CHCFGR1;

    /* Clear the CKOUTSRC bit */
    tmpreg1 &= ~(DFSDM_CHCFGR1_CKOUTSRC);

    /* Set or Reset the CKOUTSRC bit */
    tmpreg1 |= DFSDM_ClkOutSource;

    /* Write to DFSDM Channel0 CHCFGR1 */
    DFSDM1_Channel0 -> CHCFGR1 = tmpreg1;
}

/**
  * @brief  Enables or disables the specified Break_i siganl to the specified DFSDM_Channelx.
  * @param  DFSDM_Channelx: where x can be a value from 0 to 7 to select the DFSDM Channel.
  * @param  DFSDM_SCDBreak_i: where i can be a value from 0 to 3 to select the specified Break signal.
  * @param  NewState: new state of the selected DFSDM_SCDBreak_i.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DFSDM_ConfigBRKAnalogWatchDog(DFSDM_Channel_TypeDef* DFSDM_Channelx, uint32_t DFSDM_SCDBreak_i, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_CHANNEL(DFSDM_Channelx));
  assert_param(IS_DFSDM_SCD_BREAK_SIGNAL(DFSDM_SCDBreak_i));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Set the BKSCD[i] bit */
     DFSDM_Channelx -> CHAWSCDR |= DFSDM_SCDBreak_i;
  }
  else
  {
    /* Reset the BKSCD[i] bit */
    DFSDM_Channelx -> CHAWSCDR &= ~(DFSDM_SCDBreak_i);
  }
}

/**
  * @brief  Enables or disables the specified Break_i siganl to the specified DFSDM_Channelx.
  * @param  DFSDM_Channelx: where x can be a value from 0 to 7 to select the DFSDM Channel.
  * @param  DFSDM_SCDBreak_i: where i can be a value from 0 to 3 to select the specified Break signal.
  * @param  NewState: new state of the selected DFSDM_SCDBreak_i.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DFSDM_ConfigBRKShortCircuitDetector(DFSDM_Channel_TypeDef* DFSDM_Channelx, uint32_t DFSDM_SCDBreak_i, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_CHANNEL(DFSDM_Channelx));
  assert_param(IS_DFSDM_SCD_BREAK_SIGNAL(DFSDM_SCDBreak_i));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Set the BKSCD[i] bit */
     DFSDM_Channelx -> CHAWSCDR |= DFSDM_SCDBreak_i;
  }
  else
  {
    /* Reset the BKSCD[i] bit */
    DFSDM_Channelx -> CHAWSCDR &= ~(DFSDM_SCDBreak_i);
  }
}

/**
  * @brief  Defines the threshold counter for the short circuit detector for the selected DFSDM_Channelx.
  * @param  DFSDM_Channelx: where x can be a value from 0 to 7 to select the DFSDM Channel.
  * @param  DFSDM_SCDThreshold: The threshold counter, this parameter can be a value between 0 and 255.
  * @retval None
  */
void DFSDM_ConfigShortCircuitThreshold(DFSDM_Channel_TypeDef* DFSDM_Channelx, uint32_t DFSDM_SCDThreshold)
{
    uint32_t tmpreg1 = 0;

    /* Check the parameters */
    assert_param(IS_DFSDM_ALL_CHANNEL(DFSDM_Channelx));
    assert_param(IS_DFSDM_CSD_THRESHOLD_VALUE(DFSDM_SCDThreshold));

    /* Get the DFSDM_Channelx AWSCDR value */
    tmpreg1 = DFSDM_Channelx -> CHAWSCDR;

    /* Clear the SCDT bits */
    tmpreg1 &= ~(DFSDM_CHAWSCDR_SCDT);

    /* Set or Reset the SCDT bits */
    tmpreg1 |= DFSDM_SCDThreshold;

    /* Write to DFSDM Channelx AWSCDR */
    DFSDM_Channelx -> CHAWSCDR = tmpreg1;
}

/**
  * @brief  Selects the channel to be guarded by the Analog watchdog for the selected DFSDMx,
  *         and select if the fast analog watchdog is enabled or not.
  * @param  DFSDMx: where x can be a value from 0 to 1 to select the DFSDM Module.
  * @param  DFSDM_AWDChannelx: where x can be a value from 0 to 7 to select the DFSDM Channel.
  * @param  DFSDM_AWDFastMode: The analog watchdog fast mode.
  *         This parameter can be a value of @ref DFSDM_AWD_Fast_Mode_Selection.
  * @retval None
  */
void DFSDM_ConfigAnalogWatchdog(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_AWDChannelx, uint32_t DFSDM_AWDFastMode)
{
  uint32_t tmpreg1 = 0;
  uint32_t tmpreg2 = 0;

  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));
  assert_param(IS_DFSDM_AWD_CHANNEL(DFSDM_AWDChannelx));
  assert_param(IS_DFSDM_AWD_MODE(DFSDM_AWDFastMode));

  /* Get the DFSDMx CR2 value */
  tmpreg1 = DFSDMx -> FLTCR2;

  /* Clear the AWDCH bits */
  tmpreg1 &= ~(DFSDM_FLTCR2_AWDCH);

  /* Set or Reset the AWDCH bits */
  tmpreg1 |= DFSDM_AWDChannelx;

  /* Write to DFSDMx CR2 Register */
  DFSDMx -> FLTCR2 |= tmpreg1;

  /* Get the DFSDMx CR1 value */
  tmpreg2 = DFSDMx->FLTCR1;

  /* Clear the AWFSEL bit */
  tmpreg2 &= ~(DFSDM_FLTCR1_AWFSEL);

  /* Set or Reset the AWFSEL bit */
  tmpreg2 |= DFSDM_AWDFastMode;

  /* Write to DFSDMx CR1 Register */
  DFSDMx->FLTCR1 = tmpreg2;
}

/**
  * @brief  Selects the channel to be guarded by the Analog watchdog of the selected DFSDMx, and the mode to be used.
  * @param  DFSDMx: where x can be a value from 0 to 1 to select the DFSDM Module.
  * @param  DFSDM_ExtremChannelx: where x can be a value from 0 to 7 to select the Channel to be connected
  *         to the Extremes detector.
  * @retval None
  */
void DFSDM_SelectExtremesDetectorChannel(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_ExtremChannelx)
{
  uint32_t tmpreg1 = 0;

  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));
  assert_param(IS_DFSDM_EXTREM_CHANNEL(DFSDM_ExtremChannelx));

  /* Get the DFSDMx CR2 value */
  tmpreg1 = DFSDMx -> FLTCR2;

  /* Clear the EXCH bits */
  tmpreg1 &= ~(DFSDM_FLTCR2_EXCH);

  /* Set or Reset the AWDCH bits */
  tmpreg1 |= DFSDM_ExtremChannelx;

  /* Write to DFSDMx CR2 Register */
  DFSDMx -> FLTCR2 = tmpreg1;
}

/**
  * @brief  Returns the regular conversion data by the DFSDMx.
  * @param  DFSDMx: where x can be a value from 0 to 1 to select the DFSDM Module.
  * @retval The converted regular data.
  * @note   This function returns a signed value.
  */
int32_t DFSDM_GetRegularConversionData(DFSDM_TypeDef* DFSDMx)
{
  uint32_t reg = 0;
  int32_t  value = 0;
  
  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));
  
  /* Get value of data register for regular channel */
  reg = DFSDMx -> FLTRDATAR;
  
  /* Extract conversion value */
  value = ((int32_t)((reg & 0xFFFFFF00) >> 8));
  
  /* Return the conversion result */
  return  value;
}

/**
  * @brief  Returns the injected conversion data by the DFSDMx.
  * @param  DFSDMx: where x can be a value from 0 to 1 to select the DFSDM Module.
  * @retval The converted regular data.
  * @note   This function returns a signed value.
  */
int32_t DFSDM_GetInjectedConversionData(DFSDM_TypeDef* DFSDMx)
{
  uint32_t reg = 0;
  int32_t  value = 0;
  
  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));
  
  /* Get value of data register for regular channel */
  reg = DFSDMx -> FLTJDATAR;
  
  /* Extract conversion value */
  value = ((int32_t)((reg & 0xFFFFFF00) >> 8));
  
  /* Return the conversion result */
  return  value;
}

/**
  * @brief  Returns the highest value converted by the DFSDMx.
  * @param  DFSDMx: where x can be a value from 0 to 1 to select the DFSDM Module.
  * @retval The highest converted value.
  * @note   This function returns a signed value.
  */
int32_t DFSDM_GetMaxValue(DFSDM_TypeDef* DFSDMx)
{
  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));

  /* Return the highest converted value */
  return  (((int32_t)(DFSDMx -> FLTEXMAX)) >> 8);
}

/**
  * @brief  Returns the lowest value converted by the DFSDMx.
  * @param  DFSDMx: where x can be a value from 0 to 1 to select the DFSDM Module.
  * @retval The lowest converted value.
  * @note   This function returns a signed value.
  */
int32_t DFSDM_GetMinValue(DFSDM_TypeDef* DFSDMx)
{
  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));

  /* Return the lowest conversion value */
  return  (((int32_t)(DFSDMx ->FLTEXMIN )) >> 8);
}

/**
  * @brief  Returns the number of channel on which is captured the highest converted data by the DFSDMx.
  * @param  DFSDMx: where x can be a value from 0 to 1 to select the DFSDM Module.
  * @retval The highest converted value.
  */
int32_t DFSDM_GetMaxValueChannel(DFSDM_TypeDef* DFSDMx)
{
  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));

  /* Return the highest converted value */
  return  ((DFSDMx -> FLTEXMAX) & (~DFSDM_FLTEXMAX_EXMAXCH));
}

/**
  * @brief  Returns the number of channel on which is captured the lowest converted data by the DFSDMx.
  * @param  DFSDMx: where x can be a value from 0 to 1 to select the DFSDM Module.
  * @retval The lowest converted value.
  */
int32_t DFSDM_GetMinValueChannel(DFSDM_TypeDef* DFSDMx)
{
  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));

  /* Return the lowest converted value */
  return  ((DFSDMx -> FLTEXMIN) & (~DFSDM_FLTEXMIN_EXMINCH));
}

/**
  * @brief  Returns the conversion time (in 28-bit timer unit) for DFSDMx.
  * @param  DFSDMx: where x can be a value from 0 to 1 to select the DFSDM Module.
  * @retval Conversion time.
  */
uint32_t DFSDM_GetConversionTime(DFSDM_TypeDef* DFSDMx)
{
  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));

  /* Return the lowest converted value */
  return  ((DFSDMx -> FLTCNVTIMR >> 4) & 0x0FFFFFFF);
}

/**
  * @brief  Configures Sinc Filter for the Analog watchdog by setting
  *         the Sinc filter order and the Oversampling ratio for the specified DFSDM_Channelx.
  * @param  DFSDM_Channelx: where x can be a value from 0 to 7 to select the DFSDM Channel.
  * @param  DFSDM_AWDSincOrder: The Sinc Filter order this parameter can be a value of @ref DFSDM_AWD_Sinc_Order.
  * @param  DFSDM_AWDSincOverSampleRatio: The Filter Oversampling ratio, this parameter can be a value between 1 and 32.
  * @retval None
  */
void DFSDM_ConfigAWDFilter(DFSDM_Channel_TypeDef* DFSDM_Channelx, uint32_t DFSDM_AWDSincOrder, uint32_t DFSDM_AWDSincOverSampleRatio)
{
    uint32_t tmpreg1 = 0;

    /* Check the parameters */
    assert_param(IS_DFSDM_ALL_CHANNEL(DFSDM_Channelx));
    assert_param(IS_DFSDM_AWD_SINC_ORDER(DFSDM_AWDSincOrder));
    assert_param(IS_DFSDM_AWD_OVRSMPL_RATIO(DFSDM_AWDSincOverSampleRatio));

    /* Get the DFSDM_Channelx CHAWSCDR value */
    tmpreg1 = DFSDM_Channelx -> CHAWSCDR;

    /* Clear the FORD and FOSR bits */
    tmpreg1 &= ~(DFSDM_CHAWSCDR_AWFORD | DFSDM_CHAWSCDR_AWFOSR);

    /* Set or Reset the SCDT bits */
    tmpreg1 |= (DFSDM_AWDSincOrder | ((DFSDM_AWDSincOverSampleRatio -1) << 16)) ;

    /* Write to DFSDM Channelx CHAWSCDR */
    DFSDM_Channelx -> CHAWSCDR = tmpreg1;
}

/**
  * @brief  Returns the last Analog Watchdog Filter conversion result data for channelx.
  * @param  DFSDM_Channelx: where x can be a value from 0 to 7 to select the DFSDM Channel.
  * @retval The Data conversion value.
  */
uint32_t DFSDM_GetAWDConversionValue(DFSDM_Channel_TypeDef* DFSDM_Channelx)
{
  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_CHANNEL(DFSDM_Channelx));

  /* Return the last analog watchdog filter conversion value */
  return  DFSDM_Channelx -> CHWDATAR;
}


/**
  * @brief  Configures the High Threshold and the Low threshold for the Analog watchdog of the selected DFSDMx.
  * @param  DFSDM_HighThreshold: High threshold value. This parameter can be value between 0 and 0xFFFFFF.
  * @param  DFSDM_LowThreshold: Low threshold value. This parameter can be value between 0 and 0xFFFFFF.
  * @retval None.
  * @note   In case of channels transceivers monitoring (Analog Watchdog fast mode Enabled)),
  *         only the higher 16 bits define the 16-bit threshold compared with analog watchdog filter output.
  */

void DFSDM_SetAWDThreshold(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_HighThreshold, uint32_t DFSDM_LowThreshold)
{
    uint32_t tmpreg1 = 0;
    uint32_t tmpreg2 = 0;

    /* Check the parameters */
    assert_param(IS_DFSDM_HIGH_THRESHOLD(DFSDM_HighThreshold));
    assert_param(IS_DFSDM_LOW_THRESHOLD(DFSDM_LowThreshold));

    /* Get the DFSDMx AWHTR value */
    tmpreg1 = DFSDMx -> FLTAWHTR;

    /* Clear the AWHT bits */
    tmpreg1 &= ~(DFSDM_FLTAWHTR_AWHT);

    /* Set or Reset the AWHT bits */
    tmpreg1 |= (DFSDM_HighThreshold  << 8 );

    /* Write to DFSDMx AWHTR Register */
    DFSDMx -> FLTAWHTR = tmpreg1;

    /* Get the DFSDMx AWLTR value */
    tmpreg2 = DFSDMx -> FLTAWLTR;

    /* Clear the AWLTR bits */
    tmpreg2 &= ~(DFSDM_FLTAWLTR_AWLT);

    /* Set or Reset the AWLTR bits */
    tmpreg2 |= (DFSDM_LowThreshold  << 8 );

    /* Write to DFSDMx AWLTR Register */
    DFSDMx -> FLTAWLTR = tmpreg2;
}

/**
  * @brief  Selects the injected channel for the selected DFSDMx.
  * @param  DFSDMx: where x can be a value from 0 to 1 to select the DFSDM Module.
  * @param  DFSDM_InjectedChannelx: where x can be a value from 0 to 7 to select the Channel to be configuraed as
  *         injected channel.
  * @retval None
  * @note   User can select up to 8 channels.
  */
void DFSDM_SelectInjectedChannel(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_InjectedChannelx)
{
  uint32_t tmpreg1 = 0;

  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));
  assert_param(IS_DFSDM_INJECT_CHANNEL(DFSDM_InjectedChannelx));

  /* Get the DFSDMx JCHGR value */
  tmpreg1 = DFSDMx -> FLTJCHGR;

  /* Clear the JCHGR bits */
  tmpreg1 &= ~(DFSDM_FLTJCHGR_JCHG);

  /* Set or Reset the JCHGR bits */
  tmpreg1 |= DFSDM_InjectedChannelx;

  /* Write to DFSDMx JCHGR Register */
  DFSDMx -> FLTJCHGR |= tmpreg1;
}

/**
  * @brief  Selects the regular channel for the selected DFSDMx.
  * @param  DFSDMx: where x can be a value from 0 to 1 to select the DFSDM Module.
  * @param  DFSDM_RegularChannelx: where x can be a value from 0 to 7 to select the Channel to be configurated as
  *         regular channel.
  * @retval None
  * @note   User can select only one channel.
  */
void DFSDM_SelectRegularChannel(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_RegularChannelx)
{
  uint32_t tmpreg1 = 0;

  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));
  assert_param(IS_DFSDM_REGULAR_CHANNEL(DFSDM_RegularChannelx));

  /* Get the DFSDMx CR1 value */
  tmpreg1 = DFSDMx -> FLTCR1;

  /* Clear the RCH bits */
  tmpreg1 &= ~(DFSDM_FLTCR1_RCH);

  /* Set or Reset the RCH bits */
  tmpreg1 |= DFSDM_RegularChannelx;

  /* Write to DFSDMx CR1 Register */
  DFSDMx -> FLTCR1 = tmpreg1;
}

/**
  * @brief  Starts a software start for the injected group of channels of the selected DFSDMx.
  * @param  DFSDMx: where x can be a value from 0 to 1 to select the DFSDM Module.
  * @retval None
  */
void DFSDM_StartSoftwareInjectedConversion(DFSDM_TypeDef* DFSDMx)
{
  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));

  /* Write 1 to DFSDMx CR1 RSWSTAR bit */
  DFSDMx -> FLTCR1 |=  DFSDM_FLTCR1_JSWSTART;
}

/**
  * @brief  Starts a software start of the regular channel of the selected DFSDMx.
  * @param  DFSDMx: where x can be a value from 0 to 1 to select the DFSDM Module.
  * @retval None
  */
void DFSDM_StartSoftwareRegularConversion(DFSDM_TypeDef* DFSDMx)
{
  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));

  /* Write 1 to DFSDMx CR1 RSWSTAR bit */
  DFSDMx -> FLTCR1 |=  DFSDM_FLTCR1_RSWSTART;
}

/**
  * @brief  Selects the Trigger signal to launch the injected conversions of the selected DFSDMx.
  * @param  DFSDMx: where x can be a value from 0 to 1 to select the DFSDM Module.
  * @param  DFSDM_InjectedTrigger: the trigger signal.
  *         This parameter can be a value of: @ref DFSDM_Injected_Trigger_signal
  * @param  DFSDM_TriggerEdge: the edge of the selected trigger
  *         This parameter can be a value of: @ref DFSDM_Trigger_Edge_selection
  * @retval None.
  * @note   This function can be used only when the filter is disabled, use DFSDM_FilterCmd()
  *         to disable the filter.
  */
void DFSDM_ConfigInjectedTrigger(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_Trigger, uint32_t DFSDM_TriggerEdge)
{
  uint32_t tmpreg1 = 0;

  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));

  if (DFSDMx == DFSDM0)
  {
    assert_param(IS_DFSDM0_INJ_TRIGGER(DFSDM_Trigger));
  }
  else
  {
    assert_param(IS_DFSDM1_INJ_TRIGGER(DFSDM_Trigger));
  }

  assert_param(IS_DFSDM_TRIGGER_EDGE(DFSDM_TriggerEdge));

  /* Get the DFSDMx CR1 value */
  tmpreg1 = DFSDMx -> FLTCR1;

  /* Clear the JEXTSEL & JEXTEN bits */
  tmpreg1 &= ~(DFSDM_FLTCR1_JEXTSEL | DFSDM_FLTCR1_JEXTEN);

  /* Set or Reset the JEXTSEL & JEXTEN bits */
  tmpreg1 |= (DFSDM_Trigger | DFSDM_TriggerEdge);

  /* Write to DFSDMx CR1 Register */
  DFSDMx -> FLTCR1 = tmpreg1;
}

/**
  * @brief  Starts an injected conversion synchronously when in DFSDM0
  *         an injected conversion started by software.
  * @param  DFSDMx: where x can be 0 or 1 to select the DFSDM Module.
  * @retval None
  * @note   This function can be used only when the filter is disabled, use DFSDM_FilterCmd()
  *         to disable the filter.
  */
void DFSDM_SynchronousFilter0InjectedStart(DFSDM_TypeDef* DFSDMx)
{
  /* Check the parameters */
  assert_param(IS_DFSDM_SYNC_FILTER(DFSDMx));

  /* Write 1 to DFSDMx CR1 JSYNC bit */
  DFSDMx -> FLTCR1 |=  DFSDM_FLTCR1_JSYNC;
}

/**
  * @brief  Starts a regular conversion synchronously when in DFSDM0
  *         a regular conversion started by software.
  * @param  DFSDMx: where x can be 0 or 1 to select the DFSDM Module.
  * @retval None
  * @note   This function can be used only when the filter is disabled, use DFSDM_FilterCmd()
  *         to disable the filter.
  */
void DFSDM_SynchronousFilter0RegularStart(DFSDM_TypeDef* DFSDMx)
{
  /* Check the parameters */
  assert_param(IS_DFSDM_SYNC_FILTER(DFSDMx));

  /* Write 1 to DFSDMx CR1 RSYNC bit */
  DFSDMx -> FLTCR1 |=  DFSDM_FLTCR1_RSYNC;
}

/**
  * @brief  Enables or Disables the continue mode for Regular conversion for the selected filter DFSDMx.
  * @param  DFSDMx: where x can be 0 or 1 to select the DFSDM Module.
  * @param  NewState: new state of the Continuous mode.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DFSDM_RegularContinuousModeCmd(DFSDM_TypeDef* DFSDMx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

 if (NewState != DISABLE)
  {
    /* Enable the RCONT bit */
    DFSDMx -> FLTCR1 |=  DFSDM_FLTCR1_RCONT;
  }
  else
  {
    /* Disable the RCONT bit */
    DFSDMx -> FLTCR1 &=  ~(DFSDM_FLTCR1_RCONT);
  }
}

/**
  * @brief  Enables or Disables the Fast mode for the selected filter DFSDMx.
  * @param  DFSDMx: where x can be 0 or 1 to select the DFSDM Module.
  * @param  NewState: new state of the Fast mode.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  * @note   If just a single channel is selected in continuous mode (either by executing a regular
  *         conversion or by executing a injected conversion with only one channel selected),
  *         the sampling rate can be increased several times by enabling the fast mode.
  * @note   This function can be used only when the filter is disabled, use DFSDM_FilterCmd()
  *         to disable the filter.
  */
void DFSDM_FastModeCmd(DFSDM_TypeDef* DFSDMx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

 if (NewState != DISABLE)
  {
    /* Enable the FAST bit */
    DFSDMx -> FLTCR1 |=  DFSDM_FLTCR1_FAST;
  }
  else
  {
    /* Disable the FAST bit */
    DFSDMx -> FLTCR1 &=  ~(DFSDM_FLTCR1_FAST);
  }
}

/**
  * @brief  Selects the injected conversions mode for the selected DFSDMx.
  *         Injected conversions can operates in Single mode or Scan mode.
  * @param  DFSDMx: where x can be a value from 0 to 1 to select the DFSDM Module.
  * @param  DFSDM_InjectConvMode: The injected conversion mode, this parameter can be:
  *     @arg DFSDM_InjectConvMode_Single
  *     @arg DFSDM_InjectConvMode_Scan
  * @retval None.
  * @note   This function can be used only when the filter is disabled, use DFSDM_FilterCmd()
  *         to disable the filter.
  */
void DFSDM_SelectInjectedConversionMode(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_InjectConvMode)
{
  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));
  assert_param(IS_DFSDM_INJ_CONV_MODE(DFSDM_InjectConvMode));

  /* Clear the JSCAN bit */
  DFSDMx -> FLTCR1 &= ~(DFSDM_FLTCR1_JSCAN);

  /* Write to DFSDMx CR1 Register */
  DFSDMx -> FLTCR1 |= DFSDM_InjectConvMode;
}

/**
  * @brief  Enables or Disables the DMA to read data for the injected channel group of the selected filter DFSDMx.
  * @param  DFSDMx: where x can be 0 or 1 to select the DFSDM Module.
  * @param  DFSDM_DMAConversionMode: Selects the mode to be configured for DMA read  .
  *            @arg DFSDM_DMAConversionMode_Regular:  DMA channel Enabled/Disabled to read data for the regular conversion
  *            @arg DFSDM_DMAConversionMode_Injected: DMA channel Enabled/Disabled to read data for the Injected conversion
* @param  NewState: new state of the DMA channel.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None.
  * @note   This function can be used only when the filter is disabled, use DFSDM_FilterCmd()
  *         to disable the filter.
  */
void DFSDM_DMATransferConfig(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_DMAConversionMode, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));
  assert_param(IS_DFSDM_CONVERSION_MODE(DFSDM_DMAConversionMode));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

 if (NewState != DISABLE)
  {
    /* Enable the JDMAEN or RDMAEN bit */
    DFSDMx -> FLTCR1 |=  (DFSDM_FLTCR1_JDMAEN << DFSDM_DMAConversionMode) ;
  }
  else
  {
    /* Disable the JDMAEN or RDMAEN bit */
    DFSDMx -> FLTCR1 &=  ~(DFSDM_FLTCR1_JDMAEN << DFSDM_DMAConversionMode);
  }
}

/** @defgroup DFSDM_Group3 Interrupts and flags management functions
 *  @brief    Interrupts and flags management functions
 *
@verbatim
 ===============================================================================
                  Interrupts and flags management functions
 ===============================================================================
  This section provides functions allowing to configure the DFSDM Interrupts, get
  the status and clear flags bits.

  The LPT provides 7 Flags and Interrupts sources (2 flags and Interrupt sources
  are available only on LPT peripherals equipped with encoder mode interface)

  Flags and Interrupts sources:
  =============================
  1. End of injected conversion.
  2. End of regular conversion.
  3. Injected data overrun.
  4. Regular data overrun.
  5. Analog watchdog.
  6. Short circuit detector.
  7. Channel clock absence

  - To enable a specific interrupt source, use "DFSDM_ITConfig",
    "DFSDM_ITClockAbsenceCmd" and "DFSDM_ITShortCircuitDetectorCmd" functions.
  - To check if an interrupt was occurred, call "DFSDM_GetITStatus","DFSDM_GetClockAbsenceITStatusfunction"
    and "DFSDM_GetGetShortCircuitITStatus" functions and read returned values.
  - To get a flag status, call the "DFSDM_GetFlagStatus" ,"DFSDM_GetClockAbsenceFlagStatus" ,"DFSDM_GetShortCircuitFlagStatus"
    and "DFSDM_GetWatchdogFlagStatus" functions and read the returned value.
  - To clear a flag or an interrupt, use DFSDM_ClearFlag,DFSDM_ClearClockAbsenceFlag,
    DFSDM_ClearShortCircuitFlag,DFSDM_ClearAnalogWatchdogFlag functions with the
    corresponding flag (interrupt).
 
@endverbatim
  * @{
  */

/**
  * @brief  Enables or disables the specified DFSDMx interrupts.
  * @param  DFSDMx: where x can be 0 or 1 to select the DFSDM peripheral.
  * @param  DFSDM_IT: specifies the DFSDM interrupts sources to be enabled or disabled.
  *         This parameter can be any combination of the following values:
  *            @arg DFSDM_IT_JEOC: End of injected conversion Interrupt source
  *            @arg DFSDM_IT_REOC: End of regular conversion Interrupt source
  *            @arg DFSDM_IT_JOVR: Injected data overrun Interrupt source
  *            @arg DFSDM_IT_ROVR: Regular data overrun Interrupt source
  *            @arg DFSDM_IT_AWD : Analog watchdog Interrupt source
  * @param  NewState: new state of the DFSDM interrupts.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DFSDM_ITConfig(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_IT, FunctionalState NewState)
 {
  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));
  assert_param(IS_DFSDM_IT(DFSDM_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the Interrupt sources */
    DFSDMx->FLTCR2 |= DFSDM_IT;
  }
  else
  {
    /* Disable the Interrupt sources */
    DFSDMx->FLTCR2 &= ~(DFSDM_IT);
  }
}

/**
  * @brief  Enables or disables the Clock Absence Interrupt.
  * @param  NewState: new state of the interrupt.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DFSDM_ITClockAbsenceCmd(FunctionalState NewState)
 {
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the Interrupt source */
    DFSDM0->FLTCR2 |= DFSDM_IT_CKAB;
  }
  else
  {
    /* Disable the Interrupt source */
    DFSDM0->FLTCR2 &= ~(DFSDM_IT_CKAB);
  }
}

/**
  * @brief  Enables or disables the Short Circuit Detector Interrupt.
  * @param  NewState: new state of the interrupt.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DFSDM_ITShortCircuitDetectorCmd(FunctionalState NewState)
 {
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the Interrupt source */
    DFSDM0->FLTCR2 |= DFSDM_IT_SCD;
  }
  else
  {
    /* Disable the Interrupt source */
    DFSDM0->FLTCR2 &= ~(DFSDM_IT_SCD);
  }
}

/**
  * @brief  Checks whether the specified DFSDM flag is set or not.
  * @param  DFSDMx: where x can be 0 or 1 to select the DFSDM peripheral.
  * @param  LPT_FLAG: specifies the flag to check.
  *         This parameter can be any combination of the following values:
  *            @arg DFSDM_FLAG_JEOC: End of injected conversion Flag
  *            @arg DFSDM_FLAG_REOC: End of regular conversion Flag
  *            @arg DFSDM_FLAG_JOVR: Injected data overrun Flag
  *            @arg DFSDM_FLAG_ROVR: Regular data overrun Flag
  *            @arg DFSDM_FLAG_AWD:  Analog watchdog Flag
  *            @arg DFSDM_FLAG_JCIP: Injected conversion in progress status
  *            @arg DFSDM_FLAG_RCIP: Regular conversion in progress status
  * @retval None
  */
FlagStatus DFSDM_GetFlagStatus(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_FLAG)
{
  ITStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));
  assert_param(IS_DFSDM_FLAG(DFSDM_FLAG));

  if ((DFSDMx->FLTISR & DFSDM_FLAG) != RESET )
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/**
  * @brief  Checks whether the specified Clock Absence Channel flag is set or not.
  * @param  DFSDM_FLAG_CLKAbsence: specifies the flag to check.
  *         This parameter can be a value of @ref DFSDM_Clock_Absence_Flag_Definition
  * @retval None
  */
FlagStatus DFSDM_GetClockAbsenceFlagStatus(uint32_t DFSDM_FLAG_CLKAbsence)
{
  ITStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_DFSDM_CLK_ABS_FLAG(DFSDM_FLAG_CLKAbsence));

  if ((DFSDM0->FLTISR & DFSDM_FLAG_CLKAbsence) != RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/**
  * @brief  Checks whether the specified Short Circuit Channel Detector flag is set or not.
  * @param  DFSDM_FLAG_SCD: specifies the flag to check.
  *         This parameter can be a value of @ref DFSDM_SCD_Flag_Definition
  * @retval None
  */
FlagStatus DFSDM_GetShortCircuitFlagStatus(uint32_t DFSDM_FLAG_SCD)
{
  ITStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_DFSDM_SCD_FLAG(DFSDM_FLAG_SCD));

  if ((DFSDM0->FLTISR & DFSDM_FLAG_SCD) != RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/**
  * @brief  Checks whether the specified Watchdog threshold flag is set or not.
  * @param  DFSDMx: where x can be 0 or 1 to select the DFSDM peripheral. 
  * @param  DFSDM_AWDChannelx: where x can be a value from 0 to 7 to select the DFSDM Channel.
  * @param  DFSDM_Threshold: specifies the Threshold.
  *         This parameter can be a value of @ref DFSDM_Threshold_Selection.
  * @retval None
  */
FlagStatus DFSDM_GetWatchdogFlagStatus(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_AWDChannelx, uint8_t DFSDM_Threshold)
{
  ITStatus bitstatus = RESET;

  /* Check the parameters */  
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));
  assert_param(IS_DFSDM_Threshold(DFSDM_Threshold));
  assert_param(IS_DFSDM_AWD_CHANNEL(DFSDM_AWDChannelx));

  if ((DFSDMx->FLTAWSR & ((DFSDM_AWDChannelx >> 16) << DFSDM_Threshold) ) != RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/**
  * @brief  Clears the DFSDMx's pending flag.
  * @param  DFSDMx: where x can be 0 or 1 to select the DFSDM peripheral.
  * @param  DFSDM_CLEARF: specifies the pending bit to clear.
  *         This parameter can be any combination of the following values:
  *            @arg DFSDM_CLEARF_JOVR: Injected data overrun Clear Flag
  *            @arg DFSDM_CLEARF_ROVR: Regular data overrun Clear Flag
  * @retval None
  */
void DFSDM_ClearFlag(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_CLEARF)
{
  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));
  assert_param(IS_DFSDM_CLEAR_FLAG(DFSDM_CLEARF));

  /* Clear the pending Flag Bit */
  DFSDMx->FLTICR |= DFSDM_CLEARF;
}

/**
  * @brief  Clears the DFSDMx's pending Clock Absence Channel flag.
  * @param  DFSDM_CLEARF_CLKAbsence: specifies the pending bit to clear.
  *         This parameter can be any combination of @ref DFSDM_Clear_ClockAbs_Flag_Definition
  * @retval None
  */
void DFSDM_ClearClockAbsenceFlag(uint32_t DFSDM_CLEARF_CLKAbsence)
{
  /* Check the parameters */
  assert_param(IS_DFSDM_CLK_ABS_CLEARF(DFSDM_CLEARF_CLKAbsence));

  /* Clear the IT pending Flag Bit */
  DFSDM0->FLTICR |= DFSDM_CLEARF_CLKAbsence;
}

/**
  * @brief  Clears the DFSDMx's pending Short circuit Channel flag.
  * @param  DFSDM_CLEARF_SCD: specifies the pending bit to clear.
  *         This parameter can be any combination of @ref DFSDM_Clear_Short_Circuit_Flag_Definition
  * @retval None
  */
void DFSDM_ClearShortCircuitFlag(uint32_t DFSDM_CLEARF_SCD)
{
  /* Check the parameters */
  assert_param(IS_DFSDM_SCD_CHANNEL_FLAG(DFSDM_CLEARF_SCD));

  /* Clear the pending Flag Bit */
  DFSDM0->FLTICR |= DFSDM_CLEARF_SCD;
}

/**
  * @brief  Clears the DFSDMx's pending Analog watchdog Channel flag.
  * @param  DFSDMx: where x can be 0 or 1 to select the DFSDM peripheral.
  * @param  DFSDM_AWDChannelx: where x can be a value from 0 to 7 to select the DFSDM Channel.
  * @param  DFSDM_Threshold: specifies the Threshold.
  *         This parameter can be a value of @ref DFSDM_Threshold_Selection.
  * @retval None
  */
void DFSDM_ClearAnalogWatchdogFlag(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_AWDChannelx, uint8_t DFSDM_Threshold)
{
  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));  
  assert_param(IS_DFSDM_Threshold(DFSDM_Threshold));
  assert_param(IS_DFSDM_AWD_CHANNEL(DFSDM_AWDChannelx));

  if ((DFSDMx->FLTAWSR & ((DFSDM_AWDChannelx >> 16) << DFSDM_Threshold) ) != RESET)

  /* Clear the pending Flag Bit */
  DFSDMx->FLTAWCFR |= (DFSDM_AWDChannelx >> 16) << DFSDM_Threshold;
}

/**
  * @brief  Check whether the specified DFSDM interrupt has occurred or not.
  * @param  DFSDMx: where x can be 0 or 1 to select the DFSDM peripheral.
  * @param  DFSDM_IT: specifies the DFSDM interrupt source to check.
  *            @arg DFSDM_IT_JEOC: End of injected conversion Interrupt source
  *            @arg DFSDM_IT_REOC: End of regular conversion Interrupt source
  *            @arg DFSDM_IT_JOVR: Injected data overrun Interrupt source
  *            @arg DFSDM_IT_ROVR: Regular data overrun Interrupt source
  *            @arg DFSDM_IT_AWD : Analog watchdog Interrupt source
  * @retval The new state of DFSDM_IT (SET or RESET).
  */
ITStatus DFSDM_GetITStatus(DFSDM_TypeDef* DFSDMx, uint32_t DFSDM_IT)
{
  ITStatus bitstatus = RESET;
  uint32_t itstatus = 0x0, itenable = 0x0;

  /* Check the parameters */
  assert_param(IS_DFSDM_ALL_FILTER(DFSDMx));
  assert_param(IS_DFSDM_IT(DFSDM_IT));

  /* Get the Interrupt Status bit value */
  itstatus = DFSDMx->FLTISR & DFSDM_IT;

  /* Check if the Interrupt is enabled */
  itenable = DFSDMx->FLTCR2 & DFSDM_IT;

  if ((itstatus != RESET) && (itenable != RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/**
  * @brief  Check whether the specified Clock Absence channel interrupt has occurred or not.
  * @param  DFSDM_IT_CLKAbsence: specifies on which channel check the interrupt source.
  *         This parameter can be a value of @ref DFSDM_Clock_Absence_Interrupt_Definition.
  * @retval The new state of DFSDM_IT (SET or RESET).
  * @note   Clock absence interrupt is handled only by DFSDM0.
  */
ITStatus DFSDM_GetClockAbsenceITStatus(uint32_t DFSDM_IT_CLKAbsence)
{
  ITStatus bitstatus = RESET;
  uint32_t itstatus = 0x0, itenable = 0x0;

  /* Check the parameters */
  assert_param(IS_DFSDM_CLK_ABS_IT(DFSDM_IT_CLKAbsence));

  /* Get the Interrupt Status bit value */
  itstatus = DFSDM0->FLTISR & DFSDM_IT_CLKAbsence;

  /* Check if the Interrupt is enabled */
  itenable = DFSDM0->FLTCR2 & DFSDM_IT_CKAB;

  if ((itstatus != RESET) && (itenable != RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/**
  * @brief  Check whether the specified Short Circuit channel interrupt has occurred or not.
  * @param  DFSDM_IT_SCR: specifies on which channel check the interrupt source.
  *         This parameter can be a value of @ref DFSDM_SCD_Interrupt_Definition.
  * @retval The new state of DFSDM_IT (SET or RESET).
  * @note   Short circuit interrupt is handled only by DFSDM0.
  */
ITStatus DFSDM_GetGetShortCircuitITStatus(uint32_t DFSDM_IT_SCR)
{
  ITStatus bitstatus = RESET;
  uint32_t itstatus = 0x0, itenable = 0x0;

  /* Check the parameters */
  assert_param(IS_DFSDM_SCD_IT(DFSDM_IT_SCR));

  /* Get the Interrupt Status bit value */
  itstatus = DFSDM0->FLTISR & DFSDM_IT_SCR;

  /* Check if the Interrupt is enabled */
  itenable = DFSDM0->FLTCR2 & DFSDM_IT_SCD;

  if ((itstatus != RESET) && (itenable != RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/**
  * @}
  */

/**
  * @}
  */
#endif /* STM32F412xG */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
