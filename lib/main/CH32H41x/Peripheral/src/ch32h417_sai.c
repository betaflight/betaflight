/********************************** (C) COPYRIGHT
 * ******************************* File Name          : ch32h417_sai.c Author
 *          : WCH Version            : V1.0.0 Date               : 2025/03/01
 * Description        : This file provides all the SAI firmware functions.
 *********************************************************************************
 * Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "ch32h417_sai.h"
#include "ch32h417_rcc.h"

#define CFGR1_CLEAR_MASK ((uint32_t)0xFF07C010)
#define FRCR_CLEAR_MASK ((uint32_t)0xFFF88000)
#define SLOTR_CLEAR_MASK ((uint32_t)0x0000F020)

/*********************************************************************
 * @fn      SAI_DeInit
 *
 * @brief   Deinitialize the SAI peripheral registers to their default reset
 * values.
 *
 * @return  none
 */
void SAI_DeInit(void) {
  RCC_HB2PeriphResetCmd(RCC_HB2Periph_SAI, ENABLE);
  RCC_HB2PeriphResetCmd(RCC_HB2Periph_SAI, DISABLE);
}

/*********************************************************************
 * @fn      SAI_Init
 *
 * @brief   Initializes the SAI Block x peripheral according to the specified
 *        parameters in the SAI_InitStruct.
 *
 * @param   SAI_Block_x - where x can be A or B to select the SAI Block
 * peripheral. SAI_InitStruct - pointer to a SAI_InitTypeDef structure that
 *        contains the configuration information for the specified SAI Block
 * peripheral.
 *
 * @return  none
 */
void SAI_Init(SAI_Block_TypeDef *SAI_Block_x, SAI_InitTypeDef *SAI_InitStruct) {
  uint32_t tmpreg = 0;

  tmpreg = SAI_Block_x->CFGR1;

  tmpreg &= CFGR1_CLEAR_MASK;

  tmpreg |=
      (uint32_t)(SAI_InitStruct->SAI_AudioMode | SAI_InitStruct->SAI_Protocol |
                 SAI_InitStruct->SAI_DataSize | SAI_InitStruct->SAI_FirstBit |
                 SAI_InitStruct->SAI_ClockStrobing |
                 SAI_InitStruct->SAI_Synchro | SAI_InitStruct->SAI_OutDRIV |
                 SAI_InitStruct->SAI_NoDivider |
                 (uint32_t)((SAI_InitStruct->SAI_MasterDivider) << 20));

  SAI_Block_x->CFGR1 = tmpreg;

  tmpreg = SAI_Block_x->CFGR2;

  tmpreg &= ~(SAI_CFGR2_FTH);

  tmpreg |= (uint32_t)(SAI_InitStruct->SAI_FIFOThreshold);

  SAI_Block_x->CFGR2 = tmpreg;
}

/*********************************************************************
 * @fn      SAI_FrameInit
 *
 * @brief   Initializes the SAI Block Audio frame according to the specified
 *        parameters in the SAI_FrameInitStruct.
 *
 * @param   SAI_Block_x - where x can be A or B to select the SAI Block
 * peripheral. SAI_FrameInitStruct - pointer to an SAI_FrameInitTypeDef
 * structure that contains the configuration of audio frame for a specified SAI
 * Block
 *
 * @return  none
 */
void SAI_FrameInit(SAI_Block_TypeDef *SAI_Block_x,
                   SAI_FrameInitTypeDef *SAI_FrameInitStruct) {
  uint32_t tmpreg = 0;

  tmpreg = SAI_Block_x->FRCR;

  tmpreg &= FRCR_CLEAR_MASK;

  tmpreg |=
      (uint32_t)((uint32_t)(SAI_FrameInitStruct->SAI_FrameLength - 1) |
                 SAI_FrameInitStruct->SAI_FSOffset |
                 SAI_FrameInitStruct->SAI_FSDefinition |
                 SAI_FrameInitStruct->SAI_FSPolarity |
                 (uint32_t)((SAI_FrameInitStruct->SAI_ActiveFrameLength - 1)
                            << 8));

  SAI_Block_x->FRCR = tmpreg;
}

/*********************************************************************
 * @fn      SAI_SlotInit
 *
 * @brief   Initializes the SAI Block audio Slot according to the specified
 *        parameters in the SAI_SlotInitStruct.
 *
 * @param   SAI_Block_x - where x can be A or B to select the SAI Block
 * peripheral. SAI_SlotInitStruct - pointer to an SAI_SlotInitTypeDef structure
 * that contains the configuration of audio slot for a specified SAI Block
 *
 * @return  none
 */
void SAI_SlotInit(SAI_Block_TypeDef *SAI_Block_x,
                  SAI_SlotInitTypeDef *SAI_SlotInitStruct) {
  uint32_t tmpreg = 0;

  tmpreg = SAI_Block_x->SLOTR;

  tmpreg &= SLOTR_CLEAR_MASK;

  tmpreg |=
      (uint32_t)(SAI_SlotInitStruct->SAI_FirstBitOffset |
                 SAI_SlotInitStruct->SAI_SlotSize |
                 SAI_SlotInitStruct->SAI_SlotActive |
                 (uint32_t)((SAI_SlotInitStruct->SAI_SlotNumber - 1) << 8));

  SAI_Block_x->SLOTR = tmpreg;
}

/*********************************************************************
 * @fn      SAI_FrameStructInit
 *
 * @brief   Fills each SAI_FrameInitStruct member with its default value.
 *
 * @param   SAI_FrameInitStruct - pointer to a SAI_FrameInitTypeDef structure
 *        which will be initialized.
 *
 * @return  none
 */
void SAI_FrameStructInit(SAI_FrameInitTypeDef *SAI_FrameInitStruct) {
  SAI_FrameInitStruct->SAI_FrameLength = 8;

  SAI_FrameInitStruct->SAI_ActiveFrameLength = 1;

  SAI_FrameInitStruct->SAI_FSDefinition = SAI_FS_StartFrame;

  SAI_FrameInitStruct->SAI_FSPolarity = SAI_FS_ActiveLow;

  SAI_FrameInitStruct->SAI_FSOffset = SAI_FS_FirstBit;
}

/*********************************************************************
 * @fn      SAI_SlotStructInit
 *
 * @brief   Fills each SAI_SlotInitStruct member with its default value.
 *
 * @param   SAI_SlotInitStruct - pointer to a SAI_SlotInitTypeDef structure
 *        which will be initialized.
 *
 * @return  none
 */
void SAI_SlotStructInit(SAI_SlotInitTypeDef *SAI_SlotInitStruct) {
  SAI_SlotInitStruct->SAI_FirstBitOffset = 0;

  SAI_SlotInitStruct->SAI_SlotSize = SAI_SlotSize_DataSize;

  SAI_SlotInitStruct->SAI_SlotNumber = 1;

  SAI_SlotInitStruct->SAI_SlotActive = SAI_Slot_NotActive;
}

/*********************************************************************
 * @fn      SAI_Cmd
 *
 * @brief   Enables or disables the specified SAI Block peripheral.
 *
 * @param   SAI_Block_x - where x can be A or B to select the SAI Block
 * peripheral. NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void SAI_Cmd(SAI_Block_TypeDef *SAI_Block_x, FunctionalState NewState) {
  if (NewState != DISABLE) {
    SAI_Block_x->CFGR1 |= SAI_CFGR1_EN;
  } else {
    SAI_Block_x->CFGR1 &= ~(SAI_CFGR1_EN);
  }
}

/*********************************************************************
 * @fn      SAI_MonoModeConfig
 *
 * @brief   Configures the mono mode for the selected SAI block.
 *
 * @param   SAI_Block_x - where x can be A or B to select the SAI Block
 * peripheral. SAI_MonoMode - specifies the SAI block mono mode. This parameter
 * can be one of the following values: SAI_MonoMode - Set mono audio mode
 *              SAI_StreoMode - Set streo audio mode
 *
 * @return  none
 */
void SAI_MonoModeConfig(SAI_Block_TypeDef *SAI_Block_x,
                        uint32_t SAI_Mono_StreoMode) {
  SAI_Block_x->CFGR1 &= ~(SAI_CFGR1_MONO);

  SAI_Block_x->CFGR1 |= SAI_Mono_StreoMode;
}

/*********************************************************************
 * @fn      SAI_TRIStateConfig
 *
 * @brief   Configures the TRIState management on data line for the selected SAI
 * block.
 *
 * @param   SAI_Block_x - where x can be A or B to select the SAI Block
 * peripheral. SAI_TRIState - specifies the SAI block TRIState management. This
 * parameter can be one of the following values: SAI_Output_NotReleased - SD
 * output line is still driven by the SAI. SAI_Output_Released - SD output line
 * is released (HI-Z)
 *
 * @return  none
 */
void SAI_TRIStateConfig(SAI_Block_TypeDef *SAI_Block_x, uint32_t SAI_TRIState) {
  SAI_Block_x->CFGR1 &= ~(SAI_CFGR2_TRIS);

  SAI_Block_x->CFGR1 |= SAI_TRIState;
}

/*********************************************************************
 * @fn      SAI_CompandingModeConfig
 *
 * @brief   Configures the companding mode for the selected SAI block.
 *
 * @param   SAI_Block_x - where x can be A or B to select the SAI Block
 * peripheral. SAI_CompandingMode - specifies the SAI block companding mode.
 *          This parameter can be one of the following values:
 *              SAI_NoCompanding - no companding algorithm set
 *              SAI_ULaw_1CPL_Companding - Set U law (algorithm 1's complement
 * representation) SAI_ALaw_1CPL_Companding - Set A law (algorithm 1's
 * complement representation) SAI_ULaw_2CPL_Companding - Set U law (algorithm
 * 2's complement representation) SAI_ALaw_2CPL_Companding - Set A law
 * (algorithm 2's complement representation)
 *
 * @return  none
 */
void SAI_CompandingModeConfig(SAI_Block_TypeDef *SAI_Block_x,
                              uint32_t SAI_CompandingMode) {
  SAI_Block_x->CFGR2 &= ~(SAI_CFGR2_COMP);

  SAI_Block_x->CFGR2 |= SAI_CompandingMode;
}

/*********************************************************************
 * @fn      SAI_MuteModeCmd
 *
 * @brief   Enables or disables the Mute mode for the selected SAI block.
 *
 * @param   SAI_Block_x - where x can be A or B to select the SAI Block
 * peripheral. NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void SAI_MuteModeCmd(SAI_Block_TypeDef *SAI_Block_x, FunctionalState NewState) {
  if (NewState != DISABLE) {
    SAI_Block_x->CFGR2 |= SAI_CFGR2_MUTE;
  } else {
    SAI_Block_x->CFGR2 &= ~(SAI_CFGR2_MUTE);
  }
}

/*********************************************************************
 * @fn      SAI_MuteValueConfig
 *
 * @brief   Configure the mute value for the selected SAI block.
 *
 * @param   SAI_Block_x - where x can be A or B to select the SAI Block
 * peripheral. SAI_MuteValue - specifies the SAI block mute value. This
 * parameter can be one of the following values: SAI_ZeroValue - bit value 0 is
 * sent during Mute Mode SAI_LastSentValue - Last value is sent during Mute Mode
 *
 * @return  none
 */
void SAI_MuteValueConfig(SAI_Block_TypeDef *SAI_Block_x,
                         uint32_t SAI_MuteValue) {
  SAI_Block_x->CFGR2 &= ~(SAI_CFGR2_MUTEVAL);

  SAI_Block_x->CFGR2 |= SAI_MuteValue;
}

/*********************************************************************
 * @fn      SAI_MuteFrameCounterConfig
 *
 * @brief   Enables or disables the Mute mode for the selected SAI block.
 *
 * @param   SAI_Block_x - where x can be A or B to select the SAI Block
 * peripheral. SAI_MuteCounter - specifies the SAI block mute value. This
 * parameter can be a number between 0 and 63.
 *
 * @return  none
 */
void SAI_MuteFrameCounterConfig(SAI_Block_TypeDef *SAI_Block_x,
                                uint32_t SAI_MuteCounter) {
  SAI_Block_x->CFGR2 &= ~(SAI_CFGR2_MUTECNT);

  SAI_Block_x->CFGR2 |= (SAI_MuteCounter << 7);
}

/*********************************************************************
 * @fn      SAI_FlushFIFO
 *
 * @brief   Reinitialize the FIFO pointer
 *
 * @param   SAI_Block_x - where x can be A or B to select the SAI Block
 * peripheral. NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void SAI_FlushFIFO(SAI_Block_TypeDef *SAI_Block_x) {
  SAI_Block_x->CFGR2 |= SAI_CFGR2_FFLUSH;
  SAI_Block_x->CFGR2 &= ~SAI_CFGR2_FFLUSH;
}

/*********************************************************************
 * @fn      SAI_ReceiveData
 *
 * @brief   Returns the most recent received data by the SAI block x peripheral.
 *
 * @param   SAI_Block_x - where x can be A or B to select the SAI Block
 * peripheral.
 *
 * @return  The value of the received data.
 */
uint32_t SAI_ReceiveData(SAI_Block_TypeDef *SAI_Block_x) {
  return SAI_Block_x->DATAR;
}

/*********************************************************************
 * @fn      SAI_SendData
 *
 * @brief   Transmits a Data through the SAI block x peripheral.
 *
 * @param   SAI_Block_x - where x can be A or B to select the SAI Block
 * peripheral. Data - Data to be transmitted.
 *
 * @return  none
 */
void SAI_SendData(SAI_Block_TypeDef *SAI_Block_x, uint32_t Data) {
  SAI_Block_x->DATAR = Data;
}

/*********************************************************************
 * @fn      SAI_OSRCmd
 *
 * @brief   Enables or disables the SAI Block x OverSampling .
 *
 * @param   SAI_Block_x - where x can be A or B to select the SAI Block
 * peripheral. NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void SAI_OSRCmd(SAI_Block_TypeDef *SAI_Block_x, FunctionalState NewState) {
  if (NewState != DISABLE) {
    SAI_Block_x->CFGR1 |= SAI_CFGR1_OSR;
  } else {
    SAI_Block_x->CFGR1 &= ~(SAI_CFGR1_OSR);
  }
}

/*********************************************************************
 * @fn      SAI_DMACmd
 *
 * @brief   Enables or disables the SAI Block x DMA interface.
 *
 * @param   SAI_Block_x - where x can be A or B to select the SAI Block
 * peripheral. NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void SAI_DMACmd(SAI_Block_TypeDef *SAI_Block_x, FunctionalState NewState) {
  if (NewState != DISABLE) {
    SAI_Block_x->CFGR1 |= SAI_CFGR1_DMAEN;
  } else {
    SAI_Block_x->CFGR1 &= ~(SAI_CFGR1_DMAEN);
  }
}

/*********************************************************************
 * @fn      SAI_ITConfig
 *
 * @brief   Enables or disables the specified SAI Block interrupts.
 *
 * @param   SAI_Block_x - where x can be A or B to select the SAI Block
 * peripheral. SAI_IT - specifies the SAI interrupt source to be enabled or
 * disabled. This parameter can be one of the following values: SAI_IT_FREQ -
 * FIFO Request interrupt mask SAI_IT_MUTEDET - MUTE detection interrupt mask
 *              SAI_IT_OVRUDR - overrun/underrun interrupt mask
 *              SAI_IT_AFSDET - anticipated frame synchronization detection
 *                                interrupt mask
 *              SAI_IT_LFSDET - late frame synchronization detection interrupt
 *                                mask
 *              SAI_IT_CNRDY - codec not ready interrupt mask
 *              SAI_IT_WCKCFG - wrong clock configuration interrupt mask
 *
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void SAI_ITConfig(SAI_Block_TypeDef *SAI_Block_x, uint32_t SAI_IT,
                  FunctionalState NewState) {
  if (NewState != DISABLE) {
    SAI_Block_x->INTENR |= SAI_IT;
  } else {
    SAI_Block_x->INTENR &= ~(SAI_IT);
  }
}

/*********************************************************************
 * @fn      SAI_GetFlagStatus
 *
 * @brief   Checks whether the specified SAI block x flag is set or not.
 *
 * @param   SAI_Block_x - where x can be A or B to select the SAI Block
 * peripheral. SAI_FLAG - specifies the SAI block flag to check. This parameter
 * can be one of the following values: SAI_FLAG_FREQ - FIFO Request flag.
 *              SAI_FLAG_MUTEDET - MUTE detection flag.
 *              SAI_FLAG_OVRUDR - overrun/underrun flag.
 *              SAI_FLAG_WCKCFG - wrong clock configuration flag.
 *              SAI_FLAG_CNRDY - codec not ready flag.
 *              SAI_FLAG_AFSDET - anticipated frame synchronization detection
 * flag. SAI_FLAG_LFSDET - late frame synchronization detection flag.
 *
 * @return  The new state of SAI_FLAG (SET or RESET).
 */
FlagStatus SAI_GetFlagStatus(SAI_Block_TypeDef *SAI_Block_x,
                             uint32_t SAI_FLAG) {
  FlagStatus bitstatus = RESET;

  if ((SAI_Block_x->SR & SAI_FLAG) != (uint32_t)RESET) {
    bitstatus = SET;
  } else {
    bitstatus = RESET;
  }

  return bitstatus;
}

/*********************************************************************
 * @fn      SAI_ClearFlag
 *
 * @brief   Clears the specified SAI Block x flag.
 *
 * @param   SAI_Block_x - where x can be A or B to select the SAI Block
 * peripheral. SAI_FLAG - specifies the SAI block flag to check. This parameter
 * can be one of the following values: SAI_FLAG_MUTEDET - MUTE detection flag.
 *              SAI_FLAG_OVRUDR - overrun/underrun flag.
 *              SAI_FLAG_WCKCFG - wrong clock configuration flag.
 *              SAI_FLAG_CNRDY - codec not ready flag.
 *              SAI_FLAG_AFSDET - anticipated frame synchronization detection
 * flag. SAI_FLAG_LFSDET - late frame synchronization detection flag.
 *          - When the audio block is transmitter and the FIFO is full or the
 * FIFO has one data (one buffer mode) depending the bit FTH in the SAI_CFGR2
 * register.
 *          - When the audio block is receiver and the FIFO is not empty
 *
 * @return  none
 */
void SAI_ClearFlag(SAI_Block_TypeDef *SAI_Block_x, uint32_t SAI_FLAG) {
  SAI_Block_x->SR = SAI_FLAG;
}

/*********************************************************************
 * @fn      SAI_GetITStatus
 *
 * @brief   Checks whether the specified SAI Block x interrupt has occurred or
 * not.
 *
 * @param   SAI_Block_x - where x can be A or B to select the SAI Block
 * peripheral. SAI_IT - specifies the SAI interrupt source to be enabled or
 * disabled. This parameter can be one of the following values: SAI_IT_FREQ -
 * FIFO Request interrupt SAI_IT_MUTEDET - MUTE detection interrupt
 *              SAI_IT_OVRUDR - overrun/underrun interrupt
 *              SAI_IT_AFSDET - anticipated frame synchronization detection
 * interrupt SAI_IT_LFSDET - late frame synchronization detection interrupt
 *              SAI_IT_CNRDY - codec not ready interrupt
 *              SAI_IT_WCKCFG - wrong clock configuration interrupt
 *
 * @return  The new state of SAI_IT (SET or RESET).
 */
ITStatus SAI_GetITStatus(SAI_Block_TypeDef *SAI_Block_x, uint32_t SAI_IT) {
  ITStatus bitstatus = RESET;
  uint32_t enablestatus = 0;

  enablestatus = (SAI_Block_x->INTENR & SAI_IT);

  if (((SAI_Block_x->SR & SAI_IT) != (uint32_t)RESET) &&
      (enablestatus != (uint32_t)RESET)) {
    bitstatus = SET;
  } else {
    bitstatus = RESET;
  }

  return bitstatus;
}

/*********************************************************************
 * @fn      SAI_ClearITPendingBit
 *
 * @brief   Clears the SAI Block x interrupt pending bit.
 *
 * @param   SAI_Block_x - where x can be A or B to select the SAI Block
 * peripheral. SAI_IT - specifies the SAI Block interrupt pending bit to clear.
 *          This parameter can be one of the following values:
 *              SAI_IT_MUTEDET - MUTE detection interrupt.
 *              SAI_IT_OVRUDR - overrun/underrun interrupt.
 *              SAI_IT_WCKCFG - wrong clock configuration interrupt.
 *              SAI_IT_CNRDY - codec not ready interrupt.
 *              SAI_IT_AFSDET - anticipated frame synchronization detection
 * interrupt. SAI_IT_LFSDET - late frame synchronization detection interrupt.
 *          - When the audio block is transmitter and the FIFO is full or the
 * FIFO has one data (one buffer mode) depending the bit FTH in the SAI_CFGR2
 * register.
 *          - When the audio block is receiver and the FIFO is not empty
 *
 * @return  none
 */
void SAI_ClearITPendingBit(SAI_Block_TypeDef *SAI_Block_x, uint32_t SAI_IT) {
  SAI_Block_x->SR = SAI_IT;
}

/*********************************************************************
 * @fn      SAI_GetCmdStatus
 *
 * @brief   Returns the status of EN bit for the specified SAI Block x.
 *
 * @param   SAI_Block_x - where x can be A or B to select the SAI Block
 * peripheral.
 *
 * @return  Current state of the DMA (ENABLE or DISABLE).
 */
FunctionalState SAI_GetCmdStatus(SAI_Block_TypeDef *SAI_Block_x) {
  FunctionalState state = DISABLE;

  if ((SAI_Block_x->CFGR1 & (uint32_t)SAI_CFGR1_EN) != 0) {
    state = ENABLE;
  } else {
    state = DISABLE;
  }
  return state;
}

/*********************************************************************
 * @fn      SAI_GetFIFOStatus
 *
 * @brief   Returns the current SAI Block x FIFO filled level.
 *
 * @param   SAI_Block_x - where x can be A or B to select the SAI Block
 * peripheral.
 *
 * @return  The FIFO filling state.
 *           - SAI_FIFOStatus_Empty - when FIFO is empty
 *           - SAI_FIFOStatus_Less1QuarterFull - when FIFO is less than 1
 * quarter-full and not empty.
 *           - SAI_FIFOStatus_1QuarterFull - if more than 1 quarter-full.
 *           - SAI_FIFOStatus_HalfFull - if more than 1 half-full.
 *           - SAI_FIFOStatus_3QuartersFull - if more than 3 quarters-full.
 *           - SAI_FIFOStatus_Full - when FIFO is full
 */
uint32_t SAI_GetFIFOStatus(SAI_Block_TypeDef *SAI_Block_x) {
  uint32_t tmpreg = 0;

  tmpreg = (uint32_t)((SAI_Block_x->SR & SAI_SR_FLTH));

  return tmpreg;
}
