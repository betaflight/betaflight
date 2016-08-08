/**
  ******************************************************************************
  * @file    stm32f4xx_spdifrx.c
  * @author  MCD Application Team
  * @version V1.7.1
  * @date    20-May-2016
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the Serial Audio Interface (SPDIFRX):
  *           + Initialization and Configuration
  *           + Data transfers functions
  *           + DMA transfers management
  *           + Interrupts and flags management 
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
#include "stm32f4xx_spdifrx.h"
#include "stm32f4xx_rcc.h"

/** @addtogroup STM32F4xx_StdPeriph_Driver
  * @{
  */

/** @defgroup SPDIFRX 
  * @brief SPDIFRX driver modules
  * @{
  */ 
#if defined(STM32F446xx)
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define CR_CLEAR_MASK 0x000000FE7
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup SPDIFRX_Private_Functions
  * @{
  */

/** @defgroup SPDIFRX_Group1 Initialization and Configuration functions
  *  @brief   Initialization and Configuration functions 
  *
@verbatim   
 ===============================================================================
            ##### Initialization and Configuration functions #####
 ===============================================================================  
  [..]
  This section provides a set of functions allowing to initialize the SPDIFRX Audio 

  Block Mode, Audio Protocol, Data size, Synchronization between audio block, 
  Master clock Divider, FIFO threshold, Frame configuration, slot configuration,
  Tristate mode, Companding mode and Mute mode.  
  [..] 
  The SPDIFRX_Init(), SPDIFRX_FrameInit() and SPDIFRX_SlotInit() functions follows the SPDIFRX Block
  configuration procedures for Master mode and Slave mode (details for these procedures 
  are available in reference manual(RMxxxx).
  
@endverbatim
  * @{
  */

/**
  * @brief  Deinitialize the SPDIFRXx peripheral registers to their default reset values.
  * @param  void
  * @retval None
  */
void SPDIFRX_DeInit(void)
{
  /* Enable SPDIFRX reset state */
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPDIFRX, ENABLE);
  /* Release SPDIFRX from reset state */
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPDIFRX, DISABLE); 
}

/**
  * @brief  Initializes the SPDIFRX  peripheral according to the specified 
  *         parameters in the SPDIFRX_InitStruct.
  *         
  * @note   SPDIFRX clock is generated from a specific output of the PLLSPDIFRX or a specific  
  *         output of the PLLI2S or from an alternate function bypassing the PLL I2S.
  *        
  * @param  SPDIFRX_InitStruct: pointer to a SPDIFRX_InitTypeDef structure that
  *         contains the configuration information for the specified SPDIFRX Block peripheral.             
  * @retval None
  */
void SPDIFRX_Init(SPDIFRX_InitTypeDef* SPDIFRX_InitStruct)
{
  uint32_t tmpreg = 0;
  
  /* Check the SPDIFRX parameters */
  assert_param(IS_STEREO_MODE(SPDIFRX_InitStruct->SPDIFRX_StereoMode));
  assert_param(IS_SPDIFRX_INPUT_SELECT(SPDIFRX_InitStruct->SPDIFRX_InputSelection));
  assert_param(IS_SPDIFRX_MAX_RETRIES(SPDIFRX_InitStruct->SPDIFRX_Retries));
  assert_param(IS_SPDIFRX_WAIT_FOR_ACTIVITY(SPDIFRX_InitStruct->SPDIFRX_WaitForActivity));
  assert_param(IS_SPDIFRX_CHANNEL(SPDIFRX_InitStruct->SPDIFRX_ChannelSelection));
  assert_param(IS_SPDIFRX_DATA_FORMAT(SPDIFRX_InitStruct->SPDIFRX_DataFormat));
  
  /* SPDIFRX CR Configuration */
  /* Get the SPDIFRX CR value */
  tmpreg = SPDIFRX->CR;
  /* Clear INSEL, WFA, NBTR, CHSEL, DRFMT and RXSTEO bits */
  tmpreg &= CR_CLEAR_MASK;
  /* Configure SPDIFRX: Input selection, Maximum allowed re-tries during synchronization phase, 
  wait for activity, Channel Selection, Data samples format and stereo/mono mode */  
  /* Set INSEL bits according to SPDIFRX_InputSelection value   */
  /* Set WFA   bit  according to SPDIFRX_WaitForActivity value  */
  /* Set NBTR  bit  according to SPDIFRX_Retries value          */
  /* Set CHSEL bit  according to SPDIFRX_ChannelSelection value */
  /* Set DRFMT bits according to SPDIFRX_DataFormat value       */
  /* Set RXSTEO bit according to SPDIFRX_StereoMode value       */
  
  tmpreg |= (uint32_t)(SPDIFRX_InitStruct->SPDIFRX_InputSelection   | SPDIFRX_InitStruct->SPDIFRX_WaitForActivity   |
                       SPDIFRX_InitStruct->SPDIFRX_Retries          | SPDIFRX_InitStruct->SPDIFRX_ChannelSelection  |  
                       SPDIFRX_InitStruct->SPDIFRX_DataFormat       | SPDIFRX_InitStruct->SPDIFRX_StereoMode
                       );
  
  /* Write to SPDIFRX CR */
  SPDIFRX->CR = tmpreg;	
}

/**
  * @brief  Fills each SPDIFRX_InitStruct member with its default value.
  * @param  SPDIFRX_InitStruct: pointer to a SPDIFRX_InitTypeDef structure which will 
  *         be initialized.  
  * @retval None
  */
void SPDIFRX_StructInit(SPDIFRX_InitTypeDef* SPDIFRX_InitStruct)
{
  /* Reset SPDIFRX init structure parameters values */
  /* Initialize the PDIF_InputSelection member */
  SPDIFRX_InitStruct->SPDIFRX_InputSelection = SPDIFRX_Input_IN0;
  /* Initialize the SPDIFRX_WaitForActivity member */
  SPDIFRX_InitStruct->SPDIFRX_WaitForActivity = SPDIFRX_WaitForActivity_On;
  /* Initialize the SPDIFRX_Retries member */
  SPDIFRX_InitStruct->SPDIFRX_Retries = SPDIFRX_16MAX_RETRIES;
  /* Initialize the SPDIFRX_ChannelSelection member */
  SPDIFRX_InitStruct->SPDIFRX_ChannelSelection = SPDIFRX_Select_Channel_A;
  /* Initialize the SPDIFRX_DataFormat member */
  SPDIFRX_InitStruct->SPDIFRX_DataFormat = SPDIFRX_MSB_DataFormat;
  /* Initialize the SPDIFRX_StereoMode member */
  SPDIFRX_InitStruct->SPDIFRX_StereoMode = SPDIFRX_StereoMode_Enabled;
}

/**
  * @brief  Enables or disables the SPDIFRX frame x bit.
  * @param  NewState: new state of the selected SPDIFRX frame bit.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void SPDIFRX_SetPreambleTypeBit(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the selected SPDIFRX frame bit */
    SPDIFRX->CR |= SPDIFRX_CR_PTMSK;
  }
  else
  {
    /* Disable the selected SPDIFRX frame bit */
    SPDIFRX->CR &= ~(SPDIFRX_CR_PTMSK);
  }
}

/**
  * @brief  Enables or disables the SPDIFRX frame x bit.
  * @param  NewState: new state of the selected SPDIFRX frame bit.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void SPDIFRX_SetUserDataChannelStatusBits(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the selected SPDIFRX frame bit */
    SPDIFRX->CR |= SPDIFRX_CR_CUMSK;
  }
  else
  {
    /* Disable the selected SPDIFRX frame bit */
    SPDIFRX->CR &= ~(SPDIFRX_CR_CUMSK);
  }
}

/**
  * @brief  Enables or disables the SPDIFRX frame x bit.
  * @param  NewState: new state of the selected SPDIFRX frame bit.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void SPDIFRX_SetValidityBit(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the selected SPDIFRX frame bit */
    SPDIFRX->CR |= SPDIFRX_CR_VMSK;
  }
  else
  {
    /* Disable the selected SPDIFRX frame bit */
    SPDIFRX->CR &= ~(SPDIFRX_CR_VMSK);
  }
}

/**
  * @brief  Enables or disables the SPDIFRX frame x bit.
  * @param  NewState: new state of the selected SPDIFRX frame bit.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void SPDIFRX_SetParityBit(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the selected SPDIFRX frame bit */
    SPDIFRX->CR |= SPDIFRX_CR_PMSK;
  }
  else
  {
    /* Disable the selected SPDIFRX frame bit */
    SPDIFRX->CR &= ~(SPDIFRX_CR_PMSK);
  }
}

/**
  * @brief  Enables or disables the SPDIFRX DMA interface (RX).
  * @param  NewState: new state of the selected SPDIFRX DMA transfer request.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void SPDIFRX_RxDMACmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the selected SPDIFRX DMA requests */
    SPDIFRX->CR |= SPDIFRX_CR_RXDMAEN;
  }
  else
  {
    /* Disable the selected SPDIFRX DMA requests */
    SPDIFRX->CR &= ~(SPDIFRX_CR_RXDMAEN);
  }
}

/**
  * @brief  Enables or disables the SPDIFRX DMA interface (Control Buffer).
  * @param  NewState: new state of the selected SPDIFRX DMA transfer request.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void SPDIFRX_CbDMACmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the selected SPDIFRX DMA requests */
    SPDIFRX->CR |= SPDIFRX_CR_CBDMAEN;
  }
  else
  {
    /* Disable the selected SPDIFRX DMA requests */
    SPDIFRX->CR &= ~(SPDIFRX_CR_CBDMAEN);
  }
}

/**
  * @brief  Enables or disables the SPDIFRX peripheral.
  * @param  SPDIFRX_State: specifies the SPDIFRX peripheral state.
  *          This parameter can be one of the following values:
  *            @arg SPDIFRX_STATE_IDLE : Disable SPDIFRX-RX (STATE_IDLE)
  *            @arg SPDIFRX_STATE_SYNC : Enable SPDIFRX-RX Synchronization only 
  *            @arg SPDIFRX_STATE_RCV  : Enable SPDIFRX Receiver 
  * @retval None
  */
void SPDIFRX_Cmd(uint32_t SPDIFRX_State)
{
  /* Check the parameters */
  assert_param(IS_SPDIFRX_STATE(SPDIFRX_State));
	
  /* Clear SPDIFRXEN bits */
    SPDIFRX->CR &= ~(SPDIFRX_CR_SPDIFEN);
  /* Set new SPDIFRXEN value */
    SPDIFRX->CR |= SPDIFRX_State;
}

/**
  * @brief  Enables or disables the specified SPDIFRX Block interrupts.
  * @param  SPDIFRX_IT: specifies the SPDIFRX interrupt source to be enabled or disabled. 
  *          This parameter can be one of the following values:
  *            @arg SPDIFRX_IT_RXNE:  RXNE interrupt enable
  *            @arg SPDIFRX_IT_CSRNE: Control Buffer Ready Interrupt Enable
  *            @arg SPDIFRX_IT_PERRIE: Parity error interrupt enable
  *            @arg SPDIFRX_IT_OVRIE:  Overrun error Interrupt Enable 
  *            @arg SPDIFRX_IT_SBLKIE: Synchronization Block Detected Interrupt Enable 
  *            @arg SPDIFRX_IT_SYNCDIE: Synchronization Done
  *            @arg SPDIFRX_IT_IFEIE: Serial Interface Error Interrupt Enable      
  * @param  NewState: new state of the specified SPDIFRX interrupt.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void SPDIFRX_ITConfig(uint32_t SPDIFRX_IT, FunctionalState NewState)
{ 
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  assert_param(IS_SPDIFRX_CONFIG_IT(SPDIFRX_IT));

  if (NewState != DISABLE)
  {
    /* Enable the selected SPDIFRX interrupt */
    SPDIFRX->IMR |= SPDIFRX_IT;
  }
  else
  {
    /* Disable the selected SPDIFRX interrupt */
    SPDIFRX->IMR &= ~(SPDIFRX_IT);
  }
}

/**
  * @brief  Checks whether the specified SPDIFRX flag is set or not.
  * @param  SPDIFRX_FLAG: specifies the SPDIFRX flag to check. 
  *          This parameter can be one of the following values:
  *            @arg SPDIFRX_FLAG_RXNE: Read data register not empty flag.  
  *            @arg SPDIFRX_FLAG_CSRNE: The Control Buffer register is not empty flag.  
  *            @arg SPDIFRX_FLAG_PERR: Parity error flag.
  *            @arg SPDIFRX_FLAG_OVR: Overrun error flag.            
  *            @arg SPDIFRX_FLAG_SBD: Synchronization Block Detected flag. 
  *            @arg SPDIFRX_FLAG_SYNCD: Synchronization Done flag.
  *            @arg SPDIFRX_FLAG_FERR: Framing error flag.
  *            @arg SPDIFRX_FLAG_SERR: Synchronization error flag.
  *            @arg SPDIFRX_FLAG_TERR: Time-out error flag.
  * @retval The new state of SPDIFRX_FLAG (SET or RESET).
  */
FlagStatus SPDIFRX_GetFlagStatus(uint32_t SPDIFRX_FLAG)
{
  FlagStatus bitstatus = RESET;
  
  /* Check the parameters */
  assert_param(IS_SPDIFRX_FLAG(SPDIFRX_FLAG));
  
  /* Check the status of the specified SPDIFRX flag */
  if ((SPDIFRX->SR & SPDIFRX_FLAG) != (uint32_t)RESET)
  {
    /* SPDIFRX_FLAG is set */
    bitstatus = SET;
  }
  else
  {
    /* SPDIFRX_FLAG is reset */
    bitstatus = RESET;
  }
  /* Return the SPDIFRX_FLAG status */
  return  bitstatus;
}

/**
  * @brief  Clears the specified SPDIFRX flag.
  * @param  SPDIFRX_FLAG: specifies the SPDIFRX flag to check. 
  *          This parameter can be one of the following values: 
  *            @arg SPDIFRX_FLAG_PERR: Parity error flag.
  *            @arg SPDIFRX_FLAG_OVR: Overrun error flag.            
  *            @arg SPDIFRX_FLAG_SBD: Synchronization Block Detected flag. 
  *            @arg SPDIFRX_FLAG_SYNCD: Synchronization Done flag.          
  *  
  * @retval None
  */
void SPDIFRX_ClearFlag(uint32_t SPDIFRX_FLAG)
{
  /* Check the parameters */
  assert_param(IS_SPDIFRX_CLEAR_FLAG(SPDIFRX_FLAG));
    
  /* Clear the selected SPDIFRX Block flag */
  SPDIFRX->IFCR |= SPDIFRX_FLAG;
}

/**
  * @brief  Checks whether the specified SPDIFRX interrupt has occurred or not.
  * @param  SPDIFRX_IT: specifies the SPDIFRX interrupt source to be enabled or disabled. 
  *          This parameter can be one of the following values:
  *            @arg SPDIFRX_IT_RXNE:  RXNE interrupt enable
  *            @arg SPDIFRX_IT_CSRNE: Control Buffer Ready Interrupt Enable
  *            @arg SPDIFRX_IT_PERRIE: Parity error interrupt enable
  *            @arg SPDIFRX_IT_OVRIE:  Overrun error Interrupt Enable 
  *            @arg SPDIFRX_IT_SBLKIE: Synchronization Block Detected Interrupt Enable 
  *            @arg SPDIFRX_IT_SYNCDIE: Synchronization Done
  *            @arg SPDIFRX_IT_IFEIE: Serial Interface Error Interrupt Enable                 
  * @retval The new state of SPDIFRX_IT (SET or RESET).
  */
ITStatus SPDIFRX_GetITStatus(uint32_t SPDIFRX_IT)
{
  ITStatus bitstatus = RESET;
  uint32_t  enablestatus = 0;

  /* Check the parameters */
  assert_param(IS_SPDIFRX_CONFIG_IT(SPDIFRX_IT));
  
  /* Get the SPDIFRX_IT enable bit status */
  enablestatus = (SPDIFRX->IMR & SPDIFRX_IT) ;

  /* Check the status of the specified SPDIFRX interrupt */
  if (((SPDIFRX->SR & SPDIFRX_IT) != (uint32_t)RESET) && (enablestatus != (uint32_t)RESET))
  {
    /* SPDIFRX_IT is set */
    bitstatus = SET;
  }
  else
  {
    /* SPDIFRX_IT is reset */
    bitstatus = RESET;
  }
  /* Return the SPDIFRX_IT status */
  return bitstatus;
}

/**
  * @brief  Clears the SPDIFRX interrupt pending bit.
  * @param  SAI_IT: specifies the SPDIFRX interrupt pending bit to clear. 
  *          This parameter can be one of the following values:  
  *            @arg SPDIFRX_IT_MUTEDET: MUTE detection interrupt.  
  *            @arg SPDIFRX_IT_OVRUDR: overrun/underrun interrupt.
  *            @arg SPDIFRX_IT_WCKCFG: wrong clock configuration interrupt.            
  *            @arg SPDIFRX_IT_CNRDY: codec not ready interrupt. 
  *            @arg SPDIFRX_IT_AFSDET: anticipated frame synchronization detection interrupt.
  *            @arg SPDIFRX_IT_LFSDET: late frame synchronization detection interrupt. 
  *  
  * @note    FREQ (FIFO Request) flag is cleared : 
  *          - When the audio block is transmitter and the FIFO is full or the FIFO   
  *            has one data (one buffer mode) depending the bit FTH in the
  *            SPDIFRX_xCR2 register.
  *          - When the audio block is receiver and the FIFO is not empty  
  *            
  * @retval None
  */
void SPDIFRX_ClearITPendingBit(uint32_t SPDIFRX_IT)
{
  /* Check the parameters */
  assert_param(IS_SPDIFRX_CLEAR_FLAG(SPDIFRX_IT));

  /* Clear the selected SPDIFRX interrupt pending bit */
  SPDIFRX->IFCR |= SPDIFRX_IT; 
}

/**
  * @}
  */

/**
  * @}
  */ 
#endif /* STM32F446xx */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
