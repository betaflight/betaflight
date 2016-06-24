/**
  ******************************************************************************
  * @file    stm32f4xx_qspi.c
  * @author  MCD Application Team
  * @version V1.7.1
  * @date    20-May-2016
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the Serial peripheral interface (QSPI):
  *           + Initialization and Configuration
  *           + Indirect Data Read/Write functions
  *           + Memory Mapped Mode Data Read functions  
  *           + Automatic Polling functions
  *           + DMA transfers management
  *           + Interrupts and flags management 
  *
  *  @verbatim
  *
 ===============================================================================
                       ##### How to use this driver #####
 ===============================================================================
    [..]
        (#) Enable peripheral clock using   RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_QSPI,ENABLE);
            function.

        (#) Enable CLK, BK1_IO0, BK1_IO1, BK1_IO2, BK1_IO3, BK1_NCS, BK2_IO0, 
            BK2_IO1, BK2_IO2, BK2_IO3 and BK2_NCS GPIO clocks using 
            RCC_AHB1PeriphClockCmd() function. 
  
        (#) Peripherals alternate function: 
           (++) Connect the pin to the desired peripherals' Alternate 
                 Function (AF) using GPIO_PinAFConfig() function.
           (++) Configure the desired pin in alternate function by:
                 GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF.
           (++) Select the type, pull-up/pull-down and output speed via 
                 GPIO_PuPd, GPIO_OType and GPIO_Speed members.
           (++) Call GPIO_Init() function.
  
        (#) Program the Flash Size, CS High Time, Sample Shift, Prescaler, Clock Mode 
            values using the QSPI_Init() function.
            
        (#) Enable QSPI using  QSPI_Cmd() function.
        
        (#) Set QSPI Data Length using QSPI_SetDataLength() function.
  
        (#) Configure the FIFO threshold using QSPI_SetFIFOThreshold() to select 
            at which threshold the FTF event is generated.

        (#) Enable the NVIC and the corresponding interrupt using the function 
            QSPI_ITConfig() if you need to use interrupt mode. 
  
        (#) When using the DMA mode 
           (++) Configure the DMA using DMA_Init() function.
           (++) Active the needed channel Request using SPI_I2S_DMACmd() function.
   
        (#) Enable the SPI using the QSPI_DMACmd() function.
   
        (#) Enable the DMA using the DMA_Cmd() function when using DMA mode.  
  
    @endverbatim 
   *
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
#include "stm32f4xx_qspi.h"

/** @addtogroup STM32F4xx_StdPeriph_Driver
  * @{
  */

/** @defgroup QSPI
  * @brief QSPI driver modules
  * @{
  */
#if defined(STM32F412xG) || defined(STM32F446xx) || defined(STM32F469_479xx)
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define QSPI_CR_CLEAR_MASK                           0x00FFFFCF
#define QSPI_DCR_CLEAR_MASK                          0xFFE0F7FE
#define QSPI_CCR_CLEAR_MASK                          0x90800000
#define QSPI_PIR_CLEAR_MASK                          0xFFFF0000
#define QSPI_LPTR_CLEAR_MASK                         0xFFFF0000
#define QSPI_CCR_CLEAR_INSTRUCTION_MASK              0xFFFFFF00
#define QSPI_CCR_CLEAR_DCY_MASK                      0xFFC3FFFF
#define QSPI_CR_CLEAR_FIFOTHRESHOLD_MASK             0xFFFFF0FF
#define QSPI_CR_INTERRUPT_MASK                       0x001F0000
#define QSPI_SR_INTERRUPT_MASK                       0x0000001F
#define QSPI_FSR_INTERRUPT_MASK                      0x0000001B
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/* Initialization and Configuration functions *********************************/

/** @defgroup <PPP>_Private_Functions 
  * @{
  */

/** @defgroup <PPP>_Group1 Function Group1 Name
 *  @brief   Function group1 name description (copied from the header file) 
 *
@verbatim  
 ===============================================================================
     ##### < Function group1 name (copied from the header file)
 Note: do not use "Peripheral" or "PPP" word in the function group name >  #####
 ===============================================================================

       [..] < OPTIONAL:
              Add here the most important information to know about the IP features
              covered by this group of function.
        
              For system IPs, this section contains how to use this group API.
            >

@endverbatim
  * @{
  */   

/**
  * @brief  Deinitializes the QSPI peripheral registers to their default
  *         reset values.
  * @param  None
  * @retval None
  */
void QSPI_DeInit(void)
{
  /* Enable QSPI reset state */
  RCC_AHB3PeriphResetCmd(RCC_AHB3Periph_QSPI, ENABLE);
  /* Release QSPI from reset state */
  RCC_AHB3PeriphResetCmd(RCC_AHB3Periph_QSPI, DISABLE);
}

/**
  * @brief  Fills each QSPI_InitStruct member with its default value.
  * @param  QSPI_InitStruct: pointer to a QSPI_InitTypeDef structure which will be initialized.
  * @retval None
  */
void QSPI_StructInit(QSPI_InitTypeDef* QSPI_InitStruct)
{
/*--------- Reset QSPI init structure parameters default values ------------*/
  /* Initialize the QSPI_SShift member */
  QSPI_InitStruct->QSPI_SShift = QSPI_SShift_NoShift ;
  /* Initialize the QSPI_Prescaler member */  
  QSPI_InitStruct->QSPI_Prescaler = 0 ;
  /* Initialize the QSPI_CKMode member */
  QSPI_InitStruct->QSPI_CKMode = QSPI_CKMode_Mode0 ;
  /* Initialize the QSPI_CSHTime member */
  QSPI_InitStruct->QSPI_CSHTime = QSPI_CSHTime_1Cycle ;
  /* Initialize the QSPI_FSize member */
  QSPI_InitStruct->QSPI_FSize = 0 ;
  /* Initialize the QSPI_FSelect member */
  QSPI_InitStruct->QSPI_FSelect = QSPI_FSelect_1 ;
  /* Initialize the QSPI_DFlash member */
  QSPI_InitStruct->QSPI_DFlash = QSPI_DFlash_Disable ;
}

/**
  * @brief  Fills each QSPI_ComConfig_InitStruct member with its default value.
  * @param  QSPI_ComConfig_InitStruct: pointer to a QSPI_ComConfig_InitTypeDef structure which will be initialized.
  * @retval None
  */
void QSPI_ComConfig_StructInit(QSPI_ComConfig_InitTypeDef* QSPI_ComConfig_InitStruct)
{
/*--------- Reset QSPI ComConfig init structure parameters default values ------------*/
    
/* Set QSPI Communication configuration structure parameters default values */
  /* Initialize the QSPI_ComConfig_DDRMode member */
  QSPI_ComConfig_InitStruct->QSPI_ComConfig_DDRMode = QSPI_ComConfig_DDRMode_Disable ;
  /* Initialize the QSPI_ComConfig_DHHC member */
  QSPI_ComConfig_InitStruct->QSPI_ComConfig_DHHC = QSPI_ComConfig_DHHC_Disable ;
  /* Initialize the QSPI_ComConfig_SIOOMode member */
  QSPI_ComConfig_InitStruct->QSPI_ComConfig_SIOOMode = QSPI_ComConfig_SIOOMode_Disable ;
  /* Initialize the QSPI_ComConfig_FMode member */
  QSPI_ComConfig_InitStruct->QSPI_ComConfig_FMode = QSPI_ComConfig_FMode_Indirect_Write ;
  /* Initialize the QSPI_ComConfig_DMode member */
  QSPI_ComConfig_InitStruct->QSPI_ComConfig_DMode = QSPI_ComConfig_DMode_NoData ;
  /* Initialize the QSPI_ComConfig_DummyCycles member */
  QSPI_ComConfig_InitStruct->QSPI_ComConfig_DummyCycles = 0 ;
  /* Initialize the QSPI_ComConfig_ABSize member */
  QSPI_ComConfig_InitStruct->QSPI_ComConfig_ABSize = QSPI_ComConfig_ABSize_8bit ;
  /* Initialize the QSPI_ComConfig_ABMode member */
  QSPI_ComConfig_InitStruct->QSPI_ComConfig_ABMode = QSPI_ComConfig_ABMode_NoAlternateByte ;
  /* Initialize the QSPI_ComConfig_ADSize member */
  QSPI_ComConfig_InitStruct->QSPI_ComConfig_ADSize = QSPI_ComConfig_ADSize_8bit ;
  /* Initialize the QSPI_ComConfig_ADMode member */
  QSPI_ComConfig_InitStruct->QSPI_ComConfig_ADMode = QSPI_ComConfig_ADMode_NoAddress ;
  /* Initialize the QSPI_ComConfig_IMode member */
  QSPI_ComConfig_InitStruct->QSPI_ComConfig_IMode = QSPI_ComConfig_IMode_NoInstruction ;
  /* Initialize the QSPI_ComConfig_Ins member */
  QSPI_ComConfig_InitStruct->QSPI_ComConfig_Ins = 0 ;
}

/**
  * @brief  Initializes the QSPI peripheral according to the specified 
  *         parameters in the QSPI_InitStruct.
  * @param  QSPI_InitStruct: pointer to a QSPI_InitTypeDef structure that
  *         contains the configuration information for the specified QSPI peripheral.
  * @retval None
  */
void QSPI_Init(QSPI_InitTypeDef* QSPI_InitStruct)
{
  uint32_t tmpreg = 0;
  
  /* Check the QSPI parameters */
  assert_param(IS_QSPI_SSHIFT(QSPI_InitStruct->QSPI_SShift));
  assert_param(IS_QSPI_PRESCALER(QSPI_InitStruct->QSPI_Prescaler));
  assert_param(IS_QSPI_CKMODE(QSPI_InitStruct->QSPI_CKMode));
  assert_param(IS_QSPI_CSHTIME(QSPI_InitStruct->QSPI_CSHTime));
  assert_param(IS_QSPI_FSIZE(QSPI_InitStruct->QSPI_FSize));
	assert_param(IS_QSPI_FSEL(QSPI_InitStruct->QSPI_FSelect));
	assert_param(IS_QSPI_DFM(QSPI_InitStruct->QSPI_DFlash));
  
  /*------------------------ QSPI CR Configuration ------------------------*/
  /* Get the QUADSPI CR1 value */
  tmpreg = QUADSPI->CR;
  /* Clear PRESCALER and SSHIFT bits */
  tmpreg &= QSPI_CR_CLEAR_MASK;
  /* Configure QUADSPI: Prescaler and Sample Shift */
  tmpreg |= (uint32_t)(((QSPI_InitStruct->QSPI_Prescaler)<<24)
                        |(QSPI_InitStruct->QSPI_SShift)
	                      |(QSPI_InitStruct->QSPI_FSelect)
	                      |(QSPI_InitStruct->QSPI_DFlash));  
  /* Write to QUADSPI CR */
  QUADSPI->CR = tmpreg;
  
  /*------------------------ QUADSPI DCR Configuration ------------------------*/
  /* Get the QUADSPI DCR value */
  tmpreg = QUADSPI->DCR;
  /* Clear FSIZE, CSHT and CKMODE bits */
  tmpreg &= QSPI_DCR_CLEAR_MASK;
  /* Configure QSPI: Flash Size, Chip Select High Time and Clock Mode */
  tmpreg |= (uint32_t)(((QSPI_InitStruct->QSPI_FSize)<<16)
                        |(QSPI_InitStruct->QSPI_CSHTime)
                        |(QSPI_InitStruct->QSPI_CKMode));  
  /* Write to QSPI DCR */
  QUADSPI->DCR = tmpreg;  
}

/**
  * @brief  Initializes the QSPI CCR according to the specified 
  *         parameters in the QSPI_ComConfig_InitStruct.
  * @param  QSPI_ComConfig_InitStruct: pointer to a QSPI_ComConfig_InitTypeDef structure that
  *         contains the communication configuration informations about QSPI peripheral.
  * @retval None
  */
void QSPI_ComConfig_Init(QSPI_ComConfig_InitTypeDef* QSPI_ComConfig_InitStruct)
{
  uint32_t tmpreg = 0;

  /* Check the QSPI Communication Control parameters */
  assert_param(IS_QSPI_FMODE       (QSPI_ComConfig_InitStruct->QSPI_ComConfig_FMode));
  assert_param(IS_QSPI_SIOOMODE    (QSPI_ComConfig_InitStruct->QSPI_ComConfig_SIOOMode));
  assert_param(IS_QSPI_DMODE       (QSPI_ComConfig_InitStruct->QSPI_ComConfig_DMode));
  assert_param(IS_QSPI_DCY         (QSPI_ComConfig_InitStruct->QSPI_ComConfig_DummyCycles));
  assert_param(IS_QSPI_ABSIZE      (QSPI_ComConfig_InitStruct->QSPI_ComConfig_ABSize));
  assert_param(IS_QSPI_ABMODE      (QSPI_ComConfig_InitStruct->QSPI_ComConfig_ABMode));
  assert_param(IS_QSPI_ADSIZE      (QSPI_ComConfig_InitStruct->QSPI_ComConfig_ADSize));
  assert_param(IS_QSPI_ADMODE      (QSPI_ComConfig_InitStruct->QSPI_ComConfig_ADMode));
  assert_param(IS_QSPI_IMODE       (QSPI_ComConfig_InitStruct->QSPI_ComConfig_IMode));
  assert_param(IS_QSPI_INSTRUCTION (QSPI_ComConfig_InitStruct->QSPI_ComConfig_Ins));
	assert_param(IS_QSPI_DDRMODE     (QSPI_ComConfig_InitStruct->QSPI_ComConfig_DDRMode));
	assert_param(IS_QSPI_DHHC        (QSPI_ComConfig_InitStruct->QSPI_ComConfig_DHHC));
  
  /*------------------------ QUADSPI CCR Configuration ------------------------*/
  /* Get the QUADSPI CCR value */
  tmpreg = QUADSPI->CCR;
  /* Clear FMODE Mode bits */
  tmpreg &= QSPI_CCR_CLEAR_MASK;
  /* Configure QUADSPI: CCR Configuration */
  tmpreg |=  (uint32_t)( (QSPI_ComConfig_InitStruct->QSPI_ComConfig_FMode)
                       | (QSPI_ComConfig_InitStruct->QSPI_ComConfig_DDRMode)
											 | (QSPI_ComConfig_InitStruct->QSPI_ComConfig_DHHC)
                       | (QSPI_ComConfig_InitStruct->QSPI_ComConfig_SIOOMode)
                       | (QSPI_ComConfig_InitStruct->QSPI_ComConfig_DMode)
                       | (QSPI_ComConfig_InitStruct->QSPI_ComConfig_ABSize)
                       | (QSPI_ComConfig_InitStruct->QSPI_ComConfig_ABMode)                                                                       
                       | (QSPI_ComConfig_InitStruct->QSPI_ComConfig_ADSize)
                       | (QSPI_ComConfig_InitStruct->QSPI_ComConfig_ADMode)
                       | (QSPI_ComConfig_InitStruct->QSPI_ComConfig_IMode)
                       | (QSPI_ComConfig_InitStruct->QSPI_ComConfig_Ins)
                       |((QSPI_ComConfig_InitStruct->QSPI_ComConfig_DummyCycles)<<18));    
  /* Write to QUADSPI DCR */
  QUADSPI->CCR = tmpreg;      
}

/**
  * @brief  Enables or disables QSPI peripheral.
  * @param  NewState: new state of the QSPI peripheral. 
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void QSPI_Cmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable QSPI peripheral */
    QUADSPI->CR |= QUADSPI_CR_EN;
  }
  else
  {
    /* Disable QSPI peripheral */
    QUADSPI->CR &= ~ QUADSPI_CR_EN;
  }
}

/**
  * @brief  Configure the QSPI Automatic Polling Mode.
  * @param  QSPI_Match: Value to be compared with the masked status register to get a match. 
  *          This parameter can be any value between 0x00000000 and 0xFFFFFFFF.
  * @param  QSPI_Mask: Mask to be applied to the status bytes received in polling mode.. 
  *          This parameter can be any value between 0x00000000 and 0xFFFFFFFF.
  * @param  QSPI_Match_Mode: indicates which method should be used for determining a “match” during
  *         automatic polling mode. 
  *          This parameter can be any value of :
  *            @arg QSPI_PMM_AND: AND match mode- SMF is set if all the unmasked bits received from the flash match
  *                 the corresponding bits in the match register
  *            @arg QSPI_PMM_OR: OR match mode- SMF is set if any one of the unmasked bits received from the flash
                    matches its corresponding bit in the match register.
  * @note   This function is used only in Automatic Polling Mode
  * @retval None
  */
void QSPI_AutoPollingMode_Config(uint32_t QSPI_Match, uint32_t QSPI_Mask , uint32_t QSPI_Match_Mode)
{
  /* Check the parameters */
  assert_param(IS_QSPI_PMM(QSPI_Match_Mode));

  if (!(QUADSPI->SR & QUADSPI_SR_BUSY))
  /* Device is not Busy */
  {
    /* Set the Match Register */
    QUADSPI->PSMAR = QSPI_Match ;

    /* Set the Mask Register */
    QUADSPI->PSMKR = QSPI_Mask ;
    
    /* Set the Polling Match Mode */
    if(QSPI_Match_Mode)
    /* OR Match Mode */
    {
      /* Set the PMM bit */
      QUADSPI->CR |= QUADSPI_CR_PMM;
    }
    else
    /* AND Match Mode */
    {
      /* Reset the PMM bit */
      QUADSPI->CR &= ~ QUADSPI_CR_PMM;
    }
  }
}

/**
  * @brief  Sets the number of CLK cycle between two read during automatic polling phases.
  * @param  QSPI_Interval: The number of CLK cycle between two read during automatic polling phases. 
  *          This parameter can be any value of between 0x0000 and 0xFFFF
  * @note   This function is used only in Automatic Polling Mode  
  * @retval None
  */
void QSPI_AutoPollingMode_SetInterval(uint32_t QSPI_Interval)
{
  uint32_t tmpreg = 0;
  
  /* Check the parameters */
  assert_param(IS_QSPI_PIR(QSPI_Interval));

  if (!(QUADSPI->SR & QUADSPI_SR_BUSY))
  /* Device is not Busy */
  {
    /* Read the PIR Register */
    tmpreg = QUADSPI->PIR ;
    /* Clear Polling interval Bits */
    tmpreg &= QSPI_PIR_CLEAR_MASK ;
    /* Set the QSPI Polling Interval Bits */
    tmpreg |= QSPI_Interval;
    /* Write the PIR Register */
    QUADSPI->PIR = tmpreg;
  }
}

/**
  * @brief  Sets the value of the Timeout in Memory Mapped mode
  * @param  QSPI_Timeout: This field indicates how many CLK cycles QSPI waits after the 
  *         FIFO becomes full until it raises nCS, putting the flash memory 
  *         in a lowerconsumption state. 
  *         This parameter can be any value of between 0x0000 and 0xFFFF
  * @note   This function is used only in Memory Mapped Mode  
  * @retval None
  */
void QSPI_MemoryMappedMode_SetTimeout(uint32_t QSPI_Timeout)
{
  uint32_t tmpreg = 0;
  
  /* Check the parameters */
  assert_param(IS_QSPI_TIMEOUT(QSPI_Timeout));

  if (!(QUADSPI->SR & QUADSPI_SR_BUSY))
  /* Device is not Busy */
  {
    /* Read the LPTR Register */
    tmpreg = QUADSPI->LPTR ;
    /* Clear Timeout Bits */
    tmpreg &= QSPI_LPTR_CLEAR_MASK ;
    /* Set Timeout Bits */
    tmpreg |= QSPI_Timeout;
    /* Write the LPTR Register */
    QUADSPI->LPTR = tmpreg;
  }
}

/**
  * @brief  Sets the value of the Address
  * @param  QSPI_Address: Address to be send to the external flash memory.  
  *         This parameter can be any value of between 0x00000000 and 0xFFFFFFFF
  * @note   This function is used only in Indirect Mode  
  * @retval None
  */
void QSPI_SetAddress(uint32_t QSPI_Address)
{
  if (!(QUADSPI->SR & QUADSPI_SR_BUSY))
  /* Device is not Busy */
  {
    /* Write the AR Register */
    QUADSPI->AR = QSPI_Address;
  }
}

/**
  * @brief  Sets the value of the Alternate Bytes
  * @param  QSPI_AlternateByte: Optional data to be send to the external QSPI device right after the address. 
  *         This parameter can be any value of between 0x00000000 and 0xFFFFFFFF
  * @note   This function is used only in Indirect Mode  
  * @retval None
  */
void QSPI_SetAlternateByte(uint32_t QSPI_AlternateByte)
{
  if (!(QUADSPI->SR & QUADSPI_SR_BUSY))
  /* Device is not Busy */
  {
    /* Write the ABR Register */
    QUADSPI->ABR = QSPI_AlternateByte;
  }
}

/**
  * @brief  Sets the FIFO Threshold
  * @param  QSPI_FIFOThres: Defines, in indirect mode, the threshold number 
  *           of bytes in the FIFO which will cause the FIFO Threshold Flag 
  *           FTF to be set.
  *         This parameter can be any value of between 0x00 and 0x0F
  * @retval None
  */
void QSPI_SetFIFOThreshold(uint32_t QSPI_FIFOThreshold)
{
  uint32_t tmpreg = 0;
  
  /* Check the parameters */
  assert_param(IS_QSPI_FIFOTHRESHOLD(QSPI_FIFOThreshold));

  /* Read the CR Register */
  tmpreg = QUADSPI->CR ;
  /* Clear FIFO Threshold Bits */
  tmpreg &= QSPI_CR_CLEAR_FIFOTHRESHOLD_MASK ;
  /* Set FIFO Threshold Bits */
  tmpreg |= (QSPI_FIFOThreshold << 8);
  /* Write the CR Register */
  QUADSPI->CR = tmpreg;
}

/**
  * @brief  Sets number of Bytes to be transferred 
  * @param  QSPI_DataLength: Number of data to be retrieved (value+1) 
  *         in indirect and status-polling modes. A value no greater than 3 
  *         (indicating 4 bytes) should be used for status-polling mode. 
  *         All 1s in indirect mode means undefined length, where QSPI will 
  *         continue until the end of memory, as defined by FSIZE
  *         This parameter can be any value of between 0x00000000 and 0xFFFFFFFF
  *             0x0000_0000: 1 byte is to be transferred
  *             0x0000_0001: 2 bytes are to be transferred
  *             0x0000_0002: 3 bytes are to be transferred
  *             0x0000_0003: 4 bytes are to be transferred
  *             ...
  *             0xFFFF_FFFD: 4,294,967,294 (4G-2) bytes are to be transferred
  *             0xFFFF_FFFE: 4,294,967,295 (4G-1) bytes are to be transferred
  *             0xFFFF_FFFF: undefined length -- all bytes until the end of flash memory (as defined
  *             by FSIZE) are to be transferred
  * @note   This function is not used in Memory Mapped Mode.
  * @retval None
  */
void QSPI_SetDataLength(uint32_t QSPI_DataLength)
{
  if (!(QUADSPI->SR & QUADSPI_SR_BUSY))
  /* Device is not Busy */
  {
    /* Write the DLR Register */
    QUADSPI->DLR = QSPI_DataLength;
  }
}

/**
  * @brief  Enables or disables The Timeout Counter.
  * @param  NewState: new state of the Timeout Counter. 
  *          This parameter can be: ENABLE or DISABLE.
  * @note   This function is used only in Memory Mapped Mode.
  * @retval None
  */
void QSPI_TimeoutCounterCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (!(QUADSPI->SR & QUADSPI_SR_BUSY))
  /* Device is not Busy */
  {
    if (NewState != DISABLE)
    {
      /* Enable Timeout Counter */
      QUADSPI->CR |= QUADSPI_CR_TCEN;
    }
    else
    {
      /* Disable Timeout Counter */
      QUADSPI->CR &= ~ QUADSPI_CR_TCEN;
    }
  }
}

/**
  * @brief  Enables or disables Automatic Polling Mode Stop when a match occurs.
  * @param  NewState: new state of the Automatic Polling Mode Stop. 
  *          This parameter can be: ENABLE or DISABLE.
  * @note   This function is used only in Automatic Polling Mode.    
  * @retval None
  */
void QSPI_AutoPollingModeStopCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (!(QUADSPI->SR & QUADSPI_SR_BUSY))
  /* Device is not Busy */
  {
    if (NewState != DISABLE)
    {
      /* Enable Automatic Polling Mode Stop */
      QUADSPI->CR |= QUADSPI_CR_APMS;
    }
    else
    {
      /* Disable Automatic Polling Mode Stop */
      QUADSPI->CR &= ~ QUADSPI_CR_APMS;
    }
  }
}

/**
  * @brief  Abort the on-going command sequence.
  * @param  None
  * @retval None
  */
void QSPI_AbortRequest(void)
{
  /* Enable the ABORT request bit in CR */
  QUADSPI->CR |= QUADSPI_CR_ABORT;
}

/* Data transfers functions ***************************************************/

/**
  * @brief  Transmits a 8bit Data through the QSPI peripheral.
  * @param  Data: Data to be transmitted.
  * @retval None
  */
void QSPI_SendData8(uint8_t Data)
{
  uint32_t quadspibase = 0;

  quadspibase = (uint32_t)QUADSPI; 
  quadspibase += 0x20;
  
  *(__IO uint8_t *) quadspibase = Data;
}

/**
  * @brief  Transmits a 16bit Data through the QSPI peripheral.
  * @param  Data: Data to be transmitted.
  * @retval None
  */
void QSPI_SendData16(uint16_t Data)
{
  uint32_t quadspibase = 0;

  quadspibase = (uint32_t)QUADSPI; 
  quadspibase += 0x20;
  
  *(__IO uint16_t *) quadspibase = Data;
}

/**
  * @brief  Transmits a 32bit Data through the QSPI peripheral.
  * @param  Data: Data to be transmitted.
  * @retval None
  */
void QSPI_SendData32(uint32_t Data)
{
  QUADSPI->DR = Data;
}

/**
  * @brief  Returns the most recent received 8bit data by the QSPI peripheral. 
  * @retval The value of the received data.
  */
uint8_t QSPI_ReceiveData8(void)
{
  uint32_t quadspibase = 0;
  
  quadspibase = (uint32_t)QUADSPI; 
  quadspibase += 0x20;
  
  return *(__IO uint8_t *) quadspibase;
}

/**
  * @brief  Returns the most recent received 16bit data by the QSPI peripheral. 
  * @retval The value of the received data.
  */
uint16_t QSPI_ReceiveData16(void)
{
  uint32_t quadspibase = 0;
  
  quadspibase = (uint32_t)QUADSPI; 
  quadspibase += 0x20;
  
  return *(__IO uint16_t *) quadspibase;
}

/**
  * @brief  Returns the most recent received 32bit data by the QSPI peripheral. 
  * @retval The value of the received data.
  */
uint32_t QSPI_ReceiveData32(void)
{
  return QUADSPI->DR;
}

/* DMA transfers management functions *****************************************/

/**
  * @brief  Enables or disables DMA for Indirect Mode.
  * @param  NewState: new state of the Timeout Counter. 
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void QSPI_DMACmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable DMA */
    QUADSPI->CR |= QUADSPI_CR_DMAEN;
  }
  else
  {
    /* Disable DMA */
    QUADSPI->CR &= ~ QUADSPI_CR_DMAEN;
  }
}

/* Interrupts and flags management functions **********************************/

/**
  * @brief  Enables or disables the specified QSPI interrupts.
  * @param  QSPI_IT: specifies the QSPI interrupt source to be enabled or disabled. 
  *          This parameter can be one of the following values:
  *            @arg QSPI_IT_TO: Timeout interrupt
  *            @arg QSPI_IT_SM: Status Match interrupt
  *            @arg QSPI_IT_FT: FIFO Threshold
  *            @arg QSPI_IT_TC: Transfer Complete
  *            @arg QSPI_IT_TE: Transfer Error      
  * @param  NewState: new state of the specified QSPI interrupt.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void QSPI_ITConfig(uint32_t QSPI_IT, FunctionalState NewState)
{
  uint32_t tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  assert_param(IS_QSPI_IT(QSPI_IT));

  /* Read the CR Register */
  tmpreg = QUADSPI->CR ;
  
  if(NewState != DISABLE)
  {
    /* Enable the selected QSPI interrupt */
    tmpreg |= (uint32_t)(QSPI_IT & QSPI_CR_INTERRUPT_MASK);
  }
  else
  {
    /* Disable the selected QSPI interrupt */
    tmpreg &= ~(uint32_t)(QSPI_IT & QSPI_CR_INTERRUPT_MASK);
  }
  /* Write the CR Register */
  QUADSPI->CR = tmpreg ;  
}

/**
  * @brief  Returns the current QSPI FIFO filled level.
  * @retval Number of valid bytes which are being held in the FIFO.
  *         0x00 : FIFO is empty
  *         0x1F : FIFO is full    
  */
uint32_t QSPI_GetFIFOLevel(void)
{
  /* Get the QSPI FIFO level bits */
  return ((QUADSPI->SR & QUADSPI_SR_FLEVEL)>> 8);
}

/**
  * @brief  Returns the QSPI functional mode.  
  * @param  None 
  * @retval QSPI Functional Mode .The returned value can be one of the following:
  *              - 0x00000000: QSPI_FMode_Indirect_Write
  *              - 0x04000000: QSPI_FMode_Indirect_Read
  *              - 0x08000000: QSPI_FMode_AutoPolling
  *              - 0x0C000000: QSPI_FMode_MemoryMapped
  */
uint32_t QSPI_GetFMode(void)
{
  /* Return the QSPI_FMode */
  return  (QUADSPI->CCR & QUADSPI_CCR_FMODE);
}

/**
  * @brief  Checks whether the specified QSPI flag is set or not.  
  * @param  QSPI_FLAG: specifies the QSPI flag to check. 
  *          This parameter can be one of the following values:
  *            @arg QSPI_FLAG_TO: Timeout interrupt flag
  *            @arg QSPI_FLAG_SM: Status Match interrupt flag
  *            @arg QSPI_FLAG_FT: FIFO Threshold flag
  *            @arg QSPI_FLAG_TC: Transfer Complete flag
  *            @arg QSPI_FLAG_TE: Transfer Error flag
  *            @arg QSPI_FLAG_BUSY: Busy flag      
  * @retval The new state of QSPI_FLAG (SET or RESET).
  */
FlagStatus QSPI_GetFlagStatus(uint32_t QSPI_FLAG)
{
  FlagStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_QSPI_GET_FLAG(QSPI_FLAG));

  /* Check the status of the specified QSPI flag */
  if (QUADSPI->SR & QSPI_FLAG)
  {
    /* QSPI_FLAG is set */
    bitstatus = SET;
  }
  else
  {
    /* QSPI_FLAG is reset */
    bitstatus = RESET;
  }
  /* Return the QSPI_FLAG status */
  return  bitstatus;
}

/**
  * @brief  Clears the QSPI flag.
  * @param  QSPI_FLAG: specifies the QSPI flag to clear. 
  *          This parameter can be one of the following values:
  *            @arg QSPI_FLAG_TO: Timeout interrupt flag
  *            @arg QSPI_FLAG_SM: Status Match interrupt flag
  *            @arg QSPI_FLAG_TC: Transfer Complete flag
  *            @arg QSPI_FLAG_TE: Transfer Error flag
  * @retval None
  */
void QSPI_ClearFlag(uint32_t QSPI_FLAG)
{
  /* Check the parameters */
  assert_param(IS_QSPI_CLEAR_FLAG(QSPI_FLAG));

  /* Clear the selected QSPI flags */
  QUADSPI->FCR = QSPI_FLAG;
}

/**
  * @brief  Checks whether the specified QSPI interrupt has occurred or not.
  * @param  QSPI_IT: specifies the QSPI interrupt source to check. 
  *          This parameter can be one of the following values:
  *            @arg QSPI_IT_TO: Timeout interrupt 
  *            @arg QSPI_IT_SM: Status Match interrupt
  *            @arg QSPI_IT_FT: FIFO Threshold
  *            @arg QSPI_IT_TC: Transfer Complete
  *            @arg QSPI_IT_TE: Transfer Error    
  * @retval The new state of QSPI_IT (SET or RESET).
  */
ITStatus QSPI_GetITStatus(uint32_t QSPI_IT)
{
  ITStatus bitstatus = RESET;
  __IO uint32_t tmpcreg = 0, tmpsreg = 0; 

  /* Check the parameters */
  assert_param(IS_QSPI_IT(QSPI_IT));

  /* Read the QUADSPI CR */
  tmpcreg = QUADSPI->CR;  
  tmpcreg &= (uint32_t)(QSPI_IT & QSPI_CR_INTERRUPT_MASK);
  
  /* Read the QUADSPI SR */
  tmpsreg = QUADSPI->SR;  
  tmpsreg &= (uint32_t)(QSPI_IT & QSPI_SR_INTERRUPT_MASK);

  /* Check the status of the specified QSPI interrupt */
  if((tmpcreg != RESET) && (tmpsreg != RESET))
  {
    /* QSPI_IT is set */
    bitstatus = SET;
  }
  else
  {
    /* QSPI_IT is reset */
    bitstatus = RESET;
  }
  /* Return the QSPI_IT status */
  return bitstatus;
}

/**
  * @brief  Clears the QSPI's interrupt pending bits.
  * @param  QSPI_IT: specifies the QSPI pending bit to clear. 
  *          This parameter can be one of the following values:
  *            @arg QSPI_IT_TO: Timeout interrupt 
  *            @arg QSPI_IT_SM: Status Match interrupt
  *            @arg QSPI_IT_TC: Transfer Complete
  *            @arg QSPI_IT_TE: Transfer Error 
  * @retval None
  */
void QSPI_ClearITPendingBit(uint32_t QSPI_IT)
{
  /* Check the parameters */
  assert_param(IS_QSPI_CLEAR_IT(QSPI_IT));  

  QUADSPI->FCR = (uint32_t)(QSPI_IT & QSPI_FSR_INTERRUPT_MASK);
}

/**
  * @brief  Enables or disables QSPI Dual Flash Mode.
  * @param  NewState: new state of the QSPI Dual Flash Mode. 
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void QSPI_DualFlashMode_Cmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable QSPI Dual Flash Mode */
    QUADSPI->CR |= QUADSPI_CR_DFM;
  }
  else
  {
    /* Disable QSPI Dual Flash Mode */
    QUADSPI->CR &= ~ QUADSPI_CR_DFM;
  }
}

/**
  * @}
  */

/**
  * @}
  */
#endif /* STM32F412xG || STM32F446xx || STM32F469_479xx */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
