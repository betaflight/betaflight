/**
  ******************************************************************************
  * @file    stm32f4xx_fmpi2c.c
  * @author  MCD Application Team
  * @version V1.6.1
  * @date    21-October-2015
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the Inter-Integrated circuit Fast Mode Plus (FMPI2C):
  *           + Initialization and Configuration
  *           + Communications handling
  *           + SMBUS management
  *           + FMPI2C registers management
  *           + Data transfers management
  *           + DMA transfers management
  *           + Interrupts and flags management
  *
  *  @verbatim
 ============================================================================
                     ##### How to use this driver #####
 ============================================================================
   [..]
   (#) Enable peripheral clock using RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2Cx, ENABLE)
       function for FMPI2C peripheral.
   (#) Enable SDA, SCL  and SMBA (when used) GPIO clocks using 
       RCC_AHBPeriphClockCmd() function. 
   (#) Peripherals alternate function: 
       (++) Connect the pin to the desired peripherals' Alternate 
            Function (AF) using GPIO_PinAFConfig() function.
       (++) Configure the desired pin in alternate function by:
            GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF
       (++) Select the type, OpenDrain and speed via 
            GPIO_PuPd, GPIO_OType and GPIO_Speed members
       (++) Call GPIO_Init() function.
   (#) Program the Mode, Timing , Own address, Ack and Acknowledged Address 
       using the FMPI2C_Init() function.
   (#) Optionally you can enable/configure the following parameters without
       re-initialization (i.e there is no need to call again FMPI2C_Init() function):
       (++) Enable the acknowledge feature using FMPI2C_AcknowledgeConfig() function.
       (++) Enable the dual addressing mode using FMPI2C_DualAddressCmd() function.
       (++) Enable the general call using the FMPI2C_GeneralCallCmd() function.
       (++) Enable the clock stretching using FMPI2C_StretchClockCmd() function.
       (++) Enable the PEC Calculation using FMPI2C_CalculatePEC() function.
       (++) For SMBus Mode: 
            (+++) Enable the SMBusAlert pin using FMPI2C_SMBusAlertCmd() function.
   (#) Enable the NVIC and the corresponding interrupt using the function
       FMPI2C_ITConfig() if you need to use interrupt mode.
   (#) When using the DMA mode 
      (++) Configure the DMA using DMA_Init() function.
      (++) Active the needed channel Request using FMPI2C_DMACmd() function.
   (#) Enable the FMPI2C using the FMPI2C_Cmd() function.
   (#) Enable the DMA using the DMA_Cmd() function when using DMA mode in the 
       transfers.
   [..]        
   (@) When using FMPI2C in Fast Mode Plus, SCL and SDA pin 20mA current drive capability
       must be enabled by setting the driving capability control bit in SYSCFG.
       
    @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
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
#include "stm32f4xx_fmpi2c.h"
#include "stm32f4xx_rcc.h"

/** @addtogroup STM32F40x_StdPeriph_Driver
  * @{
  */

/** @defgroup FMPI2C 
  * @brief FMPI2C driver modules
  * @{
  */
#if defined(STM32F410xx) || defined(STM32F446xx)
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define CR1_CLEAR_MASK          ((uint32_t)0x00CFE0FF)  /*<! FMPI2C CR1 clear register Mask */
#define CR2_CLEAR_MASK          ((uint32_t)0x07FF7FFF)  /*<! FMPI2C CR2 clear register Mask */
#define TIMING_CLEAR_MASK       ((uint32_t)0xF0FFFFFF)  /*<! FMPI2C TIMING clear register Mask */
#define ERROR_IT_MASK           ((uint32_t)0x00003F00)  /*<! FMPI2C Error interrupt register Mask */
#define TC_IT_MASK              ((uint32_t)0x000000C0)  /*<! FMPI2C TC interrupt register Mask */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup FMPI2C_Private_Functions
  * @{
  */


/** @defgroup FMPI2C_Group1 Initialization and Configuration functions
 *  @brief   Initialization and Configuration functions 
 *
@verbatim   
 ===============================================================================
           ##### Initialization and Configuration functions #####
 ===============================================================================
    [..] This section provides a set of functions allowing to initialize the FMPI2C Mode,
         FMPI2C Timing, FMPI2C filters, FMPI2C Addressing mode, FMPI2C OwnAddress1.

    [..] The FMPI2C_Init() function follows the FMPI2C configuration procedures (these procedures 
         are available in reference manual).
         
    [..] When the Software Reset is performed using FMPI2C_SoftwareResetCmd() function, the internal
         states machines are reset and communication control bits, as well as status bits come 
         back to their reset value.
         
    [..] Before enabling Stop mode using FMPI2C_StopModeCmd() FMPI2C Clock source must be set to
         HSI and Digital filters must be disabled.
         
    [..] Before enabling Own Address 2 via FMPI2C_DualAddressCmd() function, OA2 and mask should be
         configured using FMPI2C_OwnAddress2Config() function.
         
    [..] FMPI2C_SlaveByteControlCmd() enable Slave byte control that allow user to get control of 
         each byte in slave mode when NBYTES is set to 0x01. 
             
@endverbatim
  * @{
  */

/**
  * @brief  Deinitializes the FMPI2Cx peripheral registers to their default reset values.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @retval None
  */
void FMPI2C_DeInit(FMPI2C_TypeDef* FMPI2Cx)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));

  if (FMPI2Cx == FMPI2C1)
  {
    /* Enable FMPI2C1 reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_FMPI2C1, ENABLE);
    /* Release FMPI2C1 from reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_FMPI2C1, DISABLE);      
  }
}

/**
  * @brief  Initializes the FMPI2Cx peripheral according to the specified
  *         parameters in the FMPI2C_InitStruct.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  FMPI2C_InitStruct: pointer to a FMPI2C_InitTypeDef structure that
  *         contains the configuration information for the specified FMPI2C peripheral.
  * @retval None
  */
void FMPI2C_Init(FMPI2C_TypeDef* FMPI2Cx, FMPI2C_InitTypeDef* FMPI2C_InitStruct)
{
  uint32_t tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FMPI2C_ANALOG_FILTER(FMPI2C_InitStruct->FMPI2C_AnalogFilter));
  assert_param(IS_FMPI2C_DIGITAL_FILTER(FMPI2C_InitStruct->FMPI2C_DigitalFilter));
  assert_param(IS_FMPI2C_MODE(FMPI2C_InitStruct->FMPI2C_Mode));
  assert_param(IS_FMPI2C_OWN_ADDRESS1(FMPI2C_InitStruct->FMPI2C_OwnAddress1));
  assert_param(IS_FMPI2C_ACK(FMPI2C_InitStruct->FMPI2C_Ack));
  assert_param(IS_FMPI2C_ACKNOWLEDGE_ADDRESS(FMPI2C_InitStruct->FMPI2C_AcknowledgedAddress));

  /* Disable FMPI2Cx Peripheral */
  FMPI2Cx->CR1 &= (uint32_t)~((uint32_t)FMPI2C_CR1_PE);

  /*---------------------------- FMPI2Cx FILTERS Configuration ------------------*/
  /* Get the FMPI2Cx CR1 value */
  tmpreg = FMPI2Cx->CR1;
  /* Clear FMPI2Cx CR1 register */
  tmpreg &= CR1_CLEAR_MASK;
  /* Configure FMPI2Cx: analog and digital filter */
  /* Set ANFOFF bit according to FMPI2C_AnalogFilter value */
  /* Set DFN bits according to FMPI2C_DigitalFilter value */
  tmpreg |= (uint32_t)FMPI2C_InitStruct->FMPI2C_AnalogFilter |(FMPI2C_InitStruct->FMPI2C_DigitalFilter << 8);
  
  /* Write to FMPI2Cx CR1 */
  FMPI2Cx->CR1 = tmpreg;

  /*---------------------------- FMPI2Cx TIMING Configuration -------------------*/
  /* Configure FMPI2Cx: Timing */
  /* Set TIMINGR bits according to FMPI2C_Timing */
  /* Write to FMPI2Cx TIMING */
  FMPI2Cx->TIMINGR = FMPI2C_InitStruct->FMPI2C_Timing & TIMING_CLEAR_MASK;

  /* Enable FMPI2Cx Peripheral */
  FMPI2Cx->CR1 |= FMPI2C_CR1_PE;

  /*---------------------------- FMPI2Cx OAR1 Configuration ---------------------*/
  /* Clear tmpreg local variable */
  tmpreg = 0;
  /* Clear OAR1 register */
  FMPI2Cx->OAR1 = (uint32_t)tmpreg;
  /* Clear OAR2 register */
  FMPI2Cx->OAR2 = (uint32_t)tmpreg;
  /* Configure FMPI2Cx: Own Address1 and acknowledged address */
  /* Set OA1MODE bit according to FMPI2C_AcknowledgedAddress value */
  /* Set OA1 bits according to FMPI2C_OwnAddress1 value */
  tmpreg = (uint32_t)((uint32_t)FMPI2C_InitStruct->FMPI2C_AcknowledgedAddress | \
                      (uint32_t)FMPI2C_InitStruct->FMPI2C_OwnAddress1);
  /* Write to FMPI2Cx OAR1 */
  FMPI2Cx->OAR1 = tmpreg;
  /* Enable Own Address1 acknowledgement */
  FMPI2Cx->OAR1 |= FMPI2C_OAR1_OA1EN;

  /*---------------------------- FMPI2Cx MODE Configuration ---------------------*/
  /* Configure FMPI2Cx: mode */
  /* Set SMBDEN and SMBHEN bits according to FMPI2C_Mode value */
  tmpreg = FMPI2C_InitStruct->FMPI2C_Mode;
  /* Write to FMPI2Cx CR1 */
  FMPI2Cx->CR1 |= tmpreg;

  /*---------------------------- FMPI2Cx ACK Configuration ----------------------*/
  /* Get the FMPI2Cx CR2 value */
  tmpreg = FMPI2Cx->CR2;
  /* Clear FMPI2Cx CR2 register */
  tmpreg &= CR2_CLEAR_MASK;
  /* Configure FMPI2Cx: acknowledgement */
  /* Set NACK bit according to FMPI2C_Ack value */
  tmpreg |= FMPI2C_InitStruct->FMPI2C_Ack;
  /* Write to FMPI2Cx CR2 */
  FMPI2Cx->CR2 = tmpreg;
}

/**
  * @brief  Fills each FMPI2C_InitStruct member with its default value.
  * @param  FMPI2C_InitStruct: pointer to an FMPI2C_InitTypeDef structure which will be initialized.
  * @retval None
  */
void FMPI2C_StructInit(FMPI2C_InitTypeDef* FMPI2C_InitStruct)
{
  /*---------------- Reset FMPI2C init structure parameters values --------------*/
  /* Initialize the FMPI2C_Timing member */
  FMPI2C_InitStruct->FMPI2C_Timing = 0;
  /* Initialize the FMPI2C_AnalogFilter member */
  FMPI2C_InitStruct->FMPI2C_AnalogFilter = FMPI2C_AnalogFilter_Enable;
  /* Initialize the FMPI2C_DigitalFilter member */
  FMPI2C_InitStruct->FMPI2C_DigitalFilter = 0;
  /* Initialize the FMPI2C_Mode member */
  FMPI2C_InitStruct->FMPI2C_Mode = FMPI2C_Mode_FMPI2C;
  /* Initialize the FMPI2C_OwnAddress1 member */
  FMPI2C_InitStruct->FMPI2C_OwnAddress1 = 0;
  /* Initialize the FMPI2C_Ack member */
  FMPI2C_InitStruct->FMPI2C_Ack = FMPI2C_Ack_Disable;
  /* Initialize the FMPI2C_AcknowledgedAddress member */
  FMPI2C_InitStruct->FMPI2C_AcknowledgedAddress = FMPI2C_AcknowledgedAddress_7bit;
}

/**
  * @brief  Enables or disables the specified FMPI2C peripheral.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  NewState: new state of the FMPI2Cx peripheral. 
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void FMPI2C_Cmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the selected FMPI2C peripheral */
    FMPI2Cx->CR1 |= FMPI2C_CR1_PE;
  }
  else
  {
    /* Disable the selected FMPI2C peripheral */
    FMPI2Cx->CR1 &= (uint32_t)~((uint32_t)FMPI2C_CR1_PE);
  }
}


/**
  * @brief  Enables or disables the specified FMPI2C software reset.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @retval None
  */
void FMPI2C_SoftwareResetCmd(FMPI2C_TypeDef* FMPI2Cx)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));

  /* Disable peripheral */
  FMPI2Cx->CR1 &= (uint32_t)~((uint32_t)FMPI2C_CR1_PE);

  /* Perform a dummy read to delay the disable of peripheral for minimum
     3 APB clock cycles to perform the software reset functionality */
  *(__IO uint32_t *)(uint32_t)FMPI2Cx; 

  /* Enable peripheral */
  FMPI2Cx->CR1 |= FMPI2C_CR1_PE;
}

/**
  * @brief  Enables or disables the specified FMPI2C interrupts.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  FMPI2C_IT: specifies the FMPI2C interrupts sources to be enabled or disabled. 
  *   This parameter can be any combination of the following values:
  *     @arg FMPI2C_IT_ERRI: Error interrupt mask
  *     @arg FMPI2C_IT_TCI: Transfer Complete interrupt mask
  *     @arg FMPI2C_IT_STOPI: Stop Detection interrupt mask
  *     @arg FMPI2C_IT_NACKI: Not Acknowledge received interrupt mask
  *     @arg FMPI2C_IT_ADDRI: Address Match interrupt mask  
  *     @arg FMPI2C_IT_RXI: RX interrupt mask
  *     @arg FMPI2C_IT_TXI: TX interrupt mask
  * @param  NewState: new state of the specified FMPI2C interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void FMPI2C_ITConfig(FMPI2C_TypeDef* FMPI2Cx, uint32_t FMPI2C_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  assert_param(IS_FMPI2C_CONFIG_IT(FMPI2C_IT));
  
  if (NewState != DISABLE)
  {
    /* Enable the selected FMPI2C interrupts */
    FMPI2Cx->CR1 |= FMPI2C_IT;
  }
  else
  {
    /* Disable the selected FMPI2C interrupts */
    FMPI2Cx->CR1 &= (uint32_t)~((uint32_t)FMPI2C_IT);
  }
}

/**
  * @brief  Enables or disables the FMPI2C Clock stretching.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  NewState: new state of the FMPI2Cx Clock stretching.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void FMPI2C_StretchClockCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable clock stretching */
    FMPI2Cx->CR1 &= (uint32_t)~((uint32_t)FMPI2C_CR1_NOSTRETCH);    
  }
  else
  {
    /* Disable clock stretching  */
    FMPI2Cx->CR1 |= FMPI2C_CR1_NOSTRETCH;
  }
}

/**
  * @brief  Enables or disables FMPI2Cp from stop mode.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  NewState: new state of the FMPI2Cx stop mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void FMPI2C_StopModeCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable wakeup from stop mode */
    FMPI2Cx->CR1 |= FMPI2C_CR1_WUPEN;   
  }
  else
  {
    /* Disable wakeup from stop mode */    
    FMPI2Cx->CR1 &= (uint32_t)~((uint32_t)FMPI2C_CR1_WUPEN); 
  }
}

/**
  * @brief  Enables or disables the FMPI2C own address 2.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  NewState: new state of the FMPI2C own address 2.
  *   This parameter can be: ENABLE or DISABLE.  
  * @retval None
  */
void FMPI2C_DualAddressCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable own address 2 */
    FMPI2Cx->OAR2 |= FMPI2C_OAR2_OA2EN;
  }
  else
  {
    /* Disable own address 2 */
    FMPI2Cx->OAR2 &= (uint32_t)~((uint32_t)FMPI2C_OAR2_OA2EN);
  }
}    

/**
  * @brief  Configures the FMPI2C slave own address 2 and mask.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  Address: specifies the slave address to be programmed.
  * @param  Mask: specifies own address 2 mask to be programmed.
  *   This parameter can be one of the following values:
  *     @arg FMPI2C_OA2_NoMask: no mask.
  *     @arg FMPI2C_OA2_Mask01: OA2[1] is masked and don't care.
  *     @arg FMPI2C_OA2_Mask02: OA2[2:1] are masked and don't care.
  *     @arg FMPI2C_OA2_Mask03: OA2[3:1] are masked and don't care.
  *     @arg FMPI2C_OA2_Mask04: OA2[4:1] are masked and don't care.
  *     @arg FMPI2C_OA2_Mask05: OA2[5:1] are masked and don't care.
  *     @arg FMPI2C_OA2_Mask06: OA2[6:1] are masked and don't care.
  *     @arg FMPI2C_OA2_Mask07: OA2[7:1] are masked and don't care.
  * @retval None
  */
void FMPI2C_OwnAddress2Config(FMPI2C_TypeDef* FMPI2Cx, uint16_t Address, uint8_t Mask)
{
  uint32_t tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FMPI2C_OWN_ADDRESS2(Address));
  assert_param(IS_FMPI2C_OWN_ADDRESS2_MASK(Mask));
  
  /* Get the old register value */
  tmpreg = FMPI2Cx->OAR2;

  /* Reset FMPI2Cx OA2 bit [7:1] and OA2MSK bit [1:0]  */
  tmpreg &= (uint32_t)~((uint32_t)(FMPI2C_OAR2_OA2 | FMPI2C_OAR2_OA2MSK));

  /* Set FMPI2Cx SADD */
  tmpreg |= (uint32_t)(((uint32_t)Address & FMPI2C_OAR2_OA2) | \
            (((uint32_t)Mask << 8) & FMPI2C_OAR2_OA2MSK)) ;

  /* Store the new register value */
  FMPI2Cx->OAR2 = tmpreg;
}

/**
  * @brief  Enables or disables the FMPI2C general call mode.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  NewState: new state of the FMPI2C general call mode.
  *   This parameter can be: ENABLE or DISABLE.  
  * @retval None
  */
void FMPI2C_GeneralCallCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable general call mode */
    FMPI2Cx->CR1 |= FMPI2C_CR1_GCEN;
  }
  else
  {
    /* Disable general call mode */
    FMPI2Cx->CR1 &= (uint32_t)~((uint32_t)FMPI2C_CR1_GCEN);
  }
} 

/**
  * @brief  Enables or disables the FMPI2C slave byte control.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  NewState: new state of the FMPI2C slave byte control.
  *   This parameter can be: ENABLE or DISABLE.  
  * @retval None
  */
void FMPI2C_SlaveByteControlCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable slave byte control */
    FMPI2Cx->CR1 |= FMPI2C_CR1_SBC;
  }
  else
  {
    /* Disable slave byte control */
    FMPI2Cx->CR1 &= (uint32_t)~((uint32_t)FMPI2C_CR1_SBC);
  }
}

/**
  * @brief  Configures the slave address to be transmitted after start generation.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  Address: specifies the slave address to be programmed.
  * @note   This function should be called before generating start condition.  
  * @retval None
  */
void FMPI2C_SlaveAddressConfig(FMPI2C_TypeDef* FMPI2Cx, uint16_t Address)
{
  uint32_t tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FMPI2C_SLAVE_ADDRESS(Address));
               
  /* Get the old register value */
  tmpreg = FMPI2Cx->CR2;

  /* Reset FMPI2Cx SADD bit [9:0] */
  tmpreg &= (uint32_t)~((uint32_t)FMPI2C_CR2_SADD);

  /* Set FMPI2Cx SADD */
  tmpreg |= (uint32_t)((uint32_t)Address & FMPI2C_CR2_SADD);

  /* Store the new register value */
  FMPI2Cx->CR2 = tmpreg;
}
  
/**
  * @brief  Enables or disables the FMPI2C 10-bit addressing mode for the master.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  NewState: new state of the FMPI2C 10-bit addressing mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @note   This function should be called before generating start condition.  
  * @retval None
  */
void FMPI2C_10BitAddressingModeCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable 10-bit addressing mode */
    FMPI2Cx->CR2 |= FMPI2C_CR2_ADD10;
  }
  else
  {
    /* Disable 10-bit addressing mode */
    FMPI2Cx->CR2 &= (uint32_t)~((uint32_t)FMPI2C_CR2_ADD10);
  }
} 

/**
  * @}
  */


/** @defgroup FMPI2C_Group2 Communications handling functions
 *  @brief   Communications handling functions 
 *
@verbatim
 ===============================================================================
                  ##### Communications handling functions #####
 ===============================================================================  
    [..] This section provides a set of functions that handles FMPI2C communication.
    
    [..] Automatic End mode is enabled using FMPI2C_AutoEndCmd() function. When Reload
         mode is enabled via FMPI2C_ReloadCmd() AutoEnd bit has no effect.
         
    [..] FMPI2C_NumberOfBytesConfig() function set the number of bytes to be transferred,
         this configuration should be done before generating start condition in master 
         mode.
         
    [..] When switching from master write operation to read operation in 10Bit addressing
         mode, master can only sends the 1st 7 bits of the 10 bit address, followed by 
         Read direction by enabling HEADR bit using FMPI2C_10BitAddressHeader() function.        
         
    [..] In master mode, when transferring more than 255 bytes Reload mode should be used
         to handle communication. In the first phase of transfer, Nbytes should be set to 
         255. After transferring these bytes TCR flag is set and FMPI2C_TransferHandling()
         function should be called to handle remaining communication.
         
    [..] In master mode, when software end mode is selected when all data is transferred
         TC flag is set FMPI2C_TransferHandling() function should be called to generate STOP
         or generate ReStart.                      
             
@endverbatim
  * @{
  */
  
/**
  * @brief  Enables or disables the FMPI2C automatic end mode (stop condition is 
  *         automatically sent when nbytes data are transferred).
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  NewState: new state of the FMPI2C automatic end mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @note   This function has effect if Reload mode is disabled.   
  * @retval None
  */
void FMPI2C_AutoEndCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable Auto end mode */
    FMPI2Cx->CR2 |= FMPI2C_CR2_AUTOEND;
  }
  else
  {
    /* Disable Auto end mode */
    FMPI2Cx->CR2 &= (uint32_t)~((uint32_t)FMPI2C_CR2_AUTOEND);
  }
} 

/**
  * @brief  Enables or disables the FMPI2C nbytes reload mode.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  NewState: new state of the nbytes reload mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void FMPI2C_ReloadCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable Auto Reload mode */
    FMPI2Cx->CR2 |= FMPI2C_CR2_RELOAD;
  }
  else
  {
    /* Disable Auto Reload mode */
    FMPI2Cx->CR2 &= (uint32_t)~((uint32_t)FMPI2C_CR2_RELOAD);
  }
}

/**
  * @brief  Configures the number of bytes to be transmitted/received.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  Number_Bytes: specifies the number of bytes to be programmed.
  * @retval None
  */
void FMPI2C_NumberOfBytesConfig(FMPI2C_TypeDef* FMPI2Cx, uint8_t Number_Bytes)
{
  uint32_t tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));

  /* Get the old register value */
  tmpreg = FMPI2Cx->CR2;

  /* Reset FMPI2Cx Nbytes bit [7:0] */
  tmpreg &= (uint32_t)~((uint32_t)FMPI2C_CR2_NBYTES);

  /* Set FMPI2Cx Nbytes */
  tmpreg |= (uint32_t)(((uint32_t)Number_Bytes << 16 ) & FMPI2C_CR2_NBYTES);

  /* Store the new register value */
  FMPI2Cx->CR2 = tmpreg;
}  
  
/**
  * @brief  Configures the type of transfer request for the master.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  FMPI2C_Direction: specifies the transfer request direction to be programmed.
  *    This parameter can be one of the following values:
  *     @arg FMPI2C_Direction_Transmitter: Master request a write transfer
  *     @arg FMPI2C_Direction_Receiver: Master request a read transfer 
  * @retval None
  */
void FMPI2C_MasterRequestConfig(FMPI2C_TypeDef* FMPI2Cx, uint16_t FMPI2C_Direction)
{
/* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FMPI2C_DIRECTION(FMPI2C_Direction));
  
  /* Test on the direction to set/reset the read/write bit */
  if (FMPI2C_Direction == FMPI2C_Direction_Transmitter)
  {
    /* Request a write Transfer */
    FMPI2Cx->CR2 &= (uint32_t)~((uint32_t)FMPI2C_CR2_RD_WRN);
  }
  else
  {
    /* Request a read Transfer */
    FMPI2Cx->CR2 |= FMPI2C_CR2_RD_WRN;
  }
}  
  
/**
  * @brief  Generates FMPI2Cx communication START condition.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  NewState: new state of the FMPI2C START condition generation.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void FMPI2C_GenerateSTART(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Generate a START condition */
    FMPI2Cx->CR2 |= FMPI2C_CR2_START;
  }
  else
  {
    /* Disable the START condition generation */
    FMPI2Cx->CR2 &= (uint32_t)~((uint32_t)FMPI2C_CR2_START);
  }
}  
  
/**
  * @brief  Generates FMPI2Cx communication STOP condition.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  NewState: new state of the FMPI2C STOP condition generation.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void FMPI2C_GenerateSTOP(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Generate a STOP condition */
    FMPI2Cx->CR2 |= FMPI2C_CR2_STOP;
  }
  else
  {
    /* Disable the STOP condition generation */
    FMPI2Cx->CR2 &= (uint32_t)~((uint32_t)FMPI2C_CR2_STOP);
  }
}  

/**
  * @brief  Enables or disables the FMPI2C 10-bit header only mode with read direction.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  NewState: new state of the FMPI2C 10-bit header only mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @note   This mode can be used only when switching from master transmitter mode 
  *         to master receiver mode.        
  * @retval None
  */
void FMPI2C_10BitAddressHeaderCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable 10-bit header only mode */
    FMPI2Cx->CR2 |= FMPI2C_CR2_HEAD10R;
  }
  else
  {
    /* Disable 10-bit header only mode */
    FMPI2Cx->CR2 &= (uint32_t)~((uint32_t)FMPI2C_CR2_HEAD10R);
  }
}    

/**
  * @brief  Generates FMPI2C communication Acknowledge.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  NewState: new state of the Acknowledge.
  *   This parameter can be: ENABLE or DISABLE.  
  * @retval None
  */
void FMPI2C_AcknowledgeConfig(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable ACK generation */
    FMPI2Cx->CR2 &= (uint32_t)~((uint32_t)FMPI2C_CR2_NACK);    
  }
  else
  {
    /* Enable NACK generation */
    FMPI2Cx->CR2 |= FMPI2C_CR2_NACK;
  }
}

/**
  * @brief  Returns the FMPI2C slave matched address .
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @retval The value of the slave matched address .
  */
uint8_t FMPI2C_GetAddressMatched(FMPI2C_TypeDef* FMPI2Cx)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  
  /* Return the slave matched address in the SR1 register */
  return (uint8_t)(((uint32_t)FMPI2Cx->ISR & FMPI2C_ISR_ADDCODE) >> 16) ;
}

/**
  * @brief  Returns the FMPI2C slave received request.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @retval The value of the received request.
  */
uint16_t FMPI2C_GetTransferDirection(FMPI2C_TypeDef* FMPI2Cx)
{
  uint32_t tmpreg = 0;
  uint16_t direction = 0;
  
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  
  /* Return the slave matched address in the SR1 register */
  tmpreg = (uint32_t)(FMPI2Cx->ISR & FMPI2C_ISR_DIR);
  
  /* If write transfer is requested */
  if (tmpreg == 0)
  {
    /* write transfer is requested */
    direction = FMPI2C_Direction_Transmitter;
  }
  else
  {
    /* Read transfer is requested */
    direction = FMPI2C_Direction_Receiver;
  }  
  return direction;
}

/**
  * @brief  Handles FMPI2Cx communication when starting transfer or during transfer (TC or TCR flag are set).
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  Address: specifies the slave address to be programmed.
  * @param  Number_Bytes: specifies the number of bytes to be programmed.
  *   This parameter must be a value between 0 and 255.
  * @param  ReloadEndMode: new state of the FMPI2C START condition generation.
  *   This parameter can be one of the following values:
  *     @arg FMPI2C_Reload_Mode: Enable Reload mode .
  *     @arg FMPI2C_AutoEnd_Mode: Enable Automatic end mode.
  *     @arg FMPI2C_SoftEnd_Mode: Enable Software end mode.
  * @param  StartStopMode: new state of the FMPI2C START condition generation.
  *   This parameter can be one of the following values:
  *     @arg FMPI2C_No_StartStop: Don't Generate stop and start condition.
  *     @arg FMPI2C_Generate_Stop: Generate stop condition (Number_Bytes should be set to 0).
  *     @arg FMPI2C_Generate_Start_Read: Generate Restart for read request.
  *     @arg FMPI2C_Generate_Start_Write: Generate Restart for write request.
  * @retval None
  */
void FMPI2C_TransferHandling(FMPI2C_TypeDef* FMPI2Cx, uint16_t Address, uint8_t Number_Bytes, uint32_t ReloadEndMode, uint32_t StartStopMode)
{
  uint32_t tmpreg = 0;
  
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FMPI2C_SLAVE_ADDRESS(Address));  
  assert_param(IS_RELOAD_END_MODE(ReloadEndMode));
  assert_param(IS_START_STOP_MODE(StartStopMode));
    
  /* Get the CR2 register value */
  tmpreg = FMPI2Cx->CR2;
  
  /* clear tmpreg specific bits */
  tmpreg &= (uint32_t)~((uint32_t)(FMPI2C_CR2_SADD | FMPI2C_CR2_NBYTES | FMPI2C_CR2_RELOAD | FMPI2C_CR2_AUTOEND | FMPI2C_CR2_RD_WRN | FMPI2C_CR2_START | FMPI2C_CR2_STOP));
  
  /* update tmpreg */
  tmpreg |= (uint32_t)(((uint32_t)Address & FMPI2C_CR2_SADD) | (((uint32_t)Number_Bytes << 16 ) & FMPI2C_CR2_NBYTES) | \
            (uint32_t)ReloadEndMode | (uint32_t)StartStopMode);
  
  /* update CR2 register */
  FMPI2Cx->CR2 = tmpreg;  
}  

/**
  * @}
  */


/** @defgroup FMPI2C_Group3 SMBUS management functions
 *  @brief   SMBUS management functions 
 *
@verbatim
 ===============================================================================
                      ##### SMBUS management functions #####
 ===============================================================================   
    [..] This section provides a set of functions that handles SMBus communication
         and timeouts detection.
    
    [..] The SMBus Device default address (0b1100 001) is enabled by calling FMPI2C_Init()
         function and setting FMPI2C_Mode member of FMPI2C_InitTypeDef() structure to 
         FMPI2C_Mode_SMBusDevice.
         
    [..] The SMBus Host address (0b0001 000) is enabled by calling FMPI2C_Init()
         function and setting FMPI2C_Mode member of FMPI2C_InitTypeDef() structure to 
         FMPI2C_Mode_SMBusHost.         
         
    [..] The Alert Response Address (0b0001 100) is enabled using FMPI2C_SMBusAlertCmd()
         function.
         
    [..] To detect cumulative SCL stretch in master and slave mode, TIMEOUTB should be 
         configured (in accordance to SMBus specification) using FMPI2C_TimeoutBConfig() 
         function then FMPI2C_ExtendedClockTimeoutCmd() function should be called to enable
         the detection.
         
    [..] SCL low timeout is detected by configuring TIMEOUTB using FMPI2C_TimeoutBConfig()
         function followed by the call of FMPI2C_ClockTimeoutCmd(). When adding to this 
         procedure the call of FMPI2C_IdleClockTimeoutCmd() function, Bus Idle condition 
         (both SCL and SDA high) is detected also.                
                          
@endverbatim
  * @{
  */

/**
  * @brief  Enables or disables FMPI2C SMBus alert.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  NewState: new state of the FMPI2Cx SMBus alert.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void FMPI2C_SMBusAlertCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable SMBus alert */
    FMPI2Cx->CR1 |= FMPI2C_CR1_ALERTEN;   
  }
  else
  {
    /* Disable SMBus alert */    
    FMPI2Cx->CR1 &= (uint32_t)~((uint32_t)FMPI2C_CR1_ALERTEN); 
  }
}

/**
  * @brief  Enables or disables FMPI2C Clock Timeout (SCL Timeout detection).
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  NewState: new state of the FMPI2Cx clock Timeout.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void FMPI2C_ClockTimeoutCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable Clock Timeout */
    FMPI2Cx->TIMEOUTR |= FMPI2C_TIMEOUTR_TIMOUTEN;   
  }
  else
  {
    /* Disable Clock Timeout */    
    FMPI2Cx->TIMEOUTR &= (uint32_t)~((uint32_t)FMPI2C_TIMEOUTR_TIMOUTEN); 
  }
}

/**
  * @brief  Enables or disables FMPI2C Extended Clock Timeout (SCL cumulative Timeout detection).
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  NewState: new state of the FMPI2Cx Extended clock Timeout.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void FMPI2C_ExtendedClockTimeoutCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable Clock Timeout */
    FMPI2Cx->TIMEOUTR |= FMPI2C_TIMEOUTR_TEXTEN;   
  }
  else
  {
    /* Disable Clock Timeout */    
    FMPI2Cx->TIMEOUTR &= (uint32_t)~((uint32_t)FMPI2C_TIMEOUTR_TEXTEN); 
  }
}

/**
  * @brief  Enables or disables FMPI2C Idle Clock Timeout (Bus idle SCL and SDA 
  *         high detection).
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  NewState: new state of the FMPI2Cx Idle clock Timeout.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void FMPI2C_IdleClockTimeoutCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable Clock Timeout */
    FMPI2Cx->TIMEOUTR |= FMPI2C_TIMEOUTR_TIDLE;   
  }
  else
  {
    /* Disable Clock Timeout */    
    FMPI2Cx->TIMEOUTR &= (uint32_t)~((uint32_t)FMPI2C_TIMEOUTR_TIDLE); 
  }
}

/**
  * @brief  Configures the FMPI2C Bus Timeout A (SCL Timeout when TIDLE = 0 or Bus 
  *   idle SCL and SDA high when TIDLE = 1).
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  Timeout: specifies the TimeoutA to be programmed. 
  * @retval None
  */
void FMPI2C_TimeoutAConfig(FMPI2C_TypeDef* FMPI2Cx, uint16_t Timeout)
{
  uint32_t tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FMPI2C_TIMEOUT(Timeout));
    
  /* Get the old register value */
  tmpreg = FMPI2Cx->TIMEOUTR;

  /* Reset FMPI2Cx TIMEOUTA bit [11:0] */
  tmpreg &= (uint32_t)~((uint32_t)FMPI2C_TIMEOUTR_TIMEOUTA);

  /* Set FMPI2Cx TIMEOUTA */
  tmpreg |= (uint32_t)((uint32_t)Timeout & FMPI2C_TIMEOUTR_TIMEOUTA) ;

  /* Store the new register value */
  FMPI2Cx->TIMEOUTR = tmpreg;
}

/**
  * @brief  Configures the FMPI2C Bus Timeout B (SCL cumulative Timeout).
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  Timeout: specifies the TimeoutB to be programmed. 
  * @retval None
  */
void FMPI2C_TimeoutBConfig(FMPI2C_TypeDef* FMPI2Cx, uint16_t Timeout)
{
  uint32_t tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FMPI2C_TIMEOUT(Timeout));

  /* Get the old register value */
  tmpreg = FMPI2Cx->TIMEOUTR;

  /* Reset FMPI2Cx TIMEOUTB bit [11:0] */
  tmpreg &= (uint32_t)~((uint32_t)FMPI2C_TIMEOUTR_TIMEOUTB);

  /* Set FMPI2Cx TIMEOUTB */
  tmpreg |= (uint32_t)(((uint32_t)Timeout << 16) & FMPI2C_TIMEOUTR_TIMEOUTB) ;

  /* Store the new register value */
  FMPI2Cx->TIMEOUTR = tmpreg;
}

/**
  * @brief  Enables or disables FMPI2C PEC calculation.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  NewState: new state of the FMPI2Cx PEC calculation.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void FMPI2C_CalculatePEC(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable PEC calculation */
    FMPI2Cx->CR1 |= FMPI2C_CR1_PECEN;   
  }
  else
  {
    /* Disable PEC calculation */    
    FMPI2Cx->CR1 &= (uint32_t)~((uint32_t)FMPI2C_CR1_PECEN); 
  }
}

/**
  * @brief  Enables or disables FMPI2C PEC transmission/reception request.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  NewState: new state of the FMPI2Cx PEC request.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void FMPI2C_PECRequestCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable PEC transmission/reception request */
    FMPI2Cx->CR1 |= FMPI2C_CR2_PECBYTE;   
  }
  else
  {
    /* Disable PEC transmission/reception request */    
    FMPI2Cx->CR1 &= (uint32_t)~((uint32_t)FMPI2C_CR2_PECBYTE); 
  }
}

/**
  * @brief  Returns the FMPI2C PEC.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @retval The value of the PEC .
  */
uint8_t FMPI2C_GetPEC(FMPI2C_TypeDef* FMPI2Cx)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  
  /* Return the slave matched address in the SR1 register */
  return (uint8_t)((uint32_t)FMPI2Cx->PECR & FMPI2C_PECR_PEC);
}

/**
  * @}
  */  
  
  
/** @defgroup FMPI2C_Group4 FMPI2C registers management functions
 *  @brief   FMPI2C registers management functions 
 *
@verbatim
 ===============================================================================
                ##### FMPI2C registers management functions #####
 ===============================================================================  
    [..] This section provides a functions that allow user the management of 
         FMPI2C registers.
         
@endverbatim
  * @{
  */

  /**
  * @brief  Reads the specified FMPI2C register and returns its value.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  FMPI2C_Register: specifies the register to read.
  *   This parameter can be one of the following values:
  *     @arg FMPI2C_Register_CR1: CR1 register.
  *     @arg FMPI2C_Register_CR2: CR2 register.
  *     @arg FMPI2C_Register_OAR1: OAR1 register.
  *     @arg FMPI2C_Register_OAR2: OAR2 register.
  *     @arg FMPI2C_Register_TIMINGR: TIMING register.
  *     @arg FMPI2C_Register_TIMEOUTR: TIMEOUTR register.
  *     @arg FMPI2C_Register_ISR: ISR register.
  *     @arg FMPI2C_Register_ICR: ICR register.
  *     @arg FMPI2C_Register_PECR: PECR register.
  *     @arg FMPI2C_Register_RXDR: RXDR register.
  *     @arg FMPI2C_Register_TXDR: TXDR register.
  * @retval The value of the read register.
  */
uint32_t FMPI2C_ReadRegister(FMPI2C_TypeDef* FMPI2Cx, uint8_t FMPI2C_Register)
{
  __IO uint32_t tmp = 0;

  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FMPI2C_REGISTER(FMPI2C_Register));

  tmp = (uint32_t)FMPI2Cx;
  tmp += FMPI2C_Register;

  /* Return the selected register value */
  return (*(__IO uint32_t *) tmp);
}

/**
  * @}
  */  
  
/** @defgroup FMPI2C_Group5 Data transfers management functions
 *  @brief   Data transfers management functions 
 *
@verbatim
 ===============================================================================
                ##### Data transfers management functions #####
 =============================================================================== 
    [..] This subsection provides a set of functions allowing to manage 
         the FMPI2C data transfers.
         
    [..] The read access of the FMPI2C_RXDR register can be done using 
         the FMPI2C_ReceiveData() function and returns the received value.
         Whereas a write access to the FMPI2C_TXDR can be done using FMPI2C_SendData()
         function and stores the written data into TXDR.
@endverbatim
  * @{
  */  
  
/**
  * @brief  Sends a data byte through the FMPI2Cx peripheral.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  Data: Byte to be transmitted..
  * @retval None
  */
void FMPI2C_SendData(FMPI2C_TypeDef* FMPI2Cx, uint8_t Data)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  
  /* Write in the DR register the data to be sent */
  FMPI2Cx->TXDR = (uint8_t)Data;
}

/**
  * @brief  Returns the most recent received data by the FMPI2Cx peripheral.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @retval The value of the received data.
  */
uint8_t FMPI2C_ReceiveData(FMPI2C_TypeDef* FMPI2Cx)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  
  /* Return the data in the DR register */
  return (uint8_t)FMPI2Cx->RXDR;
}  

/**
  * @}
  */ 
  
  
/** @defgroup FMPI2C_Group6 DMA transfers management functions
 *  @brief   DMA transfers management functions 
 *
@verbatim
 ===============================================================================
               ##### DMA transfers management functions #####
 ===============================================================================  
    [..] This section provides two functions that can be used only in DMA mode.
    [..] In DMA Mode, the FMPI2C communication can be managed by 2 DMA Channel 
         requests:
         (#) FMPI2C_DMAReq_Tx: specifies the Tx buffer DMA transfer request.
         (#) FMPI2C_DMAReq_Rx: specifies the Rx buffer DMA transfer request.
    [..] In this Mode it is advised to use the following function:
         (+) FMPI2C_DMACmd(FMPI2C_TypeDef* FMPI2Cx, uint32_t FMPI2C_DMAReq, FunctionalState NewState);
@endverbatim
  * @{
  */  
    
/**
  * @brief  Enables or disables the FMPI2C DMA interface.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  FMPI2C_DMAReq: specifies the FMPI2C DMA transfer request to be enabled or disabled. 
  *   This parameter can be any combination of the following values:
  *     @arg FMPI2C_DMAReq_Tx: Tx DMA transfer request
  *     @arg FMPI2C_DMAReq_Rx: Rx DMA transfer request
  * @param  NewState: new state of the selected FMPI2C DMA transfer request.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void FMPI2C_DMACmd(FMPI2C_TypeDef* FMPI2Cx, uint32_t FMPI2C_DMAReq, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  assert_param(IS_FMPI2C_DMA_REQ(FMPI2C_DMAReq));

  if (NewState != DISABLE)
  {
    /* Enable the selected FMPI2C DMA requests */
    FMPI2Cx->CR1 |= FMPI2C_DMAReq;
  }
  else
  {
    /* Disable the selected FMPI2C DMA requests */
    FMPI2Cx->CR1 &= (uint32_t)~FMPI2C_DMAReq;
  }
}
/**
  * @}
  */  


/** @defgroup FMPI2C_Group7 Interrupts and flags management functions
 *  @brief   Interrupts and flags management functions 
 *
@verbatim
 ===============================================================================
             ##### Interrupts and flags management functions  #####
 =============================================================================== 
    [..] This section provides functions allowing to configure the FMPI2C Interrupts 
         sources and check or clear the flags or pending bits status.
         The user should identify which mode will be used in his application to manage 
         the communication: Polling mode, Interrupt mode or DMA mode(refer FMPI2C_Group6) .

  *** Polling Mode ***
  ====================
    [..] In Polling Mode, the FMPI2C communication can be managed by 15 flags:
        (#) FMPI2C_FLAG_TXE: to indicate the status of Transmit data register empty flag.
        (#) FMPI2C_FLAG_TXIS: to indicate the status of Transmit interrupt status flag .
        (#) FMPI2C_FLAG_RXNE: to indicate the status of Receive data register not empty flag.
        (#) FMPI2C_FLAG_ADDR: to indicate the status of Address matched flag (slave mode).
        (#) FMPI2C_FLAG_NACKF: to indicate the status of NACK received flag.
        (#) FMPI2C_FLAG_STOPF: to indicate the status of STOP detection flag.
        (#) FMPI2C_FLAG_TC: to indicate the status of Transfer complete flag(master mode).
        (#) FMPI2C_FLAG_TCR: to indicate the status of Transfer complete reload flag.
        (#) FMPI2C_FLAG_BERR: to indicate the status of Bus error flag.
        (#) FMPI2C_FLAG_ARLO: to indicate the status of Arbitration lost flag.
        (#) FMPI2C_FLAG_OVR: to indicate the status of Overrun/Underrun flag.
        (#) FMPI2C_FLAG_PECERR: to indicate the status of PEC error in reception flag.
        (#) FMPI2C_FLAG_TIMEOUT: to indicate the status of Timeout or Tlow detection flag.
        (#) FMPI2C_FLAG_ALERT: to indicate the status of SMBus Alert flag.
        (#) FMPI2C_FLAG_BUSY: to indicate the status of Bus busy flag.

    [..] In this Mode it is advised to use the following functions:
        (+) FlagStatus FMPI2C_GetFlagStatus(FMPI2C_TypeDef* FMPI2Cx, uint32_t FMPI2C_FLAG);
        (+) void FMPI2C_ClearFlag(FMPI2C_TypeDef* FMPI2Cx, uint32_t FMPI2C_FLAG);

    [..]
        (@)Do not use the BUSY flag to handle each data transmission or reception.It is 
           better to use the TXIS and RXNE flags instead.

  *** Interrupt Mode ***
  ======================
    [..] In Interrupt Mode, the FMPI2C communication can be managed by 7 interrupt sources
         and 15 pending bits: 
    [..] Interrupt Source:
        (#) FMPI2C_IT_ERRI: specifies the interrupt source for the Error interrupt.
        (#) FMPI2C_IT_TCI: specifies the interrupt source for the Transfer Complete interrupt.
        (#) FMPI2C_IT_STOPI: specifies the interrupt source for the Stop Detection interrupt.
        (#) FMPI2C_IT_NACKI: specifies the interrupt source for the Not Acknowledge received interrupt.
        (#) FMPI2C_IT_ADDRI: specifies the interrupt source for the Address Match interrupt.  
        (#) FMPI2C_IT_RXI: specifies the interrupt source for the RX interrupt.
        (#) FMPI2C_IT_TXI: specifies the interrupt source for the TX interrupt.

    [..] Pending Bits:
        (#) FMPI2C_IT_TXIS: to indicate the status of Transmit interrupt status flag.
        (#) FMPI2C_IT_RXNE: to indicate the status of Receive data register not empty flag.
        (#) FMPI2C_IT_ADDR: to indicate the status of Address matched flag (slave mode).
        (#) FMPI2C_IT_NACKF: to indicate the status of NACK received flag.
        (#) FMPI2C_IT_STOPF: to indicate the status of STOP detection flag.
        (#) FMPI2C_IT_TC: to indicate the status of Transfer complete flag (master mode).
        (#) FMPI2C_IT_TCR: to indicate the status of Transfer complete reload flag.
        (#) FMPI2C_IT_BERR: to indicate the status of Bus error flag.
        (#) FMPI2C_IT_ARLO: to indicate the status of Arbitration lost flag.
        (#) FMPI2C_IT_OVR: to indicate the status of Overrun/Underrun flag.
        (#) FMPI2C_IT_PECERR: to indicate the status of PEC error in reception flag.
        (#) FMPI2C_IT_TIMEOUT: to indicate the status of Timeout or Tlow detection flag.
        (#) FMPI2C_IT_ALERT: to indicate the status of SMBus Alert flag.

    [..] In this Mode it is advised to use the following functions:
         (+) void FMPI2C_ClearITPendingBit(FMPI2C_TypeDef* FMPI2Cx, uint32_t FMPI2C_IT);
         (+) ITStatus FMPI2C_GetITStatus(FMPI2C_TypeDef* FMPI2Cx, uint32_t FMPI2C_IT);

@endverbatim
  * @{
  */  

/**
  * @brief  Checks whether the specified FMPI2C flag is set or not.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  FMPI2C_FLAG: specifies the flag to check. 
  *   This parameter can be one of the following values:
  *     @arg FMPI2C_FLAG_TXE: Transmit data register empty
  *     @arg FMPI2C_FLAG_TXIS: Transmit interrupt status
  *     @arg FMPI2C_FLAG_RXNE: Receive data register not empty
  *     @arg FMPI2C_FLAG_ADDR: Address matched (slave mode)
  *     @arg FMPI2C_FLAG_NACKF: NACK received flag
  *     @arg FMPI2C_FLAG_STOPF: STOP detection flag
  *     @arg FMPI2C_FLAG_TC: Transfer complete (master mode)
  *     @arg FMPI2C_FLAG_TCR: Transfer complete reload
  *     @arg FMPI2C_FLAG_BERR: Bus error
  *     @arg FMPI2C_FLAG_ARLO: Arbitration lost
  *     @arg FMPI2C_FLAG_OVR: Overrun/Underrun
  *     @arg FMPI2C_FLAG_PECERR: PEC error in reception
  *     @arg FMPI2C_FLAG_TIMEOUT: Timeout or Tlow detection flag
  *     @arg FMPI2C_FLAG_ALERT: SMBus Alert
  *     @arg FMPI2C_FLAG_BUSY: Bus busy
  * @retval The new state of FMPI2C_FLAG (SET or RESET).
  */
FlagStatus FMPI2C_GetFlagStatus(FMPI2C_TypeDef* FMPI2Cx, uint32_t FMPI2C_FLAG)
{
  uint32_t tmpreg = 0;
  FlagStatus bitstatus = RESET;
  
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FMPI2C_GET_FLAG(FMPI2C_FLAG));
  
  /* Get the ISR register value */
  tmpreg = FMPI2Cx->ISR;
  
  /* Get flag status */
  tmpreg &= FMPI2C_FLAG;
  
  if(tmpreg != 0)
  {
    /* FMPI2C_FLAG is set */
    bitstatus = SET;
  }
  else
  {
    /* FMPI2C_FLAG is reset */
    bitstatus = RESET;
  }
  return bitstatus;
}  
  
/**
  * @brief  Clears the FMPI2Cx's pending flags.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  FMPI2C_FLAG: specifies the flag to clear. 
  *   This parameter can be any combination of the following values:
  *     @arg FMPI2C_FLAG_ADDR: Address matched (slave mode)
  *     @arg FMPI2C_FLAG_NACKF: NACK received flag
  *     @arg FMPI2C_FLAG_STOPF: STOP detection flag
  *     @arg FMPI2C_FLAG_BERR: Bus error
  *     @arg FMPI2C_FLAG_ARLO: Arbitration lost
  *     @arg FMPI2C_FLAG_OVR: Overrun/Underrun
  *     @arg FMPI2C_FLAG_PECERR: PEC error in reception
  *     @arg FMPI2C_FLAG_TIMEOUT: Timeout or Tlow detection flag
  *     @arg FMPI2C_FLAG_ALERT: SMBus Alert
  * @retval The new state of FMPI2C_FLAG (SET or RESET).
  */
void FMPI2C_ClearFlag(FMPI2C_TypeDef* FMPI2Cx, uint32_t FMPI2C_FLAG)
{ 
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FMPI2C_CLEAR_FLAG(FMPI2C_FLAG));

  /* Clear the selected flag */
  FMPI2Cx->ICR = FMPI2C_FLAG;
  }

/**
  * @brief  Checks whether the specified FMPI2C interrupt has occurred or not.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  FMPI2C_IT: specifies the interrupt source to check.
  *   This parameter can be one of the following values:
  *     @arg FMPI2C_IT_TXIS: Transmit interrupt status
  *     @arg FMPI2C_IT_RXNE: Receive data register not empty
  *     @arg FMPI2C_IT_ADDR: Address matched (slave mode)
  *     @arg FMPI2C_IT_NACKF: NACK received flag
  *     @arg FMPI2C_IT_STOPF: STOP detection flag
  *     @arg FMPI2C_IT_TC: Transfer complete (master mode)
  *     @arg FMPI2C_IT_TCR: Transfer complete reload
  *     @arg FMPI2C_IT_BERR: Bus error
  *     @arg FMPI2C_IT_ARLO: Arbitration lost
  *     @arg FMPI2C_IT_OVR: Overrun/Underrun
  *     @arg FMPI2C_IT_PECERR: PEC error in reception
  *     @arg FMPI2C_IT_TIMEOUT: Timeout or Tlow detection flag
  *     @arg FMPI2C_IT_ALERT: SMBus Alert
  * @retval The new state of FMPI2C_IT (SET or RESET).
  */
ITStatus FMPI2C_GetITStatus(FMPI2C_TypeDef* FMPI2Cx, uint32_t FMPI2C_IT)
{
  uint32_t tmpreg = 0;
  ITStatus bitstatus = RESET;
  uint32_t enablestatus = 0;

  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FMPI2C_GET_IT(FMPI2C_IT));

  /* Check if the interrupt source is enabled or not */
  /* If Error interrupt */
  if((uint32_t)(FMPI2C_IT & ERROR_IT_MASK))
  {
    enablestatus = (uint32_t)((FMPI2C_CR1_ERRIE) & (FMPI2Cx->CR1));
  }
  /* If TC interrupt */
  else if((uint32_t)(FMPI2C_IT & TC_IT_MASK))
  {
    enablestatus = (uint32_t)((FMPI2C_CR1_TCIE) & (FMPI2Cx->CR1));
  }
  else
  {
    enablestatus = (uint32_t)((FMPI2C_IT) & (FMPI2Cx->CR1));
  }
  
  /* Get the ISR register value */
  tmpreg = FMPI2Cx->ISR;

  /* Get flag status */
  tmpreg &= FMPI2C_IT;

  /* Check the status of the specified FMPI2C flag */
  if((tmpreg != RESET) && enablestatus)
  {
    /* FMPI2C_IT is set */
    bitstatus = SET;
  }
  else
  {
    /* FMPI2C_IT is reset */
    bitstatus = RESET;
  }

  /* Return the FMPI2C_IT status */
  return bitstatus;
}
  
/**
  * @brief  Clears the FMPI2Cx's interrupt pending bits.
  * @param  FMPI2Cx: where x can be 1 to select the FMPI2C peripheral.
  * @param  FMPI2C_IT: specifies the interrupt pending bit to clear.
  *   This parameter can be any combination of the following values:
  *     @arg FMPI2C_IT_ADDR: Address matched (slave mode)
  *     @arg FMPI2C_IT_NACKF: NACK received flag
  *     @arg FMPI2C_IT_STOPF: STOP detection flag
  *     @arg FMPI2C_IT_BERR: Bus error
  *     @arg FMPI2C_IT_ARLO: Arbitration lost
  *     @arg FMPI2C_IT_OVR: Overrun/Underrun
  *     @arg FMPI2C_IT_PECERR: PEC error in reception
  *     @arg FMPI2C_IT_TIMEOUT: Timeout or Tlow detection flag
  *     @arg FMPI2C_IT_ALERT: SMBus Alert
  * @retval The new state of FMPI2C_IT (SET or RESET).
  */
void FMPI2C_ClearITPendingBit(FMPI2C_TypeDef* FMPI2Cx, uint32_t FMPI2C_IT)
{
  /* Check the parameters */
  assert_param(IS_FMPI2C_ALL_PERIPH(FMPI2Cx));
  assert_param(IS_FMPI2C_CLEAR_IT(FMPI2C_IT));

  /* Clear the selected flag */
  FMPI2Cx->ICR = FMPI2C_IT;
}

/**
  * @}
  */  
  
/**
  * @}
  */
#endif /* STM32F410xx || STM32F446xx */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
