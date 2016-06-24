/**
  ******************************************************************************
  * @file    stm32f4xx_dsi.c
  * @author  MCD Application Team
  * @version V1.7.1
  * @date    20-May-2016
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the Display Serial Interface (DSI):
  *           + Initialization and Configuration
  *           + Data transfers management functions
  *           + Low Power functions
  *           + Interrupts and flags management 
  *           
@verbatim

 ===================================================================
                  ##### How to use this driver #####
 ===================================================================
 [..]

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
#include "stm32f4xx_dsi.h"

/** @addtogroup STM32F4xx_StdPeriph_Driver
  * @{
  */
/** @addtogroup DSI
  * @brief DSI driver modules
  * @{
  */
#if defined(STM32F469_479xx)

/* Private types -------------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/** @addtogroup DSI_Private_Constants
  * @{
  */
#define DSI_TIMEOUT_VALUE ((uint32_t)1000)  /* 1s */

#define DSI_ERROR_ACK_MASK (DSI_ISR0_AE0 | DSI_ISR0_AE1 | DSI_ISR0_AE2 | DSI_ISR0_AE3 | \
                            DSI_ISR0_AE4 | DSI_ISR0_AE5 | DSI_ISR0_AE6 | DSI_ISR0_AE7 | \
                            DSI_ISR0_AE8 | DSI_ISR0_AE9 | DSI_ISR0_AE10 | DSI_ISR0_AE11 | \
                            DSI_ISR0_AE12 | DSI_ISR0_AE13 | DSI_ISR0_AE14 | DSI_ISR0_AE15)
#define DSI_ERROR_PHY_MASK (DSI_ISR0_PE0 | DSI_ISR0_PE1 | DSI_ISR0_PE2 | DSI_ISR0_PE3 | DSI_ISR0_PE4)
#define DSI_ERROR_TX_MASK  DSI_ISR1_TOHSTX
#define DSI_ERROR_RX_MASK  DSI_ISR1_TOLPRX
#define DSI_ERROR_ECC_MASK (DSI_ISR1_ECCSE | DSI_ISR1_ECCME)
#define DSI_ERROR_CRC_MASK DSI_ISR1_CRCE
#define DSI_ERROR_PSE_MASK DSI_ISR1_PSE
#define DSI_ERROR_EOT_MASK DSI_ISR1_EOTPE
#define DSI_ERROR_OVF_MASK DSI_ISR1_LPWRE
#define DSI_ERROR_GEN_MASK (DSI_ISR1_GCWRE | DSI_ISR1_GPWRE | DSI_ISR1_GPTXE | DSI_ISR1_GPRDE | DSI_ISR1_GPRXE)

#define DSI_MAX_RETURN_PKT_SIZE ((uint32_t)0x00000037) /*!< Maximum return packet configuration */
/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void DSI_ConfigPacketHeader(DSI_TypeDef *DSIx, uint32_t ChannelID, uint32_t DataType, uint32_t Data0, uint32_t Data1);
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup DSI_Exported_Functions
  * @{
  */

/** @defgroup DSI_Group1 Initialization and Configuration functions
 *  @brief   Initialization and Configuration functions
 *
@verbatim   
 ===============================================================================
                ##### Initialization and Configuration functions #####
 ===============================================================================  
    [..]  This section provides functions allowing to:
      (+) Initialize and configure the DSI
      (+) De-initialize the DSI 

@endverbatim
  * @{
  */

/**
  * @brief  De-initializes the DSI peripheral registers to their default reset
  *         values.
  * @param  DSIx: To select the DSIx peripheral, where x can be the different DSI instances
  * @retval None
  */
void DSI_DeInit(DSI_TypeDef *DSIx)
{  
  /* Disable the DSI wrapper */
  DSIx->WCR &= ~DSI_WCR_DSIEN;
  
  /* Disable the DSI host */
  DSIx->CR &= ~DSI_CR_EN;
  
  /* D-PHY clock and digital disable */
  DSIx->PCTLR &= ~(DSI_PCTLR_CKE | DSI_PCTLR_DEN);
  
  /* Turn off the DSI PLL */
  DSIx->WRPCR &= ~DSI_WRPCR_PLLEN;
  
  /* Disable the regulator */
  DSIx->WRPCR &= ~DSI_WRPCR_REGEN;
  
  /* Check the parameters */
  assert_param(IS_DSI_ALL_PERIPH(DSIx));
  if(DSIx == DSI)
  {
    /* Enable DSI reset state */
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_DSI, ENABLE);
    /* Release DSI from reset state */
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_DSI, DISABLE);
  }
}
  
/**
  * @brief Deinitialize the DSIx peripheral registers to their default reset values.
  * @param DSIx: To select the DSIx peripheral, where x can be the different DSI instances 
  * @param DSI_InitStruct: pointer to a DSI_InitTypeDef structure that
  *        contains the configuration information for the specified DSI peripheral.
  * @param DSI_InitTIMStruct: pointer to a DSI_TIMTypeDef structure that
  *        contains the configuration information for the specified DSI Timings.
  * @retval None
  */
void DSI_Init(DSI_TypeDef *DSIx,DSI_InitTypeDef* DSI_InitStruct, DSI_PLLInitTypeDef *PLLInit)
{
  uint32_t unitIntervalx4 = 0;
  uint32_t tempIDF = 0;
  
  /* Check function parameters */
  assert_param(IS_DSI_PLL_NDIV(PLLInit->PLLNDIV));
  assert_param(IS_DSI_PLL_IDF(PLLInit->PLLIDF));
  assert_param(IS_DSI_PLL_ODF(PLLInit->PLLODF));
  assert_param(IS_DSI_AUTO_CLKLANE_CONTROL(DSI_InitStruct->AutomaticClockLaneControl));
  assert_param(IS_DSI_NUMBER_OF_LANES(DSI_InitStruct->NumberOfLanes));
  
  /**************** Turn on the regulator and enable the DSI PLL ****************/
  
  /* Enable the regulator */
  DSIx->WRPCR |= DSI_WRPCR_REGEN;
  
  /* Wait until the regulator is ready */
  while(DSI_GetFlagStatus(DSIx, DSI_FLAG_RRS) == RESET )
  {}
  
  /* Set the PLL division factors */
  DSIx->WRPCR &= ~(DSI_WRPCR_PLL_NDIV | DSI_WRPCR_PLL_IDF | DSI_WRPCR_PLL_ODF);
  DSIx->WRPCR |= (((PLLInit->PLLNDIV)<<2) | ((PLLInit->PLLIDF)<<11) | ((PLLInit->PLLODF)<<16));
  
  /* Enable the DSI PLL */
  DSIx->WRPCR |= DSI_WRPCR_PLLEN;
  
  /* Wait for the lock of the PLL */
  while(DSI_GetFlagStatus(DSIx, DSI_FLAG_PLLLS) == RESET)
  {}
  
  /*************************** Set the PHY parameters ***************************/
  
  /* D-PHY clock and digital enable*/
  DSIx->PCTLR |= (DSI_PCTLR_CKE | DSI_PCTLR_DEN);
  
  /* Clock lane configuration */
  DSIx->CLCR &= ~(DSI_CLCR_DPCC | DSI_CLCR_ACR);
  DSIx->CLCR |= (DSI_CLCR_DPCC | DSI_InitStruct->AutomaticClockLaneControl);
  
  /* Configure the number of active data lanes */
  DSIx->PCONFR &= ~DSI_PCONFR_NL;
  DSIx->PCONFR |= DSI_InitStruct->NumberOfLanes;
  
  /************************ Set the DSI clock parameters ************************/
  /* Set the TX escape clock division factor */
  DSIx->CCR &= ~DSI_CCR_TXECKDIV;
  DSIx->CCR = DSI_InitStruct->TXEscapeCkdiv;
  
  /* Calculate the bit period in high-speed mode in unit of 0.25 ns (UIX4) */
  /* The equation is : UIX4 = IntegerPart( (1000/F_PHY_Mhz) * 4 )          */
  /* Where : F_PHY_Mhz = (NDIV * HSE_Mhz) / (IDF * ODF)                    */
  tempIDF = (PLLInit->PLLIDF > 0) ? PLLInit->PLLIDF : 1;
  unitIntervalx4 = (4000000 * tempIDF * (1 << PLLInit->PLLODF)) / ((HSE_VALUE/1000) * PLLInit->PLLNDIV);
  
  /* Set the bit period in high-speed mode */
  DSIx->WPCR[0] &= ~DSI_WPCR0_UIX4;
  DSIx->WPCR[0] |= unitIntervalx4;
  
  /****************************** Error management *****************************/
  /* Disable all error interrupts */
  DSIx->IER[0] = 0;
  DSIx->IER[1] = 0;
}

/**
  * @brief Fills each DSI_InitStruct member with its default value.
  * @param DSI_InitStruct: pointer to a DSI_InitTypeDef structure which will be initialized.
  * @retval None
  */
void DSI_StructInit(DSI_InitTypeDef* DSI_InitStruct, DSI_HOST_TimeoutTypeDef* DSI_HOST_TimeoutInitStruct)
{
  /*--------------- Reset DSI init structure parameters values ---------------*/
  /* Initialize the AutomaticClockLaneControl member */
  DSI_InitStruct->AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
  /* Initialize the NumberOfLanes member */
  DSI_InitStruct->NumberOfLanes = DSI_ONE_DATA_LANE;
  /* Initialize  the TX Escape clock division */
  DSI_InitStruct->TXEscapeCkdiv = 0;
    
  /*--------------- Reset DSI timings init structure parameters values -------*/
  /* Initialize the TimeoutCkdiv member */
  DSI_HOST_TimeoutInitStruct->TimeoutCkdiv = 0;
  /* Initialize the HighSpeedTransmissionTimeout member */
  DSI_HOST_TimeoutInitStruct->HighSpeedTransmissionTimeout = 0;
  /* Initialize the LowPowerReceptionTimeout member */
  DSI_HOST_TimeoutInitStruct->LowPowerReceptionTimeout = 0;
  /* Initialize the HighSpeedReadTimeout member */
  DSI_HOST_TimeoutInitStruct->HighSpeedReadTimeout = 0;
  /* Initialize the LowPowerReadTimeout member */
  DSI_HOST_TimeoutInitStruct->LowPowerReadTimeout = 0;
  /* Initialize the HighSpeedWriteTimeout member */
  DSI_HOST_TimeoutInitStruct->HighSpeedWriteTimeout = 0;
  /* Initialize the HighSpeedWritePrespMode member */
  DSI_HOST_TimeoutInitStruct->HighSpeedWritePrespMode = 0;
  /* Initialize the LowPowerWriteTimeout member */
  DSI_HOST_TimeoutInitStruct->LowPowerWriteTimeout = 0;
  /* Initialize the BTATimeout member */
  DSI_HOST_TimeoutInitStruct->BTATimeout = 0;
}

/**
  * @brief  Configure the Generic interface read-back Virtual Channel ID.
  * @param  DSIx: To select the DSIx peripheral, where x can be the different DSI instances
  * @param  VirtualChannelID: Virtual channel ID
  * @retval None
  */
void DSI_SetGenericVCID(DSI_TypeDef *DSIx, uint32_t VirtualChannelID)
{  
  /* Update the GVCID register */
  DSIx->GVCIDR &= ~DSI_GVCIDR_VCID;
  DSIx->GVCIDR |= VirtualChannelID;
}

/**
  * @brief  Select video mode and configure the corresponding parameters
  * @param  DSIx: To select the DSIx peripheral, where x can be the different DSI instances
  * @param  VidCfg: pointer to a DSI_VidCfgTypeDef structure that contains
  *                 the DSI video mode configuration parameters
  * @retval None
  */
void DSI_ConfigVideoMode(DSI_TypeDef *DSIx, DSI_VidCfgTypeDef *VidCfg)
{  
  /* Check the parameters */
  assert_param(IS_DSI_COLOR_CODING(VidCfg->ColorCoding));
  assert_param(IS_DSI_VIDEO_MODE_TYPE(VidCfg->Mode));
  assert_param(IS_DSI_LP_COMMAND(VidCfg->LPCommandEnable));
  assert_param(IS_DSI_LP_HFP(VidCfg->LPHorizontalFrontPorchEnable));
  assert_param(IS_DSI_LP_HBP(VidCfg->LPHorizontalBackPorchEnable));
  assert_param(IS_DSI_LP_VACTIVE(VidCfg->LPVerticalActiveEnable));
  assert_param(IS_DSI_LP_VFP(VidCfg->LPVerticalFrontPorchEnable));
  assert_param(IS_DSI_LP_VBP(VidCfg->LPVerticalBackPorchEnable));
  assert_param(IS_DSI_LP_VSYNC(VidCfg->LPVerticalSyncActiveEnable));
  assert_param(IS_DSI_FBTAA(VidCfg->FrameBTAAcknowledgeEnable));
  assert_param(IS_DSI_DE_POLARITY(VidCfg->DEPolarity));
  assert_param(IS_DSI_VSYNC_POLARITY(VidCfg->VSPolarity));
  assert_param(IS_DSI_HSYNC_POLARITY(VidCfg->HSPolarity));
  /* Check the LooselyPacked variant only in 18-bit mode */
  if(VidCfg->ColorCoding == DSI_RGB666)
  {
    assert_param(IS_DSI_LOOSELY_PACKED(VidCfg->LooselyPacked));
  }
  
  /* Select video mode by resetting CMDM and DSIM bits */
  DSIx->MCR &= ~DSI_MCR_CMDM;
  DSIx->WCFGR &= ~DSI_WCFGR_DSIM;
  
  /* Configure the video mode transmission type */
  DSIx->VMCR &= ~DSI_VMCR_VMT;
  DSIx->VMCR |= VidCfg->Mode;
  
  /* Configure the video packet size */
  DSIx->VPCR &= ~DSI_VPCR_VPSIZE;
  DSIx->VPCR |= VidCfg->PacketSize;
  
  /* Set the chunks number to be transmitted through the DSI link */
  DSIx->VCCR &= ~DSI_VCCR_NUMC;
  DSIx->VCCR |= VidCfg->NumberOfChunks;
  
  /* Set the size of the null packet */
  DSIx->VNPCR &= ~DSI_VNPCR_NPSIZE;
  DSIx->VNPCR |= VidCfg->NullPacketSize;
  
  /* Select the virtual channel for the LTDC interface traffic */
  DSIx->LVCIDR &= ~DSI_LVCIDR_VCID;
  DSIx->LVCIDR |= VidCfg->VirtualChannelID;
  
  /* Configure the polarity of control signals */
  DSIx->LPCR &= ~(DSI_LPCR_DEP | DSI_LPCR_VSP | DSI_LPCR_HSP);
  DSIx->LPCR |= (VidCfg->DEPolarity | VidCfg->VSPolarity | VidCfg->HSPolarity);
  
  /* Select the color coding for the host */
  DSIx->LCOLCR &= ~DSI_LCOLCR_COLC;
  DSIx->LCOLCR |= VidCfg->ColorCoding;
    
  /* Select the color coding for the wrapper */
  DSIx->WCFGR &= ~DSI_WCFGR_COLMUX;
  DSIx->WCFGR |= ((VidCfg->ColorCoding)<<1);
  
  /* Enable/disable the loosely packed variant to 18-bit configuration */
  if(VidCfg->ColorCoding == DSI_RGB666)
  {
    DSIx->LCOLCR &= ~DSI_LCOLCR_LPE;
    DSIx->LCOLCR |= VidCfg->LooselyPacked;
  }
  
  /* Set the Horizontal Synchronization Active (HSA) in lane byte clock cycles */
  DSIx->VHSACR &= ~DSI_VHSACR_HSA;
  DSIx->VHSACR |= VidCfg->HorizontalSyncActive;
  
  /* Set the Horizontal Back Porch (HBP) in lane byte clock cycles */
  DSIx->VHBPCR &= ~DSI_VHBPCR_HBP;
  DSIx->VHBPCR |= VidCfg->HorizontalBackPorch;
  
  /* Set the total line time (HLINE=HSA+HBP+HACT+HFP) in lane byte clock cycles */
  DSIx->VLCR &= ~DSI_VLCR_HLINE;
  DSIx->VLCR |= VidCfg->HorizontalLine;
  
  /* Set the Vertical Synchronization Active (VSA) */
  DSIx->VVSACR &= ~DSI_VVSACR_VSA;
  DSIx->VVSACR |= VidCfg->VerticalSyncActive;
  
  /* Set the Vertical Back Porch (VBP)*/
  DSIx->VVBPCR &= ~DSI_VVBPCR_VBP;
  DSIx->VVBPCR |= VidCfg->VerticalBackPorch;
  
  /* Set the Vertical Front Porch (VFP)*/
  DSIx->VVFPCR &= ~DSI_VVFPCR_VFP;
  DSIx->VVFPCR |= VidCfg->VerticalFrontPorch;
  
  /* Set the Vertical Active period*/
  DSIx->VVACR &= ~DSI_VVACR_VA;
  DSIx->VVACR |= VidCfg->VerticalActive;
  
  /* Configure the command transmission mode */
  DSIx->VMCR &= ~DSI_VMCR_LPCE;
  DSIx->VMCR |= VidCfg->LPCommandEnable;
  
  /* Low power largest packet size */
  DSIx->LPMCR &= ~DSI_LPMCR_LPSIZE;
  DSIx->LPMCR |= ((VidCfg->LPLargestPacketSize)<<16);
  
  /* Low power VACT largest packet size */
  DSIx->LPMCR &= ~DSI_LPMCR_VLPSIZE;
  DSIx->LPMCR |= VidCfg->LPVACTLargestPacketSize;
  
  /* Enable LP transition in HFP period */
  DSIx->VMCR &= ~DSI_VMCR_LPHFPE;
  DSIx->VMCR |= VidCfg->LPHorizontalFrontPorchEnable;
  
  /* Enable LP transition in HBP period */
  DSIx->VMCR &= ~DSI_VMCR_LPHBPE;
  DSIx->VMCR |= VidCfg->LPHorizontalBackPorchEnable;
  
  /* Enable LP transition in VACT period */
  DSIx->VMCR &= ~DSI_VMCR_LPVAE;
  DSIx->VMCR |= VidCfg->LPVerticalActiveEnable;
  
  /* Enable LP transition in VFP period */
  DSIx->VMCR &= ~DSI_VMCR_LPVFPE;
  DSIx->VMCR |= VidCfg->LPVerticalFrontPorchEnable;
  
  /* Enable LP transition in VBP period */
  DSIx->VMCR &= ~DSI_VMCR_LPVBPE;
  DSIx->VMCR |= VidCfg->LPVerticalBackPorchEnable;
  
  /* Enable LP transition in vertical sync period */
  DSIx->VMCR &= ~DSI_VMCR_LPVSAE;
  DSIx->VMCR |= VidCfg->LPVerticalSyncActiveEnable;
  
  /* Enable the request for an acknowledge response at the end of a frame */
  DSIx->VMCR &= ~DSI_VMCR_FBTAAE;
  DSIx->VMCR |= VidCfg->FrameBTAAcknowledgeEnable;
}

/**
  * @brief  Select adapted command mode and configure the corresponding parameters
  * @param  DSIx: To select the DSIx peripheral, where x can be the different DSI instances
  * @param  CmdCfg: pointer to a DSI_CmdCfgTypeDef structure that contains
  *                 the DSI command mode configuration parameters
  * @retval None
  */
void DSI_ConfigAdaptedCommandMode(DSI_TypeDef *DSIx, DSI_CmdCfgTypeDef *CmdCfg)
{  
  /* Check the parameters */
  assert_param(IS_DSI_COLOR_CODING(CmdCfg->ColorCoding));
  assert_param(IS_DSI_TE_SOURCE(CmdCfg->TearingEffectSource));
  assert_param(IS_DSI_TE_POLARITY(CmdCfg->TearingEffectPolarity));
  assert_param(IS_DSI_AUTOMATIC_REFRESH(CmdCfg->AutomaticRefresh));
  assert_param(IS_DSI_VS_POLARITY(CmdCfg->VSyncPol));
  assert_param(IS_DSI_TE_ACK_REQUEST(CmdCfg->TEAcknowledgeRequest));
  assert_param(IS_DSI_DE_POLARITY(CmdCfg->DEPolarity));
  assert_param(IS_DSI_VSYNC_POLARITY(CmdCfg->VSPolarity));
  assert_param(IS_DSI_HSYNC_POLARITY(CmdCfg->HSPolarity));
  
  /* Select command mode by setting CMDM and DSIM bits */
  DSIx->MCR |= DSI_MCR_CMDM;
  DSIx->WCFGR &= ~DSI_WCFGR_DSIM;
  DSIx->WCFGR |= DSI_WCFGR_DSIM;
  
  /* Select the virtual channel for the LTDC interface traffic */
  DSIx->LVCIDR &= ~DSI_LVCIDR_VCID;
  DSIx->LVCIDR |= CmdCfg->VirtualChannelID;
  
  /* Configure the polarity of control signals */
  DSIx->LPCR &= ~(DSI_LPCR_DEP | DSI_LPCR_VSP | DSI_LPCR_HSP);
  DSIx->LPCR |= (CmdCfg->DEPolarity | CmdCfg->VSPolarity | CmdCfg->HSPolarity);
  
  /* Select the color coding for the host */
  DSIx->LCOLCR &= ~DSI_LCOLCR_COLC;
  DSIx->LCOLCR |= CmdCfg->ColorCoding;
    
  /* Select the color coding for the wrapper */
  DSIx->WCFGR &= ~DSI_WCFGR_COLMUX;
  DSIx->WCFGR |= ((CmdCfg->ColorCoding)<<1);

  /* Configure the maximum allowed size for write memory command */
  DSIx->LCCR &= ~DSI_LCCR_CMDSIZE;
  DSIx->LCCR |= CmdCfg->CommandSize;
  
  /* Configure the tearing effect source and polarity and select the refresh mode */
  DSIx->WCFGR &= ~(DSI_WCFGR_TESRC | DSI_WCFGR_TEPOL | DSI_WCFGR_AR | DSI_WCFGR_VSPOL);
  DSIx->WCFGR |= (CmdCfg->TearingEffectSource | CmdCfg->TearingEffectPolarity | CmdCfg->AutomaticRefresh | CmdCfg->VSyncPol);
  
  /* Configure the tearing effect acknowledge request */
  DSIx->CMCR &= ~DSI_CMCR_TEARE;
  DSIx->CMCR |= CmdCfg->TEAcknowledgeRequest;
  
  /* Enable the Tearing Effect interrupt */
  DSI_ITConfig(DSIx, DSI_IT_TE, ENABLE);
  /* Enable the End of Refresh interrupt */
  DSI_ITConfig(DSIx, DSI_IT_ER, ENABLE);  
}

/**
  * @brief  Configure command transmission mode: High-speed or Low-power
  *         and enable/disable acknowledge request after packet transmission
  * @param  DSIx: To select the DSIx peripheral, where x can be the different DSI instances
  * @param  LPCmd: pointer to a DSI_LPCmdTypeDef structure that contains
  *                the DSI command transmission mode configuration parameters
  * @retval None
  */
void DSI_ConfigCommand(DSI_TypeDef *DSIx, DSI_LPCmdTypeDef *LPCmd)
{
  assert_param(IS_DSI_LP_GSW0P(LPCmd->LPGenShortWriteNoP));
  assert_param(IS_DSI_LP_GSW1P(LPCmd->LPGenShortWriteOneP));
  assert_param(IS_DSI_LP_GSW2P(LPCmd->LPGenShortWriteTwoP));
  assert_param(IS_DSI_LP_GSR0P(LPCmd->LPGenShortReadNoP));
  assert_param(IS_DSI_LP_GSR1P(LPCmd->LPGenShortReadOneP));
  assert_param(IS_DSI_LP_GSR2P(LPCmd->LPGenShortReadTwoP));
  assert_param(IS_DSI_LP_GLW(LPCmd->LPGenLongWrite));
  assert_param(IS_DSI_LP_DSW0P(LPCmd->LPDcsShortWriteNoP));
  assert_param(IS_DSI_LP_DSW1P(LPCmd->LPDcsShortWriteOneP));
  assert_param(IS_DSI_LP_DSR0P(LPCmd->LPDcsShortReadNoP));
  assert_param(IS_DSI_LP_DLW(LPCmd->LPDcsLongWrite));
  assert_param(IS_DSI_LP_MRDP(LPCmd->LPMaxReadPacket));
  assert_param(IS_DSI_ACK_REQUEST(LPCmd->AcknowledgeRequest));
  
  /* Select High-speed or Low-power for command transmission */
  DSIx->CMCR &= ~(DSI_CMCR_GSW0TX |\
                            DSI_CMCR_GSW1TX |\
                            DSI_CMCR_GSW2TX |\
                            DSI_CMCR_GSR0TX |\
                            DSI_CMCR_GSR1TX |\
                            DSI_CMCR_GSR2TX |\
                            DSI_CMCR_GLWTX  |\
                            DSI_CMCR_DSW0TX |\
                            DSI_CMCR_DSW1TX |\
                            DSI_CMCR_DSR0TX |\
                            DSI_CMCR_DLWTX  |\
                            DSI_CMCR_MRDPS);
  DSIx->CMCR |= (LPCmd->LPGenShortWriteNoP  |\
                           LPCmd->LPGenShortWriteOneP |\
                           LPCmd->LPGenShortWriteTwoP |\
                           LPCmd->LPGenShortReadNoP   |\
                           LPCmd->LPGenShortReadOneP  |\
                           LPCmd->LPGenShortReadTwoP  |\
                           LPCmd->LPGenLongWrite      |\
                           LPCmd->LPDcsShortWriteNoP  |\
                           LPCmd->LPDcsShortWriteOneP |\
                           LPCmd->LPDcsShortReadNoP   |\
                           LPCmd->LPDcsLongWrite      |\
                           LPCmd->LPMaxReadPacket);
  
  /* Configure the acknowledge request after each packet transmission */
  DSIx->CMCR &= ~DSI_CMCR_ARE;
  DSIx->CMCR |= LPCmd->AcknowledgeRequest;
}

/**
  * @brief  Configure the flow control parameters
  * @param  DSIx: To select the DSIx peripheral, where x can be the different DSI instances
  * @param  FlowControl: flow control feature(s) to be enabled.
  *                      This parameter can be any combination of @ref DSI_FlowControl.
  * @retval None
  */
void DSI_ConfigFlowControl(DSI_TypeDef *DSIx, uint32_t FlowControl)
{  
  /* Check the parameters */
  assert_param(IS_DSI_FLOW_CONTROL(FlowControl));
  
  /* Set the DSI Host Protocol Configuration Register */
  DSIx->PCR &= ~DSI_FLOW_CONTROL_ALL;
  DSIx->PCR |= FlowControl;
}

/**
  * @brief  Configure the DSI PHY timer parameters
  * @param  DSIx: To select the DSIx peripheral, where x can be the different DSI instances
  * @param  PhyTimers: DSI_PHY_TimerTypeDef structure that contains
  *                    the DSI PHY timing parameters
  * @retval None
  */
void DSI_ConfigPhyTimer(DSI_TypeDef *DSIx, DSI_PHY_TimerTypeDef *PhyTimers)
{ 
  uint32_t maxTime = 0;
 
  maxTime = (PhyTimers->ClockLaneLP2HSTime > PhyTimers->ClockLaneHS2LPTime)? PhyTimers->ClockLaneLP2HSTime: PhyTimers->ClockLaneHS2LPTime;

  /* Clock lane timer configuration */
  /* In Automatic Clock Lane control mode, the DSI Host can turn off the clock lane between two
     High-Speed transmission.
     To do so, the DSI Host calculates the time required for the clock lane to change from HighSpeed
     to Low-Power and from Low-Power to High-Speed.
     This timings are configured by the HS2LP_TIME and LP2HS_TIME in the DSI Host Clock Lane Timer Configuration Register (DSI_CLTCR).
     But the DSI Host is not calculating LP2HS_TIME + HS2LP_TIME but 2 x HS2LP_TIME.

     Workaround : Configure HS2LP_TIME and LP2HS_TIME with the same value being the max of HS2LP_TIME or LP2HS_TIME.
  */
  DSIx->CLTCR &= ~(DSI_CLTCR_LP2HS_TIME | DSI_CLTCR_HS2LP_TIME);
  DSIx->CLTCR |= (maxTime | ((maxTime)<<16));
  
  /* Data lane timer configuration */
  DSIx->DLTCR &= ~(DSI_DLTCR_MRD_TIME | DSI_DLTCR_LP2HS_TIME | DSI_DLTCR_HS2LP_TIME);
  DSIx->DLTCR |= (PhyTimers->DataLaneMaxReadTime | ((PhyTimers->DataLaneLP2HSTime)<<16) | ((PhyTimers->DataLaneHS2LPTime)<<24));
  
  /* Configure the wait period to request HS transmission after a stop state */
  DSIx->PCONFR &= ~DSI_PCONFR_SW_TIME;
  DSIx->PCONFR |= ((PhyTimers->StopWaitTime)<<8);
}

/**
  * @brief  Configure the DSI HOST timeout parameters
  * @param  DSIx: To select the DSIx peripheral, where x can be the different DSI instances
  * @param  HostTimeouts: DSI_HOST_TimeoutTypeDef structure that contains
  *                       the DSI host timeout parameters
  * @retval None
  */
void DSI_ConfigHostTimeouts(DSI_TypeDef *DSIx, DSI_HOST_TimeoutTypeDef *HostTimeouts)
{
  /* Set the timeout clock division factor */
  DSIx->CCR &= ~DSI_CCR_TOCKDIV;
  DSIx->CCR = ((HostTimeouts->TimeoutCkdiv)<<8);
  
  /* High-speed transmission timeout */
  DSIx->TCCR[0] &= ~DSI_TCCR0_HSTX_TOCNT;
  DSIx->TCCR[0] |= ((HostTimeouts->HighSpeedTransmissionTimeout)<<16);
  
  /* Low-power reception timeout */
  DSIx->TCCR[0] &= ~DSI_TCCR0_LPRX_TOCNT;
  DSIx->TCCR[0] |= HostTimeouts->LowPowerReceptionTimeout;
  
  /* High-speed read timeout */
  DSIx->TCCR[1] &= ~DSI_TCCR1_HSRD_TOCNT;
  DSIx->TCCR[1] |= HostTimeouts->HighSpeedReadTimeout;
  
  /* Low-power read timeout */
  DSIx->TCCR[2] &= ~DSI_TCCR2_LPRD_TOCNT;
  DSIx->TCCR[2] |= HostTimeouts->LowPowerReadTimeout;
  
  /* High-speed write timeout */
  DSIx->TCCR[3] &= ~DSI_TCCR3_HSWR_TOCNT;
  DSIx->TCCR[3] |= HostTimeouts->HighSpeedWriteTimeout;
  
  /* High-speed write presp mode */
  DSIx->TCCR[3] &= ~DSI_TCCR3_PM;
  DSIx->TCCR[3] |= HostTimeouts->HighSpeedWritePrespMode;
  
  /* Low-speed write timeout */
  DSIx->TCCR[4] &= ~DSI_TCCR4_LPWR_TOCNT;
  DSIx->TCCR[4] |= HostTimeouts->LowPowerWriteTimeout;
  
  /* BTA timeout */
  DSIx->TCCR[5] &= ~DSI_TCCR5_BTA_TOCNT;
  DSIx->TCCR[5] |= HostTimeouts->BTATimeout;
}

/**
  * @brief  Start the DSI module
  * @param  DSIx: To select the DSIx peripheral, where x can be the different DSI instances
  *               the configuration information for the DSI.
  * @retval None
  */
void DSI_Start(DSI_TypeDef *DSIx)
{  
  /* Enable the DSI host */
  DSIx->CR |= DSI_CR_EN;
  /* Enable the DSI wrapper */
  DSIx->WCR |= DSI_WCR_DSIEN;
}

/**
  * @brief  Stop the DSI module
  * @param  DSIx: To select the DSIx peripheral, where x can be the different DSI instances
  * @retval None
  */
void DSI_Stop(DSI_TypeDef *DSIx)
{  
  /* Disable the DSI host */
  DSIx->CR &= ~DSI_CR_EN;
  
  /* Disable the DSI wrapper */
  DSIx->WCR &= ~DSI_WCR_DSIEN;  
}

/**
  * @brief  Refresh the display in command mode
  * @param  DSIx: To select the DSIx peripheral, where x can be the different DSI instances
  *               the configuration information for the DSI.
  * @retval None
  */
void DSI_Refresh(DSI_TypeDef *DSIx)
{  
  /* Update the display */
  DSIx->WCR |= DSI_WCR_LTDCEN;
}

/**
  * @brief  Controls the display color mode in Video mode
  * @param  DSIx: To select the DSIx peripheral, where x can be the different DSI instances
  * @param  ColorMode: Color mode (full or 8-colors).
  *                    This parameter can be any value of @ref DSI_Color_Mode
  * @retval None
  */
void DSI_ColorMode(DSI_TypeDef *DSIx, uint32_t ColorMode)
{  
  /* Check the parameters */
  assert_param(IS_DSI_COLOR_MODE(ColorMode));
  
  /* Update the display color mode */
  DSIx->WCR &= ~DSI_WCR_COLM;
  DSIx->WCR |= ColorMode;
}

/**
  * @brief  Control the display shutdown in Video mode
  * @param  DSIx: To select the DSIx peripheral, where x can be the different DSI instances
  * @param  Shutdown: Shut-down (Display-ON or Display-OFF).
  *                   This parameter can be any value of @ref DSI_ShutDown
  * @retval None
  */
void DSI_Shutdown(DSI_TypeDef *DSIx, uint32_t Shutdown)
{
  /* Check the parameters */
  assert_param(IS_DSI_SHUT_DOWN(Shutdown));
  
  /* Update the display Shutdown */
  DSIx->WCR &= ~DSI_WCR_SHTDN;
  DSIx->WCR |= Shutdown;
}

/**
  * @}
  */
    
/** @defgroup Data transfers management functions 
 *  @brief    DSI data transfers management functions  
 *
@verbatim
 ===============================================================================
                #####  Data transfers management functions  #####
 ===============================================================================  
@endverbatim
  * @{
  */
  
/**
  * @brief  DCS or Generic short write command
  * @param  DSIx: To select the DSIx peripheral, where x can be the different DSI instances
  * @param  ChannelID: Virtual channel ID.
  * @param  Mode: DSI short packet data type.
  *               This parameter can be any value of @ref DSI_SHORT_WRITE_PKT_Data_Type.
  * @param  Param1: DSC command or first generic parameter.
  *                 This parameter can be any value of @ref DSI_DCS_Command or a
  *                 generic command code.
  * @param  Param2: DSC parameter or second generic parameter.
  * @retval None
  */
void DSI_ShortWrite(DSI_TypeDef *DSIx,
                                 uint32_t ChannelID,
                                 uint32_t Mode,
                                 uint32_t Param1,
                                 uint32_t Param2)
{
  /* Check the parameters */
  assert_param(IS_DSI_SHORT_WRITE_PACKET_TYPE(Mode));
    
  /* Wait for Command FIFO Empty */
  while((DSIx->GPSR & DSI_GPSR_CMDFE) == 0)
  {}
  
  /* Configure the packet to send a short DCS command with 0 or 1 parameter */
  DSI_ConfigPacketHeader(DSIx,
                         ChannelID,
                         Mode,
                         Param1,
                         Param2);
}

/**
  * @brief  DCS or Generic long write command
  * @param  DSIx: To select the DSIx peripheral, where x can be the different DSI instances
  * @param  ChannelID: Virtual channel ID.
  * @param  Mode: DSI long packet data type.
  *               This parameter can be any value of @ref DSI_LONG_WRITE_PKT_Data_Type.
  * @param  NbParams: Number of parameters.
  * @param  Param1: DSC command or first generic parameter.
  *                 This parameter can be any value of @ref DSI_DCS_Command or a 
  *                 generic command code
  * @param  ParametersTable: Pointer to parameter values table.
  * @retval None
  */
void DSI_LongWrite(DSI_TypeDef *DSIx,
                                uint32_t ChannelID,
                                uint32_t Mode,
                                uint32_t NbParams,
                                uint32_t Param1,
                                uint8_t* ParametersTable)
{
  uint32_t uicounter = 0;
  
  /* Check the parameters */
  assert_param(IS_DSI_LONG_WRITE_PACKET_TYPE(Mode));
      
  /* Wait for Command FIFO Empty */
  while((DSIx->GPSR & DSI_GPSR_CMDFE) == 0)
  {}
  
  /* Set the DCS code hexadecimal on payload byte 1, and the other parameters on the write FIFO command*/
  while(uicounter < NbParams)
  {
    if(uicounter == 0x00)
    {
      DSIx->GPDR=(Param1 | \
                            ((*(ParametersTable+uicounter))<<8) | \
                            ((*(ParametersTable+uicounter+1))<<16) | \
                            ((*(ParametersTable+uicounter+2))<<24));
      uicounter += 3;
    }
    else
    {
      DSIx->GPDR=((*(ParametersTable+uicounter)) | \
                            ((*(ParametersTable+uicounter+1))<<8) | \
                            ((*(ParametersTable+uicounter+2))<<16) | \
                            ((*(ParametersTable+uicounter+3))<<24));
      uicounter+=4;
    }
  }
  
  /* Configure the packet to send a long DCS command */
  DSI_ConfigPacketHeader(DSIx,
                         ChannelID,
                         Mode,
                         ((NbParams+1)&0x00FF),
                         (((NbParams+1)&0xFF00)>>8));
}

/**
  * @brief  Read command (DCS or generic)
  * @param  DSIx: To select the DSIx peripheral, where x can be the different DSI instances
  * @param  ChannelNbr: Virtual channel ID
  * @param  Array: pointer to a buffer to store the payload of a read back operation.
  * @param  Size: Data size to be read (in byte).
  * @param  Mode: DSI read packet data type.
  *               This parameter can be any value of @ref DSI_SHORT_READ_PKT_Data_Type.
  * @param  DCSCmd: DCS get/read command.
  * @param  ParametersTable: Pointer to parameter values table.
  * @retval None
  */
void DSI_Read(DSI_TypeDef *DSIx,
                               uint32_t ChannelNbr,
                               uint8_t* Array,
                               uint32_t Size,
                               uint32_t Mode,
                               uint32_t DCSCmd,
                               uint8_t* ParametersTable)
{
  
  /* Check the parameters */
  assert_param(IS_DSI_READ_PACKET_TYPE(Mode));
  
  if(Size > 2)
  {
    /* set max return packet size */
    DSI_ShortWrite(DSIx, ChannelNbr, DSI_MAX_RETURN_PKT_SIZE, ((Size)&0xFF), (((Size)>>8)&0xFF));
  }
  
  /* Configure the packet to read command */
  if (Mode == DSI_DCS_SHORT_PKT_READ)
  {
    DSI_ConfigPacketHeader(DSIx, ChannelNbr, Mode, DCSCmd, 0);
  }
  else if (Mode == DSI_GEN_SHORT_PKT_READ_P0)
  {
    DSI_ConfigPacketHeader(DSIx, ChannelNbr, Mode, 0, 0);
  }
  else if (Mode == DSI_GEN_SHORT_PKT_READ_P1)
  {
    DSI_ConfigPacketHeader(DSIx, ChannelNbr, Mode, ParametersTable[0], 0);
  }
  else if (Mode == DSI_GEN_SHORT_PKT_READ_P2)
  {
    DSI_ConfigPacketHeader(DSIx, ChannelNbr, Mode, ParametersTable[0], ParametersTable[1]);
  }
    
  /* Check that the payload read FIFO is not empty */
  while((DSIx->GPSR & DSI_GPSR_PRDFE) == DSI_GPSR_PRDFE)
  {}
  
  /* Get the first byte */
  *((uint32_t *)Array) = (DSIx->GPDR);
  if (Size > 4)
  {
    Size -= 4;
    Array += 4;
  }

  /* Get the remaining bytes if any */
  while(((int)(Size)) > 0)
  {
    if((DSIx->GPSR & DSI_GPSR_PRDFE) == 0)
    {
      *((uint32_t *)Array) = (DSIx->GPDR);
      Size -= 4;
      Array += 4;
    }   
  }
}

/**
  * @brief  Generic DSI packet header configuration
  * @param  DSIx: Pointer to DSI register base
  * @param  ChannelID: Virtual channel ID of the header packet
  * @param  DataType: Packet data type of the header packet
  *                   This parameter can be any value of :
  *                      @ref DSI_SHORT_WRITE_PKT_Data_Type
  *                   or @ref DSI_LONG_WRITE_PKT_Data_Type
  *                   or @ref DSI_SHORT_READ_PKT_Data_Type
  *                   or DSI_MAX_RETURN_PKT_SIZE
  * @param  Data0: Word count LSB
  * @param  Data1: Word count MSB
  * @retval None
  */
static void DSI_ConfigPacketHeader(DSI_TypeDef *DSIx,
                                   uint32_t ChannelID,
                                   uint32_t DataType,
                                   uint32_t Data0,
                                   uint32_t Data1)
{
  /* Update the DSI packet header with new information */
  DSIx->GHCR = (DataType | (ChannelID<<6) | (Data0<<8) | (Data1<<16));
}
  
/**
  * @}
  */

/** @defgroup DSI_Group3 Low Power functions
 *  @brief    DSI Low Power management functions 
 *
@verbatim
 ===============================================================================
                   ##### DSI Low Power functions #####
 ===============================================================================   

@endverbatim
  * @{
  */

/**
  * @brief  Enter the ULPM (Ultra Low Power Mode) with the D-PHY PLL running
  *         (only data lanes are in ULPM)
  * @param  DSIx: Pointer to DSI register base
  * @retval None
  */
void DSI_EnterULPMData(DSI_TypeDef *DSIx)
{    
  /* ULPS Request on Data Lanes */
  DSIx->PUCR |= DSI_PUCR_URDL;
  
  
  /* Wait until the D-PHY active lanes enter into ULPM */
  if((DSIx->PCONFR & DSI_PCONFR_NL) == DSI_ONE_DATA_LANE)
  {
    while((DSIx->PSR & DSI_PSR_UAN0) != 0)
    {}
  }
  else if ((DSIx->PCONFR & DSI_PCONFR_NL) == DSI_TWO_DATA_LANES)
  {
    while((DSIx->PSR & (DSI_PSR_UAN0 | DSI_PSR_UAN1)) != 0)
    {}
  }
}

/**
  * @brief  Exit the ULPM (Ultra Low Power Mode) with the D-PHY PLL running
  *         (only data lanes are in ULPM)
  * @param  DSIx: Pointer to DSI register base
  * @retval None
  */
void DSI_ExitULPMData(DSI_TypeDef *DSIx)
{  
  /* Exit ULPS on Data Lanes */
  DSIx->PUCR |= DSI_PUCR_UEDL;
  
  /* Wait until all active lanes exit ULPM */
  if((DSIx->PCONFR & DSI_PCONFR_NL) == DSI_ONE_DATA_LANE)
  {
    while((DSIx->PSR & DSI_PSR_UAN0) != DSI_PSR_UAN0)
    {}
  }
  else if ((DSIx->PCONFR & DSI_PCONFR_NL) == DSI_TWO_DATA_LANES)
  {
    while((DSIx->PSR & (DSI_PSR_UAN0 | DSI_PSR_UAN1)) != (DSI_PSR_UAN0 | DSI_PSR_UAN1))
    {}
  }
  
  /* De-assert the ULPM requests and the ULPM exit bits */
  DSIx->PUCR = 0;
}

/**
  * @brief  Enter the ULPM (Ultra Low Power Mode) with the D-PHY PLL turned off
  *         (both data and clock lanes are in ULPM)
  * @param  DSIx: Pointer to DSI register base
  * @retval None
  */
void DSI_EnterULPM(DSI_TypeDef *DSIx)
{    
  /* Clock lane configuration: no more HS request */
  DSIx->CLCR &= ~DSI_CLCR_DPCC;
  
  /* Use system PLL as byte lane clock source before stopping DSIPHY clock source */
  RCC_DSIClockSourceConfig(RCC_DSICLKSource_PLLR);
  
  /* ULPS Request on Clock and Data Lanes */
  DSIx->PUCR |= (DSI_PUCR_URCL | DSI_PUCR_URDL);
  
  /* Wait until all active lanes exit ULPM */
  if((DSIx->PCONFR & DSI_PCONFR_NL) == DSI_ONE_DATA_LANE)
  {
    while((DSIx->PSR & (DSI_PSR_UAN0 | DSI_PSR_UANC)) != 0)
    {}
  }
  else if ((DSIx->PCONFR & DSI_PCONFR_NL) == DSI_TWO_DATA_LANES)
  {
    while((DSIx->PSR & (DSI_PSR_UAN0 | DSI_PSR_UAN1 | DSI_PSR_UANC)) != 0)
    {}
  }
  
  /* Turn off the DSI PLL */
  DSIx->WRPCR &= ~DSI_WRPCR_PLLEN;
}

/**
  * @brief  Exit the ULPM (Ultra Low Power Mode) with the D-PHY PLL turned off
  *         (both data and clock lanes are in ULPM)
  * @param  DSIx: Pointer to DSI register base
  * @retval None
  */
void DSI_ExitULPM(DSI_TypeDef *DSIx)
{    
  /* Turn on the DSI PLL */
  DSIx->WRPCR |= DSI_WRPCR_PLLEN;
     
  /* Wait for the lock of the PLL */
  while(DSI_GetFlagStatus(DSIx, DSI_FLAG_PLLLS) == RESET)
  {}
  
  /* Exit ULPS on Clock and Data Lanes */
  DSIx->PUCR |= (DSI_PUCR_UECL | DSI_PUCR_UEDL);
    
  /* Wait until all active lanes exit ULPM */
  if((DSIx->PCONFR & DSI_PCONFR_NL) == DSI_ONE_DATA_LANE)
  {
    while((DSIx->PSR & (DSI_PSR_UAN0 | DSI_PSR_UANC)) != (DSI_PSR_UAN0 | DSI_PSR_UANC))
    {}
  }
  else if ((DSIx->PCONFR & DSI_PCONFR_NL) == DSI_TWO_DATA_LANES)
  {
    while((DSIx->PSR & (DSI_PSR_UAN0 | DSI_PSR_UAN1 | DSI_PSR_UANC)) != (DSI_PSR_UAN0 | DSI_PSR_UAN1 | DSI_PSR_UANC))
    {}
  }
  
  /* De-assert the ULPM requests and the ULPM exit bits */
  DSIx->PUCR = 0;
  
  /* Switch the lanbyteclock source in the RCC from system PLL to D-PHY */
  RCC_DSIClockSourceConfig(RCC_DSICLKSource_PHY);
  
  /* Restore clock lane configuration to HS */
  DSIx->CLCR |= DSI_CLCR_DPCC;
}

/**
  * @brief  Start test pattern generation
  * @param  DSIx: To select the DSIx peripheral, where x can be the different DSI instances
  * @param  Mode: Pattern generator mode
  *          This parameter can be one of the following values:
  *           0 : Color bars (horizontal or vertical)
  *           1 : BER pattern (vertical only)
  * @param  Orientation: Pattern generator orientation
  *          This parameter can be one of the following values:
  *           0 : Vertical color bars
  *           1 : Horizontal color bars
  * @retval None
  */
void DSI_PatternGeneratorStart(DSI_TypeDef *DSIx, uint32_t Mode, uint32_t Orientation)
{
  
  /* Configure pattern generator mode and orientation */
  DSIx->VMCR &= ~(DSI_VMCR_PGM | DSI_VMCR_PGO);
  DSIx->VMCR |= ((Mode<<20) | (Orientation<<24));
  
  /* Enable pattern generator by setting PGE bit */
  DSIx->VMCR |= DSI_VMCR_PGE;
  
}

/**
  * @brief  Stop test pattern generation
  * @param  DSIx: To select the DSIx peripheral, where x can be the different DSI instances
  * @retval None
  */
void DSI_PatternGeneratorStop(DSI_TypeDef *DSIx)
{  
  /* Disable pattern generator by clearing PGE bit */
  DSIx->VMCR &= ~DSI_VMCR_PGE;
}

/**
  * @brief  Set Slew-Rate And Delay Tuning
  * @param  DSIx: Pointer to DSI register base
  * @param  CommDelay: Communication delay to be adjusted.
  *                    This parameter can be any value of @ref DSI_Communication_Delay
  * @param  Lane: select between clock or data lanes.
  *               This parameter can be any value of @ref DSI_Lane_Group
  * @param  Value: Custom value of the slew-rate or delay
  * @retval None
  */
void DSI_SetSlewRateAndDelayTuning(DSI_TypeDef *DSIx, uint32_t CommDelay, uint32_t Lane, uint32_t Value)
{  
  /* Check function parameters */
  assert_param(IS_DSI_COMMUNICATION_DELAY(CommDelay));
  assert_param(IS_DSI_LANE_GROUP(Lane));
  
  switch(CommDelay)
  {
  case DSI_SLEW_RATE_HSTX:
    if(Lane == DSI_CLOCK_LANE)
    {
      /* High-Speed Transmission Slew Rate Control on Clock Lane */
      DSIx->WPCR[1] &= ~DSI_WPCR1_HSTXSRCCL;
      DSIx->WPCR[1] |= Value<<16;
    }
    else if(Lane == DSI_DATA_LANES)
    {
      /* High-Speed Transmission Slew Rate Control on Data Lanes */
      DSIx->WPCR[1] &= ~DSI_WPCR1_HSTXSRCDL;
      DSIx->WPCR[1] |= Value<<18;
    }
    break;
  case DSI_SLEW_RATE_LPTX:
    if(Lane == DSI_CLOCK_LANE)
    {
      /* Low-Power transmission Slew Rate Compensation on Clock Lane */
      DSIx->WPCR[1] &= ~DSI_WPCR1_LPSRCCL;
      DSIx->WPCR[1] |= Value<<6;
    }
    else if(Lane == DSI_DATA_LANES)
    {
      /* Low-Power transmission Slew Rate Compensation on Data Lanes */
      DSIx->WPCR[1] &= ~DSI_WPCR1_LPSRCDL;
      DSIx->WPCR[1] |= Value<<8;
    }
    break;
  case DSI_HS_DELAY:
    if(Lane == DSI_CLOCK_LANE)
    {
      /* High-Speed Transmission Delay on Clock Lane */
      DSIx->WPCR[1] &= ~DSI_WPCR1_HSTXDCL;
      DSIx->WPCR[1] |= Value;
    }
    else if(Lane == DSI_DATA_LANES)
    {
      /* High-Speed Transmission Delay on Data Lanes */
      DSIx->WPCR[1] &= ~DSI_WPCR1_HSTXDDL;
      DSIx->WPCR[1] |= Value<<2;
    }
    break;
  default:
    break;
  }
}

/**
  * @brief  Low-Power Reception Filter Tuning
  * @param  DSIx: Pointer to DSI register base
  * @param  Frequency: cutoff frequency of low-pass filter at the input of LPRX
  * @retval None
  */
void DSI_SetLowPowerRXFilter(DSI_TypeDef *DSIx, uint32_t Frequency)
{  
  /* Low-Power RX low-pass Filtering Tuning */
  DSIx->WPCR[1] &= ~DSI_WPCR1_LPRXFT;
  DSIx->WPCR[1] |= Frequency<<25;
}

/**
  * @brief  Activate an additional current path on all lanes to meet the SDDTx parameter
  *         defined in the MIPI D-PHY specification
  * @param  hdsi: pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  State: ENABLE or DISABLE
  * @retval None
  */
void DSI_SetSDD(DSI_TypeDef *DSIx, FunctionalState State)
{  
  /* Check function parameters */
  assert_param(IS_FUNCTIONAL_STATE(State));
  
  /* Activate/Disactivate additional current path on all lanes */
  DSIx->WPCR[1] &= ~DSI_WPCR1_SDDC;
  DSIx->WPCR[1] |= State<<12;
}

/**
  * @brief  Custom lane pins configuration
  * @param  DSIx: Pointer to DSI register base
  * @param  CustomLane: Function to be applyed on selected lane.
  *                     This parameter can be any value of @ref DSI_CustomLane
  * @param  Lane: select between clock or data lane 0 or data lane 1.
  *               This parameter can be any value of @ref DSI_Lane_Select
  * @param  State: ENABLE or DISABLE
  * @retval None
  */
void DSI_SetLanePinsConfiguration(DSI_TypeDef *DSIx, uint32_t CustomLane, uint32_t Lane, FunctionalState State)
{
  /* Check function parameters */
  assert_param(IS_DSI_CUSTOM_LANE(CustomLane));
  assert_param(IS_DSI_LANE(Lane));
  assert_param(IS_FUNCTIONAL_STATE(State));
  
  switch(CustomLane)
  {
  case DSI_SWAP_LANE_PINS:
    if(Lane == DSI_CLOCK_LANE)
    {
      /* Swap pins on clock lane */
      DSIx->WPCR[0] &= ~DSI_WPCR0_SWCL;
      DSIx->WPCR[0] |= (State<<6);
    }
    else if(Lane == DSI_DATA_LANE0)
    {
      /* Swap pins on data lane 0 */
      DSIx->WPCR[0] &= ~DSI_WPCR0_SWDL0;
      DSIx->WPCR[0] |= (State<<7);
    }
    else if(Lane == DSI_DATA_LANE1)
    {
      /* Swap pins on data lane 1 */
      DSIx->WPCR[0] &= ~DSI_WPCR0_SWDL1;
      DSIx->WPCR[0] |= (State<<8);
    }
    break;
  case DSI_INVERT_HS_SIGNAL:
    if(Lane == DSI_CLOCK_LANE)
    {
      /* Invert HS signal on clock lane */
      DSIx->WPCR[0] &= ~DSI_WPCR0_HSICL;
      DSIx->WPCR[0] |= (State<<9);
    }
    else if(Lane == DSI_DATA_LANE0)
    {
      /* Invert HS signal on data lane 0 */
      DSIx->WPCR[0] &= ~DSI_WPCR0_HSIDL0;
      DSIx->WPCR[0] |= (State<<10);
    }
    else if(Lane == DSI_DATA_LANE1)
    {
      /* Invert HS signal on data lane 1 */
      DSIx->WPCR[0] &= ~DSI_WPCR0_HSIDL1;
      DSIx->WPCR[0] |= (State<<11);
    }
    break;
  default:
    break;
  }
}

/**
  * @brief  Set custom timing for the PHY
  * @param  DSIx: Pointer to DSI register base
  * @param  Timing: PHY timing to be adjusted.
  *                 This parameter can be any value of @ref DSI_PHY_Timing
  * @param  State: ENABLE or DISABLE
  * @param  Value: Custom value of the timing
  * @retval None
  */
void DSI_SetPHYTimings(DSI_TypeDef *DSIx, uint32_t Timing, FunctionalState State, uint32_t Value)
{  
  /* Check function parameters */
  assert_param(IS_DSI_PHY_TIMING(Timing));
  assert_param(IS_FUNCTIONAL_STATE(State));
  
  switch(Timing)
  {
  case DSI_TCLK_POST:
    /* Enable/Disable custom timing setting */
    DSIx->WPCR[0] &= ~DSI_WPCR0_TCLKPOSTEN;
    DSIx->WPCR[0] |= (State<<27);
    
    if(State)
    {
      /* Set custom value */
      DSIx->WPCR[4] &= ~DSI_WPCR4_TCLKPOST;
      DSIx->WPCR[4] |= Value;
    }
    
    break;
  case DSI_TLPX_CLK:
    /* Enable/Disable custom timing setting */
    DSIx->WPCR[0] &= ~DSI_WPCR0_TLPXCEN;
    DSIx->WPCR[0] |= (State<<26);
    
    if(State)
    {
      /* Set custom value */
      DSIx->WPCR[3] &= ~DSI_WPCR3_TLPXC;
      DSIx->WPCR[3] |= Value;
    }
    
    break;
  case DSI_THS_EXIT:
    /* Enable/Disable custom timing setting */
    DSIx->WPCR[0] &= ~DSI_WPCR0_THSEXITEN;
    DSIx->WPCR[0] |= (State<<25);
    
    if(State)
    {
      /* Set custom value */
      DSIx->WPCR[3] &= ~DSI_WPCR3_THSEXIT;
      DSIx->WPCR[3] |= Value;
    }
    
    break;
  case DSI_TLPX_DATA:
    /* Enable/Disable custom timing setting */
    DSIx->WPCR[0] &= ~DSI_WPCR0_TLPXDEN;
    DSIx->WPCR[0] |= (State<<24);
    
    if(State)
    {
      /* Set custom value */
      DSIx->WPCR[3] &= ~DSI_WPCR3_TLPXD;
      DSIx->WPCR[3] |= Value;
    }
    
    break;
  case DSI_THS_ZERO:
    /* Enable/Disable custom timing setting */
    DSIx->WPCR[0] &= ~DSI_WPCR0_THSZEROEN;
    DSIx->WPCR[0] |= (State<<23);
    
    if(State)
    {
      /* Set custom value */
      DSIx->WPCR[3] &= ~DSI_WPCR3_THSZERO;
      DSIx->WPCR[3] |= Value;
    }
    
    break;
  case DSI_THS_TRAIL:
    /* Enable/Disable custom timing setting */
    DSIx->WPCR[0] &= ~DSI_WPCR0_THSTRAILEN;
    DSIx->WPCR[0] |= (State<<22);
    
    if(State)
    {
      /* Set custom value */
      DSIx->WPCR[2] &= ~DSI_WPCR2_THSTRAIL;
      DSIx->WPCR[2] |= Value;
    }
    
    break;
  case DSI_THS_PREPARE:
    /* Enable/Disable custom timing setting */
    DSIx->WPCR[0] &= ~DSI_WPCR0_THSPREPEN;
    DSIx->WPCR[0] |= (State<<21);
    
    if(State)
    {
      /* Set custom value */
      DSIx->WPCR[2] &= ~DSI_WPCR2_THSPREP;
      DSIx->WPCR[2] |= Value;
    }
    
    break;
  case DSI_TCLK_ZERO:
    /* Enable/Disable custom timing setting */
    DSIx->WPCR[0] &= ~DSI_WPCR0_TCLKZEROEN;
    DSIx->WPCR[0] |= (State<<20);
    
    if(State)
    {
      /* Set custom value */
      DSIx->WPCR[2] &= ~DSI_WPCR2_TCLKZERO;
      DSIx->WPCR[2] |= Value;
    }
    
    break;
  case DSI_TCLK_PREPARE:
    /* Enable/Disable custom timing setting */
    DSIx->WPCR[0] &= ~DSI_WPCR0_TCLKPREPEN;
    DSIx->WPCR[0] |= (State<<19);
    
    if(State)
    {
      /* Set custom value */
      DSIx->WPCR[2] &= ~DSI_WPCR2_TCLKPREP;
      DSIx->WPCR[2] |= Value;
    }
    
    break;
  default:
    break;
  }
}

/**
  * @brief  Force the Clock/Data Lane in TX Stop Mode
  * @param  DSIx: Pointer to DSI register base
  * @param  Lane: select between clock or data lanes.
  *               This parameter can be any value of @ref DSI_Lane_Group
  * @param  State: ENABLE or DISABLE
  * @retval None
  */
void DSI_ForceTXStopMode(DSI_TypeDef *DSIx, uint32_t Lane, FunctionalState State)
{
  /* Check function parameters */
  assert_param(IS_DSI_LANE_GROUP(Lane));
  assert_param(IS_FUNCTIONAL_STATE(State));
  
  if(Lane == DSI_CLOCK_LANE)
  {
    /* Force/Unforce the Clock Lane in TX Stop Mode */
    DSIx->WPCR[0] &= ~DSI_WPCR0_FTXSMCL;
    DSIx->WPCR[0] |= (State<<12);
  }
  else if(Lane == DSI_DATA_LANES)
  {
    /* Force/Unforce the Data Lanes in TX Stop Mode */
    DSIx->WPCR[0] &= ~DSI_WPCR0_FTXSMDL;
    DSIx->WPCR[0] |= (State<<13);
  }
}

/**
  * @brief  Forces LP Receiver in Low-Power Mode
  * @param  hdsi: pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  State: ENABLE or DISABLE
  * @retval None
  */
void DSI_ForceRXLowPower(DSI_TypeDef *DSIx, FunctionalState State)
{  
  /* Check function parameters */
  assert_param(IS_FUNCTIONAL_STATE(State));
  
  /* Force/Unforce LP Receiver in Low-Power Mode */
  DSIx->WPCR[1] &= ~DSI_WPCR1_FLPRXLPM;
  DSIx->WPCR[1] |= State<<22;
}

/**
  * @brief  Force Data Lanes in RX Mode after a BTA
  * @param  hdsi: pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  State: ENABLE or DISABLE
  * @retval None
  */
void DSI_ForceDataLanesInRX(DSI_TypeDef *DSIx, FunctionalState State)
{  
  /* Check function parameters */
  assert_param(IS_FUNCTIONAL_STATE(State));
  
  /* Force Data Lanes in RX Mode */
  DSIx->WPCR[0] &= ~DSI_WPCR0_TDDL;
  DSIx->WPCR[0] |= State<<16;
}

/**
  * @brief  Enable a pull-down on the lanes to prevent from floating states when unused
  * @param  hdsi: pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  State: ENABLE or DISABLE
  * @retval None
  */
void DSI_SetPullDown(DSI_TypeDef *DSIx, FunctionalState State)
{  
  /* Check function parameters */
  assert_param(IS_FUNCTIONAL_STATE(State));
  
  /* Enable/Disable pull-down on lanes */
  DSIx->WPCR[0] &= ~DSI_WPCR0_PDEN;
  DSIx->WPCR[0] |= State<<18;
}

/**
  * @brief  Switch off the contention detection on data lanes
  * @param  hdsi: pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  State: ENABLE or DISABLE
  * @retval None
  */
void DSI_SetContentionDetectionOff(DSI_TypeDef *DSIx, FunctionalState State)
{  
  /* Check function parameters */
  assert_param(IS_FUNCTIONAL_STATE(State));
  
  /* Contention Detection on Data Lanes OFF */
  DSIx->WPCR[0] &= ~DSI_WPCR0_CDOFFDL;
  DSIx->WPCR[0] |= State<<14;
}

/**
  * @}
  */

/** @defgroup DSI_Group4 Interrupts and flags management functions
  *  @brief   Interrupts and flags management functions
  *
@verbatim   
 ===============================================================================
            ##### Interrupts and flags management functions #####
 ===============================================================================  
 
 [..] This section provides a set of functions allowing to configure the DSI Interrupts 
      sources and check or clear the flags or pending bits status.
      The user should identify which mode will be used in his application to manage 
      the communication: Polling mode or Interrupt mode. 
    
 *** Polling Mode ***
 ====================
[..] In Polling Mode, the DSI communication can be managed by 8 flags:
  (#) DSI_FLAG_TE : Tearing Effect Interrupt Flag
  (#) DSI_FLAG_ER : End of Refresh Interrupt Flag
  (#) DSI_FLAG_BUSY : Busy Flag
  (#) DSI_FLAG_PLLLS : PLL Lock Status              
  (#) DSI_FLAG_PLLL : PLL Lock Interrupt Flag
  (#) DSI_FLAG_PLLU : PLL Unlock Interrupt Flag
  (#) DSI_FLAG_RRS: Regulator Ready Status.
  (#) DSI_FLAG_RR: Regulator Ready Interrupt Flag.


 [..] In this Mode it is advised to use the following functions:
   (+) FlagStatus DSI_GetFlagStatus(DSI_TypeDef* DSIx, uint32_t DSI_FLAG);
   (+) void DSI_ClearFlag(DSI_TypeDef* DSIx, uint32_t DSI_FLAG);

 *** Interrupt Mode ***
 ======================
 [..] In Interrupt Mode, the SPI communication can be managed by 3 interrupt sources
      and 7 pending bits: 
   (+) Pending Bits:
       (##) DSI_IT_TE : Tearing Effect Interrupt Flag
       (##) DSI_IT_ER : End of Refresh Interrupt Flag
       (##) DSI_IT_PLLL : PLL Lock Interrupt Flag           
       (##) DSI_IT_PLLU : PLL Unlock Interrupt Flag
       (##) DSI_IT_RR: Regulator Ready Interrupt Flag.

   (+) Interrupt Source:
       (##) DSI_IT_TE : Tearing Effect Interrupt Enable
       (##) DSI_IT_ER : End of Refresh Interrupt Enable
       (##) DSI_IT_PLLL : PLL Lock Interrupt Enable         
       (##) DSI_IT_PLLU : PLL Unlock Interrupt Enable
       (##) DSI_IT_RR: Regulator Ready Interrupt Enable

 [..] In this Mode it is advised to use the following functions:
   (+) void DSI_ITConfig(DSI_TypeDef* DSIx, uint32_t DSI_IT, FunctionalState NewState);
   (+) ITStatus DSI_GetITStatus(DSI_TypeDef* DSIx, uint32_t DSI_IT);
   (+) void DSI_ClearITPendingBit(DSI_TypeDef* DSIx, uint32_t DSI_IT);

@endverbatim
  * @{
  */

/**
  * @brief Enables or disables the specified DSI interrupts.
  * @param DSIx: To select the DSIx peripheral, where x can be the different DSI instances 
  * @param DSI_IT: specifies the DSI interrupt sources to be enabled or disabled. 
  *          This parameter can be any combination of the following values:
  *            @arg DSI_IT_TE  : Tearing Effect Interrupt
  *            @arg DSI_IT_ER  : End of Refresh Interrupt
  *            @arg DSI_IT_PLLL: PLL Lock Interrupt
  *            @arg DSI_IT_PLLU: PLL Unlock Interrupt
  *            @arg DSI_IT_RR  : Regulator Ready Interrupt
  * @param  NewState: new state of the specified DSI interrupt.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DSI_ITConfig(DSI_TypeDef* DSIx, uint32_t DSI_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DSI_ALL_PERIPH(DSIx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  assert_param(IS_DSI_IT(DSI_IT));

  if(NewState != DISABLE)
  {
    /* Enable the selected DSI interrupt */
    DSIx->WIER |= DSI_IT;
  }
  else
  {
    /* Disable the selected DSI interrupt */
    DSIx->WIER &= ~DSI_IT;
  }
}

/**
  * @brief Checks whether the specified DSI flag is set or not.
  * @param DSIx: To select the DSIx peripheral, where x can be the different DSI instances 
  * @param DSI_FLAG: specifies the SPI flag to be checked. 
  *          This parameter can be one of the following values:
  *            @arg DSI_FLAG_TE   : Tearing Effect Interrupt Flag 
  *            @arg DSI_FLAG_ER   : End of Refresh Interrupt Flag 
  *            @arg DSI_FLAG_BUSY : Busy Flag
  *            @arg DSI_FLAG_PLLLS: PLL Lock Status
  *            @arg DSI_FLAG_PLLL : PLL Lock Interrupt Flag
  *            @arg DSI_FLAG_PLLU : PLL Unlock Interrupt Flag
  *            @arg DSI_FLAG_RRS  : Regulator Ready Flag
  *            @arg DSI_FLAG_RR   : Regulator Ready Interrupt Flag 
  * @retval The new state of DSI_FLAG (SET or RESET).
  */
FlagStatus DSI_GetFlagStatus(DSI_TypeDef* DSIx, uint16_t DSI_FLAG)
{
  FlagStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_DSI_ALL_PERIPH(DSIx));
  assert_param(IS_DSI_GET_FLAG(DSI_FLAG));
  
  /* Check the status of the specified DSI flag */
  if((DSIx->WISR & DSI_FLAG) != (uint32_t)RESET)
  {
    /* DSI_FLAG is set */
    bitstatus = SET;
  }
  else
  {
    /* DSI_FLAG is reset */
    bitstatus = RESET;
  }
  /* Return the DSI_FLAG status */
  return  bitstatus;
}

/**
  * @brief Clears the specified DSI flag.
  * @param DSIx: To select the DSIx peripheral, where x can be the different DSI instances 
  * @param DSI_FLAG: specifies the SPI flag to be cleared. 
  *          This parameter can be one of the following values:
  *            @arg DSI_FLAG_TE   : Tearing Effect Interrupt Flag 
  *            @arg DSI_FLAG_ER   : End of Refresh Interrupt Flag 
  *            @arg DSI_FLAG_PLLL : PLL Lock Interrupt Flag
  *            @arg DSI_FLAG_PLLU : PLL Unlock Interrupt Flag
  *            @arg DSI_FLAG_RR   : Regulator Ready Interrupt Flag  
  * @retval None
  */
void DSI_ClearFlag(DSI_TypeDef* DSIx, uint16_t DSI_FLAG)
{
  /* Check the parameters */
  assert_param(IS_DSI_ALL_PERIPH(DSIx));
  assert_param(IS_DSI_CLEAR_FLAG(DSI_FLAG));
    
  /* Clear the selected DSI flag */
  DSIx->WIFCR = (uint32_t)DSI_FLAG;
}

/**
  * @brief Checks whether the specified DSIx interrupt has occurred or not.
  * @param DSIx: To select the DSIx peripheral, where x can be the different DSI instances 
  * @param DSI_IT: specifies the DSI interrupt sources to be checked. 
  *          This parameter can be one of the following values:
  *            @arg DSI_IT_TE  : Tearing Effect Interrupt
  *            @arg DSI_IT_ER  : End of Refresh Interrupt
  *            @arg DSI_IT_PLLL: PLL Lock Interrupt
  *            @arg DSI_IT_PLLU: PLL Unlock Interrupt
  *            @arg DSI_IT_RR  : Regulator Ready Interrupt 
  * @retval The new state of SPI_I2S_IT (SET or RESET).
  */
ITStatus DSI_GetITStatus(DSI_TypeDef* DSIx, uint32_t DSI_IT)
{
  ITStatus bitstatus = RESET;
  uint32_t enablestatus = 0;

  /* Check the parameters */
  assert_param(IS_DSI_ALL_PERIPH(DSIx));
  assert_param(IS_DSI_IT(DSI_IT));

  /* Get the DSI_IT enable bit status */
  enablestatus = (DSIx->WIER & DSI_IT);

  /* Check the status of the specified SPI interrupt */
  if (((DSIx->WISR & DSI_IT) != (uint32_t)RESET) && enablestatus)
  {
    /* DSI_IT is set */
    bitstatus = SET;
  }
  else
  {
    /* DSI_IT is reset */
    bitstatus = RESET;
  }
  
  /* Return the DSI_IT status */
  return bitstatus;
}

/**
  * @brief Clears the DSIx interrupt pending bit.
  * @param DSIx: To select the DSIx peripheral, where x can be the different DSI instances 
  * @param DSI_IT: specifies the DSI interrupt sources to be cleared. 
  *        This parameter can be one of the following values:
  *            @arg DSI_IT_TE  : Tearing Effect Interrupt
  *            @arg DSI_IT_ER  : End of Refresh Interrupt
  *            @arg DSI_IT_PLLL: PLL Lock Interrupt
  *            @arg DSI_IT_PLLU: PLL Unlock Interrupt
  *            @arg DSI_IT_RR  : Regulator Ready Interrupt
  * @retval None
  */
void DSI_ClearITPendingBit(DSI_TypeDef* DSIx, uint32_t DSI_IT)
{
  /* Check the parameters */
  assert_param(IS_DSI_ALL_PERIPH(DSIx));
  assert_param(IS_DSI_IT(DSI_IT));

  /* Clear the selected DSI interrupt pending bit */
  DSIx->WIFCR = (uint32_t)DSI_IT;
}

/**
  * @brief  Enable the error monitor flags 
  * @param  DSIx: To select the DSIx peripheral, where x can be the different DSI instances 
  * @param  ActiveErrors: indicates which error interrupts will be enabled.
  *                      This parameter can be any combination of @ref DSI_Error_Data_Type.
  * @retval None 
  */
void DSI_ConfigErrorMonitor(DSI_TypeDef *DSIx, uint32_t ActiveErrors)
{
  DSIx->IER[0] = 0;
  DSIx->IER[1] = 0;
    
  if(ActiveErrors & DSI_ERROR_ACK)
  {
    /* Enable the interrupt generation on selected errors */
    DSIx->IER[0] |= DSI_ERROR_ACK_MASK;
  }
  
  if(ActiveErrors & DSI_ERROR_PHY)
  {
    /* Enable the interrupt generation on selected errors */
    DSIx->IER[0] |= DSI_ERROR_PHY_MASK;
  }
  
  if(ActiveErrors & DSI_ERROR_TX)
  {
    /* Enable the interrupt generation on selected errors */
    DSIx->IER[1] |= DSI_ERROR_TX_MASK;
  }
  
  if(ActiveErrors & DSI_ERROR_RX)
  {
    /* Enable the interrupt generation on selected errors */
    DSIx->IER[1] |= DSI_ERROR_RX_MASK;
  }
  
  if(ActiveErrors & DSI_ERROR_ECC)
  {
    /* Enable the interrupt generation on selected errors */
    DSIx->IER[1] |= DSI_ERROR_ECC_MASK;
  }
  
  if(ActiveErrors & DSI_ERROR_CRC)
  {
    /* Enable the interrupt generation on selected errors */
    DSIx->IER[1] |= DSI_ERROR_CRC_MASK;
  }
  
  if(ActiveErrors & DSI_ERROR_PSE)
  {
    /* Enable the interrupt generation on selected errors */
    DSIx->IER[1] |= DSI_ERROR_PSE_MASK;
  }
  
  if(ActiveErrors & DSI_ERROR_EOT)
  {
    /* Enable the interrupt generation on selected errors */
    DSIx->IER[1] |= DSI_ERROR_EOT_MASK;
  }
  
  if(ActiveErrors & DSI_ERROR_OVF)
  {
    /* Enable the interrupt generation on selected errors */
    DSIx->IER[1] |= DSI_ERROR_OVF_MASK;
  }
  
  if(ActiveErrors & DSI_ERROR_GEN)
  {
    /* Enable the interrupt generation on selected errors */
    DSIx->IER[1] |= DSI_ERROR_GEN_MASK;
  }
}

/**
  * @}
  */

/**
  * @}
  */
#endif /* STM32F469_479xx */  
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
