/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_gpio.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file provides all the GPIO firmware functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417_gpio.h"
#include "ch32h417_rcc.h"

/* MASK */
#define ECR_PORTPINCONFIG_MASK    ((uint16_t)0xFF80)
#define LSB_MASK                  ((uint16_t)0xFFFF)
#define DBGAFR_POSITION_MASK      ((uint32_t)0x000F0000)
#define DBGAFR_SWJCFG_MASK        ((uint32_t)0xF0FFFFFF)
#define DBGAFR_LOCATION_MASK      ((uint32_t)0x00200000)
#define DBGAFR_NUMBITS_MASK       ((uint32_t)0x00100000)

/*********************************************************************
 * @fn      GPIO_DeInit
 *
 * @brief   Deinitializes the GPIOx peripheral registers to their default
 *        reset values.
 *
 * @param   GPIOx - where x can be (A..F) to select the GPIO peripheral.
 *
 * @return  none
 */
void GPIO_DeInit(GPIO_TypeDef *GPIOx)
{
    if(GPIOx == GPIOA)
    {
        RCC_HB2PeriphResetCmd(RCC_HB2Periph_GPIOA, ENABLE);
        RCC_HB2PeriphResetCmd(RCC_HB2Periph_GPIOA, DISABLE);
    }
    else if(GPIOx == GPIOB)
    {
        RCC_HB2PeriphResetCmd(RCC_HB2Periph_GPIOB, ENABLE);
        RCC_HB2PeriphResetCmd(RCC_HB2Periph_GPIOB, DISABLE);
    }
    else if(GPIOx == GPIOC)
    {
        RCC_HB2PeriphResetCmd(RCC_HB2Periph_GPIOC, ENABLE);
        RCC_HB2PeriphResetCmd(RCC_HB2Periph_GPIOC, DISABLE);
    }
    else if(GPIOx == GPIOD)
    {
        RCC_HB2PeriphResetCmd(RCC_HB2Periph_GPIOD, ENABLE);
        RCC_HB2PeriphResetCmd(RCC_HB2Periph_GPIOD, DISABLE);
    }
    else if(GPIOx == GPIOE)
    {
        RCC_HB2PeriphResetCmd(RCC_HB2Periph_GPIOE, ENABLE);
        RCC_HB2PeriphResetCmd(RCC_HB2Periph_GPIOE, DISABLE);
    }
    else if(GPIOx == GPIOF)
    {
        RCC_HB2PeriphResetCmd(RCC_HB2Periph_GPIOF, ENABLE);
        RCC_HB2PeriphResetCmd(RCC_HB2Periph_GPIOF, DISABLE);
    }
}

/*********************************************************************
 * @fn      GPIO_AFIODeInit
 *
 * @brief   Deinitializes the Alternate Functions (remap, event control
 *        and EXTI configuration) registers to their default reset values.
 *
 * @return  none
 */
void GPIO_AFIODeInit(void)
{
    RCC_HB2PeriphResetCmd(RCC_HB2Periph_AFIO, ENABLE);
    RCC_HB2PeriphResetCmd(RCC_HB2Periph_AFIO, DISABLE);
}

/*********************************************************************
 * @fn      GPIO_Init
 *
 * @brief   GPIOx - where x can be (A..F) to select the GPIO peripheral.
 *
 * @param   GPIO_InitStruct - pointer to a GPIO_InitTypeDef structure that
 *        contains the configuration information for the specified GPIO peripheral.
 *
 * @return  none
 */
__attribute__((optimize("Ofast")))
void GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_InitStruct)
{
    uint32_t currentmode = 0x00, currentpin = 0x00, pinpos = 0x00, pos = 0x00;
    uint32_t tmpreg = 0x00, pinmask = 0x00;

    currentmode = ((uint32_t)GPIO_InitStruct->GPIO_Mode) & ((uint32_t)0x0F);

    if((((uint32_t)GPIO_InitStruct->GPIO_Mode) & ((uint32_t)0x10)) != 0x00)
    {
        currentmode |= (uint32_t)0x01;
        
        tmpreg = GPIOx->SPEED;
        for(pinpos = 0x00; pinpos < 0x10; pinpos++)
        {
            pos = ((uint32_t)0x01) << pinpos;
            currentpin = (GPIO_InitStruct->GPIO_Pin) & pos;

            if(currentpin == pos)
            {
                pos = pinpos << 1;
                pinmask = ((uint32_t)0x03) << pos;
                tmpreg &= ~pinmask;
                tmpreg |= (((uint32_t)GPIO_InitStruct->GPIO_Speed) << pos);
            }
        }
        GPIOx->SPEED = tmpreg;
    }

    if(((uint32_t)GPIO_InitStruct->GPIO_Pin & ((uint32_t)0x00FF)) != 0x00)
    {
        tmpreg = GPIOx->CFGLR;

        for(pinpos = 0x00; pinpos < 0x08; pinpos++)
        {
            pos = ((uint32_t)0x01) << pinpos;
            currentpin = (GPIO_InitStruct->GPIO_Pin) & pos;

            if(currentpin == pos)
            {
                pos = pinpos << 2;
                pinmask = ((uint32_t)0x0F) << pos;
                tmpreg &= ~pinmask;
                tmpreg |= (currentmode << pos);

                if(GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPD)
                {
                    GPIOx->BCR = (((uint32_t)0x01) << pinpos);
                }
                else
                {
                    if(GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPU)
                    {
                        GPIOx->BSHR = (((uint32_t)0x01) << pinpos);
                    }
                }
            }
        }
        GPIOx->CFGLR = tmpreg;
    }

    if(GPIO_InitStruct->GPIO_Pin > 0x00FF)
    {
        tmpreg = GPIOx->CFGHR;

        for(pinpos = 0x00; pinpos < 0x08; pinpos++)
        {
            pos = (((uint32_t)0x01) << (pinpos + 0x08));
            currentpin = ((GPIO_InitStruct->GPIO_Pin) & pos);

            if(currentpin == pos)
            {
                pos = pinpos << 2;
                pinmask = ((uint32_t)0x0F) << pos;
                tmpreg &= ~pinmask;
                tmpreg |= (currentmode << pos);

                if(GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPD)
                {
                    GPIOx->BCR = (((uint32_t)0x01) << (pinpos + 0x08));
                }

                if(GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPU)
                {
                    GPIOx->BSHR = (((uint32_t)0x01) << (pinpos + 0x08));
                }
            }
        }
        GPIOx->CFGHR = tmpreg;
    }
}

/*********************************************************************
 * @fn      GPIO_StructInit
 *
 * @brief   Fills each GPIO_InitStruct member with its default
 *
 * @param   GPIO_InitStruct - pointer to a GPIO_InitTypeDef structure
 *      which will be initialized.
 *
 * @return  none
 */
void GPIO_StructInit(GPIO_InitTypeDef *GPIO_InitStruct)
{
    GPIO_InitStruct->GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStruct->GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStruct->GPIO_Mode = GPIO_Mode_IN_FLOATING;
}

/*********************************************************************
 * @fn      GPIO_ReadInputDataBit
 *
 * @brief   GPIOx - where x can be (A..F) to select the GPIO peripheral.
 *
 * @param   GPIO_Pin - specifies the port bit to read.
 *            This parameter can be GPIO_Pin_x where x can be (0..15).
 *
 * @return  The input port pin value.
 */
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    uint8_t bitstatus = 0x00;

    if((GPIOx->INDR & GPIO_Pin) != (uint32_t)Bit_RESET)
    {
        bitstatus = (uint8_t)Bit_SET;
    }
    else
    {
        bitstatus = (uint8_t)Bit_RESET;
    }

    return bitstatus;
}

/*********************************************************************
 * @fn      GPIO_ReadInputData
 *
 * @brief   Reads the specified GPIO input data port.
 *
 * @param   GPIOx - where x can be (A..F) to select the GPIO peripheral.
 *
 * @return  The output port pin value.
 */
uint16_t GPIO_ReadInputData(GPIO_TypeDef *GPIOx)
{
    return ((uint16_t)GPIOx->INDR);
}

/*********************************************************************
 * @fn      GPIO_ReadOutputDataBit
 *
 * @brief   Reads the specified output data port bit.
 *
 * @param   GPIOx - where x can be (A..F) to select the GPIO peripheral.
 *          GPIO_Pin - specifies the port bit to read.
 *            This parameter can be GPIO_Pin_x where x can be (0..15).
 *
 * @return  none
 */
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    uint8_t bitstatus = 0x00;

    if((GPIOx->OUTDR & GPIO_Pin) != (uint32_t)Bit_RESET)
    {
        bitstatus = (uint8_t)Bit_SET;
    }
    else
    {
        bitstatus = (uint8_t)Bit_RESET;
    }

    return bitstatus;
}

/*********************************************************************
 * @fn      GPIO_ReadOutputData
 *
 * @brief   Reads the specified GPIO output data port.
 *
 * @param   GPIOx - where x can be (A..F) to select the GPIO peripheral.
 *
 * @return  GPIO output port pin value.
 */
uint16_t GPIO_ReadOutputData(GPIO_TypeDef *GPIOx)
{
    return ((uint16_t)GPIOx->OUTDR);
}

/*********************************************************************
 * @fn      GPIO_SetBits
 *
 * @brief   Sets the selected data port bits.
 *
 * @param   GPIOx - where x can be (A..F) to select the GPIO peripheral.
 *          GPIO_Pin - specifies the port bits to be written.
 *            This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
 *
 * @return  none
 */
void GPIO_SetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIOx->BSHR = GPIO_Pin;
}

/*********************************************************************
 * @fn      GPIO_ResetBits
 *
 * @brief   Clears the selected data port bits.
 *
 * @param   GPIOx - where x can be (A..F) to select the GPIO peripheral.
 *          GPIO_Pin - specifies the port bits to be written.
 *            This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
 *
 * @return  none
 */
void GPIO_ResetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIOx->BCR = GPIO_Pin;
}

/*********************************************************************
 * @fn      GPIO_WriteBit
 *
 * @brief   Sets or clears the selected data port bit.
 *
 * @param   GPIO_Pin - specifies the port bit to be written.
 *            This parameter can be one of GPIO_Pin_x where x can be (0..15).
 *          BitVal - specifies the value to be written to the selected bit.
 *            Bit_RESET - to clear the port pin.
 *            Bit_SET - to set the port pin.
 *
 * @return  none
 */
void GPIO_WriteBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, BitAction BitVal)
{
    if(BitVal != Bit_RESET)
    {
        GPIOx->BSHR = GPIO_Pin;
    }
    else
    {
        GPIOx->BCR = GPIO_Pin;
    }
}

/*********************************************************************
 * @fn      GPIO_Write
 *
 * @brief   Writes data to the specified GPIO data port.
 *
 * @param   GPIOx - where x can be (A..F) to select the GPIO peripheral.
 *          PortVal - specifies the value to be written to the port output data register.
 *
 * @return  none
 */
void GPIO_Write(GPIO_TypeDef *GPIOx, uint16_t PortVal)
{
    GPIOx->OUTDR = PortVal;
}

/*********************************************************************
 * @fn      GPIO_PinLockConfig
 *
 * @brief   Locks GPIO Pins configuration registers.
 *
 * @param   GPIOx - where x can be (A..F) to select the GPIO peripheral.
 *          GPIO_Pin - specifies the port bit to be written.
 *            This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
 *
 * @return  none
 */
void GPIO_PinLockConfig(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    uint32_t tmp = 0x00010000;

    tmp |= GPIO_Pin;
    GPIOx->LCKR = tmp;
    GPIOx->LCKR = GPIO_Pin;
    GPIOx->LCKR = tmp;
    tmp = GPIOx->LCKR;
    tmp = GPIOx->LCKR;
}

/*********************************************************************
 * @fn      GPIO_PinRemapConfig
 *
 * @brief   Changes the mapping of the specified pin.
 *
 * @param   GPIO_Remap - selects the pin to remap.
 *            GPIO_Remap_PD0PD1 - PD0 and PD1 Alternate Function mapping
 *            GPIO_Remap_ADC1_ETRGINJ - ADC1 external trigger injection conversion mapping
 *            GPIO_Remap_ADC1_ETRGREG - ADC1 external trigger regular conversion mapping
 *            GPIO_Remap_ADC2_ETRGINJ - ADC1 external trigger injection conversion mapping
 *            GPIO_Remap_ADC2_ETRGREG - ADC1 external trigger regular conversion mapping
 *            GPIO_PartialRemap_UHSIF_CLK - UHSIF CLK Partial Alternate Function mapping
 *            GPIO_PartialRemap1_UHSIF_CLK - UHSIF CLK Partial1 Alternate Function mapping
 *            GPIO_FullRemap_UHSIF_CLK - UHSIF CLK Full Alternate Function mapping
 *            GPIO_PartialRemap_UHSIF_PORT - UHSIF Port Partial Alternate Function mapping
 *            GPIO_FullRemap_UHSIF_PORT - UHSIF Port Full Alternate Function mapping
 *            GPIO_PartialRemap_SDMMC - SDMMC Partial Alternate Function mapping
 *            GPIO_FullRemap_SDMMC - SDMMC Full Alternate Function mapping
 *            GPIO_Remap_TIM2ITR1 - TIM2 TRIG Partial2 Alternate Function mapping
 *            GPIO_Remap_VIO1V8_IO_HSLV - VIO(1.8V) GPIO speed configration Function mapping
 *            GPIO_Remap_VIO3V3_IO_HSLV - VIO(3.3V) GPIO speed configration Function mapping
 *            GPIO_Remap_VDD3V3_IO_HSLV - VDD(3.3V) GPIO speed configration Function mapping
 *            GPIO_Remap_SWJ_Disable - SWJ Disable Function mapping
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void GPIO_PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewState)
{
    uint32_t tmp = 0x00, tmp1 = 0x00, tmpreg = 0x00, tmpmask = 0x00;

    tmpreg = AFIO->PCFR1;
    tmpmask = (GPIO_Remap & DBGAFR_POSITION_MASK) >> 0x10;
    tmp = GPIO_Remap & LSB_MASK;

    if((GPIO_Remap & (DBGAFR_LOCATION_MASK | DBGAFR_NUMBITS_MASK)) == (DBGAFR_LOCATION_MASK | DBGAFR_NUMBITS_MASK)) /* [26:24] 3bit SWD_JTAG */
    {
        tmpreg &= DBGAFR_SWJCFG_MASK;
        AFIO->PCFR1 &= DBGAFR_SWJCFG_MASK;
    }
    else if((GPIO_Remap & DBGAFR_NUMBITS_MASK) == DBGAFR_NUMBITS_MASK) /* [15:0] 2bit */
    {
        tmp1 = ((uint32_t)0x03) << tmpmask;
        tmpreg &= ~tmp1;
        tmpreg |= ~DBGAFR_SWJCFG_MASK;
    }
    else /* [31:0] 1bit */
    {
        tmpreg &= ~(tmp << ((GPIO_Remap >> 0x15) * 0x10));
        tmpreg |= ~DBGAFR_SWJCFG_MASK;
    }

    /* Set bit */
    if(NewState != DISABLE)
    {
        tmpreg |= (tmp << (((GPIO_Remap & 0x00FFFFFF) >> 0x15) * 0x10));
    }

    AFIO->PCFR1 = tmpreg;
}

/*********************************************************************
 * @fn      GPIO_PinAFConfig
 *
 * @brief   Port multiplexing function.
 *
 * @param   GPIOx where x can be (A..F).
 *          GPIO_PinSource - specifies the pin to be configured.
 *            This parameter can be GPIO_PinSourcex where x can be (0..15).
 *          GPIO_AF - The number of the multiplexed function.
 *            This parameter can refer:CH32V417DS0.pdf.
 *
 * @return  none
 */
__attribute__((optimize("Ofast")))
void GPIO_PinAFConfig(GPIO_TypeDef *GPIOx, uint8_t GPIO_PinSource, uint8_t GPIO_AF)
{
    uint8_t tmp;

    if(GPIO_PinSource >= 0x08)
    {
        tmp = GPIO_PinSource - 0x08;
    }
    else
    {
        tmp = GPIO_PinSource;
    }

    if(GPIOx == GPIOA)
    {
        if(GPIO_PinSource >= 0x08)
        {
            AFIO->GPIOA_AFHR &= ~(0xF << (tmp << 2));
            AFIO->GPIOA_AFHR |= (GPIO_AF << (tmp << 2));
        }
        else
        {
            AFIO->GPIOA_AFLR &= ~(0xF << (tmp << 2));
            AFIO->GPIOA_AFLR |= (GPIO_AF << (tmp << 2));
        }
    }
    else if(GPIOx == GPIOB)
    {
        if(GPIO_PinSource >= 0x08)
        {
            AFIO->GPIOB_AFHR &= ~(0xF << (tmp << 2));
            AFIO->GPIOB_AFHR |= (GPIO_AF << (tmp << 2));
        }
        else
        {
            AFIO->GPIOB_AFLR &= ~(0xF << (tmp << 2));
            AFIO->GPIOB_AFLR |= (GPIO_AF << (tmp << 2));
        }
    }
    else if(GPIOx == GPIOC)
    {
        if(GPIO_PinSource >= 0x08)
        {
            AFIO->GPIOC_AFHR &= ~(0xF << (tmp << 2));
            AFIO->GPIOC_AFHR |= (GPIO_AF << (tmp << 2));
        }
        else
        {
            AFIO->GPIOC_AFLR &= ~(0xF << (tmp << 2));
            AFIO->GPIOC_AFLR |= (GPIO_AF << (tmp << 2));
        }
    }
    else if(GPIOx == GPIOD)
    {
        if(GPIO_PinSource >= 0x08)
        {
            AFIO->GPIOD_AFHR &= ~(0xF << (tmp << 2));
            AFIO->GPIOD_AFHR |= (GPIO_AF << (tmp << 2));
        }
        else
        {
            AFIO->GPIOD_AFLR &= ~(0xF << (tmp << 2));
            AFIO->GPIOD_AFLR |= (GPIO_AF << (tmp << 2));
        }
    }
    else if(GPIOx == GPIOE)
    {
        if(GPIO_PinSource >= 0x08)
        {
            AFIO->GPIOE_AFHR &= ~(0xF << (tmp << 2));
            AFIO->GPIOE_AFHR |= (GPIO_AF << (tmp << 2));
        }
        else
        {
            AFIO->GPIOE_AFLR &= ~(0xF << (tmp << 2));
            AFIO->GPIOE_AFLR |= (GPIO_AF << (tmp << 2));
        }
    }
    else if(GPIOx == GPIOF)
    {
        if(GPIO_PinSource >= 0x08)
        {
            AFIO->GPIOF_AFHR &= ~(0xF << (tmp << 2));
            AFIO->GPIOF_AFHR |= (GPIO_AF << (tmp << 2));
        }
        else
        {
            AFIO->GPIOF_AFLR &= ~(0xF << (tmp << 2));
            AFIO->GPIOF_AFLR |= (GPIO_AF << (tmp << 2));
        }
    }
}

/*********************************************************************
 * @fn      GPIO_EXTILineConfig
 *
 * @brief   Selects the GPIO pin used as EXTI Line.
 *
 * @param   GPIO_PortSource - selects the GPIO port to be used as source for EXTI lines.
 *            This parameter can be GPIO_PortSourceGPIOx where x can be (A..F) and GPIO_PortSourceCMP.
 *          GPIO_PinSource - specifies the EXTI line to be configured.
 *            This parameter can be GPIO_PinSourcex where x can be (0..15).
 *
 * @return  none
 */
void GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource)
{
    uint8_t tmp;

    if(GPIO_PinSource >= 0x8)
    {
        tmp = GPIO_PinSource - 8;
        AFIO->EXTICR2 &= ~(0xF << (tmp << 2));
        AFIO->EXTICR2 |= (GPIO_PortSource << (tmp << 2));
    }
    else
    {
        tmp = GPIO_PinSource;
        AFIO->EXTICR1 &= ~(0xF << (tmp << 2));
        AFIO->EXTICR1 |= (GPIO_PortSource << (tmp << 2));
    }
}

/*********************************************************************
 * @fn      GPIO_IPD_Unused
 *
 * @brief   Configure unused GPIO as input pull-down.
 *
 * @param   none
 *
 * @return  none
 */
void GPIO_IPD_Unused(void)
{

}


