/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_flash.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file provides all the FLASH firmware functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417_flash.h"
#include <stdlib.h>
/* Flash Control Register bits */
#define CR_PG_Set                  ((uint32_t)0x00000001)
#define CR_PG_Reset                ((uint32_t)0xFFFFFFFE)
#define CR_PER_Set                 ((uint32_t)0x00000002)
#define CR_PER_Reset               ((uint32_t)0xFFFFFFFD)
#define CR_OPTPG_Set               ((uint32_t)0x00000010)
#define CR_OPTPG_Reset             ((uint32_t)0xFFFFFFEF)
#define CR_OPTER_Set               ((uint32_t)0x00000020)
#define CR_OPTER_Reset             ((uint32_t)0xFFFFFFDF)
#define CR_STRT_Set                ((uint32_t)0x00000040)
#define CR_LOCK_Set                ((uint32_t)0x00000080)
#define CR_FLOCK_Set               ((uint32_t)0x00008000)
#define CR_PAGE_PG                 ((uint32_t)0x00010000)
#define CR_BER                     ((uint32_t)0x00040000)
#define CR_PG_STRT                 ((uint32_t)0x00200000)

/* FLASH Status Register bits */
#define SR_BSY                     ((uint32_t)0x00000001)
#define SR_WR_BSY                  ((uint32_t)0x00000002)
#define SR_WRPRTERR                ((uint32_t)0x00000010)
#define SR_EOP                     ((uint32_t)0x00000020)

/* FLASH Mask */
#define RDPRT_Mask                 ((uint32_t)0x00000002)
#define WRP0_Mask                  ((uint32_t)0x000000FF)
#define WRP1_Mask                  ((uint32_t)0x0000FF00)
#define WRP2_Mask                  ((uint32_t)0x00FF0000)
#define WRP3_Mask                  ((uint32_t)0xFF000000)
#define OB_USER_BFB2               ((uint16_t)0x0008)

/* FLASH Keys */
#define RDP_Key                    ((uint16_t)0x00A5)
#define FLASH_KEY1                 ((uint32_t)0x45670123)
#define FLASH_KEY2                 ((uint32_t)0xCDEF89AB)

/* FLASH BANK address */
#define FLASH_BANK1_END_ADDRESS    ((uint32_t)0x807FFFF)

/* Delay definition */
#define EraseTimeout               ((uint32_t)0x0A000000)
#define ProgramTimeout             ((uint32_t)0x00100000)

/* Flash Program Valid Address */
#define ValidAddrStart             (FLASH_BASE)
#define ValidAddrEnd_Dual          (FLASH_BASE + 0xF0000)
#define ValidAddrEnd_Signal        (FLASH_BASE + 0x78000)

/* FLASH Size */
#define Size_256B                  0x100
#define Size_4KB                   0x1000
#define Size_8KB                   0x2000
#define Size_32KB                  0x8000
#define Size_64KB                  0x10000

/*********************************************************************
 * @fn      FLASH_Unlock
 *
 * @brief   Unlocks the FLASH Program Erase Controller.
 *
 * @return  none
 */
void FLASH_Unlock(void)
{
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
}

/*********************************************************************
 * @fn      FLASH_Lock
 *
 * @brief   Locks the FLASH Program Erase Controller.
 *
 * @return  none
 */
void FLASH_Lock(void)
{
    FLASH->CTLR |= CR_LOCK_Set;
}

/*********************************************************************
 * @fn      FLASH_ErasePage
 *
 * @brief   Erases a specified FLASH page(page size 4KB or 8KB).
 *        If the flash is Dual flash mode,then page erase size is 8KB.
 *        If the flash is single flash mode,then page erase size is 4KB.
 *
 * @param   Page_Address - The page address to be erased.
 *
 * @return  FLASH Status - The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
 *        FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status FLASH_ErasePage(uint32_t Page_Address)
{
    if(((*(vu32*)FLASH_CFGR0_BASE) & (1<<28)) != 0)
    {
        Page_Address &= 0xFFFFE000;
    }
    else 
    {
        Page_Address &= 0xFFFFF000;
    }
    FLASH_Status status = FLASH_COMPLETE;
while((FLASH->STATR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY )
;
    // status = FLASH_WaitForLastOperation(EraseTimeout);

    if(status == FLASH_COMPLETE)
    {
        FLASH->CTLR |= CR_PER_Set;
        FLASH->ADDR = Page_Address;
        FLASH->CTLR |= CR_STRT_Set;
while((FLASH->STATR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY )
;
        // status = FLASH_WaitForLastOperation(EraseTimeout);

        FLASH->CTLR &= CR_PER_Reset;
    }

    return status;
}

/*********************************************************************
 * @fn      FLASH_EraseOptionBytes
 *
 * @brief   Erases the FLASH option bytes.
 *
 * @return  FLASH Status - The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
 *        FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status FLASH_EraseOptionBytes(void)
{
    uint16_t     rdptmp = RDP_Key;
    uint32_t     Address = 0x1FFFF800;
    __IO uint8_t i;

    FLASH_Status status = FLASH_COMPLETE;
    if(FLASH_GetReadOutProtectionStatus() != RESET)
    {
        rdptmp = 0x00;
    }
    status = FLASH_WaitForLastOperation(EraseTimeout);
    if(status == FLASH_COMPLETE)
    {
        FLASH->OBKEYR = FLASH_KEY1;
        FLASH->OBKEYR = FLASH_KEY2;

        FLASH->CTLR |= CR_OPTER_Set;
        FLASH->CTLR |= CR_STRT_Set;
        status = FLASH_WaitForLastOperation(EraseTimeout);

        if(status == FLASH_COMPLETE)
        {
            FLASH->CTLR &= CR_OPTER_Reset;
            FLASH->CTLR |= CR_OPTPG_Set;
            OB->RDPR = (uint16_t)rdptmp;
            status = FLASH_WaitForLastOperation(ProgramTimeout);

            if(status != FLASH_TIMEOUT)
            {
                FLASH->CTLR &= CR_OPTPG_Reset;
            }
        }
        else
        {
            if(status != FLASH_TIMEOUT)
            {
                FLASH->CTLR &= CR_OPTPG_Reset;
            }
        }

        /* Write 0xFF */
        FLASH->CTLR |= CR_OPTPG_Set;

        for(i = 0; i < 8; i++)
        {
            *(uint16_t *)(Address + 2 * i) = 0x00FF;
#ifdef Core_V5F
            __asm("fence");
#endif
            while(FLASH->STATR & SR_BSY)
                ;
        }

        FLASH->CTLR &= ~CR_OPTPG_Set;
    }
    return status;
}

/*********************************************************************
 * @fn      FLASH_ProgramWord
 *
 * @brief   Programs a word at a specified address.
 *
 * @param   Address - specifies the address to be programmed.
 *          Data - specifies the data to be programmed.
 *
 * @return  FLASH Status - The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
 *        FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data)
{
    FLASH_Status  status = FLASH_COMPLETE;
    __IO uint32_t tmp = 0;

    status = FLASH_WaitForLastOperation(ProgramTimeout);
    if(status == FLASH_COMPLETE)
    {
        FLASH->CTLR |= CR_PG_Set;

        *(__IO uint16_t *)Address = (uint16_t)Data;
        status = FLASH_WaitForLastOperation(ProgramTimeout);
        if(status == FLASH_COMPLETE)
        {
            tmp = Address + 2;
            *(__IO uint16_t *)tmp = Data >> 16;
            status = FLASH_WaitForLastOperation(ProgramTimeout);
            FLASH->CTLR &= CR_PG_Reset;
        }
        else
        {
            FLASH->CTLR &= CR_PG_Reset;
        }
    }

    return status;
}

/*********************************************************************
 * @fn      FLASH_ProgramHalfWord
 *
 * @brief   Programs a half word at a specified address.
 *
 * @param   Address - specifies the address to be programmed.
 *          Data - specifies the data to be programmed.
 *
 * @return  FLASH Status - The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
 *        FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data)
{
    FLASH_Status status = FLASH_COMPLETE;

    status = FLASH_WaitForLastOperation(ProgramTimeout);

    if(status == FLASH_COMPLETE)
    {
        FLASH->CTLR |= CR_PG_Set;
        *(__IO uint16_t *)Address = Data;

        status = FLASH_WaitForLastOperation(ProgramTimeout);

        FLASH->CTLR &= CR_PG_Reset;
    }

    return status;
}

/*********************************************************************
 * @fn      FLASH_ProgramOptionByteData
 *
 * @brief   Programs a half word at a specified Option Byte Data address.
 *
 * @param   Address - specifies the address to be programmed.
 *          Data - specifies the data to be programmed.
 *
 * @return  FLASH Status - The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
 *        FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status FLASH_ProgramOptionByteData(uint32_t Address, uint8_t Data)
{
    FLASH_Status status = FLASH_COMPLETE;
    uint32_t     Addr = 0x1FFFF800;
    __IO uint8_t i;
    uint16_t     pbuf[8];

    status = FLASH_WaitForLastOperation(ProgramTimeout);
    if(status == FLASH_COMPLETE)
    {
        FLASH->OBKEYR = FLASH_KEY1;
        FLASH->OBKEYR = FLASH_KEY2;

        /* Read optionbytes */
        for(i = 0; i < 8; i++)
        {
            pbuf[i] = *(uint16_t *)(Addr + 2 * i);
        }

        /* Erase optionbytes */
        FLASH->CTLR |= CR_OPTER_Set;
        FLASH->CTLR |= CR_STRT_Set;
        while(FLASH->STATR & SR_BSY)
            ;
        FLASH->CTLR &= ~CR_OPTER_Set;

        /* Write optionbytes */
        pbuf[((Address - 0x1FFFF800) / 2)] = ((((uint16_t) ~(Data)) << 8) | ((uint16_t)Data));

        FLASH->CTLR |= CR_OPTPG_Set;

        for(i = 0; i < 8; i++)
        {
            *(uint16_t *)(Addr + 2 * i) = pbuf[i];
#ifdef Core_V5F
            __asm("fence");
#endif
            while(FLASH->STATR & SR_BSY)
                ;
        }

        FLASH->CTLR &= ~CR_OPTPG_Set;
    }

    return status;
}

/*********************************************************************
 * @fn      FLASH_EnableWriteProtection
 *
 * @brief   Write protects the desired sectors
 *
 * @param   FLASH_Sectors - specifies the address of the pages to be write protected.
 *
 * @return  FLASH Status - The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
 *        FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status FLASH_EnableWriteProtection(uint32_t FLASH_Sectors)
{
    uint16_t     WRP0_Data = 0xFFFF, WRP1_Data = 0xFFFF, WRP2_Data = 0xFFFF, WRP3_Data = 0xFFFF;
    FLASH_Status status = FLASH_COMPLETE;
    uint32_t     Addr = 0x1FFFF800;
    __IO uint8_t i;
    uint16_t     pbuf[8];

    FLASH_Sectors = (uint32_t)(~FLASH_Sectors);
    WRP0_Data = (uint16_t)(FLASH_Sectors & WRP0_Mask);
    WRP1_Data = (uint16_t)((FLASH_Sectors & WRP1_Mask) >> 8);
    WRP2_Data = (uint16_t)((FLASH_Sectors & WRP2_Mask) >> 16);
    WRP3_Data = (uint16_t)((FLASH_Sectors & WRP3_Mask) >> 24);

    status = FLASH_WaitForLastOperation(ProgramTimeout);

    if(status == FLASH_COMPLETE)
    {
        FLASH->OBKEYR = FLASH_KEY1;
        FLASH->OBKEYR = FLASH_KEY2;

        /* Read optionbytes */
        for(i = 0; i < 8; i++)
        {
            pbuf[i] = *(uint16_t *)(Addr + 2 * i);
        }

        /* Erase optionbytes */
        FLASH->CTLR |= CR_OPTER_Set;
        FLASH->CTLR |= CR_STRT_Set;
        while(FLASH->STATR & SR_BSY)
            ;
        FLASH->CTLR &= ~CR_OPTER_Set;

        /* Write optionbytes */
        pbuf[4] = WRP0_Data;
        pbuf[5] = WRP1_Data;
        pbuf[6] = WRP2_Data;
        pbuf[7] = WRP3_Data;

        FLASH->CTLR |= CR_OPTPG_Set;
        for(i = 0; i < 8; i++)
        {
            *(uint16_t *)(Addr + 2 * i) = pbuf[i];
#ifdef Core_V5F
            __asm("fence");
#endif
            while(FLASH->STATR & SR_BSY)
                ;
        }
        FLASH->CTLR &= ~CR_OPTPG_Set;
    }
    return status;
}

/*********************************************************************
 * @fn      FLASH_ReadOutProtection
 *
 * @brief   Enables or disables the read out protection.
 *
 * @param   Newstate - new state of the ReadOut Protection(ENABLE or DISABLE).
 *
 * @return  FLASH Status - The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
 *        FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status FLASH_ReadOutProtection(FunctionalState NewState)
{
    FLASH_Status status = FLASH_COMPLETE;
    uint32_t     Addr = 0x1FFFF800;
    __IO uint8_t i;
    uint16_t     pbuf[8];

    status = FLASH_WaitForLastOperation(EraseTimeout);
    if(status == FLASH_COMPLETE)
    {
        FLASH->OBKEYR = FLASH_KEY1;
        FLASH->OBKEYR = FLASH_KEY2;

        /* Read optionbytes */
        for(i = 0; i < 8; i++)
        {
            pbuf[i] = *(uint16_t *)(Addr + 2 * i);
        }

        /* Erase optionbytes */
        FLASH->CTLR |= CR_OPTER_Set;
        FLASH->CTLR |= CR_STRT_Set;
        while(FLASH->STATR & SR_BSY)
            ;
        FLASH->CTLR &= ~CR_OPTER_Set;

        /* Write optionbytes */
        if(NewState == DISABLE)
            pbuf[0] = 0x5AA5;
        else
            pbuf[0] = 0x00FF;

        FLASH->CTLR |= CR_OPTPG_Set;
        for(i = 0; i < 8; i++)
        {
            *(uint16_t *)(Addr + 2 * i) = pbuf[i];
#ifdef Core_V5F
            __asm("fence");
#endif
            while(FLASH->STATR & SR_BSY)
                ;
        }
        FLASH->CTLR &= ~CR_OPTPG_Set;
    }
    return status;
}

/*********************************************************************
 * @fn      FLASH_UserOptionByteConfig
 *
 * @brief   Programs the FLASH User Option Byte - IWDG_SW.
 *
 * @param   OB_IWDG - Selects the IWDG mode
 *            OB_IWDG_SW - Software IWDG selected
 *            OB_IWDG_HW - Hardware IWDG selected
 *
 * @return  FLASH Status - The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
 *        FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status FLASH_UserOptionByteConfig(uint16_t OB_IWDG)
{
    FLASH_Status status = FLASH_COMPLETE;
    uint32_t     Addr = 0x1FFFF800;
    __IO uint8_t i;
    uint16_t     pbuf[8];
    uint16_t     temp;

    FLASH->OBKEYR = FLASH_KEY1;
    FLASH->OBKEYR = FLASH_KEY2;
    status = FLASH_WaitForLastOperation(ProgramTimeout);

    if(status == FLASH_COMPLETE)
    {
        /* Read optionbytes */
        for(i = 0; i < 8; i++)
        {
            pbuf[i] = *(uint16_t *)(Addr + 2 * i);
            // printf("pbuf[%d] - %04x\r\n",i,pbuf[i]);
        }

        temp=pbuf[1]&(~0x1);

        /* Erase optionbytes */
        FLASH->CTLR |= CR_OPTER_Set;
        FLASH->CTLR |= CR_STRT_Set;
        while(FLASH->STATR & SR_BSY)
            ;
        FLASH->CTLR &= ~CR_OPTER_Set;

        pbuf[1] = (uint16_t)(OB_IWDG | ((uint16_t)temp));

        FLASH->CTLR |= CR_OPTPG_Set;
        for(i = 0; i < 8; i++)
        {
            *(uint16_t *)(Addr + 2 * i) = pbuf[i];
#ifdef Core_V5F
            __asm("fence");
#endif
            while(FLASH->STATR & SR_BSY)
                ;
        }
        FLASH->CTLR &= ~CR_OPTPG_Set;
    }
    return status;
}

/*********************************************************************
 * @fn      FLASH_GetUserOptionByte
 *
 * @brief   Returns the FLASH User Option Bytes values.
 *
 * @return  The FLASH User Option Bytes values - IWDG_SW(Bit0)
 */
uint32_t FLASH_GetUserOptionByte(void)
{
    return (uint32_t)(FLASH->OBR >> 2);
}

/*********************************************************************
 * @fn      FLASH_GetWriteProtectionOptionByte
 *
 * @brief   Returns the FLASH Write Protection Option Bytes Register value.
 *
 * @return  The FLASH Write Protection Option Bytes Register value.
 */
uint32_t FLASH_GetWriteProtectionOptionByte(void)
{
    return (uint32_t)(FLASH->WPR);
}

/*********************************************************************
 * @fn      FLASH_GetReadOutProtectionStatus
 *
 * @brief   Checks whether the FLASH Read Out Protection Status is set or not.
 *
 * @return  FLASH ReadOut Protection Status(SET or RESET)
 */
FlagStatus FLASH_GetReadOutProtectionStatus(void)
{
    FlagStatus readoutstatus = RESET;
    if((FLASH->OBR & RDPRT_Mask) != (uint32_t)RESET)
    {
        readoutstatus = SET;
    }
    else
    {
        readoutstatus = RESET;
    }
    return readoutstatus;
}

/*********************************************************************
 * @fn      FLASH_ITConfig
 *
 * @brief   Enables or disables the specified FLASH interrupts.
 *
 * @param   FLASH_IT - specifies the FLASH interrupt sources to be enabled or disabled.
 *            FLASH_IT_ERROR - FLASH Error Interrupt
 *            FLASH_IT_EOP - FLASH end of operation Interrupt
 *          NewState - new state of the specified Flash interrupts(ENABLE or DISABLE).
 *
 * @return  FLASH Prefetch Buffer Status (SET or RESET).
 */
void FLASH_ITConfig(uint32_t FLASH_IT, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        FLASH->CTLR |= FLASH_IT;
    }
    else
    {
        FLASH->CTLR &= ~(uint32_t)FLASH_IT;
    }
}

/*********************************************************************
 * @fn      FLASH_GetFlagStatus
 *
 * @brief   Checks whether the specified FLASH flag is set or not.
 *
 * @param   FLASH_FLAG - specifies the FLASH flag to check.
 *            FLASH_FLAG_BSY - FLASH Busy flag
 *            FLASH_FLAG_WRPRTERR - FLASH Write protected error flag
 *            FLASH_FLAG_EOP - FLASH End of Operation flag
 *            FLASH_FLAG_OPTERR - FLASH Option Byte error flag
 *            FLASH_FLAG_ENHANCE -
 *            FLASH_FLAG_READY -
 *            FLASH_FLAG_LPMODE -
 *
 * @return  The new state of FLASH_FLAG (SET or RESET).
 */
FlagStatus FLASH_GetFlagStatus(uint32_t FLASH_FLAG)
{
    FlagStatus bitstatus = RESET;

    if((FLASH_FLAG & ((uint32_t)0xFFC0)) != 0)
    {
        if((FLASH->ACTLR & FLASH_FLAG) != (uint32_t)RESET)
        {
            bitstatus = SET;
        }
        else
        {
            bitstatus = RESET;
        }
    }
    else if(FLASH_FLAG == FLASH_FLAG_OPTERR)
    {
        if((FLASH->OBR & (1 << 0)) != (uint32_t)RESET)
        {
            bitstatus = SET;
        }
        else
        {
            bitstatus = RESET;
        }
    }
    else
    {
        if((FLASH->STATR & FLASH_FLAG) != (uint32_t)RESET)
        {
            bitstatus = SET;
        }
        else
        {
            bitstatus = RESET;
        }
    }
    return bitstatus;
}

/*********************************************************************
 * @fn      FLASH_ClearFlag
 *
 * @brief   Clears the FLASH's pending flags.
 *
 * @param   FLASH_FLAG - specifies the FLASH flags to clear.
 *            FLASH_FLAG_WRPRTERR - FLASH Write protected error flag
 *            FLASH_FLAG_EOP - FLASH End of Operation flag
 *
 * @return  none
 */
void FLASH_ClearFlag(uint32_t FLASH_FLAG)
{
    FLASH->STATR = FLASH_FLAG;
}

/*********************************************************************
 * @fn      FLASH_GetStatus
 *
 * @brief   Returns the FLASH Status.
 *
 * @return  FLASH Status - The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
 *        FLASH_ERROR_WRP or FLASH_COMPLETE.
 */
FLASH_Status FLASH_GetStatus(void)
{
    FLASH_Status flashstatus = FLASH_COMPLETE;
#ifdef Core_V5F
        __asm("fence");
#endif
    if((FLASH->STATR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY)
    {
        flashstatus = FLASH_BUSY;
    }
    else
    {
        if((FLASH->STATR & FLASH_FLAG_WRPRTERR) != 0)
        {
            flashstatus = FLASH_ERROR_WRP;
        }
        else
        {
            flashstatus = FLASH_COMPLETE;
        }
    }
    return flashstatus;
}

/*********************************************************************
 * @fn      FLASH_WaitForLastOperation
 *
 * @brief   Waits for a Flash operation to complete or a TIMEOUT to occur.
 *
 * @param   Timeout - FLASH programming Timeout
 *
 * @return  FLASH Status - The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
 *        FLASH_ERROR_WRP or FLASH_COMPLETE.
 */
FLASH_Status FLASH_WaitForLastOperation(uint32_t Timeout)
{
    FLASH_Status status = FLASH_COMPLETE;

    status = FLASH_GetStatus();

    while((status == FLASH_BUSY) && (Timeout != 0x00))
    {
        status = FLASH_GetStatus();
        Timeout--;
    }
    if(Timeout == 0x00)
    {
        status = FLASH_TIMEOUT;
    }
    return status;
}

/*********************************************************************
 * @fn      FLASH_Unlock_Fast
 *
 * @brief   Unlocks the Fast Program Erase Mode.
 *
 * @return  none
 */
void FLASH_Unlock_Fast(void)
{
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;

    FLASH->MODEKEYR = FLASH_KEY1;
    FLASH->MODEKEYR = FLASH_KEY2;
}

/*********************************************************************
 * @fn      FLASH_Lock_Fast
 *
 * @brief   Locks the Fast Program Erase Mode.
 *
 * @return  none
 */
void FLASH_Lock_Fast(void)
{
    FLASH->CTLR |= CR_FLOCK_Set;
}

/*********************************************************************
 * @fn      FLASH_EraseBlock_32K_Fast
 *
 * @brief   Erases a specified FLASH Block (1Block = 32KByte or 64KByte).
 *        If the flash is Dual flash mode,then block erase size is 64KB.
 *        If the flash is single flash mode,then block erase size is 32KB.
 * @param   Block_Address - The block address to be erased.
 *
 * @return  none
 */
void FLASH_EraseBlock_Fast(uint32_t Block_Address)
{
    if(((*(vu32*)FLASH_CFGR0_BASE) & (1<<28)) != 0)
    {
        Block_Address &= 0xFFFF0000;
    }
    else 
    {
        Block_Address &= 0xFFFF8000;
    }

    FLASH->CTLR |= CR_BER;
    FLASH->ADDR = Block_Address;
    FLASH->CTLR |= CR_STRT_Set;
    while(FLASH->STATR & SR_BSY)
        ;
    FLASH->CTLR &= ~CR_BER;
}

/*********************************************************************
 * @fn      FLASH_ProgramPage_Fast
 *
 * @brief   Program a specified FLASH page (1page = 256Byte).
 *
 * @param   Page_Address - The page address to be programed.
 *
 * @return  none
 */
void FLASH_ProgramPage_Fast(uint32_t Page_Address, uint32_t *pbuf)
{
    uint8_t size = 64;
    __disable_irq( );    //it would be important!

    Page_Address &= 0xFFFFFF00;

    FLASH->CTLR |= CR_PAGE_PG;
    while(FLASH->STATR & SR_BSY)
        ;
    while(FLASH->STATR & SR_WR_BSY)
        ;

    while(size)
    {
        *(uint32_t *)Page_Address = *(uint32_t *)pbuf;
        Page_Address += 4;
        pbuf += 1;
        size -= 1;
#ifdef Core_V5F
        __asm("fence");
#endif
        while(FLASH->STATR & SR_WR_BSY)
            ;
    }

    FLASH->CTLR |= CR_PG_STRT;
    while(FLASH->STATR & SR_BSY)
        ;
    FLASH->CTLR &= ~CR_PAGE_PG;
    
    __enable_irq( );
}

/*********************************************************************
 * @fn      FLASH_Enhance_Mode
 *
 * @brief   Read FLASH Enhance Mode
 *
 * @param   Newstate - new state of the ReadOut Protection(ENABLE or DISABLE).
 *
 * @return  none
 */
void FLASH_Enhance_Mode(FunctionalState NewState)
{
    if(NewState)
    {
        FLASH->ACTLR |= ( 1<< 7);
    }
    else
    {
        FLASH->ACTLR &= ~( 1<< 7);
        FLASH->CTLR |= (1 << 22);
    }
}

/*********************************************************************
 * @fn      FLASH_Access_Clock_Cfg
 *
 * @brief   Config FLASH Access Clock(Need to unlock )
 *
 * @param   FLASH_Access_CLK -
 *            FLASH_CLK_HCLKDIV1 - HCLK clock
 *            FLASH_CLK_HCLKDIV2 - HCLK clock/2
 *            FLASH_CLK_HCLKDIV4 - HCLK clock/4
 *            FLASH_CLK_HCLKDIV8 - HCLK clock/8
 *
 * @return  none
 */
void FLASH_Access_Clock_Cfg(uint32_t FLASH_Access_CLK)
{
    FLASH->ACTLR &= ~FLASH_ACTLR_SCK_CFG;
    FLASH->ACTLR |= FLASH_Access_CLK;
}

/*********************************************************************
 * @fn      FLASH_LP_Cmd
 *
 * @brief   Enables or disables the FLASH enter low power mode.
 *
 * @param   NewState - new state of the FLASH enter low power mode.
 *        (ENABLE or DISABLE).
 *
 * @return  none
 */
void FLASH_LP_Cmd(FunctionalState NewState)
{
    if(NewState)
    {
        FLASH->ACTLR |= FLASH_ACTLR_LP;
    }
    else
    {
        FLASH->ACTLR &= ~FLASH_ACTLR_LP;
    }
}

/*********************************************************************
 * @fn      SystemReset_StartMode
 *
 * @brief   Start mode after system reset.
 *
 * @param   Mode - Start mode.
 *            Start_Mode_USER - USER start after system reset
 *            Start_Mode_BOOT - Boot start after system reset
 *
 * @return  none
 */
void SystemReset_StartMode(uint32_t Mode)
{
    FLASH_Unlock();

    FLASH->BOOT_MODEKEYR = FLASH_KEY1;
    FLASH->BOOT_MODEKEYR = FLASH_KEY2;

    FLASH->STATR &= ~FLASH_STATR_BOOT_MODE;
    if(Mode == Start_Mode_BOOT)
    {
        FLASH->STATR |= FLASH_STATR_BOOT_MODE;
    }

    FLASH_Lock();
}

/*********************************************************************
 * @fn      ROM_ERASE
 *
 * @brief   Select erases a specified FLASH .
 *
 * @param   StartAddr - Erases Flash start address.
 *        If the flash is Dual flash mode - (StartAddr%(8KB) == 0).
 *        If the flash is single flash mode - (StartAddr%(4KB) == 0).
 *          Cnt - Erases count.
 *          Erase_Size - Erases size select.The returned value can be:
 *          Size_64KB, Size_32KB, Size_8KB,Size_4KB
 *
 * @return  none.
 */
static void ROM_ERASE(uint32_t StartAddr, uint32_t Cnt, uint32_t Erase_Size)
{
    do{
        if(Erase_Size == Size_64KB)
        {
            FLASH->CTLR |= CR_BER;
        }
        else if(Erase_Size == Size_32KB)
        {
            FLASH->CTLR |= CR_BER;
        }
        else if(Erase_Size == Size_8KB)
        {
            FLASH->CTLR |= CR_PER_Set;
        }
        else if(Erase_Size == Size_4KB)
        {
            FLASH->CTLR |= CR_PER_Set;
        }

        FLASH->ADDR = StartAddr;
        FLASH->CTLR |= CR_STRT_Set;
        while(FLASH->STATR & SR_BSY)
            ;

        if(Erase_Size == Size_64KB)
        {
            FLASH->CTLR &= ~CR_BER;
            StartAddr += Size_64KB;
        }
        else if(Erase_Size == Size_32KB)
        {
            FLASH->CTLR &= ~CR_BER;
            StartAddr += Size_32KB;
        }
        else if(Erase_Size == Size_8KB)
        {
            FLASH->CTLR &= ~CR_PER_Set;
            StartAddr += Size_8KB;
        }
        else if(Erase_Size == Size_4KB)
        {
            FLASH->CTLR &= ~CR_PER_Set;
            StartAddr += Size_4KB;
        }

    }while(--Cnt);
}

/*********************************************************************
 * @fn      FLASH_ROM_ERASE
 *
 * @brief   Erases a specified FLASH .
 *
 * @param   StartAddr - Erases Flash start address.
 *          Length - Erases Flash start Length.
 *        If the flash is Dual flash mode - (StartAddr%(8KB) == 0).
 *        If the flash is single flash mode - (StartAddr%(4KB) == 0).
 *
 * @return  FLASH Status - The returned value can be: FLASH_ADR_RANGE_ERROR,
 *        FLASH_ALIGN_ERROR, FLASH_OP_RANGE_ERROR or FLASH_COMPLETE.
 */
FLASH_Status FLASH_ROM_ERASE(uint32_t StartAddr, uint32_t Length)
{
    uint32_t Addr0 = 0, Addr1 = 0, Length0 = 0, Length1 = 0;

    FLASH_Status status = FLASH_COMPLETE;
    if(((*(vu32*)FLASH_CFGR0_BASE) & (1<<28)) != 0)
    {
        if((StartAddr < ValidAddrStart) || (StartAddr >= ValidAddrEnd_Dual))
        {
            return FLASH_ADR_RANGE_ERROR;
        }

        if((StartAddr + Length) > ValidAddrEnd_Dual)
        {
            return FLASH_OP_RANGE_ERROR;
        }

        if((StartAddr & (Size_8KB-1)) || (Length & (Size_8KB-1)) || (Length == 0))
        {
            return FLASH_ALIGN_ERROR;
        }

        /* Authorize the FPEC of Bank1 Access */
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;

        /* Fast mode unlock */
        FLASH->MODEKEYR = FLASH_KEY1;
        FLASH->MODEKEYR = FLASH_KEY2;

        Addr0 = StartAddr;

        if(Length >= Size_64KB)
        {
            Length0 = Size_64KB - (Addr0 & (Size_64KB - 1));
            Addr1 = StartAddr + Length0;
            Length1 = Length - Length0;
        }
        else if(Length >= Size_8KB)
        {
            Length0 = Length;
        }


        /* Erase 64KB */
        if(Length0 >= Size_64KB)//front
        {
            Length = Length0;
            if(Addr0 & (Size_64KB - 1))
            {
                Length0 = Size_64KB - (Addr0 & (Size_64KB - 1));
            }
            else
            {
                Length0 = 0;
            }

            ROM_ERASE((Addr0 + Length0), ((Length - Length0) >> 16), Size_64KB);
        }

        if(Length1 >= Size_64KB)//back
        {
            StartAddr = Addr1;
            Length = Length1;

            if((Addr1 + Length1) & (Size_64KB - 1))
            {
                Addr1 = ((StartAddr + Length1) & (~(Size_64KB - 1)));
                Length1 = (StartAddr + Length1) & (Size_64KB - 1);
            }
            else
            {
                Length1 = 0;
            }

            ROM_ERASE(StartAddr, ((Length - Length1) >> 16), Size_64KB);
        }

        /* Erase 8KB */
        if(Length0)//front
        {
            ROM_ERASE(Addr0, (Length0 >> 13), Size_8KB);
        }

        if(Length1)//back
        {
            ROM_ERASE(Addr1, (Length1 >> 13), Size_8KB);
        }
    }
    else 
    {
        if((StartAddr < ValidAddrStart) || (StartAddr >= ValidAddrEnd_Signal))
        {
            return FLASH_ADR_RANGE_ERROR;
        }

        if((StartAddr + Length) > ValidAddrEnd_Signal)
        {
            return FLASH_OP_RANGE_ERROR;
        }

        if((StartAddr & (Size_4KB-1)) || (Length & (Size_4KB-1)) || (Length == 0))
        {
            return FLASH_ALIGN_ERROR;
        }

        /* Authorize the FPEC of Bank1 Access */
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;

        /* Fast mode unlock */
        FLASH->MODEKEYR = FLASH_KEY1;
        FLASH->MODEKEYR = FLASH_KEY2;

        Addr0 = StartAddr;

        if(Length >= Size_32KB)
        {
            Length0 = Size_32KB - (Addr0 & (Size_32KB - 1));
            Addr1 = StartAddr + Length0;
            Length1 = Length - Length0;
        }
        else if(Length >= Size_4KB)
        {
            Length0 = Length;
        }


        /* Erase 32KB */
        if(Length0 >= Size_32KB)//front
        {
            Length = Length0;
            if(Addr0 & (Size_32KB - 1))
            {
                Length0 = Size_32KB - (Addr0 & (Size_32KB - 1));
            }
            else
            {
                Length0 = 0;
            }

            ROM_ERASE((Addr0 + Length0), ((Length - Length0) >> 15), Size_32KB);
        }

        if(Length1 >= Size_32KB)//back
        {
            StartAddr = Addr1;
            Length = Length1;

            if((Addr1 + Length1) & (Size_32KB - 1))
            {
                Addr1 = ((StartAddr + Length1) & (~(Size_32KB - 1)));
                Length1 = (StartAddr + Length1) & (Size_32KB - 1);
            }
            else
            {
                Length1 = 0;
            }

            ROM_ERASE(StartAddr, ((Length - Length1) >> 15), Size_32KB);
        }

        /* Erase 4KB */
        if(Length0)//front
        {
            ROM_ERASE(Addr0, (Length0 >> 12), Size_4KB);
        }

        if(Length1)//back
        {
            ROM_ERASE(Addr1, (Length1 >> 1), Size_4KB);
        }
    }

    FLASH->CTLR |= CR_FLOCK_Set;
    FLASH->CTLR |= CR_LOCK_Set;

    return status;
}

/*********************************************************************
 * @fn      FLASH_ROM_WRITE
 *
 * @brief   Writes a specified FLASH .
 *
 * @param   StartAddr - Writes Flash start address(StartAddr%256 == 0).
 *          Length - Writes Flash start Length(Length%256 == 0).
 *
 *          pbuf - Writes Flash value buffer.
 *
 * @return  FLASH Status - The returned value can be: FLASH_ADR_RANGE_ERROR,
 *        FLASH_ALIGN_ERROR, FLASH_OP_RANGE_ERROR or FLASH_COMPLETE.
 */
FLASH_Status FLASH_ROM_WRITE(uint32_t StartAddr, uint32_t *pbuf, uint32_t Length)
{
    uint32_t i;
    uint8_t size;

    FLASH_Status status = FLASH_COMPLETE;

    if(((*(vu32*)FLASH_CFGR0_BASE) & (1<<28)) != 0)
    {
        if((StartAddr < ValidAddrStart) || (StartAddr >= ValidAddrEnd_Dual))
        {
        return FLASH_ADR_RANGE_ERROR;
        }

        if((StartAddr + Length) > ValidAddrEnd_Dual)
        {
        return FLASH_OP_RANGE_ERROR;
        }
    }
    else 
    {
        if((StartAddr < ValidAddrStart) || (StartAddr >= ValidAddrEnd_Signal))
        {
        return FLASH_ADR_RANGE_ERROR;
        }

        if((StartAddr + Length) > ValidAddrEnd_Signal)
        {
        return FLASH_OP_RANGE_ERROR;
        }
    }

    if((StartAddr & (Size_256B-1)) || (Length & (Size_256B-1)) || (Length == 0))
    {
        return FLASH_ALIGN_ERROR;
    }

    i = Length >> 8;

    /* Authorize the FPEC of Bank1 Access */
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;

    /* Fast program mode unlock */
    FLASH->MODEKEYR = FLASH_KEY1;
    FLASH->MODEKEYR = FLASH_KEY2;

    do{
        FLASH->CTLR |= CR_PAGE_PG;
        while(FLASH->STATR & SR_BSY)
            ;
        while(FLASH->STATR & SR_WR_BSY)
            ;
        size = 64;
        while(size)
        {
            *(uint32_t *)StartAddr = *(uint32_t *)pbuf;
            StartAddr += 4;
            pbuf += 1;
            size -= 1;
#ifdef Core_V5F
            __asm("fence");
#endif
            while(FLASH->STATR & SR_WR_BSY)
                ;
        }

        FLASH->CTLR |= CR_PG_STRT;
        while(FLASH->STATR & SR_BSY)
            ;
        FLASH->CTLR &= ~CR_PAGE_PG;
    }while(--i);

    FLASH->CTLR |= CR_FLOCK_Set;
    FLASH->CTLR |= CR_LOCK_Set;

    return status;
}

/*********************************************************************
 * @fn      FLASH_BOOT_GetMode
 *
 * @brief   Returns the BOOT mode.
 *
 * @return BOOT mode identifier.
 *          BOOT mode List-
 *  0x00FFFFFF - UART-enable USBHS-enable USBSS-enable
 *  0x06FFFFF9 - UART-enable USBHS-Disable USBSS-disable
 *  0x05FFFFFA - UART-Disable USBHS-enable USBSS-disable
 *  0x03FFFFFC - UART-Disable USBHS-disable USBSS-enable
 *  0x04FFFFFB - UART-enable USBHS-enable USBSS-disable
 *  0x01FFFFFE - UART-disable USBHS-enable USBSS-enable
 *  0x02FFFFFD - UART-enable USBHS-Disable USBSS-enable 
 */
__attribute__((optimize("O0"))) uint32_t FLASH_BOOT_GetMode( void )
{
    return( *( uint32_t * )0x08000018 );
}

