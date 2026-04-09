/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_flash.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file contains all the functions prototypes for the FLASH  
*                      firmware library.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32H417_FLASH_H
#define __CH32H417_FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32h417.h"

/* FLASH Status */
typedef enum
{ 
  FLASH_BUSY = 1,
  FLASH_ERROR_PG,
  FLASH_ERROR_WRP,
  FLASH_COMPLETE,
  FLASH_TIMEOUT,
  FLASH_OP_RANGE_ERROR = 0xFD,
  FLASH_ALIGN_ERROR = 0xFE,
  FLASH_ADR_RANGE_ERROR = 0xFF,
}FLASH_Status;

/* Write Protect (dual flash mode - 8K bytes/sector) (single flash mode - 4K bytes/sector)*/
#define FLASH_WRProt_Sectors0          ((uint32_t)0x00000001) /* Write protection of setor 0 */
#define FLASH_WRProt_Sectors1          ((uint32_t)0x00000002) /* Write protection of setor 1 */
#define FLASH_WRProt_Sectors2          ((uint32_t)0x00000004) /* Write protection of setor 2 */
#define FLASH_WRProt_Sectors3          ((uint32_t)0x00000008) /* Write protection of setor 3 */
#define FLASH_WRProt_Sectors4          ((uint32_t)0x00000010) /* Write protection of setor 4 */
#define FLASH_WRProt_Sectors5          ((uint32_t)0x00000020) /* Write protection of setor 5 */
#define FLASH_WRProt_Sectors6          ((uint32_t)0x00000040) /* Write protection of setor 6 */
#define FLASH_WRProt_Sectors7          ((uint32_t)0x00000080) /* Write protection of setor 7 */
#define FLASH_WRProt_Sectors8          ((uint32_t)0x00000100) /* Write protection of setor 8 */
#define FLASH_WRProt_Sectors9          ((uint32_t)0x00000200) /* Write protection of setor 9 */
#define FLASH_WRProt_Sectors10         ((uint32_t)0x00000400) /* Write protection of setor 10 */
#define FLASH_WRProt_Sectors11         ((uint32_t)0x00000800) /* Write protection of setor 11 */
#define FLASH_WRProt_Sectors12         ((uint32_t)0x00001000) /* Write protection of setor 12 */
#define FLASH_WRProt_Sectors13         ((uint32_t)0x00002000) /* Write protection of setor 13 */
#define FLASH_WRProt_Sectors14         ((uint32_t)0x00004000) /* Write protection of setor 14 */
#define FLASH_WRProt_Sectors15         ((uint32_t)0x00008000) /* Write protection of setor 15 */
#define FLASH_WRProt_Sectors16         ((uint32_t)0x00010000) /* Write protection of setor 16 */
#define FLASH_WRProt_Sectors17         ((uint32_t)0x00020000) /* Write protection of setor 17 */
#define FLASH_WRProt_Sectors18         ((uint32_t)0x00040000) /* Write protection of setor 18 */
#define FLASH_WRProt_Sectors19         ((uint32_t)0x00080000) /* Write protection of setor 19 */
#define FLASH_WRProt_Sectors20         ((uint32_t)0x00100000) /* Write protection of setor 20 */
#define FLASH_WRProt_Sectors21         ((uint32_t)0x00200000) /* Write protection of setor 21 */
#define FLASH_WRProt_Sectors22         ((uint32_t)0x00400000) /* Write protection of setor 22 */
#define FLASH_WRProt_Sectors23         ((uint32_t)0x00800000) /* Write protection of setor 23 */
#define FLASH_WRProt_Sectors24         ((uint32_t)0x01000000) /* Write protection of setor 24 */
#define FLASH_WRProt_Sectors25         ((uint32_t)0x02000000) /* Write protection of setor 25 */
#define FLASH_WRProt_Sectors26         ((uint32_t)0x04000000) /* Write protection of setor 26 */
#define FLASH_WRProt_Sectors27         ((uint32_t)0x08000000) /* Write protection of setor 27 */
#define FLASH_WRProt_Sectors28         ((uint32_t)0x10000000) /* Write protection of setor 28 */
#define FLASH_WRProt_Sectors29         ((uint32_t)0x20000000) /* Write protection of setor 29 */
#define FLASH_WRProt_Sectors30         ((uint32_t)0x40000000) /* Write protection of setor 30 */
#define FLASH_WRProt_Sectors31to119    ((uint32_t)0x80000000) /* Write protection of setor 31 to 119*/

#define FLASH_WRProt_AllSectors        ((uint32_t)0xFFFFFFFF) /* Write protection of all Sectors */

/* Option_Bytes_IWatchdog */
#define OB_IWDG_SW                     ((uint16_t)0x0001)  /* Software IWDG selected */
#define OB_IWDG_HW                     ((uint16_t)0x0000)  /* Hardware IWDG selected */

/* FLASH_Interrupts */	
#define FLASH_IT_ERROR                 ((uint32_t)0x00000400)  
#define FLASH_IT_EOP                   ((uint32_t)0x00001000)  

/* FLASH_Flags */	
#define FLASH_FLAG_BSY                 ((uint32_t)0x00000001)  
#define FLASH_FLAG_EOP                 ((uint32_t)0x00000020)  
#define FLASH_FLAG_WRPRTERR            ((uint32_t)0x00000010)  
#define FLASH_FLAG_OPTERR              ((uint32_t)0x80000001)  
#define FLASH_FLAG_ENHANCE             ((uint32_t)0x00000040)
#define FLASH_FLAG_READY               ((uint32_t)0x00004000)
#define FLASH_FLAG_LPMODE              ((uint32_t)0x00008000)

/* FLASH_Access_CLK */
#define FLASH_CLK_HCLKDIV1             ((uint32_t)0x00000000)
#define FLASH_CLK_HCLKDIV2             ((uint32_t)0x00000001)
#define FLASH_CLK_HCLKDIV4             ((uint32_t)0x00000002)
#define FLASH_CLK_HCLKDIV8             ((uint32_t)0x00000003)

/* System_Reset_Start_Mode */
#define Start_Mode_USER                ((uint32_t)0x00000000)
#define Start_Mode_BOOT                ((uint32_t)0x00004000)

/*Functions used for all devices*/
void FLASH_Unlock(void);
void FLASH_Lock(void);
FLASH_Status FLASH_ErasePage(uint32_t Page_Address);
FLASH_Status FLASH_EraseOptionBytes(void);
FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);
FLASH_Status FLASH_ProgramOptionByteData(uint32_t Address, uint8_t Data);
FLASH_Status FLASH_EnableWriteProtection(uint32_t FLASH_Sectors);
FLASH_Status FLASH_ReadOutProtection(FunctionalState NewState);
FLASH_Status FLASH_UserOptionByteConfig(uint16_t OB_IWDG);
uint32_t FLASH_GetUserOptionByte(void);
uint32_t FLASH_GetWriteProtectionOptionByte(void);
FlagStatus FLASH_GetReadOutProtectionStatus(void);
void FLASH_ITConfig(uint32_t FLASH_IT, FunctionalState NewState);
FlagStatus FLASH_GetFlagStatus(uint32_t FLASH_FLAG);
void FLASH_ClearFlag(uint32_t FLASH_FLAG);
FLASH_Status FLASH_GetStatus(void);
FLASH_Status FLASH_WaitForLastOperation(uint32_t Timeout);
void FLASH_Unlock_Fast(void);
void FLASH_Lock_Fast(void);
void FLASH_EraseBlock_Fast(uint32_t Block_Address);
void FLASH_ProgramPage_Fast(uint32_t Page_Address, uint32_t* pbuf);
void FLASH_Enhance_Mode(FunctionalState NewState);
void FLASH_Access_Clock_Cfg(uint32_t FLASH_Access_CLK);
void FLASH_LP_Cmd(FunctionalState NewState);
void SystemReset_StartMode(uint32_t Mode);
FLASH_Status FLASH_ROM_ERASE(uint32_t StartAddr, uint32_t Length);
FLASH_Status FLASH_ROM_WRITE(uint32_t StartAddr, uint32_t *pbuf, uint32_t Length);
uint32_t FLASH_BOOT_GetMode( void );

#ifdef __cplusplus
}
#endif


#endif 

