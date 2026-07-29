/**
 ******************************************************************************
 * @file     um324xx_hal_flash.h
 * @author   MCU Team
 * @version  V1.00
 * @date     2023-04-11
 * @brief
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2017-2023. Unicmicro Co.,Ltd.
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UM324XX_HAL_FLASH_H_
#define __UM324XX_HAL_FLASH_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"

/** @addtogroup UM324xx_HAL_Driver
 * @{
 */

/** @addtogroup FLASH
 * @{
 */

/* Exported typedefs ---------------------------------------------------------*/
/** @defgroup FLASH_Exported_typedefs xxx Exported Typedefs
 * @{
 */
/**
 * @brief  FLASH Procedure structure definition
 */
typedef enum {
    FLASH_PROC_NONE = 0U,
    FLASH_PROC_ERASE,
    FLASH_PROC_PROGRAM
} FLASH_ProcedureTypeDef;

/**
 * @brief  FLASH handle Structure definition
 */
typedef struct
{
    __IO FLASH_ProcedureTypeDef ProcedureOnGoing; /*Internal variable to indicate which procedure is ongoing*/

    __IO uint32_t NbPagesToErase; /*Internal variable to save the remaining pages to erase*/

    __IO uint32_t Page; /*Internal variable to define the current page which is erasing*/

    __IO uint32_t Address; /*Internal variable to save address selected for program*/

    HAL_LockTypeDef Lock; /* FLASH locking object */

    __IO uint32_t ErrorCode; /* FLASH error code */

} FLASH_ProcessTypeDef;

/**
 * @brief  FLASH Erase structure definition
 */
typedef struct
{
    uint32_t TypeErase; /*!< Mass erase or sector Erase.
                             This parameter can be a value of @ref FLASHEx_Type_Erase */

    uint32_t Page_Address; /*!< Initial FLASH page to erase when Mass erase is disabled
                                This parameter must be a value of @ref FLASHEx_Pages */

    uint32_t NbPages_Address; /*!< Number of Pages to be erased.
                                   This parameter must be a value between 1 and (max number of sectors - value of Initial sector)*/

} FLASH_EraseInitTypeDef;

/**
 * @}
 */

/* Exported constants --------------------------------------------------------*/
/** @defgroup FLASH_Exported_constants FLASH Exported Constants
 * @{
 */
#if defined(UM32x42x) || defined(UM32x41x)
#define FLASH_PAGE_SIZE 4096 /* Page size */
#endif
 #if defined(UM324xF)
#define FLASH_PAGE_SIZE 8192  /* Page size */
#endif

#if defined(UM324xH)
#define FLASH_PAGE_SIZE 4096 /* Page size */
#endif
/** @defgroup FLASHEx_Pages Address FLASH Pages Address
 * @note Please select the page size according to the actual flash size.
 * @{
 */
#define FLASH_PAGE_0_ADDR  0x00000000U            /*!< Page Number 0 addess */
#define FLASH_PAGE_1_ADDR  (FLASH_PAGE_SIZE * 1)  /*!< Page Number 1 addess */
#define FLASH_PAGE_2_ADDR  (FLASH_PAGE_SIZE * 2)  /*!< Page Number 2 addess */
#define FLASH_PAGE_3_ADDR  (FLASH_PAGE_SIZE * 3)  /*!< Page Number 3 addess */
#define FLASH_PAGE_4_ADDR  (FLASH_PAGE_SIZE * 4)  /*!< Page Number 4 addess */
#define FLASH_PAGE_5_ADDR  (FLASH_PAGE_SIZE * 5)  /*!< Page Number 5 addess */
#define FLASH_PAGE_6_ADDR  (FLASH_PAGE_SIZE * 6)  /*!< Page Number 6 addess */
#define FLASH_PAGE_7_ADDR  (FLASH_PAGE_SIZE * 7)  /*!< Page Number 7 addess */
#define FLASH_PAGE_8_ADDR  (FLASH_PAGE_SIZE * 8)  /*!< Page Number 8 addess */
#define FLASH_PAGE_9_ADDR  (FLASH_PAGE_SIZE * 9)  /*!< Page Number 9 addess */
#define FLASH_PAGE_10_ADDR (FLASH_PAGE_SIZE * 10) /*!< Page Number 10 addess */
#define FLASH_PAGE_11_ADDR (FLASH_PAGE_SIZE * 11) /*!< Page Number 11 addess */
#define FLASH_PAGE_12_ADDR (FLASH_PAGE_SIZE * 12) /*!< Page Number 12 addess */
#define FLASH_PAGE_13_ADDR (FLASH_PAGE_SIZE * 13) /*!< Page Number 13 addess */
#define FLASH_PAGE_14_ADDR (FLASH_PAGE_SIZE * 14) /*!< Page Number 14 addess */
#define FLASH_PAGE_15_ADDR (FLASH_PAGE_SIZE * 15) /*!< Page Number 15 addess */
#define FLASH_PAGE_16_ADDR (FLASH_PAGE_SIZE * 16) /*!< Page Number 16 addess */
#define FLASH_PAGE_17_ADDR (FLASH_PAGE_SIZE * 17) /*!< Page Number 17 addess */
#define FLASH_PAGE_18_ADDR (FLASH_PAGE_SIZE * 18) /*!< Page Number 18 addess */
#define FLASH_PAGE_19_ADDR (FLASH_PAGE_SIZE * 19) /*!< Page Number 19 addess */
#define FLASH_PAGE_20_ADDR (FLASH_PAGE_SIZE * 20) /*!< Page Number 20 addess */
#define FLASH_PAGE_21_ADDR (FLASH_PAGE_SIZE * 21) /*!< Page Number 21 addess */
#define FLASH_PAGE_22_ADDR (FLASH_PAGE_SIZE * 22) /*!< Page Number 22 addess */
#define FLASH_PAGE_23_ADDR (FLASH_PAGE_SIZE * 23) /*!< Page Number 23 addess */
#define FLASH_PAGE_24_ADDR (FLASH_PAGE_SIZE * 24) /*!< Page Number 24 addess */
#define FLASH_PAGE_25_ADDR (FLASH_PAGE_SIZE * 25) /*!< Page Number 25 addess */
#define FLASH_PAGE_26_ADDR (FLASH_PAGE_SIZE * 26) /*!< Page Number 26 addess */
#define FLASH_PAGE_27_ADDR (FLASH_PAGE_SIZE * 27) /*!< Page Number 27 addess */
#define FLASH_PAGE_28_ADDR (FLASH_PAGE_SIZE * 28) /*!< Page Number 28 addess */
#define FLASH_PAGE_29_ADDR (FLASH_PAGE_SIZE * 29) /*!< Page Number 29 addess */
#define FLASH_PAGE_30_ADDR (FLASH_PAGE_SIZE * 30) /*!< Page Number 30 addess */
#define FLASH_PAGE_31_ADDR (FLASH_PAGE_SIZE * 31) /*!< Page Number 31 addess */
#define FLASH_PAGE_32_ADDR (FLASH_PAGE_SIZE * 32) /*!< Page Number 32 addess */
#define FLASH_PAGE_33_ADDR (FLASH_PAGE_SIZE * 33) /*!< Page Number 33 addess */
#define FLASH_PAGE_34_ADDR (FLASH_PAGE_SIZE * 34) /*!< Page Number 34 addess */
#define FLASH_PAGE_35_ADDR (FLASH_PAGE_SIZE * 35) /*!< Page Number 35 addess */
#define FLASH_PAGE_36_ADDR (FLASH_PAGE_SIZE * 36) /*!< Page Number 36 addess */
#define FLASH_PAGE_37_ADDR (FLASH_PAGE_SIZE * 37) /*!< Page Number 37 addess */
#define FLASH_PAGE_38_ADDR (FLASH_PAGE_SIZE * 38) /*!< Page Number 38 addess */
#define FLASH_PAGE_39_ADDR (FLASH_PAGE_SIZE * 39) /*!< Page Number 39 addess */
#define FLASH_PAGE_40_ADDR (FLASH_PAGE_SIZE * 40) /*!< Page Number 40 addess */
#define FLASH_PAGE_41_ADDR (FLASH_PAGE_SIZE * 41) /*!< Page Number 41 addess */
#define FLASH_PAGE_42_ADDR (FLASH_PAGE_SIZE * 42) /*!< Page Number 42 addess */
#define FLASH_PAGE_43_ADDR (FLASH_PAGE_SIZE * 43) /*!< Page Number 43 addess */
#define FLASH_PAGE_44_ADDR (FLASH_PAGE_SIZE * 44) /*!< Page Number 44 addess */
#define FLASH_PAGE_45_ADDR (FLASH_PAGE_SIZE * 45) /*!< Page Number 45 addess */
#define FLASH_PAGE_46_ADDR (FLASH_PAGE_SIZE * 46) /*!< Page Number 46 addess */
#define FLASH_PAGE_47_ADDR (FLASH_PAGE_SIZE * 47) /*!< Page Number 47 addess */
#define FLASH_PAGE_48_ADDR (FLASH_PAGE_SIZE * 48) /*!< Page Number 48 addess */
#define FLASH_PAGE_49_ADDR (FLASH_PAGE_SIZE * 49) /*!< Page Number 49 addess */
#define FLASH_PAGE_50_ADDR (FLASH_PAGE_SIZE * 50) /*!< Page Number 50 addess */
#define FLASH_PAGE_51_ADDR (FLASH_PAGE_SIZE * 51) /*!< Page Number 51 addess */
#define FLASH_PAGE_52_ADDR (FLASH_PAGE_SIZE * 52) /*!< Page Number 52 addess */
#define FLASH_PAGE_53_ADDR (FLASH_PAGE_SIZE * 53) /*!< Page Number 53 addess */
#define FLASH_PAGE_54_ADDR (FLASH_PAGE_SIZE * 54) /*!< Page Number 54 addess */
#define FLASH_PAGE_55_ADDR (FLASH_PAGE_SIZE * 55) /*!< Page Number 55 addess */
#define FLASH_PAGE_56_ADDR (FLASH_PAGE_SIZE * 56) /*!< Page Number 56 addess */
#define FLASH_PAGE_57_ADDR (FLASH_PAGE_SIZE * 57) /*!< Page Number 57 addess */
#define FLASH_PAGE_58_ADDR (FLASH_PAGE_SIZE * 58) /*!< Page Number 58 addess */
#define FLASH_PAGE_59_ADDR (FLASH_PAGE_SIZE * 59) /*!< Page Number 59 addess */
#define FLASH_PAGE_60_ADDR (FLASH_PAGE_SIZE * 60) /*!< Page Number 60 addess */
#define FLASH_PAGE_61_ADDR (FLASH_PAGE_SIZE * 61) /*!< Page Number 61 addess */
#define FLASH_PAGE_62_ADDR (FLASH_PAGE_SIZE * 62) /*!< Page Number 62 addess */
#define FLASH_PAGE_63_ADDR (FLASH_PAGE_SIZE * 63) /*!< Page Number 63 addess */
/**
 * @}
 */

/** @defgroup FLASHEx_Type_Erase FLASH Type Erase
 * @{
 */
#define FLASH_TYPEERASE_PAGES     0x00000000U /*!< Pages erase only          */
#define FLASH_TYPEERASE_MASSERASE 0x00000001U /*!< Flash Mass erase activation */
/**
 * @}
 */

/** @defgroup FLASH_Error_Code FLASH Error Code
 * @brief    FLASH Error Code
 * @{
 */
#define HAL_FLASH_ERROR_NONE 0x00000000U /*!< No error */
#define HAL_FLASH_ERROR_PVFS 0x00000004U /*!< FLASH Write check error */
#define HAL_FLASH_ERROR_EVFS 0x00000008U /*!< FLASH Erasure check error */

/**
 * @}
 */

/** @defgroup FLASH_Type_Program FLASH Type Program
 * @{
 */
#define FLASH_TYPEPROGRAM_BYTE     0x00000000U /*!< Program byte (8-bit) at a specified address           */
#define FLASH_TYPEPROGRAM_HALFWORD 0x00000001U /*!< Program a half-word (16-bit) at a specified address   */
#define FLASH_TYPEPROGRAM_WORD     0x00000002U /*!< Program a word (32-bit) at a specified address        */
/**
 * @}
 */

/** @defgroup FLASH_Flag_definition FLASH Flag definition
 * @brief Flag definition
 * @{
 */
#define FLASH_FLAG_RDYS       EFC_STATUS_RDYS       /*!< FLASH End of Operation flag */
#define FLASH_FLAG_CONPROGRDY EFC_STATUS_CONPROGRDY /*!< FLASH is in the state of waiting for the next continuous programming. */
#define FLASH_FLAG_VDDS       EFC_STATUS_VDDS       /*!< FLASH Low voltage warning flag   */
#define FLASH_FLAG_LPSPD      EFC_STATUS_LPS        /*!< FLASH Power down mode flag  */
#define FLASH_FLAG_LPSLEEP    EFC_STATUS_LPS_1      /*!< FLASH Sleeping mode flag   */
#define FLASH_FLAG_LPSLVDD    EFC_STATUS_LPS_0      /*!< FLASH Low voltage operation flag  */

#define FLASH_FLAG_OPDS  EFC_INTSTATUS_OPDS  /*!< FLASH operation or erase successfully flag  */
#define FLASH_FLAG_VDDLS EFC_INTSTATUS_VDDLS /*!< VDD voltage low interrupt status bit flag  */
#define FLASH_FLAG_PVFS  EFC_INTSTATUS_PVFS  /*!< FLASH Write check error flag  */
#define FLASH_FLAG_EVFS  EFC_INTSTATUS_EVFS  /*!< FLASH Erasure check error flag  */

#define FLASH_FLAG_SHAICV  ((uint32_t)0x00000800U) /*!< SHA_ICV successfully read from otp area flag. */
#define FLASH_FLAG_AESKEY1 ((uint32_t)0x00000100U) /*!< AES_KEY1 successfully read from otp area flag. */
#define FLASH_FLAG_AESKEY2 ((uint32_t)0x00000200U) /*!< AES_KEY2 successfully read from otp area flag. */

/**
 * @}
 */

/** @defgroup FLASH_Interrupt_definition FLASH Interrupt definition
 * @brief FLASH Interrupt definition
 * @{
 */
#define FLASH_IT_OPDE  EFC_INTEN_OPDE                                                       /*!< End of FLASH Operation Interrupt source */
#define FLASH_IT_VDDLE EFC_INTEN_VDDLE                                                      /*!< VDD is low Interrupt source */
#define FLASH_IT_PVFE  EFC_INTEN_PVFE                                                       /*!< Write check error Interrupt source */
#define FLASH_IT_EVFE  EFC_INTEN_EVFE                                                       /*!< Erasure check Interrupt source */
#define FLASH_IT_ALL   (EFC_INTEN_OPDE | EFC_INTEN_VDDLE | EFC_INTEN_PVFE | EFC_INTEN_EVFE) /*!< Enable all the Interrupt source */

/**
 * @}
 */

/** @defgroup CAHCE state register definition
 * @brief CAHCE state register definition definition
 * @{
 */
#define CACHE_STATUS_CLOSED_FLAG   CACHE_STATUS_CACHE_STATUS_Pos
#define CACHE_STATUS_OPENING_FLAG  CACHE_STATUS_CACHE_STATUS_0
#define CACHE_STATUS_OPEND_FLAG    CACHE_STATUS_CACHE_STATUS_1
#define CACHE_STATUS_CLOSEING_FLAG CACHE_STATUS_CACHE_STATUS

/**
 * @}
 */

/** @defgroup FLASH_Interrupt_definition FLASH Interrupt definition
 * @brief FLASH Interrupt definition
 * @{
 */
#define FLASH_IN_POWERDOWN EFC_LPCR_MODES      /*!<  */
#define FLASH_IN_SLEEP     (~(EFC_LPCR_MODES)) /*!<  */
/**
 * @}
 */

/**
 * @}
 */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup FLASH_Exported_Macros FLASH Exported Macro
 * @{
 */
#if defined(UM32x42x)
#define SN_BASE_ADDR (0x04001D48) // SN base addr
#define SN_CRC_ADDR  (0x04001D58) // SN CRC value

#define OPTION_BASE_ADDR  0x04000000U
#define OPT_WRITE_PROTECT (OPTION_BASE_ADDR + 0xFA0) // 32bit
#define OPT_DEBUG_DISABLE (OPTION_BASE_ADDR + 0xFB0) // 8bit  ��0xab ???????
#endif


#if defined(UM324xF)
#define SN_BASE_ADDR     						(0x04008048)                    /*!< SN base addr */
#define SN_CRC_ADDR     						(0x04008058)                    /*!< SN CRC value */

#define OPTION_BASE_ADDR  		                0x04000000U
#define OPT_WRITE_PROTECT   	                (OPTION_BASE_ADDR + 0x1FA0)     /*!< 32bit */
#define OPT_DEBUG_DISABLE   	                (OPTION_BASE_ADDR + 0x1FB0)     /*!< 8bit  Write 0xab to be prohibit debug */
#define OPT_READ_PROTECT                        (OPTION_BASE_ADDR + 0x1FC0)     /*!< 32bit */

#endif

#if defined(UM324xH)
#define SN_BASE_ADDR (0x04001D48) // SN base addr
#define SN_CRC_ADDR  (0x04001D58) // SN CRC value

#define OPTION_BASE_ADDR  0x04000000U
#define OPT_WRITE_PROTECT (OPTION_BASE_ADDR + 0xFA0) // 32bit
#define OPT_DEBUG_DISABLE (OPTION_BASE_ADDR + 0xFB0) // 8bit  ��0xab ???????
#endif

 #if defined(UM324xF)
/**
  * @brief  Enable the FLASH instruction cache.
  * @retval none
  */ 
#define __HAL_FLASH_INSTRUCTION_CACHE_ENABLE()  (ICACHE->CTRL |= CACHE_CTRL_CACHE_EN)


/**
  * @brief  Disable the FLASH instruction cache.
  * @retval none
  */ 
#define __HAL_FLASH_INSTRUCTION_CACHE_DISABLE()   (ICACHE->CTRL &= (~CACHE_CTRL_CACHE_EN))

/**
  * @brief  if the FLASH instruction cache is enable.
  * @retval none
  */
#define __HAL_FLASH_INSTRUCTION_CACHE_IS_ENABLE() ((ICACHE->CTRL & CACHE_CTRL_CACHE_EN) == CACHE_CTRL_CACHE_EN? 1 : 0)
/**
  * @brief  Enable the FLASH data cache.
  * @retval none
  */ 
#define __HAL_FLASH_DATA_CACHE_ENABLE()  (DCACHE->CTRL |= CACHE_CTRL_CACHE_EN)

 
/**
  * @brief  Disable the FLASH data cache.
  * @retval none
  */ 
#define __HAL_FLASH_DATA_CACHE_DISABLE()   (DCACHE->CTRL &= (~CACHE_CTRL_CACHE_EN))

/**
  * @brief  if the DATA_CACHE cache is enable.
  * @retval none
  */
#define __HAL_FLASH_DATA_CACHE_IS_ENABLE()  ((DCACHE->CTRL & CACHE_CTRL_CACHE_EN) == CACHE_CTRL_CACHE_EN? 1 : 0)

/**
  * @brief  if the DATA_CACHE cache is enable.
  * @retval none
  */
#define __HAL_FLASH_DATA_CACHE_GET_STATUS()  (DCACHE->STATUS & CACHE_STATUS_CACHE_STATUS)

/**
  * @brief  Enable the FLASH instruction cache prefetch buffer.
  * @retval none
  */ 
#define __HAL_FLASH_INSTRUCTION_CACHE_PREFETCH_ENABLE()  (ICACHE->CTRL |= CACHE_CTRL_PREFETCH)

/**
  * @brief  Disable the FLASH instruction cache prefetch buffer.
  * @retval none
  */ 
#define __HAL_FLASH_INSTRUCTION_CACHE_PREFETCH_DISABLE()   (ICACHE->CTRL &= (~CACHE_CTRL_PREFETCH))  


#define __HAL_FLASH_INSTRUCTION_CACHE_PREFETCH_IS_ENABLE()  (((ICACHE->CTRL & CACHE_CTRL_PREFETCH) == CACHE_CTRL_PREFETCH)? 1 : 0  )
/**
  * @brief  Enable the FLASH data cache prefetch buffer.
  * @retval none
  */ 
#define __HAL_FLASH_DATA_CACHE_PREFETCH_ENABLE()  (DCACHE->CTRL |= CACHE_CTRL_PREFETCH)

/**
  * @brief  Disable the FLASH data cache prefetch buffer.
  * @retval none
  */ 
#define __HAL_FLASH_DATA_CACHE_PREFETCH_DISABLE()   (DCACHE->CTRL &= (~CACHE_CTRL_PREFETCH))  

#define __HAL_FLASH_DATA_CACHE_PREFETCH_IS_ENABLE() (((DCACHE->CTRL & CACHE_CTRL_PREFETCH) == CACHE_CTRL_PREFETCH)? 1 : 0  )
#endif



#if defined(UM32x42x) || defined(UM32x41x)
/**
  * @brief  Enable the FLASH instruction cache.
  * @retval none
  */ 
#define __HAL_FLASH_INSTRUCTION_CACHE_ENABLE() WRITE_REG(CACHE->CTRL, (CACHE_CTRL_CACHE_EN | CACHE_CTRL_CACHE_CLR))

/**
  * @brief  Disable the FLASH instruction cache.
  * @retval none
  */ 
#define __HAL_FLASH_INSTRUCTION_CACHE_DISABLE() CLEAR_BIT(CACHE->CTRL, CACHE_CTRL_CACHE_EN)


#define __HAL_FLASH_INSTRUCTION_CACHE_IS_ENABLE()  (((CACHE->CTRL & CACHE_CTRL_CACHE_EN) == CACHE_CTRL_CACHE_EN)? 1:0)
#endif



#if defined(UM324xH)
/**
  * @brief  Enable the FLASH instruction cache.
  * @retval none
  */ 
#define __HAL_FLASH_INSTRUCTION_CACHE_ENABLE() WRITE_REG(CACHE->CTRL, (CACHE_CTRL_CACHE_EN | CACHE_CTRL_CACHE_CLR))

/**
  * @brief  Disable the FLASH instruction cache.
  * @retval none
  */ 
#define __HAL_FLASH_INSTRUCTION_CACHE_DISABLE() CLEAR_BIT(CACHE->CTRL, CACHE_CTRL_CACHE_EN)


#define __HAL_FLASH_INSTRUCTION_CACHE_IS_ENABLE()  (((CACHE->CTRL & CACHE_CTRL_CACHE_EN) == CACHE_CTRL_CACHE_EN)? 1:0)
#endif


/**
 * @brief  Set the FLASH Read wait cycles.
 * @param  __RWAITCYC__ FLASH Read wait cycles
 *         The value of this parameter depend on device used within the same series
 * @retval none
 */
#define __HAL_FLASH_SET_RWAITCYC(__RWAITCYC__) MODIFY_REG(EFC->TIME, EFC_TIME_RWAITCYC, (0xA5000000 | (__RWAITCYC__)))
/**
 * @brief  Get the FLASH Read wait cycles.
 * @retval FLASH Rwaitcyc
 */
#define __HAL_FLASH_GET_RWAITCYC() (READ_BIT(EFC->TIME, EFC_TIME_RWAITCYC))

/**
 * @brief   Behavior selection of Flash when the system enters low power consumption mode.
 * @param   __MODES__
 *           This parameter can be any combination of the following values:
 *           @arg  FLASH_IN_POWERDOWN  Flash enters power-down mode.
 *           @arg  FLASH_IN_SLEEP      Flash enters sleep mode.
 * @retval  none
 */
#define __HAL_FLASH_SET_MODES(__MODES__) MODIFY_REG(EFC->LPCR, EFC_LPCR_MODES, (0xA5000000 | (__MODES__)))

/**
 * @brief   Record the last address where erasing or programming was completed.
 * @param   none
 * @retval  Return the record the last address where erasing or programming was completed.
 */
#define __HAL_FLASH_GET_ADDRREC() (READ_REG(EFC->ADDRREC))

/**
 * @brief  Set the time scale for FLASH erase and write.
 * @param  __FREQ__ the time scale for FLASH erase and write
 *         The value of this parameter depend on device used within the same series
 * @retval none
 */
#define __HAL_FLASH_SET_FREQ(__FREQ__) MODIFY_REG(EFC->TIME, EFC_TIME_FREQ, (0xA5000000 | (__FREQ__ - 0x1)))
/**
 * @brief  Resets the FLASH instruction Cache.
 * @note   This function must be used only when the Instruction Cache is disabled.
 * @retval None
 */
#define __HAL_FLASH_INSTRUCTION_CACHE_RESET() \
    do {                                      \
        ICACHE->CTRL &= ~CACHE_CTRL_ALL_EN;   \
        ICACHE->CTRL |= CACHE_CTRL_ALL_EN;    \
    } while (0U)

/**
 * @brief  Resets the FLASH data Cache.
 * @note   This function must be used only when the data Cache is disabled.
 * @retval None
 */
#define __HAL_FLASH_DATA_CACHE_RESET()      \
    do {                                    \
        DCACHE->CTRL &= ~CACHE_CTRL_ALL_EN; \
        DCACHE->CTRL |= CACHE_CTRL_ALL_EN;  \
    } while (0U)

/**
 * @brief  Checks whether the specified CACHE flag is set or not.
 * @param  CACHEx   This parameter can be CACHEx where x can be(I or D)
 * @param  __FLAG__ specifies the CACHE flag to check.
 *         This parameter can be any combination of the following values:
 *     @arg CACHE_STATUS_CLOSED_FLAG
 *     @arg CACHE_STATUS_OPENING_FLAG
 *     @arg CACHE_STATUS_OPEND_FLAG
 *     @arg CACHE_STATUS_CLOSEING_FLAG
 * @retval The new state of __FLAG__ (SET or RESET).
 */
#define __HAL_FLASH_CACHE_GET_FLAG(CACHEx, __FLAG__) (CACHEx->STATUS & (__FLAG__))

/**
 * @brief  Get the data of Hit statistics register
 * @param  CACHEx   This parameter can be CACHEx where x can be(I or D)
 * @retval The data of Hit statistics register
 */
#define __HAL_CACHE_GET_HITCNT(CACHEx) (READ_REG(CACHEx->HITCNT))

/**
 * @brief  Get the data of Miss statistics register
 * @param  CACHEx   This parameter can be CACHEx where x can be(I or D)
 * @retval The data of Miss statistics register
 */
#define __HAL_CACHE_GET_MISSCNT(CACHEx) (READ_REG(CACHEx->MISSCNT))

/**
 * @brief  Enable the specified FLASH interrupt.
 * @param  __INTERRUPT__  FLASH interrupt
 *         This parameter can be any combination of the following values:
 *     @arg FLASH_IT_OPDE      End of FLASH Operation Interrupt source
 *     @arg FLASH_IT_VDDLE     VDD is low Interrupt source
 *     @arg FLASH_IT_PVFE      Write check error Interrupt source
 *     @arg FLASH_IT_EVFE      Erasure check Interrupt source
 *     @arg FLASH_IT_ALL       Enable all the Interrupt source
 * @retval none
 */
#define __HAL_FLASH_ENABLE_IT(__INTERRUPT__) (EFC->INTEN |= (__INTERRUPT__))

/**
 * @brief  Disable the specified FLASH interrupt.
 * @param  __INTERRUPT__  FLASH interrupt
 *         This parameter can be any combination of the following values:
 *     @arg FLASH_IT_OPDE      End of FLASH Operation Interrupt source
 *     @arg FLASH_IT_VDDLE     VDD is low Interrupt source
 *     @arg FLASH_IT_PVFE      Write check error Interrupt source
 *     @arg FLASH_IT_EVFE      Erasure check Interrupt source
 *     @arg FLASH_IT_ALL       Enable all the Interrupt source
 * @retval none
 */
#define __HAL_FLASH_DISABLE_IT(__INTERRUPT__) (EFC->INTEN &= ~(uint32_t)(__INTERRUPT__))

/**
 * @brief  Get the specified FLASH flag status.
 * @param  __FLAG__ specifies the FLASH flags to check.
 *          This parameter can be any combination of the following values:
 *            @arg FLASH_FLAG_RDYS            FLASH End of Operation flag
 *            @arg FLASH_FLAG_CONPROGRDY      FLASH is in the state of waiting for the next continuous programming.
 *            @arg FLASH_FLAG_VDDS            FLASH Low voltage warning flag
 *            @arg FLASH_FLAG_LPSPD           FLASH Power down mode flag
 *            @arg FLASH_FLAG_LPSLEEP         FLASH Sleeping mode flag
 *            @arg FLASH_FLAG_LPSLVDD         FLASH Low voltage operation flag
 * @retval The new state of __FLAG__ (SET or RESET).
 */
#define __HAL_FLASH_GET_FLAG(__FLAG__) ((EFC->STATUS & (__FLAG__)))
/**
 * @brief  Get the specified FLASH flag status.
 * @param  __FLAG__ specifies the FLASH flags to check.
 *          This parameter can be any combination of the following values:
 *            @arg FLASH_FLAG_OPDS     FLASH operation or erase successfully flag
 *            @arg FLASH_FLAG_VDDLS    VDD voltage low interrupt status bit flag
 *            @arg FLASH_FLAG_PVFS     FLASH Write check error flag
 *            @arg FLASH_FLAG_EVFS     FLASH Erasure check error flag
 * @retval The new state of __FLAG__ (SET or RESET).
 */
#define __HAL_FLASH_GET_ITFLAG(__FLAG__) ((EFC->INTSTATUS & (__FLAG__)))

/**
 * @brief  Clear the specified FLASH flags.
 * @param  __FLAG__ specifies the FLASH flags to clear.
 *          This parameter can be any combination of the following values:
 *            @arg FLASH_FLAG_OPDS     FLASH operation or erase successfully flag
 *            @arg FLASH_FLAG_VDDLS    VDD voltage low interrupt status bit flag
 *            @arg FLASH_FLAG_PVFS     FLASH Write check error flag
 *            @arg FLASH_FLAG_EVFS     FLASH Erasure check error flag
 * @retval none
 */
#define __HAL_FLASH_CLEAR_FLAG(__FLAG__) (EFC->INTSTATUS = (__FLAG__))
/**
 * @}
 */

/* Include FLASH HAL Extension module */


/* Exported functions --------------------------------------------------------*/
/** @addtogroup FLASH_Exported_Functions
 * @{
 */
/** @addtogroup FLASH_Exported_Functions_Group1
 * @{
 */
/* Program operation functions  ***********************************************/
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint32_t Data);
HAL_StatusTypeDef HAL_FLASH_Program_Continue(uint32_t Address, uint32_t *Data, uint32_t data_len);
HAL_StatusTypeDef HAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint32_t Data);
// HAL_StatusTypeDef HAL_FLASH_RewriteWord(uint32_t Address, uint32_t Data);
HAL_StatusTypeDef HAL_FLASH_Erase_Page(uint32_t Address);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *PageError);
void              HAL_FLASH_SetKeyctrl(uint8_t Sha_icv, uint8_t Aes_key2_position, uint8_t Aes_key1_position);
/* FLASH IRQ handler method */
void HAL_FLASH_IRQHandler(void);
/* Callbacks in non blocking modes */
void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue);
void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue);

#define HAL_FLASH_READ_BYTE(addr)     (*(volatile uint8_t *)(addr))
#define HAL_FLASH_READ_HALFWORD(addr) (*(volatile uint16_t *)(addr))
#define HAL_FLASH_READ_WORD(addr)     (*(volatile uint32_t *)(addr))
/**
 * @}
 */

/** @addtogroup FLASH_Exported_Functions_Group2
 * @{
 */
/* Peripheral Control functions  **********************************************/
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
/**
 * @}
 */

/** @addtogroup FLASH_Exported_Functions_Group3
 * @{
 */
/* Peripheral State functions  ************************************************/
uint32_t          HAL_FLASH_GetError(void);
HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout);
/**
 * @}
 */
/**
 * @}
 */

/* Private constants ---------------------------------------------------------*/
/** @defgroup FLASH_Rwaitcyc FLASH Read wait cycle
 * @{
 */
#if defined(UM32x42x) || defined(UM32x41x)
#define FLASH_RWAITCYC_0  (0x00000000UL)
#define FLASH_RWAITCYC_1  (EFC_TIME_RWAITCYC_0)                                                                   /*!< FLASH Read wait 1 system cycle */
#define FLASH_RWAITCYC_2  (EFC_TIME_RWAITCYC_1)                                                                   /*!< FLASH Read wait 2 system cycle */
#define FLASH_RWAITCYC_3  (EFC_TIME_RWAITCYC_1 | EFC_TIME_RWAITCYC_0)                                             /*!< FLASH Read wait 3 system cycle */
#define FLASH_RWAITCYC_4  (EFC_TIME_RWAITCYC_2)                                                                   /*!< FLASH Read wait 4 system cycle */
#define FLASH_RWAITCYC_5  (EFC_TIME_RWAITCYC_2 | EFC_TIME_RWAITCYC_0)                                             /*!< FLASH Read wait 5 system cycle */
#define FLASH_RWAITCYC_6  (EFC_TIME_RWAITCYC_2 | EFC_TIME_RWAITCYC_1)                                             /*!< FLASH Read wait 6 system cycle */
#define FLASH_RWAITCYC_7  (EFC_TIME_RWAITCYC_2 | EFC_TIME_RWAITCYC_1 | EFC_TIME_RWAITCYC_0)                       /*!< FLASH Read wait 7 system cycle */
#define FLASH_RWAITCYC_8  (EFC_TIME_RWAITCYC_3)                                                                   /*!< FLASH Read wait 8 system cycle */
#define FLASH_RWAITCYC_9  (EFC_TIME_RWAITCYC_3 | EFC_TIME_RWAITCYC_0)                                             /*!< FLASH Read wait 9 system cycle */
#define FLASH_RWAITCYC_10 (EFC_TIME_RWAITCYC_3 | EFC_TIME_RWAITCYC_1)                                             /*!< FLASH Read wait 10 system cycle */
#define FLASH_RWAITCYC_11 (EFC_TIME_RWAITCYC_3 | EFC_TIME_RWAITCYC_1 | EFC_TIME_RWAITCYC_0)                       /*!< FLASH Read wait 11 system cycle */
#define FLASH_RWAITCYC_12 (EFC_TIME_RWAITCYC_3 | EFC_TIME_RWAITCYC_2)                                             /*!< FLASH Read wait 12 system cycle */
#define FLASH_RWAITCYC_13 (EFC_TIME_RWAITCYC_3 | EFC_TIME_RWAITCYC_2 | EFC_TIME_RWAITCYC_0)                       /*!< FLASH Read wait 13 system cycle */
#define FLASH_RWAITCYC_14 (EFC_TIME_RWAITCYC_3 | EFC_TIME_RWAITCYC_2 | EFC_TIME_RWAITCYC_1)                       /*!< FLASH Read wait 14 system cycle */
#define FLASH_RWAITCYC_15 (EFC_TIME_RWAITCYC_3 | EFC_TIME_RWAITCYC_2 | EFC_TIME_RWAITCYC_1 | EFC_TIME_RWAITCYC_0) /*!< FLASH Read wait 15 system cycle */
#endif

#if defined(UM324xF)
#define FLASH_RWAITCYC_0                FLASH_EFC_TIME_RWAITCYC_0WC   /*!< FLASH Zero Read wait cycle      */
#define FLASH_RWAITCYC_1                FLASH_EFC_TIME_RWAITCYC_1WC   /*!< FLASH One Read wait cycle       */
#define FLASH_RWAITCYC_2                FLASH_EFC_TIME_RWAITCYC_2WC   /*!< FLASH Two Read wait cycles      */
#define FLASH_RWAITCYC_3                FLASH_EFC_TIME_RWAITCYC_3WC   /*!< FLASH Three Read wait cycles    */
#define FLASH_RWAITCYC_4                FLASH_EFC_TIME_RWAITCYC_4WC   /*!< FLASH Four Read wait cycles     */
#define FLASH_RWAITCYC_5                FLASH_EFC_TIME_RWAITCYC_5WC   /*!< FLASH Five Read wait cycles     */
#define FLASH_RWAITCYC_6                FLASH_EFC_TIME_RWAITCYC_6WC   /*!< FLASH Five Read wait cycles     */
#endif 

#if defined(UM324xH)
#define FLASH_RWAITCYC_0  (0x00000000UL)
#define FLASH_RWAITCYC_1  (EFC_TIME_RWAITCYC_0)                                                                   /*!< FLASH Read wait 1 system cycle */
#define FLASH_RWAITCYC_2  (EFC_TIME_RWAITCYC_1)                                                                   /*!< FLASH Read wait 2 system cycle */
#define FLASH_RWAITCYC_3  (EFC_TIME_RWAITCYC_1 | EFC_TIME_RWAITCYC_0)                                             /*!< FLASH Read wait 3 system cycle */
#define FLASH_RWAITCYC_4  (EFC_TIME_RWAITCYC_2)                                                                   /*!< FLASH Read wait 4 system cycle */
#define FLASH_RWAITCYC_5  (EFC_TIME_RWAITCYC_2 | EFC_TIME_RWAITCYC_0)                                             /*!< FLASH Read wait 5 system cycle */
#define FLASH_RWAITCYC_6  (EFC_TIME_RWAITCYC_2 | EFC_TIME_RWAITCYC_1)                                             /*!< FLASH Read wait 6 system cycle */
#define FLASH_RWAITCYC_7  (EFC_TIME_RWAITCYC_2 | EFC_TIME_RWAITCYC_1 | EFC_TIME_RWAITCYC_0)                       /*!< FLASH Read wait 7 system cycle */
#define FLASH_RWAITCYC_8  (EFC_TIME_RWAITCYC_3)                                                                   /*!< FLASH Read wait 8 system cycle */
#define FLASH_RWAITCYC_9  (EFC_TIME_RWAITCYC_3 | EFC_TIME_RWAITCYC_0)                                             /*!< FLASH Read wait 9 system cycle */
#define FLASH_RWAITCYC_10 (EFC_TIME_RWAITCYC_3 | EFC_TIME_RWAITCYC_1)                                             /*!< FLASH Read wait 10 system cycle */
#define FLASH_RWAITCYC_11 (EFC_TIME_RWAITCYC_3 | EFC_TIME_RWAITCYC_1 | EFC_TIME_RWAITCYC_0)                       /*!< FLASH Read wait 11 system cycle */
#define FLASH_RWAITCYC_12 (EFC_TIME_RWAITCYC_3 | EFC_TIME_RWAITCYC_2)                                             /*!< FLASH Read wait 12 system cycle */
#define FLASH_RWAITCYC_13 (EFC_TIME_RWAITCYC_3 | EFC_TIME_RWAITCYC_2 | EFC_TIME_RWAITCYC_0)                       /*!< FLASH Read wait 13 system cycle */
#define FLASH_RWAITCYC_14 (EFC_TIME_RWAITCYC_3 | EFC_TIME_RWAITCYC_2 | EFC_TIME_RWAITCYC_1)                       /*!< FLASH Read wait 14 system cycle */
#define FLASH_RWAITCYC_15 (EFC_TIME_RWAITCYC_3 | EFC_TIME_RWAITCYC_2 | EFC_TIME_RWAITCYC_1 | EFC_TIME_RWAITCYC_0) /*!< FLASH Read wait 15 system cycle */
#endif
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __UM324xx_HAL_FLASH_H */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/


