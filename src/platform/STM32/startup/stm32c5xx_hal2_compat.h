/**
  ******************************************************************************
  * @file    stm32c5xx_hal2_compat.h
  * @brief   Compatibility shims mapping old STM32 HAL/LL API names used by
  *          Betaflight to the new HAL2 (Cube 2.0) equivalents in STM32CubeC5.
  *
  *          Include this header AFTER the device and HAL headers.
  ******************************************************************************
  */

#ifndef STM32C5XX_HAL2_COMPAT_H
#define STM32C5XX_HAL2_COMPAT_H

/* --------------------------------------------------------------------------
 * Register-access macros: HAL2 renamed SET_BIT → STM32_SET_BIT, etc.
 * Betaflight (and CMSIS HAL1) code uses the short names everywhere.
 * -------------------------------------------------------------------------- */
#ifndef SET_BIT
#define SET_BIT     STM32_SET_BIT
#endif
#ifndef CLEAR_BIT
#define CLEAR_BIT   STM32_CLEAR_BIT
#endif
#ifndef READ_BIT
#define READ_BIT    STM32_READ_BIT
#endif
#ifndef WRITE_REG
#define WRITE_REG   STM32_WRITE_REG
#endif
#ifndef READ_REG
#define READ_REG    STM32_READ_REG
#endif
#ifndef MODIFY_REG
#define MODIFY_REG  STM32_MODIFY_REG
#endif

/* --------------------------------------------------------------------------
 * HAL status type: HAL2 uses hal_status_t, old code uses HAL_StatusTypeDef
 * -------------------------------------------------------------------------- */
typedef hal_status_t HAL_StatusTypeDef;

/* --------------------------------------------------------------------------
 * FunctionalState: removed in HAL2 CMSIS.  Many Betaflight platform files
 * and the old LL drivers depend on this enum.
 * -------------------------------------------------------------------------- */
#ifndef __FUNCTIONALSTATE_DEFINED
#define __FUNCTIONALSTATE_DEFINED
typedef enum {
    DISABLE = 0U,
    ENABLE  = !DISABLE
} FunctionalState;
#endif

/* --------------------------------------------------------------------------
 * FLASH_PAGE_SIZE: HAL2 defines this in the flash header; Betaflight also
 * defines it in target.h.  Undefine the HAL version so ours takes precedence.
 * -------------------------------------------------------------------------- */
#ifdef FLASH_PAGE_SIZE
#undef FLASH_PAGE_SIZE
#endif

/* --------------------------------------------------------------------------
 * GPIO mode/speed/pull constants: HAL2 renames all GPIO constants.
 * Betaflight's IO_CONFIG packing encodes mode in bits 0-1, otype in bit 4,
 * speed in bits 2-3, pull in bits 5-6.  These values match LL register fields.
 * -------------------------------------------------------------------------- */
#define GPIO_MODE_INPUT       0x00U
#define GPIO_MODE_OUTPUT_PP   0x01U
#define GPIO_MODE_OUTPUT_OD   0x11U
#define GPIO_MODE_AF_PP       0x02U
#define GPIO_MODE_AF_OD       0x12U
#define GPIO_MODE_ANALOG      0x03U

#define GPIO_NOPULL           0x00U
#define GPIO_PULLUP           0x01U
#define GPIO_PULLDOWN         0x02U

#define GPIO_SPEED_FREQ_LOW        0x00U
#define GPIO_SPEED_FREQ_MEDIUM     0x01U
#define GPIO_SPEED_FREQ_HIGH       0x02U
#define GPIO_SPEED_FREQ_VERY_HIGH  0x03U

/* --------------------------------------------------------------------------
 * LL DMA API name changes: HAL2 renamed many LL DMA constants.
 * -------------------------------------------------------------------------- */
#define LL_DMA_NORMAL                   0x00000000U
#define LL_DMA_DATA_ALIGN_ZEROPADD      LL_DMA_DATA_ALIGN_ZERO_PAD
#define LL_DMA_SRC_DATAWIDTH_BYTE       LL_DMA_SRC_DATA_WIDTH_BYTE
#define LL_DMA_SRC_DATAWIDTH_HALFWORD   LL_DMA_SRC_DATA_WIDTH_HALFWORD
#define LL_DMA_SRC_DATAWIDTH_WORD       LL_DMA_SRC_DATA_WIDTH_WORD
#define LL_DMA_DEST_DATAWIDTH_BYTE      LL_DMA_DEST_DATA_WIDTH_BYTE
#define LL_DMA_DEST_DATAWIDTH_HALFWORD  LL_DMA_DEST_DATA_WIDTH_HALFWORD
#define LL_DMA_DEST_DATAWIDTH_WORD      LL_DMA_DEST_DATA_WIDTH_WORD
#define LL_DMA_SRC_FIXED                LL_DMA_SRC_ADDR_FIXED
#define LL_DMA_SRC_INCREMENT            LL_DMA_SRC_ADDR_INCREMENTED
#define LL_DMA_DEST_FIXED               LL_DMA_DEST_ADDR_FIXED
#define LL_DMA_DEST_INCREMENT           LL_DMA_DEST_ADDR_INCREMENTED
#define LL_DMA_HWREQUEST_SINGLEBURST    LL_DMA_HWREQUEST_BLK
#define LL_DMA_LOW_PRIORITY_LOW_WEIGHT  LL_DMA_PRIORITY_LOW_WEIGHT_MID

/* --------------------------------------------------------------------------
 * TIM HAL constants: HAL2 renames many timer defines.
 * -------------------------------------------------------------------------- */
#define TIM_CCx_ENABLE                  LL_TIM_CHANNEL_CH1  /* placeholder for CC enable */
#define TIM_CCx_DISABLE                 0x0000U
#define TIM_ICPOLARITY_RISING           LL_TIM_IC_POLARITY_RISING
#define TIM_ICPOLARITY_FALLING          LL_TIM_IC_POLARITY_FALLING
#define TIM_ICPSC_DIV1                  LL_TIM_ICPSC_DIV1
#define TIM_ICSELECTION_DIRECTTI        LL_TIM_ACTIVEINPUT_DIRECTTI
#define TIM_OCMODE_PWM1                 LL_TIM_OCMODE_PWM1
#define TIM_OCPOLARITY_HIGH             LL_TIM_OCPOLARITY_HIGH
#define TIM_OCPOLARITY_LOW              LL_TIM_OCPOLARITY_LOW
#define TIM_OCNPOLARITY_HIGH            LL_TIM_OCPOLARITY_HIGH
#define TIM_OCNPOLARITY_LOW             LL_TIM_OCPOLARITY_LOW
#define TIM_OCIDLESTATE_SET             LL_TIM_OCIDLESTATE_HIGH
#define TIM_OCNIDLESTATE_SET            LL_TIM_OCIDLESTATE_HIGH
#define TIM_OCFAST_DISABLE              0x00000000U

/* --------------------------------------------------------------------------
 * Flash constants: HAL2 renames flash types.
 * -------------------------------------------------------------------------- */
#define FLASH_BANK_1                    LL_FLASH_BANK_1
#define FLASH_BANK_2                    LL_FLASH_BANK_2

/* --------------------------------------------------------------------------
 * NVIC priority grouping: HAL2 drops the HAL wrappers, use CMSIS directly.
 * The old NVIC_PRIORITYGROUP_2 = 0x05 maps to 2 bits preemption / 2 bits sub.
 * -------------------------------------------------------------------------- */
#define HAL_NVIC_SetPriorityGrouping  NVIC_SetPriorityGrouping
#define HAL_NVIC_SetPriority          NVIC_SetPriority
#define HAL_NVIC_EnableIRQ            NVIC_EnableIRQ
#define HAL_NVIC_DisableIRQ           NVIC_DisableIRQ

#ifndef NVIC_PRIORITYGROUP_2
#define NVIC_PRIORITYGROUP_2  ((uint32_t)0x00000005)
#endif

/* --------------------------------------------------------------------------
 * HAL RCC: HAL2 renames DeInit.
 * -------------------------------------------------------------------------- */
#define HAL_RCC_DeInit()           HAL_RCC_Reset()
#define HAL_RCC_GetSysClockFreq()  HAL_RCC_GetHCLKFreq()

/* --------------------------------------------------------------------------
 * HAL PWR: backup access control.
 * -------------------------------------------------------------------------- */
#define HAL_PWR_EnableBkUpAccess() ((void)0)  /* C5: backup regs accessible without enable */
#define __HAL_RCC_PWR_CLK_ENABLE() ((void)0)

#endif /* STM32C5XX_HAL2_COMPAT_H */
