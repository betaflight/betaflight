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
 * RCC clock enable/disable: HAL2 renames __HAL_RCC_xxx_CLK_ENABLE.
 * Add any needed aliases here as peripheral support grows.
 * -------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------
 * GPIO speed: HAL2 may rename GPIO_SPEED_FREQ_xxx constants.
 * -------------------------------------------------------------------------- */

#endif /* STM32C5XX_HAL2_COMPAT_H */
