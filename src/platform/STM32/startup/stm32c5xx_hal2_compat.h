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

#define GPIO_PIN_RESET             0U
#define GPIO_PIN_SET               1U

/* --------------------------------------------------------------------------
 * GPIO: HAL2 removed GPIO_InitTypeDef (uses LL_GPIO_InitTypeDef instead).
 * Many shared files use the old HAL GPIO_InitTypeDef for EXTI config etc.
 * -------------------------------------------------------------------------- */
typedef struct {
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Speed;
    uint32_t OutputType;
    uint32_t Pull;
    uint32_t Alternate;
} GPIO_InitTypeDef;
#define GPIO_MODE_IT_RISING          0x10110000U
#define GPIO_MODE_IT_FALLING         0x10210000U
#define GPIO_MODE_IT_RISING_FALLING  0x10310000U

/* --------------------------------------------------------------------------
 * TIM: HAL2 removed HAL TIM init structs. Stub them for struct declarations.
 * -------------------------------------------------------------------------- */
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
typedef struct {
    uint32_t OCMode; uint32_t Pulse; uint32_t OCPolarity; uint32_t OCNPolarity;
    uint32_t OCFastMode; uint32_t OCIdleState; uint32_t OCNIdleState;
} TIM_OC_InitTypeDef;
typedef struct {
    uint32_t ICPolarity; uint32_t ICSelection; uint32_t ICPrescaler; uint32_t ICFilter;
} TIM_IC_InitTypeDef;
typedef struct {
    uint32_t OCMode; uint32_t Pulse; uint32_t OCPolarity; uint32_t OCNPolarity;
    uint32_t OCFastMode; uint32_t OCIdleState; uint32_t OCNIdleState;
} LL_TIM_OC_InitTypeDef;
typedef struct {
    uint32_t ICPolarity; uint32_t ICActiveInput; uint32_t ICPrescaler; uint32_t ICFilter;
} LL_TIM_IC_InitTypeDef;
typedef struct {
    uint32_t Prescaler; uint32_t CounterMode; uint32_t Autoreload;
    uint32_t ClockDivision; uint32_t RepetitionCounter;
} LL_TIM_InitTypeDef;
typedef struct { void *Instance; } UART_HandleTypeDef;

/* --------------------------------------------------------------------------
 * Flash: HAL2 completely rewrites flash API. Stub types for config_flash.c.
 * -------------------------------------------------------------------------- */
typedef struct {
    uint32_t TypeErase;
    uint32_t Banks;
    uint32_t Sector;
    uint32_t NbSectors;
} FLASH_EraseInitTypeDef;
#define FLASH_TYPEERASE_SECTORS 0x02U
#define FLASH_TYPEPROGRAM_QUADWORD 0x03U

/* --------------------------------------------------------------------------
 * LL DMA types: HAL2 removed LL_DMA_InitTypeDef. Provide a stub struct so
 * that bus.h compiles (actual DMA config uses direct register writes).
 * -------------------------------------------------------------------------- */
typedef struct {
    uint32_t placeholder;  // HAL2 has no LL_DMA_Init; config via registers
} LL_DMA_InitTypeDef;

/* --------------------------------------------------------------------------
 * HAL DMA handle: stub for headers that reference DMA_HandleTypeDef.
 * -------------------------------------------------------------------------- */
typedef struct {
    DMA_Channel_TypeDef *Instance;
} DMA_HandleTypeDef;

/* --------------------------------------------------------------------------
 * LL DMA API name changes: HAL2 renamed many LL DMA constants.
 * -------------------------------------------------------------------------- */
#define LL_DMA_NORMAL                   0x00000000U
#define LL_DMA_DATA_ALIGN_ZEROPADD      0x00000000U  /* HAL2 removed alignment control */
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
#define LL_DMA_HWREQUEST_SINGLEBURST    LL_DMA_HARDWARE_REQUEST_BURST
#define LL_DMA_LOW_PRIORITY_LOW_WEIGHT  LL_DMA_PRIORITY_LOW_WEIGHT_MID

/* --------------------------------------------------------------------------
 * TIM HAL constants: HAL2 renames many timer defines.
 * -------------------------------------------------------------------------- */
#define TIM_CCx_ENABLE                  LL_TIM_CHANNEL_CH1  /* placeholder for CC enable */
#define TIM_CCx_DISABLE                 0x0000U
#define TIM_ICPOLARITY_RISING           LL_TIM_IC_POLARITY_RISING
#define TIM_ICPOLARITY_FALLING          LL_TIM_IC_POLARITY_FALLING
#define TIM_ICPSC_DIV1                  LL_TIM_ICPSC_DIV1
#define TIM_ICSELECTION_DIRECTTI        LL_TIM_ACTIVEINPUT_DIRECT
#define TIM_OCMODE_PWM1                 LL_TIM_OCMODE_PWM1
#define TIM_OCPOLARITY_HIGH             LL_TIM_OCPOLARITY_HIGH
#define TIM_OCPOLARITY_LOW              LL_TIM_OCPOLARITY_LOW
#define TIM_OCNPOLARITY_HIGH            LL_TIM_OCPOLARITY_HIGH
#define TIM_OCNPOLARITY_LOW             LL_TIM_OCPOLARITY_LOW
#define TIM_OCIDLESTATE_SET             LL_TIM_OCIDLESTATE_SET
#define TIM_OCNIDLESTATE_SET            LL_TIM_OCIDLESTATE_SET
#define TIM_OCFAST_DISABLE              0x00000000U

/* --------------------------------------------------------------------------
 * Flash: HAL2 completely rewrites the flash API. Provide stubs so
 * config_flash.c compiles. Actual flash operations need a HAL2 fork.
 * -------------------------------------------------------------------------- */
#define FLASH_BANK_1                    0x01U
#define FLASH_BANK_2                    0x02U

#define HAL_FLASH_Unlock()              ((void)0)
#define HAL_FLASH_Lock()                ((void)0)
#define HAL_FLASH_Program(type, addr, data) ((void)(addr), (void)(data), HAL_OK)
#define HAL_FLASHEx_Erase(pEraseInit, pSectorError) ((void)(pEraseInit), (void)(pSectorError), HAL_OK)

/* --------------------------------------------------------------------------
 * TIM HAL function renames: HAL2 simplifies TIM API.
 * -------------------------------------------------------------------------- */
#define HAL_TIM_Base_Start(htim)            do { (void)(htim); } while(0)
#define HAL_TIM_PWM_Start(htim, ch)         do { (void)(htim); (void)(ch); } while(0)
#define HAL_TIMEx_PWMN_Start(htim, ch)      do { (void)(htim); (void)(ch); } while(0)
#define HAL_TIM_PWM_ConfigChannel(htim, cfg, ch)  do { (void)(htim); (void)(cfg); (void)(ch); } while(0)
#define HAL_TIM_IC_ConfigChannel(htim, cfg, ch)   do { (void)(htim); (void)(cfg); (void)(ch); } while(0)
#define HAL_TIM_IC_Start_IT(htim, ch)       do { (void)(htim); (void)(ch); } while(0)
#define TIM_CCxChannelCmd(TIMx, ch, state)  do { (void)(TIMx); (void)(ch); (void)(state); } while(0)

/* --------------------------------------------------------------------------
 * USART LL: HAL2 renames TXE → TXFE.
 * -------------------------------------------------------------------------- */
#define LL_USART_EnableIT_TXE  LL_USART_EnableIT_TXFE

/* --------------------------------------------------------------------------
 * NVIC priority grouping: HAL2 drops the HAL wrappers, use CMSIS directly.
 * The old NVIC_PRIORITYGROUP_2 = 0x05 maps to 2 bits preemption / 2 bits sub.
 * -------------------------------------------------------------------------- */
#define HAL_NVIC_SetPriorityGrouping  NVIC_SetPriorityGrouping
#define HAL_NVIC_EnableIRQ            NVIC_EnableIRQ
#define HAL_NVIC_DisableIRQ           NVIC_DisableIRQ

// HAL_NVIC_SetPriority(irq, preempt, sub) → NVIC_SetPriority(irq, encoded)
#define HAL_NVIC_SetPriority(irq, preempt, sub) \
    NVIC_SetPriority((irq), NVIC_EncodePriority(NVIC_GetPriorityGrouping(), (preempt), (sub)))

// HAL2 HAL_GPIO_Init has a different signature (3 args). Redirect old 2-arg
// calls to a no-op so shared code compiles. Actual GPIO config uses LL.
static inline void HAL_GPIO_Init_stub_(void *a __attribute__((unused)), void *b __attribute__((unused))) {}
#define HAL_GPIO_Init(gpio, initptr) HAL_GPIO_Init_stub_((void*)(gpio), (void*)(initptr))

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
