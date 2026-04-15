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
 * TIM: channel byte offsets used by Betaflight's timer infrastructure.
 * HAL2 removed these #defines (uses enums instead).
 * -------------------------------------------------------------------------- */
#define TIM_CHANNEL_1  0x0000U
#define TIM_CHANNEL_2  0x0004U
#define TIM_CHANNEL_3  0x0008U
#define TIM_CHANNEL_4  0x000CU

/* Interrupt enable bits (old HAL TIM_IT_ names → DIER register bits) */
#define TIM_IT_UPDATE  TIM_DIER_UIE
#define TIM_IT_CC1     TIM_DIER_CC1IE
#define TIM_IT_CC2     TIM_DIER_CC2IE
#define TIM_IT_CC3     TIM_DIER_CC3IE
#define TIM_IT_CC4     TIM_DIER_CC4IE

/* DMA enable bits */
#define TIM_DMA_CC1    TIM_DIER_CC1DE
#define TIM_DMA_CC2    TIM_DIER_CC2DE
#define TIM_DMA_CC3    TIM_DIER_CC3DE
#define TIM_DMA_CC4    TIM_DIER_CC4DE

/* --------------------------------------------------------------------------
 * TIM: HAL2 removed HAL TIM init structs. Stub them for struct declarations.
 * -------------------------------------------------------------------------- */
typedef struct {
    TIM_TypeDef *Instance;
    struct {
        uint32_t Prescaler;
        uint32_t CounterMode;
        uint32_t Period;
        uint32_t ClockDivision;
        uint32_t RepetitionCounter;
    } Init;
} TIM_HandleTypeDef;
#define TIM_COUNTERMODE_UP  LL_TIM_COUNTERMODE_UP
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
 * USB PCD: HAL2 renames PCD_HandleTypeDef → hal_pcd_handle_t and changes
 * field names. The USB Device Library (usbd_cdc.c) still uses old names.
 * Include the HAL2 PCD header and typedef + map the old field names.
 * -------------------------------------------------------------------------- */
#include "stm32c5xx_hal_pcd.h"
typedef hal_pcd_handle_t PCD_HandleTypeDef;
/* HAL1 field → HAL2 field mappings for PCD handle access in USB library */
#define IN_ep       in_ep
#define OUT_ep      out_ep
#define maxpacket   max_packet
typedef struct { ADC_TypeDef *Instance; } ADC_HandleTypeDef;

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
 * LL DMA types: HAL2 removed LL_DMA_InitTypeDef / LL_DMA_Init. Provide
 * a struct with the fields used by bus_spi_hal2.c for DMA staging.
 * -------------------------------------------------------------------------- */
typedef struct {
    uint32_t SrcAddress;
    uint32_t DestAddress;
    uint32_t Direction;
    uint32_t SrcIncMode;
    uint32_t DestIncMode;
    uint32_t SrcDataWidth;
    uint32_t DestDataWidth;
    uint32_t BlkDataLength;
    uint32_t Request;
    uint32_t Priority;
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

/* Note: HAL2 TIM API is incompatible with old HAL_TIM_Base_Init/Start_IT/IRQHandler.
 * CDC interface timer handling uses LL TIM directly for C5 — see usbd_cdc_interface.c. */

/* --------------------------------------------------------------------------
 * ErrorStatus: HAL2 removed this CMSIS typedef.  LL_USART_Init returns it.
 * -------------------------------------------------------------------------- */
#ifndef __ERRORSTATUS_DEFINED
#define __ERRORSTATUS_DEFINED
typedef enum { ERROR = 0, SUCCESS = !ERROR } ErrorStatus;
#endif

/* --------------------------------------------------------------------------
 * USART LL: HAL2 renames TXE/RXNE flag and interrupt functions to
 * combined FIFO-aware names. Map old names used by serial_uart_ll.c.
 * -------------------------------------------------------------------------- */
#define LL_USART_EnableIT_TXE       LL_USART_EnableIT_TXE_TXFNF
#define LL_USART_IsEnabledIT_TXE    LL_USART_IsEnabledIT_TXE_TXFNF
#define LL_USART_IsActiveFlag_TXE   LL_USART_IsActiveFlag_TXE_TXFNF
#define LL_USART_IsEnabledIT_RXNE   LL_USART_IsEnabledIT_RXNE_RXFNE
#define LL_USART_IsActiveFlag_RXNE  LL_USART_IsActiveFlag_RXNE_RXFNE

/* USART constant renames */
#define LL_USART_DATAWIDTH_8B       LL_USART_DATAWIDTH_8_BIT
#define LL_USART_DATAWIDTH_9B       LL_USART_DATAWIDTH_9_BIT
#define LL_USART_STOPBITS_1         LL_USART_STOP_BIT_1
#define LL_USART_STOPBITS_2         LL_USART_STOP_BIT_2

/* --------------------------------------------------------------------------
 * USART LL: HAL2 removed LL_USART_InitTypeDef / Init / StructInit / DeInit.
 * Provide lightweight implementations using the individual LL setters.
 * -------------------------------------------------------------------------- */
typedef struct {
    uint32_t BaudRate;
    uint32_t DataWidth;
    uint32_t StopBits;
    uint32_t Parity;
    uint32_t TransferDirection;
    uint32_t HardwareFlowControl;
    uint32_t OverSampling;
    uint32_t PrescalerValue;
} LL_USART_InitTypeDef;

static inline void LL_USART_StructInit(LL_USART_InitTypeDef *init)
{
    init->BaudRate            = 9600U;
    init->DataWidth           = LL_USART_DATAWIDTH_8_BIT;
    init->StopBits            = LL_USART_STOP_BIT_1;
    init->Parity              = LL_USART_PARITY_NONE;
    init->TransferDirection   = LL_USART_DIRECTION_TX_RX;
    init->HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    init->OverSampling        = LL_USART_OVERSAMPLING_16;
    init->PrescalerValue      = LL_USART_PRESCALER_DIV1;
}

static inline ErrorStatus LL_USART_Init(USART_TypeDef *USARTx,
                                        const LL_USART_InitTypeDef *init)
{
    LL_USART_SetDataWidth(USARTx, init->DataWidth);
    LL_USART_SetParity(USARTx, init->Parity);
    LL_USART_SetStopBitsLength(USARTx, init->StopBits);
    LL_USART_SetTransferDirection(USARTx, init->TransferDirection);
    LL_USART_SetHWFlowCtrl(USARTx, init->HardwareFlowControl);
    LL_USART_SetOverSampling(USARTx, init->OverSampling);
    LL_USART_SetPrescaler(USARTx, init->PrescalerValue);

    uint32_t periphclk = ((uintptr_t)USARTx >= APB2PERIPH_BASE)
                         ? HAL_RCC_GetPCLK2Freq()
                         : HAL_RCC_GetPCLK1Freq();
    LL_USART_SetBaudRate(USARTx, periphclk, init->PrescalerValue,
                         init->OverSampling, init->BaudRate);

    return (USARTx->BRR != 0U) ? SUCCESS : ERROR;
}

static inline void LL_USART_DeInit(USART_TypeDef *USARTx)
{
    USARTx->CR1 = 0U;
    USARTx->CR2 = 0U;
    USARTx->CR3 = 0U;
    USARTx->BRR = 0U;
}

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

/* --------------------------------------------------------------------------
 * HAL RCC GPIO/USB clock: HAL2 renames macro-style to function-style.
 * -------------------------------------------------------------------------- */
#define __HAL_RCC_GPIOA_CLK_ENABLE()  HAL_RCC_GPIOA_EnableClock()
#define __HAL_RCC_USB_CLK_ENABLE()    HAL_RCC_USB_EnableClock()
#define __HAL_RCC_USB_CLK_DISABLE()   HAL_RCC_USB_DisableClock()
#define __HAL_RCC_TIM7_CLK_ENABLE()   HAL_RCC_TIM7_EnableClock()

/* --------------------------------------------------------------------------
 * ADC calibration constants: map generic names to C5 LL_ADC_* addresses.
 * -------------------------------------------------------------------------- */
#define VREFINT_CAL_ADDR            ((const uint16_t *)(0x08FFF810UL))
#define VREFINT_CAL_VREF            (3300UL)
#define TEMPSENSOR_CAL1_ADDR        ((const uint16_t *)(0x08FFF814UL))
#define TEMPSENSOR_CAL2_ADDR        ((const uint16_t *)(0x08FFF818UL))
#define TEMPSENSOR_CAL1_TEMP        ((int32_t) 30)
#define TEMPSENSOR_CAL2_TEMP        ((int32_t)140)
#define TEMPSENSOR_CAL_VREFANALOG   (3300UL)

/* --------------------------------------------------------------------------
 * IRQ names: HAL2 renames some timer update IRQ vectors.
 * -------------------------------------------------------------------------- */
#define TIM1_UP_IRQn   TIM1_UPD_IRQn
#define TIM8_UP_IRQn   TIM8_UPD_IRQn

/* --------------------------------------------------------------------------
 * GPIO AF: HAL2 renames GPIO_AFx_TIMy → HAL_GPIO_AFx_TIMy.
 * -------------------------------------------------------------------------- */
#define GPIO_AF1_TIM1    HAL_GPIO_AF1_TIM1
#define GPIO_AF1_TIM2    HAL_GPIO_AF1_TIM2
#define GPIO_AF1_TIM17   HAL_GPIO_AF1_TIM17
#define GPIO_AF2_TIM1    HAL_GPIO_AF2_TIM1
#define GPIO_AF2_TIM3    HAL_GPIO_AF2_TIM3
#define GPIO_AF2_TIM4    HAL_GPIO_AF2_TIM4
#define GPIO_AF2_TIM5    HAL_GPIO_AF2_TIM5
#define GPIO_AF2_TIM8    HAL_GPIO_AF2_TIM8
#define GPIO_AF2_TIM12   HAL_GPIO_AF2_TIM12
#define GPIO_AF2_TIM15   HAL_GPIO_AF2_TIM15
#define GPIO_AF3_TIM1    HAL_GPIO_AF3_TIM1
#define GPIO_AF3_TIM5    HAL_GPIO_AF3_TIM5
#define GPIO_AF3_TIM8    HAL_GPIO_AF3_TIM8
#define GPIO_AF4_TIM15   HAL_GPIO_AF4_TIM15
#define GPIO_AF8_TIM5    HAL_GPIO_AF8_TIM5
#define GPIO_AF10_TIM16  HAL_GPIO_AF10_TIM16
#define GPIO_AF13_TIM8   HAL_GPIO_AF13_TIM8
#define GPIO_AF14_TIM2   HAL_GPIO_AF14_TIM2

/* --------------------------------------------------------------------------
 * DMA: C5 has LPDMA (not GPDMA). Map H5 GPDMA names to C5 LPDMA equivalents
 * so the shared timer_def.h compiles for both.  HAL2 also renames CH→CC and
 * UP→UPD in the DMA request constants.
 * -------------------------------------------------------------------------- */

/* DMA channel instances: GPDMA1_Channelx → LPDMA1_CHx */
#define GPDMA1_Channel0  LPDMA1_CH0
#define GPDMA1_Channel1  LPDMA1_CH1
#define GPDMA1_Channel2  LPDMA1_CH2
#define GPDMA1_Channel3  LPDMA1_CH3
#define GPDMA1_Channel4  LPDMA1_CH4
#define GPDMA1_Channel5  LPDMA1_CH5
#define GPDMA1_Channel6  LPDMA1_CH6
#define GPDMA1_Channel7  LPDMA1_CH7

/* DMA channel IRQ handlers */
#define GPDMA1_CH0_HANDLER  LPDMA1_CH0_HANDLER
#define GPDMA1_CH1_HANDLER  LPDMA1_CH1_HANDLER
#define GPDMA1_CH2_HANDLER  LPDMA1_CH2_HANDLER
#define GPDMA1_CH3_HANDLER  LPDMA1_CH3_HANDLER
#define GPDMA1_CH4_HANDLER  LPDMA1_CH4_HANDLER
#define GPDMA1_CH5_HANDLER  LPDMA1_CH5_HANDLER
#define GPDMA1_CH6_HANDLER  LPDMA1_CH6_HANDLER
#define GPDMA1_CH7_HANDLER  LPDMA1_CH7_HANDLER

/* DMA timer request IDs: GPDMA1→LPDMA1, CHx→CCx, UP→UPD */
#define LL_GPDMA1_REQUEST_TIM1_CH1   LL_LPDMA1_REQUEST_TIM1_CC1
#define LL_GPDMA1_REQUEST_TIM1_CH2   LL_LPDMA1_REQUEST_TIM1_CC2
#define LL_GPDMA1_REQUEST_TIM1_CH3   LL_LPDMA1_REQUEST_TIM1_CC3
#define LL_GPDMA1_REQUEST_TIM1_CH4   LL_LPDMA1_REQUEST_TIM1_CC4
#define LL_GPDMA1_REQUEST_TIM1_UP    LL_LPDMA1_REQUEST_TIM1_UPD
#define LL_GPDMA1_REQUEST_TIM2_CH1   LL_LPDMA1_REQUEST_TIM2_CC1
#define LL_GPDMA1_REQUEST_TIM2_CH2   LL_LPDMA1_REQUEST_TIM2_CC2
#define LL_GPDMA1_REQUEST_TIM2_CH3   LL_LPDMA1_REQUEST_TIM2_CC3
#define LL_GPDMA1_REQUEST_TIM2_CH4   LL_LPDMA1_REQUEST_TIM2_CC4
#define LL_GPDMA1_REQUEST_TIM2_UP    LL_LPDMA1_REQUEST_TIM2_UPD
#define LL_GPDMA1_REQUEST_TIM3_CH1   LL_LPDMA1_REQUEST_TIM3_CC1
#define LL_GPDMA1_REQUEST_TIM3_CH2   LL_LPDMA1_REQUEST_TIM3_CC2
#define LL_GPDMA1_REQUEST_TIM3_CH3   LL_LPDMA1_REQUEST_TIM3_CC3
#define LL_GPDMA1_REQUEST_TIM3_CH4   LL_LPDMA1_REQUEST_TIM3_CC4
#define LL_GPDMA1_REQUEST_TIM3_UP    LL_LPDMA1_REQUEST_TIM3_UPD
#define LL_GPDMA1_REQUEST_TIM4_CH1   LL_LPDMA1_REQUEST_TIM4_CC1
#define LL_GPDMA1_REQUEST_TIM4_CH2   LL_LPDMA1_REQUEST_TIM4_CC2
#define LL_GPDMA1_REQUEST_TIM4_CH3   LL_LPDMA1_REQUEST_TIM4_CC3
#define LL_GPDMA1_REQUEST_TIM4_CH4   LL_LPDMA1_REQUEST_TIM4_CC4
#define LL_GPDMA1_REQUEST_TIM4_UP    LL_LPDMA1_REQUEST_TIM4_UPD
#define LL_GPDMA1_REQUEST_TIM5_CH1   LL_LPDMA1_REQUEST_TIM5_CC1
#define LL_GPDMA1_REQUEST_TIM5_CH2   LL_LPDMA1_REQUEST_TIM5_CC2
#define LL_GPDMA1_REQUEST_TIM5_CH3   LL_LPDMA1_REQUEST_TIM5_CC3
#define LL_GPDMA1_REQUEST_TIM5_CH4   LL_LPDMA1_REQUEST_TIM5_CC4
#define LL_GPDMA1_REQUEST_TIM5_UP    LL_LPDMA1_REQUEST_TIM5_UPD
#define LL_GPDMA1_REQUEST_TIM6_UP    LL_LPDMA1_REQUEST_TIM6_UPD
#define LL_GPDMA1_REQUEST_TIM7_UP    LL_LPDMA1_REQUEST_TIM7_UPD
#define LL_GPDMA1_REQUEST_TIM8_CH1   LL_LPDMA1_REQUEST_TIM8_CC1
#define LL_GPDMA1_REQUEST_TIM8_CH2   LL_LPDMA1_REQUEST_TIM8_CC2
#define LL_GPDMA1_REQUEST_TIM8_CH3   LL_LPDMA1_REQUEST_TIM8_CC3
#define LL_GPDMA1_REQUEST_TIM8_CH4   LL_LPDMA1_REQUEST_TIM8_CC4
#define LL_GPDMA1_REQUEST_TIM8_UP    LL_LPDMA1_REQUEST_TIM8_UPD
#define LL_GPDMA1_REQUEST_TIM15_CH1  LL_LPDMA1_REQUEST_TIM15_CC1
#define LL_GPDMA1_REQUEST_TIM15_UP   LL_LPDMA1_REQUEST_TIM15_UPD
#define LL_GPDMA1_REQUEST_TIM16_CH1  LL_LPDMA1_REQUEST_TIM16_CC1
#define LL_GPDMA1_REQUEST_TIM16_UP   LL_LPDMA1_REQUEST_TIM16_UPD
#define LL_GPDMA1_REQUEST_TIM17_CH1  LL_LPDMA1_REQUEST_TIM17_CC1
#define LL_GPDMA1_REQUEST_TIM17_UP   LL_LPDMA1_REQUEST_TIM17_UPD

/* --------------------------------------------------------------------------
 * SPI LL: HAL2 renames many LL_SPI constants.
 * -------------------------------------------------------------------------- */
#define LL_SPI_BAUDRATEPRESCALER_DIV2    LL_SPI_BAUD_RATE_PRESCALER_2
#define LL_SPI_BAUDRATEPRESCALER_DIV4    LL_SPI_BAUD_RATE_PRESCALER_4
#define LL_SPI_BAUDRATEPRESCALER_DIV8    LL_SPI_BAUD_RATE_PRESCALER_8
#define LL_SPI_BAUDRATEPRESCALER_DIV16   LL_SPI_BAUD_RATE_PRESCALER_16
#define LL_SPI_BAUDRATEPRESCALER_DIV32   LL_SPI_BAUD_RATE_PRESCALER_32
#define LL_SPI_BAUDRATEPRESCALER_DIV64   LL_SPI_BAUD_RATE_PRESCALER_64
#define LL_SPI_BAUDRATEPRESCALER_DIV128  LL_SPI_BAUD_RATE_PRESCALER_128
#define LL_SPI_BAUDRATEPRESCALER_DIV256  LL_SPI_BAUD_RATE_PRESCALER_256

#define LL_SPI_DATAWIDTH_8BIT            LL_SPI_DATA_WIDTH_8_BIT

#define LL_SPI_PHASE_1EDGE               LL_SPI_CLOCK_PHASE_1_EDGE
#define LL_SPI_PHASE_2EDGE               LL_SPI_CLOCK_PHASE_2_EDGE
#define LL_SPI_POLARITY_LOW              LL_SPI_CLOCK_POLARITY_LOW
#define LL_SPI_POLARITY_HIGH             LL_SPI_CLOCK_POLARITY_HIGH

#define LL_SPI_FIFO_TH_01DATA           LL_SPI_FIFO_THRESHOLD_1_DATA

#define LL_SPI_CRCCALCULATION_DISABLE    0x00000000U

/* SPI LL: HAL2 renamed LL_SPI_SetBitOrder → LL_SPI_SetTransferBitOrder */
#define LL_SPI_SetBitOrder               LL_SPI_SetTransferBitOrder

/* SPI LL: HAL2 removed LL_SPI_InitTypeDef / Init / DeInit. Provide
 * lightweight implementations using the individual LL setters. */
typedef struct {
    uint32_t TransferDirection;
    uint32_t Mode;
    uint32_t DataWidth;
    uint32_t ClockPolarity;
    uint32_t ClockPhase;
    uint32_t NSS;
    uint32_t BaudRate;
    uint32_t BitOrder;
    uint32_t CRCCalculation;
    uint32_t CRCPoly;
} LL_SPI_InitTypeDef;

static inline ErrorStatus LL_SPI_Init(SPI_TypeDef *SPIx,
                                      const LL_SPI_InitTypeDef *init)
{
    LL_SPI_SetTransferDirection(SPIx, init->TransferDirection);
    LL_SPI_SetMode(SPIx, init->Mode);
    LL_SPI_SetDataWidth(SPIx, init->DataWidth);
    LL_SPI_SetClockPolarity(SPIx, init->ClockPolarity);
    LL_SPI_SetClockPhase(SPIx, init->ClockPhase);
    LL_SPI_SetNSSMode(SPIx, init->NSS);
    LL_SPI_SetBaudRatePrescaler(SPIx, init->BaudRate);
    LL_SPI_SetTransferBitOrder(SPIx, init->BitOrder);
    if (init->CRCCalculation) {
        LL_SPI_EnableCRC(SPIx);
    }
    return SUCCESS;
}

static inline void LL_SPI_DeInit(SPI_TypeDef *SPIx)
{
    SPIx->CR1  = 0U;
    SPIx->CFG1 = 0x00070007U;  /* reset value */
    SPIx->CFG2 = 0U;
    SPIx->IER  = 0U;
    SPIx->IFCR = 0xFFFFFFFFU;  /* clear all flags */
}

/* --------------------------------------------------------------------------
 * GPIO AF: HAL2 renames GPIO_AFx_SPIy → HAL_GPIO_AFx_SPIy.
 * -------------------------------------------------------------------------- */
/* USB AF: C5 uses AF13 for USB (not AF10 like H5) */
#define GPIO_AF10_USB    HAL_GPIO_AF13_USB

#define GPIO_AF5_SPI1    HAL_GPIO_AF5_SPI1
#define GPIO_AF5_SPI2    HAL_GPIO_AF5_SPI2
#define GPIO_AF5_SPI3    HAL_GPIO_AF5_SPI3
#define GPIO_AF6_SPI3    HAL_GPIO_AF6_SPI3

/* --------------------------------------------------------------------------
 * LL_TIM_DeInit: not available in HAL2 LL. Use HAL version.
 * -------------------------------------------------------------------------- */
#define LL_TIM_DeInit(tim) do { (void)(tim); } while(0)

/* --------------------------------------------------------------------------
 * I2C: IRQ name compat — C5 uses I2Cx_ERR_IRQn, Betaflight expects I2Cx_ER_IRQn.
 * -------------------------------------------------------------------------- */
#define I2C1_ER_IRQn  I2C1_ERR_IRQn
#define I2C2_ER_IRQn  I2C2_ERR_IRQn

/* I2C: GPIO AF compat */
#define GPIO_AF4_I2C1  HAL_GPIO_AF4_I2C1
#define GPIO_AF4_I2C2  HAL_GPIO_AF4_I2C2

/* --------------------------------------------------------------------------
 * LL I2C: HAL2 removed LL_I2C_InitTypeDef / Init / DeInit / StructInit.
 * Provide inline implementations using individual LL setters.
 * -------------------------------------------------------------------------- */
typedef struct {
    uint32_t PeripheralMode;
    uint32_t Timing;
    uint32_t AnalogFilter;
    uint32_t DigitalFilter;
    uint32_t OwnAddress1;
    uint32_t TypeAcknowledge;
    uint32_t OwnAddrSize;
} LL_I2C_InitTypeDef;

static inline void LL_I2C_StructInit(LL_I2C_InitTypeDef *init)
{
    init->PeripheralMode  = LL_I2C_MODE_I2C;
    init->Timing          = 0U;
    init->AnalogFilter    = LL_I2C_ANALOGFILTER_ENABLE;
    init->DigitalFilter   = 0U;
    init->OwnAddress1     = 0U;
    init->TypeAcknowledge = LL_I2C_ACK;
    init->OwnAddrSize     = LL_I2C_OWNADDRESS1_7BIT;
}

static inline ErrorStatus LL_I2C_Init(I2C_TypeDef *I2Cx,
                                      const LL_I2C_InitTypeDef *init)
{
    LL_I2C_Disable(I2Cx);
    LL_I2C_SetMode(I2Cx, init->PeripheralMode);
    LL_I2C_SetTiming(I2Cx, init->Timing);
    if (init->AnalogFilter == LL_I2C_ANALOGFILTER_ENABLE) {
        LL_I2C_EnableAnalogFilter(I2Cx);
    } else {
        LL_I2C_DisableAnalogFilter(I2Cx);
    }
    LL_I2C_SetDigitalFilter(I2Cx, init->DigitalFilter);
    LL_I2C_SetOwnAddress1(I2Cx, init->OwnAddress1, init->OwnAddrSize);
    LL_I2C_AcknowledgeNextData(I2Cx, init->TypeAcknowledge);
    return SUCCESS;
}

static inline ErrorStatus LL_I2C_DeInit(I2C_TypeDef *I2Cx)
{
    LL_I2C_Disable(I2Cx);
    I2Cx->CR1    = 0U;
    I2Cx->CR2    = 0U;
    I2Cx->OAR1   = 0U;
    I2Cx->OAR2   = 0U;
    I2Cx->TIMINGR = 0U;
    return SUCCESS;
}

#endif /* STM32C5XX_HAL2_COMPAT_H */
