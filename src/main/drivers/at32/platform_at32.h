
#if defined(AT32F435ZMT7)

#include "at32f435_437.h"

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;

#define GPIO_TypeDef        gpio_type
#define GPIO_InitTypeDef    gpio_init_type
#define TIM_TypeDef         tmr_type
#define TIM_OCInitTypeDef   tmr_output_config_type
#define DMA_TypeDef         dma_type
#define DMA_InitTypeDef     dma_init_type
#define DMA_Channel_TypeDef dma_channel_type
#define SPI_TypeDef         spi_type
#define ADC_TypeDef         adc_type
#define USART_TypeDef       usart_type
#define TIM_OCInitTypeDef   tmr_output_config_type
#define TIM_ICInitTypeDef   tmr_input_config_type
#define SystemCoreClock     system_core_clock
#define EXTI_TypeDef        exint_type
#define EXTI_InitTypeDef    exint_init_type
#define USART_TypeDef       usart_type

// Chip Unique ID on F43X
#define U_ID_0 (*(uint32_t*)0x1ffff7e8)
#define U_ID_1 (*(uint32_t*)0x1ffff7ec)
#define U_ID_2 (*(uint32_t*)0x1ffff7f0)

#define USE_PIN_AF

#ifndef AT32F4
#define AT32F4
#endif

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#endif

#define USE_TIMER_MGMT
#define USE_DMA_SPEC
#define USE_PERSISTENT_OBJECTS
#define USE_CUSTOM_DEFAULTS_ADDRESS
#define USE_ADC_INTERNAL

#define USE_LATE_TASK_STATISTICS

#define TASK_GYROPID_DESIRED_PERIOD     1000 // 1000us = 1kHz
#define SCHEDULER_DELAY_LIMIT           100

#define DEFAULT_CPU_OVERCLOCK 0
#define FAST_IRQ_HANDLER

#define DMA_DATA_ZERO_INIT          __attribute__ ((section(".dmaram_bss"), aligned(32)))
#define DMA_DATA                    __attribute__ ((section(".dmaram_data"), aligned(32)))
#define STATIC_DMA_DATA_AUTO        static DMA_DATA

#define DMA_RAM
#define DMA_RW_AXI
#define DMA_RAM_R
#define DMA_RAM_W
#define DMA_RAM_RW

#define USE_LATE_TASK_STATISTICS
