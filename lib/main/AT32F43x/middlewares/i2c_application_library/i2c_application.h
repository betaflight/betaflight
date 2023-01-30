/**
  **************************************************************************
  * @file     i2c_application.h
  * @brief    i2c application libray header file
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

/*!< define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_APPLICATION_H
#define __I2C_APPLICATION_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "at32f435_437.h"

/** @addtogroup AT32F435_437_middlewares_i2c_application_library
  * @{
  */


/** @defgroup I2C_library_event_check_flag
  * @{
  */

#define I2C_EVENT_CHECK_NONE             ((uint32_t)0x00000000)    /*!< check flag none */
#define I2C_EVENT_CHECK_ACKFAIL          ((uint32_t)0x00000001)    /*!< check flag ackfail */
#define I2C_EVENT_CHECK_STOP             ((uint32_t)0x00000002)    /*!< check flag stop */

/**
  * @}
  */

/** @defgroup I2C_library_memory_address_width_mode
  * @{
  */

typedef enum
{
  I2C_MEM_ADDR_WIDIH_8                   = 0x01, /*!< memory address is 8 bit */
  I2C_MEM_ADDR_WIDIH_16                  = 0x02, /*!< memory address is 16 bit */
} i2c_mem_address_width_type;

/**
  * @}
  */

/** @defgroup I2C_library_transmission_mode
  * @{
  */

typedef enum
{
  I2C_INT_MA_TX = 0,
  I2C_INT_MA_RX,
  I2C_INT_SLA_TX,
  I2C_INT_SLA_RX,
  I2C_DMA_MA_TX,
  I2C_DMA_MA_RX,
  I2C_DMA_SLA_TX,
  I2C_DMA_SLA_RX,
} i2c_mode_type;

/**
  * @}
  */

/** @defgroup I2C_library_status_code
  * @{
  */

typedef enum
{
  I2C_OK = 0,          /*!< no error */
  I2C_ERR_STEP_1,      /*!< step 1 error */
  I2C_ERR_STEP_2,      /*!< step 2 error */
  I2C_ERR_STEP_3,      /*!< step 3 error */
  I2C_ERR_STEP_4,      /*!< step 4 error */
  I2C_ERR_STEP_5,      /*!< step 5 error */
  I2C_ERR_STEP_6,      /*!< step 6 error */
  I2C_ERR_STEP_7,      /*!< step 7 error */
  I2C_ERR_STEP_8,      /*!< step 8 error */
  I2C_ERR_STEP_9,      /*!< step 9 error */
  I2C_ERR_STEP_10,     /*!< step 10 error */
  I2C_ERR_STEP_11,     /*!< step 11 error */
  I2C_ERR_STEP_12,     /*!< step 12 error */
  I2C_ERR_TCRLD,       /*!< tcrld error */
  I2C_ERR_TDC,         /*!< tdc error */
  I2C_ERR_ADDR,        /*!< addr error */
  I2C_ERR_STOP,        /*!< stop error */
  I2C_ERR_ACKFAIL,     /*!< ackfail error */
  I2C_ERR_TIMEOUT,     /*!< timeout error */
  I2C_ERR_INTERRUPT,   /*!< interrupt error */
} i2c_status_type;

/**
  * @}
  */

/** @defgroup I2C_library_handler
  * @{
  */

typedef struct
{
  i2c_type                               *i2cx;                   /*!< i2c registers base address      */
  uint8_t                                *pbuff;                  /*!< pointer to i2c transfer buffer  */
  __IO uint16_t                          psize;                   /*!< i2c transfer size               */
  __IO uint16_t                          pcount;                  /*!< i2c transfer counter            */
  __IO uint32_t                          mode;                    /*!< i2c communication mode          */
  __IO uint32_t                          status;                  /*!< i2c communication status        */
  __IO i2c_status_type                   error_code;              /*!< i2c error code                  */
  dma_channel_type                       *dma_tx_channel;         /*!< dma transmit channel            */
  dma_channel_type                       *dma_rx_channel;         /*!< dma receive channel             */
  dma_init_type                          dma_init_struct;         /*!< dma init parameters             */
} i2c_handle_type;

/**
  * @}
  */

/** @defgroup I2C_library_exported_functions
  * @{
  */

void            i2c_config                (i2c_handle_type* hi2c);
void            i2c_lowlevel_init         (i2c_handle_type* hi2c);
void            i2c_reset_ctrl2_register  (i2c_handle_type* hi2c);
i2c_status_type i2c_wait_end              (i2c_handle_type* hi2c, uint32_t timeout);
i2c_status_type i2c_wait_flag             (i2c_handle_type* hi2c, uint32_t flag, uint32_t event_check, uint32_t timeout);

i2c_status_type i2c_master_transmit       (i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_master_receive        (i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_slave_transmit        (i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_slave_receive         (i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout);

i2c_status_type i2c_master_transmit_int   (i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_master_receive_int    (i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_slave_transmit_int    (i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_slave_receive_int     (i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout);

i2c_status_type i2c_master_transmit_dma   (i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_master_receive_dma    (i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_slave_transmit_dma    (i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_slave_receive_dma     (i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout);

i2c_status_type i2c_smbus_master_transmit (i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_smbus_master_receive  (i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_smbus_slave_transmit  (i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_smbus_slave_receive   (i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout);

i2c_status_type i2c_memory_write          (i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_memory_write_int      (i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_memory_write_dma      (i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_memory_read           (i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_memory_read_int       (i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout);
i2c_status_type i2c_memory_read_dma       (i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout);

void            i2c_evt_irq_handler       (i2c_handle_type* hi2c);
void            i2c_err_irq_handler       (i2c_handle_type* hi2c);
void            i2c_dma_tx_irq_handler    (i2c_handle_type* hi2c);
void            i2c_dma_rx_irq_handler    (i2c_handle_type* hi2c);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif
