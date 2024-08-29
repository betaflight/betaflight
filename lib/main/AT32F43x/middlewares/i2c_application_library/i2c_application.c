/**
  **************************************************************************
  * @file     i2c_application.c
  * @brief    the driver library of the i2c peripheral
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

#include "i2c_application.h"

/** @addtogroup AT32F435_437_middlewares_i2c_application_library
  * @{
  */

/**
  * @brief get the dma transfer direction flag through the channel
  */
#define DMA_GET_REQUEST(DMA_CHANNEL) \
(((uint32_t)(DMA_CHANNEL) == ((uint32_t)hi2c->dma_tx_channel)) ? I2C_DMA_REQUEST_TX : I2C_DMA_REQUEST_RX)

/**
  * @brief get the dma transfer complete flag through the channel
  */
#define DMA_GET_TC_FLAG(DMA_CHANNEL) \
(((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL1))? DMA1_FDT1_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL2))? DMA1_FDT2_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL3))? DMA1_FDT3_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL4))? DMA1_FDT4_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL5))? DMA1_FDT5_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL6))? DMA1_FDT6_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL7))? DMA1_FDT7_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL1))? DMA2_FDT1_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL2))? DMA2_FDT2_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL3))? DMA2_FDT3_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL4))? DMA2_FDT4_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL5))? DMA2_FDT5_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL6))? DMA2_FDT6_FLAG : \
                                                         DMA2_FDT7_FLAG)

/**
  * @brief get the dma half transfer flag through the channel
  */
#define DMA_GET_HT_FLAG(DMA_CHANNEL) \
(((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL1))? DMA1_HDT1_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL2))? DMA1_HDT2_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL3))? DMA1_HDT3_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL4))? DMA1_HDT4_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL5))? DMA1_HDT5_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL6))? DMA1_HDT6_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL7))? DMA1_HDT7_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL1))? DMA2_HDT1_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL2))? DMA2_HDT2_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL3))? DMA2_HDT3_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL4))? DMA2_HDT4_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL5))? DMA2_HDT5_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL6))? DMA2_HDT6_FLAG : \
                                                         DMA2_HDT7_FLAG)

/**
  * @brief get the dma transfer error flag through the channel
  */
#define DMA_GET_TERR_FLAG(DMA_CHANNEL) \
(((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL1))? DMA1_DTERR1_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL2))? DMA1_DTERR2_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL3))? DMA1_DTERR3_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL4))? DMA1_DTERR4_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL5))? DMA1_DTERR5_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL6))? DMA1_DTERR6_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL7))? DMA1_DTERR7_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL1))? DMA2_DTERR1_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL2))? DMA2_DTERR2_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL3))? DMA2_DTERR3_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL4))? DMA2_DTERR4_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL5))? DMA2_DTERR5_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL6))? DMA2_DTERR6_FLAG : \
                                                         DMA2_DTERR7_FLAG)

/**
  * @brief  initializes peripherals used by the i2c.
  * @param  none
  * @retval none
  */
__WEAK void i2c_lowlevel_init(i2c_handle_type* hi2c)
{
    UNUSED(hi2c);
}

/**
  * @brief  i2c peripheral initialization.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_config(i2c_handle_type* hi2c)
{
  /* reset i2c peripheral */
  i2c_reset(hi2c->i2cx);

  /* i2c peripheral initialization */
  i2c_lowlevel_init(hi2c);

  /* i2c peripheral enable */
  i2c_enable(hi2c->i2cx, TRUE);
}

/**
  * @brief  refresh i2c register.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_refresh_txdt_register(i2c_handle_type* hi2c)
{
  /* clear tdis flag */
  if (i2c_flag_get(hi2c->i2cx, I2C_TDIS_FLAG) != RESET)
  {
    hi2c->i2cx->txdt = 0x00;
  }

  /* refresh txdt register*/
  if (i2c_flag_get(hi2c->i2cx, I2C_TDBE_FLAG) == RESET)
  {
    hi2c->i2cx->sts_bit.tdbe = 1;
  }
}

/**
  * @brief  reset ctrl2 register.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_reset_ctrl2_register(i2c_handle_type* hi2c)
{
  hi2c->i2cx->ctrl2_bit.saddr   = 0;
  hi2c->i2cx->ctrl2_bit.readh10 = 0;
  hi2c->i2cx->ctrl2_bit.cnt     = 0;
  hi2c->i2cx->ctrl2_bit.rlden   = 0;
  hi2c->i2cx->ctrl2_bit.dir     = 0;
}

/**
  * @brief  wait for the flag to be set or reset, only BUSYF flag
  *         is waiting to be reset, and other flags are waiting to be set
  * @param  hi2c: the handle points to the operation information.
  * @param  flag: specifies the flag to check.
  *         this parameter can be one of the following values:
  *         - I2C_TDBE_FLAG: transmit data buffer empty flag.
  *         - I2C_TDIS_FLAG: send interrupt status.
  *         - I2C_RDBF_FLAG: receive data buffer full flag.
  *         - I2C_ADDRF_FLAG: 0~7 bit address match flag.
  *         - I2C_ACKFAIL_FLAG: acknowledge failure flag.
  *         - I2C_STOPF_FLAG: stop condition generation complete flag.
  *         - I2C_TDC_FLAG: transmit data complete flag.
  *         - I2C_TCRLD_FLAG: transmission is complete, waiting to load data.
  *         - I2C_BUSERR_FLAG: bus error flag.
  *         - I2C_ARLOST_FLAG: arbitration lost flag.
  *         - I2C_OUF_FLAG: overflow or underflow flag.
  *         - I2C_PECERR_FLAG: pec receive error flag.
  *         - I2C_TMOUT_FLAG: smbus timeout flag.
  *         - I2C_ALERTF_FLAG: smbus alert flag.
  *         - I2C_BUSYF_FLAG: bus busy flag transmission mode.
  *         - I2C_SDIR_FLAG: slave data transmit direction.
  * @param  event_check: check other error flags while waiting for the flag.
  *         parameter as following values:
  *         - I2C_EVENT_CHECK_NONE
  *         - I2C_EVENT_CHECK_ACKFAIL
  *         - I2C_EVENT_CHECK_STOP
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_wait_flag(i2c_handle_type* hi2c, uint32_t flag, uint32_t event_check, uint32_t timeout)
{
    hi2c->error_code = I2C_OK;

  if(flag == I2C_BUSYF_FLAG)
  {
    while(i2c_flag_get(hi2c->i2cx, flag) != RESET)
    {
      /* check timeout */
      if((timeout--) == 0)
      {
        hi2c->error_code = I2C_ERR_TIMEOUT;
        return hi2c->error_code;
      }
    }
  }
  else
  {
    while(i2c_flag_get(hi2c->i2cx, flag) == RESET)
    {
#if 1
        UNUSED(event_check);
#else
      /* check the ack fail flag */
      if(event_check & I2C_EVENT_CHECK_ACKFAIL)
      {
        if(i2c_flag_get(hi2c->i2cx, I2C_ACKFAIL_FLAG) != RESET)
        {
          /* clear ack fail flag */
          i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);

          hi2c->error_code = I2C_ERR_ACKFAIL;
          return hi2c->error_code;
        }
      }

      /* check the stop flag */
      if(event_check & I2C_EVENT_CHECK_STOP)
      {
        if(i2c_flag_get(hi2c->i2cx, I2C_STOPF_FLAG) != RESET)
        {
          /* clear stop flag */
          i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

          i2c_reset_ctrl2_register(hi2c);

          hi2c->error_code = I2C_ERR_STOP;
          return hi2c->error_code;
        }
      }
#endif

      /* check timeout */
      if((timeout--) == 0)
      {
        hi2c->error_code = I2C_ERR_TIMEOUT;
        return hi2c->error_code;
      }
    }
  }

  return hi2c->error_code;
}

// Initialise the data buffer
static void i2c_set_buffer(i2c_handle_type* hi2c, i2cStep_t step, uint8_t *buf, uint16_t len)
{
	  hi2c->step = step;
	  hi2c->pbuff[step] = buf;
	  hi2c->pcount[step] = len;
}

/**
  * @brief  dma transfer cofiguration.
  * @param  hi2c: the handle points to the operation information.
  * @param  dma_channelx: dma channel to be cofigured.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @retval none.
  */
void i2c_dma_config(i2c_handle_type* hi2c, dma_channel_type* dma_channel, uint8_t* pdata, uint16_t size)
{
  /* disable the dma channel */
  dma_channel_enable(dma_channel, FALSE);

  /* disable the transfer complete interrupt */
  dma_interrupt_enable(dma_channel, DMA_FDT_INT, FALSE);

  /* configure the dma channel with the buffer address and the buffer size */
  hi2c->dma_init_struct.memory_base_addr     = (uint32_t)pdata;
  hi2c->dma_init_struct.direction            = (dma_channel == hi2c->dma_tx_channel) ? DMA_DIR_MEMORY_TO_PERIPHERAL : DMA_DIR_PERIPHERAL_TO_MEMORY;
  hi2c->dma_init_struct.peripheral_base_addr = (dma_channel == hi2c->dma_tx_channel) ? (uint32_t)&hi2c->i2cx->txdt : (uint32_t)&hi2c->i2cx->rxdt;
  hi2c->dma_init_struct.buffer_size          = (uint32_t)size;
  dma_init(dma_channel, &hi2c->dma_init_struct);

  /* enable the transfer complete interrupt */
  dma_interrupt_enable(dma_channel, DMA_FDT_INT, TRUE);

  /* enable the dma channel */
  dma_channel_enable(dma_channel, TRUE);
}

/**
  * @brief  start transfer in poll mode or interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  start: config gen start condition mode.
  *         parameter as following values:
  *         - I2C_WITHOUT_START: transfer data without start condition.
  *         - I2C_GEN_START_READ: read data and generate start.
  *         - I2C_GEN_START_WRITE: send data and generate start.
  * @retval i2c status.
  */
void i2c_start_transfer(i2c_handle_type* hi2c, uint16_t address, i2c_start_mode_type start)
{
  uint16_t totalLen = hi2c->pcount[hi2c->step];

  // Writes are done in a continuous block
  if ((hi2c->step == I2C_STEP_REG) && (hi2c->mode == I2C_INT_MA_TX)) {
    totalLen = hi2c->pcount[I2C_STEP_REG] + hi2c->pcount[I2C_STEP_DATA];
  }

  if (totalLen > MAX_TRANSFER_CNT)
  {
    hi2c->psize = MAX_TRANSFER_CNT;

    i2c_transmit_set(hi2c->i2cx, address, hi2c->psize, I2C_RELOAD_MODE, start);
  }
  else
  {
    hi2c->psize = totalLen;

    if ((hi2c->step == I2C_STEP_DATA) && (hi2c->mode == I2C_INT_MA_TX)) {
      // Don't send a stop as a restart will be necessary to advance to I2C_STEP_DATA
      i2c_transmit_set(hi2c->i2cx, address, hi2c->psize, I2C_RELOAD_MODE, start);
    } else {
      if ((hi2c->mode == I2C_INT_MA_RX) || (hi2c->mode == I2C_INT_MA_TX)) {
        i2c_transmit_set(hi2c->i2cx, address, hi2c->psize, I2C_SOFT_STOP_MODE, start);
      } else {
        i2c_transmit_set(hi2c->i2cx, address, hi2c->psize, I2C_AUTO_STOP_MODE, start);
      }
    }
  }
}

/**
  * @brief  start transfer in dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  start: config gen start condition mode.
  *         parameter as following values:
  *         - I2C_WITHOUT_START: transfer data without start condition.
  *         - I2C_GEN_START_READ: read data and generate start.
  *         - I2C_GEN_START_WRITE: send data and generate start.
  * @retval i2c status.
  */
void i2c_start_transfer_dma(i2c_handle_type* hi2c, dma_channel_type* dma_channelx, uint16_t address, i2c_start_mode_type start)
{
  if (hi2c->pcount[hi2c->step] > MAX_TRANSFER_CNT)
  {
    hi2c->psize = MAX_TRANSFER_CNT;

    /* config dma */
    i2c_dma_config(hi2c, dma_channelx, hi2c->pbuff[hi2c->step], hi2c->psize);

    i2c_transmit_set(hi2c->i2cx, address, hi2c->psize, I2C_RELOAD_MODE, start);
  }
  else
  {
    hi2c->psize = hi2c->pcount[hi2c->step];

    /* config dma */
    i2c_dma_config(hi2c, dma_channelx, hi2c->pbuff[hi2c->step], hi2c->psize);

    i2c_transmit_set(hi2c->i2cx, address, hi2c->psize, I2C_AUTO_STOP_MODE, start);
  }
}

/**
  * @brief  the master transmits data through polling mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_master_transmit(i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  // Initialise the data buffer
  i2c_set_buffer(hi2c, I2C_STEP_DATA, pdata, size);

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_1;
    return hi2c->error_code;
  }

  /* start transfer */
  i2c_start_transfer(hi2c, address, I2C_GEN_START_WRITE);

  while (hi2c->pcount[hi2c->step] > 0)
  {
    /* wait for the tdis flag to be set */
    if(i2c_wait_flag(hi2c, I2C_TDIS_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
      hi2c->error_code = I2C_ERR_STEP_2;
      return hi2c->error_code;
    }

    /* send data */
    i2c_data_send(hi2c->i2cx, *hi2c->pbuff[hi2c->step]++);
    hi2c->psize--;
    hi2c->pcount[hi2c->step]--;

    if ((hi2c->psize == 0) && (hi2c->pcount[hi2c->step] != 0))
    {
      /* wait for the tcrld flag to be set  */
      if (i2c_wait_flag(hi2c, I2C_TCRLD_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
      {
        hi2c->error_code = I2C_ERR_STEP_3;
        return hi2c->error_code;
      }

      /* continue transfer */
      i2c_start_transfer(hi2c, address, I2C_WITHOUT_START);
    }
  }

  /* wait for the stop flag to be set  */
  if(i2c_wait_flag(hi2c, I2C_STOPF_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_4;
    return hi2c->error_code;
  }

  /* clear stop flag */
  i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

  /* reset ctrl2 register */
  i2c_reset_ctrl2_register(hi2c);

  return hi2c->error_code;
}

/**
  * @brief  the slave receive data through polling mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_slave_receive(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  // Initialise the data buffer
  i2c_set_buffer(hi2c, I2C_STEP_DATA, pdata, size);

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_1;
    return hi2c->error_code;
  }

  /* enable acknowledge */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* wait for the addr flag to be set */
  if (i2c_wait_flag(hi2c, I2C_ADDRF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_2;
    return hi2c->error_code;
  }

  /* clear addr flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ADDRF_FLAG);

  while (hi2c->pcount[hi2c->step] > 0)
  {
    /* wait for the rdbf flag to be set  */
    if(i2c_wait_flag(hi2c, I2C_RDBF_FLAG, I2C_EVENT_CHECK_STOP, timeout) != I2C_OK)
    {
      /* disable acknowledge */
      i2c_ack_enable(hi2c->i2cx, FALSE);

      /* if data is received, read data */
      if (i2c_flag_get(hi2c->i2cx, I2C_RDBF_FLAG) == SET)
      {
        /* read data */
        (*hi2c->pbuff[hi2c->step]++) = i2c_data_receive(hi2c->i2cx);
        hi2c->pcount[hi2c->step]--;
      }

      hi2c->error_code = I2C_ERR_STEP_4;
      return hi2c->error_code;
    }

    /* read data */
    (*hi2c->pbuff[hi2c->step]++) = i2c_data_receive(hi2c->i2cx);
    hi2c->pcount[hi2c->step]--;
  }

  /* wait for the stop flag to be set */
  if(i2c_wait_flag(hi2c, I2C_STOPF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    /* disable acknowledge */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    hi2c->error_code = I2C_ERR_STEP_5;
    return hi2c->error_code;
  }

  /* clear stop flag */
  i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    /* disable acknowledge */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    hi2c->error_code = I2C_ERR_STEP_6;
    return hi2c->error_code;
  }

  return hi2c->error_code;
}

/**
  * @brief  the master receive data through polling mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_master_receive(i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  // Initialise the data buffer
  i2c_set_buffer(hi2c, I2C_STEP_DATA, pdata, size);

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_1;
    return hi2c->error_code;
  }

  /* start transfer */
  i2c_start_transfer(hi2c, address, I2C_GEN_START_READ);

  while (hi2c->pcount[hi2c->step] > 0)
  {
    /* wait for the rdbf flag to be set  */
    if(i2c_wait_flag(hi2c, I2C_RDBF_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
      hi2c->error_code = I2C_ERR_STEP_2;
      return hi2c->error_code;
    }

    /* read data */
    (*hi2c->pbuff[hi2c->step]++) = i2c_data_receive(hi2c->i2cx);
    hi2c->pcount[hi2c->step]--;
    hi2c->psize--;

    if ((hi2c->psize == 0) && (hi2c->pcount[hi2c->step] != 0))
    {
      /* wait for the tcrld flag to be set  */
      if (i2c_wait_flag(hi2c, I2C_TCRLD_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
      {
        hi2c->error_code = I2C_ERR_STEP_3;
        return hi2c->error_code;
      }

      /* continue transfer */
      i2c_start_transfer(hi2c, address, I2C_WITHOUT_START);
    }
  }

  /* wait for the stop flag to be set  */
  if(i2c_wait_flag(hi2c, I2C_STOPF_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_4;
    return hi2c->error_code;
  }

  /* clear stop flag */
  i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

  /* reset ctrl2 register */
  i2c_reset_ctrl2_register(hi2c);

  return hi2c->error_code;
}

/**
  * @brief  the slave transmits data through polling mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_slave_transmit(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  // Initialise the data buffer
  i2c_set_buffer(hi2c, I2C_STEP_DATA, pdata, size);

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_1;
    return hi2c->error_code;
  }

  /* enable acknowledge */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* wait for the addr flag to be set */
  if (i2c_wait_flag(hi2c, I2C_ADDRF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    /* disable acknowledge */
    i2c_ack_enable(hi2c->i2cx, FALSE);
    hi2c->error_code = I2C_ERR_STEP_2;
    return hi2c->error_code;
  }

  /* clear addr flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ADDRF_FLAG);

  /* if 10-bit address mode is used */
  if (hi2c->i2cx->ctrl2_bit.addr10 != RESET)
  {
    /* wait for the addr flag to be set */
    if (i2c_wait_flag(hi2c, I2C_ADDRF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
    {
      /* disable acknowledge */
      i2c_ack_enable(hi2c->i2cx, FALSE);

      hi2c->error_code = I2C_ERR_STEP_3;
      return hi2c->error_code;
    }

    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDRF_FLAG);
  }

  while (hi2c->pcount[hi2c->step] > 0)
  {
    /* wait for the tdis flag to be set */
    if(i2c_wait_flag(hi2c, I2C_TDIS_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
      /* disable acknowledge */
      i2c_ack_enable(hi2c->i2cx, FALSE);

      hi2c->error_code = I2C_ERR_STEP_5;
      return hi2c->error_code;
    }

    /* send data */
    i2c_data_send(hi2c->i2cx, *hi2c->pbuff[hi2c->step]++);
    hi2c->pcount[hi2c->step]--;
  }

  /* wait for the ackfail flag to be set */
  if(i2c_wait_flag(hi2c, I2C_ACKFAIL_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_6;
    return hi2c->error_code;
  }

  /* clear ack fail flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);

  /* wait for the stop flag to be set */
  if(i2c_wait_flag(hi2c, I2C_STOPF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    /* disable acknowledge */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    hi2c->error_code = I2C_ERR_STEP_7;
    return hi2c->error_code;
  }

  /* clear stop flag */
  i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    /* disable acknowledge */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    hi2c->error_code = I2C_ERR_STEP_8;
    return hi2c->error_code;
  }

  /* refresh tx dt register */
  i2c_refresh_txdt_register(hi2c);

  return hi2c->error_code;
}

/**
  * @brief  the master transmits data through interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_master_transmit_int(i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_INT_MA_TX;
  hi2c->state = I2C_START;

  // Initialise the data buffer
  i2c_set_buffer(hi2c, I2C_STEP_DATA, pdata, size);

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_1;
    return hi2c->error_code;
  }

  /* start transfer */
  i2c_start_transfer(hi2c, address, I2C_GEN_START_WRITE);

  /* enable interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_TDC_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_TD_INT, TRUE);

  return hi2c->error_code;
}

/**
  * @brief  the slave receive data through interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_slave_receive_int(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_INT_SLA_RX;
  hi2c->state = I2C_START;

  // Initialise the data buffer
  i2c_set_buffer(hi2c, I2C_STEP_DATA, pdata, size);

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
      hi2c->error_code = I2C_ERR_STEP_1;
  } else {
      /* enable acknowledge */
      i2c_ack_enable(hi2c->i2cx, TRUE);

      /* enable interrupt */
      i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_TDC_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_ADDR_INT | I2C_RD_INT, TRUE);
  }

  return hi2c->error_code;
}

/**
  * @brief  the master receive data through interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_master_receive_int(i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_INT_MA_RX;
  hi2c->state = I2C_START;

  // Initialise the data buffer
  i2c_set_buffer(hi2c, I2C_STEP_DATA, pdata, size);

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
      hi2c->error_code = I2C_ERR_STEP_1;
  } else {
      /* start transfer */
      i2c_start_transfer(hi2c, address, I2C_GEN_START_READ);

      /* enable interrupt */
      i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_TDC_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_RD_INT, TRUE);
  }

  return hi2c->error_code;
}

/**
  * @brief  the slave transmits data through interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_slave_transmit_int(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_INT_SLA_TX;
  hi2c->state = I2C_START;

  // Initialise the data buffer
  i2c_set_buffer(hi2c, I2C_STEP_DATA, pdata, size);

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
      hi2c->error_code = I2C_ERR_STEP_1;
  } else {
      /* enable acknowledge */
      i2c_ack_enable(hi2c->i2cx, TRUE);

      /* enable interrupt */
      i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_TDC_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_ADDR_INT | I2C_TD_INT, TRUE);

      i2c_refresh_txdt_register(hi2c);
  }

  return hi2c->error_code;
}

/**
  * @brief  the master transmits data through dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_master_transmit_dma(i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_DMA_MA_TX;
  hi2c->state = I2C_START;

  // Initialise the data buffer
  i2c_set_buffer(hi2c, I2C_STEP_DATA, pdata, size);

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
      hi2c->error_code = I2C_ERR_STEP_1;
  } else {
      /* disable dma request */
      i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_TX, FALSE);

      /* start transfer */
      i2c_start_transfer_dma(hi2c, hi2c->dma_tx_channel, address, I2C_GEN_START_WRITE);

      /* enable i2c interrupt */
      i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_ACKFIAL_INT, TRUE);

      /* enable dma request */
      i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_TX, TRUE);
  }

  return hi2c->error_code;
}

/**
  * @brief  the slave receive data through dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_slave_receive_dma(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_DMA_SLA_RX;
  hi2c->state = I2C_START;

  // Initialise the data buffer
  i2c_set_buffer(hi2c, I2C_STEP_DATA, pdata, size);

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
      hi2c->error_code = I2C_ERR_STEP_1;
  } else {
      /* disable dma request */
      i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_RX, FALSE);

      /* config dma */
      i2c_dma_config(hi2c, hi2c->dma_rx_channel, hi2c->pbuff[hi2c->step], size);

      /* enable acknowledge */
      i2c_ack_enable(hi2c->i2cx, TRUE);

      /* enable i2c interrupt */
      i2c_interrupt_enable(hi2c->i2cx, I2C_ADDR_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_ERR_INT, TRUE);

      /* enable dma request */
      i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_RX, TRUE);
  }

  return hi2c->error_code;
}

/**
  * @brief  the master receive data through dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_master_receive_dma(i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_DMA_MA_RX;
  hi2c->state = I2C_START;

  // Initialise the data buffer
  i2c_set_buffer(hi2c, I2C_STEP_DATA, pdata, size);

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
      hi2c->error_code = I2C_ERR_STEP_1;
  } else {
      /* disable dma request */
      i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_RX, FALSE);

      /* start transfer */
      i2c_start_transfer_dma(hi2c, hi2c->dma_rx_channel, address, I2C_GEN_START_READ);

      /* enable i2c interrupt */
      i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_ACKFIAL_INT, TRUE);

      /* enable dma request */
      i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_RX, TRUE);
  }

  return hi2c->error_code;
}

/**
  * @brief  the slave transmits data through dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_slave_transmit_dma(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_DMA_SLA_TX;
  hi2c->state = I2C_START;

  // Initialise the data buffer
  i2c_set_buffer(hi2c, I2C_STEP_DATA, pdata, size);

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
      hi2c->error_code = I2C_ERR_STEP_1;
  } else {
      /* disable dma request */
      i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_TX, FALSE);

      /* config dma */
      i2c_dma_config(hi2c, hi2c->dma_tx_channel, hi2c->pbuff[hi2c->step], size);

      /* enable acknowledge */
      i2c_ack_enable(hi2c->i2cx, TRUE);

      /* enable i2c interrupt */
      i2c_interrupt_enable(hi2c->i2cx, I2C_ADDR_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_ERR_INT, TRUE);

      /* enable dma request */
      i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_TX, TRUE);
  }

  return hi2c->error_code;
}

/**
  * @brief  send memory address.
  * @param  hi2c: the handle points to the operation information.
  * @param  mem_address_width: memory address width.
  *         this parameter can be one of the following values:
  *         - I2C_MEM_ADDR_WIDIH_8:  memory address is 8 bit 
  *         - I2C_MEM_ADDR_WIDIH_16:  memory address is 16 bit 
  * @param  address: memory device address.
  * @param  mem_address: memory address.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_memory_address_send(i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t mem_address, int32_t timeout)
{
  i2c_status_type err_code;
  
  if(mem_address_width == I2C_MEM_ADDR_WIDIH_8)
  {
    /* send memory address */
    i2c_data_send(hi2c->i2cx, mem_address & 0xFF);
  }
  else
  {
    /* send memory address */
    i2c_data_send(hi2c->i2cx, (mem_address >> 8) & 0xFF);  
    
    /* wait for the tdis flag to be set */
    err_code = i2c_wait_flag(hi2c, I2C_TDIS_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout);
    
    if(err_code != I2C_OK)
    {
      return err_code;
    }
    
    /* send memory address */
    i2c_data_send(hi2c->i2cx, mem_address & 0xFF);  
  }
  
  return hi2c->error_code;
}

/**
  * @brief  write data to the memory device through polling mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  mem_address_width: memory address width.
  *         this parameter can be one of the following values:
  *         - I2C_MEM_ADDR_WIDIH_8:  memory address is 8 bit 
  *         - I2C_MEM_ADDR_WIDIH_16:  memory address is 16 bit 
  * @param  address: memory device address.
  * @param  mem_address: memory address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_memory_write(i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_MA_TX;

  // Initialise the data buffer
  i2c_set_buffer(hi2c, I2C_STEP_DATA, pdata, size + mem_address_width);

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_1;
    return hi2c->error_code;
  }

  /* start transfer */
  i2c_start_transfer(hi2c, address, I2C_GEN_START_WRITE);

  /* wait for the tdis flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDIS_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_2;
    return hi2c->error_code;
  }

  /* send memory address */
  if(i2c_memory_address_send(hi2c, mem_address_width, mem_address, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_3;
    return hi2c->error_code;
  }

  hi2c->psize -= mem_address_width;
  hi2c->pcount[hi2c->step] -= mem_address_width;

  while (hi2c->pcount[hi2c->step] > 0)
  {
    /* wait for the tdis flag to be set */
    if(i2c_wait_flag(hi2c, I2C_TDIS_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
      hi2c->error_code = I2C_ERR_STEP_4;
      return hi2c->error_code;
    }

    /* send data */
    i2c_data_send(hi2c->i2cx, *hi2c->pbuff[hi2c->step]++);
    hi2c->psize--;
    hi2c->pcount[hi2c->step]--;

    if ((hi2c->psize == 0) && (hi2c->pcount[hi2c->step] != 0))
    {
      /* wait for the tcrld flag to be set  */
      if (i2c_wait_flag(hi2c, I2C_TCRLD_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
      {
        hi2c->error_code = I2C_ERR_STEP_5;
        return hi2c->error_code;
      }

      /* continue transfer */
      i2c_start_transfer(hi2c, address, I2C_WITHOUT_START);
    }
  }

  /* wait for the stop flag to be set  */
  if(i2c_wait_flag(hi2c, I2C_STOPF_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_6;
    return hi2c->error_code;
  }

  /* clear stop flag */
  i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

  /* reset ctrl2 register */
  i2c_reset_ctrl2_register(hi2c);

  return hi2c->error_code;
}

/**
  * @brief  read data from memory device through polling mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  mem_address_width: memory address width.
  *         this parameter can be one of the following values:
  *         - I2C_MEM_ADDR_WIDIH_8:  memory address is 8 bit 
  *         - I2C_MEM_ADDR_WIDIH_16:  memory address is 16 bit 
  * @param  address: memory device address.
  * @param  mem_address: memory address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_memory_read(i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_MA_RX;

  // Initialise the data buffer
  i2c_set_buffer(hi2c, I2C_STEP_DATA, pdata, size);

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_1;
    return hi2c->error_code;
  }

  /* start transfer */
  i2c_transmit_set(hi2c->i2cx, address, mem_address_width, I2C_SOFT_STOP_MODE, I2C_GEN_START_WRITE);

  /* wait for the tdis flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDIS_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_2;
    return hi2c->error_code;
  }

  /* send memory address */
  if(i2c_memory_address_send(hi2c, mem_address_width, mem_address, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_3;
    return hi2c->error_code;
  }

  /* wait for the tdc flag to be set */
  if (i2c_wait_flag(hi2c, I2C_TDC_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_4;
    return hi2c->error_code;
  }

  /* start transfer */
  i2c_start_transfer(hi2c, address, I2C_GEN_START_READ);

  while (hi2c->pcount[hi2c->step] > 0)
  {
    /* wait for the rdbf flag to be set  */
    if (i2c_wait_flag(hi2c, I2C_RDBF_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
      hi2c->error_code = I2C_ERR_STEP_5;
    }

    /* read data */
    (*hi2c->pbuff[hi2c->step]++) = i2c_data_receive(hi2c->i2cx);
    hi2c->pcount[hi2c->step]--;
    hi2c->psize--;

    if ((hi2c->psize == 0) && (hi2c->pcount[hi2c->step] != 0))
    {
      /* wait for the tcrld flag to be set  */
      if (i2c_wait_flag(hi2c, I2C_TCRLD_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
      {
        hi2c->error_code = I2C_ERR_STEP_6;
        return hi2c->error_code;
      }

      /* continue transfer */
      i2c_start_transfer(hi2c, address, I2C_WITHOUT_START);
    }
  }

  /* wait for the stop flag to be set  */
  if (i2c_wait_flag(hi2c, I2C_STOPF_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_7;
    return hi2c->error_code;
  }

  /* clear stop flag */
  i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

  /* reset ctrl2 register */
  i2c_reset_ctrl2_register(hi2c);

  return hi2c->error_code;
}

/**
  * @brief  write data to the memory device through interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  mem_address_width: memory address width.
  *         this parameter can be one of the following values:
  *         - I2C_MEM_ADDR_WIDIH_8:  memory address is 8 bit 
  *         - I2C_MEM_ADDR_WIDIH_16:  memory address is 16 bit 
  * @param  address: memory device address.
  * @param  mem_address: memory address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_memory_write_int(i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  if (mem_address_width == I2C_MEM_ADDR_WIDIH_8) {
	  // Note that this works for little endian without adjusting the address
	  hi2c->pcount[I2C_STEP_REG] = 1;
	  // address is on the stack and we need a copy on the heap for the interrupt processing to use
	  hi2c->reg = mem_address;
  } else {
	  hi2c->pcount[I2C_STEP_REG] = 2;
	  // I2C address is big endian, so swap the address bytes
	  hi2c->reg = ((mem_address >> 8) & 0xff) | ((mem_address & 0xff) << 8);
  }

  hi2c->step = I2C_STEP_REG;
  hi2c->pbuff[I2C_STEP_REG] = (uint8_t *)&hi2c->reg;

  hi2c->pbuff[I2C_STEP_DATA] = pdata;
  hi2c->pcount[I2C_STEP_DATA] = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
	  hi2c->error_code = I2C_ERR_STEP_1;
	  return hi2c->error_code;
  }

  hi2c->state  = I2C_START;

  /* initialization parameters */
  hi2c->mode = I2C_INT_MA_TX;

  /* start transfer */
  i2c_start_transfer(hi2c, address, I2C_GEN_START_WRITE);

  /* enable interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_TDC_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_TD_INT, TRUE);

  return hi2c->error_code;
}

/**
  * @brief  read data from memory device through interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  mem_address_width: memory address width.
  *         this parameter can be one of the following values:
  *         - I2C_MEM_ADDR_WIDIH_8:  memory address is 8 bit 
  *         - I2C_MEM_ADDR_WIDIH_16:  memory address is 16 bit 
  * @param  address: memory device address.
  * @param  mem_address: memory address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_memory_read_int(i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  if (mem_address_width == I2C_MEM_ADDR_WIDIH_8) {
	  // Note that this works for little endian without adjusting the address
	  hi2c->pcount[I2C_STEP_REG] = 1;
	  // address is on the stack and we need a copy on the heap for the interrupt processing to use
	  hi2c->reg = mem_address;
  } else {
	  hi2c->pcount[I2C_STEP_REG] = 2;
	  // I2C address is big endian, so swap the address bytes
	  hi2c->reg = ((mem_address >> 8) & 0xff) | ((mem_address & 0xff) << 8);
  }

  hi2c->step = I2C_STEP_REG;
  hi2c->pbuff[I2C_STEP_REG] = (uint8_t *)&hi2c->reg;

  hi2c->pbuff[I2C_STEP_DATA] = pdata;
  hi2c->pcount[I2C_STEP_DATA] = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
      hi2c->error_code = I2C_ERR_STEP_1;
      return hi2c->error_code;
  }

  hi2c->state  = I2C_START;

  /* initialization parameters */
  hi2c->mode   = I2C_INT_MA_RX;

  /* start transfer, initially a write with the register number */
  i2c_start_transfer(hi2c, address, I2C_GEN_START_WRITE);

  /* enable i2c interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_TDC_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_TD_INT | I2C_RD_INT, TRUE);

  return hi2c->error_code;
}

/**
  * @brief  write data to the memory device through dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  mem_address_width: memory address width.
  *         this parameter can be one of the following values:
  *         - I2C_MEM_ADDR_WIDIH_8:  memory address is 8 bit 
  *         - I2C_MEM_ADDR_WIDIH_16:  memory address is 16 bit 
  * @param  address: memory device address.
  * @param  mem_address: memory address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_memory_write_dma(i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_DMA_MA_TX;
  hi2c->state = I2C_START;

  // Initialise the data buffer
  i2c_set_buffer(hi2c, I2C_STEP_DATA, pdata, size);

  hi2c->error_code = I2C_OK;

  /* disable dma request */
  i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_TX, FALSE);

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
      return hi2c->error_code;
      hi2c->error_code = I2C_ERR_STEP_1;
  }

  /* transfer config */
  i2c_transmit_set(hi2c->i2cx, address, mem_address_width, I2C_RELOAD_MODE, I2C_GEN_START_WRITE);

  /* wait for the tdis flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDIS_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
      return hi2c->error_code;
      hi2c->error_code = I2C_ERR_STEP_2;
  }

  /* send memory address */
  if(i2c_memory_address_send(hi2c, mem_address_width, mem_address, timeout) != I2C_OK)
  {
      return hi2c->error_code;
      hi2c->error_code = I2C_ERR_STEP_3;
  }

  /* wait for the tcrld flag to be set */
  if (i2c_wait_flag(hi2c, I2C_TCRLD_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
      return hi2c->error_code;
      hi2c->error_code = I2C_ERR_STEP_4;
  }

  /* start transfer */
  i2c_start_transfer_dma(hi2c, hi2c->dma_tx_channel, address, I2C_WITHOUT_START);

  /* enable i2c interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_ACKFIAL_INT, TRUE);

  /* enable dma request */
  i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_TX, TRUE);

  return hi2c->error_code;
}

/**
  * @brief  read data from memory device through polling mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  mem_address_width: memory address width.
  *         this parameter can be one of the following values:
  *         - I2C_MEM_ADDR_WIDIH_8:  memory address is 8 bit 
  *         - I2C_MEM_ADDR_WIDIH_16:  memory address is 16 bit 
  * @param  address: memory device address.
  * @param  mem_address: memory address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_memory_read_dma(i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_DMA_MA_RX;
  hi2c->state = I2C_START;

  // Initialise the data buffer
  i2c_set_buffer(hi2c, I2C_STEP_DATA, pdata, size);

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
      hi2c->error_code = I2C_ERR_STEP_1;
      return hi2c->error_code;
  }

  /* start transfer */
  i2c_transmit_set(hi2c->i2cx, address, mem_address_width, I2C_SOFT_STOP_MODE, I2C_GEN_START_WRITE);

  /* wait for the tdis flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDIS_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
      hi2c->error_code = I2C_ERR_STEP_2;
      return hi2c->error_code;
  }

  /* send memory address */
  if(i2c_memory_address_send(hi2c, mem_address_width, mem_address, timeout) != I2C_OK)
  {
      hi2c->error_code = I2C_ERR_STEP_3;
      return hi2c->error_code;
  }

  /* wait for the tdc flag to be set */
  if (i2c_wait_flag(hi2c, I2C_TDC_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
      hi2c->error_code = I2C_ERR_STEP_4;
      return hi2c->error_code;
  }

  /* disable dma request */
  i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_RX, FALSE);

  /* start transfer */
  i2c_start_transfer_dma(hi2c, hi2c->dma_rx_channel, address, I2C_GEN_START_READ);

  /* enable i2c interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_ACKFIAL_INT, TRUE);

  /* enable dma request */
  i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_RX, TRUE);

  return hi2c->error_code;
}

/**
  * @brief  the master transmits data through SMBus mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_smbus_master_transmit(i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  // Initialise the data buffer
  i2c_set_buffer(hi2c, I2C_STEP_DATA, pdata, size);

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_1;
    return hi2c->error_code;
  }

  /* enable pec calculation */
  i2c_pec_calculate_enable(hi2c->i2cx, TRUE);

  /* enable pec transmit request */
  i2c_pec_transmit_enable(hi2c->i2cx, TRUE);

  /* start transfer */
  i2c_start_transfer(hi2c, address, I2C_GEN_START_WRITE);

  hi2c->pcount[hi2c->step]--;

  while (hi2c->pcount[hi2c->step] > 0)
  {
    /* wait for the tdis flag to be set */
    if(i2c_wait_flag(hi2c, I2C_TDIS_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
      hi2c->error_code = I2C_ERR_STEP_2;
      return hi2c->error_code;
    }

    /* send data */
    i2c_data_send(hi2c->i2cx, *hi2c->pbuff[hi2c->step]++);
    hi2c->psize--;
    hi2c->pcount[hi2c->step]--;

    if ((hi2c->psize == 0) && (hi2c->pcount[hi2c->step] != 0))
    {
      /* wait for the tcrld flag to be set  */
      if (i2c_wait_flag(hi2c, I2C_TCRLD_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
      {
        hi2c->error_code = I2C_ERR_STEP_3;
        return hi2c->error_code;
      }

      /* continue transfer */
      i2c_start_transfer(hi2c, address, I2C_WITHOUT_START);
    }
  }

  /* wait for the stop flag to be set  */
  if(i2c_wait_flag(hi2c, I2C_STOPF_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_4;
    return hi2c->error_code;
  }

  /* clear stop flag */
  i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

  /* reset ctrl2 register */
  i2c_reset_ctrl2_register(hi2c);

  return hi2c->error_code;
}

/**
  * @brief  the slave receive data through SMBus mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_smbus_slave_receive(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  // Initialise the data buffer
  i2c_set_buffer(hi2c, I2C_STEP_DATA, pdata, size);

  hi2c->error_code = I2C_OK;

  /* enable pec calculation */
  i2c_pec_calculate_enable(hi2c->i2cx, TRUE);

  /* enable slave data control mode */
  i2c_slave_data_ctrl_enable(hi2c->i2cx, TRUE);

  /* enable acknowledge */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* wait for the addr flag to be set */
  if (i2c_wait_flag(hi2c, I2C_ADDRF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_1;
    return hi2c->error_code;
  }

  /* enable pec transmit request */
  i2c_pec_transmit_enable(hi2c->i2cx, TRUE);

  /* configure the number of bytes to be transmitted */
  i2c_cnt_set(hi2c->i2cx, hi2c->pcount[hi2c->step]);

  /* clear addr flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ADDRF_FLAG);

  while (hi2c->pcount[hi2c->step] > 0)
  {
    /* wait for the rdbf flag to be set  */
    if(i2c_wait_flag(hi2c, I2C_RDBF_FLAG, I2C_EVENT_CHECK_STOP, timeout) != I2C_OK)
    {
      /* disable acknowledge */
      i2c_ack_enable(hi2c->i2cx, FALSE);

      /* if data is received, read data */
      if (i2c_flag_get(hi2c->i2cx, I2C_RDBF_FLAG) == SET)
      {
        /* read data */
        (*hi2c->pbuff[hi2c->step]++) = i2c_data_receive(hi2c->i2cx);
        hi2c->pcount[hi2c->step]--;
      }

      hi2c->error_code = I2C_ERR_STEP_3;
      return hi2c->error_code;
    }

    /* read data */
    (*hi2c->pbuff[hi2c->step]++) = i2c_data_receive(hi2c->i2cx);
    hi2c->pcount[hi2c->step]--;
  }

  /* wait for the stop flag to be set */
  if(i2c_wait_flag(hi2c, I2C_STOPF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    /* disable acknowledge */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    hi2c->error_code = I2C_ERR_STEP_4;
    return hi2c->error_code;
  }

  /* clear stop flag */
  i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    /* disable acknowledge */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    hi2c->error_code = I2C_ERR_STEP_5;
    return hi2c->error_code;
  }

  /* disable slave data control mode */
  i2c_slave_data_ctrl_enable(hi2c->i2cx, FALSE);

  return hi2c->error_code;
}

/**
  * @brief  the master receive data through SMBus mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_smbus_master_receive(i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  // Initialise the data buffer
  i2c_set_buffer(hi2c, I2C_STEP_DATA, pdata, size);

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_1;
    return hi2c->error_code;
  }

  /* enable pec calculation */
  i2c_pec_calculate_enable(hi2c->i2cx, TRUE);

  /* enable pec transmit request */
  i2c_pec_transmit_enable(hi2c->i2cx, TRUE);

  /* start transfer */
  i2c_start_transfer(hi2c, address, I2C_GEN_START_READ);

  while (hi2c->pcount[hi2c->step] > 0)
  {
    /* wait for the rdbf flag to be set  */
    if(i2c_wait_flag(hi2c, I2C_RDBF_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
      hi2c->error_code = I2C_ERR_STEP_2;
      return hi2c->error_code;
    }

    /* read data */
    (*hi2c->pbuff[hi2c->step]++) = i2c_data_receive(hi2c->i2cx);
    hi2c->pcount[hi2c->step]--;
    hi2c->psize--;

    if ((hi2c->psize == 0) && (hi2c->pcount[hi2c->step] != 0))
    {
      /* wait for the tcrld flag to be set  */
      if (i2c_wait_flag(hi2c, I2C_TCRLD_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
      {
        hi2c->error_code = I2C_ERR_STEP_3;
        return hi2c->error_code;
      }

      /* continue transfer */
      i2c_start_transfer(hi2c, address, I2C_WITHOUT_START);
    }
  }

  /* wait for the stop flag to be set  */
  if(i2c_wait_flag(hi2c, I2C_STOPF_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_4;
    return hi2c->error_code;
  }

  /* clear stop flag */
  i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

  /* reset ctrl2 register */
  i2c_reset_ctrl2_register(hi2c);

  return hi2c->error_code;
}

/**
  * @brief  the slave transmits data through SMBus mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_smbus_slave_transmit(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  // Initialise the data buffer
  i2c_set_buffer(hi2c, I2C_STEP_DATA, pdata, size);

  hi2c->error_code = I2C_OK;

  /* enable pec calculation */
  i2c_pec_calculate_enable(hi2c->i2cx, TRUE);

  /* enable slave data control mode */
  i2c_slave_data_ctrl_enable(hi2c->i2cx, TRUE);

  /* enable acknowledge */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* wait for the addr flag to be set */
  if (i2c_wait_flag(hi2c, I2C_ADDRF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    /* disable acknowledge */
    i2c_ack_enable(hi2c->i2cx, FALSE);
    hi2c->error_code = I2C_ERR_STEP_1;
    return hi2c->error_code;
  }

  /* if 7-bit address mode is selected */
  if (hi2c->i2cx->ctrl2_bit.addr10 == 0)
  {
    /* enable pec transmit request */
    i2c_pec_transmit_enable(hi2c->i2cx, TRUE);

    /* configure the number of bytes to be transmitted */
    i2c_cnt_set(hi2c->i2cx, hi2c->pcount[hi2c->step]);
  }

  /* clear addr flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ADDRF_FLAG);

  /* if 10-bit address mode is used */
  if (hi2c->i2cx->ctrl2_bit.addr10 != RESET)
  {
    /* wait for the addr flag to be set */
    if (i2c_wait_flag(hi2c, I2C_ADDRF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
    {
      /* disable acknowledge */
      i2c_ack_enable(hi2c->i2cx, FALSE);

      hi2c->error_code = I2C_ERR_STEP_2;
      return hi2c->error_code;
    }

    /* enable pec transmit request */
    i2c_pec_transmit_enable(hi2c->i2cx, TRUE);

    /* configure the number of bytes to be transmitted */
    i2c_cnt_set(hi2c->i2cx, hi2c->pcount[hi2c->step]);

    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDRF_FLAG);
  }

  hi2c->pcount[hi2c->step]--;

  while (hi2c->pcount[hi2c->step] > 0)
  {
    /* wait for the tdis flag to be set */
    if(i2c_wait_flag(hi2c, I2C_TDIS_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
      /* disable acknowledge */
      i2c_ack_enable(hi2c->i2cx, FALSE);

      hi2c->error_code = I2C_ERR_STEP_4;
      return hi2c->error_code;
    }

    /* send data */
    i2c_data_send(hi2c->i2cx, *hi2c->pbuff[hi2c->step]++);
    hi2c->pcount[hi2c->step]--;
  }

  /* wait for the ackfail flag to be set */
  if(i2c_wait_flag(hi2c, I2C_ACKFAIL_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_STEP_5;
    return hi2c->error_code;
  }

  /* clear ack fail flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);

  /* wait for the stop flag to be set */
  if(i2c_wait_flag(hi2c, I2C_STOPF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    /* disable acknowledge */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    hi2c->error_code = I2C_ERR_STEP_6;
    return hi2c->error_code;
  }

  /* clear stop flag */
  i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

  /* wait for the busy flag to be reset */
  if (i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    /* disable acknowledge */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    hi2c->error_code = I2C_ERR_STEP_7;
    return hi2c->error_code;
  }

  /* reset ctrl2 register */
  i2c_reset_ctrl2_register(hi2c);

  /* refresh tx dt register */
  i2c_refresh_txdt_register(hi2c);

  /* disable slave data control mode */
  i2c_slave_data_ctrl_enable(hi2c->i2cx, FALSE);

  return hi2c->error_code;
}

/**
  * @brief  master interrupt processing function in interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @retval i2c status.
  */
i2c_status_type i2c_master_irq_handler_int(i2c_handle_type* hi2c)
{
  if (i2c_flag_get(hi2c->i2cx, I2C_ACKFAIL_FLAG) != RESET)
  {
    /* clear ackfail flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);

    /* refresh tx register */
    i2c_refresh_txdt_register(hi2c);

    if(hi2c->pcount[hi2c->step] != 0)
    {
      hi2c->error_code = I2C_ERR_ACKFAIL;
    }
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_TDIS_FLAG) != RESET)
  {
    /* send data */
    i2c_data_send(hi2c->i2cx, *hi2c->pbuff[hi2c->step]++);
    hi2c->pcount[hi2c->step]--;
    hi2c->psize--;
	if ((hi2c->pcount[hi2c->step] == 0) && (hi2c->step == I2C_STEP_REG) && (hi2c->mode == I2C_INT_MA_TX)) {
	  // Advance to next step
	  hi2c->step = I2C_STEP_DATA;
	}
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_TCRLD_FLAG) != RESET)
  {
	if ((hi2c->psize == 0) && (hi2c->pcount[hi2c->step] != 0))
	{
	  i2c_start_transfer(hi2c, i2c_transfer_addr_get(hi2c->i2cx), I2C_WITHOUT_START);
	} else {
	  hi2c->error_code = I2C_ERR_TCRLD;
    }
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_RDBF_FLAG) != RESET)
  {
    /* read data */
    (*hi2c->pbuff[hi2c->step]++) = i2c_data_receive(hi2c->i2cx);
    hi2c->pcount[hi2c->step]--;
    hi2c->psize--;
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_TDC_FLAG) != RESET)
  {
    if (hi2c->pcount[hi2c->step] == 0)
    {
		if ((hi2c->step == I2C_STEP_REG) && (hi2c->mode == I2C_INT_MA_RX)) {
		  // Advance to next step
		  hi2c->step = I2C_STEP_DATA;

		  /* restart transfer as a read */
		  i2c_start_transfer(hi2c, i2c_transfer_addr_get(hi2c->i2cx), I2C_GEN_START_READ);
		} else {
		  if (hi2c->i2cx->ctrl2_bit.astopen == 0)
		  {
			/* generate stop condtion */
			i2c_stop_generate(hi2c->i2cx);
		  }
		}
    }
    else
    {
        hi2c->error_code =  I2C_ERR_TDC;
    }
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_STOPF_FLAG) != RESET)
  {
    /* clear stop flag */
    i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

    /* reset ctrl2 register */
    i2c_reset_ctrl2_register(hi2c);

    if (i2c_flag_get(hi2c->i2cx, I2C_ACKFAIL_FLAG) != RESET)
    {
      /* clear ackfail flag */
      i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);
    }

    /* refresh tx dt register */
    i2c_refresh_txdt_register(hi2c);

    /* disable interrupts */
    i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_TDC_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_TD_INT | I2C_RD_INT, FALSE);

    /* transfer complete */
    hi2c->state = I2C_END;
  }

  return hi2c->error_code;
}

/**
  * @brief  slave interrupt processing function in interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @retval i2c status.
  */
i2c_status_type i2c_slave_irq_handler_int(i2c_handle_type* hi2c)
{
  if (i2c_flag_get(hi2c->i2cx, I2C_ACKFAIL_FLAG) != RESET)
  {
    /* transfer complete */
    if (hi2c->pcount[hi2c->step] == 0)
    {
      i2c_refresh_txdt_register(hi2c);

      /* clear ackfail flag */
      i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);
    }
    /* the transfer has not been completed */
    else
    {
      /* clear ackfail flag */
      i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);
    }
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_ADDRF_FLAG) != RESET)
  {
    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDRF_FLAG);
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_TDIS_FLAG) != RESET)
  {
    if (hi2c->pcount[hi2c->step] > 0)
    {
      /* send data */
      hi2c->i2cx->txdt = (*(hi2c->pbuff[hi2c->step]++));
      hi2c->psize--;
      hi2c->pcount[hi2c->step]--;
    }
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_RDBF_FLAG) != RESET)
  {
    if (hi2c->pcount[hi2c->step] > 0)
    {
      /* read data */
      (*hi2c->pbuff[hi2c->step]++) = i2c_data_receive(hi2c->i2cx);
      hi2c->pcount[hi2c->step]--;
      hi2c->psize--;
    }
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_STOPF_FLAG) != RESET)
  {
    /* clear stop flag */
    i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

    /* disable interrupts */
    i2c_interrupt_enable(hi2c->i2cx, I2C_ADDR_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_ERR_INT | I2C_TDC_INT | I2C_TD_INT | I2C_RD_INT, FALSE);

    /* reset ctrl2 register */
    i2c_reset_ctrl2_register(hi2c);

    /* refresh tx dt register */
    i2c_refresh_txdt_register(hi2c);

    /* if data is received, read data */
    if (i2c_flag_get(hi2c->i2cx, I2C_RDBF_FLAG) != RESET)
    {
      /* read data */
      (*hi2c->pbuff[hi2c->step]++) = i2c_data_receive(hi2c->i2cx);

      if ((hi2c->psize > 0))
      {
        hi2c->pcount[hi2c->step]--;
        hi2c->psize--;
      }
    }

    /* transfer complete */
    hi2c->state = I2C_END;
  }

  return hi2c->error_code;
}

/**
  * @brief  master interrupt processing function in dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @retval i2c status.
  */
i2c_status_type i2c_master_irq_handler_dma(i2c_handle_type* hi2c)
{
  if (i2c_flag_get(hi2c->i2cx, I2C_ACKFAIL_FLAG) != RESET)
  {
    /* clear ackfail flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);

    /* enable stop interrupt to wait for stop generate stop */
    i2c_interrupt_enable(hi2c->i2cx, I2C_STOP_INT, TRUE);

    /* refresh tx dt register */
    i2c_refresh_txdt_register(hi2c);

    if(hi2c->pcount[hi2c->step] != 0)
    {
      hi2c->error_code = I2C_ERR_ACKFAIL;
      return hi2c->error_code;
    }
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_TCRLD_FLAG) != RESET)
  {
    /* disable tdc interrupt */
    i2c_interrupt_enable(hi2c->i2cx, I2C_TDC_INT, FALSE);

    if (hi2c->pcount[hi2c->step] != 0)
    {
      /* continue transfer */
      i2c_start_transfer(hi2c, i2c_transfer_addr_get(hi2c->i2cx), I2C_WITHOUT_START);

      /* enable dma request */
      if (hi2c->dma_init_struct.direction == DMA_DIR_MEMORY_TO_PERIPHERAL)
      {
        i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_TX, TRUE);
      }
      else
      {
        i2c_dma_enable(hi2c->i2cx, I2C_DMA_REQUEST_RX, TRUE);
      }
    }
    else
    {
      hi2c->error_code = I2C_ERR_TCRLD;
      return hi2c->error_code;
    }
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_STOPF_FLAG) != RESET)
  {
    /* clear stop flag */
    i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

    /* reset ctrl2 register */
    i2c_reset_ctrl2_register(hi2c);

    if (i2c_flag_get(hi2c->i2cx, I2C_ACKFAIL_FLAG) != RESET)
    {
      /* clear ackfail flag */
      i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);
    }

    /* refresh tx dt register */
    i2c_refresh_txdt_register(hi2c);

    /* disable interrupts */
    i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT | I2C_TDC_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_TD_INT | I2C_RD_INT, FALSE);

    /* transfer complete */
    hi2c->state = I2C_END;
  }

  return hi2c->error_code;
}

/**
  * @brief  slave interrupt processing function in dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @retval i2c status.
  */
i2c_status_type i2c_slave_irq_handler_dma(i2c_handle_type* hi2c)
{
  if (i2c_flag_get(hi2c->i2cx, I2C_ACKFAIL_FLAG) != RESET)
  {
    /* clear ackfail flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_ADDRF_FLAG) != RESET)
  {
    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDRF_FLAG);
  }
  else if (i2c_flag_get(hi2c->i2cx, I2C_STOPF_FLAG) != RESET)
  {
    /* clear stop flag */
    i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

    /* disable interrupts */
    i2c_interrupt_enable(hi2c->i2cx, I2C_ADDR_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_ERR_INT | I2C_TDC_INT | I2C_TD_INT | I2C_RD_INT, FALSE);

    /* reset ctrl2 register */
    i2c_reset_ctrl2_register(hi2c);

    /* refresh tx dt register */
    i2c_refresh_txdt_register(hi2c);

    /* if data is received, read data */
    if (i2c_flag_get(hi2c->i2cx, I2C_RDBF_FLAG) != RESET)
    {
      /* read data */
      (*hi2c->pbuff[hi2c->step]++) = i2c_data_receive(hi2c->i2cx);

      if ((hi2c->psize > 0))
      {
        hi2c->pcount[hi2c->step]--;
        hi2c->psize--;
      }
    }

    /* transfer complete */
    hi2c->state = I2C_END;
  }

  return hi2c->error_code;
}

/**
  * @brief  dma processing function.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_dma_tx_rx_irq_handler(i2c_handle_type* hi2c, dma_channel_type* dma_channel)
{
  /* transfer complete */
  if (dma_flag_get(DMA_GET_TC_FLAG(dma_channel)) != RESET)
  {
    /* disable the transfer complete interrupt */
    dma_interrupt_enable(dma_channel, DMA_FDT_INT, FALSE);

    /* clear the transfer complete flag */
    dma_flag_clear(DMA_GET_TC_FLAG(dma_channel));

    /* disable dma request */
    i2c_dma_enable(hi2c->i2cx, DMA_GET_REQUEST(dma_channel), FALSE);

    /* disable dma channel */
    dma_channel_enable(dma_channel, FALSE);

    switch(hi2c->mode)
    {
      case I2C_DMA_MA_TX:
      case I2C_DMA_MA_RX:
      {
        /* update the number of transfers */
        hi2c->pcount[hi2c->step] -= hi2c->psize;

        /* transfer complete */
        if (hi2c->pcount[hi2c->step] == 0)
        {
          /* enable stop interrupt */
          i2c_interrupt_enable(hi2c->i2cx, I2C_STOP_INT, TRUE);
        }
        /* the transfer has not been completed */
        else
        {
          /* update the buffer pointer of transfers */
          hi2c->pbuff[hi2c->step] += hi2c->psize;

          /* set the number to be transferred */
          if (hi2c->pcount[hi2c->step] > MAX_TRANSFER_CNT)
          {
            hi2c->psize = MAX_TRANSFER_CNT;
          }
          else
          {
            hi2c->psize = hi2c->pcount[hi2c->step];
          }

          /* config dma channel, continue to transfer data */
          i2c_dma_config(hi2c, dma_channel, hi2c->pbuff[hi2c->step], hi2c->psize);

          /* enable tdc interrupt */
          i2c_interrupt_enable(hi2c->i2cx, I2C_TDC_INT, TRUE);
        }
      }break;
      case I2C_DMA_SLA_TX:
      case I2C_DMA_SLA_RX:
      {

      }break;

      default:break;
    }
  }
}

/**
  * @brief  dma transmission complete interrupt function.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_dma_tx_irq_handler(i2c_handle_type* hi2c)
{
  i2c_dma_tx_rx_irq_handler(hi2c, hi2c->dma_tx_channel);
}

/**
  * @brief  dma reveive complete interrupt function.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_dma_rx_irq_handler(i2c_handle_type* hi2c)
{
  i2c_dma_tx_rx_irq_handler(hi2c, hi2c->dma_rx_channel);
}

/**
  * @brief  interrupt procession function.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_evt_irq_handler(i2c_handle_type* hi2c)
{
  switch(hi2c->mode)
  {
    case I2C_INT_MA_TX:
    case I2C_INT_MA_RX:
    {
      i2c_master_irq_handler_int(hi2c);
    }break;
    case I2C_INT_SLA_TX:
    case I2C_INT_SLA_RX:
    {
      i2c_slave_irq_handler_int(hi2c);
    }break;
    case I2C_DMA_MA_TX:
    case I2C_DMA_MA_RX:
    {
      i2c_master_irq_handler_dma(hi2c);
    }break;
    case I2C_DMA_SLA_TX:
    case I2C_DMA_SLA_RX:
    {
      i2c_slave_irq_handler_dma(hi2c);
    }break;

    default:break;
  }
}

/**
  * @brief  dma reveive complete interrupt function.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_err_irq_handler(i2c_handle_type* hi2c)
{ 
  /* buserr */
  if (i2c_flag_get(hi2c->i2cx, I2C_BUSERR_FLAG) != RESET)
  {
    hi2c->error_code = I2C_ERR_INTERRUPT;
    
    /* clear flag */
    i2c_flag_clear(hi2c->i2cx, I2C_BUSERR_FLAG);
    
    /* disable interrupts */
    i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT, FALSE);
  }

  /* arlost */
  if (i2c_flag_get(hi2c->i2cx, I2C_ARLOST_FLAG) != RESET)
  {
    hi2c->error_code = I2C_ERR_INTERRUPT;
    
    /* clear flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ARLOST_FLAG);
    
    /* disable interrupts */
    i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT, FALSE);
  }

  /* ouf */
  if (i2c_flag_get(hi2c->i2cx, I2C_OUF_FLAG) != RESET)
  {
    hi2c->error_code = I2C_ERR_INTERRUPT;
    
    /* clear flag */
    i2c_flag_clear(hi2c->i2cx, I2C_OUF_FLAG);
    
    /* disable interrupts */
    i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT, FALSE);
  }

  /* pecerr */
  if (i2c_flag_get(hi2c->i2cx, I2C_PECERR_FLAG) != RESET)
  {
    hi2c->error_code = I2C_ERR_INTERRUPT;
    
    /* clear flag */
    i2c_flag_clear(hi2c->i2cx, I2C_PECERR_FLAG);
    
    /* disable interrupts */
    i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT, FALSE);

  }

  /* timeout */
  if (i2c_flag_get(hi2c->i2cx, I2C_TMOUT_FLAG) != RESET)
  {
    hi2c->error_code = I2C_ERR_INTERRUPT;
    
    /* clear flag */
    i2c_flag_clear(hi2c->i2cx, I2C_TMOUT_FLAG);
    
    /* disable interrupts */
    i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT, FALSE);
  }

  /* alertf */
  if (i2c_flag_get(hi2c->i2cx, I2C_ALERTF_FLAG) != RESET)
  {
    hi2c->error_code = I2C_ERR_INTERRUPT;
    
    /* clear flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ALERTF_FLAG);
    
    /* disable interrupts */
    i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT, FALSE);
  }
}

/**
  * @}
  */
