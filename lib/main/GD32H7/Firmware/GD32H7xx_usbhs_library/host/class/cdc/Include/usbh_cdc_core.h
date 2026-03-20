/*!
    \file    usbh_cdc_core.h
    \brief   header file for the usbh_cdc_core.c

    \version 2025-01-24, V1.4.0, firmware for GD32H7xx
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#ifndef USBH_CDC_CORE_H
#define USBH_CDC_CORE_H

#include "drv_usb_hw.h"
#include "usb_cdc.h"
#include "usbh_core.h"
#include "usbh_pipe.h"
#include "usbh_enum.h"
#include "usbh_transc.h"

#define CDC_BUFFER_SIZE                                    1024U                     /*!< CDC data transfer buff size */
#define LINE_CODING_STRUCTURE_SIZE                         0x07U                     /*!< CDC line configuration structure size */

/* states for CDC state machine */
typedef enum {
    CDC_IDLE = 0U,                                                                   /*!< CDC idle state */
    CDC_READ_DATA,                                                                   /*!< CDC read data state */
    CDC_SEND_DATA,                                                                   /*!< CDC send data state */
    CDC_DATA_SENT,                                                                   /*!< CDC data sent state */
    CDC_BUSY,                                                                        /*!< CDC busy state */
    CDC_GET_DATA,                                                                    /*!< CDC get data state */
    CDC_POLL,                                                                        /*!< CDC polling state */
    CDC_CTRL_STATE                                                                   /*!< CDC control state */
} cdc_state;

/* CDC transfer state */
typedef struct _cdc_xfer {
    volatile cdc_state cdc_cur_state;                                                /*!< CDC transfer state structure */

    uint8_t *prxtx_buff;                                                             /*!< CDC transfer buff pointer */
    uint8_t *pfill_buff;                                                             /*!< CDC transfer fill buff pointer */
    uint8_t *pempty_buff;                                                            /*!< CDC transfer empty buff pointer */
    uint32_t buffer_len;                                                             /*!< CDC transfer buffer length */
    uint16_t data_length;                                                            /*!< CDC transfer data length */
} cdc_xfer;

typedef struct _cdc_usercb {
    void  (*send)       (uint8_t *data_buf);
    void  (*receive)    (uint8_t *data_buf);
} cdc_usercb;

/* structure for CDC command interface */
typedef struct _cdc_cmd_itf {
    uint8_t              pipe_notify;                                                /*!< pipe notify */
    uint8_t              ep_notify;                                                  /*!< endpoint notify */
    uint8_t              buff[8U];                                                   /*!< command transfer buff */
    uint16_t             ep_size_notify;                                             /*!< endpoint size notify */
} cdc_cmd_itf;

/* structure for CDC data interface */
typedef struct _cdc_data_itf {
    uint8_t              pipe_in;                                                    /*!< IN pipe */
    uint8_t              pipe_out;                                                   /*!< OUT pipe */
    uint8_t              ep_in;                                                      /*!< endpoint IN */
    uint8_t              ep_out;                                                     /*!< endpoint OUT */
    uint8_t              buff[8U];                                                   /*!< endpoint transfer buff */
    uint16_t             ep_size_in;                                                 /*!< endpoint IN size */
    uint16_t             ep_size_out;                                                /*!< endpoint OUT size */
} cdc_data_itf;

/* states for CDC class state machine */
typedef enum {
    CDC_SET_LINE_CODING_RQUEST = 0U,                                                 /*!< set line coding request */
    CDC_GET_LINE_CODING_RQUEST,                                                      /*!< get line coding request */
    CDC_SET_CONTROL_LINE_STATE_REQUEST,                                              /*!< set control line state request */
    CDC_ERROR_STATE                                                                  /*!< error state request */
} cdc_requests;

/* line coding structure */
typedef union {
    uint8_t array[LINE_CODING_STRUCTURE_SIZE];                                       /*!< CDC line coding array buff */

   __ALIGN_BEGIN acm_line __ALIGN_END b;                                             /*!< CDC line coding structure */
} cdc_line_coding;

/* structure for CDC process */
typedef struct _cdc_process {
    uint8_t         rx_enabled;                                                      /*!< CDC receive enable */
    cdc_xfer        tx_param;                                                        /*!< data transmit parameter structure */
    cdc_xfer        rx_param;                                                        /*!< data receive parameter structure */
    cdc_line_coding line_code_get;                                                   /*!< line code get structure */
    cdc_line_coding line_code_set;                                                   /*!< line code set structure */
    cdc_cmd_itf     cmd_itf;                                                         /*!< command interface structure */
    cdc_data_itf    data_itf;                                                        /*!< data interface structure */
    cdc_requests    req_state;                                                       /*!< CDC control request state structure */
    cdc_usercb      user_cb;                                                         /*!< user callback function structure */
} usbh_cdc_handler;

extern usbh_class usbh_cdc;

/* function declarations */
/* send data to the device */
void cdc_data_send(usbh_host *uhost, uint8_t *data, uint16_t length);
/* send dummy data to the device */
void cdc_dummydata_send(usbh_host *uhost);
/* enable CDC receive */
void cdc_start_reception(usbh_host *uhost);
/* stop CDC receive */
void cdc_stop_reception(usbh_host *uhost);
/* get currently configured line coding */
usbh_status cdc_get_line_coding(usbh_host *uhost);
/* specify typical asynchronous line-character formatting properties */
usbh_status cdc_set_line_coding(usbh_host *uhost);
/* this request generates RS-232/V.24 style control signals */
usbh_status cdc_set_control_line_state(usbh_host *uhost);
/* this function prepares the state before issuing the class specific commands */
void cdc_change_state_to_issue_setconfig(usbh_host *uhost);
/* this function prepares the state before issuing the class specific commands */
void cdc_issue_getconfig(usbh_host *uhost);

#endif /* USBH_CDC_CORE_H */
