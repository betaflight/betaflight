/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if (CFG_TUH_ENABLED && CFG_TUH_CDC && CFG_TUH_CDC_RNDIS)

//--------------------------------------------------------------------+
// INCLUDE
//--------------------------------------------------------------------+
#include "common/tusb_common.h"
#include "cdc_host.h"
#include "cdc_rndis_host.h"

#if 0  // TODO remove subtask related macros later
// Sub Task
#define OSAL_SUBTASK_BEGIN
#define OSAL_SUBTASK_END                    return TUSB_ERROR_NONE;

#define STASK_RETURN(_error)                return _error;
#define STASK_INVOKE(_subtask, _status)     (_status) = _subtask
#define STASK_ASSERT(_cond)                 TU_VERIFY(_cond, TUSB_ERROR_OSAL_TASK_FAILED)
#endif

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+
#define RNDIS_MSG_PAYLOAD_MAX   (1024*4)

CFG_TUH_MEM_SECTION static uint8_t msg_notification[CFG_TUH_DEVICE_MAX][8];
CFG_TUH_MEM_SECTION CFG_TUH_MEM_ALIGN static uint8_t msg_payload[RNDIS_MSG_PAYLOAD_MAX];

static rndish_data_t rndish_data[CFG_TUH_DEVICE_MAX];

// TODO Microsoft requires message length for any get command must be at least 4096 bytes

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+
static tusb_error_t rndis_body_subtask(void);
static tusb_error_t send_message_get_response_subtask( uint8_t dev_addr, cdch_data_t *p_cdc,
                                                       uint8_t * p_mess, uint32_t mess_length,
                                                       uint8_t *p_response );

//--------------------------------------------------------------------+
// APPLICATION API
//--------------------------------------------------------------------+
tusb_error_t tusbh_cdc_rndis_get_mac_addr(uint8_t dev_addr, uint8_t mac_address[6])
{
  TU_ASSERT( tusbh_cdc_rndis_is_mounted(dev_addr),  TUSB_ERROR_CDCH_DEVICE_NOT_MOUNTED);
  TU_VERIFY( mac_address,  TUSB_ERROR_INVALID_PARA);

  memcpy(mac_address, rndish_data[dev_addr-1].mac_address, 6);

  return TUSB_ERROR_NONE;
}

//--------------------------------------------------------------------+
// IMPLEMENTATION
//--------------------------------------------------------------------+

// To enable the TASK_ASSERT style (quick return on false condition) in a real RTOS, a task must act as a wrapper
// and is used mainly to call subtasks. Within a subtask return statement can be called freely, the task with
// forever loop cannot have any return at all.
OSAL_TASK_FUNCTION(cdch_rndis_task) (void* param;)
{
  OSAL_TASK_BEGIN
  rndis_body_subtask();
  OSAL_TASK_END
}

static tusb_error_t rndis_body_subtask(void)
{
  static uint8_t relative_addr;

  OSAL_SUBTASK_BEGIN

  for (relative_addr = 0; relative_addr < CFG_TUH_DEVICE_MAX; relative_addr++)
  {

  }

  osal_task_delay(100);

  OSAL_SUBTASK_END
}

//--------------------------------------------------------------------+
// RNDIS-CDC Driver API
//--------------------------------------------------------------------+
void rndish_init(void)
{
  tu_memclr(rndish_data, sizeof(rndish_data_t)*CFG_TUH_DEVICE_MAX);

  //------------- Task creation -------------//

  //------------- semaphore creation for notification pipe -------------//
  for(uint8_t i=0; i<CFG_TUH_DEVICE_MAX; i++)
  {
    rndish_data[i].sem_notification_hdl = osal_semaphore_create( OSAL_SEM_REF(rndish_data[i].semaphore_notification) );
  }
}

void rndish_close(uint8_t dev_addr)
{
  osal_semaphore_reset( rndish_data[dev_addr-1].sem_notification_hdl );
//  tu_memclr(&rndish_data[dev_addr-1], sizeof(rndish_data_t)); TODO need to move semaphore & its handle out before memclr
}


static rndis_msg_initialize_t const msg_init =
{
    .type          = RNDIS_MSG_INITIALIZE,
    .length        = sizeof(rndis_msg_initialize_t),
    .request_id    = 1, // TODO should use some magic number
    .major_version = 1,
    .minor_version = 0,
    .max_xfer_size = 0x4000 // TODO mimic windows
};

static rndis_msg_query_t const msg_query_permanent_addr =
{
    .type          = RNDIS_MSG_QUERY,
    .length        = sizeof(rndis_msg_query_t)+6,
    .request_id    = 1,
    .oid           = RNDIS_OID_802_3_PERMANENT_ADDRESS,
    .buffer_length = 6,
    .buffer_offset = 20,
};

static rndis_msg_set_t const msg_set_packet_filter =
{
    .type          = RNDIS_MSG_SET,
    .length        = sizeof(rndis_msg_set_t)+4,
    .request_id    = 1,
    .oid           = RNDIS_OID_GEN_CURRENT_PACKET_FILTER,
    .buffer_length = 4,
    .buffer_offset = 20,
};

tusb_error_t rndish_open_subtask(uint8_t dev_addr, cdch_data_t *p_cdc)
{
  tusb_error_t error;

  OSAL_SUBTASK_BEGIN

  //------------- Message Initialize -------------//
  memcpy(msg_payload, &msg_init, sizeof(rndis_msg_initialize_t));
  STASK_INVOKE(
      send_message_get_response_subtask( dev_addr, p_cdc,
                                         msg_payload, sizeof(rndis_msg_initialize_t),
                                         msg_payload),
      error
  );
  if ( TUSB_ERROR_NONE != error )   STASK_RETURN(error);

  // TODO currently not support multiple data packets per xfer
  rndis_msg_initialize_cmplt_t * const p_init_cmpt = (rndis_msg_initialize_cmplt_t *) msg_payload;
  STASK_ASSERT(p_init_cmpt->type == RNDIS_MSG_INITIALIZE_CMPLT && p_init_cmpt->status == RNDIS_STATUS_SUCCESS &&
                 p_init_cmpt->max_packet_per_xfer == 1 && p_init_cmpt->max_xfer_size <= RNDIS_MSG_PAYLOAD_MAX);
  rndish_data[dev_addr-1].max_xfer_size = p_init_cmpt->max_xfer_size;

  //------------- Message Query 802.3 Permanent Address -------------//
  memcpy(msg_payload, &msg_query_permanent_addr, sizeof(rndis_msg_query_t));
  tu_memclr(msg_payload + sizeof(rndis_msg_query_t), 6); // 6 bytes for MAC address

  STASK_INVOKE(
      send_message_get_response_subtask( dev_addr, p_cdc,
                                         msg_payload, sizeof(rndis_msg_query_t) + 6,
                                         msg_payload),
      error
  );
  if ( TUSB_ERROR_NONE != error )   STASK_RETURN(error);

  rndis_msg_query_cmplt_t * const p_query_cmpt = (rndis_msg_query_cmplt_t *) msg_payload;
  STASK_ASSERT(p_query_cmpt->type == RNDIS_MSG_QUERY_CMPLT && p_query_cmpt->status == RNDIS_STATUS_SUCCESS);
  memcpy(rndish_data[dev_addr-1].mac_address, msg_payload + 8 + p_query_cmpt->buffer_offset, 6);

  //------------- Set OID_GEN_CURRENT_PACKET_FILTER to (DIRECTED | MULTICAST | BROADCAST) -------------//
  memcpy(msg_payload, &msg_set_packet_filter, sizeof(rndis_msg_set_t));
  tu_memclr(msg_payload + sizeof(rndis_msg_set_t), 4); // 4 bytes for filter flags
  ((rndis_msg_set_t*) msg_payload)->oid_buffer[0] = (RNDIS_PACKET_TYPE_DIRECTED | RNDIS_PACKET_TYPE_MULTICAST | RNDIS_PACKET_TYPE_BROADCAST);

  STASK_INVOKE(
      send_message_get_response_subtask( dev_addr, p_cdc,
                                         msg_payload, sizeof(rndis_msg_set_t) + 4,
                                         msg_payload),
      error
  );
  if ( TUSB_ERROR_NONE != error )   STASK_RETURN(error);

  rndis_msg_set_cmplt_t * const p_set_cmpt = (rndis_msg_set_cmplt_t *) msg_payload;
  STASK_ASSERT(p_set_cmpt->type == RNDIS_MSG_SET_CMPLT && p_set_cmpt->status == RNDIS_STATUS_SUCCESS);

  tusbh_cdc_rndis_mounted_cb(dev_addr);

  OSAL_SUBTASK_END
}

void rndish_xfer_isr(cdch_data_t *p_cdc, pipe_handle_t pipe_hdl, xfer_result_t event, uint32_t xferred_bytes)
{
  if ( pipehandle_is_equal(pipe_hdl, p_cdc->pipe_notification) )
  {
    osal_semaphore_post( rndish_data[pipe_hdl.dev_addr-1].sem_notification_hdl );
  }
}

//--------------------------------------------------------------------+
// INTERNAL & HELPER
//--------------------------------------------------------------------+
static tusb_error_t send_message_get_response_subtask( uint8_t dev_addr, cdch_data_t *p_cdc,
                                                       uint8_t * p_mess, uint32_t mess_length,
                                                       uint8_t *p_response)
{
  tusb_error_t error;

  OSAL_SUBTASK_BEGIN

  //------------- Send RNDIS Control Message -------------//
  STASK_INVOKE(
      usbh_control_xfer_subtask( dev_addr, bm_request_type(TUSB_DIR_OUT, TUSB_REQ_TYPE_CLASS, TUSB_REQ_RCPT_INTERFACE),
                                 CDC_REQUEST_SEND_ENCAPSULATED_COMMAND, 0, p_cdc->interface_number,
                                 mess_length, p_mess),
      error
  );
  if ( TUSB_ERROR_NONE != error )   STASK_RETURN(error);

  //------------- waiting for Response Available notification -------------//
  (void) usbh_edpt_xfer(p_cdc->pipe_notification, msg_notification[dev_addr-1], 8);
  osal_semaphore_wait(rndish_data[dev_addr-1].sem_notification_hdl, OSAL_TIMEOUT_NORMAL, &error);
  if ( TUSB_ERROR_NONE != error )   STASK_RETURN(error);
  STASK_ASSERT(msg_notification[dev_addr-1][0] == 1);

  //------------- Get RNDIS Message Initialize Complete -------------//
  STASK_INVOKE(
    usbh_control_xfer_subtask( dev_addr, bm_request_type(TUSB_DIR_IN, TUSB_REQ_TYPE_CLASS, TUSB_REQ_RCPT_INTERFACE),
                               CDC_REQUEST_GET_ENCAPSULATED_RESPONSE, 0, p_cdc->interface_number,
                               RNDIS_MSG_PAYLOAD_MAX, p_response),
    error
  );
  if ( TUSB_ERROR_NONE != error )   STASK_RETURN(error);

  OSAL_SUBTASK_END
}

//static tusb_error_t send_process_msg_initialize_subtask(uint8_t dev_addr, cdch_data_t *p_cdc)
//{
//  tusb_error_t error;
//
//  OSAL_SUBTASK_BEGIN
//
//  *((rndis_msg_initialize_t*) msg_payload) = (rndis_msg_initialize_t)
//                                            {
//                                                .type          = RNDIS_MSG_INITIALIZE,
//                                                .length        = sizeof(rndis_msg_initialize_t),
//                                                .request_id    = 1, // TODO should use some magic number
//                                                .major_version = 1,
//                                                .minor_version = 0,
//                                                .max_xfer_size = 0x4000 // TODO mimic windows
//                                            };
//
//
//
//  OSAL_SUBTASK_END
//}
#endif
