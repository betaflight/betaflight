/**************************************************************************/
/*                                                                        */
/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*                                                                        */
/**************************************************************************/

/**************************************************************************/
/**************************************************************************/
/**                                                                       */ 
/** USBX Component                                                        */ 
/**                                                                       */
/**   Device RNDIS Class                                                  */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_rndis.h"
#include "ux_device_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_rndis_msg_keep_alive               PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function analyzes and replies to the MSG KEEP_ALIVE            */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    rndis                           Pointer to rndis class              */ 
/*    transfer_request                Pointer to the transfer request     */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_long_get            Get 32-bit value                    */
/*    _ux_utility_long_put            Put 32-bit value                    */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    RNDIS Class                                                         */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_rndis_msg_keep_alive(UX_SLAVE_CLASS_RNDIS *rndis, UX_SLAVE_TRANSFER *transfer_request)
{

UCHAR            *rndis_msg;
UCHAR            *rndis_response;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_RNDIS_MSG_KEEP_ALIVE, rndis, 0, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Get the pointer to the RNDIS message.  */
    rndis_msg = transfer_request -> ux_slave_transfer_request_data_pointer;
    
    /* Get the request ID and keep it for the response.  */
    rndis -> ux_slave_class_rndis_request_id =  _ux_utility_long_get(rndis_msg + UX_DEVICE_CLASS_RNDIS_MSG_KEEP_ALIVE_REQUEST_ID);
    
    /* Now prepare the response.  */
    rndis_response = rndis -> ux_slave_class_rndis_response;
    
    /* First store the command. */
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_KEEP_ALIVE_MESSAGE_TYPE, UX_DEVICE_CLASS_RNDIS_CMPLT_KEEP_ALIVE);
    
    /* Then the length of the response. */
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_KEEP_ALIVE_MESSAGE_LENGTH, UX_DEVICE_CLASS_RNDIS_CMPLT_KEEP_ALIVE_RESPONSE_LENGTH);

    /* Store the request ID.  */
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_KEEP_ALIVE_REQUEST_ID, rndis -> ux_slave_class_rndis_request_id);
        
    /* Force the status to SUCCESS.  */
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_KEEP_ALIVE_STATUS, UX_DEVICE_CLASS_RNDIS_STATUS_SUCCESS);
        
    /* Set the response length.  */
    rndis -> ux_slave_class_rndis_response_length =   UX_DEVICE_CLASS_RNDIS_CMPLT_KEEP_ALIVE_RESPONSE_LENGTH;

    /* We are done. Return UX_SUCCESS.  */
    return(UX_SUCCESS);        
}

