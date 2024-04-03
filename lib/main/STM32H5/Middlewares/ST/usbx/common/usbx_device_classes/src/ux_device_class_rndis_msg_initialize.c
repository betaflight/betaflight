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
/*    _ux_device_class_rndis_msg_initialize               PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function analyzes and replies to the MSG INITIALIZE            */ 
/*    endpoints with a CLASS or VENDOR SPECIFIC type.                     */ 
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
UINT  _ux_device_class_rndis_msg_initialize(UX_SLAVE_CLASS_RNDIS *rndis, UX_SLAVE_TRANSFER *transfer_request)
{

UCHAR            *rndis_msg;
UCHAR            *rndis_response;

    /* Get the pointer to the RNDIS message.  */
    rndis_msg = transfer_request -> ux_slave_transfer_request_data_pointer;
    
    /* Get the request ID and keep it for the response.  */
    rndis -> ux_slave_class_rndis_request_id =  _ux_utility_long_get(rndis_msg + UX_DEVICE_CLASS_RNDIS_MSG_INITIALIZE_REQUEST_ID);
    
    /* Get the major version and store it into the RNDIS instance.  */
    rndis -> ux_slave_class_rndis_major_version =  _ux_utility_long_get(rndis_msg + UX_DEVICE_CLASS_RNDIS_MSG_INITIALIZE_MAJOR_VERSION);
    
    /* Get the minor version and store it into the RNDIS instance.  */
    rndis -> ux_slave_class_rndis_minor_version =  _ux_utility_long_get(rndis_msg + UX_DEVICE_CLASS_RNDIS_MSG_INITIALIZE_MINOR_VERSION);
    
    /* Get the max transfer size and store it into the RNDIS instance.  */
    rndis -> ux_slave_class_rndis_max_transfer_size =  _ux_utility_long_get(rndis_msg + UX_DEVICE_CLASS_RNDIS_MSG_INITIALIZE_MAX_TRANSFER_SIZE);

    /* Store the state machine to initialized.  */
    rndis -> ux_slave_class_rndis_state =  UX_DEVICE_CLASS_RNDIS_STATE_INITIALIZED;

    /* Now prepare the response.  */
    rndis_response = rndis -> ux_slave_class_rndis_response;
    
    /* First store the command. */
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_MESSAGE_TYPE, UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE);
    
    /* Then the length of the response. */
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_MESSAGE_LENGTH, UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_RESPONSE_LENGTH);

    /* Store the request ID.  */
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_REQUEST_ID, rndis -> ux_slave_class_rndis_request_id);
        
    /* Force the status to SUCCESS.  */
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_STATUS, UX_DEVICE_CLASS_RNDIS_STATUS_SUCCESS);
        
    /* Set the major version of the device  */
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_MAJOR_VERSION, UX_DEVICE_CLASS_RNDIS_VERSION_MAJOR);
        
    /* Set the minor version of the device  */
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_MINOR_VERSION, UX_DEVICE_CLASS_RNDIS_VERSION_MINOR);

    /* Set the type of connection supported.  */
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_DEVICE_FLAGS, UX_DEVICE_CLASS_RNDIS_DF_CONNECTION_SUPPORTED);
        
    /* Set the type of media supported.  */
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_MEDIUM, UX_DEVICE_CLASS_RNDIS_MEDIUM_SUPPORTED);
        
    /* Set the max packet per transfer.  */
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_MAX_PACKETS_PER_TRANSFER, UX_DEVICE_CLASS_RNDIS_MAX_PACKET_PER_TRANSFER);
        
    /* Set the max transfer size.  */
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_MAX_TRANSFER_SIZE, UX_DEVICE_CLASS_RNDIS_MAX_PACKET_TRANSFER_SIZE);
        
    /* Set the packet alignment factor.  */
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_PACKET_ALIGNMENT, UX_DEVICE_CLASS_RNDIS_PACKET_ALIGNEMENT_FACTOR);
        
    /* Set AFListOffset and AFListSize fields to 0.  */
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_AFL_LIST_OFFSET, 0);
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_AFL_LIST_SIZE, 0);
            
    /* Set the response length.  */
    rndis -> ux_slave_class_rndis_response_length =   UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_RESPONSE_LENGTH;

    /* We are done. Return UX_SUCCESS.  */
    return(UX_SUCCESS);        
}

