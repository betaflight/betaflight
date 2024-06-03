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
/**   PROLIFIC Class                                                      */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_prolific.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_prolific_transfer_request_completed  PORTABLE C      */ 
/*                                                           6.1.9        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is called by the completion thread when a transfer    */ 
/*    request has been completed either because the transfer is           */ 
/*    successful or there was an error.                                   */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    transfer_request                      Pointer to transfer request   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Transfer request              */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USBX stack                                                          */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  10-15-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            use pre-calculated value    */
/*                                            instead of wMaxPacketSize,  */
/*                                            resulting in version 6.1.9  */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_prolific_transfer_request_completed(UX_TRANSFER *transfer_request)
{

UX_HOST_CLASS_PROLIFIC                   *prolific;


    /* Get the class instance for this transfer request.  */
    prolific =  (UX_HOST_CLASS_PROLIFIC *) transfer_request -> ux_transfer_request_class_instance;
    
    /* Check the state of the transfer.  If there is an error, we do not proceed with this notification.  */
    if (transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)

        /* We do not proceed.  */
        return;

    /* Check if the class is in shutdown.  */
    if (prolific -> ux_host_class_prolific_state ==  UX_HOST_CLASS_INSTANCE_SHUTDOWN)

        /* We do not proceed.  */
        return;

    /* Increment the notification count.   */
    prolific -> ux_host_class_prolific_notification_count++;

    /* Look at what the the status is.  First ensure the length of our interrupt pipe data is correct.  */
    if (transfer_request -> ux_transfer_request_actual_length == 
                    transfer_request -> ux_transfer_request_requested_length)
    {

        /* Check if device is present.  */
        if ((*(transfer_request -> ux_transfer_request_data_pointer + UX_HOST_CLASS_PROLIFIC_DEVICE_STATE_OFFSET) & 
                UX_HOST_CLASS_PROLIFIC_DEVICE_STATE_MASK) == 0)
        
            /* Device is not present.  */
            prolific -> ux_host_class_prolific_device_state =  UX_HOST_CLASS_PROLIFIC_DEVICE_NOT_PRESENT;

        else            
        
            /* Device is present.  */
            prolific -> ux_host_class_prolific_device_state =  UX_HOST_CLASS_PROLIFIC_DEVICE_PRESENT;

        /* If there is a callback present, invoke it.  */
        if (prolific -> ux_host_class_prolific_device_status_change_callback != UX_NULL)
        
            /* There is a callback, send the status change to the application.  */
            prolific -> ux_host_class_prolific_device_status_change_callback(prolific, prolific -> ux_host_class_prolific_device_state);

    }                    

    /* Reactivate the PROLIFIC interrupt pipe.  */
    _ux_host_stack_transfer_request(transfer_request);

    /* Return to caller.  */
    return;
}

