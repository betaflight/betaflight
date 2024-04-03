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
/**   CDC_ACM Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_cdc_acm.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_cdc_acm_transfer_request_completed   PORTABLE C      */ 
/*                                                           6.1          */
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
/*    _ux_utility_short_get                 Get 16-bit value              */
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
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_cdc_acm_transfer_request_completed(UX_TRANSFER *transfer_request)
{

UX_HOST_CLASS_CDC_ACM                   *cdc_acm;
ULONG                                    notification_type;
ULONG                                    notification_value;


    /* Get the class instance for this transfer request.  */
    cdc_acm =  (UX_HOST_CLASS_CDC_ACM *) transfer_request -> ux_transfer_request_class_instance;
    
    /* Check the state of the transfer.  If there is an error, we do not proceed with this notification.  */
    if (transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)

        /* We do not proceed.  */
        return;

    /* Increment the notification count.   */
    cdc_acm -> ux_host_class_cdc_acm_notification_count++;

    /* Get the notification.  */
    notification_type = (ULONG) *(transfer_request -> ux_transfer_request_data_pointer + UX_HOST_CLASS_CDC_ACM_NPF_NOTIFICATION_TYPE);    
    
    /* And the value.  */
    notification_value = (ULONG) _ux_utility_short_get(transfer_request -> ux_transfer_request_data_pointer + UX_HOST_CLASS_CDC_ACM_NPF_VALUE);    
    
    /* If there is a callback present, invoke it.  */
    if (cdc_acm -> ux_host_class_cdc_acm_device_status_change_callback != UX_NULL)
    
        /* There is a callback, send the status change to the application.  */
        cdc_acm -> ux_host_class_cdc_acm_device_status_change_callback(cdc_acm, notification_type, notification_value);
        
    /* Reactivate the CDC_ACM interrupt pipe.  */
    _ux_host_stack_transfer_request(transfer_request);

    /* Return to caller.  */
    return;
}

