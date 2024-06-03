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
/**   Host Stack                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_stack_role_swap                            PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is called when the host or the device demand a role   */ 
/*    swap. This function may be called by an application or by the HNP   */ 
/*    thread.                                                             */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd                                       Pointer to HCD            */ 
/*    device                                    Device pointer            */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Status                                                              */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_semaphore_get                 Get semaphore             */
/*    _ux_host_stack_transfer_request           Transfer request          */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application of HNP thread.                                          */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            optimized based on compile  */
/*                                            definitions,                */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_role_swap(UX_DEVICE *device)
{

#if !defined(UX_OTG_SUPPORT)

    UX_PARAMETER_NOT_USED(device);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else

UX_ENDPOINT     *control_endpoint;
UX_TRANSFER     *transfer_request;
UINT            status;
        
    /* Retrieve the control endpoint and the transfer request associated with it.  */
    control_endpoint =  &device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Protect the control endpoint semaphore here.  It will be unprotected in the 
       transfer request function.  */
    status =  _ux_host_semaphore_get(&device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);

    /* Perform a SET_FEATURE on this device to inform the it we are ready to change role.  */
    transfer_request -> ux_transfer_request_requested_length =  0;
    transfer_request -> ux_transfer_request_function =          UX_SET_FEATURE;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_OUT | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value =             UX_OTG_FEATURE_B_HNP_ENABLE;
    transfer_request -> ux_transfer_request_index =             0;
    
    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* If the status fails, simply ignore the command but do not proceed.  */
    if (status != UX_SUCCESS)
    
        /* We have an error. Do not proceed. */
        return(status);

    /* Keep track of the event for HNP synchronization.  */
    _ux_system_otg -> ux_system_otg_change_mode_event = UX_OTG_HOST_TO_SLAVE;

    /* Call the OTG function that will perform the swap.  */
    _ux_system_otg -> ux_system_otg_function(UX_OTG_HOST_TO_SLAVE);
    
    /* Reset the event.  */
    _ux_system_otg -> ux_system_otg_change_mode_event = 0;
    
    return(UX_SUCCESS);
#endif
}
