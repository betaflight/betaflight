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
/**   HUB Class                                                           */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_hub.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hub_feature                          PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will send a command to the HUB on a specific port.    */
/*    The commands can be SET_FEATURE or CLEAR_FEATURE.                   */ 
/*                                                                        */
/*    In standalone mode, this functioin prepares the control transfer    */
/*    request context of specific command, for host stack transfer        */
/*    function to process, in next steps.                                 */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hub                                   Pointer to HUB class          */ 
/*    port                                  Port number                   */ 
/*    command                               Command to send               */ 
/*    feature                               Feature to send               */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    HUB Class                                                           */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hub_feature(UX_HOST_CLASS_HUB *hub, UINT port, UINT command, UINT function)
{

UX_ENDPOINT     *control_endpoint;
UX_TRANSFER     *transfer_request;
UINT            target;
UINT            status;


    /* We need to get the default control endpoint transfer request pointer.  */
    control_endpoint =  &hub -> ux_host_class_hub_device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* The target is DEVICE for the HUB and OTHER for the downstream ports.  */
    if (port == 0)
        target =  UX_REQUEST_TARGET_DEVICE;        
    else
        target =  UX_REQUEST_TARGET_OTHER;

    /* Create a transfer request for the SET_FEATURE or CLEAR_FEATURE request.  */
    transfer_request -> ux_transfer_request_function =          command;
    transfer_request -> ux_transfer_request_requested_length =  0;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_OUT | UX_REQUEST_TYPE_CLASS | target;
    transfer_request -> ux_transfer_request_value =             function;
    transfer_request -> ux_transfer_request_index =             port;

#if defined(UX_HOST_STANDALONE)

    /* Reset transfer state for _run.  */
    UX_TRANSFER_STATE_RESET(transfer_request);
    status = UX_SUCCESS;
#else

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);
#endif

    /* Return completion status.  */
    return(status);
}
