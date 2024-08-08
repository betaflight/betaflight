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
/*    _ux_host_class_hub_status_get                       PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will do a GET_STATUS from the HUB. This function      */ 
/*    retrieves the current status and the changed bits.                  */ 
/*                                                                        */
/*    In standalone mode, this functioin prepares the control transfer    */
/*    request data memory and request context of specific command, for    */
/*    host stack transfer function to process, in next steps. The         */
/*    allocated memory must be freed after transfer request is processed. */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hub                                   Pointer to HUB class          */ 
/*    port                                  Port of device                */ 
/*    port_status                           Destination for port status   */ 
/*    port_change                           Destination for port change   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*    _ux_utility_memory_free               Release memory block          */ 
/*    _ux_utility_short_get                 Get 16-bit word               */ 
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
UINT  _ux_host_class_hub_status_get(UX_HOST_CLASS_HUB *hub, UINT port, USHORT *port_status, USHORT *port_change)
{

UCHAR           *port_data;
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

    /* Allocate a buffer for the port status and change: 2 words.  */        
    port_data =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, 4);
    if(port_data == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Create a transfer request for the GET_STATUS request.  */
    transfer_request -> ux_transfer_request_requested_length =  4;
    transfer_request -> ux_transfer_request_data_pointer =      port_data;
    transfer_request -> ux_transfer_request_function =          UX_HOST_CLASS_HUB_GET_STATUS;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | target;
    transfer_request -> ux_transfer_request_value =             0;
    transfer_request -> ux_transfer_request_index =             port;

#if defined(UX_HOST_STANDALONE)

    /* No status change change copy.  */
    UX_PARAMETER_NOT_USED(port_status);
    UX_PARAMETER_NOT_USED(port_change);

    /* Save allocated buffer.  */
    hub -> ux_host_class_hub_allocated = port_data;

    /* Reset transfer state for _run.  */
    UX_TRANSFER_STATE_RESET(transfer_request);
    status = UX_SUCCESS;
#else

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check for error and completion of the transfer.  */
    if (status == UX_SUCCESS)
    {

        if (transfer_request -> ux_transfer_request_actual_length == 4)
        {

            /* The 2 words are now in the buffer. We need to resolve their endianness
               and report them as separate items to the called.  */
            *port_status =  (USHORT)_ux_utility_short_get(port_data);
            *port_change =  (USHORT)_ux_utility_short_get(port_data+2);
        }
        else
        {

            /* Invalid length. Return error.  */
            status =  UX_TRANSFER_DATA_LESS_THAN_EXPECTED;
        }
    }

    /* Free the buffer resource now.  */
    _ux_utility_memory_free(port_data);

#endif

    /* Return completion status.  */
    return(status);
}
