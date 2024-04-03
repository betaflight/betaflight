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
/**   Hub Class                                                           */
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
/*    _ux_host_class_hub_change_process                   PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is called by the topology thread when there has been  */
/*    activity on the HUB.                                                */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hub                                   Pointer to HUB                */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_hub_port_change_process Port change process          */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hub_change_process(UX_HOST_CLASS_HUB *hub)
{

UX_TRANSFER     *transfer_request;
USHORT          port_status_change_bits;
UINT            port_index;
UINT            status;
    

    /* Now get the transfer_request attached to the interrupt endpoint.  */
    transfer_request =  &hub -> ux_host_class_hub_interrupt_endpoint -> ux_endpoint_transfer_request;

    /* The interrupt pipe buffer contains the status change for each of the ports 
       the length of the buffer can be 1 or 2 depending on the number of ports.
       Usually, since HUBs can be bus powered the maximum number of ports is 4. 
       We must be taking precautions on how we read the buffer content for
       big endian machines.  */
    if (transfer_request -> ux_transfer_request_actual_length == 1)
        port_status_change_bits =  (USHORT) *transfer_request -> ux_transfer_request_data_pointer;
    else
        port_status_change_bits =  (USHORT)_ux_utility_short_get(transfer_request -> ux_transfer_request_data_pointer);

    /* Scan all bits and report the change on each port.  */
    for (port_index = 1; port_index <= hub -> ux_host_class_hub_descriptor.bNbPorts; port_index++)
    {

        if (port_status_change_bits & (1<<port_index))
            _ux_host_class_hub_port_change_process(hub, port_index);
    }

    /* The HUB could also have changed.  */
    if (port_status_change_bits & 1)
        _ux_host_class_hub_hub_change_process(hub);
        
    /* The actual length should be cleared for the next transfer.  */
    transfer_request -> ux_transfer_request_actual_length =  0;

    /* Resend the request to the stack.  */
    status = _ux_host_stack_transfer_request(transfer_request);

    /* Return completion status.  */
    return(status);
}
