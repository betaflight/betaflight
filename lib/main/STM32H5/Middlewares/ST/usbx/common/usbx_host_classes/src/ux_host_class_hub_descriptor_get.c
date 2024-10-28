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


#if UX_MAX_DEVICES > 1

#if defined(UX_HOST_STANDALONE)
UINT _ux_host_class_hub_descriptor_parse(UX_HOST_CLASS_HUB *hub, UCHAR *descriptor);
UINT
#else
static inline UINT
#endif
_ux_host_class_hub_descriptor_parse(UX_HOST_CLASS_HUB *hub, UCHAR *descriptor)
{
UINT            status = UX_SUCCESS;
ULONG           tt_protocols; /* (bDeviceProtocol << 8) | bInterfaceProtocol  */
ULONG           port_index;

    /* Parse the device descriptor and create the local descriptor.  */
    _ux_utility_descriptor_parse(descriptor, _ux_system_hub_descriptor_structure, UX_HUB_DESCRIPTOR_ENTRIES,
                                                            (UCHAR *) &hub -> ux_host_class_hub_descriptor);

    /* Check the protocol used by the HUB. This will indicate if the HUB supports multiple TTs in high speed mode.  */
    tt_protocols = (hub -> ux_host_class_hub_device -> ux_device_descriptor.bDeviceProtocol << 8) |
                    hub -> ux_host_class_hub_interface -> ux_interface_descriptor.bInterfaceProtocol;
    switch(tt_protocols)
    {

    /* 1. A hub operating at full-/low-speed has a device descriptor with a bDeviceProtocol field set to zero(0)
     *    and an interface descriptor with a bInterfaceProtocol field set to zero(0).
     */
    case (UX_HOST_CLASS_HUB_PROTOCOL_FS << 8):

        /* In the case of full speed hub, there are no TTs to declare */
        break;

    /* 2. A hub that has a single TT must set the bDeviceProtocol field of the device descriptor to one(1) and
     *    the interface descriptor bInterfaceProtocol field set to 0.
     * 3. A multiple TT hub must set the bDeviceProtocol field of the device descriptor to two (2).
     *    The first interface descriptor has the bInterfaceProtocol field set to one(1).
     */
    case (UX_HOST_CLASS_HUB_PROTOCOL_SINGLE_TT << 8): /* Fall through.  */
    case (UX_HOST_CLASS_HUB_PROTOCOL_MULTIPLE_TT << 8) | UX_HOST_CLASS_HUB_PROTOCOL_SINGLE_TT:

        /* Single TT hub or single TT interface.  */
        /* Check if current setting supports this Hub.  */
        if (hub -> ux_host_class_hub_descriptor.bNbPorts > UX_MAX_TT)
        {

            status = UX_TOO_MANY_HUB_PORTS;
            break;
        }

        /* In a single TT state, all the downstream ports report to the same
           TT and share the 1.1 USB segment bandwidth. This is a very crude
           but working method, we simply set all the ports bits to the first
           TT.  */
        hub -> ux_host_class_hub_device -> ux_device_hub_tt[0].ux_hub_tt_port_mapping =   UX_TT_MASK;
        hub -> ux_host_class_hub_device -> ux_device_hub_tt[0].ux_hub_tt_max_bandwidth =  UX_TT_BANDWIDTH;
        break;

    /* 3. A multiple TT hub must set the bDeviceProtocol field of the device descriptor to two (2).
        *    The first interface descriptor has the bInterfaceProtocol field set to one(1).
        *    Such a hub also has a second interface descriptor where the bInterfaceProtocol is set to two(2).
        */
    case (UX_HOST_CLASS_HUB_PROTOCOL_MULTIPLE_TT << 8) | UX_HOST_CLASS_HUB_PROTOCOL_MULTIPLE_TT:

        /* Multiple TTs.  */
        /* Check if current setting supports this Hub.  */
        if (hub -> ux_host_class_hub_descriptor.bNbPorts > UX_MAX_TT)
        {

            status = UX_TOO_MANY_HUB_PORTS;
            break;
        }

        /* In the case of multiple TTs, each downstream port can sustain the USB 1.1
            max bandwidth and therefore we allocate one TT per port with that bandwidth.  */
        for (port_index = 0; port_index < hub -> ux_host_class_hub_descriptor.bNbPorts; port_index++)
        {

            hub -> ux_host_class_hub_device -> ux_device_hub_tt[port_index].ux_hub_tt_port_mapping =   (ULONG)(1 << port_index);
            hub -> ux_host_class_hub_device -> ux_device_hub_tt[port_index].ux_hub_tt_max_bandwidth =  UX_TT_BANDWIDTH;
        }
        break;

    default: /* Invalid bDeviceProtocol and bInterfaceProtocol pair.  */

        /* We should never get here. In this case the protocol value of the HUB is illegal.  */
        status =  UX_DESCRIPTOR_CORRUPTED;
    }

    /* Return status.  */
    return(status);
}
#endif


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hub_descriptor_get                   PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function obtains the HUB descriptor. This descriptor contains  */ 
/*    the number of downstream ports and the power characteristics of the */ 
/*    HUB (self powered or bus powered).                                  */ 
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
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*    _ux_utility_memory_free               Release memory block          */ 
/*    _ux_utility_descriptor_parse          Parse descriptor              */ 
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
/*                                            optimized based on compile  */
/*                                            definitions,                */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            improved protocol handling, */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hub_descriptor_get(UX_HOST_CLASS_HUB *hub)
{

UCHAR           *descriptor;
UX_ENDPOINT     *control_endpoint;
UX_TRANSFER     *transfer_request;
UINT            status;


    /* We need to get the default control endpoint transfer request pointer.  */
    control_endpoint =  &hub -> ux_host_class_hub_device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Need to allocate memory for the descriptor.  */
    descriptor =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_HUB_DESCRIPTOR_LENGTH);
    if (descriptor == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Create a transfer request for the GET_DESCRIPTOR request.  */
    transfer_request -> ux_transfer_request_data_pointer =      descriptor;
    transfer_request -> ux_transfer_request_requested_length =  UX_HUB_DESCRIPTOR_LENGTH;
    transfer_request -> ux_transfer_request_function =          UX_HOST_CLASS_HUB_GET_DESCRIPTOR;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value =             (UX_HUB_DESCRIPTOR_ITEM << 8);
    transfer_request -> ux_transfer_request_index =             0;

#if defined(UX_HOST_STANDALONE)

    /* Save allocated buffer pointer.  */
    hub -> ux_host_class_hub_allocated = descriptor;

    /* Link to running transfer.  */
    UX_TRANSFER_STATE_RESET(transfer_request);
    hub -> ux_host_class_hub_transfer = transfer_request;
    status = UX_SUCCESS;
#else

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Did the transfer succeed?  */
    if (status == UX_SUCCESS)
    {

        /* Is the length valid?  */
        if (transfer_request -> ux_transfer_request_actual_length == UX_HUB_DESCRIPTOR_LENGTH)
        {

#if UX_MAX_DEVICES > 1
            status = _ux_host_class_hub_descriptor_parse(hub, descriptor);
            if (status != UX_SUCCESS)
            {

                /* Error trap. */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HUB, status);

                /* If trace is enabled, insert this event into the trace buffer.  */
                UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, status, descriptor, 0, 0, UX_TRACE_ERRORS, 0, 0)
            }
#endif
        }
        else
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HUB, UX_DESCRIPTOR_CORRUPTED);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, descriptor, 0, 0, UX_TRACE_ERRORS, 0, 0)

            /* The descriptor must be corrupted if we got an invalid length.  */
            status =  UX_DESCRIPTOR_CORRUPTED;
        }
    }

    /* Free the memory for the descriptor.  */
    _ux_utility_memory_free(descriptor);
#endif

    /* Return completion status.  */
    return(status);
}
