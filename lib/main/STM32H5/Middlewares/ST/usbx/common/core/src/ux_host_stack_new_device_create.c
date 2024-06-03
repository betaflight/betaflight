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
/*    _ux_host_stack_new_device_create                    PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function creates a new device on the USB. It may be called     */
/*    either by a hub or by the root hub.                                 */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    HCD                                   HCD that owns that device     */
/*    device_owner                          Either a root hub instance    */
/*                                            or a hub instance           */
/*    port_index                            Port the new device is mounted*/
/*    device_speed                          Speed at which the device is  */
/*                                            running (low, full, high)   */
/*    port_power_available                  Power available on the root   */
/*                                            or hub port. This value is  */
/*                                            used to ensure that the     */
/*                                            device can be configured    */
/*                                            without creating an         */
/*                                            OVER_CURRENT condition on   */
/*                                            the bus                     */
/*    created_device                        Destination to fill created   */
/*                                            device instance             */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*    An Error code could be an indication that the device could not be   */
/*    created in which case the error is fatal to the enumeration and     */
/*    will not be retried. The error code can also indicate a failure     */
/*    for the device to respond to the GET_DEVICE_DESCRIPTOR. This error  */
/*    is not fatal and the caller should retry the enumeration process    */
/*    after the port to which the device is attached has been reset.      */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_stack_class_device_scan      Scan class devices            */
/*    _ux_host_stack_class_interface_scan   Scan class interfaces         */
/*    _ux_host_stack_device_address_set     Set device address            */
/*    _ux_host_stack_device_descriptor_read Read device descriptor        */
/*    _ux_host_stack_configuration_enumerate                              */
/*                                          Enumerate device config       */
/*    _ux_host_stack_new_device_get         Get new device                */
/*    _ux_utility_semaphore_create          Create a semaphore            */
/*    (ux_hcd_entry_function)               HCD entry function            */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
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
/*  02-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added a new parameter to    */
/*                                            return created device,      */
/*                                            resulting in version 6.1.4  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            reset device power source,  */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            freed shared device config  */
/*                                            descriptor after enum scan, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_new_device_create(UX_HCD *hcd, UX_DEVICE *device_owner,
                                UINT port_index, UINT device_speed,
                                UINT port_max_power,
                                UX_DEVICE **created_device)
{

UX_DEVICE           *device;
UINT                status;
UX_ENDPOINT         *control_endpoint;


#if UX_MAX_DEVICES > 1
    /* Verify the number of devices attached to the HCD already. Normally a HCD
       can have up to 127 devices but that can be tailored.  */
    if (hcd -> ux_hcd_nb_devices > UX_MAX_USB_DEVICES)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_TOO_MANY_DEVICES);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TOO_MANY_DEVICES, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_TOO_MANY_DEVICES);
    }
#endif

    /* Get a new device container to store this new device.  */
    device =  _ux_host_stack_new_device_get();
    if (device == UX_NULL)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_TOO_MANY_DEVICES);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TOO_MANY_DEVICES, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_TOO_MANY_DEVICES);
    }

    /* Store the device instance.  */
    *created_device = device;

    /* Increment the number of devices on this bus.  */
    hcd -> ux_hcd_nb_devices++;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_NEW_DEVICE_CREATE, hcd, device_owner, port_index, device, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    /* At this stage the device is attached but not configured.
       we don't have to worry about power consumption yet.
       Initialize the device structure.  */
    device -> ux_device_handle =         (ULONG) (ALIGN_TYPE) device;
    device -> ux_device_state =          UX_DEVICE_ATTACHED;
    device -> ux_device_address =        0;
    device -> ux_device_speed =          device_speed;
    UX_DEVICE_MAX_POWER_SET(device, port_max_power);
    UX_DEVICE_PARENT_SET(device, device_owner);
    UX_DEVICE_HCD_SET(device, hcd);
    UX_DEVICE_PORT_LOCATION_SET(device, port_index);
    device -> ux_device_power_source =   UX_DEVICE_BUS_POWERED;

    /* Create a semaphore for the device. This is to protect endpoint 0 mostly for OTG HNP polling. The initial count is 1 as
       a mutex mechanism.  */
    status =  _ux_host_semaphore_create(&device -> ux_device_protection_semaphore, "ux_host_endpoint0_semaphore", 1);

    /* Check semaphore creation.  */
    if (status != UX_SUCCESS)
    {

        /* Return error. Device resources that have been allocated until this
           point should be freed by the caller via _ux_host_stack_device_resources_free.  */
        return(UX_SEMAPHORE_ERROR);
    }

    /* Initialize the default control endpoint permanently attached
       to the device.  */
    control_endpoint =                               &device -> ux_device_control_endpoint;
    control_endpoint -> ux_endpoint =                (ULONG) (ALIGN_TYPE) control_endpoint;
    control_endpoint -> ux_endpoint_next_endpoint =  UX_NULL;
    control_endpoint -> ux_endpoint_interface =      UX_NULL;
    control_endpoint -> ux_endpoint_device =         device;
    control_endpoint -> ux_endpoint_transfer_request.ux_transfer_request_endpoint = control_endpoint;

    /* Create a semaphore for this endpoint to be attached to its transfer request.  */
    status =  _ux_host_semaphore_create(&control_endpoint -> ux_endpoint_transfer_request.ux_transfer_request_semaphore, "ux_host_transfer_request_semaphore", 0);

    /* Check semaphore creation.  */
    if (status != UX_SUCCESS)
    {

        /* Return error. Device resources that have been allocated until this
           point should be freed by the caller via _ux_host_stack_device_resources_free.  */
        return(UX_SEMAPHORE_ERROR);
    }

    /* If the device is running in high speed the default max packet size for the control endpoint is 64.
       All other speeds the size is 8.  */
    if (device_speed == UX_HIGH_SPEED_DEVICE)
        control_endpoint -> ux_endpoint_descriptor.wMaxPacketSize =  UX_DEFAULT_HS_MPS;
    else
        control_endpoint -> ux_endpoint_descriptor.wMaxPacketSize =  UX_DEFAULT_MPS;

    /* Create the default control endpoint at the HCD level.  */
    status =  hcd -> ux_hcd_entry_function(hcd, UX_HCD_CREATE_ENDPOINT, (VOID *) control_endpoint);

#if defined(UX_HOST_STANDALONE)
    if (status == UX_SUCCESS)
    {

        /* Now control endpoint is ready, set state to running. */
        control_endpoint -> ux_endpoint_state =          UX_ENDPOINT_RUNNING;

        /* Setup default control request timeout.  */
        control_endpoint -> ux_endpoint_transfer_request.ux_transfer_request_timeout_value =
                            UX_MS_TO_TICK_NON_ZERO(UX_CONTROL_TRANSFER_TIMEOUT);

        /* Set default control request mode to wait.  */
        control_endpoint -> ux_endpoint_transfer_request.ux_transfer_request_flags |=
                                                    UX_TRANSFER_FLAG_AUTO_WAIT;
    }

    /* Enumeration steps will be done in task state machine.  */

#else

    /* Going on to do enumeration (requests).  */
    if (status == UX_SUCCESS)
    {

        /* Now control endpoint is ready, set state to running. */
        control_endpoint -> ux_endpoint_state =          UX_ENDPOINT_RUNNING;

        /* Set the address of the device. The first time a USB device is
           accessed, it responds to the address 0. We need to change the address
           to a free device address between 1 and 127 ASAP.  */
        status =  _ux_host_stack_device_address_set(device);
        if (status == UX_SUCCESS)
        {

            /* Get the device descriptor.  */
            status =  _ux_host_stack_device_descriptor_read(device);
            if (status == UX_SUCCESS)
            {

                /* Get the configuration descriptor(s) for the device
                   and parse all the configuration, interface, endpoints...  */
                status =  _ux_host_stack_configuration_enumerate(device);
            }
        }
    }

    /* Check the status of the previous operations. If there was an
       error during any of the phases, the device resources must be
       freed based on if we want to retry.  */
    if (status == UX_SUCCESS)
    {

        /* The device, configuration(s), interface(s), endpoint(s) are
           now in order for this device to work. No configuration is set
           yet. First we need to find a class driver that wants to own
           it. There is no need to have an orphan device in a configured state.   */
        status =  _ux_host_stack_class_device_scan(device);
        if (status == UX_NO_CLASS_MATCH)
        {

            status =  _ux_host_stack_class_interface_scan(device);

        }

        /* Check if there is unnecessary resource to free.  */
        if (device -> ux_device_packed_configuration &&
            device -> ux_device_packed_configuration_keep_count == 0)
        {
            _ux_utility_memory_free(device -> ux_device_packed_configuration);
            device -> ux_device_packed_configuration = UX_NULL;
        }

        /* If trace is enabled, register this object.  */
        UX_TRACE_OBJECT_REGISTER(UX_TRACE_HOST_OBJECT_TYPE_DEVICE, hcd, device_owner, port_index, 0);
    }
#endif

    /* Return status. If there's an error, device resources that have been 
       allocated until this point should be freed by the caller via _ux_host_stack_device_resources_free.  */
    return(status);
}
