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
/*    _ux_host_stack_device_remove                        PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will remove a USB device from the bus.                */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    HCD                                   Pointer to the HCD            */
/*    parent                                The parent device address     */
/*    port_index                            Index of the port on which the*/
/*                                            change of status occurred   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_stack_device_resources_free  Free all device resources     */
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
/*                                            used pointer for current    */
/*                                            selected configuration,     */
/*                                            added notification for      */
/*                                            device disconnection,       */
/*                                            resulting in version 6.1.4  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_device_remove(UX_HCD *hcd, UX_DEVICE *parent, UINT port_index)
{

#if UX_MAX_DEVICES > 1
ULONG                       container_index;
#endif
UX_DEVICE                   *device;
UX_CONFIGURATION            *configuration;
UX_INTERFACE                *interface_ptr;
UX_HOST_CLASS_COMMAND       command;

    /* We need to find the device descriptor for the removed device. We can find it
       with the parent device and the port it was attached to. Start with the first device.  */
    device =  _ux_system_host -> ux_system_host_device_array;

#if UX_MAX_DEVICES > 1
    /* Start at the beginning of the list.  */
    container_index =  0;

    /* Search the list until the end.  */
    while (container_index++ < _ux_system_host -> ux_system_host_max_devices)
    {

        /* Until we have found a used entry.  */
        if (device -> ux_device_handle != UX_UNUSED)
        {

            /* Check for the parent device and the port location and the controller.  */
            if( UX_DEVICE_PARENT_MATCH(device, parent) &&
                UX_DEVICE_PORT_LOCATION_MATCH(device, port_index) &&
                UX_DEVICE_HCD_MATCH(device, hcd))
                break;
        }

        /* Move to the next device entry.  */
        device++;
    }

    /* Device not found.  */
    if (container_index > _ux_system_host -> ux_system_host_max_devices)
#else
    UX_PARAMETER_NOT_USED(parent);
    UX_PARAMETER_NOT_USED(port_index);

    /* Device is available and HCD is expected.  */
    if (device -> ux_device_handle == UX_UNUSED ||
        !UX_DEVICE_HCD_MATCH(device, hcd))
#endif
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_DEVICE_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DEVICE_HANDLE_UNKNOWN, device, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* We get here when we could not find the device.  */
        return(UX_DEVICE_HANDLE_UNKNOWN);
    }

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_DEVICE_REMOVE, hcd, parent, port_index, device, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    /* If trace is enabled, unregister this object.  */
    UX_TRACE_OBJECT_UNREGISTER(device);

    /* We have found the device to be removed. */
    device -> ux_device_state = UX_DEVICE_REMOVED;

    /* We have found the device to be removed. Initialize the class
        command with the generic parameters.  */
    command.ux_host_class_command_request =  UX_HOST_CLASS_COMMAND_DEACTIVATE;

    /* The device may have a class associated with the device container or its interfaces.  */
    if (device -> ux_device_class_instance != UX_NULL)
    {

        /* We need to stop the class instance for the device.  */
        command.ux_host_class_command_instance =  device -> ux_device_class_instance;

        /* Call the class.  */
        device -> ux_device_class -> ux_host_class_entry_function(&command);
    }
    else
    {

        /* Search for the active configuration.  */
        configuration =  device -> ux_device_current_configuration;

        /* If configuration is activated.  */
        if (configuration != UX_NULL)
        {

            /* We have the correct configuration, search the interface(s).  */
            interface_ptr =  configuration -> ux_configuration_first_interface;

            /* Loop to perform the search.  */
            while (interface_ptr != UX_NULL)
            {

                /* Check if an instance of the interface is present.  */
                if (interface_ptr -> ux_interface_class_instance != UX_NULL)
                {

                    /* We need to stop the class instance for the device.  */
                    command.ux_host_class_command_instance =  interface_ptr -> ux_interface_class_instance;

                    /* Call the class.  */
                    interface_ptr -> ux_interface_class -> ux_host_class_entry_function(&command);
                }

                /* Move to next interface.  */
                interface_ptr =  interface_ptr -> ux_interface_next_interface;
            }
        }
    }

    /* Notify application for disconnection of existing physical device.  */
    if (_ux_system_host -> ux_system_host_change_function)
    {
        _ux_system_host -> ux_system_host_change_function(UX_DEVICE_DISCONNECTION, UX_NULL, (VOID*)device);
    }

    /* Now all the resources for this device must be free.  */
    _ux_host_stack_device_resources_free(device);

    /* Decrement the number of devices on this bus.  */
    hcd -> ux_hcd_nb_devices--;

    /* We are done with this device removal.  */
    return(UX_SUCCESS);
}
