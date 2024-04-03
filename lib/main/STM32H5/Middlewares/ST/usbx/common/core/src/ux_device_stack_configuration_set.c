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
/**   Device Stack                                                        */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_stack_configuration_set                  PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function sets the configuration from the host and will enable  */
/*    the default alternate setting 0 for all the interfaces attached to  */
/*    this configuration.                                                 */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    endpoint                              Pointer to endpoint           */
/*    configuration_value                   Configuration selected        */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */
/*    (ux_slave_class_entry_function)       Device class entry function   */ 
/*    (ux_slave_dcd_function)               DCD dispatch function         */ 
/*    _ux_device_stack_interface_delete     Delete interface              */
/*    _ux_device_stack_interface_set        Set interface                 */ 
/*    _ux_utility_descriptor_parse          Parse descriptor              */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
/*    Device Stack                                                        */
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
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_stack_configuration_set(ULONG configuration_value)
{

UX_SLAVE_DCD                    *dcd;
UCHAR *                         device_framework;
ULONG                           device_framework_length;
ULONG                           descriptor_length;
UCHAR                           descriptor_type;
UX_CONFIGURATION_DESCRIPTOR     configuration_descriptor = { 0 };
UX_INTERFACE_DESCRIPTOR         interface_descriptor;
UX_SLAVE_INTERFACE              *interface_ptr; 
#if !defined(UX_DEVICE_INITIALIZE_FRAMEWORK_SCAN_DISABLE) || UX_MAX_DEVICE_INTERFACES > 1
UX_SLAVE_INTERFACE              *next_interface; 
#endif
UX_SLAVE_CLASS                  *class_inst;
UX_SLAVE_CLASS                  *current_class =  UX_NULL;
UX_SLAVE_CLASS_COMMAND          class_command;
UX_SLAVE_DEVICE                 *device;
ULONG                           iad_flag;
ULONG                           iad_first_interface =  0;
ULONG                           iad_number_interfaces =  0;
#if UX_MAX_SLAVE_CLASS_DRIVER > 1
ULONG                           class_index;
#endif


    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_STACK_CONFIGURATION_SET, configuration_value, 0, 0, 0, UX_TRACE_DEVICE_STACK_EVENTS, 0, 0)

    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;
    
    /* Reset the IAD flag.  */
    iad_flag =  UX_FALSE;

    /* If the configuration value is already selected, keep it.  */
    if (device -> ux_slave_device_configuration_selected == configuration_value)
        return(UX_SUCCESS);

    /* We may have multiple configurations !, the index will tell us what
       configuration descriptor we need to return.  */
    device_framework = _ux_system_slave -> ux_system_slave_device_framework;
    device_framework_length =  _ux_system_slave -> ux_system_slave_device_framework_length;

    /* Parse the device framework and locate a configuration descriptor.  */
    while (device_framework_length != 0)
    {
        /* Get the length of the current descriptor.  */
        descriptor_length =  (ULONG) *device_framework;

        /* And its type.  */
        descriptor_type =  *(device_framework + 1);

        /* Check if this is a configuration descriptor.  */
        if (descriptor_type == UX_CONFIGURATION_DESCRIPTOR_ITEM)
        {
            /* Parse the descriptor in something more readable.  */
            _ux_utility_descriptor_parse(device_framework,
                        _ux_system_configuration_descriptor_structure,
                        UX_CONFIGURATION_DESCRIPTOR_ENTRIES,
                        (UCHAR *) &configuration_descriptor);

            /* Now we need to check the configuration value. It has
               to be the same as the one specified in the setup function.  */
            if (configuration_descriptor.bConfigurationValue == configuration_value)
                /* The configuration is found. */
                break;
        }

        /* Adjust what is left of the device framework.  */
        device_framework_length -= descriptor_length;
        /* Point to the next descriptor.  */
        device_framework += descriptor_length;
    }

    /* Configuration not found. */
    if (device_framework_length == 0 && configuration_value != 0)
        return(UX_ERROR);

    /* We unmount the configuration if there is previous configuration selected. */
    if (device -> ux_slave_device_configuration_selected)
    {

        /* Get the pointer to the first interface.  */
        interface_ptr =  device -> ux_slave_device_first_interface;

#if !defined(UX_DEVICE_INITIALIZE_FRAMEWORK_SCAN_DISABLE) || UX_MAX_DEVICE_INTERFACES > 1
        /* Deactivate all the interfaces if any.  */
        while (interface_ptr != UX_NULL)
        {
#endif
            /* Build all the fields of the Class Command.  */
            class_command.ux_slave_class_command_request =   UX_SLAVE_CLASS_COMMAND_DEACTIVATE;
            class_command.ux_slave_class_command_interface =  (VOID *) interface_ptr;

            /* Get the pointer to the class container of this interface.  */
            class_inst =  interface_ptr -> ux_slave_interface_class;

            /* Store the class container. */
            class_command.ux_slave_class_command_class_ptr =  class_inst;

            /* If there is a class container for this instance, deactivate it.  */
            if (class_inst != UX_NULL)

                /* Call the class with the DEACTIVATE signal.  */
                class_inst -> ux_slave_class_entry_function(&class_command);

#if !defined(UX_DEVICE_INITIALIZE_FRAMEWORK_SCAN_DISABLE) || UX_MAX_DEVICE_INTERFACES > 1
            /* Get the next interface.  */
            next_interface =  interface_ptr -> ux_slave_interface_next_interface;
#endif

            /* Remove the interface and all endpoints associated with it.  */
            _ux_device_stack_interface_delete(interface_ptr);

#if !defined(UX_DEVICE_INITIALIZE_FRAMEWORK_SCAN_DISABLE) || UX_MAX_DEVICE_INTERFACES > 1
            /* Now we refresh the interface pointer.  */
            interface_ptr =  next_interface;
        }
#endif

    }

    /* No configuration is selected.  */
    device -> ux_slave_device_configuration_selected =  0;

    /* Mark the device as attached now. */
    device -> ux_slave_device_state =  UX_DEVICE_ATTACHED;

    /* The DCD needs to update the device state too.  */
    dcd -> ux_slave_dcd_function(dcd, UX_DCD_CHANGE_STATE, (VOID *) UX_DEVICE_ATTACHED);

    /* If the host tries to unconfigure, we are done. */
    if (configuration_value == 0)
        return(UX_SUCCESS);

    /* Memorize the configuration selected.  */
    device -> ux_slave_device_configuration_selected =  configuration_value;

    /* We have found the configuration value requested by the host.
       Create the configuration descriptor and attach it to the device.  */
    _ux_utility_descriptor_parse(device_framework,
                _ux_system_configuration_descriptor_structure,
                UX_CONFIGURATION_DESCRIPTOR_ENTRIES,
                (UCHAR *) &device -> ux_slave_device_configuration_descriptor);

    /* Configuration character D6 is for Self-powered */
    _ux_system_slave -> ux_system_slave_power_state = (configuration_descriptor.bmAttributes & 0x40) ? UX_DEVICE_SELF_POWERED : UX_DEVICE_BUS_POWERED;

    /* Configuration character D5 is for Remote Wakeup */
    _ux_system_slave -> ux_system_slave_remote_wakeup_capability = (configuration_descriptor.bmAttributes & 0x20) ? UX_TRUE : UX_FALSE;

    /* Search only in current configuration */
    device_framework_length =  configuration_descriptor.wTotalLength;

    /*  We need to scan all the interface descriptors following this
        configuration descriptor and enable all endpoints associated
        with the default alternate setting of each interface.  */
    while (device_framework_length != 0)
    {

        /* Get the length of the current descriptor.  */
        descriptor_length =  (ULONG) *device_framework;

        /* And its type.  */
        descriptor_type =  *(device_framework + 1);

        /* Check if this is an interface association descriptor.  */
        if(descriptor_type == UX_INTERFACE_ASSOCIATION_DESCRIPTOR_ITEM)
        {

            /* Set the IAD flag.  */
            iad_flag = UX_TRUE;

            /* Get the first interface we have in the IAD. */
            iad_first_interface = (ULONG)  *(device_framework + 2);

            /* Get the number of interfaces we have in the IAD. */
            iad_number_interfaces = (ULONG)  *(device_framework + 3);
        }

        /* Check if this is an interface descriptor.  */
        if(descriptor_type == UX_INTERFACE_DESCRIPTOR_ITEM)
        {

            /* Parse the descriptor in something more readable.  */
            _ux_utility_descriptor_parse(device_framework,
                        _ux_system_interface_descriptor_structure,
                        UX_INTERFACE_DESCRIPTOR_ENTRIES,
                        (UCHAR *) &interface_descriptor);

            /* If the alternate setting is 0 for this interface, we need to
               memorize its class association and start it.  */
            if (interface_descriptor.bAlternateSetting == 0)
            {

                /* Are we in a IAD scenario ? */
                if (iad_flag == UX_TRUE)
                {

                    /* Check if this is the first interface from the IAD. In this case,
                       we need to match a class to this interface.  */
                    if (interface_descriptor.bInterfaceNumber == iad_first_interface)
                    {

                        /* First interface. Scan the list of classes to find a match.  */
                        class_inst =  _ux_system_slave -> ux_system_slave_class_array;

#if UX_MAX_SLAVE_CLASS_DRIVER > 1
                        /* Parse all the class drivers.  */
                        for (class_index = 0; class_index < _ux_system_slave -> ux_system_slave_max_class; class_index++)
                        {
#endif

                            /* Check if this class driver is used.  */
                            if (class_inst -> ux_slave_class_status == UX_USED)
                            {

                                /* Check if this is the same interface for the same configuration. */
                                if ((interface_descriptor.bInterfaceNumber == class_inst -> ux_slave_class_interface_number) &&
                                    (configuration_value == class_inst -> ux_slave_class_configuration_number))
                                {

                                    /* Memorize the class in the class/interface array.  */
                                    _ux_system_slave -> ux_system_slave_interface_class_array[interface_descriptor.bInterfaceNumber] = class_inst;

                                    /* And again as the current class.  */
                                    current_class = class_inst;

#if UX_MAX_SLAVE_CLASS_DRIVER > 1
                                    /* We are done here.  */
                                    break;
#endif
                                }
                            }

#if UX_MAX_SLAVE_CLASS_DRIVER > 1
                            /* Move to the next registered class.  */
                            class_inst ++;
                        }
#endif
                    }
                    else

                        /* Memorize the class in the class/interface array.  We use the current class. */
                        _ux_system_slave -> ux_system_slave_interface_class_array[interface_descriptor.bInterfaceNumber] = current_class;

                    /* Decrement the number of interfaces found in the same IAD.  */
                    iad_number_interfaces--;

                    /* If none are left, get out of the IAD state machine.  */
                    if (iad_number_interfaces == 0)

                        /* We have exhausted the interfaces within the IAD.  */
                        iad_flag = UX_FALSE;

                }
                else
                {

                    /* First interface. Scan the list of classes to find a match.  */
                    class_inst =  _ux_system_slave -> ux_system_slave_class_array;

#if UX_MAX_SLAVE_CLASS_DRIVER > 1
                    /* Parse all the class drivers.  */
                    for (class_index = 0; class_index < _ux_system_slave -> ux_system_slave_max_class; class_index++)
                    {
#endif

                        /* Check if this class driver is used.  */
                        if (class_inst -> ux_slave_class_status == UX_USED)
                        {

                            /* Check if this is the same interface for the same configuration. */
                            if ((interface_descriptor.bInterfaceNumber == class_inst -> ux_slave_class_interface_number) &&
                                    (configuration_value == class_inst -> ux_slave_class_configuration_number))
                            {

                                /* Memorize the class in the class/interface array.  */
                                _ux_system_slave -> ux_system_slave_interface_class_array[interface_descriptor.bInterfaceNumber] = class_inst;

#if UX_MAX_SLAVE_CLASS_DRIVER > 1
                                /* We are done here.  */
                                break;
#endif
                            }
                        }

#if UX_MAX_SLAVE_CLASS_DRIVER > 1
                        /* Move to the next registered class.  */
                        class_inst ++;
                    }
#endif
                }

                /* Set the interface.  */
                _ux_device_stack_interface_set(device_framework, device_framework_length, 0);
            }
        }

        /* Adjust what is left of the device framework.  */
        device_framework_length -=  descriptor_length;

        /* Point to the next descriptor.  */
        device_framework +=  descriptor_length;
    }

    /* Mark the device as configured now. */
    device -> ux_slave_device_state =  UX_DEVICE_CONFIGURED;

    /* The DCD needs to update the device state too.  */
    dcd -> ux_slave_dcd_function(dcd, UX_DCD_CHANGE_STATE, (VOID *) UX_DEVICE_CONFIGURED);

    /* Configuration mounted. */
    return(UX_SUCCESS);
}

