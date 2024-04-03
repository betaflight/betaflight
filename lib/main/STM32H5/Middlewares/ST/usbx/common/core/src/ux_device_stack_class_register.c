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
/*    _ux_device_stack_class_register                     PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function registers a slave class to the slave stack.           */
/*                                                                        */
/*    Note: The C string of class_name must be NULL-terminated and the    */
/*    length of it (without the NULL-terminator itself) must be no larger */
/*    than UX_MAX_CLASS_NAME_LENGTH.                                      */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    class_name                            Name of class                 */
/*    class_function_entry                  Class entry function          */
/*    configuration_number                  Configuration # for this class*/
/*    interface_number                      Interface # for this class    */
/*    parameter                             Parameter specific for class  */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_string_length_check       Check C string and return     */
/*                                          its length if null-terminated */
/*    _ux_utility_memory_copy               Memory copy                   */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            optimized based on compile  */
/*                                            definitions, verified       */
/*                                            memset and memcpy cases,    */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_stack_class_register(UCHAR *class_name,
                        UINT (*class_entry_function)(struct UX_SLAVE_CLASS_COMMAND_STRUCT *),
                        ULONG configuration_number,
                        ULONG interface_number,
                        VOID *parameter)
{

UX_SLAVE_CLASS              *class_inst;
UINT                        status;
UX_SLAVE_CLASS_COMMAND      command;
UINT                        class_name_length =  0;
#if UX_MAX_SLAVE_CLASS_DRIVER > 1
ULONG                       class_index;
#endif


    /* Get the length of the class name (exclude null-terminator).  */
    status =  _ux_utility_string_length_check(class_name, &class_name_length, UX_MAX_CLASS_NAME_LENGTH);
    if (status)
        return(status);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_STACK_CLASS_REGISTER, class_name, interface_number, parameter, 0, UX_TRACE_DEVICE_STACK_EVENTS, 0, 0)

    /* Get first class.  */
    class_inst =  _ux_system_slave -> ux_system_slave_class_array;

#if UX_MAX_SLAVE_CLASS_DRIVER > 1
    /* We need to parse the class table to find an empty spot.  */
    for (class_index = 0; class_index < _ux_system_slave -> ux_system_slave_max_class; class_index++)
    {
#endif

        /* Check if this class is already used.  */
        if (class_inst -> ux_slave_class_status == UX_UNUSED)
        {

#if defined(UX_NAME_REFERENCED_BY_POINTER)
            class_inst -> ux_slave_class_name = (const UCHAR *)class_name;
#else
            /* We have found a free container for the class. Copy the name (with null-terminator).  */
            _ux_utility_memory_copy(class_inst -> ux_slave_class_name, class_name, class_name_length + 1); /* Use case of memcpy is verified. */
#endif
            
            /* Memorize the entry function of this class.  */
            class_inst -> ux_slave_class_entry_function =  class_entry_function;

            /* Memorize the pointer to the application parameter.  */
            class_inst -> ux_slave_class_interface_parameter =  parameter;
            
            /* Memorize the configuration number on which this instance will be called.  */
            class_inst -> ux_slave_class_configuration_number =  configuration_number;
            
            /* Memorize the interface number on which this instance will be called.  */
            class_inst -> ux_slave_class_interface_number =  interface_number;
            
            /* Build all the fields of the Class Command to initialize the class.  */
            command.ux_slave_class_command_request    =  UX_SLAVE_CLASS_COMMAND_INITIALIZE;
            command.ux_slave_class_command_parameter  =  parameter;
            command.ux_slave_class_command_class_ptr  =  class_inst;

            /* Call the class initialization routine.  */
            status = class_entry_function(&command);
            
            /* Check the status.  */
            if (status != UX_SUCCESS)
                return(status);
            
            /* Make this class used now.  */
            class_inst -> ux_slave_class_status = UX_USED;

            /* Return successful completion.  */
            return(UX_SUCCESS);
        }

#if UX_MAX_SLAVE_CLASS_DRIVER > 1
        /* Move to the next class.  */
        class_inst ++;
    }    
#endif

    /* No more entries in the class table.  */
    return(UX_MEMORY_INSUFFICIENT);
}

