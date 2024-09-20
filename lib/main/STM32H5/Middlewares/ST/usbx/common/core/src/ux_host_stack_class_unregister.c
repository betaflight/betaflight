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
/*    _ux_host_stack_class_unregister                     PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function unregisters a USB class from the USB stack.           */
/*                                                                        */
/*    Note following steps must be done before host class unregister:     */
/*    All devices related to the class must be removed.                   */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    class_name                            Name of class                 */
/*    class_entry_function                  Entry function of the class   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*    UX_SUCCESS                            Class unregistered            */
/*    UX_NO_CLASS_MATCH                     Class not found               */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    (class_entry_function)                Entry function of the class   */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Application                                                         */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  09-30-2020     Chaoqiong Xiao           Initial Version 6.1           */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_class_unregister(UINT (*class_entry_function)(struct UX_HOST_CLASS_COMMAND_STRUCT *))
{

UX_HOST_CLASS           *class_inst;
UX_HOST_CLASS_COMMAND   class_command;
#if UX_MAX_CLASS_DRIVER > 1
ULONG                   class_index;
#endif


    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_CLASS_UNREGISTER, class_entry_function, 0, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    /* Get first class.  */
    class_inst =  _ux_system_host -> ux_system_host_class_array;

#if UX_MAX_CLASS_DRIVER > 1
    /* We need to parse the class table to find right class instance.  */
    for (class_index = 0; class_index < _ux_system_host -> ux_system_host_max_class; class_index++)
    {
#endif

        /* Check if the class is expected.  */
        if (class_inst -> ux_host_class_entry_function == class_entry_function)
        {

            /* Initialize the class command with the generic parameters.  */
            class_command.ux_host_class_command_request =  UX_HOST_CLASS_COMMAND_DESTROY;
            class_command.ux_host_class_command_class_ptr = (VOID *)class_inst;

            /* Invoke command for class destroy.  */
            class_inst -> ux_host_class_entry_function(&class_command);

            /* Mark as free.  */
            class_inst -> ux_host_class_entry_function = UX_NULL;
            class_inst -> ux_host_class_status = UX_UNUSED;

            /* Class unregistered success.  */
            return(UX_SUCCESS);
        }

#if UX_MAX_CLASS_DRIVER > 1
        /* Move to the next class.  */
        class_inst ++;
    }
#endif

    /* No more entries in the class table.  */
    return(UX_NO_CLASS_MATCH);
}
