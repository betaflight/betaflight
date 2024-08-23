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
/**   Printer Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_printer.h"
#include "ux_host_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_printer_activate                     PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function calls the USBX stack to activate the class.           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    command                               Printer class command pointer */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_class_printer_configure      Configure printer class       */
/*    _ux_host_class_printer_endpoints_get  Get endpoints of printer      */
/*    _ux_host_class_printer_name_get       Get printer name              */
/*    _ux_host_stack_class_instance_create  Create class instance         */
/*    _ux_host_stack_class_instance_destroy Destroy the class instance    */
/*    _ux_utility_memory_allocate           Allocate memory block         */
/*    _ux_utility_memory_free               Free memory block             */
/*    _ux_host_semaphore_create             Create printer semaphore      */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    _ux_host_class_printer_entry          Entry of printer class        */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_printer_activate(UX_HOST_CLASS_COMMAND *command)
{

UX_INTERFACE            *interface_ptr;
UX_HOST_CLASS_PRINTER   *printer;
#if !defined(UX_HOST_STANDALONE)
UINT                    status;
#endif


    /* The printer is always activated by the interface descriptor and not the
       device descriptor.  */
    interface_ptr =  (UX_INTERFACE *) command -> ux_host_class_command_container;

    /* Obtain memory for this class instance.  */
    printer =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HOST_CLASS_PRINTER));
    if (printer == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Store the class container into this instance.  */
    printer -> ux_host_class_printer_class =  command -> ux_host_class_command_class_ptr;

    /* Store the interface container into the printer class instance.  */
    printer -> ux_host_class_printer_interface =  interface_ptr;

    /* Store the device container into the printer class instance.  */
    printer -> ux_host_class_printer_device =  interface_ptr -> ux_interface_configuration -> ux_configuration_device;

    /* This instance of the device must also be stored in the interface container.  */
    interface_ptr -> ux_interface_class_instance =  (VOID *) printer;

    /* Create this class instance.  */
    _ux_host_stack_class_instance_create(printer -> ux_host_class_printer_class, (VOID *) printer);

#if defined(UX_HOST_STANDALONE)

    /* Do activate steps.  */
    printer -> ux_host_class_printer_enum_state = UX_STATE_WAIT;
    printer -> ux_host_class_printer_state = UX_HOST_CLASS_INSTANCE_MOUNTING;
    return(UX_SUCCESS);
#else

    /* Configure the printer.  */
    status =  _ux_host_class_printer_configure(printer);

    /* Get the printer endpoint(s). We may need to search for Bulk Out and Bulk In endpoints.  */
    if (status == UX_SUCCESS)
        status =  _ux_host_class_printer_endpoints_get(printer);

    /* Get the name of the printer from the 1284 descriptor.  */
    if (status == UX_SUCCESS)
        status =  _ux_host_class_printer_name_get(printer);

    /* Create the semaphore to protect 2 threads from accessing the same printer instance.  */
    if (status == UX_SUCCESS)
    {
        status =  _ux_host_semaphore_create(&printer -> ux_host_class_printer_semaphore, "ux_host_class_printer_semaphore", 1);
        if (status != UX_SUCCESS)
            status = UX_SEMAPHORE_ERROR;
    }

    /* Success things.  */
    if (status == UX_SUCCESS)
    {

        /* Mark the printer as live now.  */
        printer -> ux_host_class_printer_state =  UX_HOST_CLASS_INSTANCE_LIVE;

        /* If all is fine and the device is mounted, we may need to inform the application
        if a function has been programmed in the system structure.  */
        if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
        {

            /* Call system change function.  */
            _ux_system_host ->  ux_system_host_change_function(UX_DEVICE_INSERTION, printer -> ux_host_class_printer_class, (VOID *) printer);
        }

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PRINTER_ACTIVATE, printer, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

        /* If trace is enabled, register this object.  */
        UX_TRACE_OBJECT_REGISTER(UX_TRACE_HOST_OBJECT_TYPE_INTERFACE, printer, 0, 0, 0)

        /* Return success.  */
        return(UX_SUCCESS);
    }

    /* On error, free resources.  */
    _ux_host_stack_class_instance_destroy(printer -> ux_host_class_printer_class, (VOID *) printer);
    interface_ptr -> ux_interface_class_instance = UX_NULL;
    _ux_utility_memory_free(printer);

    /* Return completion status.  */
    return(status);

#endif
}

