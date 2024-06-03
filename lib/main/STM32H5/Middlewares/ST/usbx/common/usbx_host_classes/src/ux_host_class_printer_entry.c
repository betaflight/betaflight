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


#if defined(UX_HOST_STANDALONE)
#define UX_HOST_CLASS_PRINTER_ENUM_START                (UX_STATE_WAIT)
#define UX_HOST_CLASS_PRINTER_ENUM_IDLE                 (UX_STATE_IDLE)

#define UX_HOST_CLASS_PRINTER_ENUM_NAME_GET             (UX_STATE_STACK_STEP + 0)
#define UX_HOST_CLASS_PRINTER_ENUM_NAME_PARSE           (UX_STATE_STACK_STEP + 1)
#define UX_HOST_CLASS_PRINTER_ENUM_TRANSFER             (UX_STATE_STACK_STEP + 2)
#define UX_HOST_CLASS_PRINTER_ENUM_DONE                 (UX_STATE_STACK_STEP + 3)

extern VOID _ux_host_class_printer_name_parse(UX_HOST_CLASS_PRINTER *printer,
                            UCHAR *descriptor_buffer, ULONG descriptor_length);

static inline UINT _ux_host_class_printer_activate_wait(UX_HOST_CLASS_COMMAND *command);
#endif

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_printer_entry                        PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the entry point of the printer class. It will be   */ 
/*    called by the USBX stack enumeration module when there is a new     */ 
/*    printer on the bus or when the USB printer is removed.              */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                               Printer class command         */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_printer_activate       Activate printer class        */ 
/*    _ux_host_class_printer_deactivate     Deactivate printer class      */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Printer Class                                                       */ 
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
/*                                            removed compile warning,    */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_printer_entry(UX_HOST_CLASS_COMMAND *command)
{

UINT    status;


    /* The command request will tell us we need to do here, either a enumeration
       query, an activation or a deactivation.  */
    switch (command -> ux_host_class_command_request)
    {

    case UX_HOST_CLASS_COMMAND_QUERY:

        /* The query command is used to let the stack enumeration process know if we want to own
           this device or not.  */
        if((command -> ux_host_class_command_usage == UX_HOST_CLASS_COMMAND_USAGE_CSP) &&
                             (command -> ux_host_class_command_class == UX_HOST_CLASS_PRINTER_CLASS))
            return(UX_SUCCESS);                        
        else            
            return(UX_NO_CLASS_MATCH);                        
                
    case UX_HOST_CLASS_COMMAND_ACTIVATE:

        /* The activate command is used when the device inserted has found a parent and
           is ready to complete the enumeration.  */
        status =  _ux_host_class_printer_activate(command);
        return(status);

#if defined(UX_HOST_STANDALONE)
    case UX_HOST_CLASS_COMMAND_ACTIVATE_WAIT:
        status = _ux_host_class_printer_activate_wait(command);
        return(status);
#endif

    case UX_HOST_CLASS_COMMAND_DEACTIVATE:

        /* The deactivate command is used when the device has been extracted either      
           directly or when its parents has been extracted.  */
        status =  _ux_host_class_printer_deactivate(command);
        return(status);

    default: 
            
        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_FUNCTION_NOT_SUPPORTED);
    }   
}

#if defined(UX_HOST_STANDALONE)
static inline UINT _ux_host_class_printer_activate_wait(UX_HOST_CLASS_COMMAND *command)
{
UX_INTERFACE                    *interface_ptr;
UX_HOST_CLASS_PRINTER           *printer;
UX_ENDPOINT                     *control_endpoint;
UX_TRANSFER                     *transfer;
UINT                            status;

    /* Get the instance for this class.  */
    interface_ptr = (UX_INTERFACE *)command -> ux_host_class_command_container;

    /* Sanity check.  */
    if (interface_ptr == UX_NULL)
        return(UX_STATE_EXIT);

    printer =  (UX_HOST_CLASS_PRINTER *) interface_ptr -> ux_interface_class_instance;

    /* Sanity check.  */
    if (printer == UX_NULL)
        return(UX_STATE_EXIT);

    switch(printer -> ux_host_class_printer_enum_state)
    {
    case UX_HOST_CLASS_PRINTER_ENUM_START:
        status = _ux_host_class_printer_endpoints_get(printer);
        if (status != UX_SUCCESS)
        {
            printer -> ux_host_class_printer_status = status;
            printer -> ux_host_class_printer_enum_state =
                                            UX_HOST_CLASS_PRINTER_ENUM_DONE;
            return(UX_STATE_WAIT);
        }

        /* Fall through.  */
    case UX_HOST_CLASS_PRINTER_ENUM_NAME_GET:
        status = _ux_host_class_printer_name_get(printer);
        if (status != UX_SUCCESS)
        {
            printer -> ux_host_class_printer_status = status;
            printer -> ux_host_class_printer_enum_state =
                                            UX_HOST_CLASS_PRINTER_ENUM_DONE;
            return(UX_STATE_WAIT);
        }

        /* Next: transfer -> parse name.  */
        printer -> ux_host_class_printer_enum_state =
                                        UX_HOST_CLASS_PRINTER_ENUM_TRANSFER;
        printer -> ux_host_class_printer_next_state =
                                        UX_HOST_CLASS_PRINTER_ENUM_NAME_PARSE;

        /* Fall through.  */
    case UX_HOST_CLASS_PRINTER_ENUM_TRANSFER:

        /* We need to get the default control endpoint transfer request pointer.  */
        control_endpoint = &printer -> ux_host_class_printer_device -> ux_device_control_endpoint;
        transfer =  &control_endpoint -> ux_endpoint_transfer_request;

        status = _ux_host_stack_transfer_run(transfer);

        /* Transfer finished any way.  */
        if (status < UX_STATE_WAIT)
        {

            /* Error.  */
            if (transfer -> ux_transfer_request_completion_code != UX_SUCCESS)
            {
                printer -> ux_host_class_printer_status = status;
                printer -> ux_host_class_printer_enum_state =
                                            UX_HOST_CLASS_PRINTER_ENUM_DONE;
                return(UX_STATE_WAIT);
            }

            /* Next state.  */
            printer -> ux_host_class_printer_enum_state =
                                    printer -> ux_host_class_printer_next_state;

            return(UX_STATE_WAIT);
        }

        /* Wait transfer.  */
        return(UX_STATE_WAIT);

    case UX_HOST_CLASS_PRINTER_ENUM_NAME_PARSE:

        /* We need to get the default control endpoint transfer request pointer.  */
        control_endpoint = &printer -> ux_host_class_printer_device -> ux_device_control_endpoint;
        transfer =  &control_endpoint -> ux_endpoint_transfer_request;

        _ux_host_class_printer_name_parse(printer,
                                transfer -> ux_transfer_request_data_pointer,
                                transfer -> ux_transfer_request_actual_length);

        /* Fall through.  */
    case UX_HOST_CLASS_PRINTER_ENUM_DONE:

        /* Free allocated resources.  */
        if (printer -> ux_host_class_printer_allocated)
        {
            _ux_utility_memory_free(printer -> ux_host_class_printer_allocated);
            printer -> ux_host_class_printer_allocated = UX_NULL;
        }

        /* Check status.  */
        if (printer -> ux_host_class_printer_status != UX_SUCCESS)
        {

            /* On error, free resources.  */
            _ux_host_stack_class_instance_destroy(
                    printer -> ux_host_class_printer_class, (VOID *) printer);
            interface_ptr -> ux_interface_class_instance = UX_NULL;
            _ux_utility_memory_free(printer);
            return(UX_STATE_ERROR);
        }

        /* Success.  */

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
        printer -> ux_host_class_printer_enum_state = UX_STATE_IDLE;
        return(UX_STATE_NEXT);

    default: /* reset, idle, error ...   */
        break;
    }

    /* Just next phase.  */
    return(UX_STATE_NEXT);
}
#endif
