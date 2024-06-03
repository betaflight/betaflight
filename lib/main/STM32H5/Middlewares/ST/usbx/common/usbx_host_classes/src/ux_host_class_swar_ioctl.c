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
/**   Host Sierra Wireless AR module class                                */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_swar.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_swar_ioctl                           PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the ioctl entry point for the application to       */ 
/*    configure the Swar device.                                          */ 
/*                                                                        */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    swar                                  Pointer to swar class         */ 
/*    ioctl_function                        ioctl function                */ 
/*    parameter                             pointer to structure          */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_endpoint_transfer_abort                              */
/*                                          Abort transfer                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Storage Class                                                       */ 
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
UINT  _ux_host_class_swar_ioctl(UX_HOST_CLASS_SWAR *swar, ULONG ioctl_function,
                                    VOID *parameter)
{

UINT                                status;

    UX_PARAMETER_NOT_USED(parameter);

    /* The command request will tell us we need to do here.  */
    switch (ioctl_function)
    {

    case UX_HOST_CLASS_SWAR_IOCTL_ABORT_IN_PIPE :

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_SWAR_IOCTL_ABORT_IN_PIPE, swar, swar -> ux_host_class_swar_bulk_in_endpoint, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)
    
        /* We need to abort transactions on the bulk In pipe.  */
        _ux_host_stack_endpoint_transfer_abort(swar -> ux_host_class_swar_bulk_in_endpoint);
        
        /* Status is successful.  */
        status = UX_SUCCESS;
        break;

    case UX_HOST_CLASS_SWAR_IOCTL_ABORT_OUT_PIPE :

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_SWAR_IOCTL_ABORT_OUT_PIPE, swar, swar -> ux_host_class_swar_bulk_out_endpoint, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)
    
        /* We need to abort transactions on the bulk Out pipe.  */
        _ux_host_stack_endpoint_transfer_abort(swar -> ux_host_class_swar_bulk_out_endpoint);

        /* Status is successful.  */
        status = UX_SUCCESS;
        break;

    default: 

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_FUNCTION_NOT_SUPPORTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Function not supported. Return an error.  */
        status =  UX_FUNCTION_NOT_SUPPORTED;
        break;
    }   

    /* Return status to caller.  */
    return(status);
}

