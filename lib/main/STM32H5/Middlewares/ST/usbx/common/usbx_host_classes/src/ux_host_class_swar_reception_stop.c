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
/*    _ux_host_class_swar_reception_stop                  PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function starts a reception with the Sierra modem. This way    */ 
/*    allows for non blocking calls based on a packet orientated round    */ 
/*    robbin buffer. When a packet is fully or partially received, an     */
/*    application callback function is invoked and a new transfer request */
/*    is rescheduled.                                                     */
/*                                                                        */
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    swar                               Pointer to swar class            */ 
/*    swar_reception                     Pointer to reception struct      */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_class_instance_verify  Verify the class instance     */ 
/*    _ux_host_stack_endpoint_transfer_abort                              */
/*                                          Abort transfer                */
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
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_swar_reception_stop (UX_HOST_CLASS_SWAR *swar, 
                                    UX_HOST_CLASS_SWAR_RECEPTION *swar_reception)
{

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_SWAR_RECEPTION_STOP, swar, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Ensure the instance is valid.  */
    if (_ux_host_stack_class_instance_verify(_ux_system_host_class_swar_name, (VOID *) swar) != UX_SUCCESS)
    {        

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, swar, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* Check if we do have transfers for this application. If none, nothing to do. */
    if (swar_reception -> ux_host_class_swar_reception_state ==  UX_HOST_CLASS_SWAR_RECEPTION_STATE_STOPPED)
        return(UX_SUCCESS);
        
    /* We need to abort transactions on the bulk In pipe.  */
    _ux_host_stack_endpoint_transfer_abort(swar -> ux_host_class_swar_bulk_in_endpoint);

    /* Declare the reception stopped.  */
    swar_reception -> ux_host_class_swar_reception_state =  UX_HOST_CLASS_SWAR_RECEPTION_STATE_STOPPED;

    /* This function never really fails.  */
    return(UX_SUCCESS);
}

