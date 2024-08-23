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
/**   Asix Class                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_asix.h"
#include "ux_host_stack.h"


#if !defined(UX_HOST_STANDALONE)
static inline UINT _ux_host_class_asix_try_all_vid_pids(UX_HOST_CLASS_COMMAND *command);
#endif


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_asix_entry                           PORTABLE C      */ 
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the entry point of the asix class. It will be      */ 
/*    called by the USBX stack enumeration module when there is a new     */ 
/*    asix ethernet device on the bus or when the it is removed.          */ 
/*                                                                        */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                                       Asix class command    */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_asix_activate                  Activate asix class   */ 
/*    _ux_host_class_asix_deactivate                Deactivate asix class */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Asix Class                                                          */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed compile warning,      */
/*                                            refined VID/PID check flow, */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_asix_entry(UX_HOST_CLASS_COMMAND *command)
{
#if defined(UX_HOST_STANDALONE)
    UX_PARAMETER_NOT_USED(command);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else

UINT    status;


    /* The command request will tell us we need to do here, either a enumeration
       query, an activation or a deactivation.  */
    switch (command -> ux_host_class_command_request)
    {

    case UX_HOST_CLASS_COMMAND_QUERY:

        /* The query command is used to let the stack enumeration process know if we want to own
           this device or not.  */
        if(command -> ux_host_class_command_usage == UX_HOST_CLASS_COMMAND_USAGE_PIDVID) 
        {
            if (_ux_host_class_asix_try_all_vid_pids(command) == UX_SUCCESS)
                return(UX_SUCCESS);
        }
        
        /* No match.  */
        return(UX_NO_CLASS_MATCH);                        
                
    case UX_HOST_CLASS_COMMAND_ACTIVATE:

        /* The activate command is used when the device inserted has found a parent and
           is ready to complete the enumeration.  */
        status =  _ux_host_class_asix_activate(command);
        return(status);

    case UX_HOST_CLASS_COMMAND_DEACTIVATE:

        /* The deactivate command is used when the device has been extracted either      
           directly or when its parents has been extracted.  */
        status =  _ux_host_class_asix_deactivate(command);
        return(status);

    default: 
            
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_FUNCTION_NOT_SUPPORTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_FUNCTION_NOT_SUPPORTED);
    }   
#endif
}
#if !defined(UX_HOST_STANDALONE)
static const USHORT _ux_host_class_asix_vid_pid_array[] =
{
    UX_HOST_CLASS_ASIX_VID_PID_ARRAY
};

static inline UINT _ux_host_class_asix_try_all_vid_pids(UX_HOST_CLASS_COMMAND *command)
{
UINT    i, pos;
UINT    n_ids = sizeof(_ux_host_class_asix_vid_pid_array) >> 1;
UINT    n_id_pairs = n_ids >> 1;
    for (i = 0, pos = 0; i < n_id_pairs; i ++, pos += 2)
    {
        if ((command -> ux_host_class_command_pid == _ux_host_class_asix_vid_pid_array[pos + 1]) &&
            (command -> ux_host_class_command_vid == _ux_host_class_asix_vid_pid_array[pos    ]))
            return(UX_SUCCESS);
    }
    return(UX_NO_CLASS_MATCH);
}
#endif
