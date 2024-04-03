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
/**   PIMA Class                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_pima.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_pima_session_close                   PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function closes a session with the PIMA device.                */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                       Pointer to pima class    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*  _ux_host_class_pima_command                 Pima command function     */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USB application                                                     */ 
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
UINT  _ux_host_class_pima_session_close(UX_HOST_CLASS_PIMA *pima, UX_HOST_CLASS_PIMA_SESSION *pima_session)
{

UX_HOST_CLASS_PIMA_COMMAND             command;
UINT                                status;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PIMA_SESSION_CLOSE, pima, pima_session, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Check if this session is valid or not.  */
    if (pima_session -> ux_host_class_pima_session_magic != UX_HOST_CLASS_PIMA_MAGIC_NUMBER)
        return (UX_HOST_CLASS_PIMA_RC_SESSION_NOT_OPEN);

    /* Check if this session is opened or not.  */
    if (pima_session -> ux_host_class_pima_session_state != UX_HOST_CLASS_PIMA_SESSION_STATE_OPENED)
        return (UX_HOST_CLASS_PIMA_RC_SESSION_NOT_OPEN);

    /* The transaction ID in the PIMA instance should be reset.  */
    pima -> ux_host_class_pima_transaction_id =  0;
        
    /* Issue command to close the session with the PIMA device.  No parameter.  */
    command.ux_host_class_pima_command_nb_parameters =  0;
    
    /* Then set the command to CLOSE_SESSION.  */
    command.ux_host_class_pima_command_operation_code =  UX_HOST_CLASS_PIMA_OC_CLOSE_SESSION;

    /* Other parameters unused.  */
    command.ux_host_class_pima_command_parameter_1 =  0;
    command.ux_host_class_pima_command_parameter_2 =  0;
    command.ux_host_class_pima_command_parameter_3 =  0;
    command.ux_host_class_pima_command_parameter_4 =  0;
    command.ux_host_class_pima_command_parameter_5 =  0;

    /* Issue the command.  */
    status = _ux_host_class_pima_command(pima, &command, 0 , UX_NULL, 0, 0);

    /* Check the result. If OK, the session was closed properly.  */
    if (status == UX_SUCCESS)
    {

        /* Reset the PIMA session in the pima instance. */
        pima -> ux_host_class_pima_session = UX_NULL;
        
        /* Reset the magic field.  */
        pima_session -> ux_host_class_pima_session_magic =  0;
        
        /* Mark the session as closed.  */
        pima_session -> ux_host_class_pima_session_state = UX_HOST_CLASS_PIMA_SESSION_STATE_CLOSED;
    }
    
    /* Return completion status.  */
    return(status);
}

