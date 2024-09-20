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
/*    _ux_host_class_pima_session_open                    PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function opens a session with the PIMA device. The session     */ 
/*    is maintained in this state until the session is closed or the      */ 
/*    device is unmounted.                                                */ 
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
UINT  _ux_host_class_pima_session_open(UX_HOST_CLASS_PIMA *pima, UX_HOST_CLASS_PIMA_SESSION *pima_session)
{

UX_HOST_CLASS_PIMA_COMMAND             command;
ULONG                                status;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PIMA_SESSION_OPEN, pima, pima_session, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Check if there is already a session opened.  */
    if (pima -> ux_host_class_pima_session != UX_NULL)
        return (UX_HOST_CLASS_PIMA_RC_SESSION_ALREADY_OPENED);

    /* The transaction ID in the PIMA instance should be reset.  */
    pima -> ux_host_class_pima_transaction_id =  0;
        
    /* Issue command to open the session with the PIMA device.  First set the number of parameters.  */
    command.ux_host_class_pima_command_nb_parameters =  1;
    
    /* Then set the command to OPEN_SESSION.  */
    command.ux_host_class_pima_command_operation_code =  UX_HOST_CLASS_PIMA_OC_OPEN_SESSION;

    /* The session ID is the handle to the session.  */
    command.ux_host_class_pima_command_parameter_1 =  (ULONG) (ALIGN_TYPE) pima_session;
    
    /* Other parameters unused.  */
    command.ux_host_class_pima_command_parameter_2 =  0;
    command.ux_host_class_pima_command_parameter_3 =  0;
    command.ux_host_class_pima_command_parameter_4 =  0;
    command.ux_host_class_pima_command_parameter_5 =  0;

    /* Issue the command.  */
    status = _ux_host_class_pima_command(pima, &command, 0 , UX_NULL, 0, 0);

    /* Check the result. If OK, the session was opened properly.  */
    if (status == UX_SUCCESS)
    {

        /* Store the session pointer in the PIMA instance. The PIMA class instance
           only supports one opened session at a time at this stage.  */
        pima -> ux_host_class_pima_session = pima_session;
        
        /* Save the session ID in the session container. This is not too useful since
           the session ID is the session structure address.  */
        pima_session -> ux_host_class_pima_session_id = (ALIGN_TYPE) pima_session;
        
        /* Put the magic number in the session instance.  */
        pima_session -> ux_host_class_pima_session_magic = UX_HOST_CLASS_PIMA_MAGIC_NUMBER;

        /* Mark the session as opened.  */
        pima_session -> ux_host_class_pima_session_state = UX_HOST_CLASS_PIMA_SESSION_STATE_OPENED;
    }
    
    /* Return completion status.  */
    return(status);
}

