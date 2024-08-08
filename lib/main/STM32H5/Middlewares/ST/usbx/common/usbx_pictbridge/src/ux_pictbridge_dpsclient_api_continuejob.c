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
/**   Pictbridge Application                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_pictbridge.h"
#include "ux_device_class_pima.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_pictbridge_dpsclient_api_continue_job           PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function creates the tag lines of the start job function       */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pictbridge                            Pictbridge instance           */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    application                                                         */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used UX prefix to refer to  */
/*                                            TX symbols instead of using */
/*                                            them directly,              */
/*                                            resulting in version 6.1    */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used macros for RTOS calls, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT _ux_pictbridge_dpsclient_api_continue_job(UX_PICTBRIDGE *pictbridge)
{
UINT                                status;
ULONG                               actual_flags;

    /* Prepare the object for capability. */
    status = _ux_pictbridge_dpsclient_input_object_prepare(pictbridge, UX_PICTBRIDGE_OR_CONTINUE_JOB, 0, 0);
    
    /* Check status.  */
    if (status != UX_SUCCESS)
        return(status);

    /* We should wait for the host to send a script with the response.  */
    status =  _ux_system_event_flags_get(&pictbridge -> ux_pictbridge_event_flags_group, UX_PICTBRIDGE_EVENT_FLAG_CONTINUE_JOB, 
                                        UX_AND_CLEAR, &actual_flags, UX_PICTBRIDGE_EVENT_TIMEOUT);

    /* Check status.  */
    if (status != UX_SUCCESS)
        return(UX_EVENT_ERROR);

    /* Ensure the flag was set.  */
    if (actual_flags & UX_PICTBRIDGE_EVENT_FLAG_CONTINUE_JOB)
        
        /* Return completion status.  */
        return(UX_SUCCESS);    

    else
            
        /* Return completion status.  */
        return(UX_ERROR);    
}

