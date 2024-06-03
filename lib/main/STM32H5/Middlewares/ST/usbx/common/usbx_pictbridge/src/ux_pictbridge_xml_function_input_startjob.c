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


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_pictbridge_xml_function_input_startjob          PORTABLE C      */ 
/*                                                           6.1.12       */
/*                                                                        */ 
/*                                                                        */ 
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function receives an order for a new job.  It issues           */ 
/*    notification to the pictbridge thread.                              */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pictbridge                             Pictbridge instance          */ 
/*    input_variable                         Pointer to variable          */ 
/*    input_string                           Pointer to string            */ 
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
/*    _ux_pictbridge_object_parse                                         */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used macros for RTOS calls, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_pictbridge_xml_function_input_startjob(UX_PICTBRIDGE *pictbridge, 
                            UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter)
{
UX_PICTBRIDGE_EVENT         *pictbridge_next_event;
UX_PICTBRIDGE_EVENT         *pictbridge_event;

    UX_PARAMETER_NOT_USED(input_string);
    UX_PARAMETER_NOT_USED(input_variable);
    UX_PARAMETER_NOT_USED(xml_parameter);

    /* Compute the next entry in the event array.  */
    if ((pictbridge -> ux_pictbridge_event_array_head + 1) == pictbridge -> ux_pictbridge_event_array_end)
    
        /* Start at the beginning of the list.  */
        pictbridge_next_event = pictbridge -> ux_pictbridge_event_array;
    else

        /* Point to the next entry in the event array.  */
        pictbridge_next_event = pictbridge -> ux_pictbridge_event_array_head + 1;        

    /* Check to see if we can store this event.  */
    if (pictbridge_next_event == pictbridge -> ux_pictbridge_event_array_tail)
    
        /* No place to store this event, throw it away.  */
        return(UX_ERROR);

    /* Current storage is in the current head.  */
    pictbridge_event =  pictbridge -> ux_pictbridge_event_array_head;
    
    /* Store the start job event in the pictbridge event queue.  */
    pictbridge_event -> ux_pictbridge_event_code            =   UX_PICTBRIDGE_EC_START_JOB;
    pictbridge_event -> ux_pictbridge_event_parameter_1     =   0;
    pictbridge_event -> ux_pictbridge_event_parameter_2     =   0;
    pictbridge_event -> ux_pictbridge_event_parameter_3     =   0;

    /* Advance the pictbridge event queue head.  */
    pictbridge -> ux_pictbridge_event_array_head    = pictbridge_next_event;

    /* Wake up the Pictbridge notification handler thread.  */
    _ux_system_semaphore_put(&pictbridge -> ux_pictbridge_notification_semaphore);

    /* This function never fails.  */
    return(UX_SUCCESS);
}


