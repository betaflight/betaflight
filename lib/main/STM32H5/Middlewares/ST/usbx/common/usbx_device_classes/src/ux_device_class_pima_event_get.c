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
/**   Device PIMA Class                                                   */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_pima.h"
#include "ux_device_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_pima_event_get                     PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function checks if there is an event from the application      */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                     Address of pima class      */ 
/*    event                                    Pointer of the event       */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                   UX_SUCCESS if there is an  */ 
/*                                             event                      */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    ThreadX                                                             */ 
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
UINT  _ux_device_class_pima_event_get(UX_SLAVE_CLASS_PIMA *pima, 
                                      UX_SLAVE_CLASS_PIMA_EVENT *pima_event)
{

UX_SLAVE_CLASS_PIMA_EVENT   *current_pima_event;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PIMA_EVENT_GET, pima, pima_event, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Check if the head and the tail of the event array is the same.  */
    if (pima -> ux_device_class_pima_event_array_head == 
        pima -> ux_device_class_pima_event_array_tail)

        /* No event to report.  */
        return(UX_ERROR);        

    /* There is an event to report, get the current pointer to the event.  */
    current_pima_event =  pima -> ux_device_class_pima_event_array_tail;
    
    /* fill in the event structure from the user.  */
    pima_event -> ux_device_class_pima_event_code            =  current_pima_event -> ux_device_class_pima_event_code;
    pima_event -> ux_device_class_pima_event_session_id      =  pima -> ux_device_class_pima_session_id;
    pima_event -> ux_device_class_pima_event_transaction_id  =  pima -> ux_device_class_pima_transaction_id;
    pima_event -> ux_device_class_pima_event_parameter_1     =  current_pima_event -> ux_device_class_pima_event_parameter_1;
    pima_event -> ux_device_class_pima_event_parameter_2     =  current_pima_event -> ux_device_class_pima_event_parameter_2;
    pima_event -> ux_device_class_pima_event_parameter_3     =  current_pima_event -> ux_device_class_pima_event_parameter_3;


    /* Adjust the tail pointer.  Check if we are at the end.  */
    if ((current_pima_event + 1) == pima -> ux_device_class_pima_event_array_end)

        /* We are at the end, go back to the beginning.  */
        pima -> ux_device_class_pima_event_array_tail =  pima -> ux_device_class_pima_event_array;
        
    else        
        /* We are not at the end, increment the tail position.  */
        pima -> ux_device_class_pima_event_array_tail++;


    /* Return event status to the user.  */
    return(UX_SUCCESS);
}

