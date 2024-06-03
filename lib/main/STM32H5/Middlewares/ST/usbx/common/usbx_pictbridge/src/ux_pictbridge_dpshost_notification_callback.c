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
#include "ux_host_class_pima.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_pictbridge_dpshost_notification_callback        PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function gets a notification of an event and passes it to      */ 
/*    a pictbridge thread.                                                */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima_event                             Event notification structure */ 
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
/*    user application                                                    */ 
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
VOID  _ux_pictbridge_dpshost_notification_callback(UX_HOST_CLASS_PIMA_EVENT *pima_event)
{

UX_HOST_CLASS_PIMA      *pima;
UX_PICTBRIDGE           *pictbridge;
UX_PICTBRIDGE_EVENT     *pictbridge_next_event;
UX_PICTBRIDGE_EVENT     *pictbridge_event;

    /* Get the pointer to the Pima instance.  */
    pima = pima_event -> ux_host_class_pima_event_pima_instance;
    
    /* Get the pointer to the Pictbridge application.  */
    pictbridge = (UX_PICTBRIDGE *) pima -> ux_host_class_pima_application;

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
        return;

    /* Current storage is in the current head.  */
    pictbridge_event =  pictbridge -> ux_pictbridge_event_array_head;
    
    /* Store the event in the pictbridge event queue.  */
    pictbridge_event -> ux_pictbridge_event_code            =   pima_event -> ux_host_class_pima_event_code;
    pictbridge_event -> ux_pictbridge_event_parameter_1     =   pima_event -> ux_host_class_pima_event_parameter_1;
    pictbridge_event -> ux_pictbridge_event_parameter_2     =   pima_event -> ux_host_class_pima_event_parameter_2;
    pictbridge_event -> ux_pictbridge_event_parameter_3     =   pima_event -> ux_host_class_pima_event_parameter_3;

    /* Advance the pictbridge event queue head.  */
    pictbridge -> ux_pictbridge_event_array_head    = pictbridge_next_event;

    /* Wake up the Pictbridge notification handler thread.  */
    _ux_system_semaphore_put(&pictbridge -> ux_pictbridge_notification_semaphore);
    
    /* We are done.  */
    return;
    
}

