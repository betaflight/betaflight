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
/**   Trace                                                               */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#ifndef UX_SOURCE_CODE
#define UX_SOURCE_CODE
#endif


/* Include necessary system files.  */

#include "ux_api.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_trace_event_update                              PORTABLE C      */ 
/*                                                           6.1.9        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function inserts a USBX event into the current trace buffer.   */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    event                                 Event pointer                 */ 
/*    timestamp                             Timestamp of the event        */ 
/*    event_id                              User Event ID                 */ 
/*    info_field_1                          First information field       */ 
/*    info_field_2                          First information field       */ 
/*    info_field_3                          First information field       */ 
/*    info_field_4                          First information field       */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Internal USBX Functions                                             */ 
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
/*  10-15-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            improved traceX support,    */
/*                                            resulting in version 6.1.9  */
/*                                                                        */
/**************************************************************************/
#ifdef UX_ENABLE_EVENT_TRACE
VOID  _ux_trace_event_update(TX_TRACE_BUFFER_ENTRY *event, ULONG timestamp, ULONG event_id, ULONG info_field_1, ULONG info_field_2, ULONG info_field_3, ULONG info_field_4)
{

UX_INTERRUPT_SAVE_AREA


    /* Disable interrupts.  */
    UX_DISABLE

    /* Determine if the event exists and is still the event originally inserted into the trace.  */
    if ((event) && (event -> tx_trace_buffer_entry_event_id == event_id) && (event -> tx_trace_buffer_entry_time_stamp == timestamp))
    {
    
        /* Yes, update this trace entry based on the info input parameters.  */
        
        /* Check for info field 1 update.  */
        if (info_field_1)
        {
        
            /* Yes, update info field 1.  */
            event -> tx_trace_buffer_entry_information_field_1 =  info_field_1;
        }

        /* Check for info field 2 update.  */
        if (info_field_2)
        {
        
            /* Yes, update info field 2.  */
            event -> tx_trace_buffer_entry_information_field_2 =  info_field_2;
        }

        /* Check for info field 3 update.  */
        if (info_field_3)
        {
        
            /* Yes, update info field 3.  */
            event -> tx_trace_buffer_entry_information_field_3 =  info_field_3;
        }

        /* Check for info field 4 update.  */
        if (info_field_4)
        {
        
            /* Yes, update info field 4.  */
            event -> tx_trace_buffer_entry_information_field_4 =  info_field_4;
        }
    }
    /* Restore interrupts.  */
    UX_RESTORE
}
#endif

