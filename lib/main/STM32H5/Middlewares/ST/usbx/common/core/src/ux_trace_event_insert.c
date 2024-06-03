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
/*    _ux_trace_event_insert                              PORTABLE C      */ 
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
/*    event_id                              User Event ID                 */ 
/*    info_field_1                          First information field       */ 
/*    info_field_2                          First information field       */ 
/*    info_field_3                          First information field       */ 
/*    info_field_4                          First information field       */ 
/*    current_event                         Current event pointer for     */ 
/*                                            post event update           */ 
/*    current_timestamp                     Timestamp for post event      */ 
/*                                            update                      */ 
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
VOID  _ux_trace_event_insert(ULONG event_id, ULONG info_field_1, ULONG info_field_2, ULONG info_field_3, ULONG info_field_4, 
                             ULONG filter, TX_TRACE_BUFFER_ENTRY **current_event, ULONG *current_timestamp)
{

UX_INTERRUPT_SAVE_AREA

TX_TRACE_BUFFER_ENTRY  *event;
ULONG                   timestamp;


    /* Disable interrupts.  */
    UX_DISABLE

    /* Pickup the current event.  */
    event =  _tx_trace_buffer_current_ptr;
    
    /* Insert this event into the trace buffer.  */
    TX_TRACE_IN_LINE_INSERT(event_id, info_field_1, info_field_2, info_field_3, info_field_4, filter)

    /* Initialize the timestamp to 0.  */
    timestamp =  0;

    /* Determine if the event was inserted.  */
    if (event)
    {
    
        /* Was the event inserted?  */
        if (event -> tx_trace_buffer_entry_event_id == event_id)
        {
    
            /* Yes, the event was inserted in the event trace so pickup the timestamp.  */
            timestamp =  event -> tx_trace_buffer_entry_time_stamp;
        }
        else
        {
        
            /* Event was not inserted, simply set the event pointer to NULL.  */
            event =  UX_NULL;
        }
    }
    
    /* Now determine if the caller requested the current event.  */
    if (current_event)
    {
        
        /* Yes, return the event pointer of potential subsequent update.  */
        *current_event =  event;
    }
        
    /* Now determine if the current timestamp was requested.  */
    if (current_timestamp)
    {
        
        /* Yes, return the current timestamp.  */
        *current_timestamp =  timestamp;
    }

    /* Restore interrupts.  */
    UX_RESTORE
}
#endif

