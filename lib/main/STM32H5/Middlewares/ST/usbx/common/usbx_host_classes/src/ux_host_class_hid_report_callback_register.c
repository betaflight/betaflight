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
/**   HID Class                                                           */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hid_report_callback_register         PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will register a report callback to a HID report.      */
/*    This function should be called by a HID client when it has been     */
/*    instantiated by a new HID device.                                   */
/*                                                                        */
/*    The registration process will allow the HID class to call a         */ 
/*    function in the HID client when an asynchronous report is present.  */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hid                                   Pointer to HID class          */ 
/*    call_back                             HID report callback           */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_class_instance_verify  Verify class instance is valid*/ 
/*    _ux_host_semaphore_get                Get protection semaphore      */ 
/*    _ux_host_semaphore_put                Release protection semaphore  */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
/*    HID Class                                                           */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hid_report_callback_register(UX_HOST_CLASS_HID *hid, UX_HOST_CLASS_HID_REPORT_CALLBACK *call_back)
{
#if defined(UX_HOST_STANDALONE)
UX_INTERRUPT_SAVE_AREA
#endif
UINT                            status;
UX_HOST_CLASS_HID_REPORT         *hid_report;


    /* Ensure the instance is valid.  */
    if (_ux_host_stack_class_instance_verify(_ux_system_host_class_hid_name, (VOID *) hid) != UX_SUCCESS)
    {        

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* Protect thread reentry to this instance.  */
    _ux_host_class_hid_lock_fail_return(hid);

    /* Search for the report ID. Note that this can only be an Input report!  */
    hid_report =  hid -> ux_host_class_hid_parser.ux_host_class_hid_parser_input_report;
    
    /* Parse all the report IDs in search of the one specified by the user.  */
    while (hid_report != UX_NULL)
    {

        /* Check report ID.  */
        if (hid_report -> ux_host_class_hid_report_id == call_back -> ux_host_class_hid_report_callback_id)
        {
            
            /* We have found the correct report. Set the call back function, buffer and flags.  */
            hid_report -> ux_host_class_hid_report_callback_function =   call_back -> ux_host_class_hid_report_callback_function;
            hid_report -> ux_host_class_hid_report_callback_buffer   =   call_back -> ux_host_class_hid_report_callback_buffer;
            hid_report -> ux_host_class_hid_report_callback_flags    =   call_back -> ux_host_class_hid_report_callback_flags;
            hid_report -> ux_host_class_hid_report_callback_length   =   call_back -> ux_host_class_hid_report_callback_length;

            /* Unprotect thread reentry to this instance.  */
            _ux_host_class_hid_unlock(hid);

            /* Tell the user the report was OK and the call back was registered.  */
            return(UX_SUCCESS);
        }
        
        /* Jump to next report.  */
        hid_report = hid_report -> ux_host_class_hid_report_next_report; 
    }

    /* Unprotect thread reentry to this instance.  */
    _ux_host_class_hid_unlock(hid);

    /* Error trap. */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_HID_REPORT_ERROR);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_HID_REPORT_ERROR, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

    /* The report ID could not be found amongst the reports.  */
    return(UX_HOST_CLASS_HID_REPORT_ERROR);
}

