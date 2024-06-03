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
/*    _ux_host_class_hid_report_id_get                    PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function retrieves the report after                            */
/*    report_id -> ux_host_class_hid_report_get_id and stores it in the   */
/*    same pointer. If report_id -> ux_host_class_hid_report_get_id is    */ 
/*    null, retrieves the first report of the type specified by           */
/*    report_id -> ux_host_class_hid_report_get_type.                     */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hid                                   Pointer to HID class          */ 
/*    report_id                             Report id structure           */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_semaphore_get                Get protection semaphore      */ 
/*    _ux_host_semaphore_put                Release protection semaphore  */ 
/*    _ux_host_stack_class_instance_verify  Verify class instance is valid*/ 
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
UINT  _ux_host_class_hid_report_id_get(UX_HOST_CLASS_HID *hid, UX_HOST_CLASS_HID_REPORT_GET_ID *report_id)
{
#if defined(UX_HOST_STANDALONE)
UX_INTERRUPT_SAVE_AREA
#endif
UINT                        status;
UX_HOST_CLASS_HID_REPORT    *next_hid_report;
                                                            

    /* Ensure the instance is valid.  */
    if (_ux_host_stack_class_instance_verify(_ux_system_host_class_hid_name, (VOID *) hid) != UX_SUCCESS)
    {        

        /* Error trap.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* Protect thread reentry to this instance.  */
    _ux_host_class_hid_lock_fail_return(hid);

    /* Check if this is the first report to get.  */
    if (report_id -> ux_host_class_hid_report_get_report == UX_NULL)
    {

        /* Check for the type of report ID to get (Input, Output, Feature).  */
        switch (report_id -> ux_host_class_hid_report_get_type)
        {

        case UX_HOST_CLASS_HID_REPORT_TYPE_INPUT           :

            /* Search for the input report ID. */
            next_hid_report =  hid -> ux_host_class_hid_parser.ux_host_class_hid_parser_input_report;
            break;

        case UX_HOST_CLASS_HID_REPORT_TYPE_OUTPUT          :

            /* Search for the output report ID. */
            next_hid_report =  hid -> ux_host_class_hid_parser.ux_host_class_hid_parser_output_report;
            break;

        case UX_HOST_CLASS_HID_REPORT_TYPE_FEATURE         :

            /* Search for the feature report ID. */
            next_hid_report =  hid -> ux_host_class_hid_parser.ux_host_class_hid_parser_feature_report;
            break;
            
        default :

            /* Error trap.  */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_HID_REPORT_ERROR);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_HID_REPORT_ERROR, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

            /* The report ID could not be found amongst the reports.  */
            next_hid_report =  UX_NULL;
            break;
        }
    }
    else
    {

        /* We had a report ID scan previously, point to the next report.  */
        next_hid_report =  report_id -> ux_host_class_hid_report_get_report -> ux_host_class_hid_report_next_report;
    }

    /* Did we find the next report?  */
    if (next_hid_report != UX_NULL)
    {

        /* We want the first report, memorize the ID.  */
        report_id -> ux_host_class_hid_report_get_id =  next_hid_report -> ux_host_class_hid_report_id;
        
        /* And remember where we left.  */
        report_id -> ux_host_class_hid_report_get_report =  next_hid_report;

        /* Successfully found next report.  */
        status =  UX_SUCCESS;
    }
    else
    {

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_HID_REPORT_ERROR, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* No more reports.  */
        status =  UX_HOST_CLASS_HID_REPORT_ERROR;
    }
    
    /* Unprotect thread reentry to this instance.  */
    _ux_host_class_hid_unlock(hid);

    /* The status variable has been set correctly.  */
    return(status);
}

