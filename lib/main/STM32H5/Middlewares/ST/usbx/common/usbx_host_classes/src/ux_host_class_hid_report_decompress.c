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
/*    _ux_host_class_hid_report_decompress                PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will decompress a raw report into a client buffer.    */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hid                                   Pointer to HID class          */ 
/*    client_report                         Pointer to client report      */ 
/*    report_buffer                         Pointer to report buffer      */ 
/*    report_length                         Length of report              */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_hid_field_decompress   Decompress field              */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    HID Class                                                           */ 
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
UINT  _ux_host_class_hid_report_decompress(UX_HOST_CLASS_HID *hid, UX_HOST_CLASS_HID_CLIENT_REPORT *client_report,
                                           UCHAR  *report_buffer, ULONG report_length)
{

UX_HOST_CLASS_HID_REPORT     *hid_report;
UX_HOST_CLASS_HID_FIELD      *hid_field;
    
    UX_PARAMETER_NOT_USED(hid);

    /* Get the report pointer from the caller.  */
    hid_report =  client_report -> ux_host_class_hid_client_report;

    /* Check if this report has a ID field in the front. An ID field is required 
       if report ID is non null.  */
    if (hid_report -> ux_host_class_hid_report_id != 0)
    {

        /* We have an ID tag in the report. The ID tag is the first byte of the report. Skip the 
           ID tag and adjust the length.  */
        report_buffer++;
        report_length--;
    }

    /* Get the first field associated with the report.  */
    hid_field =  hid_report -> ux_host_class_hid_report_field;
    
    /* We need to decompress each field defined in the report.  */
    while (hid_field != UX_NULL)
    {

        /* Decompress a field.  */
        _ux_host_class_hid_field_decompress(hid_field, report_buffer, client_report);

        /* Move to the next field.  */
        hid_field =  hid_field -> ux_host_class_hid_field_next_field;
    }

    /* Return successful completion.  */ 
    return(UX_SUCCESS);
}

