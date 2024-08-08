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
/*    _ux_host_class_hid_report_compress                  PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will compress a client report into a report buffer.   */ 
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
/*    None                                                                */ 
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
UINT  _ux_host_class_hid_report_compress(UX_HOST_CLASS_HID *hid, UX_HOST_CLASS_HID_CLIENT_REPORT *client_report,
                                                    UCHAR *report_buffer, ULONG report_length)
{

UX_HOST_CLASS_HID_REPORT    *hid_report;
UX_HOST_CLASS_HID_FIELD     *hid_field;
ULONG                       field_usage;
ULONG                       field_report_count;
ULONG                       field_report_size;
ULONG                       *client_buffer;
ULONG                       client_value;
UCHAR                       value;
ULONG                       data_offset_bit;
UCHAR                       is_valid_usage;

    UX_PARAMETER_NOT_USED(hid);
    UX_PARAMETER_NOT_USED(report_length);

    /* Get the report pointer from the caller.  */
    hid_report =  client_report -> ux_host_class_hid_client_report;

    /* Get the pointer to the user buffer.  */
    client_buffer =  client_report -> ux_host_class_hid_client_report_buffer;

    /* Get the first field associated with the report.  */
    hid_field =  hid_report -> ux_host_class_hid_report_field;

    /* Set data offset bit.  */
    data_offset_bit =  0;

    /* We need to compress each field defined in the report.  */
    while (hid_field != UX_NULL)
    {

        /* Each report field has a report count value. This count is used to extract
           values from the incoming report and build each usage/value instance.  */
        for (field_report_count = 0; field_report_count < hid_field -> ux_host_class_hid_field_report_count; field_report_count++)
        {

            /* Ensure the usage in the client buffer is valid. How we determine
               this depends on whether the field is VARIABLE or ARRAY.  */
            is_valid_usage =  UX_FALSE;
            if (hid_field -> ux_host_class_hid_field_value & UX_HOST_CLASS_HID_ITEM_VARIABLE)
            {

                /* Go through the usage array to try and find the client's usage.  */
                for (field_usage = 0; field_usage < hid_field -> ux_host_class_hid_field_number_usage; field_usage++)
                {

                    /* Is this a usage we've recorded?  */
                    if (*client_buffer == hid_field -> ux_host_class_hid_field_usages[field_usage])
                    {

                        /* Yes, this is a valid usage.  */
                        is_valid_usage =  UX_TRUE;
                        break;
                    }
                }
            }
            else
            {

                /* Is the usage page valid?  */
                if (((*client_buffer & 0xffff0000) >> 16) == hid_field -> ux_host_class_hid_field_usage_page)
                {

                    /* Is the usage in the min and max range?  */
                    if ((*client_buffer & 0xffff) >= hid_field -> ux_host_class_hid_field_usage_min &&
                        (*client_buffer & 0xffff) <= hid_field -> ux_host_class_hid_field_usage_max)
                    {

                        /* Yes, this is a valid usage.  */
                        is_valid_usage =  UX_TRUE;
                    }
                }
            }

            /* Is the usage invalid?  */
            if (is_valid_usage == UX_FALSE)
            {

                /* Error trap. */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_HID_REPORT_ERROR);

                /* If trace is enabled, insert this event into the trace buffer.  */
                UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_HID_REPORT_ERROR, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

                return(UX_HOST_CLASS_HID_REPORT_ERROR);
            }

            /* Skip the usage and point the buffer to the value.  */
            client_buffer++;

            /* Read the client value.  */
            client_value =  *client_buffer++;

            /* Build the value field in the report buffer bit by bit.  */
            for (field_report_size = hid_field -> ux_host_class_hid_field_report_size; field_report_size > 0; field_report_size--)
            {

                /* Isolate each bit from the report value.  */
                value =  (UCHAR) client_value & 1;
                
                /* Shift the isolated bit to its right space in the report byte.  */
                value =  (UCHAR)(value << data_offset_bit);

                /* Update the report with the bit value.  */
                *report_buffer |=  value;

                /* Move to next bit.  */
                data_offset_bit++;

                /* Are we on a byte boundary.  */
                if ((data_offset_bit & 7) == 0)
                {

                    /* If so increment the report address.  */
                    report_buffer++;
            
                    /* Reset offset bit.  */
                    data_offset_bit =  0;
                }

                /* Move to the next bit.  */
                client_value =  client_value >> 1;
            }
        }        

        /* Move to the next field.  */
        hid_field =  hid_field -> ux_host_class_hid_field_next_field;
    }

    /* Return successful completion.  */
    return(UX_SUCCESS);
}
