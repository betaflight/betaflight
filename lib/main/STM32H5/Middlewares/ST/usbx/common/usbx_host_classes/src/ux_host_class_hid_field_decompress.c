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
/*    _ux_host_class_hid_field_decompress                 PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will decompress a field and return the usage/value.   */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hid_field                             Pointer to HID field          */ 
/*    report_buffer                         Pointer to report buffer      */ 
/*    client_report                         Pointer to client report      */ 
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
UINT  _ux_host_class_hid_field_decompress(UX_HOST_CLASS_HID_FIELD *hid_field, UCHAR *report_buffer, UX_HOST_CLASS_HID_CLIENT_REPORT *client_report)
{

ULONG       field_report_count;
ULONG       field_report_size;
ULONG       data_offset_byte;
ULONG       data_offset_bit;
ULONG       data_offset_bit_in_report;
ULONG       field_value;
ULONG       field_value_bit_shifting;
ULONG       field_usage;
ULONG       report_content;


    /* Calculate the address of the beginning of the field in the report.  */
    data_offset_byte =  (hid_field -> ux_host_class_hid_field_report_offset >> 3);

    /* Calculate the bit start address. Hopefully things will be on a boundary but not necessary.  */
    data_offset_bit =  hid_field -> ux_host_class_hid_field_report_offset & 7;

    /* Each report field has a report_count value. This count is used to extract values from the 
       incoming report and build each usage/value instance.  */
    for (field_report_count = 0; field_report_count < hid_field -> ux_host_class_hid_field_report_count; field_report_count++)
    {

        /* Get the report size in bits.  */
        field_report_size =  hid_field -> ux_host_class_hid_field_report_size;
        
        /* We use the bit offset for this report and not the generic field bit offset.  */
        data_offset_bit_in_report = data_offset_bit;
        
        /* Reset the local value.  */
        field_value =  0;
        
        /* And start with bit 0 in the target value.  */
        field_value_bit_shifting =  0;
        
        /* Build the value field bit by bit.  */
        while (field_report_size-- != 0)
        {

            /* Read the content. This method is redundant if we are not changing byte but it 
               makes the algorithm much easier.  */
            report_content =  (ULONG) *(report_buffer + data_offset_byte + (data_offset_bit_in_report >> 3));        
            
            /* Shift the current value content to allow space to store the new bit.  */
            field_value |=  ((report_content >> (data_offset_bit_in_report & 7)) & 1) << field_value_bit_shifting;
                
            /* Move to next bit in the report.  */
            data_offset_bit_in_report++;
            
            /* And the next bit in the value.  */
            field_value_bit_shifting++;
        }        

        /* The Usage value will depend if the data is defined as a variable or an array in the HID report.  */
        if (hid_field -> ux_host_class_hid_field_value & UX_HOST_CLASS_HID_ITEM_VARIABLE)
        {

            /* Take the usage directly from the usage array.  */
            field_usage =  *(hid_field -> ux_host_class_hid_field_usages + field_report_count);
        }
        else
        {

            /* This is an array, so compute the usage from the min value, the report count and the 
               computed report value.  */           
            field_usage =  hid_field -> ux_host_class_hid_field_usage_min + (ULONG)((SLONG)field_value - hid_field -> ux_host_class_hid_field_logical_min);

            /* Also add the usage page.  */
            field_usage |=  (hid_field -> ux_host_class_hid_field_usage_page << 16);
        }

        /* Put the value and the usage into the caller's buffer.  */
        *(client_report -> ux_host_class_hid_client_report_buffer + client_report -> ux_host_class_hid_client_report_actual_length) =  field_usage;
        client_report -> ux_host_class_hid_client_report_actual_length++;
        *(client_report -> ux_host_class_hid_client_report_buffer + client_report -> ux_host_class_hid_client_report_actual_length) =  field_value;
        client_report -> ux_host_class_hid_client_report_actual_length++;

        /* Calculate the next address for this field.  */
        data_offset_bit +=  hid_field -> ux_host_class_hid_field_report_size;
    }              

    /* Return successful completion.  */
    return(UX_SUCCESS);
}

