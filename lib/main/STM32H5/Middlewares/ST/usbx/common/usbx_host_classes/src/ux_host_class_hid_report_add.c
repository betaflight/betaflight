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
/*    _ux_host_class_hid_report_add                       PORTABLE C      */ 
/*                                                           6.1.8        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function adds a report (input/output/feature) to the current   */ 
/*    parser.                                                             */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hid                                   Pointer to HID class          */ 
/*    descriptor                            Pointer to descriptor         */ 
/*    item                                  Pointer to item               */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_hid_item_data_get      Get data item                 */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*    _ux_utility_memory_copy               Copy memory block             */ 
/*    _ux_utility_memory_free               Release memory block          */ 
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
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*  08-02-2021     Wen Wang                 Modified comment(s),          */
/*                                            fixed spelling error,       */
/*                                            resulting in version 6.1.8  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hid_report_add(UX_HOST_CLASS_HID *hid, UCHAR *descriptor, UX_HOST_CLASS_HID_ITEM *item)
{

UX_HOST_CLASS_HID_PARSER    *hid_parser;
ULONG                       hid_field_value;
ULONG                       hid_field_count;
UX_HOST_CLASS_HID_REPORT    *new_hid_report;
UX_HOST_CLASS_HID_REPORT    *hid_report;
UX_HOST_CLASS_HID_FIELD     *hid_field;
UX_HOST_CLASS_HID_FIELD     *new_hid_field;
ULONG                       current_field_address;


    /* Get the parser structure pointer.  */
    hid_parser =  &hid -> ux_host_class_hid_parser;

    /* Obtain the field value from the report.  */
    hid_field_value =  _ux_host_class_hid_item_data_get(descriptor, item);

    /* Allocate some memory to store this new report.  */
    new_hid_report =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HOST_CLASS_HID_REPORT));
    if (new_hid_report == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* We need to select the report entry based on the type of report. If the entry in the 
       report chain is NULL, this is the first report so update the start of the chain.  */
    switch (item -> ux_host_class_hid_item_report_tag)
    {

    case UX_HOST_CLASS_HID_MAIN_TAG_INPUT:

        hid_report =  hid_parser -> ux_host_class_hid_parser_input_report;
        if (hid_report == UX_NULL)
            hid_parser -> ux_host_class_hid_parser_input_report =  new_hid_report;

        /* This is a Input report.  */
        new_hid_report -> ux_host_class_hid_report_type = UX_HOST_CLASS_HID_REPORT_TYPE_INPUT;
        break;              

    case UX_HOST_CLASS_HID_MAIN_TAG_OUTPUT:

        hid_report =  hid_parser -> ux_host_class_hid_parser_output_report;
        if (hid_report == UX_NULL)
            hid_parser -> ux_host_class_hid_parser_output_report =  new_hid_report;

        /* This is output report.  */
        new_hid_report -> ux_host_class_hid_report_type = UX_HOST_CLASS_HID_REPORT_TYPE_OUTPUT;
        break;
        
    case UX_HOST_CLASS_HID_MAIN_TAG_FEATURE:

        hid_report =  hid_parser -> ux_host_class_hid_parser_feature_report;
        if (hid_report == UX_NULL)
            hid_parser -> ux_host_class_hid_parser_feature_report =  new_hid_report;

        /* This is a Feature report.  */
        new_hid_report -> ux_host_class_hid_report_type = UX_HOST_CLASS_HID_REPORT_TYPE_FEATURE;
        break;              


    default:

        _ux_utility_memory_free(new_hid_report);

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_HID_REPORT_ERROR);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_HID_REPORT_ERROR, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Return an error.  */
        return(UX_HOST_CLASS_HID_REPORT_ERROR);
    }    

    /* If there is a preceding report, locate the end of the report chain.  */
    if (hid_report != UX_NULL)
    {

        while (hid_report -> ux_host_class_hid_report_next_report != UX_NULL)
            hid_report =  hid_report -> ux_host_class_hid_report_next_report;
    }       

    /* If this report is part of the current global report, use the last report 
       to add the fields.  */
    if ((hid_report != UX_NULL) && (hid_report -> ux_host_class_hid_report_id == hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_report_id))
    {

        /* So we did not need a new report after all!  */
        _ux_utility_memory_free(new_hid_report);
        new_hid_report =  hid_report;
    }
    else
    {

        /* We do have to build a new report. Add the new one to the chain.  */
        if (hid_report != UX_NULL)
            hid_report -> ux_host_class_hid_report_next_report =  new_hid_report;

        /* Add the new report ID.  */
        new_hid_report -> ux_host_class_hid_report_id =  hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_report_id;
    }

    /* Compute the size of the report. The size is first calculated in bits.  */
    current_field_address =  new_hid_report -> ux_host_class_hid_report_bit_length;
    new_hid_report -> ux_host_class_hid_report_bit_length +=  hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_report_size*
                                                            hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_report_count;

    /* Now compute the size in bytes (easier for the end/receive reports functions).  */
    new_hid_report -> ux_host_class_hid_report_byte_length =  new_hid_report -> ux_host_class_hid_report_bit_length >> 3;

    /* Take care of the bit padding if necessary.  */
    if (new_hid_report -> ux_host_class_hid_report_bit_length & 7)
        new_hid_report -> ux_host_class_hid_report_byte_length++;
    
    /* Get the number of field usage for this report, this value depends on the report type. If this is 
       an array, we use the number of usages; if this a variable, we use the report count.  */
    if (hid_field_value & UX_HOST_CLASS_HID_ITEM_VARIABLE)
        hid_field_count =  hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_report_count;
    else
        hid_field_count =  hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_number_usage;

    /* If the field count is null, this is only padding and there is no field to be allocated to the report.  */
    if (hid_field_count == 0)       
        return(UX_SUCCESS);

    /* Create the field structure.  */
    new_hid_field =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HOST_CLASS_HID_FIELD));
    if (new_hid_field == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Attach the new field to the report. The report may already contain a field, if so parse the chain 
       until we reach the end of the chain.  */    
    new_hid_report -> ux_host_class_hid_report_number_item += hid_field_count;
    if (new_hid_report -> ux_host_class_hid_report_field == UX_NULL)
    {

        /* This is the first field for the report.  */
        new_hid_report -> ux_host_class_hid_report_field =  new_hid_field;
    }
    else
    {    

        /* We have previous HID fields, so search for the end of the chain.  */
        hid_field =  new_hid_report -> ux_host_class_hid_report_field;
        while(hid_field -> ux_host_class_hid_field_next_field != UX_NULL)
            hid_field =  hid_field -> ux_host_class_hid_field_next_field;

        /* Attach the new field to the end of the chain.  */
        hid_field -> ux_host_class_hid_field_next_field =  new_hid_field;
    } 
                          
    /* From the parser structure, update the new field values. Start with logical values.  */
    new_hid_field -> ux_host_class_hid_field_logical_min =  hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_logical_min;
    new_hid_field -> ux_host_class_hid_field_logical_max =  hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_logical_max;

    /* Then the usage values. Note that these are only used if the item is an array.  */
    new_hid_field -> ux_host_class_hid_field_usage_page =  hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_usage_page;
    new_hid_field -> ux_host_class_hid_field_usage_min =  hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_usage_min;
    new_hid_field -> ux_host_class_hid_field_usage_max =  hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_usage_max;

    /* Then physical values.  */
    new_hid_field -> ux_host_class_hid_field_physical_min =  hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_physical_min;
    new_hid_field -> ux_host_class_hid_field_physical_max =  hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_physical_max;

    /* Then unit values.  */
    new_hid_field -> ux_host_class_hid_field_unit =       hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_unit;
    new_hid_field -> ux_host_class_hid_field_unit_expo =  hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_unit_expo;

    /* Then report values.  */
    new_hid_field -> ux_host_class_hid_field_report_type =   item -> ux_host_class_hid_item_report_tag;
    new_hid_field -> ux_host_class_hid_field_report_id =     hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_report_id;
    new_hid_field -> ux_host_class_hid_field_report_size =   hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_report_size;
    new_hid_field -> ux_host_class_hid_field_report_count =  hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_report_count;
    new_hid_field -> ux_host_class_hid_field_report_offset = current_field_address;

    /* Save the HID field value.  */
    new_hid_field -> ux_host_class_hid_field_value =  hid_field_value;

    /* We need some memory for the values.  */
    new_hid_field -> ux_host_class_hid_field_values =  _ux_utility_memory_allocate_mulc_safe(UX_NO_ALIGN, UX_REGULAR_MEMORY,
                                                                                    new_hid_field -> ux_host_class_hid_field_report_count, 4);

    /* Check the memory pointer. */
    if (new_hid_field -> ux_host_class_hid_field_values == UX_NULL)
    {

        _ux_utility_memory_free(new_hid_field);
        return(UX_MEMORY_INSUFFICIENT);
    }
    
    /* We need some memory for the usages, but only for variable items; usage
       values for array items can be calculated.  */
    if (hid_field_value & UX_HOST_CLASS_HID_ITEM_VARIABLE)
    {

        /* Allocate memory for the usages.  */
        new_hid_field -> ux_host_class_hid_field_usages =  _ux_utility_memory_allocate_mulc_safe(UX_NO_ALIGN, UX_REGULAR_MEMORY, hid_field_count, 4);
        if (new_hid_field -> ux_host_class_hid_field_usages == UX_NULL)                   
        {

            _ux_utility_memory_free(new_hid_field -> ux_host_class_hid_field_values);
            _ux_utility_memory_free(new_hid_field);
            return(UX_MEMORY_INSUFFICIENT);
        }
        
        /* Copy the current usages in the field structure.  */
        _ux_utility_memory_copy(new_hid_field -> ux_host_class_hid_field_usages, hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_usages, hid_field_count*4); /* Use case of memcpy is verified. */
    }

    /* Save the number of usages.  */
    new_hid_field -> ux_host_class_hid_field_number_usage =  hid_field_count;

    /* Return successful completion.  */
    return(UX_SUCCESS);
}

