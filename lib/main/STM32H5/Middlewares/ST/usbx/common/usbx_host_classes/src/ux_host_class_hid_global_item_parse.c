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
/*    _ux_host_class_hid_global_item_parse                PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function parses a global item from the report descriptor.      */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hid                                   Pointer to HID class          */ 
/*    item                                  Pointer to item               */ 
/*    descriptor                            Pointer to descriptor         */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_hid_item_data_get      Get data item                 */ 
/*    _ux_utility_memory_copy               Copy memory block             */ 
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hid_global_item_parse(UX_HOST_CLASS_HID *hid, UX_HOST_CLASS_HID_ITEM *item, UCHAR *descriptor)
{

UX_HOST_CLASS_HID_PARSER     *hid_parser;


    /* Get the temporary parser structure pointer.  */
    hid_parser =  &hid -> ux_host_class_hid_parser;

    /* Get the tag of the item structure and process it.  */
    switch(item -> ux_host_class_hid_item_report_tag)
    {

    case UX_HOST_CLASS_HID_GLOBAL_TAG_USAGE_PAGE:

        /* Usage Page Tag.  */
        hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_usage_page =  
                                                _ux_host_class_hid_item_data_get(descriptor, item);
        break;


    case UX_HOST_CLASS_HID_GLOBAL_TAG_LOGICAL_MINIMUM:

        /* Logical Minimum Tag.  */
        hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_logical_min =  
                                                (SLONG) _ux_host_class_hid_item_data_get(descriptor, item);
        break;


    case UX_HOST_CLASS_HID_GLOBAL_TAG_LOGICAL_MAXIMUM:

        /* Logical Maximum Tag.  */
        hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_logical_max =  
                                                (SLONG) _ux_host_class_hid_item_data_get(descriptor, item);
        break;


    case UX_HOST_CLASS_HID_GLOBAL_TAG_PHYSICAL_MINIMUM:

        /* Physical Minimum Tag.  */
        hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_physical_min =  
                                                (SLONG) _ux_host_class_hid_item_data_get(descriptor, item);
        break;


    case UX_HOST_CLASS_HID_GLOBAL_TAG_PHYSICAL_MAXIMUM:

        /* Physical Maximum Tag.  */
        hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_physical_max =  
                                                (SLONG) _ux_host_class_hid_item_data_get(descriptor, item);
        break;


    case UX_HOST_CLASS_HID_GLOBAL_TAG_UNIT_EXPONENT:

        /* Unit Exponent Tag.  */
        hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_unit_expo =  
                                                _ux_host_class_hid_item_data_get(descriptor, item);
        break;

    
    case UX_HOST_CLASS_HID_GLOBAL_TAG_UNIT:

        /* Unit tag.  */
        hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_unit =  
                                                _ux_host_class_hid_item_data_get(descriptor, item);
        break;


    case UX_HOST_CLASS_HID_GLOBAL_TAG_REPORT_SIZE:

        /* Report Size tag.  */
        hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_report_size =  
                                                _ux_host_class_hid_item_data_get(descriptor, item);

        if (hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_report_size > UX_HOST_CLASS_HID_REPORT_SIZE)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_HID_REPORT_OVERFLOW);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_HID_REPORT_OVERFLOW, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)
        
            return(UX_HOST_CLASS_HID_REPORT_OVERFLOW);
        }        

        break;


    case UX_HOST_CLASS_HID_GLOBAL_TAG_REPORT_ID:

        /* Report ID tag.  */
        hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_report_id =  
                                                _ux_host_class_hid_item_data_get(descriptor, item);
        break;


    case UX_HOST_CLASS_HID_GLOBAL_TAG_REPORT_COUNT:

        /* Report Count tag.  */
        hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_report_count =  
                                                _ux_host_class_hid_item_data_get(descriptor, item);

        if (hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_report_count > UX_HOST_CLASS_HID_USAGES)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_HID_USAGE_OVERFLOW);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_HID_USAGE_OVERFLOW, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_HOST_CLASS_HID_USAGE_OVERFLOW);
        }
        
        break;

    case UX_HOST_CLASS_HID_GLOBAL_TAG_PUSH:

        /* Push tag.  */
        if (hid_parser -> ux_host_class_hid_parser_number_global >= UX_HOST_CLASS_HID_MAX_GLOBAL)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_HID_PUSH_OVERFLOW);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_HID_PUSH_OVERFLOW, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_HOST_CLASS_HID_PUSH_OVERFLOW);
        }
                        
        else
            _ux_utility_memory_copy(&hid_parser -> ux_host_class_hid_parser_global_pool[hid_parser -> ux_host_class_hid_parser_number_global++],
                                        &hid_parser -> ux_host_class_hid_parser_global, sizeof(UX_HOST_CLASS_HID_GLOBAL_ITEM)); /* Use case of memcpy is verified. */
        break;

     
    case UX_HOST_CLASS_HID_GLOBAL_TAG_POP:

        /* Pop tag.  */
        if(hid_parser -> ux_host_class_hid_parser_number_global == 0)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_HID_POP_UNDERFLOW);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_HID_POP_UNDERFLOW, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_HOST_CLASS_HID_POP_UNDERFLOW);
        }
        else
            _ux_utility_memory_copy(&hid_parser -> ux_host_class_hid_parser_global,
                                   &hid_parser -> ux_host_class_hid_parser_global_pool[--hid_parser -> ux_host_class_hid_parser_number_global],
                                        sizeof(UX_HOST_CLASS_HID_GLOBAL_ITEM)); /* Use case of memcpy is verified. */

        break;


    default:

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_HID_TAG_UNSUPPORTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_HID_TAG_UNSUPPORTED, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* This tag was not recognized or is not supported.  */
        return(UX_HOST_CLASS_HID_TAG_UNSUPPORTED);
    }                                               

    /* We get here when the tag has been processed successfully.  */
    return(UX_SUCCESS);
}

