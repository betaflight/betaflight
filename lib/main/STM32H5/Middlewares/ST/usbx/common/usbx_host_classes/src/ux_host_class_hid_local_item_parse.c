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
/*    _ux_host_class_hid_local_item_parse                 PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function parses a local item from the report descriptor.       */ 
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
UINT  _ux_host_class_hid_local_item_parse(UX_HOST_CLASS_HID *hid, UX_HOST_CLASS_HID_ITEM *item, UCHAR *descriptor)
{

UX_HOST_CLASS_HID_PARSER    *hid_parser;
ULONG                       usage;
ULONG                       usage_min;
ULONG                       usage_max;
ULONG                       delimiter_set;

    /* Get the temporary parser structure pointer.  */
    hid_parser =  &hid -> ux_host_class_hid_parser;
    
    /* Analyze the tag.  */
    switch (item -> ux_host_class_hid_item_report_tag)
    {
 
    case UX_HOST_CLASS_HID_LOCAL_TAG_USAGE:

        /* Local usage tag, check if we have an overflow.  */
        if (hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_number_usage == UX_HOST_CLASS_HID_USAGES)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_HID_USAGE_OVERFLOW);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_HID_USAGE_OVERFLOW, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_HOST_CLASS_HID_USAGE_OVERFLOW); 
        }

        /* Obtain the usage from the descriptor.  */
        usage =  _ux_host_class_hid_item_data_get(descriptor, item);

        /* Combine the global usage with the local usage to form a unique usage ID.  */
        usage |= (hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_usage_page << 16);

        /* Add the usage to the local usage table.  */
        hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_usages[hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_number_usage] =  usage;

        /* We have one more usage now.  */
        hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_number_usage++;

        break;


    case UX_HOST_CLASS_HID_LOCAL_TAG_USAGE_MINIMUM:

        /* Usage Minimum tag.  */
        hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_usage_min =  
                                                (ULONG) _ux_host_class_hid_item_data_get(descriptor, item);

        break;


    case UX_HOST_CLASS_HID_LOCAL_TAG_USAGE_MAXIMUM:

        /* Usage Maximum tag.  */
        hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_usage_max =  
                                                (ULONG) _ux_host_class_hid_item_data_get(descriptor, item);

        /* Check if the maximum value is coherent with the minimum.  */
        if (hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_usage_max < hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_usage_min)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_HID_MIN_MAX_ERROR);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_HID_MIN_MAX_ERROR, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_HOST_CLASS_HID_MIN_MAX_ERROR);
        }
        
        /* Get the boundaries for the usage values which are defined when encountering the USAGE MAX tag.  */
        usage_min =  (ULONG)(hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_usage_min);
        usage_max =  (ULONG)(hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_usage_max);

        while (usage_min <= usage_max)
        {

            /* Check if we can still add this usage.  */
            if (hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_number_usage == UX_HOST_CLASS_HID_USAGES)
            {

                /* Error trap. */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_HID_USAGE_OVERFLOW);

                /* If trace is enabled, insert this event into the trace buffer.  */
                UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_HID_USAGE_OVERFLOW, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

                return(UX_HOST_CLASS_HID_USAGE_OVERFLOW);

            }
                
            /* Combine the global usage with the local usage to form a unique usage ID.  */
            usage =  usage_min | (hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_usage_page << 16);

            /* Add the usage to the local usage table.  */
            hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_usages[hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_number_usage] =  usage;

            /* We have one more usage now.  */
            hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_number_usage++;
        
            /* Next usage value.  */
            usage_min++;                        
        }

        break;

    case UX_HOST_CLASS_HID_LOCAL_TAG_DELIMITER:

        /* Obtain the delimiter set from the descriptor.  */
        delimiter_set =  _ux_host_class_hid_item_data_get(descriptor, item);

        /* We should have either an open or a close.  */
        switch (delimiter_set)
        {

        case UX_HOST_CLASS_HID_DELIMITER_OPEN:

            /* Recursive delimiter opens are not supported.  */
            if (hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_delimiter_level == 1)
            {

                /* Error trap. */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_HID_DELIMITER_ERROR);

                return(UX_HOST_CLASS_HID_DELIMITER_ERROR);
            }                

            /* Mark the opening of the delimiter.  */
            hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_delimiter_level =  1;

            break;                    

        case UX_HOST_CLASS_HID_DELIMITER_CLOSE:

            /* Ensure we had an open delimiter before.  */
            if (hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_delimiter_level == 0)
            {

                /* Error trap. */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_HID_DELIMITER_ERROR);

                return(UX_HOST_CLASS_HID_DELIMITER_ERROR);
            }                

            /* Mark the closing of the delimiter.  */
            hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_delimiter_level =  0;

            break;                    

        default:

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_HID_DELIMITER_ERROR);

            /* We got a wrong delimiter set.  */
            return(UX_HOST_CLASS_HID_DELIMITER_ERROR);
        } 
        break;

    default:

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_HID_TAG_UNSUPPORTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_HID_TAG_UNSUPPORTED, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* This tag is either unknown or unsupported.  */
        return(UX_HOST_CLASS_HID_TAG_UNSUPPORTED);
    }                                               
    
    /* Return status. Always SUCCESS if we get here.*/
    return(UX_SUCCESS);
}

