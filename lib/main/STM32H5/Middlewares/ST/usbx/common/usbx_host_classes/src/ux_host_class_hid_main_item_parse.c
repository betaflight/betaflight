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
/*    _ux_host_class_hid_main_item_parse                  PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function parses a main item from the report descriptor.        */ 
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
/*    _ux_host_class_hid_report_add         Add report                    */ 
/*    _ux_utility_memory_set                Memory block set              */ 
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
UINT  _ux_host_class_hid_main_item_parse(UX_HOST_CLASS_HID *hid, UX_HOST_CLASS_HID_ITEM *item, UCHAR *descriptor)
{

UX_HOST_CLASS_HID_PARSER        *hid_parser;
UINT                            status =  UX_ERROR;
ULONG                           collection_type;


    /* Get the temporary parser structure pointer.  */
    hid_parser =  &hid -> ux_host_class_hid_parser;

    /* Analyze the tag.  */
    switch (item -> ux_host_class_hid_item_report_tag)
    {

    case UX_HOST_CLASS_HID_MAIN_TAG_COLLECTION:

        /* We have a new collection to open. If the collection type is application, 
           we have to differentiate the first collection.  */
        collection_type =  _ux_host_class_hid_item_data_get(descriptor, item);

        /* Check the collection type.  */
        if (collection_type == UX_HOST_CLASS_HID_COLLECTION_APPLICATION)
        {

            /* We have a collection of type application, check if this is the first one */
            if ((hid_parser -> ux_host_class_hid_parser_main_page == 0) && (hid_parser -> ux_host_class_hid_parser_main_usage == 0))
            {

                /* It is the first application. Since the main usage and page have not yet
                   been defined, we use the global page and the current local usage.  */          
                hid_parser -> ux_host_class_hid_parser_main_page =   hid_parser -> ux_host_class_hid_parser_global.ux_host_class_hid_global_item_usage_page;
                hid_parser -> ux_host_class_hid_parser_main_usage =  hid_parser -> ux_host_class_hid_parser_local.ux_host_class_hid_local_item_usages[0];
            }

            /* Memorize the application.  */
            hid_parser -> ux_host_class_hid_parser_application =  collection_type;

            /* Add one collection to this report */
            hid_parser -> ux_host_class_hid_parser_number_collection++;
            
            /* Set the status to success.  */
            status =  UX_SUCCESS;
        }
        else
        {

            if (hid_parser -> ux_host_class_hid_parser_number_collection >= UX_HOST_CLASS_HID_MAX_COLLECTION)
            {
            
                /* Error trap. */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_HID_COLLECTION_OVERFLOW);

                /* If trace is enabled, insert this event into the trace buffer.  */
                UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_HID_COLLECTION_OVERFLOW, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

                return(UX_HOST_CLASS_HID_COLLECTION_OVERFLOW);
            }
            else
            {

                hid_parser -> ux_host_class_hid_parser_collection[hid_parser -> ux_host_class_hid_parser_number_collection] =  collection_type;

                /* Add one collection to this report.  */
                hid_parser -> ux_host_class_hid_parser_number_collection++;

                /* Set the status to success.  */
                status =  UX_SUCCESS;
            }
        }
        break;      

    case UX_HOST_CLASS_HID_MAIN_TAG_END_COLLECTION:

        /* We need to pop back the last collection.  */               
        if (hid_parser -> ux_host_class_hid_parser_number_collection == 0)
        {
            
            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_HID_COLLECTION_OVERFLOW);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_HID_COLLECTION_OVERFLOW, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_HOST_CLASS_HID_COLLECTION_OVERFLOW);
        }
        
        else

            hid_parser -> ux_host_class_hid_parser_number_collection--;

        status =  UX_SUCCESS;
        break;

    case UX_HOST_CLASS_HID_MAIN_TAG_INPUT:
    case UX_HOST_CLASS_HID_MAIN_TAG_OUTPUT:
    case UX_HOST_CLASS_HID_MAIN_TAG_FEATURE:

        /* We need to add a report.  */
        status =  _ux_host_class_hid_report_add(hid, descriptor, item);
        break;
    }  
                                                 
    /* We have a new main item, so the local instances have to be cleaned.  */
    _ux_utility_memory_set(&hid_parser -> ux_host_class_hid_parser_local, 0, sizeof(UX_HOST_CLASS_HID_LOCAL_ITEM)); /* Use case of memset is verified. */

    /* Return completion status.  */
    return(status);
}

