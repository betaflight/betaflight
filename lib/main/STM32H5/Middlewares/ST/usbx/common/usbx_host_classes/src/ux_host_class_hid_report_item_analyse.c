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
/*    _ux_host_class_hid_report_item_analyse              PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function gets the report descriptor and analyzes it.           */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    descriptor                            Pointer to descriptor         */ 
/*    item                                  Pointer to item               */ 
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
UINT  _ux_host_class_hid_report_item_analyse(UCHAR *descriptor, UX_HOST_CLASS_HID_ITEM *item)
{

UCHAR       item_byte;
    

    /* Get the first byte from the descriptor.  */
    item_byte =  *descriptor;
    
    /*  We need to determine if this is a short or long item.
        For long items, the tag is always 1111.  */   
    if ((item_byte & UX_HOST_CLASS_HID_ITEM_TAG_MASK) == UX_HOST_CLASS_HID_ITEM_TAG_LONG)
    {

        /* We have a long item, mark its format.  */
        item -> ux_host_class_hid_item_report_format =  UX_HOST_CLASS_HID_ITEM_TAG_LONG;

        /* Set the type.  */
        item -> ux_host_class_hid_item_report_type =  (item_byte >> 2) & 3;

        /* Get its length (byte 1).  */
        item -> ux_host_class_hid_item_report_length =  (USHORT) *(descriptor + 1);

        /* Then the tag (byte 2).  */
        item -> ux_host_class_hid_item_report_tag =  *(descriptor + 2);
    }
    else
    {

        /* We have a short item. Mark its format */
        item -> ux_host_class_hid_item_report_format =  UX_HOST_CLASS_HID_ITEM_TAG_SHORT;
        
        /* Get the length of the item.  */
        switch (item_byte & UX_HOST_CLASS_HID_ITEM_LENGTH_MASK)
        {

        case 3:

            item -> ux_host_class_hid_item_report_length =  4;
            break;

        default:

            item -> ux_host_class_hid_item_report_length =  item_byte & UX_HOST_CLASS_HID_ITEM_LENGTH_MASK;
            break;
        }

        /* Set the type.  */
        item -> ux_host_class_hid_item_report_type =  (item_byte >> 2) & 3;

        /* Set the tag.  */
        item -> ux_host_class_hid_item_report_tag =  item_byte >> 4;
    }

    /* Return successful completion.  */
    return(UX_SUCCESS);
}

