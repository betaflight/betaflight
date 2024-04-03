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
/**   Pictbridge Application                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_pictbridge.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_pictbridge_tag_name_scan                        PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function scans the tag name from a set of allowed tags         */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    tag_item                               list of allowed tags         */ 
/*    tag_name                               the tag name to scan         */ 
/*    tag_entry                              Address of the found tag     */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _ux_pictbridge_object_parse                                         */ 
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
UINT  _ux_pictbridge_tag_name_scan(UX_PICTBRIDGE_XML_ITEM *tag_item,
                                  UCHAR *tag_name,
                                  UX_PICTBRIDGE_XML_ITEM **tag_entry)                                  
{
UINT    tag_name_length = 0;
UINT    tag_item_length = 0;
UINT    status;


    /* Calculate the length of the tag item. */
    status = _ux_utility_string_length_check(tag_name, &tag_name_length, UX_PICTBRIDGE_MAX_TAG_SIZE);
    if (status != UX_SUCCESS)
        return(status);

    /* Parse all the tags contained in the tag_item list.  */
    while(tag_item -> ux_pictbridge_xml_item_tag_name[0] != 0)
    {
        
        /* There is a valid entry in the tag list.  Calculate the length of the tag name from the list.  */
        status = _ux_utility_string_length_check(tag_item -> ux_pictbridge_xml_item_tag_name, &tag_item_length, UX_PICTBRIDGE_MAX_TAG_SIZE);
        if (status != UX_SUCCESS)
            return(status);

        /* If both length do not match, no need to check for a match.  */
        if (tag_item_length == tag_name_length)
        {

            /* Both length match, we may have a tag match. Check the names */
            if (_ux_utility_memory_compare(tag_name, tag_item -> ux_pictbridge_xml_item_tag_name, 
                                        tag_item_length) == UX_SUCCESS)
            {
            
                /* We have found the tag. Save the entry in the caller tag_entry field.  */
                *tag_entry = tag_item;
                
                /* We are done here.  */
                return(UX_SUCCESS);
               
            }                                        
        }

        /* The tags length or the tag names did not match.  Proceed to the next tag.  */
        tag_item++;        
    }       

    /* We get here when we reached the end of the tag list and no match.  */
    return(UX_PICTBRIDGE_ERROR_PARAMETER_UNKNOWN);    
}

