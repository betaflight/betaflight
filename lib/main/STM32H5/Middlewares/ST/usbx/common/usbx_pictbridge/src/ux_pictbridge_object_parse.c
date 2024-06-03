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
/*    _ux_pictbridge_object_parse                         PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function parses a XML based pictbridge object.                 */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pictbridge                             Pictbridge instance          */ 
/*    xml_object_buffer                      Pointer to object buffer     */ 
/*    xml_object_length                      Length of the object         */ 
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
/*    _ux_pictbridge_object_get                                           */ 
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
UINT  _ux_pictbridge_object_parse(UX_PICTBRIDGE *pictbridge, UCHAR *xml_object_buffer,
                                    ULONG xml_object_length)
{
UX_PICTBRIDGE_XML_ITEM                  *tag_item;
UX_PICTBRIDGE_XML_ITEM                  *tag_entry =  UX_NULL;
UCHAR                                   tag_name[UX_PICTBRIDGE_MAX_TAG_SIZE];
UCHAR                                   variable_name[UX_PICTBRIDGE_MAX_VARIABLE_SIZE];
UCHAR                                   variable_string[UX_PICTBRIDGE_MAX_STRING_SIZE];
UCHAR                                   xml_parameter[UX_PICTBRIDGE_MAX_STRING_SIZE];
UX_PICTBRIDGE_XML_ITEM                  *tag_history[UX_PICTBRIDGE_MAX_TAG_DEPTH];
ULONG                                   tag_history_index;
ULONG                                   tag_flag;
ULONG                                   closing_tag_count;
UINT                                    tag_name_length;
UINT                                    tag_history_length;
UINT                                    status;

    /* Set the tag position at root.  */
    tag_item = _ux_pictbridge_xml_item_root;
    
    /* Clear the input tags.  */
    pictbridge -> ux_pictbridge_input_tags =  0;

    /* Clear the input request.  */
    pictbridge -> ux_pictbridge_input_request =  0;
    
    /* Tag history index is at root.  */
    tag_history_index =  0;
    
    /* Closing tag count is reset.  */
    closing_tag_count = 0;

    /* This variable remains untouched if the XML object's length is 0.  */
    status =  UX_SUCCESS;

    /* We parse the object until all items are parsed or there is a syntax error
       or a short object.  */
    while(xml_object_length)
    {

        /* Scan the object buffer for a tag.  */
        status = _ux_pictbridge_tag_name_get(xml_object_buffer, xml_object_length, tag_name, 
                                        variable_name, variable_string, xml_parameter,       
                                        &xml_object_buffer, &xml_object_length, &tag_flag);
        
        /* We may have an error.  Check if this is an empty line in which case we are done. */
        if (status != UX_SUCCESS)
        {
        
            /* Check for empty line.  */
            if (status == UX_PICTBRIDGE_ERROR_EMPTY_LINE)
            
                /* Yes, we have an empty line. Do not proceed but we have a successful completion. */
                status = UX_SUCCESS;
                
            /* Do not proceed.  */
            break;
        }

        /* Check if this is a closing tag ? */
        if (tag_flag & UX_PICTBRIDGE_TAG_FLAG_CLOSING)
        {
            
            /* Assume the worst.  */
            status = UX_PICTBRIDGE_ERROR_SCRIPT_SYNTAX_ERROR;

            /* Ensure we do not have a closing tag without a prior normal tag entry.  */
            if (tag_history_index == 0)

                /* Syntax error.  */
                break;

            /* One step back in the tag history.  */
            tag_history_index--;

            /* Calculate the length of the tag item.  */
            status = _ux_utility_string_length_check(tag_name, &tag_name_length, UX_PICTBRIDGE_MAX_TAG_SIZE);
            if (status != UX_SUCCESS)

                /* Bad name, we should never here if _tag_name_get is OK.  */
                break;

            /* Calculate the length of the current tag name in the tag history.  */
            status = _ux_utility_string_length_check(tag_history[tag_history_index] -> ux_pictbridge_xml_item_tag_name, &tag_history_length, UX_PICTBRIDGE_MAX_TAG_SIZE);
            if (status != UX_SUCCESS)

                /* Bad name, we should never here if _tag_name_get is OK.  */
                return(status);

            /* If both length do not match, no need to check for a match.  */
            if (tag_history_length == tag_name_length)
            {

                /* Both length match, we may have a tag match. Check the names */
                if (_ux_utility_memory_compare(tag_name, tag_history[tag_history_index] -> ux_pictbridge_xml_item_tag_name,
                                        tag_name_length) != UX_SUCCESS)

                    /* Syntax error.  */
                    break;
            }
            else

                /* Syntax error.  */
                break;            
            
            /* Increment the closing tag count. */
            closing_tag_count++;
            
            /* If we have more than one closing tag consecutively, look for the parent.  */
            if (closing_tag_count > 1)
            {
            
                /* Get the parent for this item and set it as the current tag_item.  We may already be at the root.  */
                if (tag_entry -> ux_pictbridge_xml_item_parent != UX_NULL)
            
                    /* We have a parent, set it.  */
                    tag_item =  tag_entry -> ux_pictbridge_xml_item_parent;
            }                    

            /* We set status to success.  */
            status = UX_SUCCESS;

        }
        else
        {
            
            /* The tag name is in the tag_name variable but has not been verified yet.  */
            status = _ux_pictbridge_tag_name_scan(tag_item, tag_name, &tag_entry);
        
            /* We may have an error.  */
            if (status != UX_SUCCESS)

                /* Do not proceed.  */
                break;

            /* Check if this is a comment tag ? */
            if ((tag_flag & UX_PICTBRIDGE_TAG_FLAG_COMMENT) == 0)
            {
        
                /* Reset the closing tag count.  */
                closing_tag_count =  0;

                /* Save the current tag in the tag history.  */
                tag_history[tag_history_index] = tag_entry;
                
                /* Increase the current tag index.  */
                tag_history_index++;

                /* Check if there is a main tag specified in the tag entry. */
                if (tag_entry -> ux_pictbridge_xml_item_tag_code != 0)
                {

                    /* If this is a leaf, the tag is not a main request but a variable to be returned.  */
                    if (tag_entry -> ux_pictbridge_xml_item_child == UX_PICTBRIDGE_XML_LEAF)

                        /* This is a variable. Add it to the other ones.  */
                        pictbridge -> ux_pictbridge_input_tags |=  tag_entry -> ux_pictbridge_xml_item_tag_code;
                        
                    else

                        /* Yes we have a tag code, memorize it.  This is a main input request. */
                        pictbridge -> ux_pictbridge_input_request =  tag_entry -> ux_pictbridge_xml_item_tag_code;
                }
                /* We may have a function associated with this tag. Do a sanity check.  */
                if (tag_entry -> ux_pictbridge_xml_item_function != UX_NULL)
                {
                    /* There is a function associated, so call it.  */
                    status = tag_entry -> ux_pictbridge_xml_item_function(pictbridge, variable_name, variable_string, xml_parameter);
                
                    /* We may have an error.  */
                    if (status != UX_SUCCESS)
            
                       /* Do not proceed.  */
                        break;
                }        

                /* Check if the tag has a child attached to it. 
                   If there is neither a child or a function, it means we do not treat this tag name. */
                if (tag_entry -> ux_pictbridge_xml_item_child != UX_NULL && tag_entry -> ux_pictbridge_xml_item_child != UX_PICTBRIDGE_XML_LEAF)
                {            
    
                    /* We have a child, set it.  */
                    tag_item =  tag_entry -> ux_pictbridge_xml_item_child;
                    
                    /* Reset the tag codes.  */
                    pictbridge -> ux_pictbridge_input_tags =  0;
                }                

                /* Check if this is a self closing tag ? */
                if (tag_flag & UX_PICTBRIDGE_TAG_FLAG_SELF_CLOSING)
                {
    
                    /* Assume the worst.  */
                    status = UX_PICTBRIDGE_ERROR_SCRIPT_SYNTAX_ERROR;
    
                    /* Ensure we do not have a closing tag without a prior normal tag entry.  */
                    if (tag_history_index == 0)
    
                        /* Syntax error.  */
                        break;
    
                    /* One step back in the tag history.  */
                    tag_history_index--;

                    /* Calculate the length of the tag item.   */
                    status = _ux_utility_string_length_check(tag_name, &tag_name_length, UX_PICTBRIDGE_MAX_TAG_SIZE);
                    if (status != UX_SUCCESS)

                        /* Bad name, we should never here if _tag_name_scan is OK!  */
                        break;

                    /* Calculate the length of the current tag name in the tag history.  */
                    status = _ux_utility_string_length_check(tag_history[tag_history_index] -> ux_pictbridge_xml_item_tag_name, &tag_history_length, UX_PICTBRIDGE_MAX_TAG_SIZE);
                    if (status != UX_SUCCESS)

                        /* Bad name, we should never here if _tag_name_scan is OK!  */
                        break;

                    /* If both length do not match, no need to check for a match.  */
                    if (tag_history_length == tag_name_length)
                    {
    
                        /* Both length match, we may have a tag match. Check the names */
                        if (_ux_utility_memory_compare(tag_name, tag_history[tag_history_index] -> ux_pictbridge_xml_item_tag_name,
                                            tag_name_length) != UX_SUCCESS)
    
                            /* Syntax error.  */
                            break;
                    }
                    else
    
                        /* Syntax error.  */
                        break;            
                
                    /* Increment the closing tag count. */
                    closing_tag_count++;

                    /* We set status to success.  */
                    status = UX_SUCCESS;

                }            
            }
        }
    }       

    /* Return completion status.  */
    return(status);    
}

