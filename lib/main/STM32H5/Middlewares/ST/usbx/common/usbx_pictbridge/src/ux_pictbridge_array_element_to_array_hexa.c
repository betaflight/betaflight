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
/*    _ux_pictbridge_array_element_to_array_hexa          PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function translates an array of elements into an hexa array    */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    element                                Address of the element       */ 
/*    hexa_array                             address of array             */ 
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
/*  04-25-2022     Yajun Xia                Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_pictbridge_array_element_to_array_hexa(UCHAR *element, ULONG *hexa_array)
{

ULONG                   element_array_length;
ULONG                   local_hexa_value;
UCHAR                   element_content;
UCHAR                   element_hexa;
UCHAR                   *element_position;
ULONG                   remaining_length;
ULONG                   element_length;
ULONG                   saved_element_length = 0;
UINT                    string_length = UX_PICTBRIDGE_MAX_ELEMENT_SIZE;
UINT                    status;


    /* Get the string length.  */
    status = _ux_utility_string_length_check(element, &string_length, UX_PICTBRIDGE_MAX_ELEMENT_SIZE);
    if (status != UX_SUCCESS)
        return(status);

    /* Get the element length.  */
    element_array_length = string_length + 1;
       
    /* Parse the entire element array.  */
    while(element_array_length != 0)
    {
    
        /* Search for the space if delimiter or the terminating 0.  */
        element_position = element;
        
        /* The length must be saved. */
        remaining_length = element_array_length;
        
        /* Parse this element.  */
        while(remaining_length !=0)
        {

            /* Check for delimiter.  */
            if (*element_position == UX_PICTBRIDGE_TAG_CHAR_SPACE || *element_position == 0)
            {

                /* We found an element.  Mark its end.  */
                *element_position = 0;
                
                /* Get the string length, max length 8 for 32-bit hex value.  */
                status = _ux_utility_string_length_check(element, &string_length, 8);
                if (status != UX_SUCCESS)
                    return(status);

                /* Get the element length.  */
                element_length = string_length;
                
                /* Save it.  */
                saved_element_length =  element_length + 1;
                
                /* Reset the local hexa value.  */
                local_hexa_value =  0;
                
                /* We parse the element and build the hexa value one byte at a type.  */
                while(element_length)
                {
                
                    /* Shift the previous content by 1 nibble.  */
                    local_hexa_value = (local_hexa_value << 4) & 0xFFFFFFFFu;
                
                    /* Get the element content.  */
                    element_content = *element;
                    
                    /* Check for the element content. Should be >0 <9 or 'A' to 'F'.  */
                    if ((element_content >= '0' && element_content <= '9') || 
                         (element_content >= 'a' && element_content <= 'f') ||
                         (element_content >= 'A' && element_content <= 'F'))
                    {
                        
                        /* We have a valid element content.  Turn it into a hexa decimal value.  */
                        if (element_content >= '0' && element_content <= '9')
                            
                            /* We have a digit.  */
                            element_hexa = (UCHAR)(element_content - '0');
                        
                        else
                        {                
                            /* We have a 'A' to 'F' or 'a' to 'f' value.  */
                            if (element_content >= 'a' && element_content <= 'f') 
            
                                /* We have a 'a' to 'f' char.  */
                                element_hexa = (UCHAR)(element_content - 'a' + 10);
                            
                            else                        
                            
                                /* We have a 'A' to 'F' char.  */
                                element_hexa = (UCHAR)(element_content - 'A' + 10);
                                                
                        }                
                    }
                    else            
                    {
                    
                        /* Reset the last position in the array.  */
                        *hexa_array = 0;

                        /* We have a syntax violation.  */
                        return(UX_ERROR);                     
                    }                        
            
                    /* Add the found value to the current cumulated hexa value.  */
                    local_hexa_value |= element_hexa;

                    /* Next position.  */
                    element++;
                
                    /* Update length.  */
                    element_length--;
                    
                }       
            
                /* We have finished building the 32 bit hexa value.  Save it in the array.  */
                *hexa_array = local_hexa_value;    
                
                /* Next element in the array.  */
                hexa_array++;
                
                /* We are done with this element.  */
                break;
                
            }

            else

                /* Next element position.  */
                element_position++;            

            /* Adjust the remaining length.  */
            remaining_length--;
        }            
        
        /* Increment the element position to the next one if there.  */
        element++;
        
        /* Adjust the length of the array. */
        element_array_length -= saved_element_length;        
        
    }
    
    /* Reset the last position in the array.  */
    *hexa_array = 0;
               
    /* Operation was successful.  */
    return(UX_SUCCESS);    
}

