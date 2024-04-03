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
/*    _ux_pictbridge_element_to_hexa                      PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function translates an element into an hexa value.             */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    element                                Where to store the element   */ 
/*    hexa_value                             Value to be returned         */ 
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
UINT  _ux_pictbridge_element_to_hexa(UCHAR *element, ULONG *hexa_value)
{

ULONG                   element_length;
ULONG                   local_hexa_value;
UCHAR                   element_content;
UCHAR                   element_hexa;
UINT                    string_length = 0;
UINT                    status;


    /* Get the string length, max 0xFFFFFFFF, 8 digits.  */
    status = _ux_utility_string_length_check(element, &string_length, 8);
    if (status != UX_SUCCESS)
        return(status);

    /* Get the element length. Should not be more than 8 characters.  */
    element_length = string_length;
       
    /* Check the length. Error if 0.  */
    if (element_length == 0)
        
        /* We have a syntax violation.  */
        return(UX_ERROR);                     

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
            /* We have a syntax violation.  */
            return(UX_ERROR);                     

        /* Add the found value to the current cumulated hexa value.  */
        local_hexa_value |= element_hexa;

        /* Next position.  */
        element++;
    
        /* Update length.  */
        element_length--;
        
    }       

    /* We have finished building the 32 bit hexa value.  */
    *hexa_value = local_hexa_value;    

    /* Operation was successful.  */
    return(UX_SUCCESS);    
}

