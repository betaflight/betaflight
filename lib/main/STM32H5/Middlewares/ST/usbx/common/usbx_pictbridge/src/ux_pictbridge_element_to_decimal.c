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
/*    _ux_pictbridge_element_to_decimal                   PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function translates an element into a decimal value.           */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    element                                Where to store the element   */ 
/*    decimal_value                          Value to be returned         */ 
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
UINT  _ux_pictbridge_element_to_decimal(UCHAR *element, ULONG *decimal_value)
{

ULONG                   element_length;
ULONG                   local_decimal_value;
UCHAR                   element_content;
UCHAR                   element_decimal;
ULONG                   multiplier;
ULONG                   multiplier_length;
UINT                    string_length = 0;
UINT                    status;


    /* Get the string length, max 4294967295 (0xFFFFFFFF), 10 digits.  */
    status = _ux_utility_string_length_check(element, &string_length, 10);
    if (status != UX_SUCCESS)
        return(status);

    /* Get the element length. Should not be more than 8 characters.  */
    element_length = string_length;
       
    /* Check the length. Error if 0.  */
    if (element_length == 0)
        
        /* We have a syntax violation.  */
        return(UX_ERROR);                     

    /* Reset the local decimal value.  */
    local_decimal_value =  0;
    
    /* Calculate the multiplier value.  First reset it. */
    multiplier = 1;
    
    /* Set the length of the string.  */
    multiplier_length = element_length;
    
    /* Build the multiplier : 10 square length.  */
    while (multiplier_length-- > 1)
    
        /* Add another decimal.  */
        multiplier = multiplier * 10;
    
    /* We parse the element and build the decimal value one byte at a type.  */
    while(element_length)
    {
    
        /* Get the element content.  */
        element_content = *element;
        
        /* Check for the element content. Should be >0 <9.  */
        if (element_content >= '0' && element_content <= '9')
            
            /* We have a digit.  */
            element_decimal = (UCHAR)(element_content - '0');

        else            
            /* We have a syntax violation.  */
            return(UX_ERROR);                     

        /* Add the found value to the current cumulated decimal value.  */
        local_decimal_value += (ULONG) element_decimal * multiplier;

        /* Next position.  */
        element++;
    
        /* Update length.  */
        element_length--;

        /* Reduce the multiplier by one decimal. */
        multiplier = multiplier / 10;
        
    }       

    /* We have finished building the 32 bit decimal value.  */
    *decimal_value = local_decimal_value;    

    /* Operation was successful.  */
    return(UX_SUCCESS);    
}

