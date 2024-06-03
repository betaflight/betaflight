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
/*    _ux_pictbridge_hexa_to_decimal_string               PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function translates an hexa value into a decimal string.       */ 
/*                                                                        */
/*    Note:                                                               */
/*    Number of max_digit_string_size must be large enough to fit digits  */
/*    converted.                                                          */
/*    The space of the decimal_string buffer must be equal to or larger   */
/*    than max_digit_string_size to accept converted characters.          */
/*    When leading zeros are off, the converted string ends before        */
/*    reaching buffer end and zeros are padded.                           */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hexa_value                             Value to be translated       */ 
/*    decimal_string                         Where to store the element   */ 
/*    leading_zero_flag                      If leading zeroes are OK     */
/*    max_digit_string_size                  Max number of digits (<=8)   */
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
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*  04-25-2022     Yajun Xia                Modified comment(s),          */
/*                                            fixed maximum decimal       */
/*                                            calculation issue,          */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_pictbridge_hexa_to_decimal_string(ULONG hexa_value, UCHAR *decimal_string, 
                                            ULONG leading_zero_flag, ULONG max_digit_string_size)
{

ULONG                   decimal_string_shift;
ULONG                   decimal_value;
ULONG                   leading_flag;
ULONG                   decimal_max[8] = {9, 99, 999, 9999, 99999, 999999, 9999999, 99999999};

    /* Set the value of the shift decimal.*/
    switch(max_digit_string_size)
    {
        case 1 :
            decimal_string_shift = 1;
            break;
        case 2 :
            decimal_string_shift = 10;
            break;
        case 3 :
            decimal_string_shift = 100;
            break;
        case 4 :
            decimal_string_shift = 1000;
            break;
        case 5 :
            decimal_string_shift = 10000;
            break;
        case 6 :
            decimal_string_shift = 100000;
            break;
        case 7 :
            decimal_string_shift = 1000000;
            break;
        case 8 :
            decimal_string_shift = 10000000;
            break;
        default :
            return(UX_ERROR);
    }

    /* If value exceeds, do not proceed.  */
    if (hexa_value > decimal_max[max_digit_string_size - 1])
        return(UX_ERROR);

    /* Set the leading zero flag.  */
    if (leading_zero_flag == UX_PICTBRIDGE_LEADING_ZERO_ON)
        leading_flag = UX_FALSE;
    else        
        leading_flag = UX_TRUE;
    
    /* Reset the decimal_string buffer.  */
    _ux_utility_memory_set(decimal_string, 0, max_digit_string_size); /* Use case of memset is verified. */

    /* We parse the hexa value and build the decimal string one byte at a type.  */
    while(decimal_string_shift)
    {
        /* Divide the hexa value by the shift and decode the leading digital.  */
        decimal_value = hexa_value / decimal_string_shift;

        /* If the result is non zero, we can insert that decimal in the string.  
           There is a special case if the value is the last decimal or if the leading flag is off. */
        if (decimal_value != 0 || decimal_string_shift == 1 || leading_flag == UX_FALSE)
        {
            /* Insert the value.  */
            *decimal_string++ = (UCHAR)(decimal_value + '0');
            
            /* Reset the leading flag.  */
            leading_flag = UX_FALSE;
        }

        /* Reduce the shift value to the next decimal.  */
        decimal_string_shift = decimal_string_shift / 10;  
        
    }       

    /* Operation was successful.  */
    return(UX_SUCCESS);    
}

