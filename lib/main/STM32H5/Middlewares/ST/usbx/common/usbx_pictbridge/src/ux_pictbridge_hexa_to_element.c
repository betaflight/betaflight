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
/*    _ux_pictbridge_hexa_to_element                      PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function translates an hexa value into an element.             */ 
/*                                                                        */ 
/*    Note: the size of the element buffer must be equal to or larger than*/
/*    8 to fill all converted characters.                                 */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hexa_value                             Value to be translated       */ 
/*    element                                Where to store the element   */ 
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
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_pictbridge_hexa_to_element(ULONG hexa_value, UCHAR *element)
{

ULONG                   element_length;
ULONG                   local_hexa_value;
UCHAR                   element_hexa;
ULONG                   element_shift;
    
    /* Reset the element buffer.  */
    _ux_utility_memory_set(element, 0, 8); /* Use case of memset is verified. */
       
    /* Reset the value of the length.*/
    element_length = 0;

    /* Init the shift value.  */
    element_shift = 32 - 4;

    /* We parse the hexa value element and build the hexa value one byte at a type.  */
    while(element_length < 8)
    {
    
        /* Shift the 4 bit value we are interested in.  We keep the lowest nibble.  */
        local_hexa_value = (hexa_value >> element_shift) & 0x0f;
    
        /* See if this value is from 0-9 or A to F.  */
        if (local_hexa_value <= 9)
            
            /* We have a digit.  */
            element_hexa = (UCHAR)(local_hexa_value + '0');
        
        else

            /* We have  'A' to 'F' value.  */
            element_hexa = (UCHAR)(local_hexa_value - 10 + 'A');
        
        /* Store the converted hexa value.  */
        *element = element_hexa;

        /* Next position.  */
        element++;
    
        /* Update length.  */
        element_length++;

        /* Continue shifting by one nibble.  */
        if (element_shift >= 4)
        {
            element_shift = element_shift - 4;
        }
        else
        {
            break;
        }
    }       

    /* Operation was successful.  */
    return(UX_SUCCESS);    
}

