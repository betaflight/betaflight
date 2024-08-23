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
/**   Utility                                                             */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_utility_string_to_unicode                       PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function converts a ascii string to a unicode string.          */ 
/*                                                                        */
/*    Note:                                                               */
/*    The unicode string length (including NULL-terminator) is limited by */
/*    length in a byte, so max ascii string length must be no more than   */
/*    254 (NULL-terminator excluded). Only first 254 characters           */
/*    are converted if the string is too long.                            */
/*    The buffer of destination must have enough space for result, at     */
/*    least 1 + (strlen(source) + 1) * 2 bytes.                           */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    source                                Ascii String                  */ 
/*    destination                           Unicode String                */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    none                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_string_length_check       Check and return C string     */
/*                                          length                        */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USBX Components                                                     */ 
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
VOID  _ux_utility_string_to_unicode(UCHAR *source, UCHAR *destination)
{

UINT   string_length;

    /* Get the ascii string length, when there is error length is not modified so max length is used.  */
    string_length = 254;
    _ux_utility_string_length_check(source, &string_length, 254);

    /* Set the length of the string as the first byte of the unicode string.  
       The length is casted as a byte since Unicode strings cannot be more than 255 chars.  */
    *destination++ = (UCHAR)(string_length + 1);

    while(string_length--)
    {
        /* First character is from the source.  */
        *destination++ = *source++;

        /* Second character of unicode word is 0.  */
        *destination++ = 0;
    }    

    /* Finish with a 0.  */
    *destination++ = 0;

    /* Finish with a 0.  */
    *destination++ = 0;

    /* We are done.  */
    return;
}

