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
#include "ux_utility.h"
#include "ux_pictbridge.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_pictbridge_hexa_to_major_minor                  PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function translates an hexa value into an major.minor value    */ 
/*    (xx.xx).                                                            */
/*                                                                        */ 
/*    Note: the space of the output_buffer must be equal to or larger     */
/*    than 5 to fill all converted characters.                            */
/*                                                                        */
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hexa_value                             Value to be translated       */ 
/*    output_buffer                          Where to store the result    */ 
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_pictbridge_hexa_to_major_minor(ULONG hexa_value, UCHAR *output_buffer)
{

UCHAR                   hexa_major;
UCHAR                   hexa_minor;
UINT                    status;


    /* Reset the output buffer.  */
    _ux_utility_memory_set(output_buffer, 0, 5); /* Use case of memset is verified. */
       
    /* Isolate the major.  */
    hexa_major = (UCHAR)(hexa_value >> 16);

    /* Isolate the minor.  */                                           
    hexa_minor = (UCHAR)(hexa_value & 0xFFFF);

    /* Insert the decimal value of the major.  */
    status = _ux_pictbridge_hexa_to_decimal_string(hexa_major, output_buffer, UX_PICTBRIDGE_LEADING_ZERO_ON, 2);
    if (status != UX_SUCCESS)
        return(status);

    /* Update the address of the destination.  */
    output_buffer += 2;
    
    /* Insert the '.' between major and minor.  */
    *output_buffer++ = '.';

    /* Insert the decimal value of the minor.  */
    status = _ux_pictbridge_hexa_to_decimal_string(hexa_minor, output_buffer, UX_PICTBRIDGE_LEADING_ZERO_ON, 2);
    if (status != UX_SUCCESS)
        return(status);

    /* Operation was successful.  */
    return(UX_SUCCESS);    
}

