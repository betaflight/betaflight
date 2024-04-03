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
/*    _ux_utility_descriptor_parse                        PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will unpack a USB descriptor from the bus into a      */
/*    memory aligned structure.                                           */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    raw_descriptor                        Pointer to packed descriptor  */
/*    descriptor_structure                  Components of the descriptor  */
/*    descriptor_entries                    Number of entries in the      */ 
/*                                            descriptor                  */
/*    descriptor                            Pointer to the unpacked       */ 
/*                                            descriptor                  */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_long_get                  Get 32-bit value              */
/*    _ux_utility_short_get                 Get 16-bit value              */
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
VOID  _ux_utility_descriptor_parse(UCHAR * raw_descriptor, UCHAR * descriptor_structure,
                        UINT descriptor_entries, UCHAR * descriptor)
{

    /* Loop on all the entries in this descriptor.  */
    while(descriptor_entries--)
    {

        /* Get the length of that component.  */
        switch(*descriptor_structure++)
        {

        /* Check the size then build the component from the source and
           insert it into the target descriptor.  */
        case 4:

            *((ULONG *) descriptor) =  _ux_utility_long_get(raw_descriptor);
            raw_descriptor +=  4;
            break;                   

        case 2:

            *((ULONG *) descriptor) = (ULONG) _ux_utility_short_get(raw_descriptor);
            raw_descriptor += 2;
            break;                   

        default:

            *((ULONG *) descriptor) =  (ULONG) *raw_descriptor;
            raw_descriptor++;
        }

        /* Add the size of the component to the destination.  */
        descriptor +=  4;
    }

    /* Return to caller.  */
    return;
}

