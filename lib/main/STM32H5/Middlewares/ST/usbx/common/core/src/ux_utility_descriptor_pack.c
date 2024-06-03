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
/*    _ux_utility_descriptor_pack                         PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will pack an application structure into a USB         */
/*    descriptor.                                                         */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    descriptor                            Pointer to the unpacked       */ 
/*                                            descriptor                  */
/*    descriptor_structure                  Components of the descriptor  */
/*    descriptor_entries                    Number of entries in the      */ 
/*                                            descriptor                  */
/*    raw_descriptor                        Pointer to packed descriptor  */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_long_put                  Put 32-bit value              */
/*    _ux_utility_short_put                 Put 16-bit value              */
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
VOID  _ux_utility_descriptor_pack(UCHAR * descriptor, UCHAR * descriptor_structure,
                        UINT descriptor_entries, UCHAR * raw_descriptor)
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

            _ux_utility_long_put(raw_descriptor, *((ULONG *) descriptor));
            raw_descriptor +=  4;
            break;                   

        case 2:

            _ux_utility_short_put(raw_descriptor, (USHORT)*((ULONG *) descriptor));
            raw_descriptor += 2;
            break;                   

        default:

            *raw_descriptor =  (UCHAR) *((ULONG *) descriptor);
            raw_descriptor++;
        }

        /* Add the size of the component to the destination.  */
        descriptor +=  4;
    }

    /* Return to caller.  */
    return;
}

