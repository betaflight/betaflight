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
/*    _ux_utility_pci_read                                PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function reads a 32/16/8 bit value from a specific PCI bus     */ 
/*    at a certain offset.                                                */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    bus_number                            PCI bus number                */ 
/*    device_number                         Device number                 */ 
/*    function_number                       Function number               */ 
/*    offset                                Offset                        */ 
/*    read_size                             Size of read                  */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    32-bit value                                                        */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    inpl                                  PCI input long                */ 
/*    inpw                                  PCI input word                */ 
/*    inpb                                  PCI input byte                */ 
/*    outpl                                 PCI output function           */
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
ULONG  _ux_utility_pci_read(ULONG bus_number, ULONG device_number, ULONG function_number,
                                        ULONG offset, UINT read_size)
{

ULONG   destination_address;
ULONG   cfg_ctrl;

    
    /* Calculate the destination address.  */
    destination_address =  (((bus_number << 16) & 0x00ff0000) | ((device_number << 11) & 0x0000f800) |
                                ((function_number << 8) & 0x00000700));

    /* Calculate the configure control value.  */
    cfg_ctrl = destination_address | offset | 0x80000000;

    /* Read based on the size requested.  */
    switch(read_size)
    {

    case 32:

        /* Write the address we need to read from.  */
        outpl(UX_PCI_CFG_CTRL_ADDRESS, cfg_ctrl);

        /* Return the 32 bit content of this address.  */
        return(inpl(UX_PCI_CFG_DATA_ADDRESS));
    
    case 16:

        /* Write the address we need to read from.  */
        outpl(UX_PCI_CFG_CTRL_ADDRESS, cfg_ctrl);

        /* Return the 16 bit content of this address.  */
        return((USHORT)(inpw(UX_PCI_CFG_DATA_ADDRESS)));

    case 8:

        /* Write the address we need to read from.  */
        outpl(UX_PCI_CFG_CTRL_ADDRESS, cfg_ctrl);

        /* Return the 8 bit content of this address */
        return((ULONG)(inpb(UX_PCI_CFG_DATA_ADDRESS)));

    default:

        return(0);
    }
}
