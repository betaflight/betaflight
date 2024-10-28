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
/*    _ux_utility_pci_write                               PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function writes a 32/16/8 bit value to a specific PCI bus      */ 
/*    at a certain offset.                                                */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    bus_number                            PCI bus number                */ 
/*    device_number                         Device number                 */ 
/*    function_number                       Function number               */ 
/*    offset                                Offset                        */ 
/*    value                                 Value to write                */ 
/*    write_size                            Size of write                 */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    outpl                                 PCI output long function      */
/*    outpw                                 PCI output word function      */ 
/*    outpb                                 PCI output byte function      */ 
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
VOID  _ux_utility_pci_write(ULONG bus_number, ULONG device_number, ULONG function_number,
                                ULONG offset, ULONG value, UINT write_size)
{

ULONG   destination_address;
ULONG   cfg_ctrl;


    /* Calculate the destination address.  */
    destination_address =  (((bus_number << 16) & 0x00ff0000) | ((device_number << 11) & 0x0000f800) |
                                    ((function_number << 8) & 0x00000700));

    /* Calculate the configure control value.  */
    cfg_ctrl = destination_address | offset | 0x80000000;

    /* Process relative to write size.  */
    switch(write_size)
    {

    case 32:

        /* Write the address we need to write to.  */
        outpl(UX_PCI_CFG_CTRL_ADDRESS, cfg_ctrl);

        /* Write the 32 bit content of this address.  */
        outpl(UX_PCI_CFG_DATA_ADDRESS, value);
        break;
        
    case 16:
            
        /* Write the address we need to write to.  */
        outpl(UX_PCI_CFG_CTRL_ADDRESS, cfg_ctrl);

        /* Write the 16 bit content of this address.  */
        outpw(UX_PCI_CFG_DATA_ADDRESS + (offset & 2), (USHORT) value);
        break;
    
    case 8:

        /* Write the address we need to write to.  */
        outpl(UX_PCI_CFG_CTRL_ADDRESS, cfg_ctrl);

        /* Write the 8 bit content of this address */
        outpb(UX_PCI_CFG_DATA_ADDRESS + (offset & 3), (UCHAR) value);
        break;

    default:
            
        break;
    }
}

