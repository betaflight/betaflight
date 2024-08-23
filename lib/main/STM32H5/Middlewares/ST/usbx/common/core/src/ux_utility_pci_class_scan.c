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
/*    _ux_utility_pci_class_scan                          PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function scans the PCI bus from a certain position for a       */ 
/*    specific PCI class.                                                 */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pci_class                             PCI class requested           */ 
/*    bus_number                            PCI bus number                */ 
/*    device_number                         Device number                 */ 
/*    function_number                       Function number               */ 
/*    current_bus_number                    Current bus number            */ 
/*    current_device_number                 Current device number         */ 
/*    current_function_number               Current function number       */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    32-bit value                                                        */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_pci_read                  PCI read utility              */ 
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
ULONG  _ux_utility_pci_class_scan(ULONG pci_class, ULONG bus_number, ULONG device_number, 
                   ULONG function_number, ULONG *current_bus_number,
                   ULONG *current_device_number, ULONG *current_function_number)
{

ULONG   bus_number_index;
ULONG   device_number_index;
ULONG   function_number_index;
ULONG   value;
ULONG   current_pci_class;


    /* Scan all bus.  */
    for (bus_number_index = bus_number; bus_number_index <= UX_PCI_NB_BUS; bus_number_index++)
    {

        /* Scan all devices.  */
        for(device_number_index = device_number;device_number_index <= UX_PCI_NB_DEVICE; device_number_index++)
        {

            /* Scan all functions.  */
            for(function_number_index = function_number; function_number_index <= UX_PCI_NB_FUNCTIONS; function_number_index++)
            {

                /* Reset all PCI address for next loop.  */
                function_number = 0;
                device_number =   0;
                bus_number =      0;

                /* Read the PCI class bus/device/function.  */
                value =  _ux_utility_pci_read(bus_number_index, device_number_index, function_number_index,
                                    UX_PCI_CFG_REVISION,32);

                /* Isolate the class code which is in the upper 3 bytes.  */
                current_pci_class =  (value >> 8) & 0x00ffffff;

                /* Do we have a match with the demanded class?  */
                if(current_pci_class == pci_class)
                {

                    /* Return the position of this device on the PCI */
                    *current_bus_number =       bus_number_index;
                    *current_device_number =    device_number_index;
                    *current_function_number =  function_number_index;

                    /* Return success!  */
                    return(UX_SUCCESS);
                }
            }
        }
    }

    /* Return an error since we didn't find anything.  */
    return(UX_ERROR);
}

