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
/**   Host Stack                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_stack.h"


#if UX_MAX_DEVICES > 1
/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_stack_bandwidth_claim                      PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will reserve bandwidth for a periodic endpoint. The   */ 
/*    bandwidth requirement is calculated by the MaxPacketSize field of   */
/*    endpoint and the speed of the endpoint. If the device is on a 1.1   */ 
/*    bus or it is a 1.1 device behind a 2.0 hub on a 2.0 bus, the device */
/*    bandwidth must be multiplied by 8 on the 1.1 segment.               */
/*                                                                        */
/*    This algorithm takes into account both TT bandwidth and HCD         */ 
/*    bandwidth. The TTs are attached to the device structure and not     */ 
/*    the hub structure in order to make the stack agnostic of the hub    */ 
/*    class.                                                              */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    HCD                                   Pointer to HCD                */ 
/*    endpoint                              Pointer to endpoint           */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
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
/*                                            optimized based on compile  */
/*                                            definitions,                */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_stack_bandwidth_claim(UX_HCD *hcd, UX_ENDPOINT *endpoint)
{

UX_DEVICE       *device;
UX_DEVICE       *parent_device;
USHORT          hcd_bandwidth_claimed;
USHORT          max_packet_size;
LONG            packet_size;
USHORT          tt_bandwidth_claimed =  0;
ULONG           port_index;
ULONG           port_map;
ULONG           tt_index;
const UCHAR     overheads[4][3] = {
/*   LS  FS   HS   */
    {63, 45, 173}, /* Control */
    { 0,  9,  38}, /* Isochronous */
    { 0, 13,  55}, /* Bulk */
    {19, 13,  55}  /* Interrupt */
};

    /* Get the pointer to the device.  */
    device =  endpoint -> ux_endpoint_device;

    /* Calculate the bandwidth. From USB spec.
     *
     * The frame unit consumed per byte is like follow:
     *              Bytes/FrameUnit     FrameUnit/byte  FrameUnit/byte
     *              (Overhead included) (HS baseline)   (FS baseline)
     * Low Speed       187.5                40             8
     * Full Speed     1500                   5             1
     * High Speed     7500                   1            1/5
     * 
     * The overhead is like follow:
     *               Control Isochronous Bulk Interrupt
     * bmAttribute     (0)       (1)     (2)     (3)
     * Low Speed        63       --      --      19
     * Full Speed       45        9      13      13
     * High Speed      173       38      55      55
     * 
     * Worst case bit stuffing is calculated as 1.1667 (7/6) times the raw time.
     */

    /* Get maximum packet size.  */
    max_packet_size  = endpoint -> ux_endpoint_descriptor.wMaxPacketSize & UX_MAX_PACKET_SIZE_MASK;

    /* Rough time for possible Bit Stuffing.  */
    packet_size = (max_packet_size * 7 + 5) / 6;

    /* Add overhead.  */
    packet_size += overheads[endpoint -> ux_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE][device -> ux_device_speed];
    max_packet_size = (USHORT)packet_size;

    /* Check for high-speed endpoint.  */
    if (device -> ux_device_speed == UX_HIGH_SPEED_DEVICE)
    {

        /* Get number of transactions.  */
        max_packet_size = (USHORT)(max_packet_size *
                    (((endpoint -> ux_endpoint_descriptor.wMaxPacketSize & UX_MAX_NUMBER_OF_TRANSACTIONS_MASK) >>
                        UX_MAX_NUMBER_OF_TRANSACTIONS_SHIFT) + 1));
    }

    /* Calculate the bandwidth claimed by this endpoint for the main bus.  */
    if (hcd -> ux_hcd_version != 0x200)
    {

        if (device -> ux_device_speed == UX_LOW_SPEED_DEVICE)
            /* Low speed transfer takes 40x more units than high speed. */
            hcd_bandwidth_claimed =  (USHORT)(max_packet_size * 8 * 5);
        else
        {

            if (device -> ux_device_speed == UX_FULL_SPEED_DEVICE)
                /* Full speed transfer takes 5x more units than high speed. */
                hcd_bandwidth_claimed =  (USHORT)(max_packet_size * 5);
            else
                /* Use high speed timing as base for bus bandwidth calculation. */
                hcd_bandwidth_claimed =  (USHORT)max_packet_size;
        }
    }
    else        
    {

        hcd_bandwidth_claimed =  (USHORT)max_packet_size;
        if (device -> ux_device_speed == UX_LOW_SPEED_DEVICE)
            /* Low speed transfer takes 8x more units than full speed. */
            tt_bandwidth_claimed =  (USHORT)(max_packet_size * 8);
        else
            /* Use full speed timing as base for TT bandwidth calculation. */
            tt_bandwidth_claimed =  (USHORT)max_packet_size;
    }

    /* Allocate the HCD bandwidth, since it's already checked by _bandwidth_check.  */
    hcd -> ux_hcd_available_bandwidth -=  hcd_bandwidth_claimed;

    /* We need to take care of the case where the endpoint belongs to a USB 1.1
       device that sits behind a 2.0 hub. We ignore cases where the device
       is either high speed or the bus is 1.1.  */
    if ((device -> ux_device_speed == UX_HIGH_SPEED_DEVICE) || (hcd -> ux_hcd_version != 0x200))
    {

        /* The device is high speed, therefore no need for TT.  */
        return;
    }

    /* We have a 1.1 device, check if the parent is a 2.0 hub.  */
    parent_device =  device -> ux_device_parent;
    if (parent_device == UX_NULL)
    {

        /* We are at the root, must be a 1.1 controller then!  */
        return;
    }

    /* We get here when the parent is a hub. The problem occurs when the hub is 
       itself connected to a chain of hubs. We need to find the first 2.0 hub 
       parent to this chain to check the TT. We need to remember the port on 
       which the first 1.1 device is hooked to.  */
    port_index =  device -> ux_device_port_location - 1;

    /* Scan the chain of hubs upward.  */
    while (parent_device != UX_NULL)
    {

        /* Is the device high speed?  */
        if (parent_device -> ux_device_speed == UX_HIGH_SPEED_DEVICE)
        {

            /* The device is a high speed hub, find the TT that manages the port. 
               The first 1.1 device is connected to. First we calculate the port 
               mapping bit.  */
            port_map =  (ULONG)(1 << port_index);

            /* Parse all the TTs attached to the hub.
               Since we confirmed exist of TT in previous _check,
               just do while loop here.
             */
            tt_index = 0;
            while(1)
            {
                /* Check if this TT owns the port where the device is attached.  */
                if ((parent_device -> ux_device_hub_tt[tt_index].ux_hub_tt_port_mapping & port_map) != 0)
                {

                    /* We have found the port, check if the tt can give us the bandwidth
                       we want to claim.  */
                    parent_device -> ux_device_hub_tt[tt_index].ux_hub_tt_max_bandwidth -=  tt_bandwidth_claimed;
                    return;
                }

                /* Try next index.  */
                tt_index ++;
            }
        }

        /* We now remember where this hub is located on the parent.  */
        port_index =  parent_device -> ux_device_port_location - 1;
        
        /* We go up one level in the hub chain.  */
        parent_device =  parent_device -> ux_device_parent;
    }

    /* We get here when we have not found a 2.0 hub in the list and we got
       to the root port.  */
    return;
}
#endif /* #if UX_MAX_DEVICES > 1 */
