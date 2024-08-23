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
/**   EHCI Controller Driver                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_hcd_ehci.h"
#include "ux_host_stack.h"


/* EHCI HCD extension for host mode select.  */
#ifndef UX_HCD_EHCI_EXT_USBPHY_HIGHSPEED_MODE_SET

#if defined(K66)

#define UX_EHCI_USBPHY_CTRL_K66             0x400A2000
#define UX_EHCI_USBPHY_CTRL_SET_BIT1        ((*(volatile ULONG *)(UX_EHCI_USBPHY_CTRL_K66 + 0x34)) = 0x02)
#define UX_EHCI_USBPHY_CTRL_CLEAR_BIT1      ((*(volatile ULONG *)(UX_EHCI_USBPHY_CTRL_K66 + 0x38)) = 0x02)

#define UX_HCD_EHCI_EXT_USBPHY_HIGHSPEED_MODE_SET(hcd_ehci, on_off) do          \
{                                                                               \
    if (on_off)                                                                 \
        UX_EHCI_USBPHY_CTRL_SET_BIT1;                                           \
    else                                                                        \
        UX_EHCI_USBPHY_CTRL_CLEAR_BIT1;                                         \
} while(0)

#elif defined(IMX6UL) || defined(MIMXRT)

#if defined(IMX6UL)
#define UX_EHCI_USBPHY1                     (0x020C9000)
#define UX_EHCI_USBPHY2                     (0x020CA000)
#define UX_EHCI_BASE1                       (0x02184100)
#define UX_EHCI_BASE2                       (0x02184300)
#elif defined(MIMXRT)
#define UX_EHCI_USBPHY1                     (0x400D9000u)
#define UX_EHCI_USBPHY2                     (0x400DA000u)
#define UX_EHCI_BASE1                       (0x402E0100u)
#define UX_EHCI_BASE2                       (0x402E0300u)
#endif

#define UX_EHCI_USBPHY_CTRL_SET_BIT1(base)                  ((*(volatile ULONG *) ( base + 0x34)) = 0x02)
#define UX_EHCI_USBPHY_CTRL_CLEAR_BIT1(base)                ((*(volatile ULONG *) ( base + 0x38)) = 0x02)

#define UX_HCD_EHCI_EXT_USBPHY_HIGHSPEED_MODE_SET(hcd_ehci, on_off) do          \
{                                                                               \
    ULONG base;                                                                 \
    if ((ULONG)hcd_ehci -> ux_hcd_ehci_base == UX_EHCI_BASE1)                   \
        base = (UX_EHCI_USBPHY1);                                               \
    else                                                                        \
        base = (UX_EHCI_USBPHY2);                                               \
    if (on_off)                                                                 \
        UX_EHCI_USBPHY_CTRL_SET_BIT1(base);                                     \
    else                                                                        \
        UX_EHCI_USBPHY_CTRL_CLEAR_BIT1(base);                                   \
} while(0)

#else
#define UX_HCD_EHCI_EXT_USBPHY_HIGHSPEED_MODE_SET(hcd_ehci, on_off)
#endif

#endif /* ifndef UX_HCD_EHCI_EXT_USBPHY_HIGHSPEED_MODE_SET */

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_hcd_ehci_port_reset                             PORTABLE C      */ 
/*                                                           6.1.8        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will reset a specific port attached to the root HUB.  */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_ehci                              Pointer to EHCI controller    */ 
/*    port_index                            Port index to reset           */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_hcd_ehci_register_read            Read EHCI register            */ 
/*    _ux_hcd_ehci_register_write           Write EHCI register           */ 
/*    _ux_utility_delay_ms                  Delay                         */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    EHCI Controller Driver                                              */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed NXP register base,    */
/*                                            resulting in version 6.1    */
/*  08-02-2021     Wen Wang                 Modified comment(s),          */
/*                                            fixed spelling error,       */
/*                                            resulting in version 6.1.8  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_ehci_port_reset(UX_HCD_EHCI *hcd_ehci, ULONG port_index)
{

ULONG       ehci_register_port_status;
INT         i;


    /* Check to see if this port is valid on this controller.  */
    if (hcd_ehci -> ux_hcd_ehci_nb_root_hubs < port_index)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_PORT_INDEX_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_PORT_INDEX_UNKNOWN, port_index, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_PORT_INDEX_UNKNOWN);
    }
    
    /* Ensure that the downstream port has a device attached. It is unnatural to perform 
       a port reset if there is no device.  */
    ehci_register_port_status =  _ux_hcd_ehci_register_read(hcd_ehci, EHCI_HCOR_PORT_SC + port_index);
                                    
    /* Check Device Connection Status.  */
    if ((ehci_register_port_status & EHCI_HC_PS_CCS) == 0)
    {
    
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_NO_DEVICE_CONNECTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_NO_DEVICE_CONNECTED, port_index, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_NO_DEVICE_CONNECTED);
    }
    
    /* Check for the speed of the device. If low speed, we need to send the connection signal 
       to the companion chip.  Unless we are on a controller with a built-in TT. */
    if ((ehci_register_port_status & EHCI_HC_PS_SPEED_MASK) != EHCI_HC_PS_SPEED_LOW || (hcd_ehci -> ux_hcd_ehci_embedded_tt == UX_TRUE))
    {

        for (i = 0; ; i ++)
        {

            /* Before reset, phy does not know the speed.  */
            UX_HCD_EHCI_EXT_USBPHY_HIGHSPEED_MODE_SET(hcd_ehci, UX_FALSE);

            /* The device may be high speed or full speed, we try to reset the port for some time 
            and see if the port is enabled by the host controller.  */
            _ux_hcd_ehci_register_write(hcd_ehci, EHCI_HCOR_PORT_SC + port_index, (ehci_register_port_status | EHCI_HC_PS_PR));

            /* Wait until the port has been reset.  */
            _ux_utility_delay_ms(EHCI_HC_RH_RESET_DELAY );

            /* Now turn off the port reset.  */
            ehci_register_port_status =  _ux_hcd_ehci_register_read(hcd_ehci, EHCI_HCOR_PORT_SC + port_index);
            if (ehci_register_port_status & EHCI_HC_PS_PR)
            {

                ehci_register_port_status &= ~EHCI_HC_PS_PR;
                _ux_hcd_ehci_register_write(hcd_ehci, EHCI_HCOR_PORT_SC + port_index, ehci_register_port_status);

                /* According to the USB 2.0 spec, the controller may take 2ms to reset the port bit from 1 to 0 after 
                we write a 0. Wait until the port reset bit has been turned off completely.  */
                _ux_utility_delay_ms(EHCI_HC_RH_RESET_SETTLE_DELAY);
            }

            /* Now check port speed.  */
            ehci_register_port_status =  _ux_hcd_ehci_register_read(hcd_ehci, EHCI_HCOR_PORT_SC + port_index);
            if (hcd_ehci -> ux_hcd_ehci_embedded_tt &&
                (ehci_register_port_status & EHCI_HC_PS_EMBEDDED_TT_SPEED_MASK) == EHCI_HC_PS_EMBEDDED_TT_SPEED_HIGH)
                break;

            /* Seems we need to set the Port to a Suspend state before forcing the reset. Otherwise some devices fail the 
               HS detection handshake.  */
            if (i == 0)
            {
                _ux_hcd_ehci_register_write(hcd_ehci, EHCI_HCOR_PORT_SC + port_index, (ehci_register_port_status | EHCI_HC_PS_SUSPEND));
                _ux_utility_delay_ms(UX_HIGH_SPEED_DETECTION_HANDSHAKE_SUSPEND_WAIT);
                _ux_hcd_ehci_register_write(hcd_ehci, EHCI_HCOR_PORT_SC + port_index, (ehci_register_port_status & ~EHCI_HC_PS_SUSPEND));
            }
            else
                break;
        }

        /* Now we can read the port enable bit.  */
        ehci_register_port_status =  _ux_hcd_ehci_register_read(hcd_ehci, EHCI_HCOR_PORT_SC + port_index);
        if ((ehci_register_port_status & EHCI_HC_PS_PE) != 0)
        {

            /* After reset, adjust phy speed.  */
            UX_HCD_EHCI_EXT_USBPHY_HIGHSPEED_MODE_SET(hcd_ehci, UX_TRUE);
            return(UX_SUCCESS);
        }
    }

    /* We come here when the device is either low speed or full speed. In this case, we release 
       the ownership of the port to the companion chip.  */
    _ux_hcd_ehci_register_write(hcd_ehci, EHCI_HCOR_PORT_SC + port_index, (ehci_register_port_status | EHCI_HC_PS_PO));
                        
    /* Delay.  */
    _ux_utility_delay_ms(EHCI_HC_RH_RESET_DELAY);
    
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_PORT_RESET_FAILED, port_index, 0, 0, UX_TRACE_ERRORS, 0, 0)

    /* Error trap. */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_PORT_RESET_FAILED);

    /* When the root HUB sees an error message, it will give up on this device and the companion chip root HUB 
       will pick up the insertion signal again and reawake the root HUB driver.  */
    return(UX_PORT_RESET_FAILED);       
}

