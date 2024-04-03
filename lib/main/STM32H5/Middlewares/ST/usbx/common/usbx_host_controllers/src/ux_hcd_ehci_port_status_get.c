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
/*    _ux_hcd_ehci_port_status_get                        PORTABLE C      */ 
/*                                                           6.1.8        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will return the status for each port attached to the  */
/*    root HUB.                                                           */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_ehci                              Pointer to EHCI controller    */ 
/*    port_index                            Port index to get status for  */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Port Status                                                         */ 
/*                                                                        */ 
/*      Status of the root hub port with the following format:            */
/*                                                                        */ 
/*               bit 0         device connection status                   */
/*                             if 0 : no device connected                 */
/*                             if 1 : device connected to the port        */
/*               bit 1         port enable status                         */
/*                             if 0 : port disabled                       */
/*                             if 1 : port enabled                        */
/*               bit 2         port suspend status                        */
/*                             if 0 : port is not suspended               */
/*                             if 1 : port is suspended                   */
/*               bit 3         port overcurrent status                    */
/*                             if 0 : port has no overcurrent condition   */
/*                             if 1 : port has overcurrent condition      */
/*               bit 4         port reset status                          */
/*                             if 0 : port is not in reset                */
/*                             if 1 : port is in reset                    */
/*               bit 5         port power status                          */
/*                             if 0 : port power is off                   */
/*                             if 1 : port power is on                    */
/*               bit 6-7       device attached speed                      */
/*                             if 00 : low speed device attached          */
/*                             if 01 : full speed device attached         */
/*                             if 10 : high speed device attached         */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_hcd_ehci_register_read            Read EHCI register            */ 
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
ULONG  _ux_hcd_ehci_port_status_get(UX_HCD_EHCI *hcd_ehci, ULONG port_index)
{

ULONG       ehci_register_port_status;
ULONG       port_status;


    /* Check to see if this port is valid on this controller.  */
    if (hcd_ehci -> ux_hcd_ehci_nb_root_hubs < port_index)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_PORT_INDEX_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_PORT_INDEX_UNKNOWN, port_index, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_PORT_INDEX_UNKNOWN);
    }
    
    /* The port is valid, build the status mask for this port. This function
       returns a controller agnostic bit field.  */
    port_status =  0;
    ehci_register_port_status =  _ux_hcd_ehci_register_read(hcd_ehci, EHCI_HCOR_PORT_SC + port_index);
                                    
    /* Device Connection Status.  */
    if (ehci_register_port_status & EHCI_HC_PS_CCS)
        port_status |=  UX_PS_CCS;
    else
    {

        /* When disconnected PHY does not know speed.  */
        UX_HCD_EHCI_EXT_USBPHY_HIGHSPEED_MODE_SET(hcd_ehci, UX_FALSE);
    }
                                    
    /* Port Enable Status.  */
    if (ehci_register_port_status & EHCI_HC_PS_PE)
        port_status |=  UX_PS_PES;

    /* Port Suspend Status.  */
    if (ehci_register_port_status & EHCI_HC_PS_SUSPEND)
    {
        port_status |=  UX_PS_PSS;

        /* When suspend put PHY in normal to avoid wrong disconnect status.  */
        UX_HCD_EHCI_EXT_USBPHY_HIGHSPEED_MODE_SET(hcd_ehci, UX_FALSE);
    }

    /* Port Overcurrent Status.  */
    if (ehci_register_port_status & EHCI_HC_PS_OCC)
        port_status |=  UX_PS_POCI;

    /* Port Reset Status.  */
    if (ehci_register_port_status & EHCI_HC_PS_PR)
        port_status |=  UX_PS_PRS;

    /* Port Power Status.  */
    if (ehci_register_port_status & EHCI_HC_PS_PP)
        port_status |=  UX_PS_PPS;

    /* Port Device Attached speed. This field is valid only if the CCS bit is active. 
       Only EHCI high speed devices are meaningful in a regular EHCI controller. 
       In embedded EHCI with built-in TTs some bits reflect the true speed of
       the device behind the TT. */
    if (ehci_register_port_status & EHCI_HC_PS_CCS)
    {
        /* Check for EHCI with embedded TT.  */
        if (hcd_ehci -> ux_hcd_ehci_embedded_tt == UX_TRUE)
        {
    
            /* Isolate speed from the non EHCI compliant POTSC bits.  */
            switch (ehci_register_port_status & EHCI_HC_PS_EMBEDDED_TT_SPEED_MASK)
            {
            
                case EHCI_HC_PS_EMBEDDED_TT_SPEED_FULL        :

                    /* Full speed.  */
                    port_status |=  UX_PS_DS_FS;
                    break;        

                case EHCI_HC_PS_EMBEDDED_TT_SPEED_LOW         :

                    /* Low speed.  */
                    port_status |=  UX_PS_DS_LS;
                    break;        

                case EHCI_HC_PS_EMBEDDED_TT_SPEED_HIGH        :

                    /* High speed.  */
                    port_status |=  UX_PS_DS_HS;
                    break;        

            }
        }
        else

            /* No embedded TT. Fall back to default HS.  */
            port_status |=  UX_PS_DS_HS;
    }
            
    /* Return port status.  */
    return(port_status);            
}

