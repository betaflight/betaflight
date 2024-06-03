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
#ifndef UX_HCD_EHCI_EXT_USB_HOST_MODE_ENABLE

#if defined(K66) || defined(IMX6UL) || defined(XILINX_ZYNQ) || defined(MIMXRT)
#define UX_HCD_EHCI_EXT_USB_HOST_MODE_ENABLE(hcd_ehci) do                               \
{                                                                                       \
    _ux_hcd_ehci_register_write(hcd_ehci, (hcd_ehci -> ux_hcd_ehci_hcor + 0x1A), 0x03); \
} while(0)
#else
#define UX_HCD_EHCI_EXT_USB_HOST_MODE_ENABLE(hcd_ehci)
#endif

#endif /* ifndef UX_HCD_EHCI_EXT_USB_HOST_MODE_ENABLE */

#ifndef UX_HCD_EHCI_EXT_EMBEDDED_TT_SUPPORT

#if defined(IMX25) || defined(IMX6UL) || defined(K66) || defined(LPC3131) ||            \
    defined(MCF5445X) || defined(MIMXRT)
#define UX_HCD_EHCI_EXT_EMBEDDED_TT_SUPPORT UX_TRUE
#else
#define UX_HCD_EHCI_EXT_EMBEDDED_TT_SUPPORT UX_FALSE
#endif

#endif /* ifndef UX_HCD_EHCI_EXT_EMBEDDED_TT_SUPPORT */

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_hcd_ehci_initialize                             PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function initializes the EHCI controller. It sets the DMA      */
/*    areas, programs all the EHCI registers, sets up the ED and TD       */
/*    containers, sets the control, bulk and periodic lists.              */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    HCD                                   Pointer to HCD                */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_hcd_ehci_periodic_tree_create     Create periodic tree          */
/*    _ux_hcd_ehci_power_root_hubs          Power root HUBs               */
/*    _ux_hcd_ehci_register_read            Read EHCI register            */
/*    _ux_hcd_ehci_register_write           Write EHCI register           */
/*    _ux_utility_memory_allocate           Allocate memory block         */
/*    _ux_utility_memory_delete             Delete memory block           */
/*    _ux_utility_physical_address          Get physical address          */
/*    _ux_host_semaphore_create             Create semaphore              */
/*    _ux_host_semaphore_delete             Delete semaphore              */
/*    _ux_host_mutex_create                 Create mutex                  */
/*    _ux_host_mutex_delete                 Delete mutex                  */
/*    _ux_utility_set_interrupt_handler     Set interrupt handler         */
/*    _ux_utility_delay_ms                  Delay ms                      */
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
/*                                            optimized based on compile  */
/*                                            definitions,                */
/*                                            resulting in version 6.1    */
/*  08-02-2021     Wen Wang                 Modified comment(s),          */
/*                                            fixed spelling error,       */
/*                                            resulting in version 6.1.8  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_ehci_initialize(UX_HCD *hcd)
{
#if defined(UX_HOST_STANDALONE)
    UX_PARAMETER_NOT_USED(hcd);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else

UX_HCD_EHCI             *hcd_ehci;
UX_EHCI_ED              *ed;
UX_EHCI_LINK_POINTER    lp;
ULONG                   ehci_register;
ULONG                   port_index;
UINT                    status = UX_SUCCESS;


    /* The controller initialized here is of EHCI type.  */
    hcd -> ux_hcd_controller_type =  UX_EHCI_CONTROLLER;

#if UX_MAX_DEVICES > 1
    /* Initialize the max bandwidth for periodic endpoints. On EHCI,
       the spec says no more than 90% to be allocated for periodic.  */
    hcd -> ux_hcd_available_bandwidth =  UX_EHCI_AVAILABLE_BANDWIDTH;
#endif

    /* Allocate memory for this EHCI HCD instance.  */
    hcd_ehci =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HCD_EHCI));
    if (hcd_ehci == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Set the pointer to the EHCI HCD.  */
    hcd -> ux_hcd_controller_hardware =  (VOID *) hcd_ehci;

    /* Save the register memory address.  */
    hcd_ehci -> ux_hcd_ehci_base =  (ULONG *) hcd -> ux_hcd_io;

    /* Obtain the address of the HCOR registers. This is a byte offset from the
       HCOR Cap registers.  */
    ehci_register =  _ux_hcd_ehci_register_read(hcd_ehci, EHCI_HCCR_CAP_LENGTH);
    hcd_ehci -> ux_hcd_ehci_hcor =  (ehci_register & 0xff) >> 2;

    /* Set the generic HCD owner for the EHCI HCD.  */
    hcd_ehci -> ux_hcd_ehci_hcd_owner =  hcd;

    /* Initialize the function entry for this HCD.  */
    hcd -> ux_hcd_entry_function =  _ux_hcd_ehci_entry;

    /* Set the state of the controller to HALTED first.  */
    hcd -> ux_hcd_status =  UX_HCD_STATUS_HALTED;

    /* Read the EHCI Controller Command register. */
    ehci_register =  _ux_hcd_ehci_register_read(hcd_ehci, EHCI_HCOR_USB_COMMAND);

    /* Isolate the Frame list size.  */
    ehci_register = (ehci_register >> 2) & 3;

    /* Frame list size selection.  */
    switch (ehci_register)
    {

        case 0 :

            /* Frame list size is 1024 entries.  */
            hcd_ehci -> ux_hcd_ehci_frame_list_size = 1024;
            break;

        case 1 :

            /* Frame list size is 512 entries.  */
            hcd_ehci -> ux_hcd_ehci_frame_list_size = 512;
            break;
        case 2 :

            /* Frame list size is 256 entries.  */
            hcd_ehci -> ux_hcd_ehci_frame_list_size = 256;
            break;

        default :

            /* Error, Wrong frame size. This should never happen.  */
            status = (UX_ERROR);
    }

    /* Allocate the EHCI controller frame list. The memory alignment is 4K.
       The number of entries may be changeable in some controllers. We get the value in the command register.  */
    if (status == UX_SUCCESS)
    {
        hcd_ehci -> ux_hcd_ehci_frame_list =  _ux_utility_memory_allocate(UX_ALIGN_4096, UX_CACHE_SAFE_MEMORY, (hcd_ehci -> ux_hcd_ehci_frame_list_size * 4));
        if (hcd_ehci -> ux_hcd_ehci_frame_list == UX_NULL)
            status = (UX_MEMORY_INSUFFICIENT);
    }

    /* Allocate the list of eds. All eds are allocated on 32 byte memory boundary.  */
    if (status == UX_SUCCESS)
    {
        hcd_ehci -> ux_hcd_ehci_ed_list =  _ux_utility_memory_allocate(UX_ALIGN_32, UX_CACHE_SAFE_MEMORY, sizeof(UX_EHCI_ED) * _ux_system_host -> ux_system_host_max_ed);
        if (hcd_ehci -> ux_hcd_ehci_ed_list == UX_NULL)
            status = (UX_MEMORY_INSUFFICIENT);
    }

    /* Allocate the list of tds. All tds are allocated on 32 byte memory boundary.  */
    if (status == UX_SUCCESS)
    {
        hcd_ehci -> ux_hcd_ehci_td_list =  _ux_utility_memory_allocate(UX_ALIGN_32, UX_CACHE_SAFE_MEMORY, sizeof(UX_EHCI_TD) * _ux_system_host -> ux_system_host_max_td);
        if (hcd_ehci -> ux_hcd_ehci_td_list == UX_NULL)
            status = (UX_MEMORY_INSUFFICIENT);
    }

#if UX_MAX_ISO_TD == 0 || !defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)
    hcd_ehci -> ux_hcd_ehci_fsiso_td_list = UX_NULL;
#else

    /* Allocate the list of isochronous Full Speed tds. All tds are allocated on 32 byte
       memory boundary.  */
    if (status == UX_SUCCESS)
    {
        hcd_ehci -> ux_hcd_ehci_fsiso_td_list =  _ux_utility_memory_allocate(UX_ALIGN_32, UX_CACHE_SAFE_MEMORY, sizeof(UX_EHCI_FSISO_TD) * _ux_system_host -> ux_system_host_max_iso_td);
        if (hcd_ehci -> ux_hcd_ehci_fsiso_td_list == UX_NULL)
            status = (UX_MEMORY_INSUFFICIENT);
    }
#endif

#if UX_MAX_ISO_TD == 0
    hcd_ehci -> ux_hcd_ehci_hsiso_td_list = UX_NULL;
#else

    /* Allocate the list of isochronous High Speed tds. All tds are allocated on 32 byte memory boundary.  */
    if (status == UX_SUCCESS)
    {
        hcd_ehci -> ux_hcd_ehci_hsiso_td_list =  _ux_utility_memory_allocate(UX_ALIGN_64, UX_CACHE_SAFE_MEMORY, sizeof(UX_EHCI_HSISO_TD) * _ux_system_host -> ux_system_host_max_iso_td);
        if (hcd_ehci -> ux_hcd_ehci_hsiso_td_list == UX_NULL)
            status = (UX_MEMORY_INSUFFICIENT);
    }
#endif

    /* Initialize the periodic tree.  */
    if (status == UX_SUCCESS)
        status =  _ux_hcd_ehci_periodic_tree_create(hcd_ehci);

    if (status == UX_SUCCESS)
    {

#if UX_MAX_DEVICES > 1

        /* Since this is a USB 2.0 controller, we can safely hardwire the version.  */
        hcd -> ux_hcd_version =  0x200;
#endif

        /* The EHCI Controller should not be running. */
        ehci_register =  _ux_hcd_ehci_register_read(hcd_ehci, EHCI_HCOR_USB_COMMAND);
        ehci_register &=  ~EHCI_HC_IO_RS;
        _ux_hcd_ehci_register_write(hcd_ehci, EHCI_HCOR_USB_COMMAND, ehci_register);
        _ux_utility_delay_ms(2);

        /* Perform a global reset to the controller.  */
        ehci_register =  _ux_hcd_ehci_register_read(hcd_ehci, EHCI_HCOR_USB_COMMAND);
        ehci_register |=  EHCI_HC_IO_HCRESET;
        _ux_hcd_ehci_register_write(hcd_ehci, EHCI_HCOR_USB_COMMAND, ehci_register);

        /* Ensure the reset is complete.  */
        while (ehci_register & EHCI_HC_IO_HCRESET)
        {

            ehci_register =  _ux_hcd_ehci_register_read(hcd_ehci, EHCI_HCOR_USB_COMMAND);
        }

        /* Enable host mode for hardware peripheral.  */
        UX_HCD_EHCI_EXT_USB_HOST_MODE_ENABLE(hcd_ehci);

        /* Set the Frame List register of the controller.  */
        lp.void_ptr = _ux_utility_physical_address(hcd_ehci -> ux_hcd_ehci_frame_list);
        _ux_hcd_ehci_register_write(hcd_ehci, EHCI_HCOR_FRAME_LIST_BASE_ADDRESS, lp.value);

        /* We need one endpoint to be inserted into the Asynchronous list.  */
        ed =  _ux_hcd_ehci_ed_obtain(hcd_ehci);
        if (ed == UX_NULL)
            status = (UX_NO_ED_AVAILABLE);
    }

    if (status == UX_SUCCESS)
    {

        /* Make this ED point to itself.  */
        lp.void_ptr = _ux_utility_physical_address(ed);

        /* Store the physical address of this Ed into the asynch list.  */
        _ux_hcd_ehci_register_write(hcd_ehci, EHCI_HCOR_ASYNCH_LIST_ADDRESS, lp.value);

        /* Store LP with QH Typ.  */
        lp.value |= UX_EHCI_QH_TYP_QH;
        ed -> ux_ehci_ed_queue_head = lp.ed_ptr;

        /* This ED will be the HEAD ED in the asynch list.  */
        ed -> ux_ehci_ed_cap0 =  UX_EHCI_QH_HEAD;

        /* We keep this ED as being the first, the last and the head ED.  */
        hcd_ehci -> ux_hcd_ehci_asynch_head_list =   ed;
        hcd_ehci -> ux_hcd_ehci_asynch_first_list =  ed;
        hcd_ehci -> ux_hcd_ehci_asynch_last_list =   ed;

        /* Set the EHCI Interrupt threshold default value (1 or 8 per ms)
        and the size of the frame list.  */
        _ux_hcd_ehci_register_write(hcd_ehci, EHCI_HCOR_USB_COMMAND, EHCI_HC_IO_ITC);

        /* Get the number of ports on the controller. The number of ports
        needs to be reflected both for the generic HCD container and the
        local ehci container.  */
        ehci_register =                         _ux_hcd_ehci_register_read(hcd_ehci, EHCI_HCCR_HCS_PARAMS);
        hcd -> ux_hcd_nb_root_hubs =            (UINT) (ehci_register & 0xf);
        if (hcd -> ux_hcd_nb_root_hubs > UX_MAX_ROOTHUB_PORT)
            hcd -> ux_hcd_nb_root_hubs = UX_MAX_ROOTHUB_PORT;
        hcd_ehci -> ux_hcd_ehci_nb_root_hubs =  hcd -> ux_hcd_nb_root_hubs;

        /* The controller transceiver can now send the device connection/extraction
        signals to the EHCI controller.  */
        _ux_hcd_ehci_register_write(hcd_ehci, EHCI_HCOR_CONFIG_FLAG, UX_EHCI_ROUTE_TO_LOCAL_HC);

        /* Create mutex for periodic list modification.  */
        status = _ux_host_mutex_create(&hcd_ehci -> ux_hcd_ehci_periodic_mutex, "ehci_periodic_mutex");
        if (status != UX_SUCCESS)
            status = (UX_MUTEX_ERROR);
    }

    /* We must enable the HCD protection semaphore.  */
    if (status == UX_SUCCESS)
    {
        status =  _ux_host_semaphore_create(&hcd_ehci -> ux_hcd_ehci_protect_semaphore, "ux_hcd_protect_semaphore", 1);
        if (status != UX_SUCCESS)
            status = (UX_SEMAPHORE_ERROR);
    }

    /* We must enable the HCD doorbell semaphore.  */
    if (status == UX_SUCCESS)
    {
        status =  _ux_host_semaphore_create(&hcd_ehci -> ux_hcd_ehci_doorbell_semaphore, "ux_hcd_doorbell_semaphore", 0);
        if (status != UX_SUCCESS)
            status = (UX_SEMAPHORE_ERROR);
    }

    if (status == UX_SUCCESS)
    {

        /* The EHCI Controller can now be Started. */
        ehci_register =  _ux_hcd_ehci_register_read(hcd_ehci, EHCI_HCOR_USB_COMMAND);

        /* Set the Frame list size and the RUN bit.. */
        ehci_register |= UX_EHCI_FRAME_LIST_MASK
                        | EHCI_HC_IO_RS
                        | EHCI_HC_IO_ASE
                        | EHCI_HC_IO_PSE;
        _ux_hcd_ehci_register_write(hcd_ehci, EHCI_HCOR_USB_COMMAND, ehci_register);

        /* Regular EHCI with embedded TT.  */
        hcd_ehci -> ux_hcd_ehci_embedded_tt = UX_HCD_EHCI_EXT_EMBEDDED_TT_SUPPORT;

        /* All ports must now be powered to pick up device insertion.  */
        _ux_hcd_ehci_power_root_hubs(hcd_ehci);

        /* Set the state of the controller to OPERATIONAL.  */
        hcd -> ux_hcd_status =  UX_HCD_STATUS_OPERATIONAL;

        /* Set the EHCI Interrupt Register.  */
        _ux_hcd_ehci_register_write(hcd_ehci, EHCI_HCOR_USB_INTERRUPT, EHCI_HC_INTERRUPT_ENABLE_NORMAL);

        /* The controller interrupt must have a handler and be active now.  */
        _ux_utility_set_interrupt_handler(hcd -> ux_hcd_irq, _ux_hcd_ehci_interrupt_handler);


        /* Force a enum process if CCS detected.
        ** Because CSC may keep zero in this case.
        */
        for (port_index = 0, status = 0; port_index < hcd_ehci -> ux_hcd_ehci_nb_root_hubs; port_index ++)
        {

            /* Read register.  */
            ehci_register = _ux_hcd_ehci_register_read(hcd_ehci, EHCI_HCOR_PORT_SC + port_index);

            /* Check CCS.  */
            if (ehci_register & EHCI_HC_PS_CCS)
            {
                hcd_ehci -> ux_hcd_ehci_hcd_owner -> ux_hcd_root_hub_signal[port_index]++;
                status ++;
            }
        }

        /* Wakeup enum thread.  */
        if (status != 0)
            _ux_host_semaphore_put(&_ux_system_host -> ux_system_host_enum_semaphore);

        /* Return successful status.  */
        return(UX_SUCCESS);
    }

    /* Error! Free resources!  */
    if (hcd_ehci -> ux_hcd_ehci_frame_list)
        _ux_utility_memory_free(hcd_ehci -> ux_hcd_ehci_frame_list);
    if (hcd_ehci -> ux_hcd_ehci_ed_list)
        _ux_utility_memory_free(hcd_ehci -> ux_hcd_ehci_ed_list);
    if (hcd_ehci -> ux_hcd_ehci_td_list)
        _ux_utility_memory_free(hcd_ehci -> ux_hcd_ehci_td_list);
#if UX_MAX_ISO_TD && defined(UX_HCD_EHCI_SPLIT_TRANSFER_ENABLE)
    if (hcd_ehci -> ux_hcd_ehci_fsiso_td_list)
        _ux_utility_memory_free(hcd_ehci -> ux_hcd_ehci_fsiso_td_list);
#endif
#if UX_MAX_ISO_TD
    if (hcd_ehci -> ux_hcd_ehci_hsiso_td_list)
        _ux_utility_memory_free(hcd_ehci -> ux_hcd_ehci_hsiso_td_list);
#endif
    if (hcd_ehci -> ux_hcd_ehci_periodic_mutex.tx_mutex_id != 0)
        _ux_host_mutex_delete(&hcd_ehci -> ux_hcd_ehci_periodic_mutex);
    if (hcd_ehci -> ux_hcd_ehci_protect_semaphore.tx_semaphore_id != 0)
        _ux_host_semaphore_delete(&hcd_ehci -> ux_hcd_ehci_protect_semaphore);
    if (hcd_ehci -> ux_hcd_ehci_doorbell_semaphore.tx_semaphore_id != 0)
        _ux_host_semaphore_delete(&hcd_ehci -> ux_hcd_ehci_doorbell_semaphore);
    _ux_utility_memory_free(hcd_ehci);

    /* Return error status code.  */
    return(status);
#endif
}
