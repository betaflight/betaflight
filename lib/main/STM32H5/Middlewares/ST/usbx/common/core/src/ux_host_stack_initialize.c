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

UX_COMPILE_TIME_ASSERT(!UX_OVERFLOW_CHECK_MULC_ULONG(sizeof(UX_HCD), UX_MAX_HCD), UX_MAX_HCD_mul_ovf)
UX_COMPILE_TIME_ASSERT(!UX_OVERFLOW_CHECK_MULC_ULONG(sizeof(UX_HOST_CLASS), UX_MAX_CLASS_DRIVER), UX_MAX_CLASS_DRIVER_mul_ovf)
UX_COMPILE_TIME_ASSERT(!UX_OVERFLOW_CHECK_MULC_ULONG(sizeof(UX_DEVICE), UX_MAX_DEVICES), UX_MAX_DEVICES_mul_ovf)

#ifndef UX_HOST_HNP_POLLING_THREAD_STACK_SIZE
#define UX_HOST_HNP_POLLING_THREAD_STACK_SIZE UX_THREAD_STACK_SIZE
#endif

/* Defined USBX host variables.  */

UX_SYSTEM_HOST     *_ux_system_host;

/* Define table of periodic tree entries, properly indexed.  */

UINT _ux_system_host_hcd_periodic_tree_entries[32] = { 
                                                                            0x00, 0x10, 0x08, 0x18, 0x04, 0x14, 0x0c, 0x1c,
                                                                            0x02, 0x12, 0x0a, 0x1a, 0x06, 0x16, 0x0e, 0x1e,
                                                                            0x01, 0x11, 0x09, 0x19, 0x05, 0x15, 0x0d, 0x1d,
                                                                            0x03, 0x13, 0x0b, 0x1b, 0x07, 0x17, 0x0f, 0x1f};

/* Define the names of all the USB Classes of USBX.  */

UCHAR _ux_system_host_class_hub_name[] =                                    "ux_host_class_hub";
UCHAR _ux_system_host_class_printer_name[] =                                "ux_host_class_printer";
UCHAR _ux_system_host_class_storage_name[] =                                "ux_host_class_storage";
UCHAR _ux_system_host_class_hid_name[] =                                    "ux_host_class_hid";
UCHAR _ux_system_host_class_audio_name[] =                                  "ux_host_class_audio";
UCHAR _ux_system_host_class_cdc_acm_name[] =                                "ux_host_class_cdc_acm";
UCHAR _ux_system_host_class_cdc_dlc_name[] =                                "ux_host_class_cdc_dlc";
UCHAR _ux_system_host_class_cdc_ecm_name[] =                                "ux_host_class_cdc_ecm";
UCHAR _ux_system_host_class_prolific_name[] =                               "ux_host_class_prolific";
UCHAR _ux_system_host_class_pima_name[] =                                   "ux_host_class_pima";
UCHAR _ux_system_host_class_dpump_name[] =                                  "ux_host_class_dpump";
UCHAR _ux_system_host_class_asix_name[] =                                   "ux_host_class_asix";
UCHAR _ux_system_host_class_swar_name[] =                                   "ux_host_class_sierra_wireless";
UCHAR _ux_system_host_class_gser_name[] =                                   "ux_host_class_generic_serial";
UCHAR _ux_system_host_class_hid_client_remote_control_name[] =              "ux_host_class_hid_client_remote_control";
UCHAR _ux_system_host_class_hid_client_mouse_name[] =                       "ux_host_class_hid_client_mouse";
UCHAR _ux_system_host_class_hid_client_keyboard_name[] =                    "ux_host_class_hid_client_keyboard";

/* Define the name of all the USB Host Controllers of USBX.  */

UCHAR _ux_system_host_hcd_ohci_name[] =                                     "ux_hcd_ohci";
UCHAR _ux_system_host_hcd_ehci_name[] =                                     "ux_hcd_ehci";
UCHAR _ux_system_host_hcd_isp1161_name[] =                                  "ux_hcd_isp1161";
UCHAR _ux_system_host_hcd_isp1362_name[] =                                  "ux_hcd_isp1362";
UCHAR _ux_system_host_hcd_sh2_name[] =                                      "ux_hcd_rx";
UCHAR _ux_system_host_hcd_rx_name[] =                                       "ux_hcd_sh2";
UCHAR _ux_system_host_hcd_pic32_name[] =                                    "ux_hcd_pic32";
UCHAR _ux_system_host_hcd_stm32_name[] =                                    "ux_hcd_stm32";
UCHAR _ux_system_host_hcd_musb_name[] =                                     "ux_hcd_musb";
UCHAR _ux_system_host_hcd_atm7_name[] =                                     "ux_hcd_atm7";
UCHAR _ux_system_host_hcd_simulator_name[] =                                "ux_hcd_simulator";

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_stack_initialize                           PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function initializes all the host code for USBX to work on a   */ 
/*    specific platform.                                                  */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    (ux_system_host_change_function)        Function pointer to the     */ 
/*                                            callback function for a     */ 
/*                                            device change               */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_memory_allocate           Allocate host memory          */ 
/*    _ux_utility_semaphore_create          Create host semaphore         */ 
/*    _ux_utility_mutex_create             Create host mutex              */ 
/*    _ux_utility_thread_create             Create host thread            */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            optimized based on compile  */
/*                                            definitions, used UX prefix */
/*                                            to refer to TX symbols      */
/*                                            instead of using them       */
/*                                            directly,                   */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_initialize(UINT (*ux_system_host_change_function)(ULONG, UX_HOST_CLASS *, VOID *))
{

UINT        status;
UCHAR       *memory;
#if defined(UX_HOST_STANDALONE)
UINT        i;
UX_DEVICE   *device;
#endif

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_INITIALIZE, 0, 0, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    /* Initialize some of the global so that we don't have to recompile the
       core code when one item is adjusted.  */
    _ux_system_host -> ux_system_host_max_ed =        UX_MAX_ED;
    _ux_system_host -> ux_system_host_max_td =        UX_MAX_TD;
    _ux_system_host -> ux_system_host_max_iso_td =    UX_MAX_ISO_TD;
    UX_SYSTEM_HOST_MAX_CLASS_SET(UX_MAX_CLASS_DRIVER);
    UX_SYSTEM_HOST_MAX_HCD_SET(UX_MAX_HCD);
    UX_SYSTEM_HOST_MAX_DEVICES_SET(UX_MAX_DEVICES);
    
    /* Set the change device function address.  */
    _ux_system_host -> ux_system_host_change_function =  ux_system_host_change_function;

    /* Allocate memory for the HCDs.
     * sizeof(UX_HCD)*UX_MAX_HCD overflow is checked outside of the function.
     */
    memory =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HCD)*UX_MAX_HCD);

    /* Check for successful allocation.  */
    if (memory == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Set to success by default.  */
    status = UX_SUCCESS;

    /* Store memory in system structure.  */
    _ux_system_host -> ux_system_host_hcd_array =  (UX_HCD *) memory;

    /* Allocate memory for the classes.
     * sizeof(UX_HOST_CLASS)*UX_MAX_CLASS_DRIVER overflow is checked outside of the function.
     */
    memory =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HOST_CLASS)*UX_MAX_CLASS_DRIVER);

    /* Check for successful allocation.  */
    if (memory == UX_NULL)
        status = UX_MEMORY_INSUFFICIENT;
    else

        /* Store memory in system structure.  */
        _ux_system_host -> ux_system_host_class_array =  (UX_HOST_CLASS *) memory;

    /* Allocate memory for the device containers.
     * sizeof(UX_DEVICE)*UX_MAX_DEVICES overflow is checked outside of the function.
     */
    if (status == UX_SUCCESS)
    {
        memory =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_DEVICE)*UX_MAX_DEVICES);

        /* Check for successful allocation.  */
        if(memory == UX_NULL)
            status = UX_MEMORY_INSUFFICIENT;
        else

            /* Store memory in system structure.  */
            _ux_system_host -> ux_system_host_device_array =  (UX_DEVICE *) memory;

#if defined(UX_HOST_STANDALONE)

        /* Add devices to the enumeration list, with ENUM flags cleared.  */
        if (status == UX_SUCCESS)
        {

            /* Start from the last device instance.  */
            device = &_ux_system_host -> ux_system_host_device_array[UX_MAX_DEVICES - 1];

            /* Insert all devices to enumeration list head.  */
            for (i = 0; i < UX_MAX_DEVICES; i ++)
            {

                /* Insert to head.  */
                device -> ux_device_enum_next = _ux_system_host -> ux_system_host_enum_device;
                _ux_system_host -> ux_system_host_enum_device = device;

                /* Next device.  */
                device --;
            }
        }
#endif

    }

#if !defined(UX_HOST_STANDALONE)
    /* Obtain enough stack for the two USBX host threads.  */
    if (status == UX_SUCCESS)
    {
        _ux_system_host -> ux_system_host_enum_thread_stack =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY,
                                                                            UX_HOST_ENUM_THREAD_STACK_SIZE);

        /* Check for successful allocation.  */
        if (_ux_system_host -> ux_system_host_enum_thread_stack == UX_NULL)
            status = UX_MEMORY_INSUFFICIENT;
    }

    /* Allocate another stack area.  */
    if (status == UX_SUCCESS)
    {
        _ux_system_host -> ux_system_host_hcd_thread_stack =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY,
                                                                            UX_HOST_HCD_THREAD_STACK_SIZE);

        /* Check for successful allocation.  */
        if (_ux_system_host -> ux_system_host_hcd_thread_stack == UX_NULL)
            status = UX_MEMORY_INSUFFICIENT;
    }

    /* Create the semaphores used by the hub and root hub to awake the enumeration thread.  */
    if (status == UX_SUCCESS)
    {
        status =  _ux_utility_semaphore_create(&_ux_system_host -> ux_system_host_enum_semaphore, "ux_system_host_enum_semaphore", 0);
        if(status != UX_SUCCESS)
            status = UX_SEMAPHORE_ERROR;
    }

    /* Create the semaphores used by the HCD to perform the completion phase of transfer_requests.  */
    if (status == UX_SUCCESS)
    {
        status =  _ux_utility_semaphore_create(&_ux_system_host -> ux_system_host_hcd_semaphore, "ux_system_host_hcd_semaphore", 0);
        if(status != UX_SUCCESS)
            status = UX_SEMAPHORE_ERROR;
    }

    /* Create the enumeration thread of USBX.  */
    if (status == UX_SUCCESS)
    {
        status =  _ux_utility_thread_create(&_ux_system_host -> ux_system_host_enum_thread, "ux_system_host_enum_thread", _ux_host_stack_enum_thread_entry,
                            0, _ux_system_host -> ux_system_host_enum_thread_stack,
                            UX_HOST_ENUM_THREAD_STACK_SIZE, UX_THREAD_PRIORITY_ENUM,
                            UX_THREAD_PRIORITY_ENUM, UX_NO_TIME_SLICE, UX_AUTO_START);
                            
        /* Check the completion status.  */
        if(status != UX_SUCCESS)
            status = UX_THREAD_ERROR;
    }

    /* Create the HCD thread of USBX.  */
    if (status == UX_SUCCESS)
    {
        status =  _ux_utility_thread_create(&_ux_system_host -> ux_system_host_hcd_thread, "ux_host_stack_hcd_thread", _ux_host_stack_hcd_thread_entry,
                            0, _ux_system_host -> ux_system_host_hcd_thread_stack,
                            UX_HOST_HCD_THREAD_STACK_SIZE, UX_THREAD_PRIORITY_HCD,
                            UX_THREAD_PRIORITY_HCD, UX_NO_TIME_SLICE,UX_AUTO_START);

        /* Check the completion status.  */
        if(status != UX_SUCCESS)
            status = UX_THREAD_ERROR;
    }
#endif

#if defined(UX_OTG_SUPPORT) && !defined(UX_OTG_STANDALONE)
    /* Allocate another stack area for the HNP polling thread.  */
    if (status == UX_SUCCESS)
    {
        _ux_system_host -> ux_system_host_hnp_polling_thread_stack =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY,
                                                                            UX_HOST_HNP_POLLING_THREAD_STACK_SIZE);

        /* Check for successful allocation.  */
        if (_ux_system_host -> ux_system_host_hnp_polling_thread_stack == UX_NULL)
            status = UX_MEMORY_INSUFFICIENT;
    }
        
    /* Create the HNP polling thread of USBX.  */
    if (status == UX_SUCCESS)
    {
        status =  _ux_utility_thread_create(&_ux_system_host -> ux_system_host_hnp_polling_thread, "ux_host_stack_hnp_polling_thread", _ux_host_stack_hnp_polling_thread_entry,
                            0, _ux_system_host -> ux_system_host_hnp_polling_thread_stack,
                            UX_HOST_HNP_POLLING_THREAD_STACK_SIZE, UX_THREAD_PRIORITY_ENUM,
                            UX_THREAD_PRIORITY_ENUM, UX_NO_TIME_SLICE, UX_AUTO_START);

        /* Check the completion status.  */
        if (status != UX_SUCCESS)
            status = UX_THREAD_ERROR;

        /* Return success (SUCCESS).  */
        else
            return(status);
    }

    /* Free up resources on error cases.  */

    /* Last resource, _ux_system_host -> ux_system_host_hnp_polling_thread is not created or created error,
     * no need to delete it.  */

    /* Free _ux_system_host -> ux_system_host_hnp_polling_thread_stack.  */
    if (_ux_system_host -> ux_system_host_hnp_polling_thread_stack)
        _ux_utility_memory_free(_ux_system_host -> ux_system_host_hnp_polling_thread_stack);

    /* Delete _ux_system_host -> ux_system_host_hcd_thread.  */
    if (_ux_system_host -> ux_system_host_hcd_thread.tx_thread_id != 0)
        _ux_utility_thread_delete(&_ux_system_host -> ux_system_host_hcd_thread);
#else

    /* Return completion status to caller if success.  */
    if (status == UX_SUCCESS)
        return(status);

    /* Free up resources on error cases.  */

    /* Last resource, _ux_system_host -> ux_system_host_hcd_thread is not created or created error,
     * no need to delete it.  */
#endif

#if !defined(UX_HOST_STANDALONE)
    /* Delete _ux_system_host -> ux_system_host_enum_thread.  */
    if (_ux_system_host -> ux_system_host_enum_thread.tx_thread_id != 0)
        _ux_utility_thread_delete(&_ux_system_host -> ux_system_host_enum_thread);
    
    /* Delete _ux_system_host -> ux_system_host_hcd_semaphore.  */
    if (_ux_system_host -> ux_system_host_hcd_semaphore.tx_semaphore_id != 0)
        _ux_utility_semaphore_delete(&_ux_system_host -> ux_system_host_hcd_semaphore);

    /* Delete _ux_system_host -> ux_system_host_enum_semaphore.  */
    if (_ux_system_host -> ux_system_host_enum_semaphore.tx_semaphore_id != 0)
        _ux_utility_semaphore_delete(&_ux_system_host -> ux_system_host_enum_semaphore);

    /* Free _ux_system_host -> ux_system_host_hcd_thread_stack.  */
    if (_ux_system_host -> ux_system_host_hcd_thread_stack)
        _ux_utility_memory_free(_ux_system_host -> ux_system_host_hcd_thread_stack);

    /* Free _ux_system_host -> ux_system_host_enum_thread_stack.  */
    if (_ux_system_host -> ux_system_host_enum_thread_stack)
        _ux_utility_memory_free(_ux_system_host -> ux_system_host_enum_thread_stack);
#endif

    /* Free _ux_system_host -> ux_system_host_device_array.  */
    if (_ux_system_host -> ux_system_host_device_array)
        _ux_utility_memory_free(_ux_system_host -> ux_system_host_device_array);
    
    /* Free _ux_system_host -> ux_system_host_class_array.  */
    if (_ux_system_host -> ux_system_host_class_array)
        _ux_utility_memory_free(_ux_system_host -> ux_system_host_class_array);

    /* Free _ux_system_host -> ux_system_host_hcd_array.  */
    _ux_utility_memory_free(_ux_system_host -> ux_system_host_hcd_array);

    /* Return completion status to caller.  */
    return(status);
}

