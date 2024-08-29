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


#if defined(UX_OTG_SUPPORT)
/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_stack_hnp_polling_thread_entry             PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is awaken every 2 seconds to check if there is a      */ 
/*    OTG device on the bus and if so perform a GET_STATUS as the device  */ 
/*    may request a change of role.                                       */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    none                                                                */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Transfer request              */
/*    _ux_utility_thread_sleep              Sleep thread                  */
/*    _ux_utility_semaphore_get             Get semaphore                 */
/*    _ux_utility_memory_allocate           Allocate memory               */
/*    _ux_utility_memory_free               Free memory                   */
/*    _ux_host_stack_role_swap              Swapping role                 */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    ThreadX                                                             */ 
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
/*  02-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used pointer for current    */
/*                                            selected configuration,     */
/*                                            resulting in version 6.1.4  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_stack_hnp_polling_thread_entry(ULONG argument)
{

UINT                        hcd_index;
UX_DEVICE                   *device;
UX_CONFIGURATION            *configuration;
UX_ENDPOINT                 *control_endpoint;
UX_TRANSFER                 *transfer_request;
UX_HCD                      *hcd;
UCHAR                       *otg_status;
ULONG                       port_index;
ULONG                       port_status;
ULONG                       container_index;
UINT                        status;
    
    UX_PARAMETER_NOT_USED(argument);

    /* This thread goes on forever once started.  */
    while(1)
    {

        /* We need to wake every 2 seconds or so.  */
        _ux_utility_thread_sleep(UX_OTG_HNP_THREAD_SLEEP_TIME); 

        /* We need to parse the controller driver table to find all controllers that registered 
               as OTG.  */
        for (hcd_index = 0; hcd_index < _ux_system_host -> ux_system_host_registered_hcd; hcd_index++)
        {

            /* Pickup HCD pointer.  */
            hcd =  &_ux_system_host -> ux_system_host_hcd_array[hcd_index];

            /* Check type of controller.  Is it OTG capable ? Must be operational too. */
            if ((hcd -> ux_hcd_otg_capabilities & UX_HCD_OTG_CAPABLE) &&
                (hcd -> ux_hcd_status == UX_HCD_STATUS_OPERATIONAL))
            {

                /* Yes, we can parse the root hub and see if any devices attached to it.  */
                for (port_index = 0; port_index < hcd -> ux_hcd_nb_root_hubs; port_index++)
                {

                    /* Call HCD for port status.  */
                    port_status =  hcd -> ux_hcd_entry_function(hcd, UX_HCD_GET_PORT_STATUS, (VOID *)((ALIGN_TYPE)port_index));
    
                    /* Check return status.  */
                    if (port_status != UX_PORT_INDEX_UNKNOWN)
                    {
    
                        /* the port_status value is valid and will tell us if there is
                           a device attached\detached on the downstream port and if the port is powered.  */
                        if ((port_status & UX_PS_CCS) && (port_status & UX_PS_PPS))
                        {
             
                             /* There is a device attached to one of the root hub port.  Parse the device
                               to find out which one it is.  */
                            device =  _ux_system_host -> ux_system_host_device_array;    

                            /* Start at the beginning of the list.  */
                            container_index =  0;
                        
                            /* Search the list until the end.  */
                            while (container_index++ < UX_SYSTEM_HOST_MAX_DEVICES_GET())
                            {
                        
                                /* Until we have found a used entry.  */
                                if (device -> ux_device_handle != UX_UNUSED)
                                {
                        
                                    /* Check for the parent device and the port location and the controller.  */
                                    if(UX_DEVICE_PORT_LOCATION_MATCH(device, port_index) &&
                                       UX_DEVICE_HCD_MATCH(device, hcd))
                                    {

                                        /* We have a device on a OTG port. But is it a OTG HNP capable device ?  
                                           We need to parse the configuration until we find the one current. */
                                        configuration = device -> ux_device_current_configuration;

                                        /* Check for OTG HNP support.  */
                                        if (configuration -> ux_configuration_otg_capabilities & UX_OTG_HNP_SUPPORT)
                                        {
                                
                                            /* Allocate memory for the OTG status.  */
                                            otg_status =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, 16);

                                            /* Check for status.  */
                                            if (otg_status == UX_NULL)
                                                return;
                                
                                            /* Retrieve the control endpoint and the transfer request associated with it.  */
                                            control_endpoint =  &device -> ux_device_control_endpoint;
                                            transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

                                            /* Protect the control endpoint semaphore here.  It will be unprotected in the 
                                               transfer request function.  */
                                            status =  _ux_host_semaphore_get(&device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);

                                            /* Perform a GET_STATUS on this device to see if it wants to become the host.  */
                                            /* Create a transfer_request for the SET_CONFIGURATION request. No data for this request.  */
                                            transfer_request -> ux_transfer_request_data_pointer =      otg_status;
                                            transfer_request -> ux_transfer_request_requested_length =  1;
                                            transfer_request -> ux_transfer_request_function =          UX_GET_STATUS;
                                            transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_DEVICE;
                                            transfer_request -> ux_transfer_request_value =             0;
                                            transfer_request -> ux_transfer_request_index =             UX_OTG_STATUS_SELECTOR;
                                        
                                            /* Send request to HCD layer.  */
                                            status =  _ux_host_stack_transfer_request(transfer_request);
                                        
                                            /* Check completion status.  */
                                            if(status == UX_SUCCESS && transfer_request -> ux_transfer_request_actual_length == 1)
                                            {
    
                                                /* We have an answer from the device. Check the HNP flag.  */
                                                if (*otg_status & UX_OTG_HOST_REQUEST_FLAG)
                                                {

                                                    /* The device has requested a Host swap. Initiate the command and perform the
                                                        stopping of the host.  */
                                                    _ux_host_stack_role_swap(device);
                                                }
                                            
                                            }
                                        
                                            /* Free all used resources.  */
                                            _ux_utility_memory_free(otg_status);

                                        }
                                    }
                                }

                                /* Move to the next device entry.  */
                                device++;
                
                            }
                        }
                    }
                }
            }                                                       
        }
    }
}
#endif /* #if defined(UX_OTG_SUPPORT) */
