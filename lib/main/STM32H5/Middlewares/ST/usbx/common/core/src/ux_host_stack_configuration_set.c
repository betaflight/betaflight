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


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_stack_configuration_set                    PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function performs a setting of a device configuration.         */ 
/*                                                                        */
/*    In RTOS mode, this function is blocking.                            */
/*    If the host is OTG capable and the device has an OTG descriptor     */ 
/*    that supports HNP we perform a SET_FEATURE with b_hnp_support.      */ 
/*                                                                        */ 
/*    In standalone mode, when device enumeration is in progress, this    */
/*    function is non-blocking, it prepares transfer for enum step of     */
/*    SET_CONFIGURE request. Otherwise it blocks until transfer request   */
/*    done.                                                               */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    configuration                         Pointer to configuration      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
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
/*  02-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used pointer for current    */
/*                                            selected configuration,     */
/*                                            resulting in version 6.1.4  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            set device power source,    */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_configuration_set(UX_CONFIGURATION *configuration)
{

UX_DEVICE       *device;
UX_TRANSFER     *transfer_request;
UINT            status;
UX_ENDPOINT     *control_endpoint;
#ifdef UX_OTG_SUPPORT
UX_HCD          *hcd;
#endif


    /* A configuration is selected. Retrieve the pointer to the control endpoint 
       and its transfer request.  */
    device =            configuration -> ux_configuration_device;
    control_endpoint =  &device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

#ifdef UX_OTG_SUPPORT
    /* Check if the configuration has an OTG device with HNP feature.  */
    if (configuration -> ux_configuration_otg_capabilities & UX_OTG_HNP_SUPPORT)
    {

        /* For HNP to work the device has to be connected directly to the Root Hub and not
           a down stream hub.  If the parent is NULL, the device is on the root hub.  */
        if (UX_DEVICE_PARENT_IS_ROOTHUB(device))
        {

            /* With the device we have the pointer to the HCD.  */
            hcd = UX_DEVICE_HCD_GET(device);
    
            /* Check the HCD to ensure we have an OTG host controller.  */
            if (hcd -> ux_hcd_otg_capabilities & UX_HCD_OTG_CAPABLE)
            {
    
                /* The Host controller is OTG aware.  Perform a SET_FEATURE with b_hnp_support.  */
                transfer_request -> ux_transfer_request_data_pointer =      UX_NULL;
                transfer_request -> ux_transfer_request_requested_length =  0;
                transfer_request -> ux_transfer_request_function =          UX_SET_FEATURE;
                transfer_request -> ux_transfer_request_type =              UX_REQUEST_OUT| UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_DEVICE;
                transfer_request -> ux_transfer_request_value =             UX_OTG_FEATURE_A_HNP_SUPPORT;
                transfer_request -> ux_transfer_request_index =             0;
            
                /* Send request to HCD layer.  */
                status =  _ux_host_stack_transfer_request(transfer_request);
            
                /* If the device fails this command we turn off its OTG capabilities.  */
                if (status != UX_SUCCESS)

                    /* Reset the OTG capabilities of the device.  */
                    configuration -> ux_configuration_otg_capabilities = 0;

            }
        }
    }
#endif

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_CONFIGURATION_SET, configuration, 0, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    /* Create a transfer_request for the SET_CONFIGURATION request. No data for this request.  */
    transfer_request -> ux_transfer_request_requested_length =  0;
    transfer_request -> ux_transfer_request_function =          UX_SET_CONFIGURATION;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_OUT | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value =             (USHORT) configuration -> ux_configuration_descriptor.bConfigurationValue;
    transfer_request -> ux_transfer_request_index =             0;

#if defined(UX_HOST_STANDALONE)
    if (device -> ux_device_flags &= UX_DEVICE_FLAG_ENUM)
    {

        /* Special case for enumeration process, non-blocking.  */
        device -> ux_device_enum_trans = transfer_request;
        status = UX_SUCCESS;
        return(status);
    }

    /* Tend to be blocking after enumeration done.  */
#endif

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check completion status.  */
    if(status == UX_SUCCESS)
    {

        /* Change the device state to configured.  */
        device -> ux_device_state =  UX_DEVICE_CONFIGURED;
    
        /* Store the new configuration value in the device container.  */
        device -> ux_device_current_configuration =  configuration;

        /* Save current device power source.  */
        device -> ux_device_power_source = (configuration ->
                                            ux_configuration_descriptor.bmAttributes &
                                            UX_CONFIGURATION_DEVICE_SELF_POWERED) ?
                                UX_DEVICE_SELF_POWERED : UX_DEVICE_BUS_POWERED;
    }

    /* Return status to caller.  */
    return(status);
}
