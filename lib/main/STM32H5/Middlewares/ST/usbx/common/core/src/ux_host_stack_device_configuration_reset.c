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
/*    _ux_host_stack_device_configuration_reset           PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function resets the configuration of the device to zero.       */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    configuration                          Pointer to configuration     */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_configuration_instance_delete                        */
/*                                           Delete configuration instance*/ 
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
/*  02-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used pointer for current    */
/*                                            selected configuration,     */
/*                                            resulting in version 6.1.4  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed device state support, */
/*                                            reset device power source,  */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            reset shared device config  */
/*                                            descriptor for enum scan,   */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_device_configuration_reset(UX_DEVICE *device)
{

UX_TRANSFER             *transfer_request;
UX_ENDPOINT             *control_endpoint;
UX_CONFIGURATION        *current_configuration;
UINT                    status;
    
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_DEVICE_CONFIGURATION_SELECT, device, 0, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    /* A configuration is selected. Retrieve the pointer to the control endpoint 
       and its transfer request.  */
    control_endpoint =  &device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Check for the state of the device . If the device is already configured, 
       we need to cancel the existing configuration before resetting it.   */
    if (device -> ux_device_state == UX_DEVICE_CONFIGURED)
    {

        /* The device is configured. Get the first configuration pointer.  */
        current_configuration =  device -> ux_device_current_configuration;

        /* Deselect this instance */
        _ux_host_stack_configuration_instance_delete(current_configuration);
    }

    /* No configuration is selected now.  */
    device -> ux_device_current_configuration = UX_NULL;

    /* Packed descriptor are not valid now.  */
    if (device -> ux_device_packed_configuration)
    {
        _ux_utility_memory_free(device -> ux_device_packed_configuration);
        device -> ux_device_packed_configuration = UX_NULL;
        device -> ux_device_packed_configuration_keep_count = 0;
    }

    /* Set state of device to ADDRESSED.  */
    device -> ux_device_state = UX_DEVICE_ADDRESSED;

    /* Reset power source.  */
    device -> ux_device_power_source = UX_DEVICE_BUS_POWERED;

    /* Create a transfer_request for the SET_CONFIGURATION request. No data for this request.  */
    transfer_request -> ux_transfer_request_requested_length =  0;
    transfer_request -> ux_transfer_request_function =          UX_SET_CONFIGURATION;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_OUT | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value =             0;
    transfer_request -> ux_transfer_request_index =             0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Return status.  */
    return(status);
}

