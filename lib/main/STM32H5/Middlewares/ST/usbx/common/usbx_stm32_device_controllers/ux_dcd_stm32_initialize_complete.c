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
/**   STM32 Controller Driver                                             */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE
#define UX_DCD_STM32_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_system.h"
#include "ux_utility.h"
#include "ux_dcd_stm32.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_dcd_stm32_initialize_complete                   PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function completes the initialization of the USB slave         */
/*    controller for the STM32 chip.                                      */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    (ux_slave_dcd_function)               Process the DCD function      */
/*    _ux_utility_descriptor_parse          Parse descriptor              */
/*    _ux_utility_memory_allocate           Allocate memory               */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    STM32 Controller Driver                                             */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            prefixed UX to MS_TO_TICK,  */
/*                                            used ST HAL library to      */
/*                                            drive the controller,       */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_dcd_stm32_initialize_complete(VOID)
{

UX_SLAVE_DCD            *dcd;
UX_DCD_STM32            *dcd_stm32;
UX_SLAVE_DEVICE         *device;
UCHAR                     *device_framework;
UX_SLAVE_TRANSFER       *transfer_request;


    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the pointer to the STM32 DCD.  */
    dcd_stm32 = (UX_DCD_STM32 *) dcd -> ux_slave_dcd_controller_hardware;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Are we in DFU mode ? If so, check if we are in a Reset mode.  */
    if (_ux_system_slave -> ux_system_slave_device_dfu_state_machine == UX_SYSTEM_DFU_STATE_APP_DETACH)
    {

        /* The device is now in DFU reset mode. Switch to the DFU device framework.  */
        _ux_system_slave -> ux_system_slave_device_framework =  _ux_system_slave -> ux_system_slave_dfu_framework;
        _ux_system_slave -> ux_system_slave_device_framework_length =  _ux_system_slave -> ux_system_slave_dfu_framework_length;

    }
    else
    {

        /* Set State to App Idle. */
        _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_APP_IDLE;

        /* Check the speed and set the correct descriptor.  */
        if (_ux_system_slave -> ux_system_slave_speed ==  UX_FULL_SPEED_DEVICE)
        {

            /* The device is operating at full speed.  */
            _ux_system_slave -> ux_system_slave_device_framework =  _ux_system_slave -> ux_system_slave_device_framework_full_speed;
            _ux_system_slave -> ux_system_slave_device_framework_length =  _ux_system_slave -> ux_system_slave_device_framework_length_full_speed;
        }
        else
        {

            /* The device is operating at high speed.  */
            _ux_system_slave -> ux_system_slave_device_framework =  _ux_system_slave -> ux_system_slave_device_framework_high_speed;
            _ux_system_slave -> ux_system_slave_device_framework_length =  _ux_system_slave -> ux_system_slave_device_framework_length_high_speed;
        }
    }

    /* Get the device framework pointer.  */
    device_framework =  _ux_system_slave -> ux_system_slave_device_framework;

    /* And create the decompressed device descriptor structure.  */
    _ux_utility_descriptor_parse(device_framework,
                                _ux_system_device_descriptor_structure,
                                UX_DEVICE_DESCRIPTOR_ENTRIES,
                                (UCHAR *) &device -> ux_slave_device_descriptor);

    /* Now we create a transfer request to accept the first SETUP packet
       and get the ball running. First get the address of the endpoint
       transfer request container.  */
    transfer_request =  &device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

    /* Set the timeout to be for Control Endpoint.  */
    transfer_request -> ux_slave_transfer_request_timeout =  UX_MS_TO_TICK(UX_CONTROL_TRANSFER_TIMEOUT);

    /* Adjust the current data pointer as well.  */
    transfer_request -> ux_slave_transfer_request_current_data_pointer =
                            transfer_request -> ux_slave_transfer_request_data_pointer;

    /* Update the transfer request endpoint pointer with the default endpoint.  */
    transfer_request -> ux_slave_transfer_request_endpoint =  &device -> ux_slave_device_control_endpoint;

    /* The control endpoint max packet size needs to be filled manually in its descriptor.  */
    transfer_request -> ux_slave_transfer_request_endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize =
                                device -> ux_slave_device_descriptor.bMaxPacketSize0;

    /* On the control endpoint, always expect the maximum.  */
    transfer_request -> ux_slave_transfer_request_requested_length =
                                device -> ux_slave_device_descriptor.bMaxPacketSize0;

    /* Attach the control endpoint to the transfer request.  */
    transfer_request -> ux_slave_transfer_request_endpoint =  &device -> ux_slave_device_control_endpoint;

    /* Create the default control endpoint attached to the device.
       Once this endpoint is enabled, the host can then send a setup packet
       The device controller will receive it and will call the setup function
       module.  */
    dcd -> ux_slave_dcd_function(dcd, UX_DCD_CREATE_ENDPOINT,
                                    (VOID *) &device -> ux_slave_device_control_endpoint);

    /* Open Control OUT endpoint.  */
    HAL_PCD_EP_Flush(dcd_stm32 -> pcd_handle, 0x00U);
    HAL_PCD_EP_Open(dcd_stm32 -> pcd_handle, 0x00U, device -> ux_slave_device_descriptor.bMaxPacketSize0, UX_CONTROL_ENDPOINT);

    /* Open Control IN endpoint.  */
    HAL_PCD_EP_Flush(dcd_stm32 -> pcd_handle, 0x80U);
    HAL_PCD_EP_Open(dcd_stm32 -> pcd_handle, 0x80U, device -> ux_slave_device_descriptor.bMaxPacketSize0, UX_CONTROL_ENDPOINT);

    /* Ensure the control endpoint is properly reset.  */
    device -> ux_slave_device_control_endpoint.ux_slave_endpoint_state = UX_ENDPOINT_RESET;

    /* Mark the phase as SETUP.  */
    transfer_request -> ux_slave_transfer_request_type =  UX_TRANSFER_PHASE_SETUP;

    /* Mark this transfer request as pending.  */
    transfer_request -> ux_slave_transfer_request_status =  UX_TRANSFER_STATUS_PENDING;

    /* Ask for 8 bytes of the SETUP packet.  */
    transfer_request -> ux_slave_transfer_request_requested_length =    UX_SETUP_SIZE;
    transfer_request -> ux_slave_transfer_request_in_transfer_length =  UX_SETUP_SIZE;

    /* Reset the number of bytes sent/received.  */
    transfer_request -> ux_slave_transfer_request_actual_length =  0;

    /* Check the status change callback.  */
    if(_ux_system_slave -> ux_system_slave_change_function != UX_NULL)
    {

        /* Inform the application if a callback function was programmed.  */
        _ux_system_slave -> ux_system_slave_change_function(UX_DEVICE_ATTACHED);
    }

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_STACK_CONNECT, 0, 0, 0, 0, UX_TRACE_DEVICE_STACK_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_REGISTER(UX_TRACE_DEVICE_OBJECT_TYPE_DEVICE, device, 0, 0, 0)

    /* We are now ready for the USB device to accept the first packet when connected.  */
    return(UX_SUCCESS);
}

