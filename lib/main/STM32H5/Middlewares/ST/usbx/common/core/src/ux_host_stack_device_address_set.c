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
/*    _ux_host_stack_device_address_set                   PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function sets the device address to the new device.            */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    device                                Pointer to device             */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_delay_ms                  Thread sleep                  */ 
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
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_device_address_set(UX_DEVICE *device)
{


UINT            status = UX_ERROR;
UX_TRANSFER     *transfer_request;
UX_ENDPOINT     *control_endpoint;
USHORT          device_address;
#if UX_MAX_DEVICES > 1
UX_HCD          *hcd;
UINT            address_byte_index;
UINT            address_bit_index;
UCHAR           device_address_byte;
#endif

    /* Retrieve the pointer to the control endpoint.  */
    control_endpoint =  &device -> ux_device_control_endpoint;

    /* Retrieve the transfer request pointer.  */
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Initialize device address to 1.  */
    device_address =  1;

#if UX_MAX_DEVICES > 1

    /* We need the HCD pointer as well.  */
    hcd = UX_DEVICE_HCD_GET(device);

    /* Calculate the new address of this device. We start with address 1.  */
    for (address_byte_index = 0; address_byte_index < 16; address_byte_index++)
    {

        /* Get the address mask byte.  */
        device_address_byte =  hcd -> ux_hcd_address[address_byte_index];

        /* Scan each bit for an empty spot.  */
        for (address_bit_index = 0; address_bit_index < 8; address_bit_index++)
        {

            if ((device_address_byte & (1 << address_bit_index)) == 0)
            {

                /* We have found an empty spot. Reserve this address.  */
                device_address_byte = (UCHAR)((UCHAR)device_address_byte | (UCHAR)(1 << address_bit_index));

                /* Store the address mask byte.  */
                hcd -> ux_hcd_address[address_byte_index] =  device_address_byte;

                /* OK, apply address.  */
                status = UX_SUCCESS;
                break;
            }

            /* This address was already taken, increment to the next address.  */
            device_address++;
        }

        /* If address found, break the loop.  */
        if (status == UX_SUCCESS)
        {
            break;
        }
    }
    if (status == UX_ERROR)

        /* We should never get here!  */
        return(UX_ERROR);
#endif

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_DEVICE_ADDRESS_SET, device, device_address, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    /* Create a transfer request for the SET_ADDRESS request.  */
    transfer_request -> ux_transfer_request_data_pointer =      UX_NULL;
    transfer_request -> ux_transfer_request_requested_length =  0;
    transfer_request -> ux_transfer_request_function =          UX_SET_ADDRESS;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_OUT | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value =             device_address;
    transfer_request -> ux_transfer_request_index =             0;

#if defined(UX_HOST_STANDALONE)
    device -> ux_device_enum_trans = transfer_request;
    status = UX_SUCCESS;
    return(status);
#else

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Now, this address will be the one used in future transfers.  The transfer may have failed and therefore
        all the device resources including the new address will be free.*/
    device -> ux_device_address =  (ULONG) device_address;

    /* Check completion status.  */
    if (status == UX_SUCCESS)
    {

        /* Mark the device as ADDRESSED now.  */
        device -> ux_device_state = UX_DEVICE_ADDRESSED;

        /* Some devices need some time to accept this address.  */
        _ux_utility_delay_ms(UX_DEVICE_ADDRESS_SET_WAIT);

        /* Return successful status.  */
        return(status);
    }
    else
    {

        /* We have an error at the first device transaction. This is mostly
            due to the device having failed on the reset after power up.
            we will try again either at the root hub or regular hub. */   
        return(status);
    }
#endif
}
