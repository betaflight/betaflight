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
/**   Device Stack                                                        */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_stack_transfer_request                   PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function performs a USB transaction. On entry the              */
/*    transfer request gives the endpoint pipe selected for this          */
/*    transaction and the parameters associated with the transfer         */
/*    (data payload, length of transaction).                              */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    transfer_request                      Pointer to transfer request   */
/*    slave_length                          Length returned by host       */
/*    host_length                           Length asked by host          */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    (ux_slave_dcd_function)               Slave DCD dispatch function   */ 
/*    _ux_utility_delay_ms                  Delay ms                      */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
/*    Device Stack                                                        */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used UX prefix to refer to  */
/*                                            TX symbols instead of using */
/*                                            them directly,              */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_stack_transfer_request(UX_SLAVE_TRANSFER *transfer_request, 
                                            ULONG slave_length, 
                                            ULONG host_length)
{
#if defined(UX_DEVICE_STANDALONE)
UINT            status;

    /* Start a transfer request without waiting it end.  */
    UX_SLAVE_TRANSFER_STATE_RESET(transfer_request);
    status = _ux_device_stack_transfer_run(transfer_request, slave_length, host_length);
    if (status == UX_STATE_LOCK)
        return(UX_BUSY);
    if (status < UX_STATE_NEXT)
        return(transfer_request -> ux_slave_transfer_request_completion_code);

    /* Started/done, things will be done in BG  */
    return(UX_SUCCESS);
#else
UX_INTERRUPT_SAVE_AREA

UX_SLAVE_DCD            *dcd;
UINT                    status;
UX_SLAVE_ENDPOINT       *endpoint;
ULONG                   device_state;


    /* Do we have to skip this transfer?  */
    if (transfer_request -> ux_slave_transfer_request_status_phase_ignore == UX_TRUE)
        return(UX_SUCCESS);

    /* Disable interrupts to prevent the disconnection ISR from preempting us
       while we check the device state and set the transfer status.  */
    UX_DISABLE

    /* Get the device state.  */
    device_state =  _ux_system_slave -> ux_system_slave_device.ux_slave_device_state;

    /* We can only transfer when the device is ATTACHED, ADDRESSED OR CONFIGURED.  */
    if ((device_state == UX_DEVICE_ATTACHED) || (device_state == UX_DEVICE_ADDRESSED)
            || (device_state == UX_DEVICE_CONFIGURED))

        /* Set the transfer to pending.  */
        transfer_request -> ux_slave_transfer_request_status =  UX_TRANSFER_STATUS_PENDING; 

    else
    {

        /* The device is in an invalid state. Restore interrupts and return error.  */
        UX_RESTORE
        return(UX_TRANSFER_NOT_READY);
    }

    /* Restore interrupts.  */
    UX_RESTORE
                    
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_STACK_TRANSFER_REQUEST, transfer_request, 0, 0, 0, UX_TRACE_DEVICE_STACK_EVENTS, 0, 0)

    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the endpoint associated with this transaction.  */
    endpoint =  transfer_request -> ux_slave_transfer_request_endpoint;
    
    /* If the endpoint is non Control, check the endpoint direction and set the data phase direction.  */
    if ((endpoint -> ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) != UX_CONTROL_ENDPOINT)
    {

        /* Check if the endpoint is STALLED. In this case, we must refuse the transaction until the endpoint
           has been reset by the host.  */
        while (endpoint -> ux_slave_endpoint_state == UX_ENDPOINT_HALTED)

            /* Wait for 100ms for endpoint to be reset by a CLEAR_FEATURE command.  */
            _ux_utility_delay_ms(100);

        /* Isolate the direction from the endpoint address.  */
        if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_IN)
            transfer_request -> ux_slave_transfer_request_phase =  UX_TRANSFER_PHASE_DATA_OUT;
        else    
            transfer_request -> ux_slave_transfer_request_phase =  UX_TRANSFER_PHASE_DATA_IN;
    }    

    /* See if we need to force a zero length packet at the end of the transfer. 
       This happens on a DATA IN and when the host requested length is not met
       and the last packet is on a boundary. If slave_length is zero, then it is 
       a explicit ZLP request, no need to force ZLP.  */
    if ((transfer_request -> ux_slave_transfer_request_phase ==  UX_TRANSFER_PHASE_DATA_OUT) &&
        (slave_length != 0) && (host_length != slave_length) && 
        (slave_length % endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize) == 0)
    {

        /* If so force Zero Length Packet.  */
        transfer_request -> ux_slave_transfer_request_force_zlp =  UX_TRUE;
    }
    else
    {

        /* Condition is not met, do not force a Zero Length Packet.  */
        transfer_request -> ux_slave_transfer_request_force_zlp =  UX_FALSE;
    }

    /* Reset the number of bytes sent/received.  */
    transfer_request -> ux_slave_transfer_request_actual_length =  0;

    /* Determine how many bytes to send in this transaction.  We keep track of the original
        length and have a working length.  */
    transfer_request -> ux_slave_transfer_request_requested_length =    slave_length;
    transfer_request -> ux_slave_transfer_request_in_transfer_length =  slave_length;

    /* Save the buffer pointer.  */
    transfer_request -> ux_slave_transfer_request_current_data_pointer =  
                            transfer_request -> ux_slave_transfer_request_data_pointer;

    /* Call the DCD driver transfer function.   */
    status =  dcd -> ux_slave_dcd_function(dcd, UX_DCD_TRANSFER_REQUEST, transfer_request);

    /* And return the status.  */
    return(status);

#endif
}

