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
/**   Device CDC ACM Class                                                */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"
#include "ux_device_stack.h"

#if defined(UX_DEVICE_STANDALONE)


#ifndef UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE
static inline VOID _ux_device_class_cdc_acm_transmission_read_run(UX_SLAVE_CLASS_CDC_ACM *cdc_acm);
static inline VOID _ux_device_class_cdc_acm_transmission_write_run(UX_SLAVE_CLASS_CDC_ACM *cdc_acm);
#endif


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_cdc_acm_tasks_run                  PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function runs tasks of the CDC ACM class.                      */
/*    E.g., Transmission state machine.                                   */
/*                                                                        */
/*    It's for standalone mode.                                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    instance                              Address of CDC ACM instance   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    UX_STATE_RESET                        Tasks suspended               */
/*    UX_STATE_IDLE                         Activated but no task ran     */
/*    (others > UX_STATE_IDLE)              Tasks running                 */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_transfer_run         Run transfer state machine    */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Device Stack                                                        */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            supported write auto ZLP,   */
/*                                            resulting in version 6.1.12 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed compile warnings,     */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_cdc_acm_tasks_run(VOID *instance)
{

UINT                                status = UX_STATE_IDLE;

#ifndef UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE
UX_SLAVE_DEVICE                    *device;
UX_SLAVE_CLASS_CDC_ACM             *cdc_acm;

    /* Get CDC ACM instance.  */
    cdc_acm = (UX_SLAVE_CLASS_CDC_ACM*) instance;

    /* Check if transmission is started.  */
    if (!cdc_acm -> ux_slave_class_cdc_acm_transmission_status)
        return(status);

    /* Check if device state is good.  */
    device =  &_ux_system_slave -> ux_system_slave_device;
    if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
    {
        cdc_acm -> ux_slave_class_cdc_acm_transmission_status = UX_FALSE;
        cdc_acm -> ux_device_class_cdc_acm_read_state = UX_STATE_RESET;
        cdc_acm -> ux_device_class_cdc_acm_read_status = UX_CONFIGURATION_HANDLE_UNKNOWN;
        cdc_acm -> ux_device_class_cdc_acm_write_state = UX_STATE_RESET;
        cdc_acm -> ux_device_class_cdc_acm_write_status = UX_CONFIGURATION_HANDLE_UNKNOWN;
        return(status);
    }

    /* Run state machine for read.  */
    _ux_device_class_cdc_acm_transmission_read_run(cdc_acm);

    /* Run state machine for write.  */
    _ux_device_class_cdc_acm_transmission_write_run(cdc_acm);

    /* There must be something running.  */
    status = UX_STATE_WAIT;
#endif

    return(status);
}

#ifndef UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE
static inline VOID _ux_device_class_cdc_acm_transmission_read_run(UX_SLAVE_CLASS_CDC_ACM *cdc_acm)
{

UINT                        status;
UX_SLAVE_ENDPOINT           *endpoint;
UX_SLAVE_INTERFACE          *interface_ptr;
UX_SLAVE_TRANSFER           *transfer_request;
ULONG                       max_transfer_length;


    /* Get the interface to the class.  */
    interface_ptr =  cdc_acm -> ux_slave_class_cdc_acm_interface;

    /* Locate the endpoints.  */
    endpoint =  interface_ptr -> ux_slave_interface_first_endpoint;

    /* Check the endpoint direction, if OUT we have the correct endpoint.  */
    if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) != UX_ENDPOINT_OUT)
    {

        /* So the next endpoint has to be the OUT endpoint.  */
        endpoint =  endpoint -> ux_slave_endpoint_next_endpoint;
    }

    /* All CDC reading  are on the endpoint OUT, from the host.  */
    transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;

    /* Handle state cases.  */
    switch(cdc_acm -> ux_device_class_cdc_acm_read_state)
    {
    case UX_STATE_RESET:
        cdc_acm -> ux_device_class_cdc_acm_read_state = UX_DEVICE_CLASS_CDC_ACM_READ_WAIT;
        cdc_acm -> ux_device_class_cdc_acm_read_status = UX_TRANSFER_NO_ANSWER;

        /* Use wMaxPacketSize for faster action, UX_SLAVE_REQUEST_DATA_MAX_LENGTH for better performance.  */
        max_transfer_length = endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize;
        cdc_acm -> ux_device_class_cdc_acm_read_transfer_length = max_transfer_length;

        /* Next state.  */
        cdc_acm -> ux_device_class_cdc_acm_read_state = UX_DEVICE_CLASS_CDC_ACM_READ_WAIT;
        UX_SLAVE_TRANSFER_STATE_RESET(transfer_request);

        /* Fall through.  */
    case UX_DEVICE_CLASS_CDC_ACM_READ_WAIT:

        /* Run the transfer state machine.  */
        status = _ux_device_stack_transfer_run(transfer_request,
                    cdc_acm -> ux_device_class_cdc_acm_read_transfer_length,
                    cdc_acm -> ux_device_class_cdc_acm_read_transfer_length);

        /* Error case.  */
        if (status <= UX_STATE_NEXT)
        {

            /* Do it again.  */
            cdc_acm -> ux_device_class_cdc_acm_read_state = UX_STATE_RESET;

            /* Last transfer status.  */
            cdc_acm -> ux_device_class_cdc_acm_read_status =
                transfer_request -> ux_slave_transfer_request_completion_code;

            if (cdc_acm -> ux_device_class_cdc_acm_read_callback)
            {
                cdc_acm -> ux_device_class_cdc_acm_read_callback(cdc_acm,
                        transfer_request -> ux_slave_transfer_request_completion_code,
                        transfer_request -> ux_slave_transfer_request_data_pointer,
                        transfer_request -> ux_slave_transfer_request_actual_length);
            }
            return;
        }

        /* Keep waiting.  */
        return;

    default: /* Error.  */
        cdc_acm -> ux_device_class_cdc_acm_read_state = UX_STATE_RESET;
        cdc_acm -> ux_device_class_cdc_acm_read_status = UX_INVALID_STATE;
        break;
    }
}
static inline VOID _ux_device_class_cdc_acm_transmission_write_run(UX_SLAVE_CLASS_CDC_ACM *cdc_acm)
{

UINT                        status;
UX_SLAVE_ENDPOINT           *endpoint;
UX_SLAVE_INTERFACE          *interface_ptr;
UX_SLAVE_TRANSFER           *transfer_request;
UINT                        zlp = UX_FALSE;
ULONG                       requested_length;


    /* If write not started, return.  */
    if (!cdc_acm -> ux_slave_class_cdc_acm_scheduled_write)
        return;

    /* We need the interface to the class.  */
    interface_ptr =  cdc_acm -> ux_slave_class_cdc_acm_interface;

    /* Locate the endpoints.  */
    endpoint =  interface_ptr -> ux_slave_interface_first_endpoint;

    /* Check the endpoint direction, if IN we have the correct endpoint.  */
    if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) != UX_ENDPOINT_IN)
    {

        /* So the next endpoint has to be the IN endpoint.  */
        endpoint =  endpoint -> ux_slave_endpoint_next_endpoint;
    }

    /* We are writing to the IN endpoint.  */
    transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;

    /* Handle state cases.  */
    switch(cdc_acm -> ux_device_class_cdc_acm_write_state)
    {
    case UX_STATE_RESET:
        cdc_acm -> ux_device_class_cdc_acm_write_state = UX_DEVICE_CLASS_CDC_ACM_WRITE_START;
        cdc_acm -> ux_device_class_cdc_acm_write_status = UX_TRANSFER_NO_ANSWER;
        cdc_acm -> ux_device_class_cdc_acm_write_actual_length = 0;
        cdc_acm -> ux_device_class_cdc_acm_write_host_length = UX_SLAVE_REQUEST_DATA_MAX_LENGTH;
        if (cdc_acm -> ux_device_class_cdc_acm_write_requested_length == 0)
            zlp = UX_TRUE;

        /* Fall through.  */
    case UX_DEVICE_CLASS_CDC_ACM_WRITE_START:

        /* Get remaining requested length.  */
        requested_length = cdc_acm -> ux_device_class_cdc_acm_write_requested_length -
                        cdc_acm -> ux_device_class_cdc_acm_write_actual_length;

        /* There is no remaining, we are done.  */
        if (requested_length == 0 && !zlp)
        {
            cdc_acm -> ux_device_class_cdc_acm_write_state = UX_STATE_RESET;
            cdc_acm -> ux_device_class_cdc_acm_write_status = UX_SUCCESS;
            cdc_acm -> ux_slave_class_cdc_acm_scheduled_write = UX_FALSE;
            if (cdc_acm -> ux_device_class_cdc_acm_write_callback)
            {
                cdc_acm -> ux_device_class_cdc_acm_write_callback(cdc_acm,
                        UX_SUCCESS, 0);
            }
            return;
        }

        /* Check if we have enough in the local buffer.  */
        if (requested_length > UX_SLAVE_REQUEST_DATA_MAX_LENGTH)

            /* We have too much to transfer.  */
            cdc_acm -> ux_device_class_cdc_acm_write_transfer_length =
                                            UX_SLAVE_REQUEST_DATA_MAX_LENGTH;

        else
        {

            /* We can proceed with the demanded length.  */
            cdc_acm -> ux_device_class_cdc_acm_write_transfer_length = requested_length;

#if !defined(UX_DEVICE_CLASS_CDC_ACM_WRITE_AUTO_ZLP)

            /* Assume expected length matches length to send.  */
            cdc_acm -> ux_device_class_cdc_acm_write_host_length = requested_length;
#else

            /* Assume expected more to let stack append ZLP if needed.  */
            cdc_acm -> ux_device_class_cdc_acm_write_host_length = UX_SLAVE_REQUEST_DATA_MAX_LENGTH + 1;
#endif
        }

        /* On a out, we copy the buffer to the caller. Not very efficient but it makes the API
           easier.  */
        _ux_utility_memory_copy(transfer_request -> ux_slave_transfer_request_data_pointer, 
                            cdc_acm -> ux_device_class_cdc_acm_write_buffer,
                            cdc_acm -> ux_device_class_cdc_acm_write_transfer_length); /* Use case of memcpy is verified. */

        /* Next state.  */
        cdc_acm -> ux_device_class_cdc_acm_write_state = UX_DEVICE_CLASS_CDC_ACM_WRITE_WAIT;
        UX_SLAVE_TRANSFER_STATE_RESET(transfer_request);

        /* Fall through.  */
    case UX_DEVICE_CLASS_CDC_ACM_WRITE_WAIT:

        /* Send the request to the device controller.  */
        status =  _ux_device_stack_transfer_run(transfer_request,
                            cdc_acm -> ux_device_class_cdc_acm_write_transfer_length,
                            cdc_acm -> ux_device_class_cdc_acm_write_host_length);

        /* Error case.  */
        if (status < UX_STATE_NEXT)
        {
            cdc_acm -> ux_device_class_cdc_acm_write_state = UX_STATE_RESET;
            cdc_acm -> ux_device_class_cdc_acm_write_status =
                transfer_request -> ux_slave_transfer_request_completion_code;
            cdc_acm -> ux_slave_class_cdc_acm_scheduled_write = UX_FALSE;
            if (cdc_acm -> ux_device_class_cdc_acm_write_callback)
            {
                cdc_acm -> ux_device_class_cdc_acm_write_callback(cdc_acm,
                        cdc_acm -> ux_device_class_cdc_acm_write_status,
                        cdc_acm -> ux_device_class_cdc_acm_write_actual_length);
            }
            return;
        }

        /* Success case.  */
        if (status == UX_STATE_NEXT)
        {

            /* Next buffer address.  */
            cdc_acm -> ux_device_class_cdc_acm_write_buffer +=
                    transfer_request -> ux_slave_transfer_request_actual_length;

            /* Set the length actually received. */
            cdc_acm -> ux_device_class_cdc_acm_write_actual_length +=
                    transfer_request -> ux_slave_transfer_request_actual_length;

            /* Last transfer status.  */
            cdc_acm -> ux_device_class_cdc_acm_write_status =
                transfer_request -> ux_slave_transfer_request_completion_code;

            /* Check ZLP case.  */
            if (cdc_acm -> ux_device_class_cdc_acm_write_requested_length == 0)
            {
                cdc_acm -> ux_device_class_cdc_acm_write_state = UX_STATE_RESET;
                cdc_acm -> ux_slave_class_cdc_acm_scheduled_write = UX_FALSE;
                if (cdc_acm -> ux_device_class_cdc_acm_write_callback)
                {
                    cdc_acm -> ux_device_class_cdc_acm_write_callback(cdc_acm,
                            cdc_acm -> ux_device_class_cdc_acm_write_status,
                            cdc_acm -> ux_device_class_cdc_acm_write_actual_length);
                }
                return;
            }

            /* Next state.  */
            cdc_acm -> ux_device_class_cdc_acm_write_state = UX_DEVICE_CLASS_CDC_ACM_WRITE_START;
        }

        /* Keep waiting.  */
        return;

    default: /* Error.  */
        cdc_acm -> ux_device_class_cdc_acm_write_state = UX_STATE_RESET;
        break;
    }
}

#endif /* !defined(UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE)  */

#endif /* defined(UX_DEVICE_STANDALONE)  */
