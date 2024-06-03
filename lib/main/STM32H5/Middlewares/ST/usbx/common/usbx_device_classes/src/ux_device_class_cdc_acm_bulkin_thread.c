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
/**   Device CDC_ACM Class                                                */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"
#include "ux_device_stack.h"


#if !defined(UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE) && !defined(UX_DEVICE_STANDALONE)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_cdc_acm_bulkin_thread              PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is the thread of the cdc_acm bulkin endpoint. The bulk*/
/*    IN endpoint is used when the device wants to write data to be sent  */
/*    to the host.                                                        */
/*                                                                        */
/*    It's for RTOS mode.                                                 */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    cdc_acm_class                             Address of cdc_acm class  */
/*                                                container               */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_transfer_request     Request transfer              */
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
/*                                            verified memset and memcpy  */
/*                                            cases, used UX prefix to    */
/*                                            refer to TX symbols instead */
/*                                            of using them directly,     */
/*                                            resulting in version 6.1    */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added macro to disable      */
/*                                            transmission support,       */
/*                                            resulting in version 6.1.6  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used whole buffer for write,*/
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            added auto ZLP support,     */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_device_class_cdc_acm_bulkin_thread(ULONG cdc_acm_class)
{

UX_SLAVE_CLASS_CDC_ACM          *cdc_acm;
UX_SLAVE_DEVICE                 *device;
UX_SLAVE_ENDPOINT               *endpoint;
UX_SLAVE_INTERFACE              *interface_ptr;
UX_SLAVE_TRANSFER               *transfer_request;
UINT                            status;
ULONG                           actual_flags;
ULONG                           transfer_length;
ULONG                           host_length;
ULONG                           total_length;
ULONG                           sent_length;


    /* Get the cdc_acm instance from this class container.  */
    UX_THREAD_EXTENSION_PTR_GET(cdc_acm, UX_SLAVE_CLASS_CDC_ACM, cdc_acm_class)

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* This is the first time we are activated. We need the interface to the class.  */
    interface_ptr =  cdc_acm -> ux_slave_class_cdc_acm_interface;

    /* Locate the endpoints.  */
    endpoint =  interface_ptr -> ux_slave_interface_first_endpoint;

    /* Check the endpoint direction, if IN we have the correct endpoint.  */
    if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) != UX_ENDPOINT_IN)
    {

        /* So the next endpoint has to be the IN endpoint.  */
        endpoint =  endpoint -> ux_slave_endpoint_next_endpoint;
    }

    /* This thread runs forever but can be suspended or resumed.  */
    while(1)
    {

        /* Get the transfer request for the bulk IN pipe.  */
        transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;

        /* As long as the device is in the CONFIGURED state.  */
        while (device -> ux_slave_device_state == UX_DEVICE_CONFIGURED)
        {

            /* Wait until we have a event sent by the application. */
            status =  _ux_utility_event_flags_get(&cdc_acm -> ux_slave_class_cdc_acm_event_flags_group, UX_DEVICE_CLASS_CDC_ACM_WRITE_EVENT,
                                                                                            UX_OR_CLEAR, &actual_flags, UX_WAIT_FOREVER);

            /* Check the completion code. */
            if (status == UX_SUCCESS)
            {

                /* Get the length of the entire buffer to send.  */
                total_length = cdc_acm -> ux_slave_class_cdc_acm_callback_total_length;

                /* Duplicate the data pointer to keep a current pointer.  */
                cdc_acm -> ux_slave_class_cdc_acm_callback_current_data_pointer = cdc_acm -> ux_slave_class_cdc_acm_callback_data_pointer;

                /* Reset sent length.  */
                sent_length = 0;

                /* Special ZLP case.  */
                if (total_length == 0)

                    /* Send the zlp to the host.  */
                    status =  _ux_device_stack_transfer_request(transfer_request, 0, 0);

                else
                {

                    /* We should send the total length.  But we may have a case of ZLP. */
                    host_length = UX_SLAVE_REQUEST_DATA_MAX_LENGTH;
                    while (total_length)
                    {

                        /* Check the length remaining to send.  */
                        if (total_length > UX_SLAVE_REQUEST_DATA_MAX_LENGTH)

                            /* We can't fit all the length.  */
                            transfer_length = UX_SLAVE_REQUEST_DATA_MAX_LENGTH;

                        else
                        {

                            /* We can send everything.  */
                            transfer_length =  total_length;

#if !defined(UX_DEVICE_CLASS_CDC_ACM_WRITE_AUTO_ZLP)

                            /* Assume expected length matches length to send.  */
                            host_length = total_length;
#else

                            /* Assume expected more to let stack append ZLP if needed.  */
                            host_length = UX_SLAVE_REQUEST_DATA_MAX_LENGTH + 1;
#endif
                        }

                        /* Copy the payload locally.  */
                        _ux_utility_memory_copy (transfer_request -> ux_slave_transfer_request_data_pointer,
                                                cdc_acm -> ux_slave_class_cdc_acm_callback_current_data_pointer,
                                                transfer_length); /* Use case of memcpy is verified. */

                        /* Send the acm payload to the host.  */
                        status =  _ux_device_stack_transfer_request(transfer_request, transfer_length, host_length);

                        /* Check the status.  */
                        if (status != UX_SUCCESS)
                        {

                            /* Reset total_length as we need to get out of loop. */
                            total_length = 0;
                        }
                        else
                        {

                            /* Update sent length.  */
                            sent_length += transfer_length;

                            /* Decrement length to be sent.  */
                            total_length -= transfer_length;

                            /* And update the current pointer.  */
                            cdc_acm -> ux_slave_class_cdc_acm_callback_current_data_pointer += transfer_length;
                        }
                    }
                }

                /* Schedule of transmission was completed.  */
                cdc_acm -> ux_slave_class_cdc_acm_scheduled_write = UX_FALSE;

                /* We get here when the entire user data payload has been sent or if there is an error. */
                /* If there is a callback defined by the application, send the transaction event to it.  */
                if (cdc_acm -> ux_device_class_cdc_acm_write_callback != UX_NULL)

                    /* Callback exists. */
                    cdc_acm -> ux_device_class_cdc_acm_write_callback(cdc_acm, status, sent_length);

                /* Now we return to wait for an event from the application or the user to stop the transmission.  */
            }
            else
            {
                break;
            }
        }

    /* We need to suspend ourselves. We will be resumed by the device enumeration module or when a change of alternate setting happens.  */
    _ux_device_thread_suspend(&cdc_acm -> ux_slave_class_cdc_acm_bulkin_thread);
    }
}
#endif
