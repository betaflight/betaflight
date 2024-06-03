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
/*    _ux_device_class_cdc_acm_bulkout_thread             PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is the thread of the cdc_acm bulk out endpoint. It    */
/*    is waiting for the host to send data on the bulk out endpoint to    */
/*    the device.                                                         */
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
/*                                            resulting in version 6.1    */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added macro to disable      */
/*                                            transmission support,       */
/*                                            resulting in version 6.1.6  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_device_class_cdc_acm_bulkout_thread(ULONG cdc_acm_class)
{

UX_SLAVE_CLASS_CDC_ACM          *cdc_acm;
UX_SLAVE_DEVICE                 *device;
UX_SLAVE_ENDPOINT               *endpoint;
UX_SLAVE_INTERFACE              *interface_ptr;
UX_SLAVE_TRANSFER               *transfer_request;
UINT                            status;

    /* Cast properly the cdc_acm instance.  */
    UX_THREAD_EXTENSION_PTR_GET(cdc_acm, UX_SLAVE_CLASS_CDC_ACM, cdc_acm_class)

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* This is the first time we are activated. We need the interface to the class.  */
    interface_ptr =  cdc_acm -> ux_slave_class_cdc_acm_interface;

    /* Locate the endpoints.  */
    endpoint =  interface_ptr -> ux_slave_interface_first_endpoint;

    /* Check the endpoint direction, if OUT we have the correct endpoint.  */
    if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) != UX_ENDPOINT_OUT)
    {

        /* So the next endpoint has to be the OUT endpoint.  */
        endpoint =  endpoint -> ux_slave_endpoint_next_endpoint;
    }

    /* This thread runs forever but can be suspended or resumed by the user application.  */
    while(1)
    {

        /* Select the transfer request associated with BULK OUT endpoint.   */
        transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;

        /* As long as the device is in the CONFIGURED state.  */
        while (device -> ux_slave_device_state == UX_DEVICE_CONFIGURED)
        {

            /* Send the request to the device controller.  */
            status =  _ux_device_stack_transfer_request(transfer_request, endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize,
                                                                endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize);

            /* Check the completion code. */
            if (status == UX_SUCCESS)
            {

                /* Check the state of the transfer.  If there is an error, we do not proceed with this report. */
                if (transfer_request -> ux_slave_transfer_request_completion_code == UX_SUCCESS)
                {

                    /* If there is a callback defined by the application, send the transaction event to it.  */
                    if (cdc_acm -> ux_device_class_cdc_acm_read_callback != UX_NULL)

                        /* Callback exists. */
                        cdc_acm -> ux_device_class_cdc_acm_read_callback(cdc_acm, UX_SUCCESS, transfer_request -> ux_slave_transfer_request_data_pointer,
                                                                                    transfer_request -> ux_slave_transfer_request_actual_length);

                }
                else
                {

                    /* We have an error. If there is a callback defined by the application, send the transaction event to it.  */
                    if (cdc_acm -> ux_device_class_cdc_acm_read_callback != UX_NULL)

                        /* Callback exists. */
                        cdc_acm -> ux_device_class_cdc_acm_read_callback(cdc_acm, status, UX_NULL, 0);

                }
            }
        }

    /* We need to suspend ourselves. We will be resumed by the application if needed.  */
    _ux_device_thread_suspend(&cdc_acm -> ux_slave_class_cdc_acm_bulkout_thread);
    }
}
#endif
