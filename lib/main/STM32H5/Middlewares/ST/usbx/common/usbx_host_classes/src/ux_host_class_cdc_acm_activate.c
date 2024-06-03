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
/**   CDC ACM Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_cdc_acm.h"
#include "ux_host_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_cdc_acm_activate                     PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function creates the ACM instance, configure the device ...    */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    command                                ACM  class command pointer   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_class_cdc_acm_configure        Configure cdc_acm class     */
/*    _ux_host_class_cdc_acm_endpoints_get    Get endpoints of cdc_acm    */
/*    _ux_host_class_cdc_acm_ioctl            IOCTL function for ACM      */
/*    _ux_host_stack_class_instance_destroy   Destroy the class instance  */
/*    _ux_host_stack_endpoint_transfer_abort  Abort transfer              */
/*    _ux_utility_memory_allocate             Allocate memory block       */
/*    _ux_utility_memory_free                 Free memory                 */
/*    _ux_host_semaphore_create               Create cdc_acm semaphore    */
/*    _ux_host_semaphore_delete               Delete semaphore            */
/*    _ux_utility_delay_ms                    Delay                       */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    _ux_host_class_cdc_acm_entry          Entry of cdc_acm class        */
/*    _ux_utility_delay_ms                  Delay ms                      */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            used defined line coding    */
/*                                            instead of magic number,    */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            improved error handling,    */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_cdc_acm_activate(UX_HOST_CLASS_COMMAND *command)
{

UX_INTERFACE                        *interface_ptr;
UX_HOST_CLASS_CDC_ACM               *cdc_acm;
UINT                                status;
#if defined(UX_HOST_STANDALONE)
UX_HOST_CLASS                       *cdc_acm_class;
UX_HOST_CLASS_CDC_ACM               *cdc_acm_inst;
UX_ENDPOINT                         *control_endpoint;
UX_TRANSFER                         *transfer_request;
ULONG                               descriptors_length;
#else
UX_HOST_CLASS_CDC_ACM_LINE_CODING   line_coding;
UX_HOST_CLASS_CDC_ACM_LINE_STATE    line_state;
#endif

    /* The CDC ACM class is always activated by the interface descriptor and not the
       device descriptor.  */
    interface_ptr =  (UX_INTERFACE *) command -> ux_host_class_command_container;

    /* Obtain memory for this class instance.  */
    cdc_acm =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_CACHE_SAFE_MEMORY, sizeof(UX_HOST_CLASS_CDC_ACM));

    /* Instance creation fail. */
    if (cdc_acm == UX_NULL)

        /* Memory allocation fail. */
        return(UX_MEMORY_INSUFFICIENT);

#if !defined(UX_HOST_STANDALONE)

    /* Create the semaphore to protect 2 threads from accessing the same acm instance.  */
    status =  _ux_host_semaphore_create(&cdc_acm -> ux_host_class_cdc_acm_semaphore, "ux_host_class_cdc_acm_semaphore", 1);
    if (status != UX_SUCCESS)
    {

        /* Free instance memory. */
        _ux_utility_memory_free(cdc_acm);

        /* Semaphore creation error. */
        return(UX_SEMAPHORE_ERROR);
    }
#endif

    /* Store the class container into this instance.  */
    cdc_acm -> ux_host_class_cdc_acm_class =  command -> ux_host_class_command_class_ptr;

    /* Store the interface container into the cdc_acm class instance.  */
    cdc_acm -> ux_host_class_cdc_acm_interface =  interface_ptr;

    /* Store the device container into the cdc_acm class instance.  */
    cdc_acm -> ux_host_class_cdc_acm_device =  interface_ptr -> ux_interface_configuration -> ux_configuration_device;

    /* This instance of the device must also be stored in the interface container.  */
    interface_ptr -> ux_interface_class_instance =  (VOID *) cdc_acm;

    /* Create this class instance.  */
    _ux_host_stack_class_instance_create(cdc_acm -> ux_host_class_cdc_acm_class, (VOID *) cdc_acm);

#if defined(UX_HOST_STANDALONE)

    /* Get the cdc_acm endpoint(s). Depending on the interface type, we will need to search for
        Bulk Out and Bulk In endpoints and the optional interrupt endpoint.  */
    status =  _ux_host_class_cdc_acm_endpoints_get(cdc_acm);
    if (status == UX_SUCCESS)
    {

        /* Mark the cdc_acm as mounting.  */
        cdc_acm -> ux_host_class_cdc_acm_state = UX_HOST_CLASS_INSTANCE_MOUNTING;

        /* If we have the Control Class, we process default setup command sequence.  */
        if (interface_ptr -> ux_interface_descriptor.bInterfaceClass == UX_HOST_CLASS_CDC_CONTROL_CLASS)
        {

            /* Get descriptors to see capabilities.  */

            /* Get default control transfer.  */
            control_endpoint = &cdc_acm -> ux_host_class_cdc_acm_device -> ux_device_control_endpoint;
            transfer_request = &control_endpoint -> ux_endpoint_transfer_request;

            /* Allocate memory for the descriptors.  */
            descriptors_length = interface_ptr -> ux_interface_configuration ->
                                        ux_configuration_descriptor.wTotalLength;
            cdc_acm -> ux_host_class_cdc_acm_allocated =
                    _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY,
                                                            descriptors_length);
            if (cdc_acm -> ux_host_class_cdc_acm_allocated != UX_NULL)
            {

                transfer_request -> ux_transfer_request_data_pointer =
                                            cdc_acm -> ux_host_class_cdc_acm_allocated;

                /* Create transfer for GET_DESCRIPTOR.  */
                transfer_request -> ux_transfer_request_requested_length = descriptors_length;
                transfer_request -> ux_transfer_request_function =         UX_GET_DESCRIPTOR;
                transfer_request -> ux_transfer_request_type =             UX_REQUEST_IN | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_DEVICE;
                transfer_request -> ux_transfer_request_value =            UX_CONFIGURATION_DESCRIPTOR_ITEM << 8;
                transfer_request -> ux_transfer_request_index =            0;
                UX_TRANSFER_STATE_RESET(transfer_request);

                /* Set state to wait and next is "next".  */
                cdc_acm -> ux_host_class_cdc_acm_cmd_state = UX_STATE_WAIT;
                cdc_acm -> ux_host_class_cdc_acm_next_state = UX_STATE_NEXT;

                /* ACTIVATE_WAIT will be processed to finish next steps.  */
                return(UX_SUCCESS);
            }
            else
                status = UX_MEMORY_INSUFFICIENT;
        }
        else
        {

            /* We scan CDC ACM instances to find the master instance.  */
            /* Get class.  */
            cdc_acm_class = cdc_acm -> ux_host_class_cdc_acm_class;

            /* Get first instance linked to the class.  */
            cdc_acm_inst = (UX_HOST_CLASS_CDC_ACM *)cdc_acm_class -> ux_host_class_first_instance;

            /* Scan all instances.  */
            while(cdc_acm_inst)
            {

                /* If this data interface is inside the associate list, link it.  */
                if (cdc_acm_inst -> ux_host_class_cdc_acm_interfaces_bitmap &
                    (1ul << interface_ptr -> ux_interface_descriptor.bInterfaceNumber))
                {

                    /* Save control instance and we are done.  */
                    cdc_acm -> ux_host_class_cdc_acm_control = cdc_acm_inst;
                    break;
                }

                /* Next instance.  */
                cdc_acm_inst = cdc_acm_inst -> ux_host_class_cdc_acm_next_instance;
            }

            /* Mark the cdc_acm as live now.  Both interfaces need to be live. */
            cdc_acm -> ux_host_class_cdc_acm_state = UX_HOST_CLASS_INSTANCE_LIVE;

            /* If all is fine and the device is mounted, we may need to inform the application
                if a function has been programmed in the system structure.  */
            if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
            {

                /* Call system change function.  */
                _ux_system_host ->  ux_system_host_change_function(UX_DEVICE_INSERTION, cdc_acm -> ux_host_class_cdc_acm_class, (VOID *) cdc_acm);
            }

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_CDC_ACM_ACTIVATE, cdc_acm, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

            /* If trace is enabled, register this object.  */
            UX_TRACE_OBJECT_REGISTER(UX_TRACE_HOST_OBJECT_TYPE_INTERFACE, cdc_acm, 0, 0, 0)

            /* We are done success. */
            return(UX_SUCCESS);
        }
    }

#else

    /* Configure the cdc_acm.  */
    status =  _ux_host_class_cdc_acm_configure(cdc_acm);

    /* If we are done success, go on to get endpoints.  */
    if (status == UX_SUCCESS)

        /* Get the cdc_acm endpoint(s). Depending on the interface type, we will need to search for
           Bulk Out and Bulk In endpoints and the optional interrupt endpoint.  */
        status =  _ux_host_class_cdc_acm_endpoints_get(cdc_acm);

    /* If we are done success, go on to mount interface.  */
    if (status == UX_SUCCESS)
    {
        /* Mark the cdc_acm as mounting now.  Both interfaces need to be mounting. */
        cdc_acm -> ux_host_class_cdc_acm_state =  UX_HOST_CLASS_INSTANCE_MOUNTING;

        /* If we have the Control Class, we have to configure the speed, parity ... */
        if (cdc_acm -> ux_host_class_cdc_acm_interface -> ux_interface_descriptor.bInterfaceClass == UX_HOST_CLASS_CDC_CONTROL_CLASS)
        {

            /* We need to wait for some device to settle. The Radicom USB Modem is an example of
               these device who fail the first Set_Line_Coding command if sent too quickly.
               The timing does not have to be precise so we use the thread sleep function.
               The default sleep value is 1 seconds.  */
            _ux_utility_delay_ms(UX_HOST_CLASS_CDC_ACM_DEVICE_INIT_DELAY);

            /* Do a GET_LINE_CODING first.  */
            status = _ux_host_class_cdc_acm_ioctl(cdc_acm, UX_HOST_CLASS_CDC_ACM_IOCTL_GET_LINE_CODING, (VOID *) &line_coding);

            /* If we are done success, go on.  */
            if (status == UX_SUCCESS)
            {

                /* Set the default values to the device, first line coding.  */
                line_coding.ux_host_class_cdc_acm_line_coding_dter      = UX_HOST_CLASS_CDC_ACM_LINE_CODING_DEFAULT_RATE;
                line_coding.ux_host_class_cdc_acm_line_coding_stop_bit  = UX_HOST_CLASS_CDC_ACM_LINE_CODING_DEFAULT_STOP_BIT;
                line_coding.ux_host_class_cdc_acm_line_coding_parity    = UX_HOST_CLASS_CDC_ACM_LINE_CODING_DEFAULT_PARITY;
                line_coding.ux_host_class_cdc_acm_line_coding_data_bits = UX_HOST_CLASS_CDC_ACM_LINE_CODING_DEFAULT_DATA_BIT;
                status = _ux_host_class_cdc_acm_ioctl(cdc_acm, UX_HOST_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING, (VOID *) &line_coding);
            }

            /* If we are done success, go on.  */
            if (status == UX_SUCCESS)
            {

                /* Set the default values to the device, line state.  */
                line_state.ux_host_class_cdc_acm_line_state_rts       = 1;
                line_state.ux_host_class_cdc_acm_line_state_dtr       = 1;
                status = _ux_host_class_cdc_acm_ioctl(cdc_acm, UX_HOST_CLASS_CDC_ACM_IOCTL_SET_LINE_STATE, (VOID *) &line_state);
            }

            /* If we are done success, go on.  */
            if (status == UX_SUCCESS)
            {

                /* Get the capabilities of the device. We need to know if the commands are multiplexed over the comm
                interface or the data interface.  */
                status =  _ux_host_class_cdc_acm_capabilities_get(cdc_acm);
            }
        }

        /* If we are done success, go on.  */
        if (status == UX_SUCCESS)
        {
            /* Mark the cdc_acm as live now.  Both interfaces need to be live. */
            cdc_acm -> ux_host_class_cdc_acm_state =  UX_HOST_CLASS_INSTANCE_LIVE;

            /* If all is fine and the device is mounted, we may need to inform the application
               if a function has been programmed in the system structure.  */
            if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
            {

                /* Call system change function.  */
                _ux_system_host ->  ux_system_host_change_function(UX_DEVICE_INSERTION, cdc_acm -> ux_host_class_cdc_acm_class, (VOID *) cdc_acm);
            }

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_CDC_ACM_ACTIVATE, cdc_acm, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

            /* If trace is enabled, register this object.  */
            UX_TRACE_OBJECT_REGISTER(UX_TRACE_HOST_OBJECT_TYPE_INTERFACE, cdc_acm, 0, 0, 0)

            /* We are done success. */
            return(UX_SUCCESS);
        }
    }
#endif

    /* On error case, it's possible data buffer allocated for interrupt endpoint and transfer started, stop and free it.  */
    if (cdc_acm -> ux_host_class_cdc_acm_interrupt_endpoint && 
        cdc_acm -> ux_host_class_cdc_acm_interrupt_endpoint -> ux_endpoint_transfer_request.ux_transfer_request_data_pointer)
    {

        /* The first transfer request has already been initiated. Abort it.  */
        _ux_host_stack_endpoint_transfer_abort(cdc_acm -> ux_host_class_cdc_acm_interrupt_endpoint);

        /* Free the memory for the data pointer.  */
        _ux_utility_memory_free(cdc_acm -> ux_host_class_cdc_acm_interrupt_endpoint -> ux_endpoint_transfer_request.ux_transfer_request_data_pointer);
    }

    /* Destroy the instance.  */
    _ux_host_stack_class_instance_destroy(cdc_acm -> ux_host_class_cdc_acm_class, (VOID *) cdc_acm);

#if !defined(UX_HOST_STANDALONE)

    /* Destroy the semaphore.  */
    _ux_host_semaphore_delete(&cdc_acm -> ux_host_class_cdc_acm_semaphore);
#endif

    /* Unmount instance. */
    interface_ptr -> ux_interface_class_instance = UX_NULL;

    /* Free instance. */
    _ux_utility_memory_free(cdc_acm);

    return(status);
}

