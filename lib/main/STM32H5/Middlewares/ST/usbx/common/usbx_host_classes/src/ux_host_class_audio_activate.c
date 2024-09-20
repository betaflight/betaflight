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
/**   Audio Class                                                         */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_audio.h"
#include "ux_host_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_audio_activate                       PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*     This function activates the audio class. It may be called twice by */
/*     the same device if there is a audio control interface to this      */
/*     device.                                                            */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    command                               Pointer to command            */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_class_audio_configure        Configure the audio class     */
/*    _ux_host_class_audio_descriptor_get   Get audio descriptor          */
/*    _ux_host_class_audio_device_controls_list_get Get controls list     */
/*    _ux_host_class_audio_device_type_get  Get device type               */
/*    _ux_host_class_audio_streaming_terminal_get Get streaming terminal  */
/*    _ux_host_stack_class_instance_create  Create class instance         */
/*    _ux_host_stack_class_instance_destroy Destroy class instance        */
/*    _ux_utility_memory_allocate           Allocate a memory block       */
/*    _ux_host_mutex_create                 Create protection mutex       */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Audio Class                                                         */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            added interrupt support,    */
/*                                            protect reentry with mutex, */
/*                                            added feedback support,     */
/*                                            refined error handling,     */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_audio_activate(UX_HOST_CLASS_COMMAND *command)
{

UX_INTERFACE            *interface_ptr;
UX_HOST_CLASS_AUDIO     *audio;
#if defined(UX_HOST_CLASS_AUDIO_INTERRUPT_SUPPORT)
UX_HOST_CLASS_AUDIO_AC  *ac;
UX_ENDPOINT             *endp;
UX_TRANSFER             *trans;
UCHAR                   *desc;
UCHAR                   *iad;
UCHAR                   *uac10_ifd;
UCHAR                   *uac10_acd;
UCHAR                   desc_type;
UCHAR                   desc_length;
ULONG                   pos;
ULONG                   as_count;
#endif
UINT                    status;


    /* The audio is always activated by the interface descriptor and not the
       device descriptor.  */
    interface_ptr =  (UX_INTERFACE *) command -> ux_host_class_command_container;

    /* Check the subclass of the new device. If it is a Audio Control Interface,
       we don't need to create an instance of this function. When we get the streaming interface,
       we will search the audio control interface for the device.  */
    if (interface_ptr -> ux_interface_descriptor.bInterfaceSubClass == UX_HOST_CLASS_AUDIO_SUBCLASS_CONTROL)
    {
#if defined(UX_HOST_CLASS_AUDIO_INTERRUPT_SUPPORT)

        /* Allocate memory for AC instance.  */
        ac = (UX_HOST_CLASS_AUDIO_AC *) _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HOST_CLASS_AUDIO_AC));
        if (ac == UX_NULL)
            return(UX_MEMORY_INSUFFICIENT);

        /* Get descriptor to check AC/AS interfaces layouts.  */
        ac -> ux_host_class_audio_device = interface_ptr -> ux_interface_configuration -> ux_configuration_device;
        status = _ux_host_class_audio_descriptor_get((UX_HOST_CLASS_AUDIO *)ac);
        if (status != UX_SUCCESS)
        {
            _ux_utility_memory_free(ac);
            return(status);
        }

        /* Search for IAD or UAC1.0 AC header descriptor.  */
        as_count = 0;
        iad = UX_NULL;
        uac10_acd = UX_NULL;
        desc = ac -> ux_host_class_audio_configuration_descriptor;
        for (pos = 0; pos < ac -> ux_host_class_audio_configuration_descriptor_length;)
        {

            /* Get bLength@0, bDescriptorType@1.  */
            desc_length = *(desc);
            desc_type =   *(desc + 1);

            /* Check length error.  */
            if (desc_length < 2)
            {
                _ux_utility_memory_free(ac);
                return(UX_DESCRIPTOR_CORRUPTED);
            }

            /* Check descriptor type.  */
            switch(desc_type)
            {
            case UX_INTERFACE_ASSOCIATION_DESCRIPTOR_ITEM:

                /* Check bFirstInterface@2, must match AC interface if exist.  */
                if (desc[2] == interface_ptr -> ux_interface_descriptor.bInterfaceNumber)
                    iad = desc;
                break;

            case UX_INTERFACE_DESCRIPTOR_ITEM:

                /* Check bInterfaceNumber@2, if no IAD.  */
                if (desc[2] == interface_ptr -> ux_interface_descriptor.bInterfaceNumber)
                    uac10_ifd = desc;
                else
                    uac10_ifd = UX_NULL;
                break;

            case UX_HOST_CLASS_AUDIO_CS_INTERFACE:

                /* Check bDescriptorSubtype@3, if inside correct interface.  */
                if (uac10_ifd)
                {
                    if (desc[2] == UX_HOST_CLASS_AUDIO_CS_HEADER)
                        uac10_acd = desc;
                }
                break;

            default:
                break;

            } /* switch(desc_type) */

            /* Check if IAD exists.  */
            if (iad)
            {

                /* In this case, bFirstInterface@2 is AC, followed ASes.  */
                /* Use bInterfaceCount@3.  */
                as_count = (ULONG)iad[3] - 1;
                break;
            }

            /* Check if AC header exists.  */
            if (uac10_acd)
            {

                /* In this case, bInCollection@7 is AS count.  */
                as_count = (ULONG)uac10_acd[7];
                break;
            }

            /* Move pos and descriptor pointer.  */
            pos += desc_length;
            desc += desc_length;
        }

        /* If there are too many ASes, reallocate memory.  */
        if (as_count > 2)
        {
            _ux_utility_memory_free(ac);

            /* as_count - 2 < 255 - 2, so (as_count - 2) * sizeof(UX_HOST_CLASS_AUDIO*) does not overflow.  */
            /* sizeof(UX_HOST_CLASS_AUDIO_AC) is not big (< 255), total memory size calculation does not overflow.  */
            ac = (UX_HOST_CLASS_AUDIO_AC *) _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY,
                            sizeof(UX_HOST_CLASS_AUDIO_AC) + (as_count - 2) * sizeof(UX_HOST_CLASS_AUDIO*));
            if (ac == UX_NULL)
                return(UX_MEMORY_INSUFFICIENT);
        }

        /* Get interrupt endpoint.  */
        endp = interface_ptr -> ux_interface_first_endpoint;
        while(endp)
        {
            if (((endp -> ux_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) ==
                  UX_INTERRUPT_ENDPOINT) &&
                ((endp -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION)))
            {
                ac -> ux_host_class_audio_interrupt_endpoint = endp;
                trans = &endp -> ux_endpoint_transfer_request;

                /* Allocate buffer.  */
                trans -> ux_transfer_request_data_pointer =
                    _ux_utility_memory_allocate(UX_NO_ALIGN, UX_CACHE_SAFE_MEMORY,
                                    trans -> ux_transfer_request_packet_length);
                if (trans -> ux_transfer_request_data_pointer == UX_NULL)
                {
                    _ux_utility_memory_free(ac);
                    return(UX_MEMORY_INSUFFICIENT);
                }

                /* This transfer always have the IN direction.  */
                trans -> ux_transfer_request_type = UX_REQUEST_IN;

                /* This transfer always try read a full packet.  */
                trans -> ux_transfer_request_requested_length =
                                    trans -> ux_transfer_request_packet_length;

                break;
            }
            endp = endp -> ux_endpoint_next_endpoint;
        }

        /* If there is no interrupt endpoint, that's fine.  */
        /* Fill the fields.  */
        ac -> ux_host_class_audio_class = command -> ux_host_class_command_class_ptr;
        ac -> ux_host_class_audio_interface = interface_ptr;
        ac -> ux_host_class_audio_device = interface_ptr -> ux_interface_configuration -> ux_configuration_device;

        ac -> ux_host_class_audio_configuration_descriptor = ac -> ux_host_class_audio_device -> ux_device_packed_configuration;
        ac -> ux_host_class_audio_configuration_descriptor_length = interface_ptr -> ux_interface_configuration -> ux_configuration_descriptor.wTotalLength;
        ac -> ux_host_class_audio_as_count = as_count;

        /* Link to interface and class.  */
        interface_ptr -> ux_interface_class_instance = (VOID *)ac;
        _ux_host_stack_class_instance_create(ac -> ux_host_class_audio_class, (VOID *)ac);

        /* If all is fine and the AC is mounted, we may need to inform the application
           if a function has been programmed in the system structure.  */
        if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
            _ux_system_host ->  ux_system_host_change_function(UX_DEVICE_INSERTION, ac -> ux_host_class_audio_class, (VOID *) ac);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_AUDIO_ACTIVATE, ac, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

        /* If trace is enabled, register this object.  */
        UX_TRACE_OBJECT_REGISTER(UX_TRACE_HOST_OBJECT_TYPE_INTERFACE, ac, 0, 0, 0)
#endif

        return(UX_SUCCESS);
    }

    /* Obtain memory for this class instance.  */
    audio =  (UX_HOST_CLASS_AUDIO *) _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HOST_CLASS_AUDIO));
    if (audio == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Store the class container into this instance.  */
    audio -> ux_host_class_audio_class =  command -> ux_host_class_command_class_ptr;

    /* Store the interface container into the audio class instance.  */
    audio -> ux_host_class_audio_streaming_interface =  interface_ptr;

    /* Store the device container into the audio class instance.  */
    audio -> ux_host_class_audio_device =  interface_ptr -> ux_interface_configuration -> ux_configuration_device;

    /* This instance of the device must also be stored in the interface container.  */
    interface_ptr -> ux_interface_class_instance =  (VOID *) audio;

    /* Create this class instance.  */
    _ux_host_stack_class_instance_create(audio -> ux_host_class_audio_class, (VOID *) audio);

    /* Configure the audio.  */
    status =  _ux_host_class_audio_configure(audio);

    /* Get the audio descriptor (all the class specific stuff) and memorize them
       as we will need these descriptors to change settings.  */
    if (status == UX_SUCCESS)
        status =  _ux_host_class_audio_descriptor_get(audio);

    /* Locate the audio device streaming terminal.  */
    if (status == UX_SUCCESS)
        status =  _ux_host_class_audio_streaming_terminal_get(audio);

    /* Get the audio device type. Here we only support input and output devices.  */
    if (status == UX_SUCCESS)
        status =  _ux_host_class_audio_device_type_get(audio);

    /* Get the audio device controls.  */
#if !defined(UX_HOST_CLASS_AUDIO_DISABLE_CONTROLS)
    if (status == UX_SUCCESS)
        status =  _ux_host_class_audio_device_controls_list_get(audio);
#endif

    /* Create the mutex to protect multiple threads from accessing the same
       audio instance.  */
    if (status == UX_SUCCESS)
    {
        status =  _ux_host_mutex_create(&audio -> ux_host_class_audio_mutex, "ux_hot_class_audio_mutex");
        if (status != UX_SUCCESS)
            status = UX_MUTEX_ERROR;
    }

#if defined(UX_HOST_CLASS_AUDIO_INTERRUPT_SUPPORT)

    /* Find AC and link them.  */
    ac = (UX_HOST_CLASS_AUDIO_AC *)audio -> ux_host_class_audio_class -> ux_host_class_first_instance;
    while(ac)
    {

        /* Check interface number to see if it's right AC.  */
        if (audio -> ux_host_class_audio_control_interface_number ==
            ac -> ux_host_class_audio_interface -> ux_interface_descriptor.bInterfaceNumber)
        {

            /* Save AS to AC controlled list.  */
            for (pos = 0; pos < ac -> ux_host_class_audio_as_count; pos ++)
            {
                if (ac -> ux_host_class_audio_as[pos] == UX_NULL)
                {

                    /* This AC is for current AS.  */
                    audio -> ux_host_class_audio_ac = ac;
                    ac -> ux_host_class_audio_as[pos] = audio;
                    break;
                }
            }

            /* If there is no place for AS, something wrong.  */
            if (pos >= ac -> ux_host_class_audio_as_count)
            {
                status = UX_DESCRIPTOR_CORRUPTED;
                break;
            }
        }

        /* If AC is found and linked, done.  */
        if (audio -> ux_host_class_audio_ac != UX_NULL)
            break;

        /* If there is error, break.  */
        if (status != UX_SUCCESS)
            break;

        /* Check next AC/AS.  */
        ac = (UX_HOST_CLASS_AUDIO_AC *)ac -> ux_host_class_audio_next_instance;
    }
#endif

    /* Activate is done success.  */
    if (status == UX_SUCCESS)
    {

        /* Mark the audio as live now.  */
        audio -> ux_host_class_audio_state =  UX_HOST_CLASS_INSTANCE_LIVE;

        /* If all is fine and the device is mounted, we may need to inform the application
        if a function has been programmed in the system structure.  */
        if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
        {

            /* Call system change function.  */
            _ux_system_host ->  ux_system_host_change_function(UX_DEVICE_INSERTION, audio -> ux_host_class_audio_class, (VOID *) audio);
        }

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_AUDIO_ACTIVATE, audio, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

        /* If trace is enabled, register this object.  */
        UX_TRACE_OBJECT_REGISTER(UX_TRACE_HOST_OBJECT_TYPE_INTERFACE, audio, 0, 0, 0)

        /* Return completion status.  */
        return(UX_SUCCESS);
    }

    /* Error trap. */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, status);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, status, audio, 0, 0, UX_TRACE_ERRORS, 0, 0)

    /* Error, destroy the class instance.  */
    _ux_host_stack_class_instance_destroy(audio -> ux_host_class_audio_class, (VOID *) audio);

    /* Free resources.  */
#if defined(UX_HOST_CLASS_AUDIO_FEEDBACK_SUPPORT)
    if (audio -> ux_host_class_audio_feedback_endpoint -> ux_endpoint_transfer_request.ux_transfer_request_data_pointer)
        _ux_utility_memory_free(audio -> ux_host_class_audio_feedback_endpoint -> ux_endpoint_transfer_request.ux_transfer_request_data_pointer);
#endif
    _ux_utility_memory_free(audio);

    /* Unlink instance from interface.  */
    interface_ptr -> ux_interface_class_instance = UX_NULL;

    /* Return the error.  */
    return(status);
}
