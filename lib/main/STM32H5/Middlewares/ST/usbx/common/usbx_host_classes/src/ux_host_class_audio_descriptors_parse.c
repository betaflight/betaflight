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
/*    _ux_host_class_audio_descriptors_parse              PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function parsees all interface descriptors and their audio     */
/*    class specific descriptors relates to current audio instance.       */
/*                                                                        */
/*    The function scans the device configuration descriptor. Once the    */
/*    interface descriptors of the audio instance are found, each audio   */
/*    class descriptors belong to the interface are passed to parse       */
/*    function for caller one by one to parse. The interface descriptor   */
/*    is also passed to provide more information. Another pointer to      */
/*    arguments is also available as parse function parameter for caller  */
/*    to pass necessary caller specific data to process in parse function.*/
/*                                                                        */
/*    The descriptor parsing can be terminated by returning non-zero      */
/*    code in parse function.                                             */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    audio                                 Pointer to audio instance     */
/*    parse_function                        Parse function for each       */
/*                                          audio class descriptor        */
/*    arg                                   Parse function argument       */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Audio Class                                                         */
/*    Application                                                         */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  07-29-2022     Chaoqiong Xiao           Initial Version 6.1.12        */
/*                                                                        */
/**************************************************************************/
UINT _ux_host_class_audio_descriptors_parse(UX_HOST_CLASS_AUDIO *audio,
        UINT(*parse_function)(VOID  *arg,
                              UCHAR *packed_interface_descriptor,
                              UCHAR *packed_endpoint_descriptor,
                              UCHAR *packed_audio_descriptor),
        VOID* arg)
{

UCHAR                                           *descriptor;
UCHAR                                           *interface_descriptor;
UCHAR                                           *endpoint_descriptor;
ULONG                                           ac_interface, as_interface;
ULONG                                           total_descriptor_length;
ULONG                                           descriptor_length;
ULONG                                           descriptor_type;
UINT                                            status;


    /* Ensure the instance is valid.  */
    if (_ux_host_stack_class_instance_verify(_ux_system_host_class_audio_name, (VOID *) audio) != UX_SUCCESS)
    {        

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, audio, 0, 0, UX_TRACE_ERRORS, 0, 0)
        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* Device state check.  */
    if (audio -> ux_host_class_audio_device -> ux_device_state != UX_DEVICE_CONFIGURED)
        return(UX_INVALID_STATE);

    /* Get the descriptor to the selected format.  */
    descriptor =  audio -> ux_host_class_audio_configuration_descriptor;
    total_descriptor_length =  audio -> ux_host_class_audio_configuration_descriptor_length;

    /* Get target interface number of control/streaming.  */
    ac_interface = audio -> ux_host_class_audio_control_interface_number;
    as_interface = audio -> ux_host_class_audio_streaming_interface -> ux_interface_descriptor.bInterfaceNumber;

    /* Haven't found interface, endpoint yet.  */
    interface_descriptor = UX_NULL;
    endpoint_descriptor = UX_NULL;

    /* Scan the descriptor for the Audio Control/Streaming interface.  */
    while (total_descriptor_length)
    {

        /* Gather the length, type and subtype of the descriptor.  */
        descriptor_length =   *descriptor;
        descriptor_type =     *(descriptor + 1);

        /* Make sure this descriptor has at least the minimum length.  */
        if (descriptor_length < 3)
            return(UX_DESCRIPTOR_CORRUPTED);

        /* Process relative to descriptor type.  */
        switch (descriptor_type)
        {

        case UX_INTERFACE_DESCRIPTOR_ITEM:

            /* Ensure we have the correct interface for Audio Control/Streaming.  */
            if (descriptor[2] == ac_interface || descriptor[2] == as_interface)
            {

                /* Mark we have found it.  */
                interface_descriptor = descriptor;
            }
            else
            {

                /* Haven't found it.  */
                interface_descriptor = UX_NULL;
            }
            break;

        case UX_ENDPOINT_DESCRIPTOR_ITEM:

            /* Log endpoint descriptor for following CS_ENDPOINT.  */
            endpoint_descriptor = descriptor;
            break;

        case UX_HOST_CLASS_AUDIO_CS_ENDPOINT:
        case UX_HOST_CLASS_AUDIO_CS_INTERFACE:

            /* Have we found the audio interface yet?  */
            if (interface_descriptor != UX_NULL)
            {

                /* Yes, parse the audio specific descriptor.  */
                status = parse_function(arg, interface_descriptor, endpoint_descriptor, descriptor);

                /* Terminate the parsing if status is not 0.  */
                if (status)
                {

                    /* Parsing terminated by user, it's OK.  */
                    return(UX_SUCCESS);
                }
            }
            break;

        default:
            break;
        }

        /* Verify if the descriptor is still valid.  */
        if (descriptor_length > total_descriptor_length)
            return(UX_DESCRIPTOR_CORRUPTED);

        /* Jump to the next descriptor if we have not reached the end.  */
        descriptor +=  descriptor_length;

        /* And adjust the length left to parse in the descriptor.  */
        total_descriptor_length -=  descriptor_length;
    }

    /* We get here when all descriptors scanned.  */
    return(UX_SUCCESS);
}
