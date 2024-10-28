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
/*    _ux_host_class_audio_device_type_get                PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function obtains the device type.                              */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    audio                                 Pointer to audio class        */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_descriptor_parse          Parse descriptor              */ 
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
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            removed protocol store,     */
/*                                            added audio 2.0 support,    */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_audio_device_type_get(UX_HOST_CLASS_AUDIO *audio)
{

UCHAR                                           *iad;
UCHAR                                           *descriptor;
UCHAR                                           *interface_descriptor;
ULONG                                           total_descriptor_length;
ULONG                                           descriptor_length;
ULONG                                           descriptor_type;
ULONG                                           descriptor_subtype;
ULONG                                           descriptor_found;
ULONG                                           interface_number;
UINT                                            i;

    /* Get the descriptor to the entire configuration.  */
    descriptor =               audio -> ux_host_class_audio_configuration_descriptor;
    total_descriptor_length =  audio -> ux_host_class_audio_configuration_descriptor_length;
    
    /* Default is Interface descriptor not yet found.  */    
    descriptor_found =  UX_FALSE;
    iad = UX_NULL;
    interface_descriptor = UX_NULL;
    interface_number = audio -> ux_host_class_audio_streaming_interface
                                    -> ux_interface_descriptor.bInterfaceNumber;
    
    /* Scan the descriptor for the Audio Streaming interface.  */
    while (total_descriptor_length)
    {

        /* Gather the length, type and subtype of the descriptor.  */
        descriptor_length =   *descriptor;
        descriptor_type =     *(descriptor + 1);
        descriptor_subtype =  *(descriptor + 2);

        /* Make sure this descriptor has at least the minimum length.  */
        if (descriptor_length < 3)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_DESCRIPTOR_CORRUPTED);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, descriptor, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_DESCRIPTOR_CORRUPTED);
        }

        /* Process relative to the descriptor type.  */
        switch (descriptor_type)
        {

        /* There is IAD in case UAC 1.0 with IAD or UAC 2.0/3.0.  */
        case UX_INTERFACE_ASSOCIATION_DESCRIPTOR_ITEM:

            /* Ensure we have the correct IAD that includes the interface.  */
            if ((descriptor[4] == UX_HOST_CLASS_AUDIO_CLASS) &&
                (descriptor[2] <= interface_number) &&
                (descriptor[2] + descriptor[3] > interface_number))
            {
                iad = descriptor;

                /* Control interface must be the first interface (UAC 2.0/3.0).  */
                audio -> ux_host_class_audio_control_interface_number = descriptor[2];
            }
            else
                iad = UX_NULL;
            break;

        case UX_INTERFACE_DESCRIPTOR_ITEM:

            /* Ensure we have the correct interface for Audio Control.  */
            if ((descriptor[5] == UX_HOST_CLASS_AUDIO_CLASS) &&
                (descriptor[6] == UX_HOST_CLASS_AUDIO_SUBCLASS_CONTROL))
                interface_descriptor = descriptor;
            else
            {
                interface_descriptor = UX_NULL;
                descriptor_found = UX_FALSE;
            }

            /* Check IAD.  */
            if (iad)
            {

                /* Check if interface is out of IAD.  */
                if ((iad[2] > interface_descriptor[2]) ||
                    (iad[2] + iad[3] <= interface_descriptor[2]))
                    iad = UX_NULL;
            }
            break;

        case UX_HOST_CLASS_AUDIO_CS_INTERFACE:

            /* First make sure we have found the correct generic interface descriptor.  */
            if (interface_descriptor != UX_NULL)
            {

                /* Check the sub type.  */
                switch (descriptor_subtype)
                {

                /* UAC 1.0, AC interface contains AS interface information.  */
                case UX_HOST_CLASS_AUDIO_CS_HEADER:

                    if (interface_descriptor[7] == UX_HOST_CLASS_AUDIO_PROTOCOL_IP_VERSION_01_00)
                    {

                        /* Check baInterfaceNr@8 to see if it's the right interface.  */
                        for (i = 0; i < descriptor[7]; i ++)
                        {
                            if (descriptor[8 + i] == interface_number)
                            {
                                descriptor_found = UX_TRUE;
                                break;
                            }
                        }
                    }
                    else if (iad)
                    {

                        /* UAC 2.0 or 3.0 IAD first interface indicates AC.  */
                        descriptor_found = UX_TRUE;
                    }
                    break;

                case UX_HOST_CLASS_AUDIO_CS_INPUT_TERMINAL:
                case UX_HOST_CLASS_AUDIO_CS_OUTPUT_TERMINAL:

                    /* Ensure it's right AC ITT/OTT.  */
                    if (descriptor_found)
                    {

                        /* In UAC 1.0/2.0/3.0, bTerminalID at 3, wTerminalType at 4.  */
                        if (descriptor[3] == audio -> ux_host_class_audio_terminal_link)
                        {

                            /* Connect to ITT, it's output, connect to OTT it's input.  */
                            if (descriptor_subtype == UX_HOST_CLASS_AUDIO_CS_INPUT_TERMINAL)
                                audio -> ux_host_class_audio_type = UX_HOST_CLASS_AUDIO_OUTPUT;
                            else
                                audio -> ux_host_class_audio_type = UX_HOST_CLASS_AUDIO_INPUT;

                            /* Return successful completion.  */
                            return(UX_SUCCESS);
                        }
                    }
                    break;

                default:
                    break;
                }
            }
            break;

        default:
            break;
        }       

        /* Verify if the descriptor is still valid.  */
        if (descriptor_length > total_descriptor_length)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_DESCRIPTOR_CORRUPTED);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, descriptor, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_DESCRIPTOR_CORRUPTED);
        }            

        /* Jump to the next descriptor if we have not reached the end.  */
        descriptor +=  descriptor_length;

        /* And adjust the length left to parse in the descriptor.  */
        total_descriptor_length -=  descriptor_length;
    }

    /* Error trap. */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_AUDIO_WRONG_TYPE);

    /* We get here when either the report descriptor has a problem or we could
       not find the right audio device.  */
    return(UX_HOST_CLASS_AUDIO_WRONG_TYPE);
}

