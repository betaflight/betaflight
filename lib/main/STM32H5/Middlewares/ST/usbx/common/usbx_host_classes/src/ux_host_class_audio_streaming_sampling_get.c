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
/*    _ux_host_class_audio_streaming_sampling_get         PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function obtains successive sampling characteristics for the   */ 
/*    audio streaming channel.                                            */ 
/*                                                                        */
/*    Note only Audio 1.0 and RAW (PCM like) format is supported.         */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    audio                                 Pointer to audio class        */ 
/*    audio_sampling                        Pointer to audio sampling     */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_class_instance_verify  Verify instance is valid      */ 
/*    _ux_utility_descriptor_parse          Parse the descriptor          */ 
/*    _ux_host_mutex_on                     Get mutex                     */
/*    _ux_host_mutex_off                    Put mutex                     */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
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
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            protect reentry with mutex, */
/*                                            fixed error return code,    */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_audio_streaming_sampling_get(UX_HOST_CLASS_AUDIO *audio, UX_HOST_CLASS_AUDIO_SAMPLING_CHARACTERISTICS *audio_sampling)
{

UCHAR *                                  descriptor;
UX_INTERFACE_DESCRIPTOR                  interface_descriptor;
UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR audio_interface_descriptor;
ULONG                                    total_descriptor_length;
UINT                                     descriptor_length;
UINT                                     descriptor_type;
UINT                                     descriptor_subtype;
UINT                                     interface_found;
ULONG                                    lower_frequency;
ULONG                                    higher_frequency;
UINT                                     specific_frequency_count;
UINT                                     previous_match_found;
    
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_AUDIO_STREAMING_SAMPLING_GET, audio, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Ensure the instance is valid.  */
    if (_ux_host_stack_class_instance_verify(_ux_system_host_class_audio_name, (VOID *) audio) != UX_SUCCESS)
    {        

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, audio, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* Protect thread reentry to this instance.  */
    _ux_host_mutex_on(&audio -> ux_host_class_audio_mutex);

    /* Reset the match flag.  */
    previous_match_found =  UX_FALSE;

    /* Get the descriptor to the entire configuration.  */
    descriptor =               audio -> ux_host_class_audio_configuration_descriptor;
    total_descriptor_length =  audio -> ux_host_class_audio_configuration_descriptor_length;
    
    /* Default is Interface descriptor not yet found.  */    
    interface_found =  UX_FALSE;
    
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

            /* Unprotect thread reentry to this instance.  */
            _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);

            return(UX_DESCRIPTOR_CORRUPTED);
        }

        /* Process relative to descriptor type.  */
        switch(descriptor_type)
        {


        case UX_INTERFACE_DESCRIPTOR_ITEM:

            /* Parse the interface descriptor and make it machine independent */
            _ux_utility_descriptor_parse(descriptor, _ux_system_interface_descriptor_structure,
                                            UX_INTERFACE_DESCRIPTOR_ENTRIES, (UCHAR *) &interface_descriptor);

            /* Ensure we have the correct interface for Audio streaming.  */
            if ((interface_descriptor.bInterfaceClass == UX_HOST_CLASS_AUDIO_CLASS) &&
                (interface_descriptor.bInterfaceSubClass == UX_HOST_CLASS_AUDIO_SUBCLASS_STREAMING))
            {

                /* Mark we have found it.  */
                interface_found =  UX_TRUE;
            }
            else
            {
                interface_found =  UX_FALSE;
            }
            break;
                

        case UX_HOST_CLASS_AUDIO_CS_INTERFACE:

            /* First make sure we have found the correct generic interface descriptor.  */
            if ((interface_found == UX_TRUE) && (descriptor_subtype == UX_HOST_CLASS_AUDIO_CS_FORMAT_TYPE))
            {

                /* Parse the FORMAT_TYPE descriptor and make it machine independent.  */
                _ux_utility_descriptor_parse(descriptor, _ux_system_class_audio_interface_descriptor_structure,
                                                UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_ENTRIES, (UCHAR *) &audio_interface_descriptor);

                /* This descriptor must refer to a PCM audio type.  */
                if (audio_interface_descriptor.bFormatType != UX_HOST_CLASS_AUDIO_FORMAT_TYPE_I)    
                    break;

                /* If this is the first time we ask for a streaming interface characteristics
                   we return the first we find.  */
                if ((audio_sampling -> ux_host_class_audio_sampling_characteristics_channels == 0) && 
                    (audio_sampling -> ux_host_class_audio_sampling_characteristics_resolution == 0))
                {

                    audio_sampling -> ux_host_class_audio_sampling_characteristics_channels =    audio_interface_descriptor.bNrChannels;
                    audio_sampling -> ux_host_class_audio_sampling_characteristics_resolution =  audio_interface_descriptor.bBitResolution;

                    if (audio_interface_descriptor.bSamFreqType == 0)
                    {
                        
                        /* The declaration of frequency is contiguous, so get the minimum and maximum */
                        audio_sampling -> ux_host_class_audio_sampling_characteristics_frequency_low =  
                                                                          (ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH) |
                                                                          ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 1)) << 8 |
                                                                          ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 2)) << 16;

                        audio_sampling -> ux_host_class_audio_sampling_characteristics_frequency_high = 
                                                                          (ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 3) |
                                                                          ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 4)) << 8 |                                       
                                                                          ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 5)) << 16;
                    }
                    else
                    {

                        /* The declaration of the frequency is declared as an array of specific values.  
                           We take the first one here.  */
                        audio_sampling -> ux_host_class_audio_sampling_characteristics_frequency_low =  
                                                                          (ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH ) |
                                                                          ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 1 )) << 8 |
                                                                          ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 2 )) << 16;

                        /* High and low frequencies are the same here.  */
                        audio_sampling -> ux_host_class_audio_sampling_characteristics_frequency_high =  audio_sampling -> ux_host_class_audio_sampling_characteristics_frequency_low;

                        /* Unprotect thread reentry to this instance.  */
                        _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);

                        /* We have found the first streaming characteristics.  */
                        return(UX_SUCCESS);
                    }
                }
                else
                {

                    /* This is not the second time we ask for the characteristics, we need
                       to find 1st where we left off and look at the next characteristics.  */
                    if ((audio_sampling -> ux_host_class_audio_sampling_characteristics_channels == audio_interface_descriptor.bNrChannels) &&
                        (audio_sampling -> ux_host_class_audio_sampling_characteristics_resolution == audio_interface_descriptor.bBitResolution))
                    {

                        if (audio_interface_descriptor.bSamFreqType == 0)
                        {
                        
                            /* The declaration of frequency is contiguous, so get the minimum and maximum.  */
                            lower_frequency =  (ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH) |
                                               ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 1)) << 8 |
                                               ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 2)) << 16;

                            higher_frequency =  (ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 3) |
                                                ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 4)) << 8 |                                       
                                                ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 5)) << 16;

                            if ((audio_sampling -> ux_host_class_audio_sampling_characteristics_frequency_low == lower_frequency) &&
                                (audio_sampling -> ux_host_class_audio_sampling_characteristics_frequency_high == higher_frequency))
                            {

                                /* We have a match.  */
                                previous_match_found =  UX_TRUE;
                                break;
                            }
                        }
                        else
                        {

                            /* The declaration of the frequency is declared as an array of specific values.  */
                            for (specific_frequency_count = 0; specific_frequency_count < audio_interface_descriptor.bSamFreqType; specific_frequency_count++)
                            {

                                lower_frequency =  (ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + (specific_frequency_count * 3)) |
                                                   ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 1 + (specific_frequency_count * 3))) << 8 |
                                                   ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 2 + (specific_frequency_count * 3))) << 16;

                                /* Compare the frequency.  */
                                if (audio_sampling -> ux_host_class_audio_sampling_characteristics_frequency_low == lower_frequency)
                                {
                                
                                    previous_match_found =  UX_TRUE;
                                }
                                else
                                {

                                    /* Now the frequency is different, if we had a match before
                                       this becomes the next characteristics found.  */
                                    if (previous_match_found == UX_TRUE)
                                    {

                                        /* We return this characteristics.  */
                                        audio_sampling -> ux_host_class_audio_sampling_characteristics_channels =        audio_interface_descriptor.bNrChannels;
                                        audio_sampling -> ux_host_class_audio_sampling_characteristics_resolution =      audio_interface_descriptor.bBitResolution;
                                        audio_sampling -> ux_host_class_audio_sampling_characteristics_frequency_low =   lower_frequency;
                                        audio_sampling -> ux_host_class_audio_sampling_characteristics_frequency_high =  lower_frequency;

                                        /* Unprotect thread reentry to this instance.  */
                                        _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);

                                        /* Return successful completion.  */
                                        return(UX_SUCCESS);
                                    }
                                }                                   
                            }
                        }
                    }
                    else
                    {

                        /* We come here when the current characteristics does not match
                           the one given by the application. We need to check if we had found a match
                           before in which case, this is the one we need to return.  */
                        if (previous_match_found == UX_TRUE)
                        {

                            /* We return this characteristics.  */
                            audio_sampling -> ux_host_class_audio_sampling_characteristics_channels =    audio_interface_descriptor.bNrChannels;
                            audio_sampling -> ux_host_class_audio_sampling_characteristics_resolution =  audio_interface_descriptor.bBitResolution;

                            if (audio_interface_descriptor.bSamFreqType == 0)
                            {
                        
                                /* The declaration of frequency is contiguous, so get the minimum and maximum.  */
                                lower_frequency =  (ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH) |
                                                   ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 1)) << 8 |
                                                   ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 2)) << 16;

                                higher_frequency =  (ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 3) |
                                                    ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 4)) << 8 |                                       
                                                    ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 5)) << 16;

                                audio_sampling -> ux_host_class_audio_sampling_characteristics_frequency_low =   lower_frequency;
                                audio_sampling -> ux_host_class_audio_sampling_characteristics_frequency_high =  higher_frequency;

                                /* Unprotect thread reentry to this instance.  */
                                _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);

                                /* Return successful completion.  */
                                return(UX_SUCCESS);
                            }
                            else        
                            {

                                lower_frequency =  (ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH) |
                                                   ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 1)) << 8 |
                                                   ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 2)) << 16;

                                audio_sampling -> ux_host_class_audio_sampling_characteristics_frequency_low =   lower_frequency;
                                audio_sampling -> ux_host_class_audio_sampling_characteristics_frequency_high =  lower_frequency;

                                /* Unprotect thread reentry to this instance.  */
                                _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);

                                /* Return successful completion.  */
                                return(UX_SUCCESS);
                            }
                        }
                    }
                }                        
            }
        }       

        /* Verify the descriptor is still valid.  */
        if (descriptor_length > total_descriptor_length)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_DESCRIPTOR_CORRUPTED);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, descriptor, 0, 0, UX_TRACE_ERRORS, 0, 0)

            /* Unprotect thread reentry to this instance.  */
            _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);

            return(UX_DESCRIPTOR_CORRUPTED);
        }

        /* Jump to the next descriptor if we have not reached the end.  */
        descriptor +=  descriptor_length;

        /* And adjust the length left to parse in the descriptor.  */
        total_descriptor_length -=  descriptor_length;
    }

    /* Unprotect thread reentry to this instance.  */
    _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);

    /* Error trap. */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_NO_ALTERNATE_SETTING);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_NO_ALTERNATE_SETTING, audio, 0, 0, UX_TRACE_ERRORS, 0, 0)

    /* We get here when either the report descriptor has a problem or we could
       not find the right audio device.  */
    return(UX_NO_ALTERNATE_SETTING);
}

