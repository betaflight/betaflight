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


#if defined(UX_HOST_CLASS_AUDIO_2_SUPPORT)
static inline UINT _ux_host_class_audio_alternate_setting_locate_2(
    UX_HOST_CLASS_AUDIO *audio,
    UX_HOST_CLASS_AUDIO_SAMPLING *audio_sampling,
    UINT *alternate_setting);
#endif


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_audio_alternate_setting_locate       PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function finds the right alternate setting according to the   */
/*     sampling desired.                                                  */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    audio                                 Pointer to audio class        */ 
/*    audio_sampling                        Pointer to audio sampling     */ 
/*    alternate_setting                     Pointer to located alternate  */ 
/*                                            setting                     */ 
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
/*                                            added audio 2.0 support,    */
/*                                            resulting in version 6.1.12 */
/**************************************************************************/
UINT  _ux_host_class_audio_alternate_setting_locate(UX_HOST_CLASS_AUDIO *audio, UX_HOST_CLASS_AUDIO_SAMPLING *audio_sampling,
                                                    UINT *alternate_setting)
{

#if defined(UX_HOST_CLASS_AUDIO_2_SUPPORT)
    return(_ux_host_class_audio_alternate_setting_locate_2(audio, audio_sampling, alternate_setting));
#else

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
    

    /* Get the descriptor to the entire configuration.  */
    descriptor =  audio -> ux_host_class_audio_configuration_descriptor;
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

            return(UX_DESCRIPTOR_CORRUPTED);
        }
        /* Process relative to descriptor type.  */
        switch (descriptor_type)
        {


        case UX_INTERFACE_DESCRIPTOR_ITEM:

            /* Parse the interface descriptor and make it machine independent.  */
            _ux_utility_descriptor_parse(descriptor, _ux_system_interface_descriptor_structure,
                                            UX_INTERFACE_DESCRIPTOR_ENTRIES, (UCHAR *) &interface_descriptor);

            /* Ensure we have the correct interface for Audio streaming.  */
            if ((interface_descriptor.bInterfaceClass == UX_HOST_CLASS_AUDIO_CLASS) &&
                (interface_descriptor.bInterfaceSubClass == UX_HOST_CLASS_AUDIO_SUBCLASS_STREAMING))
            {

                /* Mark we have found it.  */
                interface_found =  UX_TRUE;
                    
                /* And memorize the alternate setting.  */
                *alternate_setting =  interface_descriptor.bAlternateSetting;
            }
            else
            {

                /* Haven't found it.  */
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

                /* The number of channels demanded by the application must match.  */
                if (audio_sampling -> ux_host_class_audio_sampling_channels != audio_interface_descriptor.bNrChannels)
                    break;

                /* The resolution demanded by the application must match.  */
                if (audio_sampling -> ux_host_class_audio_sampling_resolution != audio_interface_descriptor.bBitResolution)
                    break;

                /* Check the frequency demanded. The descriptor frequency is either defined
                   as a min and max frequency or an array of specified values.  */
                if (audio_interface_descriptor.bSamFreqType == 0)
                {
                        
                    /* The declaration of frequency is contiguous, so get the minimum and maximum */
                    lower_frequency =  (ULONG)  *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH) |
                                       ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 1)) << 8 |
                                       ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 2)) << 16;

                    higher_frequency =  (ULONG)  *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 3) |
                                        ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 4)) << 8 |                                       
                                        ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 5)) << 16;

                    /* Now compare with what is required.  */
                    if ((audio_sampling -> ux_host_class_audio_sampling_frequency >= lower_frequency) && 
                        (audio_sampling -> ux_host_class_audio_sampling_frequency <= higher_frequency))
                    {

                        /* We have found the right alternate setting.  */
                        audio -> ux_host_class_audio_sampling_descriptor = descriptor;
                        return(UX_SUCCESS);
                    }
                }
                else
                {

                    /* The declaration of the frequency is declared as an array of specific values.  */
                    for (specific_frequency_count = 0; specific_frequency_count < audio_interface_descriptor.bSamFreqType;
                            specific_frequency_count++)
                    {

                        lower_frequency =  (ULONG)  *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + (specific_frequency_count * 3)) |
                                           ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 1 + (specific_frequency_count * 3))) << 8 |
                                           ((ULONG) *(descriptor + UX_HOST_CLASS_AUDIO_INTERFACE_DESCRIPTOR_LENGTH + 2 + (specific_frequency_count * 3))) << 16;

                        /* Now compare with what is required.  */
                        if (audio_sampling -> ux_host_class_audio_sampling_frequency == lower_frequency)
                        {

                            /* We have found the right alternate setting.  */
                            audio -> ux_host_class_audio_sampling_descriptor = descriptor;
                            return(UX_SUCCESS);
                        }
                    }
                }                        
            }
            break;
        }       

        /* Verify if the descriptor is still valid.  */
        if (descriptor_length > total_descriptor_length)
        {

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, descriptor, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_DESCRIPTOR_CORRUPTED);
        }
        
        /* Jump to the next descriptor if we have not reached the end.  */
        descriptor +=  descriptor_length;

        /* And adjust the length left to parse in the descriptor.  */
        total_descriptor_length -=  descriptor_length;
    }

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_NO_ALTERNATE_SETTING, audio, 0, 0, UX_TRACE_ERRORS, 0, 0)

    /* We get here when either the report descriptor has a problem or we could
       not find the right audio device.  */
    return(UX_NO_ALTERNATE_SETTING);
#endif
}

#if defined(UX_HOST_CLASS_AUDIO_2_SUPPORT)
struct UX_HOST_CLASS_AUDIO_ALT_LOCATE_PARSER {
    UX_HOST_CLASS_AUDIO_SAMPLING    *sampling;
    UCHAR                           *clock_descriptor;
    ULONG                           alt;
    UINT                            status;
};
static UINT _ux_host_class_audio_alt_locate_parse(VOID *arg,
                        UCHAR *packed_interface_descriptor,
                        UX_HOST_CLASS_AUDIO_SAMPLING_CHARACTERISTICS *sam_attr)
{
struct UX_HOST_CLASS_AUDIO_ALT_LOCATE_PARSER    *parser = (struct UX_HOST_CLASS_AUDIO_ALT_LOCATE_PARSER *)arg;
UX_HOST_CLASS_AUDIO_SAMPLING                    *sampling = parser -> sampling;
ULONG                                           frequency_low, frequency_high;

    /* Check bNrChannels.  */
    if (sampling -> ux_host_class_audio_sampling_channels !=
        sam_attr -> ux_host_class_audio_sampling_characteristics_channels)
        return(0);

    /* Check bBitResolution.  */
    if (sampling -> ux_host_class_audio_sampling_resolution !=
        sam_attr -> ux_host_class_audio_sampling_characteristics_resolution)
        return(0);

    /* Calculate frequency low.  */
    frequency_low = sam_attr -> ux_host_class_audio_sampling_characteristics_frequency_low;
    if (UX_OVERFLOW_CHECK_MULV_ULONG(frequency_low, sam_attr -> ux_host_class_audio_sampling_characteristics_clock_mul))
    {

        /* Math error.  */
        parser -> status = UX_MATH_OVERFLOW;
        return(1);
    }
    frequency_low *= sam_attr -> ux_host_class_audio_sampling_characteristics_clock_mul;
    frequency_low /= sam_attr -> ux_host_class_audio_sampling_characteristics_clock_div;

    /* Calculate frequency high.  */
    frequency_high = sam_attr -> ux_host_class_audio_sampling_characteristics_frequency_high;
    if (UX_OVERFLOW_CHECK_MULV_ULONG(frequency_high, sam_attr -> ux_host_class_audio_sampling_characteristics_clock_mul))
    {

        /* Math error.  */
        parser -> status = UX_MATH_OVERFLOW;
        return(1);
    }
    frequency_high *= sam_attr -> ux_host_class_audio_sampling_characteristics_clock_mul;
    frequency_high /= sam_attr -> ux_host_class_audio_sampling_characteristics_clock_div;

    /* Check frequency in [low, high].  */
    if ((frequency_low <= sampling -> ux_host_class_audio_sampling_frequency) &&
        (frequency_high >= sampling -> ux_host_class_audio_sampling_frequency))
    {

        /* Save bAlternateSetting @ 3.  */
        parser -> alt = (ULONG)packed_interface_descriptor[3];

        /* Save UAC 1.0 FormatTypeI or UAC 2.0 CSD.  */
        parser -> clock_descriptor =
            sam_attr -> ux_host_class_audio_sampling_characteristics_descriptor;
        return(1);
    }

    /* Continue parsing.  */
    return(0);
}
static inline UINT _ux_host_class_audio_alternate_setting_locate_2(
    UX_HOST_CLASS_AUDIO *audio,
    UX_HOST_CLASS_AUDIO_SAMPLING *audio_sampling,
    UINT *alternate_setting)
{
struct UX_HOST_CLASS_AUDIO_ALT_LOCATE_PARSER    parser;
UINT                                            status;

    /* Parse specific sampling setting to get alt setting.  */
    parser.sampling = audio_sampling;
    parser.clock_descriptor = UX_NULL;
    parser.alt = 0xFF;
    parser.status = UX_SUCCESS;
    status = _ux_host_class_audio_raw_sampling_parse(audio, _ux_host_class_audio_alt_locate_parse, (VOID*)&parser);

    /* Check descriptor error.  */
    if (status != UX_SUCCESS)
        return(status);

    /* Check if valid alternate setting is found.  */
    if (parser.alt == 0xFF)
    {

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_NO_ALTERNATE_SETTING, audio, 0, 0, UX_TRACE_ERRORS, 0, 0)
        return(UX_NO_ALTERNATE_SETTING);
    }

    /* Save alternate setting.  */
    *alternate_setting = parser.alt;
    audio -> ux_host_class_audio_sampling_descriptor = parser.clock_descriptor;
    return(parser.status);
}
#endif
