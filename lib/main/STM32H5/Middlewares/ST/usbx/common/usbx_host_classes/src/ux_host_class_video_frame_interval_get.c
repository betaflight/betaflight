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
/**   Video Class                                                         */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_video.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_video_frame_interval_get             PORTABLE C      */ 
/*                                                           6.1.2        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function finds the frame intervals within the frame.           */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    video                                 Pointer to video class        */ 
/*    interval_parameter                    Interval request structure    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_system_error_handler              Log system error              */
/*    _ux_utility_descriptor_parse          Parse descriptor              */ 
/*    _ux_utility_long_get                  Get 32-bit value              */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Video Class                                                         */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  11-09-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed compile warnings 64b, */
/*                                            resulting in version 6.1.2  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_video_frame_interval_get(UX_HOST_CLASS_VIDEO *video, UX_HOST_CLASS_VIDEO_PARAMETER_FRAME_INTERVAL *interval_parameter)
{

UCHAR                                           *descriptor;
UX_HOST_CLASS_VIDEO_FRAME_DESCRIPTOR            frame_descriptor;
ULONG                                           total_descriptor_length;
ULONG                                           descriptor_length;
ULONG                                           descriptor_type;
ULONG                                           descriptor_subtype;
ULONG                                           intervals_to_copy;
ULONG                                           i;

    /* Get the descriptor to the selected format.  */
    descriptor =  video -> ux_host_class_video_current_format_address;
    total_descriptor_length =  video -> ux_host_class_video_length_formats;
    
    /* Descriptors are arranged in order. First FORMAT then FRAME.  */
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
            //UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, descriptor, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_DESCRIPTOR_CORRUPTED);
        }

        /* Must be CS_INTERFACE.  */
        if (descriptor_type == UX_HOST_CLASS_VIDEO_CS_INTERFACE)
        {

            /* Process relative to descriptor type.  */
            switch (descriptor_subtype)
            {
    
                case UX_HOST_CLASS_VIDEO_VS_FRAME_UNCOMPRESSED  :
                case UX_HOST_CLASS_VIDEO_VS_FRAME_MJPEG         :
                case UX_HOST_CLASS_VIDEO_VS_FRAME_FRAME_BASED   :
                
                    /* We found a Frame descriptor.  Is it the right one ? */
                    if (interval_parameter -> ux_host_class_video_parameter_frame_requested == *(descriptor + 3))
                    {
    
                        /* Make the descriptor machine independent.  */
                        _ux_utility_descriptor_parse(descriptor, _ux_system_class_video_frame_descriptor_structure,
                                                     UX_HOST_CLASS_VIDEO_FRAME_DESCRIPTOR_ENTRIES, (UCHAR *) &frame_descriptor);
                                                            
                        /* Check the frame interval type.  */
                        if (frame_descriptor.bFrameIntervalType == 0)
                        {

                            /* Frame type is continuous, copy Min, Max and Step.  */
                            intervals_to_copy = 3;
                        }
                        else
                        {

                            /* Frame type is discrete, copy number of frame intervals.  */
                            intervals_to_copy = frame_descriptor.bFrameIntervalType;
                        }

                        /* Check if we have enough space to copy.  */
                        if (intervals_to_copy * sizeof(ULONG) > interval_parameter -> ux_host_class_video_parameter_frame_interval_buffer_length)
                        {
                            intervals_to_copy = (ULONG)(interval_parameter -> ux_host_class_video_parameter_frame_interval_buffer_length / sizeof(ULONG));
                        }

                        /* Return bytes copied.  */
                        interval_parameter -> ux_host_class_video_parameter_frame_interval_buffer_length_written = (ULONG)(intervals_to_copy * sizeof(ULONG));

                        /* Loop to copy interval data.  */
                        for (i = 0; i < intervals_to_copy; i++)
                        {

                            /* Copy intervals to user buffer.  */
                            interval_parameter -> ux_host_class_video_parameter_frame_interval_buffer[i] =
                                _ux_utility_long_get(descriptor + 26 + i * sizeof(ULONG));
                        }

                        /* We are done here. */
                        return(UX_SUCCESS);
                    }
                    
                    break;
            
            }                    
        }

        /* Verify if the descriptor is still valid.  */
        if (descriptor_length > total_descriptor_length)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_DESCRIPTOR_CORRUPTED);

            /* If trace is enabled, insert this event into the trace buffer.  */
            //UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, descriptor, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_DESCRIPTOR_CORRUPTED);
        }            

        /* Jump to the next descriptor if we have not reached the end.  */
        descriptor +=  descriptor_length;

        /* And adjust the length left to parse in the descriptor.  */
        total_descriptor_length -=  descriptor_length;
    }

    /* Error trap. */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_VIDEO_WRONG_TYPE);

    /* We get here when either the report descriptor has a problem or we could
       not find the right video device.  */
    return(UX_HOST_CLASS_VIDEO_WRONG_TYPE);
}

