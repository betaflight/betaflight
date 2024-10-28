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
/*    _ux_host_class_video_input_format_get               PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function finds the input format(s) for the input terminal.     */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    video                                 Pointer to video class        */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_descriptor_parse          Parse descriptor              */ 
/*    _ux_system_error_handler              Log system error              */ 
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_video_input_format_get(UX_HOST_CLASS_VIDEO *video)
{

UCHAR                                           *descriptor;
UX_INTERFACE_DESCRIPTOR                         interface_descriptor;
UX_HOST_CLASS_VIDEO_INPUT_HEADER_DESCRIPTOR     input_header_descriptor;
ULONG                                           total_descriptor_length;
ULONG                                           descriptor_length;
ULONG                                           descriptor_type;
ULONG                                           descriptor_subtype;
ULONG                                           descriptor_found;


    /* Get the descriptor to the entire configuration.  */
    descriptor =               video -> ux_host_class_video_configuration_descriptor;
    total_descriptor_length =  video -> ux_host_class_video_configuration_descriptor_length;
    
    /* Default is Interface descriptor not yet found.  */    
    descriptor_found =  UX_FALSE;
    
    /* Scan the descriptor for the Video Streaming interface.  */
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

        /* Process relative to the descriptor type.  */
        switch (descriptor_type)
        {


        case UX_INTERFACE_DESCRIPTOR_ITEM:

            /* Parse the interface descriptor and make it machine independent.  */
            _ux_utility_descriptor_parse(descriptor, _ux_system_interface_descriptor_structure,
                                            UX_INTERFACE_DESCRIPTOR_ENTRIES, (UCHAR *) &interface_descriptor);

            /* Ensure we have the correct interface for Video Streaming.  */
            if ((interface_descriptor.bInterfaceClass == UX_HOST_CLASS_VIDEO_CLASS) &&
                (interface_descriptor.bInterfaceSubClass == UX_HOST_CLASS_VIDEO_SUBCLASS_STREAMING))
            {

                /* Mark we have found it.  */
                descriptor_found =  UX_TRUE;
            }
            else
            {

                descriptor_found =  UX_FALSE;
            }
            break;
                

        case UX_HOST_CLASS_VIDEO_CS_INTERFACE:

            /* First make sure we have found the correct generic interface descriptor.  */
            if (descriptor_found == UX_TRUE)
            {

                /* Check the sub type.  */
                switch (descriptor_subtype)
                {


                case UX_HOST_CLASS_VIDEO_VS_INPUT_HEADER:
                                        
                    /* Make the descriptor machine independent.  */
                    _ux_utility_descriptor_parse(descriptor, _ux_system_class_video_input_header_descriptor_structure,
                                                 UX_HOST_CLASS_VIDEO_INPUT_HEADER_DESCRIPTOR_ENTRIES, (UCHAR *) &input_header_descriptor);
                                                        
                    /* Get the number of formats.  */
                    video -> ux_host_class_video_number_formats = input_header_descriptor.bNumFormats;

                    /* Get the length of formats.  */
                    video -> ux_host_class_video_length_formats = input_header_descriptor.wTotalLength;

                    /* Save the descriptor where the formats reside.  */
                    video -> ux_host_class_video_format_address = descriptor;

                    /* We are done here.  */
                    return(UX_SUCCESS);

                }
            }
            break;
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

