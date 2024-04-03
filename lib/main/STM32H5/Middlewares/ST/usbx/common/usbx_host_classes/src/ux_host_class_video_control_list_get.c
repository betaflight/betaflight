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
/*    _ux_host_class_video_control_list_get               PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function obtains the controls for the video device.            */
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
UINT  _ux_host_class_video_control_list_get(UX_HOST_CLASS_VIDEO *video)
{

UCHAR                                           *descriptor;
UX_INTERFACE_DESCRIPTOR                         interface_descriptor;
UX_HOST_CLASS_VIDEO_PROCESSING_UNIT_DESCRIPTOR  processing_unit_descriptor;
ULONG                                           total_descriptor_length;
ULONG                                           descriptor_length;
ULONG                                           descriptor_type;
ULONG                                           descriptor_subtype;
ULONG                                           interface_found;


    /* Get the descriptor to the selected format.  */
    descriptor =  video -> ux_host_class_video_configuration_descriptor;
    total_descriptor_length =  video -> ux_host_class_video_configuration_descriptor_length;
    
    /* Haven't found it yet.  */
    interface_found =  UX_FALSE;

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

        /* Process relative to descriptor type.  */
        switch (descriptor_type)
        {

            case UX_INTERFACE_DESCRIPTOR_ITEM:
    
                /* Parse the interface descriptor and make it machine independent.  */
                _ux_utility_descriptor_parse(descriptor, _ux_system_interface_descriptor_structure,
                                                UX_INTERFACE_DESCRIPTOR_ENTRIES, (UCHAR *) &interface_descriptor);
    
                /* Ensure we have the correct interface for Video Control.  */
                if ((interface_descriptor.bInterfaceClass == UX_HOST_CLASS_VIDEO_CLASS) &&
                    (interface_descriptor.bInterfaceSubClass == UX_HOST_CLASS_VIDEO_SUBCLASS_CONTROL))
                {
    
                    /* Mark we have found it.  */
                    interface_found =  UX_TRUE;

                    /* Get the interface number of this descriptor and save it in the video
                       instance. This will be useful to program the video controls.  */
                    video -> ux_host_class_video_control_interface_number =  interface_descriptor.bInterfaceNumber;
                }
                else
                {
    
                    /* Haven't found it.  */
                    interface_found =  UX_FALSE;
                }
                break;
            
                    
            case UX_HOST_CLASS_VIDEO_CS_INTERFACE:
    
                /* First make sure we have found the correct generic interface descriptor.  */
                if ((interface_found == UX_TRUE) && (descriptor_subtype == UX_HOST_CLASS_VIDEO_VC_PROCESSING_UNIT))
                {

                    /* Parse the interface descriptor and make it machine independent.  */
                    _ux_utility_descriptor_parse(descriptor, _ux_system_class_video_input_terminal_descriptor_structure,
                                                 UX_HOST_CLASS_VIDEO_PROCESSING_UNIT_DESCRIPTOR_ENTRIES, (UCHAR *) &processing_unit_descriptor);

                    video -> ux_host_class_video_feature_unit_id = processing_unit_descriptor.bUnitID;

                    /* We are done here.  */
                    return(UX_SUCCESS);
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

