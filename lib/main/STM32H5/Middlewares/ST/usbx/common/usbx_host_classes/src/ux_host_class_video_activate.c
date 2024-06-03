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

UCHAR _ux_system_class_video_interface_descriptor_structure[] =             {1,1,1,1,1,1,1,1};
UCHAR _ux_system_class_video_input_terminal_descriptor_structure[] =        {1,1,1,1,2,1,1};
UCHAR _ux_system_class_video_input_header_descriptor_structure[] =          {1,1,1,1,2,1,1,1,1,1,1,1};
UCHAR _ux_system_class_video_processing_unit_descriptor_structure[] =       {1,1,1,1,1,2,1,1};
UCHAR _ux_system_class_video_streaming_interface_descriptor_structure[] =   {1,1,1,1,1,1};
UCHAR _ux_system_class_video_streaming_endpoint_descriptor_structure[] =    {1,1,1,1,1,1};
UCHAR _ux_system_class_video_frame_descriptor_structure[] =                 {1,1,1,1,1,2,2,4,4,4,4,1};

UCHAR _ux_system_host_class_video_name[] =                                  "ux_host_class_video";

typedef struct UX_HOST_CLASS_VIDEO_DESCRIPTORS_PARSER_STRUCT
{
    UX_HOST_CLASS_VIDEO         *video;
    ULONG                        parsed_flags;
} UX_HOST_CLASS_VIDEO_DESCRIPTORS_PARSER;
#define UX_HOST_CLASS_VIDEO_DESCRIPTORS_PARSER_VC_HEADER       1
#define UX_HOST_CLASS_VIDEO_DESCRIPTORS_PARSER_VC_IT           2
#define UX_HOST_CLASS_VIDEO_DESCRIPTORS_PARSER_VC_PU           4
#define UX_HOST_CLASS_VIDEO_DESCRIPTORS_PARSER_VS_HEADER       8
#define UX_HOST_CLASS_VIDEO_DESCRIPTORS_PARSER_DONE            (1|2|4|8)


static UINT _ux_host_class_video_descriptors_parser(VOID  *arg,
                              UCHAR *packed_interface_descriptor,
                              UCHAR *packed_entity_descriptor) 
{

UX_HOST_CLASS_VIDEO_DESCRIPTORS_PARSER  *parser;
UX_HOST_CLASS_VIDEO                     *video;
UX_INTERFACE                            *streaming_interface;
UCHAR                                   bInCollection;
UCHAR                                   *baInterfaceNr;

    /* Get parse data.  */
    parser = (UX_HOST_CLASS_VIDEO_DESCRIPTORS_PARSER *)arg;

    /* Get video instance.  */
    video = parser -> video;

    if (packed_interface_descriptor[6] == UX_HOST_CLASS_VIDEO_SUBCLASS_CONTROL)
    {

        /* Parse VC descriptors.  */

        /* Locate control interface.  */
        if (video -> ux_host_class_video_control_interface_number == 0xFF)
        {

            /* Parse header.  */
            if (packed_entity_descriptor[2] == UX_HOST_CLASS_VIDEO_VC_HEADER)
            {

                /* Get streaming interface.  */
                streaming_interface = video -> ux_host_class_video_streaming_interface;

                /* Check if this the VC interface is expected.  */
                bInCollection = packed_entity_descriptor[11];
                baInterfaceNr = packed_entity_descriptor + 12;

                /* Validation:
                 * baInterfaceNr not exceeding current descriptor.
                 */
                if (packed_entity_descriptor[0] + packed_entity_descriptor < baInterfaceNr + bInCollection)
                    return(1);

                while(bInCollection)
                {

                    /* Streaming interface belongs to this control interface.  */
                    if (*baInterfaceNr == streaming_interface -> ux_interface_descriptor.bInterfaceNumber)
                    {
                        video -> ux_host_class_video_control_interface_number = packed_interface_descriptor[2];
                        parser -> parsed_flags |= UX_HOST_CLASS_VIDEO_DESCRIPTORS_PARSER_VC_HEADER;
                        return(0);
                    }

                    /* Next interface number in descriptor.  */
                    baInterfaceNr ++;
                    bInCollection --;
                }
            }

            /* Not expected interface, just try next descriptor.  */
            return(0);
        }
        else if (packed_interface_descriptor[2] == video -> ux_host_class_video_control_interface_number)
        {

            /* It's the expected VC interface.  */

            /* Parse UX_HOST_CLASS_VIDEO_VC_PROCESSING_UNIT as feature_unit_id.  */
            if (packed_entity_descriptor[2] == UX_HOST_CLASS_VIDEO_VC_PROCESSING_UNIT &&
                video -> ux_host_class_video_feature_unit_id == 0)
            {
                parser -> parsed_flags |= UX_HOST_CLASS_VIDEO_DESCRIPTORS_PARSER_VC_PU;

                /* Save as feature unit ID.  */
                video -> ux_host_class_video_feature_unit_id = packed_entity_descriptor[3];
            }

            /* Parse UX_HOST_CLASS_VIDEO_VC_INPUT_TERMINAL and type (first only).  */
            if (packed_entity_descriptor[2] == UX_HOST_CLASS_VIDEO_VC_INPUT_TERMINAL &&
                video -> ux_host_class_video_terminal_id == 0)
            {
                parser -> parsed_flags |= UX_HOST_CLASS_VIDEO_DESCRIPTORS_PARSER_VC_IT;

                /* Save the video terminal ID.  */
                video -> ux_host_class_video_terminal_id = packed_entity_descriptor[3];

                /* Save the video terminal type.  */
                video -> ux_host_class_video_terminal_type = _ux_utility_short_get(packed_entity_descriptor + 4);
            }
        }
    }
    else
    {

        /* Parse VS descriptors.  */
        if (packed_entity_descriptor[2] == UX_HOST_CLASS_VIDEO_VS_INPUT_HEADER)
        {
            parser -> parsed_flags |= UX_HOST_CLASS_VIDEO_DESCRIPTORS_PARSER_VS_HEADER;

            /* Get the number of formats.  */
            video -> ux_host_class_video_number_formats = packed_entity_descriptor[3];

            /* Get the length of formats.  */
            video -> ux_host_class_video_length_formats = _ux_utility_short_get(packed_entity_descriptor + 4);

            /* Save the descriptor where the formats reside.  */
            video -> ux_host_class_video_format_address = packed_entity_descriptor;
        }
    }
    return(0);
}


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_video_activate                       PORTABLE C      */ 
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function activates the video class. It may be called twice by */
/*     the same device if there is a video control interface to this      */ 
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
/*    _ux_host_class_video_configure        Configure the video class     */ 
/*    _ux_host_class_video_descriptor_get   Get video descriptor          */ 
/*    _ux_host_class_video_input_terminal_get                             */
/*                                          Get input terminal            */
/*    _ux_host_class_video_input_format_get Get input format              */
/*    _ux_host_class_video_control_list_get Get controls                  */
/*    _ux_host_stack_class_instance_create  Create class instance         */ 
/*    _ux_host_stack_class_instance_destroy Destroy class instance        */ 
/*    _ux_utility_memory_allocate           Allocate a memory block       */ 
/*    _ux_utility_memory_free               Free a memory block           */ 
/*    _ux_host_semaphore_create             Create protection semaphore   */ 
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
/*                                            used entities parsing API,  */
/*                                            created new semaphore to    */
/*                                            protect control requests,   */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            improved VC header check,   */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_video_activate(UX_HOST_CLASS_COMMAND *command)
{

UX_INTERFACE                            *interface_ptr;
UX_HOST_CLASS_VIDEO                     *video;
UINT                                    status;
UX_HOST_CLASS_VIDEO_DESCRIPTORS_PARSER  parser;


    /* The video is always activated by the interface descriptor and not the
       device descriptor.  */
    interface_ptr =  (UX_INTERFACE *) command -> ux_host_class_command_container;

    /* Check the subclass of the new device. If it is a Video Control Interface,
       we don't need to create an instance of this function. When we get the streaming interface,
       we will search the video control interface for the device.  */
    if (interface_ptr -> ux_interface_descriptor.bInterfaceSubClass == UX_HOST_CLASS_VIDEO_SUBCLASS_CONTROL)
        return(UX_SUCCESS);
    
    /* Obtain memory for this class instance.  */
    video =  (UX_HOST_CLASS_VIDEO *) _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HOST_CLASS_VIDEO));
    if (video == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Store the class container into this instance.  */
    video -> ux_host_class_video_class =  command -> ux_host_class_command_class_ptr;

    /* Store the interface container into the video class instance.  */
    video -> ux_host_class_video_streaming_interface =  interface_ptr;

    /* Store the device container into the video class instance.  */
    video -> ux_host_class_video_device =  interface_ptr -> ux_interface_configuration -> ux_configuration_device;

    /* This instance of the device must also be stored in the interface container.  */
    interface_ptr -> ux_interface_class_instance =  (VOID *) video;

    /* Create this class instance.  */
    _ux_host_stack_class_instance_create(video -> ux_host_class_video_class, (VOID *) video);
        
    /* Configure the video.  */
    status =  _ux_host_class_video_configure(video);     

    /* Get the video descriptor (all the class specific stuff) and memorize them
       as we will need these descriptors to change settings.  */
    if (status == UX_SUCCESS)
        status =  _ux_host_class_video_descriptor_get(video);

    /* Use parser to locate streaming terminal, formats and video controls.  */
    if (status == UX_SUCCESS)
    {
        parser.video = video;
        parser.parsed_flags = 0;
        video -> ux_host_class_video_control_interface_number = 0xFF;
        status = _ux_host_class_video_entities_parse(video,
                        _ux_host_class_video_descriptors_parser, (VOID *)&parser);
        if (parser.parsed_flags != UX_HOST_CLASS_VIDEO_DESCRIPTORS_PARSER_DONE)

            /* Some of expected descriptors not found.  */
            status = UX_HOST_CLASS_VIDEO_WRONG_TYPE;
    }

    /* Create the semaphore to protect multiple threads from accessing the same
       video instance.  */
    if (status == UX_SUCCESS)
    {
        status =  _ux_host_semaphore_create(&video -> ux_host_class_video_semaphore, "ux_video_semaphore", 1);
        if (status != UX_SUCCESS)
            status = UX_SEMAPHORE_ERROR;
    }

    /* Create the semaphore to protect multiple threads from issuing the control
       request at the same time.  */
    if (status == UX_SUCCESS)
    {
        status = _ux_host_semaphore_create(&video -> ux_host_class_video_semaphore_control_request, "ux_video_semaphore_control", 1);
        if (status != UX_SUCCESS)
            status = UX_SEMAPHORE_ERROR;
    }

    if (status == UX_SUCCESS)
    {

        /* Mark the video as live now.  */
        video -> ux_host_class_video_state =  UX_HOST_CLASS_INSTANCE_LIVE;

        /* If all is fine and the device is mounted, we may need to inform the application
        if a function has been programmed in the system structure.  */
        if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
        {
            
            /* Call system change function.  */
            _ux_system_host ->  ux_system_host_change_function(UX_DEVICE_INSERTION, video -> ux_host_class_video_class, (VOID *) video);
        }

        /* If trace is enabled, insert this event into the trace buffer.  */
        //UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_VIDEO_ACTIVATE, video, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

        /* If trace is enabled, register this object.  */
        UX_TRACE_OBJECT_REGISTER(UX_TRACE_HOST_OBJECT_TYPE_INTERFACE, video, 0, 0, 0)

        /* Return success.  */
        return(UX_SUCCESS);
    }

    /* There was error, free resources.  */

    /* The last resource, video -> ux_host_class_video_semaphore_control_request is not created or created error,
       no need to free.  */

    /* Destroy the semaphore.  */
    if (_ux_host_semaphore_created(&video -> ux_host_class_video_semaphore))
        _ux_host_semaphore_delete(&video -> ux_host_class_video_semaphore);

    /* Destroy the class instance.  */
    _ux_host_stack_class_instance_destroy(video -> ux_host_class_video_class, (VOID *) video);

    /* This instance of the device must also be cleared in the interface container.  */
    interface_ptr -> ux_interface_class_instance = UX_NULL;

    /* Free instance memory.  */
    _ux_utility_memory_free(video);

    /* Return completion status.  */
    return(status);    
}

