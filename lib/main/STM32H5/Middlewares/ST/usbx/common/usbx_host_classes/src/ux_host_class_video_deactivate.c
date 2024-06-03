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
/*    _ux_host_class_video_deactivate                     PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is called when this instance of the video has been    */
/*    removed from the bus either directly or indirectly. The iso pipes   */ 
/*    will be destroyed and the instanced removed.                        */ 
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
/*    _ux_host_stack_class_instance_destroy Destroy class instance        */ 
/*    _ux_host_stack_endpoint_transfer_abort                              */
/*                                          Abort outstanding transfer    */ 
/*    _ux_host_semaphore_get                Get semaphore                 */ 
/*    _ux_host_semaphore_delete             Delete semaphore              */ 
/*    _ux_utility_memory_free               Release memory block          */ 
/*    _ux_utility_thread_schedule_other     Schedule other threads        */
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
/*                                            fixed issue of aborting     */
/*                                            transfer without endpoint   */
/*                                            ready, deleted new semaphore*/
/*                                            for control requests,       */
/*                                            resulting in version 6.1    */
/*  11-09-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            freed descriptor memory,    */
/*                                            resulting in version 6.1.2  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_video_deactivate(UX_HOST_CLASS_COMMAND *command)
{

UX_HOST_CLASS_VIDEO     *video;
UINT                    status;

    /* Get the instance for this class.  */
    video= (UX_HOST_CLASS_VIDEO *) command -> ux_host_class_command_instance;

    /* The video is being shut down.  */
    video -> ux_host_class_video_state =  UX_HOST_CLASS_INSTANCE_SHUTDOWN;

    /* Protect thread reentry to this instance.  */
    status =  _ux_host_semaphore_get(&video -> ux_host_class_video_semaphore, UX_WAIT_FOREVER);
    if (status != UX_SUCCESS)

        /* Return error.  */
        return(status);
    
    /* We need to abort transactions on the iso out pipe.  */
    if (video -> ux_host_class_video_isochronous_endpoint)
        _ux_host_stack_endpoint_transfer_abort(video -> ux_host_class_video_isochronous_endpoint);

    /* The enumeration thread needs to sleep a while to allow the application or the class that may be using
       endpoints to exit properly.  */
    _ux_host_thread_schedule_other(UX_THREAD_PRIORITY_ENUM); 

    /* Free descriptor memory.  */
    if (video -> ux_host_class_video_configuration_descriptor)
        _ux_utility_memory_free(video -> ux_host_class_video_configuration_descriptor);

    /* Destroy the instance.  */
    _ux_host_stack_class_instance_destroy(video -> ux_host_class_video_class, (VOID *) video);

    /* Destroy the semaphores.  */
    _ux_host_semaphore_delete(&video -> ux_host_class_video_semaphore);
    _ux_host_semaphore_delete(&video -> ux_host_class_video_semaphore_control_request);

    /* Before we free the device resources, we need to inform the application
        that the device is removed.  */
    if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
    {
        
        /* Inform the application the device is removed.  */
        _ux_system_host -> ux_system_host_change_function(UX_DEVICE_REMOVAL, video -> ux_host_class_video_class, (VOID *) video);
    }
    
    /* If trace is enabled, insert this event into the trace buffer.  */
    //UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_VIDEO_DEACTIVATE, video, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    //UX_TRACE_OBJECT_UNREGISTER(video);

    /* Free the video instance memory.  */
    _ux_utility_memory_free(video);
    
    /* Return successful completion.  */
    return(UX_SUCCESS);         
}

