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
/*    _ux_host_class_video_configure                      PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function calls the USBX stack to do a SET_CONFIGURATION to    */
/*     the device. Once the device is configured, its interface(s) will   */
/*     be activated.                                                      */ 
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
/*    _ux_host_stack_configuration_interface_get                          */
/*                                          Get interface                 */ 
/*    _ux_host_stack_device_configuration_get                             */
/*                                          Get configuration             */ 
/*    _ux_host_stack_device_configuration_select                          */
/*                                          Select configuration          */  
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
/*                                            optimized based on compile  */
/*                                            definitions,                */
/*                                            resulting in version 6.1    */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_video_configure(UX_HOST_CLASS_VIDEO *video)
{

UINT                    status;
UX_CONFIGURATION        *configuration;
UX_INTERFACE            *interface;
ULONG                   interface_number;
#if UX_MAX_DEVICES > 1
UX_DEVICE               *parent_device;
#endif


    /* If the device has been configured already, we don't need to do it
       again. */
    if (video -> ux_host_class_video_device -> ux_device_state == UX_DEVICE_CONFIGURED)
        return(UX_SUCCESS);

    /* An video device normally has one configuration. So retrieve the 1st configuration
       only.  */
    status =  _ux_host_stack_device_configuration_get(video -> ux_host_class_video_device, 0, &configuration);
    if (status != UX_SUCCESS)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CONFIGURATION_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONFIGURATION_HANDLE_UNKNOWN, video -> ux_host_class_video_device, 0, 0, UX_TRACE_ERRORS, 0, 0)
    
        return(UX_CONFIGURATION_HANDLE_UNKNOWN);
     
    }

#if UX_MAX_DEVICES > 1
    /* Check the video power source and check the parent power source for 
       incompatible connections.  */
    if (video -> ux_host_class_video_device -> ux_device_power_source == UX_DEVICE_BUS_POWERED)
    {

        /* Get parent device.  */
        parent_device =  video -> ux_host_class_video_device -> ux_device_parent;
        
        /* If the device is NULL, the parent is the root video and we don't have to worry 
           if the parent is not the root video, check for its power source.  */
        if ((parent_device != UX_NULL) && (parent_device -> ux_device_power_source == UX_DEVICE_BUS_POWERED))
        {                        

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CONNECTION_INCOMPATIBLE);

            /* If trace is enabled, insert this event into the trace buffer.  */
            //UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONNECTION_INCOMPATIBLE, video, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_CONNECTION_INCOMPATIBLE);
        }            
    }
#endif

    /* We have the valid configuration. Ask the USBX stack to set this configuration */        
    status =  _ux_host_stack_device_configuration_select(configuration);
    if (status != UX_SUCCESS)
        return(status);

    /* Start with interface number 0.  */
    interface_number =  0;

    /* We only need to retrieve the video streaming interface.  */
    do 
    {

        /* Pickup interface.  */
        status =  _ux_host_stack_configuration_interface_get(configuration, (UINT) interface_number, 0, &interface);

        /* Check completion status.  */
        if (status == UX_SUCCESS)
        {

            /* Check the type of interface we have found - is it streaming?  */
            if (interface -> ux_interface_descriptor.bInterfaceSubClass == UX_HOST_CLASS_VIDEO_SUBCLASS_STREAMING)
            {
 
                video -> ux_host_class_video_streaming_interface =  interface;
                video -> ux_host_class_video_streaming_interface -> ux_interface_class_instance =  (VOID *)video;
                break;
            }
        }

        /* Move to next interface.  */
        interface_number++;
    } while(status == UX_SUCCESS);

    /* After we have parsed the video interfaces, ensure the streaming interfaces is resent.  */
    if (video -> ux_host_class_video_streaming_interface == UX_NULL)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_INTERFACE_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        //UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_INTERFACE_HANDLE_UNKNOWN, interface, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* We get here when we could not locate the interface(s) handle */
        return(UX_INTERFACE_HANDLE_UNKNOWN);
    }

    return(UX_SUCCESS);        

}

