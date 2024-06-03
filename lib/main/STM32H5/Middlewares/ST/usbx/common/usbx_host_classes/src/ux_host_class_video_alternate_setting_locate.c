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
/*    _ux_host_class_video_alternate_setting_locate       PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function finds the alternate setting for the specific format.  */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    video                                 Pointer to video class        */ 
/*    alternate_setting                     Pointer to located alternate  */ 
/*                                            setting                     */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
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
/*  10-15-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            use pre-calculated value    */
/*                                            instead of wMaxPacketSize,  */
/*                                            resulting in version 6.1.9  */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_video_alternate_setting_locate(UX_HOST_CLASS_VIDEO *video, UINT max_payload_size, UINT *alternate_setting)
{

UX_CONFIGURATION        *configuration;
UX_INTERFACE            *interface_ptr;
UX_ENDPOINT             *endpoint;
UINT                    streaming_interface;
UINT                    payload_size;
UINT                    current_payload_size = 0;
UINT                    alternate_setting_found;


    configuration =        video -> ux_host_class_video_streaming_interface -> ux_interface_configuration;
    interface_ptr =            configuration -> ux_configuration_first_interface;
    streaming_interface =  video -> ux_host_class_video_streaming_interface -> ux_interface_descriptor.bInterfaceNumber;

    alternate_setting_found = UX_FALSE;

    /* Scan all interfaces.  */
    while (interface_ptr != UX_NULL)
    {

        /* Search for the streaming interface with a endpoint.  */
        if ((interface_ptr -> ux_interface_descriptor.bInterfaceNumber == streaming_interface) &&
            (interface_ptr -> ux_interface_first_endpoint != 0))
        {
            
            /* Get the max packet size of the endpoint.  */
            endpoint = interface_ptr -> ux_interface_first_endpoint;
            payload_size = endpoint -> ux_endpoint_transfer_request.ux_transfer_request_packet_length;

            /* Check if the payload size is equal or greater than the required payload size.  */
            if (payload_size >= max_payload_size)
            {

                /* Check if we have found any alternate settings yet.  */
                if(alternate_setting_found == UX_FALSE)
                {

                    /* Save the current payload size and mark the found flag.  */
                    alternate_setting_found = UX_TRUE;
                    current_payload_size = payload_size;

                    /* Save the found alternate setting number.  */
                    *alternate_setting = interface_ptr -> ux_interface_descriptor.bAlternateSetting;
                }
                else
                {

                    /* Check if the payload size is smaller.  */
                    if (payload_size < current_payload_size)
                    {

                        /* Smaller payload size found, select the current setting.  */
                        current_payload_size = payload_size;

                        /* Save the found alternate setting number.  */
                        *alternate_setting = interface_ptr -> ux_interface_descriptor.bAlternateSetting;
                    }
                }
            }
        }

        /* Move to next interface.  */
        interface_ptr =  interface_ptr -> ux_interface_next_interface;
    }

    /* Check if we found the alternate setting.  */
    if (alternate_setting_found)
    {

        /* Return successful completion.  */
        return(UX_SUCCESS);
    }

    /* Error trap. */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_VIDEO_WRONG_TYPE);

    /* We get here when either the report descriptor has a problem or we could
       not find the right video device.  */
    return(UX_HOST_CLASS_VIDEO_WRONG_TYPE);
}
