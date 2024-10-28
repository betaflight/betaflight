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
/**   Device Video Class                                                  */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_video.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_video_control_request              PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function manages the based sent by the host on the control     */
/*    endpoints with a CLASS or VENDOR SPECIFIC type.                     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    command                               Pointer to class command      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_endpoint_stall       Endpoint stall                */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Device Video Class                                                  */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  04-25-2022     Chaoqiong Xiao           Initial Version 6.1.11        */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_video_control_request(UX_SLAVE_CLASS_COMMAND *command)
{

UX_SLAVE_TRANSFER             *transfer_request;
UX_SLAVE_DEVICE               *device;
UX_SLAVE_CLASS                *class_inst;
UX_DEVICE_CLASS_VIDEO         *video;
UX_DEVICE_CLASS_VIDEO_STREAM  *stream;
ULONG                         stream_index;
UCHAR                         request_type;
UCHAR                         request;
UCHAR                         index_low;
ULONG                         value_cs;
UCHAR                         *buffer;
UINT                          status;


    /* Get the class container.  */
    class_inst =  command -> ux_slave_class_command_class_ptr;

    /* Get the video instance from this class container.  */
    video =  (UX_DEVICE_CLASS_VIDEO *) class_inst -> ux_slave_class_instance;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Get the pointer to the transfer request associated with the control endpoint.  */
    transfer_request =  &device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

    /* Get bmRequestType, wValue and wIndex low byte.  */
    request_type = transfer_request -> ux_slave_transfer_request_setup[UX_SETUP_REQUEST_TYPE];
    request = transfer_request -> ux_slave_transfer_request_setup[UX_SETUP_REQUEST];
    value_cs = transfer_request -> ux_slave_transfer_request_setup[UX_SETUP_VALUE + 1];
    index_low = transfer_request -> ux_slave_transfer_request_setup[UX_SETUP_INDEX];
    buffer = transfer_request -> ux_slave_transfer_request_data_pointer;

    /* Check request target (only target interface is supported now).  */
    if ((request_type & UX_REQUEST_TARGET) != UX_REQUEST_TARGET_INTERFACE)
        return(UX_ERROR);

    /* Check request interface (wIndex low) of control.  */
    if (index_low == video -> ux_device_class_video_interface -> ux_slave_interface_descriptor.bInterfaceNumber)
    {

        /* Check control selector.  */
        switch(value_cs)
        {
        case UX_DEVICE_CLASS_VIDEO_VC_REQUEST_ERROR_CODE_CONTROL:

            /* Must have at least 1 byte buffer.  */
            if (transfer_request->ux_slave_transfer_request_requested_length < 1)
                return(UX_ERROR);

            /* GET_CUR.  */
            if (request == UX_DEVICE_CLASS_VIDEO_GET_CUR)
            {

                /* Fill bRequestErrorCode.  */
                *buffer = (UCHAR)video -> ux_device_class_video_error;

                /* Send data.  */
                status = _ux_device_stack_transfer_request(transfer_request, 1, 1);
                return(status);
            }

            /* GET_INFO.  */
            if (request == UX_DEVICE_CLASS_VIDEO_GET_INFO)
            {

                /* Support GET_CUR | GET_INFO.  */
                *buffer = UX_DEVICE_CLASS_VIDEO_INFO_GET_REQUEST_SUPPORT;

                /* Send data.  */
                status = _ux_device_stack_transfer_request(transfer_request, 1, 1);
                return(status);
            }

            /* Request not expected.  */
            return(UX_ERROR);

        default:
            break;
        }

        /* By default it's not handled.  */
        status = UX_ERROR;

        /* Invoke callback.  */
        if (video -> ux_device_class_video_callbacks.ux_device_class_video_request != UX_NULL)
        {

            /* Handled by callback.  */
            status = video -> ux_device_class_video_callbacks.ux_device_class_video_request(video, transfer_request);
        }
        return(status);
    }

    /* Check request index of stream.  */
    stream = video -> ux_device_class_video_streams;
    for (stream_index = 0; stream_index < video -> ux_device_class_video_streams_nb; stream_index ++)
    {

        /* No stream.  */
        if (stream -> ux_device_class_video_stream_interface == UX_NULL)
            break;

        /* Check interface number.  */
        if (index_low == stream -> ux_device_class_video_stream_interface ->
                                ux_slave_interface_descriptor.bInterfaceNumber)
        {

            /* Check control selector.  */
            switch(value_cs)
            {
            case UX_DEVICE_CLASS_VIDEO_VS_STREAM_ERROR_CODE_CONTROL:

                /* Must have at least 1 byte buffer.  */
                if (transfer_request->ux_slave_transfer_request_requested_length < 1)
                    return(UX_ERROR);

                /* GET_CUR.  */
                if (request == UX_DEVICE_CLASS_VIDEO_GET_CUR)
                {

                    /* Fill bRequestErrorCode.  */
                    *buffer = (UCHAR)stream -> ux_device_class_video_stream_error;

                    /* Send data.  */
                    status = _ux_device_stack_transfer_request(transfer_request, 1, 1);
                    return(status);
                }

                /* GET_INFO.  */
                if (request == UX_DEVICE_CLASS_VIDEO_GET_INFO)
                {

                    /* Support GET_CUR | GET_INFO.  */
                    *buffer = UX_DEVICE_CLASS_VIDEO_INFO_GET_REQUEST_SUPPORT;

                    /* Send data.  */
                    status = _ux_device_stack_transfer_request(transfer_request, 1, 1);
                    return(status);
                }

                /* Request not expected.  */
                return(UX_ERROR);

            default:
                break;
            }

            /* By default it's not handled.  */
            status = UX_ERROR;

            /* Invoke callback.  */
            if (stream -> ux_device_class_video_stream_callbacks.ux_device_class_video_stream_request != UX_NULL)
            {

                /* Handled by callback.  */
                status = stream -> ux_device_class_video_stream_callbacks.ux_device_class_video_stream_request(stream, transfer_request);
            }
            return(status);
        }

        /* Next stream.  */
        stream ++;
    }

    /* Not handled.  */
    return(UX_ERROR);
}
