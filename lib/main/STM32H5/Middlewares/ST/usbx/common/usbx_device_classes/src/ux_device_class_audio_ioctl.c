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
/**   Device Audio Class                                                  */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_audio.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_audio_ioctl                        PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function performs certain functions on the audio instance      */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    audio                                   Address of audio class      */
/*                                                instance                */
/*    ioctl_function                          IOCTL function code         */
/*    parameter                               Parameter for function      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Status                                                              */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    ThreadX                                                             */
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
UINT _ux_device_class_audio_ioctl(UX_DEVICE_CLASS_AUDIO *audio, ULONG ioctl_function,
                                    VOID *parameter)
{

UINT                                                status;
VOID                                                **pptr_parameter;


    /* Let's be optimist ! */
    status = UX_SUCCESS;

    /* The command request will tell us what we need to do here.  */
    switch (ioctl_function)
    {

        case UX_DEVICE_CLASS_AUDIO_IOCTL_GET_ARG:

            /* Properly cast the parameter pointer.  */
            pptr_parameter = (VOID **) parameter;

            /* Save argument.  */
            *pptr_parameter = audio -> ux_device_class_audio_callbacks.ux_device_class_audio_arg;

            break;

        default:

            /* Function not supported. Return an error.  */
            status =  UX_FUNCTION_NOT_SUPPORTED;
    }

    /* Return status to caller.  */
    return(status);

}

