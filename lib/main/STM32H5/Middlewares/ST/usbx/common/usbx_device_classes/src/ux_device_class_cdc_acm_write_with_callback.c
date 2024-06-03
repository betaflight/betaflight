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
/**   Device CDC Class                                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_cdc_acm_write_with_callback        PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function writes to  the CDC class with callback                */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    cdc_acm                               Address of cdc_acm class      */
/*                                                instance                */
/*    buffer                                Pointer to data to write      */
/*    requested_length                      Length of bytes to write      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*   _ux_device_stack_transfer_request                                    */
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
/*                                            used UX prefix to refer to  */
/*                                            TX symbols instead of using */
/*                                            them directly,              */
/*                                            resulting in version 6.1    */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added macro to disable      */
/*                                            transmission support,       */
/*                                            resulting in version 6.1.6  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_cdc_acm_write_with_callback(UX_SLAVE_CLASS_CDC_ACM *cdc_acm, UCHAR *buffer,
                                ULONG requested_length)
{
#ifdef UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE
    UX_PARAMETER_NOT_USED(cdc_acm);
    UX_PARAMETER_NOT_USED(buffer);
    UX_PARAMETER_NOT_USED(requested_length);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else
UX_SLAVE_DEVICE             *device;
UINT                        status;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_CDC_ACM_WRITE, cdc_acm, buffer, requested_length, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* As long as the device is in the CONFIGURED state.  */
    if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CONFIGURATION_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONFIGURATION_HANDLE_UNKNOWN, device, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Cannot proceed with command, the interface is down.  */
        return(UX_CONFIGURATION_HANDLE_UNKNOWN);
    }

    /* Are we already in transmission mode ?  */
    if (cdc_acm -> ux_slave_class_cdc_acm_transmission_status != UX_TRUE)
    {

        /* We should not to that ! */
        return(UX_ERROR);

    }

    /* Have we already scheduled a buffer ?   */
    if (cdc_acm -> ux_slave_class_cdc_acm_scheduled_write == UX_TRUE)
    {

        /* We should not to that ! */
        return(UX_ERROR);
    }

#if defined(UX_DEVICE_STANDALONE)

    /* Save the length to be sent. */
    cdc_acm -> ux_device_class_cdc_acm_write_requested_length = requested_length;

    /* And the buffer pointer.  */
    cdc_acm -> ux_device_class_cdc_acm_write_buffer = buffer;

    /* Schedule a transmission.  */
    cdc_acm -> ux_slave_class_cdc_acm_scheduled_write = UX_TRUE;

    /* Status success.  */
    status = (UX_SUCCESS);
#else

    /* Save the length to be sent. */
    cdc_acm -> ux_slave_class_cdc_acm_callback_total_length = requested_length;

    /* And the buffer pointer.  */
    cdc_acm -> ux_slave_class_cdc_acm_callback_data_pointer = buffer;

    /* Schedule a transmission.  */
    cdc_acm -> ux_slave_class_cdc_acm_scheduled_write = UX_TRUE;

    /* Invoke the bulkin thread by sending a flag .  */
    status = _ux_device_event_flags_set(&cdc_acm -> ux_slave_class_cdc_acm_event_flags_group, UX_DEVICE_CLASS_CDC_ACM_WRITE_EVENT, UX_OR);
#endif

    /* Simply return the last function result.  When we leave this function, the deferred writing has been scheduled. */
    return(status);
#endif
}

