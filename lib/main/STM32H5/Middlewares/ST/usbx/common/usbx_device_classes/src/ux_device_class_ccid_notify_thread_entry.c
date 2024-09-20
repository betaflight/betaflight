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
/**   Device CCID Class                                                   */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"

#if !defined(UX_DEVICE_STANDALONE)
#include "ux_device_class_ccid.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_ccid_notify_thread_entry           PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is the thread of the CCID interrupt IN endpoint. It   */
/*    is waiting for the application event to start interrupt IN.         */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    ccid_inst                             Address of ccid class         */
/*                                            container (32-bit)          */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_transfer_request     Request transfer              */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    ThreadX                                                             */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  04-25-2022     Chaoqiong Xiao           Initial Version 6.1.11        */
/*                                                                        */
/**************************************************************************/
VOID  _ux_device_class_ccid_notify_thread_entry(ULONG ccid_inst)
{

UX_SLAVE_DEVICE                                     *device;
UX_DEVICE_CLASS_CCID                                *ccid;
UX_DEVICE_CLASS_CCID_SLOT                           *slot;
UX_DEVICE_CLASS_CCID_PARAMETER                      *parameter;
UX_SLAVE_ENDPOINT                                   *endpoint;
UX_SLAVE_TRANSFER                                   *transfer;
UCHAR                                               *buffer;
ULONG                                               length;
INT                                                 i;
UCHAR                                               icc_mask;
UINT                                                byte_pos, bits_pos;
UINT                                                status;

    /* Cast properly the ccid instance.  */
    UX_THREAD_EXTENSION_PTR_GET(ccid, UX_DEVICE_CLASS_CCID, ccid_inst)

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* This thread runs forever but can be suspended or resumed by the user application.  */
    status = UX_SUCCESS;
    while(1)
    {

        /* Error cases.  */
        if (status != UX_SUCCESS)
        {

            /* We need to suspend ourselves. We will be resumed by the
               application if needed.  */
            _ux_utility_thread_suspend(&ccid -> ux_device_class_ccid_notify_thread);
            status = UX_SUCCESS;
            continue;
        }
        status = UX_ERROR;

        /* Check device state.  */
        if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
            continue;

        /* Check endpoint.  */
        endpoint = ccid -> ux_device_class_ccid_endpoint_notify;
        if (endpoint == UX_NULL)
            continue;
        transfer = &endpoint -> ux_slave_endpoint_transfer_request;

        /* Wait event.  */
        status = _ux_utility_semaphore_get(&ccid -> ux_device_class_ccid_notify_semaphore,
                                           UX_WAIT_FOREVER);
        if (status != UX_SUCCESS)
            continue;

        /* Build slot change/hardware error message.  */
        buffer = transfer -> ux_slave_transfer_request_data_pointer;
        length = 0;

        /* By default no message.  */
        buffer[UX_DEVICE_CLASS_CCID_OFFSET_MESSAGE_TYPE] = 0;

        /* Build slot hardware error message.  */
        parameter = &ccid -> ux_device_class_ccid_parameter;

        _ux_device_mutex_on(&ccid -> ux_device_class_ccid_mutex);

        slot = ccid -> ux_device_class_ccid_slots;
        for (i = 0; i < parameter -> ux_device_class_ccid_max_n_slots; i ++)
        {

            /* Check if there is hardware error notification.  */
            if (slot -> ux_device_class_ccid_slot_flags &
                UX_DEVICE_CLASS_CCID_FLAG_NOTIFY_HW_ERROR)
            {
                slot -> ux_device_class_ccid_slot_flags &=
                                (UCHAR)~UX_DEVICE_CLASS_CCID_FLAG_NOTIFY_HW_ERROR;
                buffer[UX_DEVICE_CLASS_CCID_OFFSET_MESSAGE_TYPE] =
                                UX_DEVICE_CLASS_CCID_RDR_TO_PC_HARDWARE_ERROR;
                buffer[UX_DEVICE_CLASS_CCID_OFFSET_HW_ERROR_SLOT] = (UCHAR)i;
                buffer[UX_DEVICE_CLASS_CCID_OFFSET_HW_ERROR_SEQ] =
                                slot -> ux_device_class_ccid_slot_hw_error_seq;
                buffer[UX_DEVICE_CLASS_CCID_OFFSET_HW_ERROR_CODE] =
                                slot -> ux_device_class_ccid_slot_hw_error;
                length = 4;
                break;
            }

            /* Next slot.  */
            slot ++;
        }

        /* Build slot changes message.  */
        if (buffer[UX_DEVICE_CLASS_CCID_OFFSET_MESSAGE_TYPE] == 0)
        {

            /* Scan slots.  */
            slot = ccid -> ux_device_class_ccid_slots;
            for (i = 0, byte_pos = 1; ; byte_pos ++)
            {
                /* Reset bits.  */
                buffer[byte_pos] = 0;

                /* Scan 4 slots.  */
                for(bits_pos = 0; bits_pos < 8; bits_pos += 2)
                {

                    /* Slot state bit.  */
                    icc_mask = (UCHAR)((slot -> ux_device_class_ccid_slot_icc_status ==
                            UX_DEVICE_CLASS_CCID_SLOT_STATUS_ICC_NOT_PRESENT) ?
                            0u : 1u);

                    /* Check if there is change notification.  */
                    if (slot -> ux_device_class_ccid_slot_flags &
                        UX_DEVICE_CLASS_CCID_FLAG_NOTIFY_CHANGE)
                    {
                        slot -> ux_device_class_ccid_slot_flags &=
                                    (UCHAR)~UX_DEVICE_CLASS_CCID_FLAG_NOTIFY_CHANGE;

                        /* Message type.  */
                        buffer[UX_DEVICE_CLASS_CCID_OFFSET_MESSAGE_TYPE] =
                            UX_DEVICE_CLASS_CCID_RDR_TO_PC_NOTIFY_SLOT_CHANGE;

                        /* Slot change bit.  */
                        icc_mask |= (UCHAR)0x02u;
                    }

                    /* Modify bits.  */
                    buffer[byte_pos] |= (UCHAR)(icc_mask << bits_pos);

                    /* Next slot.  */
                    i ++;
                    if (i >= parameter -> ux_device_class_ccid_max_n_slots)
                        break;
                    slot ++;
                }
                if (i >= parameter -> ux_device_class_ccid_max_n_slots)
                    break;
            }

            /* Buffer length should include last byte.  */
            length = byte_pos + 1;
        }

        _ux_device_mutex_off(&ccid -> ux_device_class_ccid_mutex);

        /* Check message to see if there is message to send.  */
        if (buffer[UX_DEVICE_CLASS_CCID_OFFSET_MESSAGE_TYPE] == 0)
            continue;

        /* Send request.  */
        status = _ux_device_stack_transfer_request(transfer, length, length);
    }
}
#endif
