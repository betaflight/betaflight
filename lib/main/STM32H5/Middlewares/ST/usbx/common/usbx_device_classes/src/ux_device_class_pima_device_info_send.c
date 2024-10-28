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
/**   Device Pima Class                                                   */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_pima.h"
#include "ux_device_stack.h"

/* Basic buffer length check: larger than DeviceInfo with all string and array 0.  */
#if UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH < 35
#error UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH too small
#endif

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_pima_device_info_send              PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function returns the device info structure to the host.        */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    pima                                  Pointer to pima class         */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_transfer_request     Transfer request              */
/*    _ux_utility_long_put                  Put 32-bit value              */
/*    _ux_utility_short_put                 Put 16-bit value              */
/*    _ux_utility_memory_set                Set memory                    */
/*    _ux_utility_string_to_unicode         Ascii string to unicode       */
/*    _ux_device_class_pima_response_send   Send PIMA response            */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Device Storage Class                                                */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added sanity checks,        */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_pima_device_info_send(UX_SLAVE_CLASS_PIMA *pima)
{
UINT                    status;
UX_SLAVE_TRANSFER       *transfer_request;
ULONG                   device_info_length;
UCHAR                   *device_info;
UCHAR                   *device_info_pointer;
ULONG                   array_field_counter;
USHORT                  *array_pointer;


    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PIMA_DEVICE_INFO_SEND, pima, 0, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Obtain the pointer to the transfer request.  */
    transfer_request =  &pima -> ux_device_class_pima_bulk_in_endpoint -> ux_slave_endpoint_transfer_request;

    /* Obtain memory for this object info. We use the transfer request pre-allocated buffer.  */
    device_info =  transfer_request -> ux_slave_transfer_request_data_pointer;

    /* Fill in the data container type.  */
    _ux_utility_short_put(device_info + UX_DEVICE_CLASS_PIMA_DATA_HEADER_TYPE,
                            UX_DEVICE_CLASS_PIMA_CT_DATA_BLOCK);

    /* Fill in the data code.  */
    _ux_utility_short_put(device_info + UX_DEVICE_CLASS_PIMA_DATA_HEADER_CODE,
                            UX_DEVICE_CLASS_PIMA_OC_GET_DEVICE_INFO);

    /* Fill in the Transaction ID.  */
    _ux_utility_long_put(device_info + UX_DEVICE_CLASS_PIMA_DATA_HEADER_TRANSACTION_ID,
                            pima -> ux_device_class_pima_transaction_id);

    /* Fill in the standard version field.  */
    _ux_utility_short_put(device_info + UX_DEVICE_CLASS_PIMA_DEVICE_INFO_STANDARD_VERSION,
                            UX_DEVICE_CLASS_PIMA_STANDARD_VERSION);

    /* Fill in the vendor extension ID field.  */
    _ux_utility_long_put(device_info + UX_DEVICE_CLASS_PIMA_DEVICE_INFO_VENDOR_EXTENSION_ID,
                            UX_DEVICE_CLASS_PIMA_VENDOR_EXTENSION_ID);

    /* Fill in the vendor extension version.  */
    _ux_utility_short_put(device_info + UX_DEVICE_CLASS_PIMA_DEVICE_INFO_VENDOR_EXTENSION_VERSION,
                            UX_DEVICE_CLASS_PIMA_EXTENSION_VERSION);

    /* Validate buffer space available.  */
    UX_ASSERT(UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH >
                (UX_DEVICE_CLASS_PIMA_DEVICE_INFO_VENDOR_EXTENSION_DESC +
                 _ux_utility_string_length_get(_ux_device_class_pima_vendor_extension_descriptor) * 2 + 1));

    /* Fill in the vendor extension description.  */
    _ux_utility_string_to_unicode(_ux_device_class_pima_vendor_extension_descriptor,
                            device_info + UX_DEVICE_CLASS_PIMA_DEVICE_INFO_VENDOR_EXTENSION_DESC);

    /* Calculate the address of the dynamic area of the device info.  */
    device_info_pointer = device_info + UX_DEVICE_CLASS_PIMA_DEVICE_INFO_VENDOR_EXTENSION_DESC;

    /* Update the device info pointer after the Unicode string.  */
    device_info_pointer += (ULONG) *(device_info_pointer) * 2 + 1;

    /* Validate buffer space available.  */
    UX_ASSERT(UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH >
                ((ULONG)(device_info_pointer - device_info) + sizeof(USHORT)));

    /* Add the functional mode which is set to standard. */
    _ux_utility_short_put(device_info_pointer, UX_DEVICE_CLASS_PIMA_STANDARD_MODE);

    /* Update the device info pointer.  */
    device_info_pointer += sizeof(USHORT);

    /* Fill in the supported operations.  This table is defined locally. */
    array_field_counter = 0;
    while (_ux_device_class_pima_supported_operations[array_field_counter] != 0)
    {

        /* Validate buffer space available.  */
        UX_ASSERT(UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH >
                    ((ULONG)(device_info_pointer - device_info) + sizeof(ULONG) + (sizeof(USHORT) * array_field_counter)));

        /* Add one element.  */
        _ux_utility_short_put(device_info_pointer + sizeof(ULONG) + (sizeof(USHORT) * array_field_counter),
                                _ux_device_class_pima_supported_operations[array_field_counter]);

        /* Next element.  */
        array_field_counter++;
    }

    /* Put the length of this variable array at the beginning of the field.  */
    _ux_utility_long_put(device_info_pointer, array_field_counter);

    /* Update the device info pointer.  */
    device_info_pointer += sizeof(ULONG) + (sizeof(USHORT) * array_field_counter);

    /* Fill in the supported events.  This table is defined locally. */
    array_field_counter = 0;
    while (_ux_device_class_pima_supported_events[array_field_counter] != 0)
    {

        /* Validate buffer space available.  */
        UX_ASSERT(UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH >
                    ((ULONG)(device_info_pointer - device_info) + sizeof(ULONG) + (sizeof(USHORT) * array_field_counter)));

        /* Add one element.  */
        _ux_utility_short_put(device_info_pointer + sizeof(ULONG) + (sizeof(USHORT) * array_field_counter),
                                _ux_device_class_pima_supported_events[array_field_counter]);

        /* Next element.  */
        array_field_counter++;
    }

    /* Put the length of this variable array at the beginning of the field.  */
    _ux_utility_long_put(device_info_pointer, array_field_counter);

    /* Update the device info pointer.  */
    device_info_pointer += sizeof(ULONG) + (sizeof(USHORT) * array_field_counter);

    /* Fill in the supported properties.  This table can be configured by the user. If it is declared NULL, use the local default one. */
    array_pointer = pima -> ux_device_class_pima_device_properties_list;

    /* Check if array defined.  */
    if (array_pointer == UX_NULL)

        /* Use local array.  */
        array_pointer = _ux_device_class_pima_device_prop_supported;

    /* Reset counter.  */
    array_field_counter = 0;

    /* Parse the table. */
    while (*(array_pointer + array_field_counter) != 0)
    {

        /* Validate buffer space available.  */
        UX_ASSERT(UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH >
                    ((ULONG)(device_info_pointer - device_info) + sizeof(ULONG) + (sizeof(USHORT) * array_field_counter)));

        /* Add one element.  */
        _ux_utility_short_put(device_info_pointer + sizeof(ULONG) + (sizeof(USHORT) * array_field_counter),
                                *(array_pointer + array_field_counter));

        /* Next element.  */
        array_field_counter++;
    }

    /* Put the length of this variable array at the beginning of the field.  */
    _ux_utility_long_put(device_info_pointer, array_field_counter);

    /* Update the device info pointer.  */
    device_info_pointer += sizeof(ULONG) + (sizeof(USHORT) * array_field_counter);

    /* Fill in the supported capture formats.  This table can be configured by the user. If it is declared NULL, use the local default one. */
    array_pointer = pima -> ux_device_class_pima_supported_capture_formats_list;

    /* Check if array defined.  */
    if (array_pointer == UX_NULL)

        /* Use local array.  */
        array_pointer = _ux_device_class_pima_supported_capture_formats;

    /* Reset counter.  */
    array_field_counter = 0;

    /* Parse the table. */
    while (*(array_pointer + array_field_counter) != 0)
    {

        /* Validate buffer space available.  */
        UX_ASSERT(UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH >
                    ((ULONG)(device_info_pointer - device_info) + sizeof(ULONG) + (sizeof(USHORT) * array_field_counter)));

        /* Add one element.  */
        _ux_utility_short_put(device_info_pointer + sizeof(ULONG) + (sizeof(USHORT) * array_field_counter),
                                *(array_pointer + array_field_counter));

        /* Next element.  */
        array_field_counter++;
    }

    /* Put the length of this variable array at the beginning of the field.  */
    _ux_utility_long_put(device_info_pointer, array_field_counter);

    /* Update the device info pointer.  */
    device_info_pointer += sizeof(ULONG) + (sizeof(USHORT) * array_field_counter);

    /* Fill in the supported image formats.  This table can be configured by the user. If it is declared NULL, use the local default one. */
    array_pointer = pima -> ux_device_class_pima_supported_image_formats_list;

    /* Check if array defined.  */
    if (array_pointer == UX_NULL)

        /* Use local array.  */
        array_pointer = _ux_device_class_pima_supported_image_formats;

    /* Reset counter.  */
    array_field_counter = 0;

    /* Parse the table. */
    while (*(array_pointer + array_field_counter) != 0)
    {

        /* Validate buffer space available.  */
        UX_ASSERT(UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH >
                    ((ULONG)(device_info_pointer - device_info) + sizeof(ULONG) + (sizeof(USHORT) * array_field_counter)));

        /* Add one element.  */
        _ux_utility_short_put(device_info_pointer + sizeof(ULONG) + (sizeof(USHORT) * array_field_counter),
                                *(array_pointer + array_field_counter));

        /* Next element.  */
        array_field_counter++;
    }

    /* Put the length of this variable array at the beginning of the field.  */
    _ux_utility_long_put(device_info_pointer, array_field_counter);

    /* Update the device info pointer.  */
    device_info_pointer += sizeof(ULONG) + (sizeof(USHORT) * array_field_counter);

    /* Validate buffer space available.  */
    UX_ASSERT(UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH >
                ((ULONG)(device_info_pointer - device_info) +
                 _ux_utility_string_length_get(pima -> ux_device_class_pima_manufacturer) * 2 + 1));

    /* Fill in the manufacturer string.  */
    _ux_utility_string_to_unicode(pima -> ux_device_class_pima_manufacturer, device_info_pointer);

    /* Update the device info pointer.  */
    device_info_pointer += (ULONG) (*device_info_pointer * 2) + 1;

    /* Validate buffer space available.  */
    UX_ASSERT(UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH >
                ((ULONG)(device_info_pointer - device_info) +
                 _ux_utility_string_length_get(pima -> ux_device_class_pima_model) * 2 + 1));

    /* Fill in the model string.  */
    _ux_utility_string_to_unicode(pima -> ux_device_class_pima_model, device_info_pointer);

    /* Update the device info pointer.  */
    device_info_pointer += (ULONG) (*device_info_pointer * 2) + 1;

    /* Validate buffer space available.  */
    UX_ASSERT(UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH >
                ((ULONG)(device_info_pointer - device_info) +
                 _ux_utility_string_length_get(pima -> ux_device_class_pima_device_version) * 2 + 1));

    /* Fill in the device version string.  */
    _ux_utility_string_to_unicode(pima -> ux_device_class_pima_device_version, device_info_pointer);

    /* Update the device info pointer.  */
    device_info_pointer += (ULONG) (*device_info_pointer * 2) + 1;

    /* Validate buffer space available.  */
    UX_ASSERT(UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH >
                ((ULONG)(device_info_pointer - device_info) +
                 _ux_utility_string_length_get(pima -> ux_device_class_pima_serial_number) * 2 + 1));

    /* Fill in the device serial number.  */
    _ux_utility_string_to_unicode(pima -> ux_device_class_pima_serial_number, device_info_pointer);

    /* Update the device info pointer.  */
    device_info_pointer += (ULONG) (*device_info_pointer * 2) + 1;

    /* Compute the overall length of the device info structure.  */
    device_info_length = (ULONG) ((ALIGN_TYPE) device_info_pointer - (ALIGN_TYPE) device_info);

    /* Set the transfer data pointer.  */
    transfer_request -> ux_slave_transfer_request_data_pointer =  device_info;

    /* Fill in the size of the response header.  */
    _ux_utility_long_put(device_info + UX_DEVICE_CLASS_PIMA_DATA_HEADER_LENGTH,
                            device_info_length);

    /* Send a data payload with the device info data set.  */
    status =  _ux_device_stack_transfer_request(transfer_request, device_info_length, 0);

    /* Now we return a response with success.  */
    _ux_device_class_pima_response_send(pima, UX_DEVICE_CLASS_PIMA_RC_OK, 0, 0, 0, 0);

    /* Return completion status.  */
    return(status);
}
