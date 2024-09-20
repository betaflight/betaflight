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
/**   Audio Class                                                         */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_audio.h"
#include "ux_host_stack.h"


struct UX_HOST_CLASS_AUDIO10_SAM_PARSER {
    UINT    (*parse_function)(VOID  *arg,
                              UCHAR *packed_interface_descriptor,
                              UX_HOST_CLASS_AUDIO_SAMPLING_CHARACTERISTICS *sam_attr
                            );
    UCHAR   *arg;
    UX_HOST_CLASS_AUDIO
            *audio;
};
struct UX_HOST_CLASS_AUDIO20_SAM_PARSER {
    UINT    (*parse_function)(VOID  *arg,
                              UCHAR *packed_interface_descriptor,
                              UX_HOST_CLASS_AUDIO_SAMPLING_CHARACTERISTICS *sam_attr
                            );
    UCHAR   *arg;
    UX_HOST_CLASS_AUDIO
            *audio;
    UCHAR   *as_header;
    UINT    status;
};


static UINT _ux_host_class_audio10_sam_parse_func(VOID *arg,
                            UCHAR *packed_interface_descriptor,
                            UCHAR *packed_endpoint_descriptor,
                            UCHAR *packed_audio_descriptor);
static inline UINT _ux_host_class_audio10_sampling_parse(UX_HOST_CLASS_AUDIO *audio,
        UINT(*parse_function)(VOID  *arg,
                              UCHAR *packed_interface_descriptor,
                              UX_HOST_CLASS_AUDIO_SAMPLING_CHARACTERISTICS *sam_attr),
        VOID* arg);
#if defined(UX_HOST_CLASS_AUDIO_2_SUPPORT)
static UINT _ux_host_class_audio20_sam_parse_func(VOID *arg,
                            UCHAR *packed_interface_descriptor,
                            UCHAR *packed_endpoint_descriptor,
                            UCHAR *packed_audio_descriptor);

static inline UINT _ux_host_class_audio20_sampling_parse(UX_HOST_CLASS_AUDIO *audio,
        UINT(*parse_function)(VOID  *arg,
                              UCHAR *packed_interface_descriptor,
                              UX_HOST_CLASS_AUDIO_SAMPLING_CHARACTERISTICS *sam_attr),
        VOID* arg);
#endif


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_audio_raw_sampling_parse             PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function parses all possible RAW (PCM like) sampling           */
/*    characteristics for current clock source and clock selector         */
/*    settings.                                                           */
/*                                                                        */
/*    For sampling characteristics of different clock settings,           */
/*    use ux_host_class_audio_descriptors_parse to get their IDs and      */
/*    use _ux_host_class_audio_control_request to change settings.        */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    audio                                 Pointer to audio class        */
/*    parse_function                        Pointer to parse function,    */
/*                                          returns 0 to continue parse,  */
/*                                          others to terminate parse.    */
/*    arg                                   Pointer to parse arguments    */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_stack_class_instance_verify  Verify instance is valid      */
/*    _ux_host_semaphore_get                Get semaphore                 */
/*    _ux_host_semaphore_put                Put semaphore                 */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Application                                                         */
/*    Audio Class                                                         */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  07-29-2022     Chaoqiong Xiao           Initial Version 6.1.12        */
/*                                                                        */
/**************************************************************************/
UINT _ux_host_class_audio_raw_sampling_parse(UX_HOST_CLASS_AUDIO *audio,
        UINT(*parse_function)(VOID  *arg,
                              UCHAR *packed_interface_descriptor,
                              UX_HOST_CLASS_AUDIO_SAMPLING_CHARACTERISTICS *sam_attr),
        VOID* arg)
{

    /* Ensure the instance is valid.  */
    if (_ux_host_stack_class_instance_verify(_ux_system_host_class_audio_name, (VOID *) audio) != UX_SUCCESS)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, audio, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

#if defined(UX_HOST_CLASS_AUDIO_2_SUPPORT)
    if (_ux_host_class_audio_protocol_get(audio) == UX_HOST_CLASS_AUDIO_PROTOCOL_IP_VERSION_02_00)
        return(_ux_host_class_audio20_sampling_parse(audio, parse_function, arg));
#endif

    return(_ux_host_class_audio10_sampling_parse(audio, parse_function, arg));
}

static UINT _ux_host_class_audio10_sam_parse_func(VOID *arg,
                            UCHAR *packed_interface_descriptor,
                            UCHAR *packed_endpoint_descriptor,
                            UCHAR *packed_audio_descriptor)
{
struct UX_HOST_CLASS_AUDIO10_SAM_PARSER           *parser = (struct UX_HOST_CLASS_AUDIO10_SAM_PARSER *)arg;
UX_HOST_CLASS_AUDIO_SAMPLING_CHARACTERISTICS    sam_attr;
ULONG                                           n, offset;
UINT                                            status;

    UX_PARAMETER_NOT_USED(packed_endpoint_descriptor);

    /* Check bInterfaceNumber @ 2.  */
    if (packed_interface_descriptor[2] !=
        parser -> audio -> ux_host_class_audio_streaming_interface ->
                            ux_interface_descriptor.bInterfaceNumber)
        return(0);

    /* Check bDescriptorType @ 1.  */
    if (packed_audio_descriptor[1] != UX_HOST_CLASS_AUDIO_CS_INTERFACE)
        return(0);

    /* Check bDescriptorSubType @ 2.  */
    if (packed_audio_descriptor[2] != UX_HOST_CLASS_AUDIO_CS_FORMAT_TYPE)
        return(0);

    /* Check bFormatType @ 3.  */
    if (packed_audio_descriptor[3] != UX_HOST_CLASS_AUDIO_FORMAT_TYPE_I)
        return(0);

    /* Get bNrChannels @ 4.  */
    sam_attr.ux_host_class_audio_sampling_characteristics_channels = packed_audio_descriptor[4];

    /* Get bBitResolution @ 6.  */
    sam_attr.ux_host_class_audio_sampling_characteristics_resolution = packed_audio_descriptor[6];

    /* No clock ID (set to 0), no mul, div (set to 1).  */
    sam_attr.ux_host_class_audio_sampling_characteristics_descriptor = packed_audio_descriptor;
    sam_attr.ux_host_class_audio_sampling_characteristics_clock_mul = 1;
    sam_attr.ux_host_class_audio_sampling_characteristics_clock_div = 1;

    /* Check bSamFreqType @ 7.  */
    if (packed_audio_descriptor[7] == 0)
    {

        /* Continuous, get dLowSamFreq and dHighSamFreq.  */
        sam_attr.ux_host_class_audio_sampling_characteristics_frequency_low =
                                    ((ULONG)packed_audio_descriptor[8]) +
                                    ((ULONG)packed_audio_descriptor[9]  << 8) +
                                    ((ULONG)packed_audio_descriptor[10] << 16);
        sam_attr.ux_host_class_audio_sampling_characteristics_frequency_high =
                                    ((ULONG)packed_audio_descriptor[11]) +
                                    ((ULONG)packed_audio_descriptor[12]  << 8) +
                                    ((ULONG)packed_audio_descriptor[13] << 16);

        /* Parse this sampling characteristic.  */
        status = parser->parse_function(parser->arg, packed_interface_descriptor, &sam_attr);
        return(status);
    }
    else
    {

        /* Parse list of sampling characteristics.  */
        for (n = 0, offset = 8;
             n < packed_audio_descriptor[7];
             n ++, offset += 3)
        {
            sam_attr.ux_host_class_audio_sampling_characteristics_frequency_low =
                                    ((ULONG)packed_audio_descriptor[offset]) +
                                    ((ULONG)packed_audio_descriptor[offset+1]  << 8) +
                                    ((ULONG)packed_audio_descriptor[offset+2] << 16);

            /* Not range, frequency high = low.  */
            sam_attr.ux_host_class_audio_sampling_characteristics_frequency_high =
                sam_attr.ux_host_class_audio_sampling_characteristics_frequency_low;

            /* Parse this sampling characteristic.  */
            status = parser->parse_function(parser->arg, packed_interface_descriptor, &sam_attr);

            /* If status is not 0, it terminate parsing.  */
            if (status != 0)
                return(status);
        }
    }
    return(0);
}

static inline UINT _ux_host_class_audio10_sampling_parse(UX_HOST_CLASS_AUDIO *audio,
        UINT(*parse_function)(VOID  *arg,
                              UCHAR *packed_interface_descriptor,
                              UX_HOST_CLASS_AUDIO_SAMPLING_CHARACTERISTICS *sam_attr),
        VOID* arg)
{

struct UX_HOST_CLASS_AUDIO10_SAM_PARSER     parser;

    /* No need to protect thread reentry to this instance (no requests).  */

    parser.parse_function = parse_function;
    parser.arg = arg;
    parser.audio = audio;
    return(_ux_host_class_audio_descriptors_parse(audio,
                    _ux_host_class_audio10_sam_parse_func, (VOID *)&parser));
}

#if defined(UX_HOST_CLASS_AUDIO_2_SUPPORT)
struct UX_HOST_CLASS_AUDIO_AC_DESCR_FINDER_STRUCT
{
    UCHAR   *descriptor;
    UCHAR   interface_num;
    UCHAR   subtype;
    UCHAR   id;
};
static UINT  _ux_host_class_audio_ac_find_parse(VOID  *arg,
                              UCHAR *packed_interface_descriptor,
                              UCHAR *packed_endpoint_descriptor,
                              UCHAR *packed_audio_descriptor)
{
struct UX_HOST_CLASS_AUDIO_AC_DESCR_FINDER_STRUCT *finder = (struct UX_HOST_CLASS_AUDIO_AC_DESCR_FINDER_STRUCT *)arg;

    UX_PARAMETER_NOT_USED(packed_endpoint_descriptor);

    /* Check interface number (bInterfaceNumber @ 2).  */
    if (packed_interface_descriptor[2] != finder -> interface_num)
        return(0);

    /* Check bDescriptorType @ 1.  */
    if (packed_audio_descriptor[1] != UX_HOST_CLASS_AUDIO_CS_INTERFACE)
        return(0);

    /* Get finding data.  */
    finder = (struct UX_HOST_CLASS_AUDIO_AC_DESCR_FINDER_STRUCT *)arg;

    /* Check bDescriptorSubType @ 2.  */
    if (packed_audio_descriptor[2] != finder -> subtype)
        return(0);

    /* Check bEntityID @ 3.  */
    if (packed_audio_descriptor[3] != finder -> id)
        return(0);

    /* Found it, break parsing loop.  */
    finder -> descriptor = packed_audio_descriptor;
    return(1);
}
static UCHAR *_ux_host_class_audio_ac_find(UX_HOST_CLASS_AUDIO *audio, UCHAR subtype, UCHAR id)
{

struct UX_HOST_CLASS_AUDIO_AC_DESCR_FINDER_STRUCT   finder;
UINT                                                status;

    finder.descriptor    = UX_NULL;
    finder.interface_num = (UCHAR)audio -> ux_host_class_audio_control_interface_number;
    finder.subtype       = subtype;
    finder.id            = id;
    status = _ux_host_class_audio_descriptors_parse(audio,
                            _ux_host_class_audio_ac_find_parse,
                            &finder);
    if (status != UX_SUCCESS)
        return(UX_NULL);
    return(finder.descriptor);
}

static inline UINT _ux_host_class_audio20_clock_get(UX_HOST_CLASS_AUDIO *audio,
                    UCHAR terminal_link,
                    UCHAR **csd, ULONG *clk_numerator, ULONG *clk_denominator)
{

UX_DEVICE       *device;
UX_ENDPOINT     *endpoint;
UX_TRANSFER     *request;
UCHAR           *control_buffer = UX_NULL;
UCHAR           *descriptor;
UCHAR           id;
ULONG           numerator = 1, denominator = 1;
UCHAR           scan_round = 0;
UINT            status;

    if (audio -> ux_host_class_audio_type == UX_HOST_CLASS_AUDIO_INPUT)
    {
    
        /* If audio input, streaming is from output terminal (OT).  */
        descriptor = _ux_host_class_audio_ac_find(audio,
                                        UX_CLASS_AUDIO20_AC_OUTPUT_TERMINAL,
                                        (UCHAR)terminal_link);
        if (descriptor == UX_NULL)
            return(UX_DESCRIPTOR_CORRUPTED);

        /* Get OTD::bCSourceID @ 8.  */
        id = descriptor[8];
    }
    else
    {

        /* If audio output, streaming is to input terminal (IT).  */
        descriptor = _ux_host_class_audio_ac_find(audio,
                                        UX_CLASS_AUDIO20_AC_INPUT_TERMINAL,
                                        (UCHAR)terminal_link);
        if (descriptor == UX_NULL)
            return(UX_DESCRIPTOR_CORRUPTED);

        /* Get ITD::bCSourceID @ 7.  */
        id = descriptor[7];
    }

    /* Get control endpoint and request, for possible control requests.  */
    device = audio -> ux_host_class_audio_device;
    endpoint = &device -> ux_device_control_endpoint;
    request = &endpoint -> ux_endpoint_transfer_request;

    /* Get clock source descriptor.  */
    do
    {

        /* Try to find ClockSource CSD.  */
        descriptor = _ux_host_class_audio_ac_find(audio,
                                    UX_CLASS_AUDIO20_AC_CLOCK_SOURCE, id);
        if (descriptor != UX_NULL)
        {

            /* Finally we get clock source.  */
            *csd = descriptor;
            *clk_numerator = numerator;
            *clk_denominator = denominator;
            status = UX_SUCCESS;
            break;
        }

        /* Try to find ClockMultiplier CMD.  */
        descriptor = _ux_host_class_audio_ac_find(audio,
                                    UX_CLASS_AUDIO20_AC_CLOCK_MULTIPLIER, id);
        if (descriptor != UX_NULL)
        {

            /* Get numerator, denominator.  */

            /* Allocate buffer for requests.  */
            control_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, 4);
            if (control_buffer == UX_NULL)
            {

                /* Unprotect device control.  */
                status = UX_MEMORY_INSUFFICIENT;
                break;
            }

            /* Protect device control endpoint.  */
            status = _ux_host_semaphore_get(&device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);
            if (status != UX_SUCCESS)
            {
                status = UX_SEMAPHORE_ERROR;
                break;
            }

            /* Create a transfer request for the GET_CUR LAYOUT 2 request.  */
            request -> ux_transfer_request_data_pointer =      control_buffer;
            request -> ux_transfer_request_requested_length =  2;
            request -> ux_transfer_request_function =          UX_CLASS_AUDIO20_CUR;
            request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
            request -> ux_transfer_request_value =             0 | (UX_CLASS_AUDIO20_CM_NUMERATOR_CONTROL << 8);
            request -> ux_transfer_request_index =             audio -> ux_host_class_audio_control_interface_number | ((ULONG)id << 8);
            *(ULONG *)control_buffer = 0;
            status = _ux_host_stack_transfer_request(request);
            if ((status != UX_SUCCESS) ||
                (request -> ux_transfer_request_actual_length != 2) ||
                (*(ULONG *)control_buffer == 0))
            {
                status = UX_TRANSFER_ERROR;
                break;
            }

            /* Update numerator.  */
            numerator *= *(ULONG *)control_buffer;

            /* Protect device control endpoint.  */
            status = _ux_host_semaphore_get(&device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);
            if (status != UX_SUCCESS)
            {
                status = UX_SEMAPHORE_ERROR;
                break;
            }

            /* Create a transfer request for the GET_CUR request.  */
            request -> ux_transfer_request_data_pointer =      control_buffer;
            request -> ux_transfer_request_requested_length =  2;
            request -> ux_transfer_request_function =          UX_CLASS_AUDIO20_CUR;
            request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
            request -> ux_transfer_request_value =             0 | (UX_CLASS_AUDIO20_CM_DENOMINATOR_CONTROL << 8);
            request -> ux_transfer_request_index =             audio -> ux_host_class_audio_control_interface_number | ((ULONG)id << 8);
            *(ULONG *)control_buffer = 0;
            status = _ux_host_stack_transfer_request(request);
            if ((status != UX_SUCCESS) ||
                (request -> ux_transfer_request_actual_length != 2) ||
                (*(ULONG *)control_buffer == 0))
            {
                status = UX_TRANSFER_ERROR;
                break;
            }

            /* Update denominator.  */
            denominator *= *(ULONG *)control_buffer;

            /* Get CMD::bCSourceID @ 4.  */
            id = descriptor[4];

            /* Next round of scan.  */
            scan_round ++;
            continue;
        }

        /* Try to find ClockSelector CSD.  */
        descriptor = _ux_host_class_audio_ac_find(audio,
                                    UX_CLASS_AUDIO20_AC_CLOCK_SELECTOR, id);
        if (descriptor != UX_NULL)
        {

            /* Get current selected InPin.  */

            /* Allocate buffer for requests.  */
            if (control_buffer == UX_NULL)
            {
                control_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, 4);
                if (control_buffer == UX_NULL)
                {

                    /* Unprotect device control.  */
                    status = UX_MEMORY_INSUFFICIENT;
                    break;
                }
            }

            /* Protect device control endpoint.  */
            status = _ux_host_semaphore_get(&device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);
            if (status != UX_SUCCESS)
            {
                status = UX_SEMAPHORE_ERROR;
                break;
            }

            /* Create a transfer request for the GET_CUR LAYOUT 1 request.  */
            request -> ux_transfer_request_data_pointer =      control_buffer;
            request -> ux_transfer_request_requested_length =  1;
            request -> ux_transfer_request_function =          UX_CLASS_AUDIO20_CUR;
            request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
            request -> ux_transfer_request_value =             0 | (UX_CLASS_AUDIO20_CX_CLOCK_SELECTOR_CONTROL << 8);
            request -> ux_transfer_request_index =             audio -> ux_host_class_audio_control_interface_number | ((ULONG)id << 8);
            *(ULONG *)control_buffer = 0;
            status = _ux_host_stack_transfer_request(request);
            if ((status != UX_SUCCESS) ||
                (request -> ux_transfer_request_actual_length != 1) ||
                (*(ULONG *)control_buffer == 0))
            {
                status = UX_TRANSFER_ERROR;
                break;
            }

            /* Update id from CSD::baCSourceID @ 5.  */
            id = descriptor[5 + *(ULONG *)control_buffer - 1];

            /* Next round of scan.  */
            scan_round ++;
            continue;
        }

        /* Nothing found!!!  */
        status = UX_DESCRIPTOR_CORRUPTED;
    } while(status == UX_SUCCESS);

    /* Free allocated resources.  */
    if (control_buffer)
        _ux_utility_memory_free(control_buffer);

    /* Return with status.  */
    return(status);
}

static UINT _ux_host_class_audio20_sam_parse_func(VOID *arg,
                            UCHAR *packed_interface_descriptor,
                            UCHAR *packed_endpoint_descriptor,
                            UCHAR *packed_audio_descriptor)
{

struct UX_HOST_CLASS_AUDIO20_SAM_PARSER         *parser = (struct UX_HOST_CLASS_AUDIO20_SAM_PARSER *)arg;
UX_HOST_CLASS_AUDIO                             *audio = parser -> audio;

UCHAR                                           *csd;
ULONG                                           clk_id, clk_mul = 1, clk_div = 1;
UINT                                            status;
UX_DEVICE                                       *device;
UX_ENDPOINT                                     *endpoint;
UX_TRANSFER                                     *transfer;
UCHAR                                           *buffer;
ULONG                                           n_sub, param_len, offset;
UX_HOST_CLASS_AUDIO_SAMPLING_CHARACTERISTICS    sam_attr;

    UX_PARAMETER_NOT_USED(packed_endpoint_descriptor);

    /* Check bInterfaceNumber @ 2 to confirm inside AS.  */
    if (packed_interface_descriptor[2] !=
        audio -> ux_host_class_audio_streaming_interface ->
                    ux_interface_descriptor.bInterfaceNumber)
        return(0);

    /* Check bDescriptorType @ 1 to confirm AS CS descriptor.  */
    if (packed_audio_descriptor[1] != UX_CLASS_AUDIO20_CS_INTERFACE)
        return(0);

    /* Check bDescriptorSubType@2 to confirm AS_HEADER.  */
    if (packed_audio_descriptor[2] == UX_CLASS_AUDIO20_AS_GENERAL)
    {

        /* Save AS_HEADER for future use.  */
        parser -> as_header = packed_audio_descriptor;
        return(0);
    }

    /* Check bDescriptorSubType@2, bFormatType@3 to confirm FORMAT_TYPE_I.  */
    if (packed_audio_descriptor[2] != UX_CLASS_AUDIO20_AS_FORMAT_TYPE)
        return(0);
    if (packed_audio_descriptor[3] != UX_CLASS_AUDIO20_FORMAT_TYPE_I)
        return(0);

    /* Save FORMAT_TYPE_I::bBitResolution@5.  */
    sam_attr.ux_host_class_audio_sampling_characteristics_resolution = packed_audio_descriptor[5];

    /* If AS_HEADER is not ready, fail.  */
    if (parser -> as_header == UX_NULL)
    {
        parser -> status = UX_DESCRIPTOR_CORRUPTED;
        return(1);
    }

    /* Check AS_HEADER::bTerminalLink@3 to find frequencies.  */
    status = _ux_host_class_audio20_clock_get(audio, parser -> as_header[3],
                                                &csd, &clk_mul, &clk_div);
    if (status != UX_SUCCESS)
    {
        parser -> status = status;
        return(1);
    }

    /* Get CSD::bClockID@3.  */
    clk_id = csd[3];

    /* Issue GET_RANGE to get sampling frequency.  */

    /* Get device, endpoint, transfer.  */
    device = audio -> ux_host_class_audio_device;
    endpoint = &device -> ux_device_control_endpoint;
    transfer = &endpoint -> ux_endpoint_transfer_request;

    /* Allocate buffer for GET_RANGE.  */
    buffer = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_CACHE_SAFE_MEMORY, 2);
    if (buffer == UX_NULL)
    {
        parser -> status = UX_MEMORY_INSUFFICIENT;
        return(1);
    }

    /* Protect control transfer.  */
    parser -> status = _ux_host_semaphore_get(&device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);
    if (parser -> status != UX_SUCCESS)
    {
        _ux_utility_memory_free(buffer);
        return(1);
    }

    /* Issue GET_RANGE request.  */
    transfer -> ux_transfer_request_data_pointer =     buffer;
    transfer -> ux_transfer_request_requested_length = 2;
    transfer -> ux_transfer_request_function =         UX_CLASS_AUDIO20_RANGE;
    transfer -> ux_transfer_request_type =             UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
    transfer -> ux_transfer_request_value =            0 | (UX_CLASS_AUDIO20_CS_SAM_FREQ_CONTROL << 8);
    transfer -> ux_transfer_request_index =            audio -> ux_host_class_audio_control_interface_number | (clk_id << 8);
    parser -> status = _ux_host_stack_transfer_request(transfer);

    /* Get wNumSubRanges.  */
    n_sub = (ULONG)(*(USHORT *)buffer);
    _ux_utility_memory_free(buffer);

    /* Check errors.  */
    if (parser -> status != UX_SUCCESS)
        return(1);
    if (n_sub == 0)
    {
        parser -> status = UX_TRANSFER_ERROR;
        return(1);
    }

    /* Calculate parameter length, with overflow check.  */
    param_len = n_sub * (4 * 3);
    if (param_len > 0xFFFF)
    {
        parser -> status = UX_MATH_OVERFLOW;
        return(1);
    }
    param_len += 2;
    if (param_len > 0xFFFF)
    {
        parser -> status = UX_MATH_OVERFLOW;
        return(1);
    }
 
    /* Allocate buffer for GET_RANGE.  */
    buffer = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_CACHE_SAFE_MEMORY, param_len);
    if (buffer == UX_NULL)
    {
        parser -> status = UX_MEMORY_INSUFFICIENT;
        return(1);
    }

    /* Protect control transfer.  */
    parser -> status = _ux_host_semaphore_get(&device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);
    if (parser -> status != UX_SUCCESS)
    {
        _ux_utility_memory_free(buffer);
        return(1);
    }

    /* Issue GET_RANGE request.  */
    transfer -> ux_transfer_request_data_pointer =     buffer;
    transfer -> ux_transfer_request_requested_length = param_len;
    transfer -> ux_transfer_request_function =         UX_CLASS_AUDIO20_RANGE;
    transfer -> ux_transfer_request_type =             UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
    transfer -> ux_transfer_request_value =            0 | (UX_CLASS_AUDIO20_CS_SAM_FREQ_CONTROL << 8);
    transfer -> ux_transfer_request_index =            audio -> ux_host_class_audio_control_interface_number | (clk_id << 8);
    parser -> status = _ux_host_stack_transfer_request(transfer);
    if (parser -> status != UX_SUCCESS)
    {
        _ux_utility_memory_free(buffer);
        return(1);
    }

    /* Save AS_HEADER::bNrChannels@10.  */
    sam_attr.ux_host_class_audio_sampling_characteristics_channels = parser -> as_header[10];

    /* Save CSD.  */
    sam_attr.ux_host_class_audio_sampling_characteristics_descriptor = csd;

    /* Parse ranges.  */
    for (offset = 2; offset < param_len; offset += (4*3))
    {

        /* Save dMIN, dMAX.  */
        sam_attr.ux_host_class_audio_sampling_characteristics_frequency_low =
                                    _ux_utility_long_get(buffer + offset);
        sam_attr.ux_host_class_audio_sampling_characteristics_frequency_high =
                                    _ux_utility_long_get(buffer + offset + 4);

        /* Save MUL, DIV.  */
        sam_attr.ux_host_class_audio_sampling_characteristics_clock_mul = clk_mul;
        sam_attr.ux_host_class_audio_sampling_characteristics_clock_div = clk_div;

        /* Parse frequency.  */
        status = parser -> parse_function(parser -> arg, packed_interface_descriptor, &sam_attr);
        if (status)
        {
            _ux_utility_memory_free(buffer);
            return(status);
        }
    }
    _ux_utility_memory_free(buffer);

    /* Continue parsing.  */
    return(0);
}
static inline UINT _ux_host_class_audio20_sampling_parse(UX_HOST_CLASS_AUDIO *audio,
        UINT(*parse_function)(VOID  *arg,
                              UCHAR *packed_interface_descriptor,
                              UX_HOST_CLASS_AUDIO_SAMPLING_CHARACTERISTICS *sam_attr),
        VOID* arg)
{
struct UX_HOST_CLASS_AUDIO20_SAM_PARSER parser;
UINT                                    status;

    /* Run parse process.  */
    parser.parse_function = parse_function;
    parser.arg = arg;
    parser.audio = audio;
    parser.as_header = UX_NULL;
    parser.status = UX_SUCCESS;
    status = _ux_host_class_audio_descriptors_parse(audio,
                    _ux_host_class_audio20_sam_parse_func, (VOID *)&parser);

    /* Return parser status.  */
    if (status == UX_SUCCESS)
        return(parser.status);

    /* Return parsing error status.  */
    return(status);
}
#endif /* defined(UX_HOST_CLASS_AUDIO_2_SUPPORT) */
