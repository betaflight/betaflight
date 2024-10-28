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

/**************************************************************************/
/*                                                                        */
/*  COMPONENT DEFINITION                                   RELEASE        */
/*                                                                        */
/*    ux_device_class_ccid.h                              PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This file defines the equivalences for the USBX Device Class CCID   */
/*    (Smart-Card Integrated Circuit(s) Card) component.                  */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  04-25-2022     Chaoqiong Xiao           Initial Version 6.1.11        */
/*                                                                        */
/**************************************************************************/

#ifndef UX_DEVICE_CLASS_CCID_H
#define UX_DEVICE_CLASS_CCID_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard
   C is used to process the API information.  */

#ifdef   __cplusplus

/* Yes, C++ compiler is present.  Use standard C.  */
extern   "C" {

#endif


/* Define CCID max number of slots, 32 for 32 bit data width.  */
#define UX_DEVICE_CLASS_CCID_MAX_N_SLOTS                                    (sizeof(ALIGN_TYPE)*8)

#define UX_DEVICE_CLASS_CCID_THREAD_STACK_SIZE                              UX_THREAD_STACK_SIZE
#define UX_DEVICE_CLASS_CCID_NOTIFY_THREAD_STACK_SIZE                       UX_THREAD_STACK_SIZE
#define UX_DEVICE_CLASS_CCID_RUNNER_THREAD_STACK_SIZE                       UX_THREAD_STACK_SIZE

/* Define CCID Class USB Class constants.  */
#define UX_DEVICE_CLASS_CCID_CLASS                                          0x0B

#define UX_DEVICE_CLASS_CCID_SUBCLASS                                       0

#define UX_DEVICE_CLASS_CCID_PROTOCOL                                       0

#define UX_DEVICE_CLASS_CCID_BCD_CCID_1_10                                  0x0110

#define UX_DEVICE_CLASS_CCID_VOLTAGE_SUPPORT_5_0                            1u
#define UX_DEVICE_CLASS_CCID_VOLTAGE_SUPPORT_3_0                            2u
#define UX_DEVICE_CLASS_CCID_VOLTAGE_SUPPORT_1_8                            4u

#define UX_DEVICE_CLASS_CCID_PROTOCOLS_T0                                   1u
#define UX_DEVICE_CLASS_CCID_PROTOCOLS_T1                                   2u

#define UX_DEVICE_CLASS_CCID_SYNCH_PROTOCOLS_2_WIRE                         1u
#define UX_DEVICE_CLASS_CCID_SYNCH_PROTOCOLS_3_WIRE                         2u
#define UX_DEVICE_CLASS_CCID_SYNCH_PROTOCOLS_I2C                            4u

#define UX_DEVICE_CLASS_CCID_MECHANICAL_NO_SPECIAL                          0u
#define UX_DEVICE_CLASS_CCID_MECHANICAL_ACCEPT                              1u
#define UX_DEVICE_CLASS_CCID_MECHANICAL_EJECT                               2u
#define UX_DEVICE_CLASS_CCID_MECHANICAL_CAPTURE                             4u
#define UX_DEVICE_CLASS_CCID_MECHANICAL_LOCK_UNLOCK                         8u

#define UX_DEVICE_CLASS_CCID_FEATURES_NO_SPECIAL                            0u
#define UX_DEVICE_CLASS_CCID_FEATURES_AUTO_PARAMETER_ATR                    2u
#define UX_DEVICE_CLASS_CCID_FEATURES_AUTO_ACTIVATION                       4u
#define UX_DEVICE_CLASS_CCID_FEATURES_AUTO_VOLTAGE                          8u
#define UX_DEVICE_CLASS_CCID_FEATURES_AUTO_CLOCK_FREQUENCY                  0x10u
#define UX_DEVICE_CLASS_CCID_FEATURES_AUTO_BAUD_RATE                        0x20u
#define UX_DEVICE_CLASS_CCID_FEATURES_AUTO_PARAMETER_NEGOTIATION            0x40u
#define UX_DEVICE_CLASS_CCID_FEATURES_AUTO_PPS                              0x80u
#define UX_DEVICE_CLASS_CCID_FEATURES_CAN_SET_ICC_IN_CLOCK_STOP             0x100u
#define UX_DEVICE_CLASS_CCID_FEATURES_NAD_NOT_0_ACCEPTED                    0x200u
#define UX_DEVICE_CLASS_CCID_FEATURES_AUTO_IFSD_EXCHANGE_1ST                0x400u
#define UX_DEVICE_CLASS_CCID_FEATURES_TPUD_EXCHANGES                        0x10000u
#define UX_DEVICE_CLASS_CCID_FEATURES_SHORT_APUD_EXCHANGES                  0x20000u
#define UX_DEVICE_CLASS_CCID_FEATURES_SHORT_AND_EXTENDED_APUD_EXCHANGES     0x40000u
#define UX_DEVICE_CLASS_CCID_FEATURES_WAKEUP_ON_INSERTION_REMOVAL           0x100000u

#define UX_DEVICE_CLASS_CCID_LCD_LAYOUT_NO_LCD                              0
#define UX_DEVICE_CLASS_CCID_LCD_LAYOUT(n_lines,n_per_line)                 (((n_lines) << 8) | (n_per_line))

#define UX_DEVICE_CLASS_CCID_PIN_SUPPORT_VERIFICATION                       1u
#define UX_DEVICE_CLASS_CCID_PIN_SUPPORT_MODIFICATION                       2u

#define UX_DEVICE_CLASS_CCID_MAX_CCID_MESSAGE_EXT_APDU_MIN                  (261+10)
#define UX_DEVICE_CLASS_CCID_MAX_CCID_MESSAGE_EXT_APDU_MAX                  (65544+10)


/* Device CCID Requests */
#define UX_DEVICE_CLASS_CCID_ABORT                                          0x01
#define UX_DEVICE_CLASS_CCID_GET_CLOCK_FREQUENCIES                          0x02
#define UX_DEVICE_CLASS_CCID_GET_DATA_RATES                                 0x03


/* CCID message offsets, values.  */

/* General.  */
#define UX_DEVICE_CLASS_CCID_MESSAGE_HEADER_LENGTH                          10

#define UX_DEVICE_CLASS_CCID_OFFSET_MESSAGE_TYPE                            0
#define UX_DEVICE_CLASS_CCID_OFFSET_LENGTH                                  1
#define UX_DEVICE_CLASS_CCID_OFFSET_SLOT                                    5
#define UX_DEVICE_CLASS_CCID_OFFSET_SEQ                                     6

#define UX_DEVICE_CLASS_CCID_OFFSET_DATA                                    10

/* PC_to_RDR_IccPowerOn.  */
#define UX_DEVICE_CLASS_CCID_OFFSET_POWER_SELECT                            7
#define UX_DEVICE_CLASS_CCID_POWER_SELECT_AUTO                              0
#define UX_DEVICE_CLASS_CCID_POWER_SELECT_5_0_V                             1
#define UX_DEVICE_CLASS_CCID_POWER_SELECT_3_0_V                             2
#define UX_DEVICE_CLASS_CCID_POWER_SELECT_1_8_V                             3

/* PC_to_RDR_XfrBlock/PC_to_RDR_Secure.  */
#define UX_DEVICE_CLASS_CCID_OFFSET_BWI                                     7
#define UX_DEVICE_CLASS_CCID_OFFSET_LEVEL_PARAMETER                         8
#define UX_DEVICE_CLASS_CCID_LEVEL_PARAMETER_RFU                            0x00u
#define UX_DEVICE_CLASS_CCID_LEVEL_PARAMETER_BEGIN_END                      0x00u
#define UX_DEVICE_CLASS_CCID_LEVEL_PARAMETER_BEGIN_CONTINUE                 0x01u
#define UX_DEVICE_CLASS_CCID_LEVEL_PARAMETER_CONTINUE_END                   0x02u
#define UX_DEVICE_CLASS_CCID_LEVEL_PARAMETER_CONTINUE                       0x03u
#define UX_DEVICE_CLASS_CCID_LEVEL_PARAMETER_EMPTY_DATA                     0x10u

/* PC_to_RDR_SetParameters.  */
#define UX_DEVICE_CLASS_CCID_OFFSET_SET_PARAMETERS_PROTOCOL_NUM             7

#define UX_DEVICE_CLASS_CCID_PROTOCOL_T_0                                   0
#define UX_DEVICE_CLASS_CCID_PROTOCOL_T_1                                   1
#define UX_DEVICE_CLASS_CCID_PROTOCOL_2_WIRE                                0x80
#define UX_DEVICE_CLASS_CCID_PROTOCOL_3_WIRE                                0x81
#define UX_DEVICE_CLASS_CCID_PROTOCOL_I2C                                   0x82

/* Parameters.  */
#define UX_DEVICE_CLASS_CCID_OFFSET_FI_DI                                   10

#define UX_DEVICE_CLASS_CCID_OFFSET_TCCKS_T0                                11
#define UX_DEVICE_CLASS_CCID_OFFSET_GUARD_TIME_T0                           12
#define UX_DEVICE_CLASS_CCID_OFFSET_WAITING_INTEGER_T0                      13
#define UX_DEVICE_CLASS_CCID_OFFSET_CLOCK_STOP_T0                           14

#define UX_DEVICE_CLASS_CCID_OFFSET_TCCKS_T1                                11
#define UX_DEVICE_CLASS_CCID_OFFSET_GUARD_TIME_T1                           12
#define UX_DEVICE_CLASS_CCID_OFFSET_WAITING_INTEGERS_T1                     13
#define UX_DEVICE_CLASS_CCID_OFFSET_CLOCK_STOP_T1                           14
#define UX_DEVICE_CLASS_CCID_OFFSET_IFSC                                    15
#define UX_DEVICE_CLASS_CCID_OFFSET_NAD_VALUE                               16

/* PC_to_RDR_IccClock.  */
#define UX_DEVICE_CLASS_CCID_OFFSET_CLOCK_COMMAND                           7
#define UX_DEVICE_CLASS_CCID_CLOCK_COMMAND_RESTART                          0
#define UX_DEVICE_CLASS_CCID_CLOCK_COMMAND_STOP                             1

/* PC_to_RDR_T0APDU.  */
#define UX_DEVICE_CLASS_CCID_OFFSET_CHANGES                                 7
#define UX_DEVICE_CLASS_CCID_OFFSET_CLASS_GET_RESPONSE                      8
#define UX_DEVICE_CLASS_CCID_OFFSET_CLASS_ENVELOPE                          9
#define UX_DEVICE_CLASS_CCID_CHANGE_CLASS_GET_RESPONSE                      (1u << 0)
#define UX_DEVICE_CLASS_CCID_CHANGE_CLASS_ENVELOPE                          (1u << 1)

/* PC_to_RDR_Secure.  */
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_OPERATION                           10
#define UX_DEVICE_CLASS_CCID_PIN_VERIFICATION                               0
#define UX_DEVICE_CLASS_CCID_PIN_MODIFICATION                               1
#define UX_DEVICE_CLASS_CCID_PIN_TRANSFER                                   2
#define UX_DEVICE_CLASS_CCID_PIN_WAIT_ICC_RESP                              3
#define UX_DEVICE_CLASS_CCID_PIN_CANCEL                                     4
#define UX_DEVICE_CLASS_CCID_PIN_RESEND_I_BLOCK                             5
#define UX_DEVICE_CLASS_CCID_PIN_NEXT_APDU                                  6

#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_DATA                                11

#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_TIME_OUT                            11
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_FORMAT_STRING                       12

#define UX_DEVICE_CLASS_CCID_FORMAT_STRING_FORMAT_TYPE_MASK                 (0x3u << 0)
#define UX_DEVICE_CLASS_CCID_FORMAT_STRING_BINARY                           (0x0u << 0)
#define UX_DEVICE_CLASS_CCID_FORMAT_STRING_BCD                              (0x1u << 0)
#define UX_DEVICE_CLASS_CCID_FORMAT_STRING_ASCII                            (0x2u << 0)
#define UX_DEVICE_CLASS_CCID_FORMAT_STRING_JUSTIFICATION                    (0x1u << 2)
#define UX_DEVICE_CLASS_CCID_FORMAT_STRING_LEFT_JUSTIFY                     (0x0u << 2)
#define UX_DEVICE_CLASS_CCID_FORMAT_STRING_RIGHT_JUSTIFY                    (0x1u << 2)
#define UX_DEVICE_CLASS_CCID_FORMAT_STRING_POSITION_MASK                    (0xFu << 3)
#define UX_DEVICE_CLASS_CCID_FORMAT_STRING_POSITION(v)                      (((v) >> 3) & 0xFu)
#define UX_DEVICE_CLASS_CCID_FORMAT_STRING_UNIT_TYPE                        (0x1u << 7)
#define UX_DEVICE_CLASS_CCID_FORMAT_STRING_UNIT_BITS                        (0x0u << 7)
#define UX_DEVICE_CLASS_CCID_FORMAT_STRING_UNIT_BYTES                       (0x1u << 7)

#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_PIN_BLOCK_STRING                    13
#define UX_DEVICE_CLASS_CCID_PIN_BLOCK_STRING_PIN_LENGTH_BITS_MASK          (0xFu << 3)
#define UX_DEVICE_CLASS_CCID_PIN_BLOCK_STRING_PIN_BLOCK_SIZE_BYTES_MASK     (0xFu << 0)
#define UX_DEVICE_CLASS_CCID_PIN_BLOCK_STRING_PIN_LENGTH_BITS(v)            ((v) >> 4) & 0xFu)
#define UX_DEVICE_CLASS_CCID_PIN_BLOCK_STRING_PIN_BLOCK_SIZE_BYTES(v)       ((v) >> 0) & 0xFu)

#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_PIN_LENGTH_FORMAT                   14
#define UX_DEVICE_CLASS_CCID_PIN_LENGTH_FORMAT_UNIT_TYPE                    (1u << 4)
#define UX_DEVICE_CLASS_CCID_PIN_LENGTH_FORMAT_UNIT_BITS                    (0u << 4)
#define UX_DEVICE_CLASS_CCID_PIN_LENGTH_FORMAT_UNIT_BYTES                   (1u << 4)
#define UX_DEVICE_CLASS_CCID_PIN_LENGTH_FORMAT_POSITION_MASK                (0xFu << 0)
#define UX_DEVICE_CLASS_CCID_PIN_LENGTH_FORMAT_POSITION(v)                  (((v) << 0) & 0xFu)

#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_VERIFI_TIME_OUT                     11
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_VERIFI_FORMAT_STRING                12
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_VERIFI_PIN_BLOCK_STRING             13
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_VERIFI_PIN_LENGTH_FORMAT            14

#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_VERIFI_PIN_MAX_EXTRA_DIGIT          15
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_VERIFI_PIN_MAX_EXTRA_DIGIT_MIN      16
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_VERIFI_PIN_MAX_EXTRA_DIGIT_MAX      15

#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_VERIFI_ENTRY_VALIDATION_CONDITION   17
#define UX_DEVICE_CLASS_CCID_PIN_ENTRY_VALIDATION_MAX_SIZE                  1u
#define UX_DEVICE_CLASS_CCID_PIN_ENTRY_VALIDATION_KEY_PRESS                 2u
#define UX_DEVICE_CLASS_CCID_PIN_ENTRY_VALIDATION_TIMEOUT                   4u

#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_VERIFI_NUMBER_MESSAGE               18
#define UX_DEVICE_CLASS_CCID_PIN_NUMBER_MESSAGE_NO                          0
#define UX_DEVICE_CLASS_CCID_PIN_NUMBER_MESSAGE_1                           0
#define UX_DEVICE_CLASS_CCID_PIN_NUMBER_MESSAGE_2                           0
#define UX_DEVICE_CLASS_CCID_PIN_NUMBER_MESSAGE_3                           0
#define UX_DEVICE_CLASS_CCID_PIN_NUMBER_MESSAGE_DEFAULT                     0xFF

#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_VERIFI_LANG_ID                      19
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_VERIFI_MSG_INDEX                    21
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_VERIFI_TEO_PROLOGUE                 22
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_VERIFI_PIN_APDU                     25

#define UX_DEVICE_CLASS_CCID_PIN_MESSAGE_INSERT_PROMPT                      0
#define UX_DEVICE_CLASS_CCID_PIN_MESSAGE_MODIFY_PROMPT                      1
#define UX_DEVICE_CLASS_CCID_PIN_MESSAGE_NEW_CONFIRM_PROMPT                 2

#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_MODIFI_TIME_OUT                     11
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_MODIFI_FORMAT_STRING                12
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_MODIFI_PIN_BLOCK_STRING             13
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_MODIFI_PIN_LENGTH_FORMAT            14

#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_MODIFI_INSERTION_OFFSET_OLD         15
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_MODIFI_INSERTION_OFFSET_NEW         16

#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_MODIFI_PIN_MAX_EXTRA_DIGIT          17
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_MODIFI_PIN_MAX_EXTRA_DIGIT_MIN      18
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_MODIFI_PIN_MAX_EXTRA_DIGIT_MAX      17

#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_MODIFI_CONFIRM_PIN                  19
#define UX_DEVICE_CLASS_CCID_CONFIRM_PIN_CONFIRM_REQUESTED                  (1u << 0)
#define UX_DEVICE_CLASS_CCID_CONFIRM_PIN_ENTRY_REQUESTED                    (1u << 1)

#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_MODIFI_ENTRY_VALIDATION_CONDITION   20
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_MODIFI_NUMBER_MESSAGE               21
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_MODIFI_LANG_ID                      22
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_MODIFI_MSG_INDEX1                   24
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_MODIFI_MSG_INDEX2                   25
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_MODIFI_MSG_INDEX3                   26
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_MODIFI_TEO_PROLOGUE1                25
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_MODIFI_TEO_PROLOGUE2                26
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_MODIFI_TEO_PROLOGUE3                27
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_MODIFI_PIN_APDU1                    28
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_MODIFI_PIN_APDU2                    29
#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_MODIFI_PIN_APDU3                    30

#define UX_DEVICE_CLASS_CCID_OFFSET_PIN_NEXT_APDU_TEO_PROLOGUE              11

/* PC_to_RDR_Mechanical.  */
#define UX_DEVICE_CLASS_CCID_OFFSET_FUNCTION                                7
#define UX_DEVICE_CLASS_CCID_FUNCTION_ACCEPT_CARD                           0x01
#define UX_DEVICE_CLASS_CCID_FUNCTION_EJECT_CARD                            0x02
#define UX_DEVICE_CLASS_CCID_FUNCTION_CAPTURE_CARD                          0x03
#define UX_DEVICE_CLASS_CCID_FUNCTION_LOCK_CARD                             0x04
#define UX_DEVICE_CLASS_CCID_FUNCTION_UNLOCK_CARD                           0x05

/* PC_to_RDR_SetDataRateAndClockFrequency/RDR_to_PC_DataRateAndClockFrequency.  */
#define UX_DEVICE_CLASS_CCID_OFFSET_CLOCK_FREQUENCY                         10
#define UX_DEVICE_CLASS_CCID_OFFSET_DATA_RATE                               14

/* RDR_to_PC_   */
#define UX_DEVICE_CLASS_CCID_OFFSET_STATUS                                  7
#define UX_DEVICE_CLASS_CCID_OFFSET_ERROR                                   8

/* RDR_to_PC_DataBlock.  */
#define UX_DEVICE_CLASS_CCID_OFFSET_CHAIN_PARAMETER                         9
#define UX_DEVICE_CLASS_CCID_CHAIN_PARAMETER_BEGIN_END                      0x00
#define UX_DEVICE_CLASS_CCID_CHAIN_PARAMETER_BEGIN_CONTINUE                 0x01
#define UX_DEVICE_CLASS_CCID_CHAIN_PARAMETER_CONTINUE_END                   0x02
#define UX_DEVICE_CLASS_CCID_CHAIN_PARAMETER_CONTINUE                       0x03
#define UX_DEVICE_CLASS_CCID_CHAIN_PARAMETER_EMPTY                          0x10

/* RDR_to_PC_SlotStatus.  */
#define UX_DEVICE_CLASS_CCID_OFFSET_CLOCK_STATUS                            9

/* RDR_to_PC_Parameters.  */
#define UX_DEVICE_CLASS_CCID_OFFSET_PARAMETERS_PROTOCOL_NUM                 9

#define UX_DEVICE_CLASS_CCID_PARAMETERS_T0_LENGTH                           5
#define UX_DEVICE_CLASS_CCID_PARAMETERS_T1_LENGTH                           7

/* RDR_to_PC_NotifySlotChange.  */
#define UX_DEVICE_CLASS_CCID_OFFSET_SLOT_ICC_STATE                          1

/* RDR_to_PC_HardwareError.  */
#define UX_DEVICE_CLASS_CCID_OFFSET_HW_ERROR_SLOT                           1
#define UX_DEVICE_CLASS_CCID_OFFSET_HW_ERROR_SEQ                            2
#define UX_DEVICE_CLASS_CCID_OFFSET_HW_ERROR_CODE                           3
#define UX_DEVICE_CLASS_CCID_HW_ERROR_OVERCURRENT                           1


/* CCID bMessageTypes.  */
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_ICC_POWER_ON                         0x62
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_ICC_POWER_OFF                        0x63
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_GET_SLOT_STATUS                      0x64
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_XFR_BLOCK                            0x6F
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_GET_PARAMETERS                       0x6C
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_RESET_PARAMETERS                     0x6D
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_SET_PARAMETERS                       0x61
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_ESCAPE                               0x6B
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_ICC_CLOCK                            0x6E
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_T0_APDU                              0x6A
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_SECURE                               0x69
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_MECHANICAL                           0x71
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_ABORT                                0x72
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_SET_DATA_RATE_AND_CLOCK_FREQ         0x73

#define UX_DEVICE_CLASS_CCID_N_COMMANDS                                     14

#define UX_DEVICE_CLASS_CCID_RDR_TO_PC_DATA_BLOCK                           0x80
#define UX_DEVICE_CLASS_CCID_RDR_TO_PC_SLOT_STATUS                          0x81
#define UX_DEVICE_CLASS_CCID_RDR_TO_PC_PARAMETERS                           0x82
#define UX_DEVICE_CLASS_CCID_RDR_TO_PC_ESCAPE                               0x83
#define UX_DEVICE_CLASS_CCID_RDR_TO_PC_DATA_RATE_AND_CLOCK_FREQ             0x84

#define UX_DEVICE_CLASS_CCID_RDR_TO_PC_NOTIFY_SLOT_CHANGE                   0x50
#define UX_DEVICE_CLASS_CCID_RDR_TO_PC_HARDWARE_ERROR                       0x51

#define UX_DEVICE_CLASS_CCID_COMMAND_ABORTABLE(bMessageType)                \
    (((bMessageType) == 0x62) || ((bMessageType) == 0x6F) ||                \
     ((bMessageType) == 0x6B) || ((bMessageType) == 0x69) ||                \
     ((bMessageType) == 0x71) || ((bMessageType) == 0x72))

#define UX_DEVICE_CLASS_CCID_COMMAND_HW_ERROR_CHECK(bMessageType)           \
    (((bMessageType) != 0x63) && ((bMessageType) != 0x6B) &&                \
     ((bMessageType) != 0x6A) && ((bMessageType) != 0x72) &&                \
     ((bMessageType) != 0x73))

#define UX_DEVICE_CLASS_CCID_COMMAND_AUTO_SEQUENCE_CHECK(bMessageType)      \
    (((bMessageType) != 0x62) && ((bMessageType) != 0x65) &&                \
     ((bMessageType) != 0x6B) && ((bMessageType) != 0x6A) &&                \
     ((bMessageType) != 0x72) && ((bMessageType) != 0x73))

#define UX_DEVICE_CLASS_CCID_COMMAND_RESP_DATA_BLOCK(m)                     \
    ((m) == 0x62 || (m) == 0x6F || (m) == 0x69)
#define UX_DEVICE_CLASS_CCID_COMMAND_RESP_SLOT_STATUS(m)                    \
    ((m) == 0x63 || (m) == 0x64 || (m) == 0x6E || (m) == 0x6A ||            \
     (m) == 0x71 || (m) == 0x72 || (m) == 0x65)
#define UX_DEVICE_CLASS_CCID_COMMAND_RESP_PARAMETERS(m)                     \
    ((m) == 0x6C || (m) == 0x6D || (m) == 0x61)
#define UX_DEVICE_CLASS_CCID_COMMAND_RESP_ESCAPE(m)                         \
    ((m) == 0x6B)
#define UX_DEVICE_CLASS_CCID_COMMAND_RESP_DATA_RATE_AND_CLOCK_FREQ(m)       \
    ((m) == 0x73)

#define UX_DEVICE_CLASS_CCID_COMMAND_RESP_TYPE(m)                           \
    (UX_DEVICE_CLASS_CCID_COMMAND_RESP_DATA_BLOCK(m) ? 0x80 :               \
     UX_DEVICE_CLASS_CCID_COMMAND_RESP_SLOT_STATUS(m) ? 0x81 :              \
     UX_DEVICE_CLASS_CCID_COMMAND_RESP_PARAMETERS(m) ? 0x82 :               \
     UX_DEVICE_CLASS_CCID_COMMAND_RESP_ESCAPE(m) ? 0x83 :                   \
     UX_DEVICE_CLASS_CCID_COMMAND_RESP_DATA_RATE_AND_CLOCK_FREQ(m) ? 0x84 : \
     0)

/* CCID Slot error binary code.  */
#define UX_DEVICE_CLASS_CCID_CMD_ABORTED                                    0xFF
#define UX_DEVICE_CLASS_CCID_ICC_MUTE                                       0xFE
#define UX_DEVICE_CLASS_CCID_XFR_PARITY_ERROR                               0xFD
#define UX_DEVICE_CLASS_CCID_XFR_OVERRUN                                    0xFC
#define UX_DEVICE_CLASS_CCID_HW_ERROR                                       0xFB
#define UX_DEVICE_CLASS_CCID_BAD_ATR_TS                                     0xF8
#define UX_DEVICE_CLASS_CCID_BAD_ATR_TCK                                    0xF7
#define UX_DEVICE_CLASS_CCID_ICC_PROTOCOL_NOT_SUPPORTED                     0xF6
#define UX_DEVICE_CLASS_CCID_ICC_CLASS_NOT_SUPPORTED                        0xF5
#define UX_DEVICE_CLASS_CCID_PROCEDURE_BYTE_CONFLICT                        0xF4
#define UX_DEVICE_CLASS_CCID_DEACTIVATED_PROTOCOL                           0xF3
#define UX_DEVICE_CLASS_CCID_BUSY_WITH_AUTO_SEQUENCE                        0xF2
#define UX_DEVICE_CLASS_CCID_PIN_TIMEOUT                                    0xF0
#define UX_DEVICE_CLASS_CCID_PIN_CANCELLED                                  0xEF
#define UX_DEVICE_CLASS_CCID_CMD_SLOT_BUSY                                  0xE0


/* CCID bStatus.  */
#define UX_DEVICE_CLASS_CCID_SLOT_STATUS_TIME_EXTENSION                     0x80

#define UX_DEVICE_CLASS_CCID_SLOT_STATUS_ICC_ACTIVE                         (0x0u)
#define UX_DEVICE_CLASS_CCID_SLOT_STATUS_ICC_INACTIVE                       (0x1u)
#define UX_DEVICE_CLASS_CCID_SLOT_STATUS_ICC_NOT_PRESENT                    (0x2u)
#define UX_DEVICE_CLASS_CCID_SLOT_STATUS_ICC_MASK                           (0x3u)

#define UX_DEVICE_CLASS_CCID_SLOT_STATUS_CMD_NO_ERROR                       (0x0u << 6)
#define UX_DEVICE_CLASS_CCID_SLOT_STATUS_CMD_FAILED                         (0x1u << 6)
#define UX_DEVICE_CLASS_CCID_SLOT_STATUS_CMD_TIME_EXTENSION                 (0x2u << 6)
#define UX_DEVICE_CLASS_CCID_SLOT_STATUS_CMD_MASK                           (0x3u << 6)

#define UX_DEVICE_CLASS_CCID_SLOT_STATUS(bmICCStatus, bmCommandStatus)      ((bmICCStatus) | ((bmCommandStatus) << 6))
#define UX_DEVICE_CLASS_CCID_ICC_ACTIVE                                     0
#define UX_DEVICE_CLASS_CCID_ICC_INACTIVE                                   1
#define UX_DEVICE_CLASS_CCID_ICC_NOT_PRESENT                                2
#define UX_DEVICE_CLASS_CCID_CMD_NO_ERROR                                   0
#define UX_DEVICE_CLASS_CCID_CMD_FAILED                                     1
#define UX_DEVICE_CLASS_CCID_CMD_TIME_EXTENSION                             2


/* CCID events.  */
#define UX_DEVICE_CLASS_CCID_EVENT_SLOT(n)                                  (1u<<(n))


/* CCID flags.  */
#define UX_DEVICE_CLASS_CCID_FLAG_ABORTABLE                                 0x80u
#define UX_DEVICE_CLASS_CCID_FLAG_BUSY                                      0x40u

#define UX_DEVICE_CLASS_CCID_FLAG_NOTIFY_CHANGE                             0x01u
#define UX_DEVICE_CLASS_CCID_FLAG_NOTIFY_HW_ERROR                           0x02u

#define UX_DEVICE_CLASS_CCID_FLAG_AUTO_SEQUENCING                           0x04u
#define UX_DEVICE_CLASS_CCID_FLAG_HW_ERROR                                  0x08u


#define UX_DEVICE_CLASS_CCID_COMMAND_FLAG_ABORTABLE(bMessageType)           \
    (UX_DEVICE_CLASS_CCID_COMMAND_ABORTABLE(bMessageType) ?                 \
     UX_DEVICE_CLASS_CCID_FLAG_ABORTABLE : 0)

#define UX_DEVICE_CLASS_CCID_COMMAND_FLAG_AUTO_SEQUENCE_CHECK(bMessageType) \
    (UX_DEVICE_CLASS_CCID_COMMAND_AUTO_SEQUENCE_CHECK(bMessageType) ?       \
     UX_DEVICE_CLASS_CCID_FLAG_AUTO_SEQUENCING : 0)

#define UX_DEVICE_CLASS_CCID_COMMAND_FLAG_HW_ERROR_CHECK(bMessageType)      \
    (UX_DEVICE_CLASS_CCID_COMMAND_HW_ERROR_CHECK(bMessageType) ?            \
     UX_DEVICE_CLASS_CCID_FLAG_HW_ERROR : 0)

#define UX_DEVICE_CLASS_CCID_COMMAND_FLAGS(bMessageType)                    \
    (UX_DEVICE_CLASS_CCID_COMMAND_FLAG_ABORTABLE(bMessageType) |            \
     UX_DEVICE_CLASS_CCID_COMMAND_FLAG_AUTO_SEQUENCE_CHECK(bMessageType) |  \
     UX_DEVICE_CLASS_CCID_COMMAND_FLAG_HW_ERROR_CHECK(bMessageType))


/* CCID message structures.  */

/* General message for both PC_to_RDR and RDR_to_PC.  */
typedef struct UX_DEVICE_CLASS_CCID_MESSAGES_STRUCT
{
    UCHAR       *ux_device_class_ccid_messages_pc_to_rdr;
    UCHAR       *ux_device_class_ccid_messages_rdr_to_pc;
    ULONG       ux_device_class_ccid_messages_rdr_to_pc_length; /* Total buffer length including header.  */
} UX_DEVICE_CLASS_CCID_MESSAGES;

/* Define CCID message header.  */
typedef struct UX_DEVICE_CLASS_CCID_MESSAGE_HEADER_STRUCT
{
    UCHAR       bMessageType;
    UCHAR       dwLength[4];
    UCHAR       bSlot;
    UCHAR       bSeq;
    UCHAR       bRFU[3];
} UX_DEVICE_CLASS_CCID_MESSAGE_HEADER;
#define UX_DEVICE_CLASS_CCID_MESSAGE_TYPE(msg)                      (((UX_DEVICE_CLASS_CCID_MESSAGE_HEADER*)(msg))->bMessageType)
#define UX_DEVICE_CLASS_CCID_MESSAGE_LENGTH_GET(msg)                _ux_utility_long_get(((UX_DEVICE_CLASS_CCID_MESSAGE_HEADER*)(msg))->dwLength)
#define UX_DEVICE_CLASS_CCID_MESSAGE_LENGTH_SET(msg,len)            _ux_utility_long_put(((UX_DEVICE_CLASS_CCID_MESSAGE_HEADER*)(msg))->dwLength, (len))
#define UX_DEVICE_CLASS_CCID_MESSAGE_SLOT(msg)                      (((UX_DEVICE_CLASS_CCID_MESSAGE_HEADER*)(msg))->bSlot)
#define UX_DEVICE_CLASS_CCID_MESSAGE_SEQ(msg)                       (((UX_DEVICE_CLASS_CCID_MESSAGE_HEADER*)(msg))->bSeq)
#define UX_DEVICE_CLASS_CCID_MESSAGE_DATA(msg)                      (((UCHAR *)(msg) + UX_DEVICE_CLASS_CCID_MESSAGE_HEADER_LENGTH))

/* Define CCID PC_to_RDR_IccPowerOn.  */
typedef struct UX_DEVICE_CLASS_CCID_PC_TO_RDR_ICC_POWER_ON_HEADER_STRUCT
{
    UCHAR       bMessageType;
    UCHAR       dwLength[4];
    UCHAR       bSlot;
    UCHAR       bSeq;
    UCHAR       bPowerSelect;
    UCHAR       bRFU[2];
} UX_DEVICE_CLASS_CCID_PC_TO_RDR_ICC_POWER_ON_HEADER;
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_POWER_SELECT(msg)            (((UX_DEVICE_CLASS_CCID_PC_TO_RDR_ICC_POWER_ON_HEADER*)(msg))->bPowerSelect)

/* Define CCID PC_to_RDR_IccPowerOff.  */
typedef UX_DEVICE_CLASS_CCID_MESSAGE_HEADER UX_DEVICE_CLASS_CCID_PC_TO_RDR_ICC_POWER_OFF_HEADER;

/* Define CCID PC_to_RDR_GetSlotStatus.  */
typedef UX_DEVICE_CLASS_CCID_MESSAGE_HEADER UX_DEVICE_CLASS_CCID_PC_TO_RDR_GET_SLOT_STATUS_HEADER;

/* Define CCID PC_to_RDR_XfrBlock.  */
typedef struct UX_DEVICE_CLASS_CCID_PC_TO_RDR_XFR_BLOCK_HEADER_STRUCT
{
    UCHAR       bMessageType;
    UCHAR       dwLength[4];
    UCHAR       bSlot;
    UCHAR       bSeq;
    UCHAR       bBWI;
    USHORT      wLevelParameter;
} UX_DEVICE_CLASS_CCID_PC_TO_RDR_XFR_BLOCK_HEADER;
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_BWI(msg)                     (((UX_DEVICE_CLASS_CCID_PC_TO_RDR_XFR_BLOCK_HEADER*)(msg))->bBWI)
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_LEVEL_PARAMETER(msg)         (((UX_DEVICE_CLASS_CCID_PC_TO_RDR_XFR_BLOCK_HEADER*)(msg))->wLevelParameter)

/* Define CCID PC_to_RDR_GetParameters.  */
typedef UX_DEVICE_CLASS_CCID_MESSAGE_HEADER UX_DEVICE_CLASS_CCID_PC_TO_RDR_GET_PARAMETERS_HEADER;

/* Define CCID PC_to_RDR_ResetParameters.  */
typedef UX_DEVICE_CLASS_CCID_MESSAGE_HEADER UX_DEVICE_CLASS_CCID_PC_TO_RDR_RESET_PARAMETERS_HEADER;

/* Define CCID PC_to_RDR_SetParameters.  */
typedef struct UX_DEVICE_CLASS_CCID_PC_TO_RDR_SET_PARAMETERS_HEADER_STRUCT
{
    UCHAR       bMessageType;
    UCHAR       dwLength[4];
    UCHAR       bSlot;
    UCHAR       bSeq;
    UCHAR       bProtocolNum;
    UCHAR       bRFU[2];
} UX_DEVICE_CLASS_CCID_PC_TO_RDR_SET_PARAMETERS_HEADER;
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PROTOCOL_NUM(msg)            (((UX_DEVICE_CLASS_CCID_PC_TO_RDR_SET_PARAMETERS_HEADER*)(msg))->bProtocolNum)
typedef struct UX_DEVICE_CLASS_CCID_PC_TO_RDR_SET_PARAMETERS_T0_STRUCT
{
    UCHAR       bMessageType;
    UCHAR       dwLength[4];
    UCHAR       bSlot;
    UCHAR       bSeq;
    UCHAR       bProtocolNum;
    UCHAR       bRFU[2];
    UCHAR       bmFindexDindex;
    UCHAR       bmTCCKST0;
    UCHAR       bGuardTimeT0;
    UCHAR       bWaitingIntegerT0;
    UCHAR       bClockStop;
} UX_DEVICE_CLASS_CCID_PC_TO_RDR_SET_PARAMETERS_T0;
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_SET_PARAMETERS_T0_LENGTH     5
typedef struct UX_DEVICE_CLASS_CCID_PC_TO_RDR_SET_PARAMETERS_T1_STRUCT
{
    UCHAR       bMessageType;
    UCHAR       dwLength[4];
    UCHAR       bSlot;
    UCHAR       bSeq;
    UCHAR       bProtocolNum;
    UCHAR       bRFU[2];
    UCHAR       bmFindexDindex;
    UCHAR       bmTCCKST1;
    UCHAR       bGuardTimeT1;
    UCHAR       bmWaitingIntegersT1;
    UCHAR       bClockStop;
    UCHAR       bIFSC;
    UCHAR       bNadValue;
} UX_DEVICE_CLASS_CCID_PC_TO_RDR_SET_PARAMETERS_T1;
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_SET_PARAMETERS_T1_LENGTH     7

/* Define CCID PC_to_RDR_Escape.  */
typedef UX_DEVICE_CLASS_CCID_MESSAGE_HEADER UX_DEVICE_CLASS_CCID_PC_TO_RDR_ESCAPE_HEADER;

/* Define CCID PC_to_RDR_IccClock.  */
typedef struct UX_DEVICE_CLASS_CCID_PC_TO_RDR_ICC_CLOCK_HEADER_STRUCT
{
    UCHAR       bMessageType;
    UCHAR       dwLength[4];
    UCHAR       bSlot;
    UCHAR       bSeq;
    UCHAR       bClockCommand;
    UCHAR       bRFU[2];
} UX_DEVICE_CLASS_CCID_PC_TO_RDR_ICC_CLOCK_HEADER;
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_CLOCK_COMMAND(msg)           (((UX_DEVICE_CLASS_CCID_PC_TO_RDR_ICC_CLOCK_HEADER*)(msg))->bClockCommand)

/* Define CCID PC_to_RDR_T0APDU.  */
typedef struct UX_DEVICE_CLASS_CCID_PC_TO_RDR_T0_APDU_HEADER_STRUCT
{
    UCHAR       bMessageType;
    UCHAR       dwLength[4];
    UCHAR       bSlot;
    UCHAR       bSeq;
    UCHAR       bmChanges;
    UCHAR       bClassGetResponse;
    UCHAR       bClassEnvelope;
} UX_DEVICE_CLASS_CCID_PC_TO_RDR_T0_APDU_HEADER;
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_T0_APDU_CHANGES(msg)         (((UX_DEVICE_CLASS_CCID_PC_TO_RDR_T0_APDU_HEADER*)(msg))->bmChanges)
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_T0_APDU_CLASS_GET_RESP(msg)  (((UX_DEVICE_CLASS_CCID_PC_TO_RDR_T0_APDU_HEADER*)(msg))->bClassGetResponse)
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_T0_APDU_CLASS_ENVELOPE(msg)  (((UX_DEVICE_CLASS_CCID_PC_TO_RDR_T0_APDU_HEADER*)(msg))->bClassEnvelope)

/* Define CCID PC_to_RDR_Secure.  */
typedef UX_DEVICE_CLASS_CCID_PC_TO_RDR_XFR_BLOCK_HEADER UX_DEVICE_CLASS_CCID_PC_TO_RDR_SECURE_HEADER;

typedef struct UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_HEADER_STRUCT
{
    UCHAR       bMessageType;
    UCHAR       dwLength[4];
    UCHAR       bSlot;
    UCHAR       bSeq;
    UCHAR       bBWI;
    USHORT      wLevelParameter;
    UCHAR       bPINOperation;
} UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_HEADER;
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_BWI(msg)                 (((UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_HEADER*)(msg))->bBWI)
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_LEVEL_PARAMETER(msg)     (((UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_HEADER*)(msg))->wLevelParameter)
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_OPERATION(msg)           (((UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_HEADER*)(msg))->bPINOperation)

typedef struct UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_VERIFICATION_STRUCT
{
    UCHAR       bMessageType;
    UCHAR       dwLength[4];
    UCHAR       bSlot;
    UCHAR       bSeq;
    UCHAR       bBWI;
    USHORT      wLevelParameter;
    UCHAR       bPINOperation;

    UCHAR       bTimeOut;
    UCHAR       bmFormatString;
    UCHAR       bmPINBlockString;
    UCHAR       bmPINLengthFormat;

    UCHAR       wPINMaxExtraDigit[2]; /* XXYY, XX: min size, YY: max size.  */
    UCHAR       bEntryValidationCondition;
    UCHAR       bNumberMessage;
    UCHAR       wLangId[2];
    UCHAR       bMsgIndex;
    UCHAR       bTeoPrologue[3];
} UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_VERIFICATION;
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_TIMEOUT(msg)             (((UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_VERIFICATION*)(msg))->bTimeOut)
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_FORMAT_STRING(msg)       (((UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_VERIFICATION*)(msg))->bmFormatString)
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_BLOCK_STRING(msg)        (((UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_VERIFICATION*)(msg))->bmPINBlockString)
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_LENGTH_FORMAT(msg)       (((UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_VERIFICATION*)(msg))->bmPINLengthFormat)

#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_VERIFICATION_PIN_MAX_EXTRA_DIGIT(msg)            _ux_utility_short_get(((UX_DEVICE_CLASS_CCID_PC_TO_RDR_SECURE_PIN_VERIFICATION*)(msg))->wPINMaxExtraDigit)
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_VERIFICATION_PIN_MIN_PIN_SIZE(msg)               (((UX_DEVICE_CLASS_CCID_PC_TO_RDR_SECURE_PIN_VERIFICATION*)(msg))->wPINMaxExtraDigit[1])
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_VERIFICATION_PIN_MAX_PIN_SIZE(msg)               (((UX_DEVICE_CLASS_CCID_PC_TO_RDR_SECURE_PIN_VERIFICATION*)(msg))->wPINMaxExtraDigit[0])
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_VERIFICATION_ENTRY_VALIDATION_CONDITION(msg)     (((UX_DEVICE_CLASS_CCID_PC_TO_RDR_SECURE_PIN_VERIFICATION*)(msg))->bEntryValidationCondition)
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_VERIFICATION_NUMBER_MESSAGE(msg)                 (((UX_DEVICE_CLASS_CCID_PC_TO_RDR_SECURE_PIN_VERIFICATION*)(msg))->bNumberMessage)
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_VERIFICATION_LANG_ID(msg)                        _ux_utility_short_get(((UX_DEVICE_CLASS_CCID_PC_TO_RDR_SECURE_PIN_VERIFICATION*)(msg))->wLangId)
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_VERIFICATION_MESSAGE_INDEX(msg)                  (((UX_DEVICE_CLASS_CCID_PC_TO_RDR_SECURE_PIN_VERIFICATION*)(msg))->bMsgIndex)
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_VERIFICATION_TEO_PROLOGUE(msg)                   (((UX_DEVICE_CLASS_CCID_PC_TO_RDR_SECURE_PIN_VERIFICATION*)(msg))->bTeoPrologue)
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_VERIFICATION_DATA(msg)                           ((UCHAR*)(msg) + 25)
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_VERIFICATION_APDU(msg)                           ((UCHAR*)(msg) + 25)

typedef struct UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_MODIFICATION_STRUCT
{
    UCHAR       bMessageType;
    UCHAR       dwLength[4];
    UCHAR       bSlot;
    UCHAR       bSeq;
    UCHAR       bBWI;
    USHORT      wLevelParameter;
    UCHAR       bPINOperation;

    UCHAR       bTimeOut;
    UCHAR       bmFormatString;
    UCHAR       bmPINBlockString;
    UCHAR       bmPINLengthFormat;

    UCHAR       bInsertionOffsetOld;
    UCHAR       bInsertionOffsetNew;
    UCHAR       wPINMaxExtraDigit[2]; /* XXYY, XX: min size, YY: max size.  */
    UCHAR       bConfirmPIN;
    UCHAR       bEntryValidationCondition;
    UCHAR       bNumberMessage;
    UCHAR       wLangId[2];
    UCHAR       bMsgIndex1;
    UCHAR       bTeoPrologue[3];
} UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_MODIFICATION;
typedef UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_MODIFICATION UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_MODIFICATION1;
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_MODIFICATION1_TEO_PROLOGUE(msg)   ((UCHAR*)(msg) + 25)
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_MODIFICATION1_DATA(msg)           ((UCHAR*)(msg) + 28)
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_MODIFICATION1_APDU(msg)           ((UCHAR*)(msg) + 28)
typedef struct UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_MODIFICATION2_STRUCT
{
    UCHAR       bMessageType;
    UCHAR       dwLength[4];
    UCHAR       bSlot;
    UCHAR       bSeq;
    UCHAR       bBWI;
    USHORT      wLevelParameter;
    UCHAR       bPINOperation;
    UCHAR       bTimeOut;
    UCHAR       bmFormatString;
    UCHAR       bmPINBlockString;
    UCHAR       bmPINLengthFormat;
    UCHAR       bInsertionOffsetOld;
    UCHAR       bInsertionOffsetNew;
    UCHAR       wPINMaxExtraDigit[2]; /* XXYY, XX: min size, YY: max size.  */
    UCHAR       bConfirmPIN;
    UCHAR       bEntryValidationCondition;
    UCHAR       bNumberMessage;
    UCHAR       wLangId[2];
    UCHAR       bMsgIndex1;
    UCHAR       bMsgIndex2;
    UCHAR       bTeoPrologue[3];
} UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_MODIFICATION2;
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_MODIFICATION2_TEO_PROLOGUE(msg)   ((UCHAR*)(msg) + 26)
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_MODIFICATION2_DATA(msg)           ((UCHAR*)(msg) + 29)
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_MODIFICATION2_APDU(msg)           ((UCHAR*)(msg) + 29)
typedef struct UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_MODIFICATION3_STRUCT
{
    UCHAR       bMessageType;
    UCHAR       dwLength[4];
    UCHAR       bSlot;
    UCHAR       bSeq;
    UCHAR       bBWI;
    USHORT      wLevelParameter;
    UCHAR       bPINOperation;
    UCHAR       bTimeOut;
    UCHAR       bmFormatString;
    UCHAR       bmPINBlockString;
    UCHAR       bmPINLengthFormat;
    UCHAR       bInsertionOffsetOld;
    UCHAR       bInsertionOffsetNew;
    UCHAR       wPINMaxExtraDigit[2]; /* XXYY, XX: min size, YY: max size.  */
    UCHAR       bConfirmPIN;
    UCHAR       bEntryValidationCondition;
    UCHAR       bNumberMessage;
    UCHAR       wLangId[2];
    UCHAR       bMsgIndex1;
    UCHAR       bMsgIndex2;
    UCHAR       bMsgIndex3;
    UCHAR       bTeoPrologue[3];
} UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_MODIFICATION3;
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_MODIFICATION3_TEO_PROLOGUE(msg)   ((UCHAR*)(msg) + 27)
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_MODIFICATION3_DATA(msg)           ((UCHAR*)(msg) + 30)
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_PIN_MODIFICATION3_APDU(msg)           ((UCHAR*)(msg) + 30)


/* Define CCID PC_to_RDR_Mechanical.  */
typedef struct UX_DEVICE_CLASS_CCID_PC_TO_RDR_MECHANICAL_HEADER_STRUCT
{
    UCHAR       bMessageType;
    UCHAR       dwLength[4];
    UCHAR       bSlot;
    UCHAR       bSeq;
    UCHAR       bFunction;
    UCHAR       bRFU[2];
} UX_DEVICE_CLASS_CCID_PC_TO_RDR_MECHANICAL_HEADER;
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_MECHANICAL_FUNCTION(msg)     (((UX_DEVICE_CLASS_CCID_PC_TO_RDR_MECHANICAL_HEADER*)(msg))->bFunction)

/* Define CCID PC_to_RDR_Abort.  */
typedef UX_DEVICE_CLASS_CCID_MESSAGE_HEADER UX_DEVICE_CLASS_CCID_PC_TO_RDR_ABORT_HEADER;

/* Define CCID PC_to_RDR_SetDataRateAndClockFrequency.  */
typedef UX_DEVICE_CLASS_CCID_MESSAGE_HEADER UX_DEVICE_CLASS_CCID_PC_TO_RDR_SET_DATA_RATE_AND_CLOCK_FREQUENCY_HEADER;
typedef struct UX_DEVICE_CLASS_CCID_PC_TO_RDR_SET_DATA_RATE_AND_CLOCK_FREQUENCY_STRUCT
{
    UCHAR       bMessageType;
    UCHAR       dwLength[4];
    UCHAR       bSlot;
    UCHAR       bSeq;
    UCHAR       bRFU[3];
    UCHAR       dwClockFrequency[4];
    UCHAR       dwDataRate[4];
} UX_DEVICE_CLASS_CCID_PC_TO_RDR_SET_DATA_RATE_AND_CLOCK_FREQUENCY;
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_CLOCK_FREQUENCY_GET(msg)     _ux_utility_long_get(((UX_DEVICE_CLASS_CCID_PC_TO_RDR_SET_DATA_RATE_AND_CLOCK_FREQUENCY*)(msg))->dwClockFrequency)
#define UX_DEVICE_CLASS_CCID_PC_TO_RDR_DATA_RATE_GET(msg)           _ux_utility_long_get(((UX_DEVICE_CLASS_CCID_PC_TO_RDR_SET_DATA_RATE_AND_CLOCK_FREQUENCY*)(msg))->dwDataRate)

/* Define CCID RDR_to_PC_ header.  */
typedef struct UX_DEVICE_CLASS_CCID_RDR_TO_PC_HEADER_STRUCT
{
    UCHAR       bMessageType;
    UCHAR       dwLength[4];
    UCHAR       bSlot;
    UCHAR       bSeq;
    UCHAR       bStatus;
    UCHAR       bError;
    UCHAR       bRFU;
} UX_DEVICE_CLASS_CCID_RDR_TO_PC_HEADER;
#define UX_DEVICE_CLASS_CCID_RDR_TO_PC_STATUS(msg)                  (((UX_DEVICE_CLASS_CCID_RDR_TO_PC_HEADER*)(msg))->bStatus)
#define UX_DEVICE_CLASS_CCID_RDR_TO_PC_ERROR(msg)                   (((UX_DEVICE_CLASS_CCID_RDR_TO_PC_HEADER*)(msg))->bError)

/* Define CCID Slot Status Register.  */
typedef struct UX_DEVICE_CLASS_CCID_SLOT_STATUS_REGISTER_BITMAP_STRUCT
{
    UCHAR bmICCStatus:2;
    UCHAR bmRFU:4;
    UCHAR bmCommandStatus:2;
} UX_DEVICE_CLASS_CCID_SLOT_STATUS_REGISTER_BITMAP;

/* Define CCID RDR_to_PC_DataBlock.  */
typedef struct UX_DEVICE_CLASS_CCID_RDR_TO_PC_DATA_BLOCK_HEADER_STRUCT
{
    UCHAR       bMessageType;
    UCHAR       dwLength[4];
    UCHAR       bSlot;
    UCHAR       bSeq;
    UCHAR       bStatus;
    UCHAR       bError;
    UCHAR       bChainParameter;
} UX_DEVICE_CLASS_CCID_RDR_TO_PC_DATA_BLOCK_HEADER;
#define UX_DEVICE_CLASS_CCID_RDR_TO_PC_CHAIN_PARAMETER(msg)         (((UX_DEVICE_CLASS_CCID_RDR_TO_PC_DATA_BLOCK_HEADER*)(msg))->bChainParameter)

/* Define CCID RDR_to_PC_SlotStatus.  */
typedef struct UX_DEVICE_CLASS_CCID_RDR_TO_PC_SLOT_STATUS_HEADER_STRUCT
{
    UCHAR       bMessageType;
    UCHAR       dwLength[4];
    UCHAR       bSlot;
    UCHAR       bSeq;
    UCHAR       bStatus;
    UCHAR       bError;
    UCHAR       bClockStatus;
} UX_DEVICE_CLASS_CCID_RDR_TO_PC_SLOT_STATUS_HEADER;
#define UX_DEVICE_CLASS_CCID_RDR_TO_PC_CLOCK_STATUS(msg)            (((UX_DEVICE_CLASS_CCID_RDR_TO_PC_SLOT_STATUS_HEADER*)(msg))->bClockStatus)

/* Define CCID RDR_to_PC_Parameters.  */
typedef struct UX_DEVICE_CLASS_CCID_RDR_TO_PC_PARAMETERS_HEADER_STRUCT
{
    UCHAR       bMessageType;
    UCHAR       dwLength[4];
    UCHAR       bSlot;
    UCHAR       bSeq;
    UCHAR       bStatus;
    UCHAR       bError;
    UCHAR       bProtocolNum;
} UX_DEVICE_CLASS_CCID_RDR_TO_PC_PARAMETERS_HEADER;
#define UX_DEVICE_CLASS_CCID_RDR_TO_PC_PROTOCOL_NUM(msg)            (((UX_DEVICE_CLASS_CCID_RDR_TO_PC_PARAMETERS_HEADER*)(msg))->bProtocolNum)

typedef struct UX_DEVICE_CLASS_CCID_RDR_TO_PC_PARAMETERS_T0_STRUCT
{
    UCHAR       bMessageType;
    UCHAR       dwLength[4];
    UCHAR       bSlot;
    UCHAR       bSeq;
    UCHAR       bStatus;
    UCHAR       bError;
    UCHAR       bProtocolNum;
    UCHAR       bmFindexDindex;
    UCHAR       bmTCCKST0;
    UCHAR       bGuardTimeT0;
    UCHAR       bWaitingIntegerT0;
    UCHAR       bClockStop;
} UX_DEVICE_CLASS_CCID_RDR_TO_PC_PARAMETERS_T0;
#define UX_DEVICE_CLASS_CCID_RDR_TO_PC_PARAMETERS_T0_LENGTH         5

typedef struct UX_DEVICE_CLASS_CCID_RDR_TO_PC_PARAMETERS_T1_STRUCT
{
    UCHAR       bMessageType;
    UCHAR       dwLength[4];
    UCHAR       bSlot;
    UCHAR       bSeq;
    UCHAR       bStatus;
    UCHAR       bError;
    UCHAR       bProtocolNum;
    UCHAR       bmFindexDindex;
    UCHAR       bmTCCKST1;
    UCHAR       bGuardTimeT1;
    UCHAR       bmWaitingIntegersT1;
    UCHAR       bClockStop;
    UCHAR       bIFSC;
    UCHAR       bNadValue;
} UX_DEVICE_CLASS_CCID_RDR_TO_PC_PARAMETERS_T1;
#define UX_DEVICE_CLASS_CCID_RDR_TO_PC_PARAMETERS_T1_LENGTH         7

/* Define CCID RDR_to_PC_Escape.  */
typedef UX_DEVICE_CLASS_CCID_RDR_TO_PC_HEADER UX_DEVICE_CLASS_CCID_RDR_TO_PC_ESCAPE_HEADER;

/* Define CCID RDR_to_PC_DataRateAndClockFrequency.  */
typedef UX_DEVICE_CLASS_CCID_RDR_TO_PC_HEADER UX_DEVICE_CLASS_CCID_RDR_TO_PC_DATA_RATE_AND_CLOCK_FREQUENCY_HEADER;
typedef struct UX_DEVICE_CLASS_CCID_RDR_TO_PC_DATA_RATE_AND_CLOCK_FREQUENCY_STRUCT
{
    UCHAR       bMessageType;
    UCHAR       dwLength[4];
    UCHAR       bSlot;
    UCHAR       bSeq;
    UCHAR       bStatus;
    UCHAR       bError;
    UCHAR       bRFU;
    UCHAR       dwClockFrequency[4];
    UCHAR       dwDataRate[4];
} UX_DEVICE_CLASS_CCID_RDR_TO_PC_DATA_RATE_AND_CLOCK_FREQUENCY;
#define UX_DEVICE_CLASS_CCID_RDR_TO_PC_CLOCK_FREQUENCY_SET(msg,v)   _ux_utility_long_put(((UX_DEVICE_CLASS_CCID_RDR_TO_PC_DATA_RATE_AND_CLOCK_FREQUENCY*)(msg))->dwClockFrequency,(v))
#define UX_DEVICE_CLASS_CCID_RDR_TO_PC_DATA_RATE_SET(msg,v)         _ux_utility_long_put(((UX_DEVICE_CLASS_CCID_RDR_TO_PC_DATA_RATE_AND_CLOCK_FREQUENCY*)(msg))->dwDataRate,(v))


/* Define CCID message command handles, if command handle is not defined device
   reports command not supported to host.  */
typedef UINT (*UX_DEVICE_CLASS_CCID_HANDLE)(ULONG slot, UX_DEVICE_CLASS_CCID_MESSAGES*);
typedef struct UX_DEVICE_CLASS_CCID_HANDLES_STRUCT
{
    UX_DEVICE_CLASS_CCID_HANDLE ux_device_class_ccid_handles_icc_power_on;
    UX_DEVICE_CLASS_CCID_HANDLE ux_device_class_ccid_handles_icc_power_off;
    UX_DEVICE_CLASS_CCID_HANDLE ux_device_class_ccid_handles_get_slot_status;
    UX_DEVICE_CLASS_CCID_HANDLE ux_device_class_ccid_handles_xfr_block;
    UX_DEVICE_CLASS_CCID_HANDLE ux_device_class_ccid_handles_get_parameters;
    UX_DEVICE_CLASS_CCID_HANDLE ux_device_class_ccid_handles_reset_parameters;
    UX_DEVICE_CLASS_CCID_HANDLE ux_device_class_ccid_handles_set_parameters;
    UX_DEVICE_CLASS_CCID_HANDLE ux_device_class_ccid_handles_escape;
    UX_DEVICE_CLASS_CCID_HANDLE ux_device_class_ccid_handles_icc_clock;
    UX_DEVICE_CLASS_CCID_HANDLE ux_device_class_ccid_handles_t0_apdu;
    UX_DEVICE_CLASS_CCID_HANDLE ux_device_class_ccid_handles_secure;
    UX_DEVICE_CLASS_CCID_HANDLE ux_device_class_ccid_handles_mechanical;
    UX_DEVICE_CLASS_CCID_HANDLE ux_device_class_ccid_handles_abort;
    UX_DEVICE_CLASS_CCID_HANDLE ux_device_class_ccid_handles_set_data_rate_and_clock_frequency;
} UX_DEVICE_CLASS_CCID_HANDLES;

typedef struct UX_DEVICE_CLASS_CCID_COMMAND_SETT_STRUCT
{
    UCHAR ux_device_class_ccid_command_sett_command_type;
    UCHAR ux_device_class_ccid_command_sett_response_type;
    UCHAR ux_device_class_ccid_command_sett_flags;
    CHAR  ux_device_class_ccid_command_sett_handle_index;
} UX_DEVICE_CLASS_CCID_COMMAND_SETT;


/* Define CCID slot structure.  */
typedef struct UX_DEVICE_CLASS_CCID_SLOT_STRUCT
{
    CHAR                    ux_device_class_ccid_slot_runner;
    UCHAR                   ux_device_class_ccid_slot_flags;

    UCHAR                   ux_device_class_ccid_slot_icc_status;
    UCHAR                   ux_device_class_ccid_slot_clock_status;

    UCHAR                   ux_device_class_ccid_slot_hw_error;
    UCHAR                   ux_device_class_ccid_slot_hw_error_seq;

    UCHAR                   ux_device_class_ccid_slot_aborting_seq;
    UCHAR                   ux_device_class_ccid_slot_aborting;
} UX_DEVICE_CLASS_CCID_SLOT;

/* Define CCID runner structure.  */
typedef struct UX_DEVICE_CLASS_CCID_RUNNER_STRUCT
{
    struct UX_DEVICE_CLASS_CCID_STRUCT
                            *ux_device_class_ccid_runner_ccid;
    UCHAR                   *ux_device_class_ccid_runner_command;
    UCHAR                   *ux_device_class_ccid_runner_response;

    CHAR                    ux_device_class_ccid_runner_slot;
    CHAR                    ux_device_class_ccid_runner_id;
    CHAR                    ux_device_class_ccid_runner_command_index;
    UCHAR                   reserved;

#if !defined(UX_DEVICE_STANDALONE)
    UX_THREAD               ux_device_class_ccid_runner_thread;
    UCHAR                   *ux_device_class_ccid_runner_thread_stack;
#endif
} UX_DEVICE_CLASS_CCID_RUNNER;

/* Define Device CCID Class Calling Parameter structure */
typedef struct UX_DEVICE_CLASS_CCID_PARAMETER_STRUCT
{
    VOID                    (*ux_device_class_ccid_instance_activate)(VOID *);
    VOID                    (*ux_device_class_ccid_instance_deactivate)(VOID *);
    UX_DEVICE_CLASS_CCID_HANDLES
                            *ux_device_class_ccid_handles;
    ULONG                   *ux_device_class_ccid_clocks;
    ULONG                   *ux_device_class_ccid_data_rates;
    ULONG                   ux_device_class_ccid_max_transfer_length;
    UCHAR                   ux_device_class_ccid_max_n_slots;
    UCHAR                   ux_device_class_ccid_max_n_busy_slots;
    UCHAR                   ux_device_class_ccid_n_clocks;
    UCHAR                   ux_device_class_ccid_n_data_rates;
} UX_DEVICE_CLASS_CCID_PARAMETER;


/* Define CCID Class structure.  */

typedef struct UX_DEVICE_CLASS_CCID_STRUCT
{
    UX_SLAVE_INTERFACE      *ux_device_class_ccid_interface;
    UX_SLAVE_ENDPOINT       *ux_device_class_ccid_endpoint_out;
    UX_SLAVE_ENDPOINT       *ux_device_class_ccid_endpoint_in;
    UX_SLAVE_ENDPOINT       *ux_device_class_ccid_endpoint_notify;

    UX_DEVICE_CLASS_CCID_PARAMETER
                            ux_device_class_ccid_parameter;

    UX_DEVICE_CLASS_CCID_RUNNER
                            *ux_device_class_ccid_runners;
    UX_DEVICE_CLASS_CCID_SLOT
                            *ux_device_class_ccid_slots;

    UCHAR                   ux_device_class_ccid_header[UX_DEVICE_CLASS_CCID_MESSAGE_HEADER_LENGTH];
    SHORT                   ux_device_class_ccid_n_busy;

#if !defined(UX_DEVICE_STANDALONE)
    UX_THREAD               ux_device_class_ccid_thread;
    UCHAR                   *ux_device_class_ccid_thread_stack;

    UX_THREAD               ux_device_class_ccid_notify_thread;
    UCHAR                   *ux_device_class_ccid_notify_thread_stack;

    UX_EVENT_FLAGS_GROUP    ux_device_class_ccid_events;
    UX_MUTEX                ux_device_class_ccid_mutex;
    UX_MUTEX                ux_device_class_ccid_response_mutex;
    UX_SEMAPHORE            ux_device_class_ccid_notify_semaphore;
#endif
} UX_DEVICE_CLASS_CCID;

/* Define Device CCID command settings.  */
extern const UX_DEVICE_CLASS_CCID_COMMAND_SETT _ux_device_class_ccid_command_sett[];

/* Define Device CCID Class prototypes.  */
UINT  _ux_device_class_ccid_initialize(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_ccid_uninitialize(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_ccid_activate(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_ccid_deactivate(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_ccid_control_request(UX_SLAVE_CLASS_COMMAND *command);

VOID  _ux_device_class_ccid_thread_entry(ULONG ccid_instance);
VOID  _ux_device_class_ccid_notify_thread_entry(ULONG ccid_instance);
VOID  _ux_device_class_ccid_runner_thread_entry(ULONG runner_instance);

UINT  _ux_device_class_ccid_control_abort(UX_DEVICE_CLASS_CCID *ccid, ULONG slot, ULONG seq);

UINT  _ux_device_class_ccid_response(UX_DEVICE_CLASS_CCID *ccid, UCHAR *buffer, ULONG length);

UINT  _ux_device_class_ccid_entry(UX_SLAVE_CLASS_COMMAND *command);

UINT  _ux_device_class_ccid_icc_insert(UX_DEVICE_CLASS_CCID *ccid, ULONG slot, ULONG seq_start);
UINT  _ux_device_class_ccid_icc_remove(UX_DEVICE_CLASS_CCID *ccid, ULONG slot);
UINT  _ux_device_class_ccid_auto_seq_start(UX_DEVICE_CLASS_CCID *ccid, ULONG slot);
UINT  _ux_device_class_ccid_auto_seq_done(UX_DEVICE_CLASS_CCID *ccid, ULONG slot, ULONG icc_status);
UINT  _ux_device_class_ccid_time_extension(UX_DEVICE_CLASS_CCID *ccid, ULONG slot, ULONG wt);
UINT  _ux_device_class_ccid_hardware_error(UX_DEVICE_CLASS_CCID *ccid, ULONG slot, ULONG error);

/* Define Device CCID Class API prototypes.  */

#define ux_device_class_ccid_entry               _ux_device_class_ccid_entry

#define ux_device_class_ccid_icc_insert          _ux_device_class_ccid_icc_insert
#define ux_device_class_ccid_icc_remove          _ux_device_class_ccid_icc_remove
#define ux_device_class_ccid_auto_seq_done       _ux_device_class_ccid_auto_seq_done
#define ux_device_class_ccid_time_extension      _ux_device_class_ccid_time_extension
#define ux_device_class_ccid_hardware_error      _ux_device_class_ccid_hardware_error


/* Determine if a C++ compiler is being used.  If so, complete the standard
   C conditional started above.  */
#ifdef __cplusplus
}
#endif

#endif /* UX_DEVICE_CLASS_CCID_H */
