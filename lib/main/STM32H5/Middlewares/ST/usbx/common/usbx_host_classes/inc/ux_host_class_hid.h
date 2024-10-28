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
/**   HID Class                                                           */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_host_class_hid.h                                 PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX HID class.                                                     */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used UX prefix to refer to  */
/*                                            TX symbols instead of using */
/*                                            them directly, fixed struct */
/*                                            field definition issues,    */
/*                                            resulting in version 6.1    */
/*  08-02-2021     Wen Wang                 Modified comment(s),          */
/*                                            added extern "C" keyword    */
/*                                            for compatibility with C++, */
/*                                            resulting in version 6.1.8  */
/*  01-31-2022     Xiuwen Cai, CQ Xiao      Modified comment(s),          */
/*                                            added interrupt OUT support,*/
/*                                            added standalone mode,      */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/

#ifndef UX_HOST_CLASS_HID_H
#define UX_HOST_CLASS_HID_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard 
   C is used to process the API information.  */ 

#ifdef   __cplusplus 

/* Yes, C++ compiler is present.  Use standard C.  */ 
extern   "C" { 

#endif  


/* Define HID Class constants.  */

#define UX_HOST_CLASS_HID_CLASS                                 3
#define UX_HOST_CLASS_HID_FIELDS                                16
#define UX_HOST_CLASS_HID_MAX_COLLECTION                        4
#define UX_HOST_CLASS_HID_MAX_REPORT                            8
#define UX_HOST_CLASS_HID_REPORT_SIZE                           32
#define UX_HOST_CLASS_HID_DESCRIPTOR                            0x21
#define UX_HOST_CLASS_HID_ITEM_LENGTH_MASK                      3
#define UX_HOST_CLASS_HID_ITEM_TAG_MASK                         0xf0
#define UX_HOST_CLASS_HID_ITEM_TAG_SHORT                        1
#define UX_HOST_CLASS_HID_ITEM_TAG_LONG                         0xf0
#define UX_HOST_CLASS_HID_MAX_CLIENTS                           8
#define UX_HOST_CLASS_HID_MAX_CLIENT_NAME_LENGTH                63 /* Exclude string null-terminator */

#ifndef UX_HOST_CLASS_HID_DECOMPRESSION_BUFFER
#define UX_HOST_CLASS_HID_DECOMPRESSION_BUFFER                  4096
#endif
#define UX_HOST_CLASS_HID_REPORT_DECOMPRESSED                   0
#define UX_HOST_CLASS_HID_REPORT_RAW                            1
#define UX_HOST_CLASS_HID_REPORT_INDIVIDUAL_USAGE               2
#define UX_HOST_CLASS_HID_INTERRUPT_ENDPOINT_READY              1
#define UX_HOST_CLASS_HID_INTERRUPT_ENDPOINT_ACTIVE             2


/* Define HID Class item types.  */

#define UX_HOST_CLASS_HID_TYPE_MAIN                             0x0
#define UX_HOST_CLASS_HID_TYPE_GLOBAL                           0x1
#define UX_HOST_CLASS_HID_TYPE_LOCAL                            0x2
#define UX_HOST_CLASS_HID_TYPE_RESERVED                         0x3
                                                

/* Define HID Class main tags.  */

#define UX_HOST_CLASS_HID_MAIN_TAG_INPUT                        0x8
#define UX_HOST_CLASS_HID_MAIN_TAG_OUTPUT                       0x9
#define UX_HOST_CLASS_HID_MAIN_TAG_FEATURE                      0xb
#define UX_HOST_CLASS_HID_MAIN_TAG_COLLECTION                   0xa
#define UX_HOST_CLASS_HID_MAIN_TAG_END_COLLECTION               0xc
                                                

/* Define HID Class global tags.  */

#define UX_HOST_CLASS_HID_GLOBAL_TAG_USAGE_PAGE                 0x0
#define UX_HOST_CLASS_HID_GLOBAL_TAG_LOGICAL_MINIMUM            0x1
#define UX_HOST_CLASS_HID_GLOBAL_TAG_LOGICAL_MAXIMUM            0x2
#define UX_HOST_CLASS_HID_GLOBAL_TAG_PHYSICAL_MINIMUM           0x3
#define UX_HOST_CLASS_HID_GLOBAL_TAG_PHYSICAL_MAXIMUM           0x4
#define UX_HOST_CLASS_HID_GLOBAL_TAG_UNIT_EXPONENT              0x5
#define UX_HOST_CLASS_HID_GLOBAL_TAG_UNIT                       0x6
#define UX_HOST_CLASS_HID_GLOBAL_TAG_REPORT_SIZE                0x7
#define UX_HOST_CLASS_HID_GLOBAL_TAG_REPORT_ID                  0x8
#define UX_HOST_CLASS_HID_GLOBAL_TAG_REPORT_COUNT               0x9
#define UX_HOST_CLASS_HID_GLOBAL_TAG_PUSH                       0xa
#define UX_HOST_CLASS_HID_GLOBAL_TAG_POP                        0xb


/* Define HID Class local tags.  */

#define UX_HOST_CLASS_HID_LOCAL_TAG_USAGE                       0x0
#define UX_HOST_CLASS_HID_LOCAL_TAG_USAGE_MINIMUM               0x1
#define UX_HOST_CLASS_HID_LOCAL_TAG_USAGE_MAXIMUM               0x2
#define UX_HOST_CLASS_HID_LOCAL_TAG_DESIGNATOR_INDEX            0x3
#define UX_HOST_CLASS_HID_LOCAL_TAG_DESIGNATOR_MINIMUM          0x4
#define UX_HOST_CLASS_HID_LOCAL_TAG_DESIGNATOR_MAXIMUM          0x5
#define UX_HOST_CLASS_HID_LOCAL_TAG_STRING_INDEX                0x7
#define UX_HOST_CLASS_HID_LOCAL_TAG_STRING_MINIMUM              0x8
#define UX_HOST_CLASS_HID_LOCAL_TAG_STRING_MAXIMUM              0x9
#define UX_HOST_CLASS_HID_LOCAL_TAG_DELIMITER                   0xa


/* Define HID Class collection item types.  */

#define UX_HOST_CLASS_HID_COLLECTION_PHYSICAL                   0
#define UX_HOST_CLASS_HID_COLLECTION_APPLICATION                1
#define UX_HOST_CLASS_HID_COLLECTION_LOGICAL                    2


/* Define HID Class delimiter set.  */

#define UX_HOST_CLASS_HID_DELIMITER_OPEN                        1
#define UX_HOST_CLASS_HID_DELIMITER_CLOSE                       0


/* Define HID Class item bit masks.  */

#define UX_HOST_CLASS_HID_ITEM_CONSTANT                         0x0001
#define UX_HOST_CLASS_HID_ITEM_VARIABLE                         0x0002
#define UX_HOST_CLASS_HID_ITEM_RELATIVE                         0x0004
#define UX_HOST_CLASS_HID_ITEM_WRAP                             0x0008
#define UX_HOST_CLASS_HID_ITEM_NON_LINEAR                       0x0010
#define UX_HOST_CLASS_HID_ITEM_NO_PREFERRED_STATE               0x0020
#define UX_HOST_CLASS_HID_ITEM_NULL_STATE                       0x0040
#define UX_HOST_CLASS_HID_ITEM_VOLATILE                         0x0080
#define UX_HOST_CLASS_HID_ITEM_BUFFERED_BYTES                   0x0100


/* Define HID Class commands.  */

#define UX_HOST_CLASS_HID_GET_REPORT                            0x01       
#define UX_HOST_CLASS_HID_GET_IDLE                              0x02
#define UX_HOST_CLASS_HID_GET_PROTOCOL                          0x03
#define UX_HOST_CLASS_HID_SET_REPORT                            0x09       
#define UX_HOST_CLASS_HID_SET_IDLE                              0x0A
#define UX_HOST_CLASS_HID_SET_PROTOCOL                          0x0B


/* Define HID Class descriptors.  */

#define UX_HOST_CLASS_HID_DESCRIPTOR                            0x21
#define UX_HOST_CLASS_HID_REPORT_DESCRIPTOR                     0x22
#define UX_HOST_CLASS_HID_PHYSICAL_DESCRIPTOR                   0x23


#define UX_HID_DESCRIPTOR_ENTRIES                               7
#define UX_HID_DESCRIPTOR_LENGTH                                9


/* Define HID Class page constants.  */

#define UX_HOST_CLASS_HID_PAGE_GENERIC_DESKTOP_CONTROLS         0x01 
#define UX_HOST_CLASS_HID_PAGE_SIMULATION_CONTROLS              0x02 
#define UX_HOST_CLASS_HID_PAGE_VR_CONTROLS                      0x03 
#define UX_HOST_CLASS_HID_PAGE_SPORT_CONTROLS                   0x04 
#define UX_HOST_CLASS_HID_PAGE_GAME_CONTROLS                    0x05 
#define UX_HOST_CLASS_HID_PAGE_GENERIC_DEVICE_CONTROLS          0x06 
#define UX_HOST_CLASS_HID_PAGE_KEYBOARD_KEYPAD                  0x07 
#define UX_HOST_CLASS_HID_PAGE_LEDS                             0x08 
#define UX_HOST_CLASS_HID_PAGE_BUTTON                           0x09 
#define UX_HOST_CLASS_HID_PAGE_ORDINAL                          0x0A 
#define UX_HOST_CLASS_HID_PAGE_TELEPHONY                        0x0B 
#define UX_HOST_CLASS_HID_PAGE_CONSUMER                         0x0C 
#define UX_HOST_CLASS_HID_PAGE_DIGITIZER                        0x0D 
#define UX_HOST_CLASS_HID_PAGE_PHYSICAL_INTERFACE_DEVICE        0x0F 
#define UX_HOST_CLASS_HID_PAGE_UNICODE                          0x10 
#define UX_HOST_CLASS_HID_PAGE_ALPHANUMERIC_DISPLAY             0x14 
#define UX_HOST_CLASS_HID_PAGE_MEDICAL_INSTRUMENTS              0x40 
#define UX_HOST_CLASS_HID_PAGE_BAR_CODE_SCANNER                 0x8C 
#define UX_HOST_CLASS_HID_PAGE_SCALE_PAGE                       0x8D 
#define UX_HOST_CLASS_HID_PAGE_MAGNETIC_STRIPE_READING          0x8E 
#define UX_HOST_CLASS_HID_PAGE_CAMERA_CONTROL_PAGE              0x90 


/* Define HID Class generic desktop page constants.  */

#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_UNDEFINED             0x00 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_POINTER               0x01 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_MOUSE                 0x02 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_RESERVED              0x03 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_JOYSTICK              0x04 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_GAME PAD              0x05 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_KEYBOARD              0x06 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_KEYPAD                0x07 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_MULTI_AXIS_CONTROLLER 0x08 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_X                     0x30 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_Y                     0x31 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_Z                     0x32 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_RX                    0x33 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_RY                    0x34 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_RZ                    0x35 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SLIDER                0x36 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_DIAL                  0x37 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_WHEEL                 0x38 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_HAT_SWITCH            0x39 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_COUNTED_BUFFER        0x3A 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_BYTE_COUNT            0x3B 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_MOTION_WAKEUP         0x3C 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_START                 0x3D 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SELECT                0x3E 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_VX                    0x40 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_VY                    0x41 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_VZ                    0x42 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_VBRX                  0x43 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_VBRY                  0x44 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_VBRZ                  0x45 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_VNO                   0x46 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_FEATURE_NOTIFICATION  0x47 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_CONTROL        0x80 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_POWER_DOWN     0x81 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_SLEEP          0x82 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_WAKE_UP        0x83 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_CONTEXT_MENU   0x84 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_MAIN_MENU      0x85 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_APP_MENU       0x86 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_MENU_HELP      0x87 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_MENU_EXIT      0x88 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_MENU_SELECT    0x89 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_MENU_RIGHT     0x8A 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_MENU_LEFT      0x8B 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_MENU_UP        0x8C 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_MENU_DOWN      0x8D 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_COLD_RESTART   0x8E 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_WARM_RESTART   0x8F 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_D_PAD_UP              0x90 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_D_PAD_DOWN            0x91 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_D_PAD_RIGHT           0x92 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_D_PAD_LEFT            0x93 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_DOCK           0xA0 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_UNDOCK         0xA1 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_SETUP          0xA2 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_BREAK          0xA3 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_DEBUGGER_BREAK 0xA4 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_APPLICAION_BREAK      0xA5 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_APPLICATION_DEBUGGER_BREAK 0xA6 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_SPEAKER_MUTE   0xA7 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_HIBERNATE      0xA8 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_DISPLAY_INVERT 0xB0 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_DISPLAY_INTERNAL 0xB1 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_DISPLAY_EXTERNAL 0xB2 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_DISPLAY_BOTH   0xB3 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_DISPLAY_DUAL   0xB4 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_DISPLAY_TOGGLE 0xB5 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_DISPLAY_SWAP   0xB6 
#define UX_HOST_CLASS_HID_GENERIC_DESKTOP_SYSTEM_DISPLAY_LCD_AUTOSCALE 0xB7 


/* Define HID Class game control page constants.  */

#define UX_HOST_CLASS_HID_GAME_CONTROL_UNDEFINED                0x00 
#define UX_HOST_CLASS_HID_GAME_CONTROL_3D_GAME_CONTROLLER       0x01 
#define UX_HOST_CLASS_HID_GAME_CONTROL_PINBALL_DEVICE           0x02 
#define UX_HOST_CLASS_HID_GAME_CONTROL_GUN_DEVICE               0x03 
#define UX_HOST_CLASS_HID_GAME_CONTROL_POINT_OF_VIEW            0x20 
#define UX_HOST_CLASS_HID_GAME_CONTROL_TURN_RIGHT_LEFT          0x21 
#define UX_HOST_CLASS_HID_GAME_CONTROL_PITCH_FORWARD_BACKWARD   0x22 
#define UX_HOST_CLASS_HID_GAME_CONTROL_ROLL_RIGHT_LEFT          0x23 
#define UX_HOST_CLASS_HID_GAME_CONTROL_MOVE_RIGHT_LEFT          0x24 
#define UX_HOST_CLASS_HID_GAME_CONTROL_MOVE_FORWARD_BACKWARD    0x25 
#define UX_HOST_CLASS_HID_GAME_CONTROL_MOVE_UP_DOWN             0x26 
#define UX_HOST_CLASS_HID_GAME_CONTROL_LEAN_RIGHT_LEFT          0x27 
#define UX_HOST_CLASS_HID_GAME_CONTROL_LEAN_FORWARD_BACKWARD    0x28 
#define UX_HOST_CLASS_HID_GAME_CONTROL_HEIGHT_OF_POV            0x29 
#define UX_HOST_CLASS_HID_GAME_CONTROL_FLIPPER                  0x2A 
#define UX_HOST_CLASS_HID_GAME_CONTROL_SECONDARY_FLIPPER        0x2B 
#define UX_HOST_CLASS_HID_GAME_CONTROL_BUMP                     0x2C 
#define UX_HOST_CLASS_HID_GAME_CONTROL_NEW_GAME                 0x2D 
#define UX_HOST_CLASS_HID_GAME_CONTROL_SHOOT_BALL               0x2E 
#define UX_HOST_CLASS_HID_GAME_CONTROL_PLAYER                   0x2F 
#define UX_HOST_CLASS_HID_GAME_CONTROL_GUN_BOLT                 0x30 
#define UX_HOST_CLASS_HID_GAME_CONTROL_GUN_CLIP                 0x31 
#define UX_HOST_CLASS_HID_GAME_CONTROL_GUN_SELECTOR             0x32 
#define UX_HOST_CLASS_HID_GAME_CONTROL_GUN_SINGLE_SHOT          0x33        
#define UX_HOST_CLASS_HID_GAME_CONTROL_GUN_BURST                0x34 
#define UX_HOST_CLASS_HID_GAME_CONTROL_GUN_AUTOMATIC            0x35 
#define UX_HOST_CLASS_HID_GAME_CONTROL_GUN_SAFETY               0x36    
#define UX_HOST_CLASS_HID_GAME_CONTROL_GAMEAD_FIRE_JUMP         0x37 
#define UX_HOST_CLASS_HID_GAME_CONTROL_GAMEPAD_TRIGGER          0x39 


/* Define HID Class LED page constants.  */

#define UX_HOST_CLASS_HID_LED_UNDEFINED                         0x00
#define UX_HOST_CLASS_HID_LED_NUM_LOCK                          0x01
#define UX_HOST_CLASS_HID_LED_CAPS_LOCK                         0x02
#define UX_HOST_CLASS_HID_LED_SCROLL_LOCK                       0x03
#define UX_HOST_CLASS_HID_LED_COMPOSE                           0x04
#define UX_HOST_CLASS_HID_LED_KANA                              0x05
#define UX_HOST_CLASS_HID_LED_POWER                             0x06
#define UX_HOST_CLASS_HID_LED_SHIFT                             0x07
#define UX_HOST_CLASS_HID_LED_DO_NOT_DISTURB                    0x08
#define UX_HOST_CLASS_HID_LED_MUTE                              0x09
#define UX_HOST_CLASS_HID_LED_TONE_ENABLE                       0x0A
#define UX_HOST_CLASS_HID_LED_HIGH_CUT_FILTER                   0x0B
#define UX_HOST_CLASS_HID_LED_LOW_CUT_FILTER                    0x0C
#define UX_HOST_CLASS_HID_LED_EQUALIZER_ENABLE                  0x0D
#define UX_HOST_CLASS_HID_LED_SOUND_FIELD_ON                    0x0E
#define UX_HOST_CLASS_HID_LED_SURROUND_ON                       0x0F
#define UX_HOST_CLASS_HID_LED_REPEAT                            0x10
#define UX_HOST_CLASS_HID_LED_STEREO                            0x11
#define UX_HOST_CLASS_HID_LED_SAMPLING_RATE_DETECT              0x12
#define UX_HOST_CLASS_HID_LED_SPINNING                          0x13
#define UX_HOST_CLASS_HID_LED_CAV                               0x14
#define UX_HOST_CLASS_HID_LED_CLV                               0x15
#define UX_HOST_CLASS_HID_LED_RECORDING_FORMAT_DETECT           0x16
#define UX_HOST_CLASS_HID_LED_OFF_HOOK                          0x17
#define UX_HOST_CLASS_HID_LED_RING                              0x18
#define UX_HOST_CLASS_HID_LED_MESSAGE_WAITING                   0x19
#define UX_HOST_CLASS_HID_LED_DATA_MODE                         0x1A
#define UX_HOST_CLASS_HID_LED_BATTERY_OPERATION                 0x1B
#define UX_HOST_CLASS_HID_LED_BATTERY_OK                        0x1C
#define UX_HOST_CLASS_HID_LED_BATTERY_LOW                       0x1D
#define UX_HOST_CLASS_HID_LED_SPEAKER                           0x1E
#define UX_HOST_CLASS_HID_LED_HEAD_SET                          0x1F
#define UX_HOST_CLASS_HID_LED_HOLD                              0x20
#define UX_HOST_CLASS_HID_LED_MICROPHONE                        0x21
#define UX_HOST_CLASS_HID_LED_COVERAGE                          0x22
#define UX_HOST_CLASS_HID_LED_NIGHT_MODE                        0x23
#define UX_HOST_CLASS_HID_LED_SEND_CALLS                        0x24
#define UX_HOST_CLASS_HID_LED_CALL_PICKUP                       0x25
#define UX_HOST_CLASS_HID_LED_CONFERENCE                        0x26
#define UX_HOST_CLASS_HID_LED_STAND_BY                          0x27
#define UX_HOST_CLASS_HID_LED_CAMERA_ON                         0x28
#define UX_HOST_CLASS_HID_LED_CAMERA_OFF                        0x29
#define UX_HOST_CLASS_HID_LED_ON_LINE                           0x2A
#define UX_HOST_CLASS_HID_LED_OFF_LINE                          0x2B
#define UX_HOST_CLASS_HID_LED_BUSY                              0x2C
#define UX_HOST_CLASS_HID_LED_READY                             0x2D
#define UX_HOST_CLASS_HID_LED_PAPER_OUT                         0x2E
#define UX_HOST_CLASS_HID_LED_PAPER_JAM                         0x2F
#define UX_HOST_CLASS_HID_LED_REMOTE                            0x30
#define UX_HOST_CLASS_HID_LED_FORWARD                           0x31
#define UX_HOST_CLASS_HID_LED_REVERSE                           0x32
#define UX_HOST_CLASS_HID_LED_STOP                              0x33
#define UX_HOST_CLASS_HID_LED_REWIND                            0x34
#define UX_HOST_CLASS_HID_LED_FAST_FORWARD                      0x35
#define UX_HOST_CLASS_HID_LED_PLAY                              0x36
#define UX_HOST_CLASS_HID_LED_PAUSE                             0x37
#define UX_HOST_CLASS_HID_LED_RECORD                            0x38
#define UX_HOST_CLASS_HID_LED_ERROR                             0x39
#define UX_HOST_CLASS_HID_LED_USAGE_SELECTED_INDICATOR          0x3A
#define UX_HOST_CLASS_HID_LED_USAGE_IN_USE_INDICATOR            0x3B
#define UX_HOST_CLASS_HID_LED_USAGE MULTI_MODE_INDICATOR        0x3C
#define UX_HOST_CLASS_HID_LED_INDICATOR_ON                      0x3D
#define UX_HOST_CLASS_HID_LED_INDICATOR_FLASH                   0x3E
#define UX_HOST_CLASS_HID_LED_INDICATOR_SLOW_BLINK              0x3F
#define UX_HOST_CLASS_HID_LED_INDICATOR_FAST_BLINK              0x40
#define UX_HOST_CLASS_HID_LED_INDICATOR_OFF                     0x41
#define UX_HOST_CLASS_HID_LED_FLASH_ON_TIME                     0x42
#define UX_HOST_CLASS_HID_LED_SLOW_BLINK_ON_TIME                0x43
#define UX_HOST_CLASS_HID_LED_SLOW_BLINK_OFF_TIME               0x44
#define UX_HOST_CLASS_HID_LED_FAST_BLINK_ON_TIME                0x45
#define UX_HOST_CLASS_HID_LED_FAST_BLINK_OFF_TIME               0x46
#define UX_HOST_CLASS_HID_LED_USAGE_INDICATOR_COLOR             0x47
#define UX_HOST_CLASS_HID_LED_INDICATOR_RED                     0x48
#define UX_HOST_CLASS_HID_LED_INDICATOR_GREEN                   0x49
#define UX_HOST_CLASS_HID_LED_INDICATOR_AMBER                   0x4A
#define UX_HOST_CLASS_HID_LED_GENERIC_INDICATOR                 0x4B
#define UX_HOST_CLASS_HID_LED_SYSTEM_SUSPEND                    0x4C
#define UX_HOST_CLASS_HID_LED_EXTERNAL_POWER_CONNECTED          0x4D


/* Define HID Class consumer page constants.  */

#define UX_HOST_CLASS_HID_CONSUMER_UNASSIGNED                   0x00
#define UX_HOST_CLASS_HID_CONSUMER_REMOTE_CONTROL               0x01
#define UX_HOST_CLASS_HID_CONSUMER_NUMERIC_KEY_PAD              0x02
#define UX_HOST_CLASS_HID_CONSUMER_PROGRAMMABLE_BUTTONS         0x03
#define UX_HOST_CLASS_HID_CONSUMER_MICROPHONE                   0x04
#define UX_HOST_CLASS_HID_CONSUMER_HEADPHONE                    0x05
#define UX_HOST_CLASS_HID_CONSUMER_GRAPHIC_EQUALIZER            0x06
#define UX_HOST_CLASS_HID_CONSUMER_PLUS_10                      0x20
#define UX_HOST_CLASS_HID_CONSUMER_PLUS_100                     0x21
#define UX_HOST_CLASS_HID_CONSUMER_AM_PM                        0x22
#define UX_HOST_CLASS_HID_CONSUMER_POWER                        0x30
#define UX_HOST_CLASS_HID_CONSUMER_RESET                        0x31
#define UX_HOST_CLASS_HID_CONSUMER_SLEEP                        0x32
#define UX_HOST_CLASS_HID_CONSUMER_SLEEP_AFTER                  0x33
#define UX_HOST_CLASS_HID_CONSUMER_SLEEP_MODE_RTC               0x34
#define UX_HOST_CLASS_HID_CONSUMER_ILLUMINATION                 0x35
#define UX_HOST_CLASS_HID_CONSUMER_FUNCTION_BUTTONS             0x36
#define UX_HOST_CLASS_HID_CONSUMER_MENU                         0x40
#define UX_HOST_CLASS_HID_CONSUMER_MENU_PICK                    0x41
#define UX_HOST_CLASS_HID_CONSUMER_MENU_UP                      0x42
#define UX_HOST_CLASS_HID_CONSUMER_MENU_DOWN                    0x43
#define UX_HOST_CLASS_HID_CONSUMER_MENU_LEFT                    0x44
#define UX_HOST_CLASS_HID_CONSUMER_MENU_RIGHT                   0x45
#define UX_HOST_CLASS_HID_CONSUMER_MENU_ESCAPE                  0x46
#define UX_HOST_CLASS_HID_CONSUMER_MENU_VALUE_INCREASE          0x47
#define UX_HOST_CLASS_HID_CONSUMER_MENU_VALUE_DECREASE          0x48
#define UX_HOST_CLASS_HID_CONSUMER_DATA_ON_SCREEN               0x60
#define UX_HOST_CLASS_HID_CONSUMER_CLOSED_CAPTION               0x61
#define UX_HOST_CLASS_HID_CONSUMER_CLOSED_CAPTION_SELECT        0x62
#define UX_HOST_CLASS_HID_CONSUMER_VCR_TV                       0x63
#define UX_HOST_CLASS_HID_CONSUMER_BROADCAST_MODE               0x64
#define UX_HOST_CLASS_HID_CONSUMER_SNAPSHOT                     0x65
#define UX_HOST_CLASS_HID_CONSUMER_STILL                        0x66
#define UX_HOST_CLASS_HID_CONSUMER_SELECTION                    0x80
#define UX_HOST_CLASS_HID_CONSUMER_ASSIGN_SELECTION             0x81
#define UX_HOST_CLASS_HID_CONSUMER_MODE_STEP                    0x82
#define UX_HOST_CLASS_HID_CONSUMER_RECALL_LAST                  0x83
#define UX_HOST_CLASS_HID_CONSUMER_ENTER_CHANNEL                0x84
#define UX_HOST_CLASS_HID_CONSUMER_ORDER_MOVIE                  0x85
#define UX_HOST_CLASS_HID_CONSUMER_CHANNEL_L                    0x86
#define UX_HOST_CLASS_HID_CONSUMER_MEDIA_SELECTION              0x87
#define UX_HOST_CLASS_HID_CONSUMER_MEDIA_SELECT_COMPUTER        0x88
#define UX_HOST_CLASS_HID_CONSUMER_MEDIA_SELECT_TV              0x89
#define UX_HOST_CLASS_HID_CONSUMER_MEDIA_SELECT_WWW             0x8A
#define UX_HOST_CLASS_HID_CONSUMER_MEDIA_SELECT_DVD             0x8B
#define UX_HOST_CLASS_HID_CONSUMER_MEDIA_SELECT_TELEPHONE       0x8C
#define UX_HOST_CLASS_HID_CONSUMER_MEDIA_SELECT_PROGRAM_GUIDE   0x8D
#define UX_HOST_CLASS_HID_CONSUMER_MEDIA_SELECT_VIDEO_PHONE     0x8E
#define UX_HOST_CLASS_HID_CONSUMER_MEDIA_SELECT_GAMES           0x8F
#define UX_HOST_CLASS_HID_CONSUMER_MEDIA_SELECT_MESSAGES        0x90
#define UX_HOST_CLASS_HID_CONSUMER_MEDIA_SELECT_CD              0x91
#define UX_HOST_CLASS_HID_CONSUMER_MEDIA_SELECT_VCR             0x92
#define UX_HOST_CLASS_HID_CONSUMER_MEDIA_SELECT_TUNER           0x93
#define UX_HOST_CLASS_HID_CONSUMER_QUIT                         0x94
#define UX_HOST_CLASS_HID_CONSUMER_HELP                         0x95
#define UX_HOST_CLASS_HID_CONSUMER_MEDIA_SELECT_TAPE            0x96
#define UX_HOST_CLASS_HID_CONSUMER_MEDIA_SELECT_CABLE           0x97
#define UX_HOST_CLASS_HID_CONSUMER_MEDIA_SELECT_SATELLITE       0x98
#define UX_HOST_CLASS_HID_CONSUMER_MEDIA_SELECT_SECURITY        0x99
#define UX_HOST_CLASS_HID_CONSUMER_MEDIA_SELECT_HOME            0x9A
#define UX_HOST_CLASS_HID_CONSUMER_MEDIA_SELECT_CALL            0x9B
#define UX_HOST_CLASS_HID_CONSUMER_CHANNEL_INCREMENT            0x9C
#define UX_HOST_CLASS_HID_CONSUMER_CHANNEL_DECREMENT            0x9D
#define UX_HOST_CLASS_HID_CONSUMER_MEDIA_SELECT                 0x9E
#define UX_HOST_CLASS_HID_CONSUMER_VCR_PLUS                     0xA0
#define UX_HOST_CLASS_HID_CONSUMER_ONCE                         0xA1
#define UX_HOST_CLASS_HID_CONSUMER_DAILY                        0xA2
#define UX_HOST_CLASS_HID_CONSUMER_WEEKLY                       0xA3
#define UX_HOST_CLASS_HID_CONSUMER_MONTHLY                      0xA4
#define UX_HOST_CLASS_HID_CONSUMER_PLAY                         0xB0
#define UX_HOST_CLASS_HID_CONSUMER_PAUSE                        0xB1
#define UX_HOST_CLASS_HID_CONSUMER_RECORD                       0xB2
#define UX_HOST_CLASS_HID_CONSUMER_FAST_FORWARD                 0xB3
#define UX_HOST_CLASS_HID_CONSUMER_REWIND                       0xB4
#define UX_HOST_CLASS_HID_CONSUMER_SCAN_NEXT_TRACK              0xB5
#define UX_HOST_CLASS_HID_CONSUMER_SCAN_PREVIOUS_TRACK          0xB6
#define UX_HOST_CLASS_HID_CONSUMER_STOP                         0xB7
#define UX_HOST_CLASS_HID_CONSUMER_EJECT                        0xB8
#define UX_HOST_CLASS_HID_CONSUMER_RANDOM_PLAY                  0xB9
#define UX_HOST_CLASS_HID_CONSUMER_SELECT_DISC                  0xBA
#define UX_HOST_CLASS_HID_CONSUMER_ENTER_DISC                   0xBB
#define UX_HOST_CLASS_HID_CONSUMER_REPEAT                       0xBC
#define UX_HOST_CLASS_HID_CONSUMER_TRACKING                     0xBD
#define UX_HOST_CLASS_HID_CONSUMER_TRACK_NORMAL                 0xBE
#define UX_HOST_CLASS_HID_CONSUMER_SLOW_TRACKING                0xBF
#define UX_HOST_CLASS_HID_CONSUMER_FRAME_FORWARD                0xC0
#define UX_HOST_CLASS_HID_CONSUMER_FRAME_BACK                   0xC1
#define UX_HOST_CLASS_HID_CONSUMER_MARK                         0xC2
#define UX_HOST_CLASS_HID_CONSUMER_CLEAR_MARK                   0xC3
#define UX_HOST_CLASS_HID_CONSUMER_REPEAT_FROM_MARK             0xC4
#define UX_HOST_CLASS_HID_CONSUMER_RETURN_TO_MARK               0xC5
#define UX_HOST_CLASS_HID_CONSUMER_SEARCH_MARK_FORWARD          0xC6
#define UX_HOST_CLASS_HID_CONSUMER_SEARCH_MARK_BACKWARDS        0xC7
#define UX_HOST_CLASS_HID_CONSUMER_COUNTER_RESET                0xC8
#define UX_HOST_CLASS_HID_CONSUMER_SHOW_COUNTER                 0xC9
#define UX_HOST_CLASS_HID_CONSUMER_TRACKING_INCREMENT           0xCA
#define UX_HOST_CLASS_HID_CONSUMER_TRACKING_DECREMENT           0xCB
#define UX_HOST_CLASS_HID_CONSUMER_STOP_EJECT                   0xCC
#define UX_HOST_CLASS_HID_CONSUMER_PLAY_PAUSE                   0xCD
#define UX_HOST_CLASS_HID_CONSUMER_PLAY_SKIP                    0xCE
#define UX_HOST_CLASS_HID_CONSUMER_VOLUME                       0xE0
#define UX_HOST_CLASS_HID_CONSUMER_BALANCE                      0xE1
#define UX_HOST_CLASS_HID_CONSUMER_MUTE                         0xE2
#define UX_HOST_CLASS_HID_CONSUMER_BASS                         0xE3
#define UX_HOST_CLASS_HID_CONSUMER_TREBLE                       0xE4
#define UX_HOST_CLASS_HID_CONSUMER_BASS_BOOST                   0xE5
#define UX_HOST_CLASS_HID_CONSUMER_SURROUND_MODE                0xE6
#define UX_HOST_CLASS_HID_CONSUMER_LOUDNESS                     0xE7
#define UX_HOST_CLASS_HID_CONSUMER_MPX                          0xE8
#define UX_HOST_CLASS_HID_CONSUMER_VOLUME_INCREMENT             0xE9
#define UX_HOST_CLASS_HID_CONSUMER_VOLUME_DECREMENT             0xEA
#define UX_HOST_CLASS_HID_CONSUMER_SPEED_SELECT                 0xF0
#define UX_HOST_CLASS_HID_CONSUMER_PLAYBACK_SPEED               0xF1
#define UX_HOST_CLASS_HID_CONSUMER_STANDARD_PLAY                0xF2
#define UX_HOST_CLASS_HID_CONSUMER_LONG_PLAY                    0xF3
#define UX_HOST_CLASS_HID_CONSUMER_EXTENDED_PLAY                0xF4
#define UX_HOST_CLASS_HID_CONSUMER_SLOW                         0xF5
#define UX_HOST_CLASS_HID_CONSUMER_FAN_ENABLE                   0xF6
#define UX_HOST_CLASS_HID_CONSUMER_FAN_SPEED                    0x100
#define UX_HOST_CLASS_HID_CONSUMER_LIGHT_ENABLE                 0x101
#define UX_HOST_CLASS_HID_CONSUMER_LIGHT_ILLUMINATION_LEVEL     0x102
#define UX_HOST_CLASS_HID_CONSUMER_CLIMATE_CONTROL_ENABLE       0x103
#define UX_HOST_CLASS_HID_CONSUMER_ROOM_TEMPERATURE             0x104
#define UX_HOST_CLASS_HID_CONSUMER_SECURITY_ENABLE              0x105
#define UX_HOST_CLASS_HID_CONSUMER_FIRE_ALARM                   0x106
#define UX_HOST_CLASS_HID_CONSUMER_POLICE_ALARM                 0x107
#define UX_HOST_CLASS_HID_CONSUMER_PROXIMITY                    0x108
#define UX_HOST_CLASS_HID_CONSUMER_MOTION                       0x109
#define UX_HOST_CLASS_HID_CONSUMER_DURESS_ALARM                 0x10A
#define UX_HOST_CLASS_HID_CONSUMER_HOLDUP_ALARM                 0x10B
#define UX_HOST_CLASS_HID_CONSUMER_MEDICAL_ALARM                0x10C
#define UX_HOST_CLASS_HID_CONSUMER_BALANCE_RIGHT                0x10D
#define UX_HOST_CLASS_HID_CONSUMER_BALANCE_LEFT                 0x150
#define UX_HOST_CLASS_HID_CONSUMER_BASS_INCREMENT               0x151
#define UX_HOST_CLASS_HID_CONSUMER_BASS_DECREMENT               0x152
#define UX_HOST_CLASS_HID_CONSUMER_TREBLE_INCREMENT             0x153
#define UX_HOST_CLASS_HID_CONSUMER_TREBLE_DECREMENT             0x154
#define UX_HOST_CLASS_HID_CONSUMER_SPEAKER_SYSTEM               0x155
#define UX_HOST_CLASS_HID_CONSUMER_CHANNEL_LEFT                 0x160
#define UX_HOST_CLASS_HID_CONSUMER_CHANNEL_RIGHT                0x161
#define UX_HOST_CLASS_HID_CONSUMER_CHANNEL_CENTER               0x162
#define UX_HOST_CLASS_HID_CONSUMER_CHANNEL_FRONT                0x163
#define UX_HOST_CLASS_HID_CONSUMER_CHANNEL_CENTER_FRONT         0x164
#define UX_HOST_CLASS_HID_CONSUMER_CHANNEL_SIDE                 0x165
#define UX_HOST_CLASS_HID_CONSUMER_CHANNEL_SURROUND             0x166
#define UX_HOST_CLASS_HID_CONSUMER_CHANNEL_LOW_FREQUENCY        0x167
#define UX_HOST_CLASS_HID_CONSUMER_CHANNEL_TOP                  0x168
#define UX_HOST_CLASS_HID_CONSUMER_CHANNEL_UNKNOWN              0x169
#define UX_HOST_CLASS_HID_CONSUMER_SUB_CHANNEL                  0x16A
#define UX_HOST_CLASS_HID_CONSUMER_SUB_CHANNEL_INCREMENT        0x170
#define UX_HOST_CLASS_HID_CONSUMER_SUB_CHANNEL_DECREMENT        0x171
#define UX_HOST_CLASS_HID_CONSUMER_ALTERNATE_AUDIO_INCREMENT    0x172
#define UX_HOST_CLASS_HID_CONSUMER_ALTERNATE_AUDIO_DECREMENT    0x173
#define UX_HOST_CLASS_HID_CONSUMER_APPLICATION_LAUNCH_BUTTONS   0x174
#define UX_HOST_CLASS_HID_CONSUMER_AL_LAUNCH_BUTTON_CONFIGURATION 0x180
#define UX_HOST_CLASS_HID_CONSUMER_AL_PROGRAMMABLE_BUTTON       0x181
#define UX_HOST_CLASS_HID_CONSUMER_AL_CONSUMER_CONTROL_CONFIGURATION 0x182
#define UX_HOST_CLASS_HID_CONSUMER_AL_WORD_PROCESSOR            0x183
#define UX_HOST_CLASS_HID_CONSUMER_AL_TEXT_EDITOR               0x184
#define UX_HOST_CLASS_HID_CONSUMER_AL_SPREADSHEET               0x185
#define UX_HOST_CLASS_HID_CONSUMER_AL_GRAPHICS_EDITOR           0x186
#define UX_HOST_CLASS_HID_CONSUMER_AL_PRESENTATION_APP          0x187
#define UX_HOST_CLASS_HID_CONSUMER_AL_DATABASE_APP              0x188
#define UX_HOST_CLASS_HID_CONSUMER_AL_EMAIL_READER              0x189
#define UX_HOST_CLASS_HID_CONSUMER_AL_NEWSREADER                0x18A
#define UX_HOST_CLASS_HID_CONSUMER_AL_VOICEMAIL                 0x18B
#define UX_HOST_CLASS_HID_CONSUMER_AL_CONTACTS_ADDRESS_BOOK     0x18C
#define UX_HOST_CLASS_HID_CONSUMER_AL_CALENDAR_SCHEDULE         0x18D
#define UX_HOST_CLASS_HID_CONSUMER_AL_TASK_PROJECT_MANAGER      0x18E
#define UX_HOST_CLASS_HID_CONSUMER_AL_LOG_JOURNAL_TIMECARD      0x18F
#define UX_HOST_CLASS_HID_CONSUMER_AL_CHECKBOOK_FINANCE         0x190
#define UX_HOST_CLASS_HID_CONSUMER_AL_CALCULATOR                0x191
#define UX_HOST_CLASS_HID_CONSUMER_AL_A_V_CAPTURE_PLAYBACK      0x192
#define UX_HOST_CLASS_HID_CONSUMER_AL_LOCAL_MACHINE_BROWSER     0x193
#define UX_HOST_CLASS_HID_CONSUMER_AL_LAN_WAN_BROWSER           0x194
#define UX_HOST_CLASS_HID_CONSUMER_AL_INTERNET_BROWSER          0x195
#define UX_HOST_CLASS_HID_CONSUMER_AL_REMOTE_NETWORKING         0x196
#define UX_HOST_CLASS_HID_CONSUMER_AL_NETWORK_CONFERENCE        0x197
#define UX_HOST_CLASS_HID_CONSUMER_AL_NETWORK_CHAT              0x198
#define UX_HOST_CLASS_HID_CONSUMER_AL_TELEPHONY_DIALER          0x199
#define UX_HOST_CLASS_HID_CONSUMER_AL_LOGON                     0x19A
#define UX_HOST_CLASS_HID_CONSUMER_AL_LOGOFF                    0x19B
#define UX_HOST_CLASS_HID_CONSUMER_AL_LOGON_LOGOFF              0x19C
#define UX_HOST_CLASS_HID_CONSUMER_AL_SCREENSAVER               0x19D
#define UX_HOST_CLASS_HID_CONSUMER_AL_CONTROL_PANEL             0x19E
#define UX_HOST_CLASS_HID_CONSUMER_AL_COMMAND_LINE_PROCESSOR    0x19F
#define UX_HOST_CLASS_HID_CONSUMER_AL_PROCESS_MANAGER           0x1A0
#define UX_HOST_CLASS_HID_CONSUMER_AL_SELECT_APPLICATION        0x1A1
#define UX_HOST_CLASS_HID_CONSUMER_AL_NEXT_APPLICATION          0x1A2
#define UX_HOST_CLASS_HID_CONSUMER_AL_PREVIOUS_APPLICATION      0x1A3
#define UX_HOST_CLASS_HID_CONSUMER_AL_PREEMPTIVE_HALT_APPLICATION 0x1A4
#define UX_HOST_CLASS_HID_CONSUMER_AL_INTEGRATED_HELP_CENTER    0x1A5
#define UX_HOST_CLASS_HID_CONSUMER_AL_DOCUMENTS                 0x1A6
#define UX_HOST_CLASS_HID_CONSUMER_AL_THESAURUS                 0x1A7
#define UX_HOST_CLASS_HID_CONSUMER_AL_DICTIONARY                0x1A8
#define UX_HOST_CLASS_HID_CONSUMER_AL_DESKTOP                   0x1A9
#define UX_HOST_CLASS_HID_CONSUMER_AL_SPELL_CHECK               0x1AA
#define UX_HOST_CLASS_HID_CONSUMER_AL_GRAMMAR_CHECK             0x1AB
#define UX_HOST_CLASS_HID_CONSUMER_AL_WIRELESS_STATUS           0x1AC
#define UX_HOST_CLASS_HID_CONSUMER_AL_KEYBOARD_LAYOUT           0x1AD
#define UX_HOST_CLASS_HID_CONSUMER_AL_VIRUS_PROTECTION          0x1AE
#define UX_HOST_CLASS_HID_CONSUMER_AL_ENCRYPTION                0x1AF
#define UX_HOST_CLASS_HID_CONSUMER_AL_SCREEN_SAVER              0x1B0
#define UX_HOST_CLASS_HID_CONSUMER_AL_ALARMS                    0x1B1
#define UX_HOST_CLASS_HID_CONSUMER_AL_CLOCK                     0x1B2
#define UX_HOST_CLASS_HID_CONSUMER_AL_FILE_BROWSER              0x1B3
#define UX_HOST_CLASS_HID_CONSUMER_AL_POWER_STATUS              0x1B4
#define UX_HOST_CLASS_HID_CONSUMER_AC_NEW                       0x1B5
#define UX_HOST_CLASS_HID_CONSUMER_AC_OPEN                      0x201
#define UX_HOST_CLASS_HID_CONSUMER_AC_CLOSE                     0x202
#define UX_HOST_CLASS_HID_CONSUMER_AC_EXIT                      0x203
#define UX_HOST_CLASS_HID_CONSUMER_AC_MAXIMIZE                  0x204
#define UX_HOST_CLASS_HID_CONSUMER_AC_MINIMIZE                  0x205
#define UX_HOST_CLASS_HID_CONSUMER_AC_SAVE                      0x206
#define UX_HOST_CLASS_HID_CONSUMER_AC_PRINT                     0x207
#define UX_HOST_CLASS_HID_CONSUMER_AC_PROPERTIES                0x208
#define UX_HOST_CLASS_HID_CONSUMER_AC_UNDO                      0x209
#define UX_HOST_CLASS_HID_CONSUMER_AC_COPY                      0x21A
#define UX_HOST_CLASS_HID_CONSUMER_AC_CUT                       0x21B
#define UX_HOST_CLASS_HID_CONSUMER_AC_PASTE                     0x21C
#define UX_HOST_CLASS_HID_CONSUMER_AC_SELECT_ALL                0x21D
#define UX_HOST_CLASS_HID_CONSUMER_AC_FIND                      0x21E
#define UX_HOST_CLASS_HID_CONSUMER_AC_FIND_AND_REPLACE          0x21F
#define UX_HOST_CLASS_HID_CONSUMER_AC_SEARCH                    0x220
#define UX_HOST_CLASS_HID_CONSUMER_AC_GO_TO                     0x221
#define UX_HOST_CLASS_HID_CONSUMER_AC_HOME                      0x222
#define UX_HOST_CLASS_HID_CONSUMER_AC_BACK                      0x223
#define UX_HOST_CLASS_HID_CONSUMER_AC_FORWARD                   0x224
#define UX_HOST_CLASS_HID_CONSUMER_AC_STOP                      0x225
#define UX_HOST_CLASS_HID_CONSUMER_AC_REFRESH                   0x226
#define UX_HOST_CLASS_HID_CONSUMER_AC_PREVIOUS_LINK             0x227
#define UX_HOST_CLASS_HID_CONSUMER_AC_NEXT_LINK                 0x228
#define UX_HOST_CLASS_HID_CONSUMER_AC_BOOKMARKS                 0x229
#define UX_HOST_CLASS_HID_CONSUMER_AC_HISTORY                   0x22A
#define UX_HOST_CLASS_HID_CONSUMER_AC_SUBSCRIPTIONS             0x22B
#define UX_HOST_CLASS_HID_CONSUMER_AC_ZOOM_IN                   0x22C
#define UX_HOST_CLASS_HID_CONSUMER_AC_ZOOM_OUT                  0x22D
#define UX_HOST_CLASS_HID_CONSUMER_AC_ZOOM                      0x22E
#define UX_HOST_CLASS_HID_CONSUMER_AC_FULL_SCREEN_VIEW          0x22F
#define UX_HOST_CLASS_HID_CONSUMER_AC_NORMAL_VIEW               0x230
#define UX_HOST_CLASS_HID_CONSUMER_AC_VIEW_TOGGLE               0x231
#define UX_HOST_CLASS_HID_CONSUMER_AC_SCROLL_UP                 0x232
#define UX_HOST_CLASS_HID_CONSUMER_AC_SCROLL_DOWN               0x233
#define UX_HOST_CLASS_HID_CONSUMER_AC_SCROLL                    0x234
#define UX_HOST_CLASS_HID_CONSUMER_AC_PAN_LEFT                  0x235
#define UX_HOST_CLASS_HID_CONSUMER_AC_PAN_RIGHT                 0x236
#define UX_HOST_CLASS_HID_CONSUMER_AC_PAN                       0x237
#define UX_HOST_CLASS_HID_CONSUMER_AC_NEW_WINDOW                0x238
#define UX_HOST_CLASS_HID_CONSUMER_AC_TILE_HORIZONTALLY         0x239
#define UX_HOST_CLASS_HID_CONSUMER_AC_TILE_VERTICALLY           0x23A
#define UX_HOST_CLASS_HID_CONSUMER_AC_FORMAT                    0x23B
#define UX_HOST_CLASS_HID_CONSUMER_AC_EDIT                      0x23C
#define UX_HOST_CLASS_HID_CONSUMER_AC_BOLD                      0x23D
#define UX_HOST_CLASS_HID_CONSUMER_AC_ITALICS                   0x23E
#define UX_HOST_CLASS_HID_CONSUMER_AC_UNDERLINE                 0x23F
#define UX_HOST_CLASS_HID_CONSUMER_AC_STRIKETHROUGH             0x240
#define UX_HOST_CLASS_HID_CONSUMER_AC_SUBSCRIPT                 0x241
#define UX_HOST_CLASS_HID_CONSUMER_AC_SUPERSCRIPT               0x242
#define UX_HOST_CLASS_HID_CONSUMER_AC_ALL_CAPS                  0x243
#define UX_HOST_CLASS_HID_CONSUMER_AC_ROTATE                    0x244
#define UX_HOST_CLASS_HID_CONSUMER_AC_RESIZE                    0x245
#define UX_HOST_CLASS_HID_CONSUMER_AC_FLIP_HORIZONTAL           0x246
#define UX_HOST_CLASS_HID_CONSUMER_AC_FLIP_VERTICAL             0x247
#define UX_HOST_CLASS_HID_CONSUMER_AC_MIRROR_HORIZONTAL         0x248
#define UX_HOST_CLASS_HID_CONSUMER_AC_MIRROR_VERTICAL           0x249
#define UX_HOST_CLASS_HID_CONSUMER_AC_FONT_SELECT               0x24A
#define UX_HOST_CLASS_HID_CONSUMER_AC_FONT_COLOR                0x24B
#define UX_HOST_CLASS_HID_CONSUMER_AC_FONT_SIZE                 0x24C
#define UX_HOST_CLASS_HID_CONSUMER_AC_JUSTIFY_LEFT              0x24D
#define UX_HOST_CLASS_HID_CONSUMER_AC_JUSTIFY_CENTER_H          0x24E
#define UX_HOST_CLASS_HID_CONSUMER_AC_JUSTIFY_RIGHT             0x24F
#define UX_HOST_CLASS_HID_CONSUMER_AC_JUSTIFY_BLOCK_H           0x250
#define UX_HOST_CLASS_HID_CONSUMER_AC_JUSTIFY_TOP               0x251
#define UX_HOST_CLASS_HID_CONSUMER_AC_JUSTIFY_CENTER_V          0x252
#define UX_HOST_CLASS_HID_CONSUMER_AC_JUSTIFY_BOTTOM            0x253
#define UX_HOST_CLASS_HID_CONSUMER_AC_JUSTIFY_BLOCK_V           0x254
#define UX_HOST_CLASS_HID_CONSUMER_AC_INDENT_DECREASE           0x255
#define UX_HOST_CLASS_HID_CONSUMER_AC_INDENT_INCREASE           0x256
#define UX_HOST_CLASS_HID_CONSUMER_AC_NUMBERED_LIST             0x257
#define UX_HOST_CLASS_HID_CONSUMER_AC_RESTART_NUMBERING         0x258
#define UX_HOST_CLASS_HID_CONSUMER_AC_BULLETED_LIST             0x259
#define UX_HOST_CLASS_HID_CONSUMER_AC_PROMOTE                   0x25A
#define UX_HOST_CLASS_HID_CONSUMER_AC_DEMOTE                    0x25B
#define UX_HOST_CLASS_HID_CONSUMER_AC_YES                       0x25C
#define UX_HOST_CLASS_HID_CONSUMER_AC_NO                        0x25D
#define UX_HOST_CLASS_HID_CONSUMER_AC_CANCEL                    0x25E
#define UX_HOST_CLASS_HID_CONSUMER_AC_CATALOG                   0x25F
#define UX_HOST_CLASS_HID_CONSUMER_AC_BUY_CHECKOUT              0x260
#define UX_HOST_CLASS_HID_CONSUMER_AC_ADD_TO_CART               0x261
#define UX_HOST_CLASS_HID_CONSUMER_AC_EXPAND                    0x262
#define UX_HOST_CLASS_HID_CONSUMER_AC_EXPAND_ALL                0x263
#define UX_HOST_CLASS_HID_CONSUMER_AC_COLLAPSE                  0x264
#define UX_HOST_CLASS_HID_CONSUMER_AC_COLLAPSE_ALL              0x265
#define UX_HOST_CLASS_HID_CONSUMER_AC_PRINT_PREVIEW             0x266
#define UX_HOST_CLASS_HID_CONSUMER_AC_PASTE_SPECIAL             0x267
#define UX_HOST_CLASS_HID_CONSUMER_AC_INSERT_MODE               0x268
#define UX_HOST_CLASS_HID_CONSUMER_AC_DELETE                    0x269
#define UX_HOST_CLASS_HID_CONSUMER_AC_LOCK                      0x26A
#define UX_HOST_CLASS_HID_CONSUMER_AC_UNLOCK                    0x26B
#define UX_HOST_CLASS_HID_CONSUMER_AC_PROTECT                   0x26C
#define UX_HOST_CLASS_HID_CONSUMER_AC_UNPROTECT                 0x26D
#define UX_HOST_CLASS_HID_CONSUMER_AC_ATTACH_COMMENT            0x26E
#define UX_HOST_CLASS_HID_CONSUMER_AC_DELETE_COMMENT            0x26F
#define UX_HOST_CLASS_HID_CONSUMER_AC_VIEW_COMMENT              0x270
#define UX_HOST_CLASS_HID_CONSUMER_AC_SELECT_WORD               0x271
#define UX_HOST_CLASS_HID_CONSUMER_AC_SELECT_SENTENCE           0x272
#define UX_HOST_CLASS_HID_CONSUMER_AC_SELECT_PARAGRAPH          0x273
#define UX_HOST_CLASS_HID_CONSUMER_AC_SELECT_COLUMN             0x274
#define UX_HOST_CLASS_HID_CONSUMER_AC_SELECT_ROW                0x275
#define UX_HOST_CLASS_HID_CONSUMER_AC_SELECT_TABLE              0x276
#define UX_HOST_CLASS_HID_CONSUMER_AC_SELECT_OBJECT             0x277
#define UX_HOST_CLASS_HID_CONSUMER_AC_REDO_REPEAT               0x278
#define UX_HOST_CLASS_HID_CONSUMER_AC_SORT                      0x279
#define UX_HOST_CLASS_HID_CONSUMER_AC_SORT_ASCENDING            0x27A
#define UX_HOST_CLASS_HID_CONSUMER_AC_SORT_DESCENDING           0x27B
#define UX_HOST_CLASS_HID_CONSUMER_AC_FILTER                    0x27C
#define UX_HOST_CLASS_HID_CONSUMER_AC_SET_CLOCK                 0x27D
#define UX_HOST_CLASS_HID_CONSUMER_AC_VIEW_CLOCK                0x27E
#define UX_HOST_CLASS_HID_CONSUMER_AC_SELECT_TIME_ZONE          0x27F
#define UX_HOST_CLASS_HID_CONSUMER_AC_EDIT_TIME_ZONES           0x280
#define UX_HOST_CLASS_HID_CONSUMER_AC_SET_ALARM                 0x281
#define UX_HOST_CLASS_HID_CONSUMER_AC_CLEAR_ALARM               0x282
#define UX_HOST_CLASS_HID_CONSUMER_AC_SNOOZE_ALARM              0x283
#define UX_HOST_CLASS_HID_CONSUMER_AC_RESET_ALARM               0x284
#define UX_HOST_CLASS_HID_CONSUMER_AC_SYNCHRONIZE               0x285
#define UX_HOST_CLASS_HID_CONSUMER_AC_SEND_RECEIVE              0x286
#define UX_HOST_CLASS_HID_CONSUMER_AC_SEND_TO                   0x287
#define UX_HOST_CLASS_HID_CONSUMER_AC_REPLY                     0x288
#define UX_HOST_CLASS_HID_CONSUMER_AC_REPLY_ALL                 0x289
#define UX_HOST_CLASS_HID_CONSUMER_AC_FORWARD_MSG               0x28A
#define UX_HOST_CLASS_HID_CONSUMER_AC_SEND                      0x28B
#define UX_HOST_CLASS_HID_CONSUMER_AC_ATTACH_FILE               0x28C
#define UX_HOST_CLASS_HID_CONSUMER_AC_UPLOAD                    0x28D
#define UX_HOST_CLASS_HID_CONSUMER_AC_DOWNLOAD                  0x28E
#define UX_HOST_CLASS_HID_CONSUMER_AC_SET_BORDERS               0x28F
#define UX_HOST_CLASS_HID_CONSUMER_AC_INSERT_ROW                0x290
#define UX_HOST_CLASS_HID_CONSUMER_AC_INSERT_COLUMN             0x291
#define UX_HOST_CLASS_HID_CONSUMER_AC_INSERT_FILE               0x292
#define UX_HOST_CLASS_HID_CONSUMER_AC_INSERT_PICTURE            0x293
#define UX_HOST_CLASS_HID_CONSUMER_AC_INSERT_OBJECT             0x294
#define UX_HOST_CLASS_HID_CONSUMER_AC_INSERT_SYMBOL             0x295
#define UX_HOST_CLASS_HID_CONSUMER_AC_SAVE_CLOSE                0x296
#define UX_HOST_CLASS_HID_CONSUMER_AC_RENAME                    0x297
#define UX_HOST_CLASS_HID_CONSUMER_AC_MERGE                     0x298
#define UX_HOST_CLASS_HID_CONSUMER_AC_SPLIT                     0x299
#define UX_HOST_CLASS_HID_CONSUMER_AC_DISRIBUTE_HORIZONTALLY    0x29A
#define UX_HOST_CLASS_HID_CONSUMER_AC_DISTRIBUTE_VERTICALLY     0x29B

/* Define HID Report Types.  */

#define UX_HOST_CLASS_HID_REPORT_TYPE_INPUT                     0x1
#define UX_HOST_CLASS_HID_REPORT_TYPE_OUTPUT                    0x2
#define UX_HOST_CLASS_HID_REPORT_TYPE_FEATURE                   0x3

/* Define HID Class report callback structure.  */

#ifndef UX_HOST_CLASS_HID_USAGES
#define UX_HOST_CLASS_HID_USAGES                                1024
#endif

#define UX_HOST_CLASS_HID_MAX_GLOBAL                            4
#define UX_HOST_CLASS_HID_MAX_COLLECTION                        4

/* Define HID flags.  */
#define UX_HOST_CLASS_HID_FLAG_LOCK                             1ul
#define UX_HOST_CLASS_HID_FLAG_PROTECT                          2ul

#ifndef UX_HOST_CLASS_HID_REPORT_TRANSFER_TIMEOUT
#define UX_HOST_CLASS_HID_REPORT_TRANSFER_TIMEOUT               10000
#endif

/* Define HID Class descriptor.  */

typedef struct UX_HID_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bcdHID;
    ULONG           bCountryCode;
    ULONG           bNumDescriptor;
    ULONG           bReportDescriptorType;
    ULONG           wItemLength;
} UX_HID_DESCRIPTOR;


typedef struct UX_HOST_CLASS_HID_REPORT_CALLBACK_STRUCT
{

    struct UX_HOST_CLASS_HID_CLIENT_STRUCT        
                    *ux_host_class_hid_report_callback_client;
    ULONG           ux_host_class_hid_report_callback_id;
    ULONG           ux_host_class_hid_report_callback_status;
    ULONG           ux_host_class_hid_report_callback_flags;
    ULONG           ux_host_class_hid_report_callback_value;
    ULONG           ux_host_class_hid_report_callback_usage;
    ULONG           ux_host_class_hid_report_callback_length;
    ULONG           ux_host_class_hid_report_callback_actual_length;
    VOID            *ux_host_class_hid_report_callback_buffer;
    VOID            (*ux_host_class_hid_report_callback_function) (struct UX_HOST_CLASS_HID_REPORT_CALLBACK_STRUCT *);
} UX_HOST_CLASS_HID_REPORT_CALLBACK;



typedef struct UX_HOST_CLASS_HID_REPORT_GET_ID_STRUCT
{

    ULONG           ux_host_class_hid_report_get_id;
    ULONG           ux_host_class_hid_report_get_type;
    struct UX_HOST_CLASS_HID_REPORT_STRUCT           
                    *ux_host_class_hid_report_get_report;
} UX_HOST_CLASS_HID_REPORT_GET_ID;


/* Define HID Class local item structure.  */

typedef struct UX_HOST_CLASS_HID_LOCAL_ITEM_STRUCT
{

    ULONG           ux_host_class_hid_local_item_usages[UX_HOST_CLASS_HID_USAGES];
    ULONG           ux_host_class_hid_local_item_number_usage;
    ULONG           ux_host_class_hid_local_item_usage_min;
    ULONG           ux_host_class_hid_local_item_usage_max;
    ULONG           ux_host_class_hid_local_item_delimiter_level;
    ULONG           ux_host_class_hid_local_item_delimiter_branch;
} UX_HOST_CLASS_HID_LOCAL_ITEM;


/* Define HID Class global item structure.  */

typedef struct UX_HOST_CLASS_HID_GLOBAL_ITEM_STRUCT
{

    ULONG           ux_host_class_hid_global_item_usage_page;
    SLONG           ux_host_class_hid_global_item_logical_min;
    SLONG           ux_host_class_hid_global_item_logical_max;
    SLONG           ux_host_class_hid_global_item_physical_min;
    SLONG           ux_host_class_hid_global_item_physical_max;
    ULONG           ux_host_class_hid_global_item_unit_expo;
    ULONG           ux_host_class_hid_global_item_unit;
    ULONG           ux_host_class_hid_global_item_report_size;
    ULONG           ux_host_class_hid_global_item_report_id;
    ULONG           ux_host_class_hid_global_item_report_count;
} UX_HOST_CLASS_HID_GLOBAL_ITEM;


/* Define HID Class field structure.  */

typedef struct UX_HOST_CLASS_HID_FIELD_STRUCT
{

    ULONG           ux_host_class_hid_field_physical;
    ULONG           ux_host_class_hid_field_logical;
    ULONG           ux_host_class_hid_field_application;
    ULONG           ux_host_class_hid_field_usage_page;
    ULONG           ux_host_class_hid_field_usage_min;
    ULONG           ux_host_class_hid_field_usage_max;
    SLONG           ux_host_class_hid_field_logical_min;
    SLONG           ux_host_class_hid_field_logical_max;
    SLONG           ux_host_class_hid_field_physical_min;
    SLONG           ux_host_class_hid_field_physical_max;
    ULONG           ux_host_class_hid_field_unit;
    ULONG           ux_host_class_hid_field_unit_expo;
    ULONG           ux_host_class_hid_field_report_type;
    ULONG           ux_host_class_hid_field_report_id;
    ULONG           ux_host_class_hid_field_report_offset;
    ULONG           ux_host_class_hid_field_report_size;
    ULONG           ux_host_class_hid_field_report_count;
    ULONG           ux_host_class_hid_field_value;
    ULONG           *ux_host_class_hid_field_usages;
    ULONG           ux_host_class_hid_field_number_usage;
    ULONG           *ux_host_class_hid_field_values;
    ULONG           ux_host_class_hid_field_number_values;
    struct UX_HOST_CLASS_HID_REPORT_STRUCT 
                    *ux_host_class_hid_field_report;
    struct UX_HOST_CLASS_HID_FIELD_STRUCT 
                    *ux_host_class_hid_field_next_field;
} UX_HOST_CLASS_HID_FIELD;


/* Define HID Class report structure.  */

typedef struct UX_HOST_CLASS_HID_REPORT_STRUCT
{

    ULONG           ux_host_class_hid_report_id;
    ULONG           ux_host_class_hid_report_type;
    struct UX_HOST_CLASS_HID_FIELD_STRUCT 
                    *ux_host_class_hid_report_field;
    ULONG           ux_host_class_hid_report_number_item;
    ULONG           ux_host_class_hid_report_byte_length;
    ULONG           ux_host_class_hid_report_bit_length;
    ULONG           ux_host_class_hid_report_callback_flags;
    VOID            *ux_host_class_hid_report_callback_buffer;
    ULONG           ux_host_class_hid_report_callback_length;
    VOID            (*ux_host_class_hid_report_callback_function) (struct UX_HOST_CLASS_HID_REPORT_CALLBACK_STRUCT *);
    struct UX_HOST_CLASS_HID_REPORT_STRUCT 
                    *ux_host_class_hid_report_next_report;
} UX_HOST_CLASS_HID_REPORT;


/* Define HID main parser structure.  */

typedef struct UX_HOST_CLASS_HID_PARSER_STRUCT
{

    UX_HOST_CLASS_HID_GLOBAL_ITEM
                    ux_host_class_hid_parser_global;
    UX_HOST_CLASS_HID_GLOBAL_ITEM                
                    ux_host_class_hid_parser_global_pool[UX_HOST_CLASS_HID_MAX_GLOBAL];
    ULONG           ux_host_class_hid_parser_number_global;
    UX_HOST_CLASS_HID_LOCAL_ITEM                 
                    ux_host_class_hid_parser_local;
    ULONG           ux_host_class_hid_parser_application;
    ULONG           ux_host_class_hid_parser_collection[UX_HOST_CLASS_HID_MAX_COLLECTION];
    ULONG           ux_host_class_hid_parser_number_collection;
    ULONG           ux_host_class_hid_parser_main_page;
    ULONG           ux_host_class_hid_parser_main_usage;
    UX_HOST_CLASS_HID_REPORT                     
                    *ux_host_class_hid_parser_input_report;
    UX_HOST_CLASS_HID_REPORT                     
                    *ux_host_class_hid_parser_output_report;
    UX_HOST_CLASS_HID_REPORT                     
                    *ux_host_class_hid_parser_feature_report;
} UX_HOST_CLASS_HID_PARSER;


/* Define HID Class item analysis structure.  */

typedef struct UX_HOST_CLASS_HID_ITEM_STRUCT
{

    UCHAR           ux_host_class_hid_item_report_type;
    UCHAR           ux_host_class_hid_item_report_tag;
    USHORT          ux_host_class_hid_item_report_length;
    USHORT          ux_host_class_hid_item_report_format;
}  UX_HOST_CLASS_HID_ITEM;


/* Define HID Class instance structure.  */

typedef struct UX_HOST_CLASS_HID_STRUCT
{

    struct UX_HOST_CLASS_HID_STRUCT              
                    *ux_host_class_hid_next_instance;
    UX_HOST_CLASS   *ux_host_class_hid_class;
    UX_DEVICE       *ux_host_class_hid_device;
    UX_ENDPOINT     *ux_host_class_hid_interrupt_endpoint;
#if defined(UX_HOST_CLASS_HID_INTERRUPT_OUT_SUPPORT)
    UX_ENDPOINT     *ux_host_class_hid_interrupt_out_endpoint;
#endif
    UINT            ux_host_class_hid_interrupt_endpoint_status;
    UX_INTERFACE    *ux_host_class_hid_interface;
    ULONG           ux_host_class_hid_state;
    struct UX_HID_DESCRIPTOR_STRUCT         
                    ux_host_class_hid_descriptor;
    UX_HOST_CLASS_HID_PARSER                     
                    ux_host_class_hid_parser;
    struct UX_HOST_CLASS_HID_CLIENT_STRUCT       
                    *ux_host_class_hid_client;
#if !defined(UX_HOST_STANDALONE)
    UX_SEMAPHORE    ux_host_class_hid_semaphore;
#else
    ULONG           ux_host_class_hid_flags;
    UCHAR           *ux_host_class_hid_allocated;
    UINT            ux_host_class_hid_status;
    UCHAR           ux_host_class_hid_enum_state;
    UCHAR           ux_host_class_hid_next_state;
    UCHAR           ux_host_class_hid_cmd_state;
    UCHAR           reserved[1];
#endif
} UX_HOST_CLASS_HID;

#if defined(UX_HOST_STANDALONE)
#define _ux_host_class_hid_lock_fail_return(hid)                                \
    UX_DISABLE                                                                  \
    if (hid -> ux_host_class_hid_flags & UX_HOST_CLASS_HID_FLAG_LOCK)           \
    {                                                                           \
        UX_RESTORE                                                              \
        status = UX_BUSY;                                                       \
        return(status);                                                         \
    }                                                                           \
    hid -> ux_host_class_hid_flags |= UX_HOST_CLASS_HID_FLAG_LOCK;              \
    UX_RESTORE
#define _ux_host_class_hid_unlock(hid) do {                                     \
        hid -> ux_host_class_hid_flags &= ~UX_HOST_CLASS_HID_FLAG_LOCK;         \
    } while(0)
#else
#define _ux_host_class_hid_lock_fail_return(hid)                                \
    status =  _ux_host_semaphore_get(&hid -> ux_host_class_hid_semaphore,       \
                                    UX_WAIT_FOREVER);                           \
    if (status != UX_SUCCESS)                                                   \
        return(status);
#define _ux_host_class_hid_unlock(hid)                                          \
    _ux_host_semaphore_put(&hid -> ux_host_class_hid_semaphore);
#endif


/* Define HID Class client command format structure.  */

typedef struct UX_HOST_CLASS_HID_CLIENT_COMMAND_STRUCT
{

    UINT            ux_host_class_hid_client_command_request;
    VOID            *ux_host_class_hid_client_command_container;
    UX_HOST_CLASS_HID    *ux_host_class_hid_client_command_instance;
    ULONG           ux_host_class_hid_client_command_page;
    ULONG           ux_host_class_hid_client_command_usage;
} UX_HOST_CLASS_HID_CLIENT_COMMAND;


/* Define HID Class report command structure.  */

typedef struct UX_HOST_CLASS_HID_CLIENT_REPORT_STRUCT
{

    UX_HOST_CLASS_HID_REPORT
                    *ux_host_class_hid_client_report;
    ULONG           *ux_host_class_hid_client_report_buffer;
    ULONG           ux_host_class_hid_client_report_length; 
    ULONG           ux_host_class_hid_client_report_actual_length; 
    UINT            ux_host_class_hid_client_report_flags;
} UX_HOST_CLASS_HID_CLIENT_REPORT;


/* Define HID Class client structure.  */

typedef struct UX_HOST_CLASS_HID_CLIENT_STRUCT
{

    ULONG           ux_host_class_hid_client_status;
#if defined(UX_NAME_REFERENCED_BY_POINTER)
    UCHAR           *ux_host_class_hid_client_name;
#else
    UCHAR           ux_host_class_hid_client_name[UX_HOST_CLASS_HID_MAX_CLIENT_NAME_LENGTH + 1]; /* "+1" for string null-terminator */
#endif
    UINT            (*ux_host_class_hid_client_handler) (struct UX_HOST_CLASS_HID_CLIENT_COMMAND_STRUCT *);
    VOID            *ux_host_class_hid_client_local_instance;  
#if defined(UX_HOST_STANDALONE)
    VOID            (*ux_host_class_hid_client_function)(struct UX_HOST_CLASS_HID_CLIENT_STRUCT *);
#endif
} UX_HOST_CLASS_HID_CLIENT;

/* Define HID Class function prototypes.  */

UINT    _ux_host_class_hid_activate(UX_HOST_CLASS_COMMAND  *command);
UINT    _ux_host_class_hid_client_register(UCHAR *hid_client_name,
                                UINT (*hid_client_handler)(struct UX_HOST_CLASS_HID_CLIENT_COMMAND_STRUCT *));
UINT    _ux_host_class_hid_client_search(UX_HOST_CLASS_HID *hid);
UINT    _ux_host_class_hid_configure(UX_HOST_CLASS_HID *hid);
UINT    _ux_host_class_hid_deactivate(UX_HOST_CLASS_COMMAND *command);
UINT    _ux_host_class_hid_descriptor_parse(UX_HOST_CLASS_HID *hid);
UINT    _ux_host_class_hid_entry(UX_HOST_CLASS_COMMAND *command);
UINT    _ux_host_class_hid_field_decompress(UX_HOST_CLASS_HID_FIELD *hid_field, UCHAR *report_buffer, UX_HOST_CLASS_HID_CLIENT_REPORT *client_report);
UINT    _ux_host_class_hid_global_item_parse(UX_HOST_CLASS_HID *hid, UX_HOST_CLASS_HID_ITEM *item, UCHAR *descriptor);
UINT    _ux_host_class_hid_idle_get(UX_HOST_CLASS_HID *hid, USHORT *idle_time, USHORT report_id);
UINT    _ux_host_class_hid_idle_set(UX_HOST_CLASS_HID *hid, USHORT idle_time, USHORT report_id);
UINT    _ux_host_class_hid_instance_clean(UX_HOST_CLASS_HID *hid);
UINT    _ux_host_class_hid_interrupt_endpoint_search(UX_HOST_CLASS_HID *hid);
ULONG   _ux_host_class_hid_item_data_get(UCHAR *descriptor, UX_HOST_CLASS_HID_ITEM *item);
UINT    _ux_host_class_hid_local_item_parse(UX_HOST_CLASS_HID *hid, UX_HOST_CLASS_HID_ITEM *item, UCHAR *descriptor);
UINT    _ux_host_class_hid_main_item_parse(UX_HOST_CLASS_HID *hid, UX_HOST_CLASS_HID_ITEM *item, UCHAR *descriptor);
UINT    _ux_host_class_hid_periodic_report_start(UX_HOST_CLASS_HID *hid);
UINT    _ux_host_class_hid_periodic_report_stop(UX_HOST_CLASS_HID *hid);
UINT    _ux_host_class_hid_report_add(UX_HOST_CLASS_HID *hid, UCHAR *descriptor, UX_HOST_CLASS_HID_ITEM *item);
UINT    _ux_host_class_hid_report_callback_register(UX_HOST_CLASS_HID *hid, UX_HOST_CLASS_HID_REPORT_CALLBACK *call_back);
UINT    _ux_host_class_hid_report_compress(UX_HOST_CLASS_HID *hid, UX_HOST_CLASS_HID_CLIENT_REPORT *client_report,
                                                    UCHAR *report_buffer, ULONG report_length);
UINT    _ux_host_class_hid_report_decompress(UX_HOST_CLASS_HID *hid, UX_HOST_CLASS_HID_CLIENT_REPORT *client_report,
                                                    UCHAR  *report_buffer, ULONG report_length);
UINT    _ux_host_class_hid_report_descriptor_get(UX_HOST_CLASS_HID *hid, ULONG length);
UINT    _ux_host_class_hid_report_get(UX_HOST_CLASS_HID *hid, UX_HOST_CLASS_HID_CLIENT_REPORT *client_report);
UINT    _ux_host_class_hid_report_id_get(UX_HOST_CLASS_HID *hid, UX_HOST_CLASS_HID_REPORT_GET_ID *report_id);
UINT    _ux_host_class_hid_report_item_analyse(UCHAR *descriptor, UX_HOST_CLASS_HID_ITEM *item);
UINT    _ux_host_class_hid_report_set(UX_HOST_CLASS_HID *hid, UX_HOST_CLASS_HID_CLIENT_REPORT *client_report);
UINT    _ux_host_class_hid_resources_free(UX_HOST_CLASS_HID *hid);
VOID    _ux_host_class_hid_transfer_request_completed(UX_TRANSFER *transfer_request);

UINT    _ux_host_class_hid_tasks_run(UX_HOST_CLASS *hid_class);
UINT    _ux_host_class_hid_idle_set_run(UX_HOST_CLASS_HID *hid, USHORT idle_time, USHORT report_id);
UINT    _ux_host_class_hid_report_set_run(UX_HOST_CLASS_HID *hid, UX_HOST_CLASS_HID_CLIENT_REPORT *client_report);

/* Define HID Class API prototypes.  */

#define ux_host_class_hid_client_register                   _ux_host_class_hid_client_register
#define ux_host_class_hid_client_search                     _ux_host_class_hid_client_search
#define ux_host_class_hid_descriptor_parse                  _ux_host_class_hid_descriptor_parse
#define ux_host_class_hid_entry                             _ux_host_class_hid_entry
#define ux_host_class_hid_idle_get                          _ux_host_class_hid_idle_get
#define ux_host_class_hid_idle_set                          _ux_host_class_hid_idle_set
#define ux_host_class_hid_periodic_report_start             _ux_host_class_hid_periodic_report_start
#define ux_host_class_hid_periodic_report_stop              _ux_host_class_hid_periodic_report_stop
#define ux_host_class_hid_report_add                        _ux_host_class_hid_report_add
#define ux_host_class_hid_report_callback_register          _ux_host_class_hid_report_callback_register
#define ux_host_class_hid_report_descriptor_get             _ux_host_class_hid_report_descriptor_get
#define ux_host_class_hid_report_get                        _ux_host_class_hid_report_get
#define ux_host_class_hid_report_id_get                     _ux_host_class_hid_report_id_get
#define ux_host_class_hid_report_set                        _ux_host_class_hid_report_set

/* Determine if a C++ compiler is being used.  If so, complete the standard 
   C conditional started above.  */   
#ifdef __cplusplus
} 
#endif 

#endif

