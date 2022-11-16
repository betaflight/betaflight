# DisplayPort MSP Extensions

Betaflight supports what is sometimes referred to as Canvas Mode whereby the OSD can send arbitrary character strings to be displayed at given screen coordinates.

## DisplayPort MSP commands

### MSP\_SET\_OSD\_CANVAS

The MSP\_SET\_OSD\_CANVAS command is sent by the VTX, or display device, to the FC to indicate the size of the canvas available to the DisplayPort rendering when in HD mode. HD mode, as is indicated by `vcd_video_system = HD` is automatically set on reception of this command.

| Command | Msg Id | Direction | Notes |
|---------|--------|-----------|-------|
| MSP\_SET\_OSD\_CANVAS | 188 | to FC | Sets the canvas size|

| Data | Type | Notes |
|------|------|-------|
| canvas_cols | uint8 | The number of columns |
| canvas_rows | uint8 | The number of rows |

### MSP\_OSD\_CANVAS

The MSP\_OSD\_CANVAS command is sent by the configurator to the FC to determine the size of the canvas available to the DisplayPort rendering when in HD mode. This is then used on the OSD tab to show the correct number of rows/columns when editing the OSD element positions.

| Command | Msg Id | Direction | Notes |
|---------|--------|-----------|-------|
| MSP\_OSD\_CANVAS | 189 | to FC | Gets the canvas size|

Response is two bytes.

| Data | Type | Notes |
|------|------|-------|
| canvas_cols | uint8 | The number of columns |
| canvas_rows | uint8 | The number of rows |

### MSP\_DISPLAYPORT

The MSP\_DISPLAYPORT command is sent by the FC to the display device/VTX to perform a DisplayPort operation.

| Command | Msg Id | Direction | Notes |
|---------|--------|-----------|-------|
| MSP\_DISPLAYPORT | 182 | from FC | DisplayPort specific commands follow |

One of the following sub-commands will then follow.

## DisplayPort sub-commands


#### MSP\_DP\_HEARTBEAT
| Command | Msg Id | Notes |
|---------|--------|-------|
| MSP\_DP\_HEARTBEAT | 0 | Prevent OSD Slave boards from displaying a 'disconnected' status |
 
#### MSP\_DP\_RELEASE
| Command | Msg Id | Notes |
|---------|--------|-------|
| MSP\_DP\_RELEASE | 1 | Clears the display and allows local rendering on the display device based on telemetry informtation etc. |

#### MSP\_DP\_CLEAR\_SCREEN
| Command | Msg Id | Notes |
|---------|--------|-------|
| MSP\_DP\_CLEAR\_SCREEN | 2 | Clear the display |

#### MSP\_DP\_WRITE\_STRING
| Command | Msg Id | Notes |
|---------|--------|-------|
| MSP\_DP\_WRITE\_STRING | 3 | Write a string |

| Data | Type | Notes |
|------|------|-------|
| row | uint8 | Row on which to position the first character of the string |
| column | uint8 | Column on which to position the first character of the string |
| attribute | uint8 | Byte indicating the font to use and if the text should flash |
| string | uint8 x n | NULL terminated string of up to 30 characters in length |

The `attribute` parameter is encoded thus.

| Field | Bits | Comment |
| ----- | ---- | ------- |
| Version | 7 | Must be 0 |
| DISPLAYPORT\_MSP\_ATTR\_BLINK | 6 | Set to have the display device automatically blink the string |
| Reserved | 2 - 5 | Must be 0 |
| Font number | 0 - 1 | Selects one of four fonts, each of 256 8 bit characters |

#### MSP\_DP\_DRAW\_SCREEN
| Command | Msg Id | Notes |
|---------|--------|-------|
| MSP\_DP\_DRAW\_SCREEN | 4 | Triggers the display of a frame after it has been cleared/rendered |

#### MSP\_DP\_OPTIONS
| Command | Msg Id | Notes |
|---------|--------|-------|
| MSP\_DP\_OPTIONS | 5 | Not used by Betaflight. Used by INAV and Ardupilot to set display resolution. |

#### MSP\_DP\_SYS
| Command | Msg Id | Notes |
|---------|--------|-------|
| MSP\_DP\_SYS | 6 | Display system element displayportSystemElement_e at given coordinates |

| Data | Type | Notes |
|------|------|-------|
| row | uint8 | Row on which to position the first character of the string |
| column | uint8 | Column on which to position the first character of the string |
| system_element | uint8 | System element to be rendered by the VTX/goggle/display device |

`system_element` will be one of the following as defined by `displayPortSystemElement_e `. Once one MSP\_DP\_SYS sub-command has been received by the VTX/goggle/display device then the default system elements should no longer be displayed in their default locations, but only explicitly as directed by this command. In this way, the default behaviour is as before, but as soon as any system element is explicitly positioned these OSD elements behave just like any other and can be called up is specific locations by any given OSD profile.

```
// System elements rendered by VTX or Goggles
typedef enum {
    DISPLAYPORT_SYS_GOGGLE_VOLTAGE = 0,
    DISPLAYPORT_SYS_VTX_VOLTAGE = 1,
    DISPLAYPORT_SYS_BITRATE = 2,
    DISPLAYPORT_SYS_DELAY = 3,
    DISPLAYPORT_SYS_DISTANCE = 4,
    DISPLAYPORT_SYS_LQ = 5,
    DISPLAYPORT_SYS_GOGGLE_DVR = 6,
    DISPLAYPORT_SYS_VTX_DVR = 7,
    DISPLAYPORT_SYS_WARNINGS = 8,
    DISPLAYPORT_SYS_VTX_TEMP = 9,
    DISPLAYPORT_SYS_FAN_SPEED = 10,
    DISPLAYPORT_SYS_COUNT,
} displayPortSystemElement_e;
```
