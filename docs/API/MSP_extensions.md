# MSP Extensions

Cleanflight includes a number of extensions to the MultiWii Serial
Protocol (MSP). This document describes those extensions in order that
3rd part tools may identify cleanflight firmware and react appropriately.

## MSP_IDENT capabilities

Cleanflight sets a number of non-standard bits in the MSP_IDENT
message in order to identify the firmware. For completeness, this
includes flags defined for baseflight.

|Value|Mnemonic|Notes|Reference|
|-----|--------|-----|---------|
| 1 << 29 | CAP\_CLEANFLIGHT\_CONFIG  | The firmware supports cleanflight configuration options. This is the definitive test for cleanflight firmware | src/main/io/serial_msp.c|
| 1 << 30 | CAP\_BASEFLIGHT\_CONFIG | The firmware supports baseflight configuration options. See the baseflight documentation for details. |  src/main/io/serial_msp.c |
| 1 << 31 | CAP\_PLATFORM\_32BIT |  This is a 32bit platform. |  src/main/io/serial_msp.c |

## Modes Extension

The cleanflight modes extension to MSP supports the extended
auxiliary modes introduced 2014-10-14. The CAP\_CLEANFLIGHT\_CONFIG
capabilities flag should be used to confirm the availability of
extended auxiliary modes. If this flag is not present, the legacy MSP
MSP\_BOX and MSP\_SET\_BOX messages should be used.

Note: In the following descriptions, the format of the original MSP
documentation is followed.

### MSP\_MODE\_RANGES

The MSP\_MODE\_RANGES returns the current auxiliary mode settings from
the flight controller. It should be invoked before any modification is
made to the configuration.

The message returns a group of 4 unsigned bytes for each 'slot'
available in the flight controller. The number of slots should be
calculated from the size of the returned message.

| Command | Msg Id | Direction | Data | Type | Notes |
|---------|--------|-----------|------|------|-------|
| MSP\_MODE\_RANGES | 34 | to FC | (none) | N/A | Following this command, the FC returns a block of 4 bytes for each auxiliary mode 'slot'|

Unassigned slots have rangeStartStep == rangeEndStep. Each element contains the following fields.

| Command | Msg Id | Direction | Data | Type | Notes |
|---------|--------|-----------|------|------|-------|
|  |  | returned from FC | permanentId | uint8 | See Modes.md for a definition of the permanent ids |
|  |  | returned from FC | auxChannelIndex | uint8 | The Aux switch number (indexed from 0) |
|  |  | returned from FC | rangeStartStep | uint8 | The start value for this element in 'blocks' of 25  where 0 == 900 and 48 == 2100 |
|  |  | returned from FC | rangeEndStep | uint8 | The end value for this element in 'blocks' of 25 where 0 == 900 and 48 == 2100 |

Thus, for a cleanflight firmware with 40 slots for auxiliary settings,
160 bytes would be returned in response to MSP\_MODE\_RANGES,

### MSP\_SET\_MODE\_RANGE

The MSP\_SET\_MODE\_RANGE is used to inform the flight controller of
auxiliary mode settings. The client *must* return all auxiliary
elements, including those that have been disabled or are undefined, by
sending this message for all auxiliary slots.

| Command | Msg Id | Direction | Data | Type | Notes |
|---------|--------|-----------|------|------|-------|
| MSP\_SET\_MODE\_RANGE | 35 | to FC | sequence id | uint8 | A monotonically increasing ID, from 0 to the number of slots -1 |
|  |  |  | permanentId | uint8 | See Modes.md for a definition of the permanent ids |
|  |  |  | auxChannelIndex | uint8 | The Aux switch number (indexed from 0) |
|  |  |  | rangeStartStep | uint8 | The start value for this element in 'blocks' of 25  where 0 == 900 and 48 == 2100 |
|  |  |  | rangeEndStep | uint8 | The end value for this element in 'blocks' of 25 where 0 == 900 and 48 == 2100 |

## Implementation Notes

* The client should make no assumptions about the number of slots
  available. Rather, the number should be computed from the size of
  the MSP\_MODE\_RANGES message divided by the size of the returned
  data element (4 bytes);
* The client should ensure that all changed items are returned to the
  flight controller, including those where a switch or range has been
  disabled;
* A 'null' return (with rangeStartStep == rangeEndStep) must be made for all
  unused slots, up to the maximum number of slots calculated from the initial
  message.

## Deprecated MSP

The following MSP commands are replaced by the MSP\_MODE\_RANGES and
MSP\_SET\_MODE\_RANGE extensions, and are not recognised by
cleanflight where CAP\_CLEANFLIGHT\_CONFIG is set.

* MSP\_BOX
* MSP\_SET\_BOX

See also
--------
Modes.md describes the user visible implementation for the cleanflight
modes extension.
