# MSP Extensions

INAV includes a number of extensions to the MultiWii Serial Protocol (MSP). This document describes 
those extensions in order that 3rd party tools may identify INAV firmware and react appropriately.

Issue the MSP_API_VERSION command to find out if the firmware supports them.

## Mode Ranges

### MSP\_MODE\_RANGES

The MSP\_MODE\_RANGES returns the current auxiliary mode settings from the flight controller. It should be invoked
before any modification is made to the configuration.

The message returns a group of 4 unsigned bytes for each 'slot' available in the flight controller. The number of
slots should be calculated from the size of the returned message.

| Command | Msg Id | Direction | Notes |
|---------|--------|-----------|-------|
| MSP\_MODE\_RANGES | 34 | to FC | Following this command, the FC returns a block of 4 bytes for each auxiliary mode 'slot'|

Unassigned slots have rangeStartStep == rangeEndStep. Each element contains the following fields.

| Data | Type | Notes |
|------|------|-------|
| permanentId | uint8 | See Modes.md for a definition of the permanent ids |
| auxChannelIndex | uint8 | The Aux switch number (indexed from 0) |
| rangeStartStep | uint8 | The start value for this element in 'blocks' of 25  where 0 == 900 and 48 == 2100 |
| rangeEndStep | uint8 | The end value for this element in 'blocks' of 25 where 0 == 900 and 48 == 2100 |

Thus, for a INAV firmware with 40 slots 160 bytes would be returned in response to MSP\_MODE\_RANGES,

### MSP\_SET\_MODE\_RANGE

The MSP\_SET\_MODE\_RANGE is used to inform the flight controller of
auxiliary mode settings. The client *must* return all auxiliary
elements, including those that have been disabled or are undefined, by
sending this message for all auxiliary slots.

| Command | Msg Id | Direction |
|---------|--------|-----------|
| MSP\_SET\_MODE\_RANGE | 35 | to FC |


| Data | Type | Notes |
|------|------|-------|
| sequence id | uint8 | A monotonically increasing ID, from 0 to the number of slots -1 |
| permanentId | uint8 | See Modes.md for a definition of the permanent ids |
| auxChannelIndex | uint8 | The Aux channel number (indexed from 0) |
| rangeStartStep | uint8 | The start value for this element in 'blocks' of 25  where 0 == 900 and 48 == 2100 |
| rangeEndStep | uint8 | The end value for this element in 'blocks' of 25 where 0 == 900 and 48 == 2100 |

### Implementation Notes

* The client should make no assumptions about the number of slots available. Rather, the number should be computed
  from the size of the MSP\_MODE\_RANGES message divided by the size of the returned data element (4 bytes);
* The client should ensure that all changed items are returned to the flight controller, including those where a
  switch or range has been disabled;
* A 'null' return, with all values other than the sequence id set to 0, must be made for all unused slots, up to
  the maximum number of slots calculated from the initial message.

## Adjustment Ranges

### MSP\_ADJUSTMENT\_RANGES

The MSP\_ADJUSTMENT\_RANGES returns the current adjustment range settings from
the flight controller. It should be invoked before any modification is
made to the configuration.

The message returns a group of 6 unsigned bytes for each 'slot'
available in the flight controller. The number of slots should be
calculated from the size of the returned message.

| Command | Msg Id | Direction | Notes |
|---------|--------|-----------|-------|
| MSP\_ADJUSTMENT\_RANGES | 52 | to FC | Following this command, the FC returns a block of 6 bytes for each adjustment range 'slot'|

Unassigned slots have rangeStartStep == rangeEndStep. Each element contains the following fields.

| Data | Type | Notes |
|------|------|-------|
| adjustmentStateIndex | uint8 | See below |
| auxChannelIndex | uint8 | The Aux channel number (indexed from 0) used to activate the adjustment |
| rangeStartStep | uint8 | The start value for this element in 'blocks' of 25  where 0 == 900 and 48 == 2100 |
| rangeEndStep | uint8 | The end value for this element in 'blocks' of 25 where 0 == 900 and 48 == 2100 |
| adjustmentFunction | uint8 | See below |
| auxSwitchChannelIndex | uint8 | The Aux channel number used to perform the function (indexed from 0) |

Thus, for a INAV firmware with 12 slots 72 bytes would be returned in response to MSP\_ADJUSTMENT\_RANGES,

### MSP\_SET\_ADJUSTMENT\_RANGE

The MSP\_SET\_ADJUSTMENT\_RANGE is used to inform the flight controller of
adjustment range settings. The client *must* return all adjustment range
elements, including those that have been disabled or are undefined, by
sending this message for all adjustment range slots.

| Command | Msg Id | Direction |
|---------|--------|-----------|
| MSP\_SET\_ADJUSTMENT\_RANGE | 53 | to FC |


| Data | Type | Notes |
|------|------|-------|
| sequence id | uint8 | A monotonically increasing ID, from 0 to the number of slots -1 |
| adjustmentStateIndex | uint8 | See below |
| auxChannelIndex | uint8 | The Aux channel number (indexed from 0) |
| rangeStartStep | uint8 | The start value for this element in 'blocks' of 25  where 0 == 900 and 48 == 2100 |
| rangeEndStep | uint8 | The end value for this element in 'blocks' of 25 where 0 == 900 and 48 == 2100 |
| adjustmentFunction | uint8 | See below |
| auxSwitchChannelIndex | uint8 | The Aux channel number used to perform the function (indexed from 0) |

### MSP\_SET\_1WIRE

The MSP\_SET\_1WIRE is used to enable serial1wire passthrough
note: it would be ideal to disable this when armed

| Command | Msg Id | Direction |
|---------|--------|-----------|
| MSP\_SET\_1WIRE  | 243 | to FC |

| Data | Type | Notes |
|------|------|-------|
| esc id | uint8 | A monotonically increasing ID, from 0 to the number of escs -1 |

#### AdjustmentIndex

The FC maintains internal state for each adjustmentStateIndex, currently 4 simultaneous adjustment states are maintained.  Multiple adjustment ranges
can be configured to use the same state but care should be taken not to send multiple adjustment ranges that when active would confict.

e.g.  Configuring two identical adjustment ranges using the same slot would conflict, but configuring two adjustment ranges that used
only one half of the possible channel range each but used the same adjustmentStateIndex would not conflict.

The FC does NOT check for conflicts.

#### AdjustmentFunction

There are many adjustments that can be made, the numbers of them and their use is found in the documentation of the cli `adjrange` command in the 'Inflight Adjustents' section.

### Implementation Notes

* The client should make no assumptions about the number of slots available. Rather, the number should be computed
  from the size of the MSP\_ADJUSTMENT\_RANGES message divided by the size of the returned data element (6 bytes);
* The client should ensure that all changed items are returned to the flight controller, including those where a
  switch or range has been disabled;
* A 'null' return, with all values except for the sequence id set to 0, must be made for all unused slots,
  up to the maximum number of slots calculated from the initial message.

## Deprecated MSP

The following MSP commands are replaced by the MSP\_MODE\_RANGES and
MSP\_SET\_MODE\_RANGE extensions, and are not recognised by
INAV.

* MSP\_BOX
* MSP\_SET\_BOX

See also
--------
Modes.md describes the user visible implementation for the INAV
modes extension.
