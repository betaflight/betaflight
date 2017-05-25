# Blackbox logging internals

The Blackbox is designed to record the raw internal state of the flight controller at near-maximum rate. By logging the
raw inputs and outputs of key flight systems, the Blackbox log aims to allow the offline bench-top simulation, debugging,
and testing of flight control algorithms using data collected from real flights.

A typical logging regime might capture 30 different state variables (for an average of 28 bytes per frame) at a sample
rate of 900Hz. That's about 25,000 bytes per second, which is 250,000 baud with a typical 8-N-1 serial encoding.

## References

Please refer to the source code to clarify anything this document leaves unclear:

* INAV's Blackbox logger: [blackbox.c](https://github.com/iNavFlight/inav/blob/master/src/main/blackbox/blackbox.c),
[blackbox_io.c](https://github.com/iNavFlight/inav/blob/master/src/main/blackbox/blackbox_io.c),
[blackbox_fielddefs.h](https://github.com/iNavFlight/inav/blob/master/src/main/blackbox/blackbox_fielddefs.h)
* [C implementation of the Blackbox log decoder](https://github.com/cleanflight/blackbox-tools/blob/master/src/parser.c)
* [JavaScript implementation of the Blackbox log decoder](https://github.com/cleanflight/blackbox-log-viewer/blob/master/js/flightlog_parser.js)

## Logging cycle
Blackbox is designed for flight controllers that are based around the concept of a "main loop". During each main loop
iteration, the flight controller will read some state from sensors, run some flight control algorithms, and produce some
outputs. For each of these loop iterations, a Blackbox "logging iteration" will be executed. This will read data that
was stored during the execution of the main loop and log this data to an attached logging device. The data will include
algorithm inputs such as sensor and RC data, intermediate results from flight control algorithms, and algorithm outputs
such as motor commands.

## Log frame types
Each event which is recorded to the log is packaged as a "log frame". Blackbox only uses a handful of different types of
log frames. Each frame type is identified by a single capital letter.

### Main frames: I, P
The most basic kind of logged frames are the "main frames". These record the primary state of the flight controller (RC
input, gyroscopes, flight control algorithm intermediates, motor outputs), and are logged during every logging
iteration.

Each main frame must contain at least two fields, "loopIteration" which records the index of the current main loop
iteration (starting at zero for the first logged iteration), and "time" which records the timestamp of the beginning of
the main loop in microseconds (this needn't start at zero, on INAV it represents the system uptime).

There are two kinds of main frames, "I" and "P". "I", or "intra" frames are like video keyframes. They can be decoded
without reference to any previous frame, so they allow log decoding to be resynchronized in the event of log damage. "P"
or "inter" frames use an encoding which references previously logged frames in order to reduce the required datarate.
When one interframe fails to decode, all following interframes will be undecodable up until the next intraframe.

### GPS frames: G, H
Because the GPS is updated so infrequently, GPS data is logged in its own dedicated frames. These are recorded whenever
the GPS data changes (not necessarily alongside every main frame). Like the main frames, the GPS frames have their own
intra/inter encoding system.

The "H" or "home" frame records the lat/lon of a reference point. The "G" or "GPS" frame records the current state of
the GPS system (current position, altitude etc.) based on the reference point. The reference point can be updated
(infrequently) during the flight, and is logged whenever it changes.

To allow "G" frames to continue be decoded in the event that an "H" update is dropped from the log, the "H" frame is
logged periodically even if it has not changed (say, every 10 seconds). This caps the duration of unreadble "G" frames
that will result from a single missed "H" change.

### Slow frames: S
Some flight controller state is updated very infrequently (on the order of once or twice a minute). Logging the fact
that this data had not been updated during every single logging iteration would be a waste of bandwidth, so these frames
are only logged when the "slow" state actually changes.

All Slow frames are logged as intraframes. An interframe encoding scheme can't be used for Slow frames, because one
damaged frame causes all subsequent interframes to be undecodable. Because Slow frames are written so infrequently, one
missing Slow frame could invalidate minutes worth of Slow state.

On INAV, Slow frames are currently used to log data like the user-chosen flight mode and the current failsafe
state.

### Event frames: E
Some flight controller data is updated so infrequently or exists so transiently that we do not log it as a flight
controller "state". Instead, we log it as a state *transition* . This data is logged in "E" or "event" frames. Each event
frame payload begins with a single byte "event type" field. The format of the rest of the payload is not encoded in the
flight log, so its interpretation is left up to an agreement of the writer and the decoder.

For example, one event that INAV logs is that the user has adjusted a system setting (such as a PID setting)
using INAV's inflight adjustments feature. The event payload notes which setting was adjusted and the new value
for the setting.

Because these setting updates are so rare, it would be wasteful to treat the settings as "state" and log the fact that
the setting had not been changed during every logging iteration. It would be infeasible to periodically log the system
settings using an intra/interframe scheme, because the intraframes would be so large. Instead we only log the
transitions as events, accept the small probability that any one of those events will be damaged/absent in the log, and
leave it up to log readers to decide the extent to which they are willing to assume that the state of the setting
between successfully-decoded transition events was truly unchanged.

## Log field format
For every field in a given frame type, there is an associated name, predictor, and encoding.

When a field is written, the chosen predictor is computed for the field, then this predictor value is subtracted from
the raw field value. Finally, the encoder is used to transform the value into bytes to be written to the logging device.

### Field predictors
The job of the predictor is to bring the value to be encoded as close to zero as possible. The predictor may be based
on the values seen for the field in a previous frame, or some other value such as a fixed value or a value recorded in
the log headers. For example, the battery voltage values in "I" intraframes in INAV use a reference voltage that
is logged as part of the headers as a predictor. This assumes that battery voltages will be broadly similar to the
initial pack voltage of the flight (e.g. 4S battery voltages are likely to lie within a small range for the whole
flight). In "P" interframes, the battery voltage will instead use the previously-logged voltage as a predictor, because
the correlation between successive voltage readings is high.

These predictors are presently available:

#### Predict zero (0)
This predictor is the null-predictor which doesn't modify the field value at all. It is a common choice for fields
which are already close to zero, or where no better history is available (e.g. in intraframes which may not rely on the
previous value of fields).

#### Predict last value (1)
This is the most common predictor in interframes. The last-logged value of the field will be used as the predictor, and
subtracted from the raw field value. For fields which don't change very often, this will make their encoded value be
normally zero. Most fields have some time-correlation, so this predictor should reduce the magnitude of all but the
noisiest fields.

#### Predict straight line (2)
This predictor assumes that the slope between the current measurement and the previous one will be similar to the
slope between the previous measurement and the one before that. This is common for fields which increase at a steady rate,
such as the "time" field. The predictor is `history_age_2 - 2 * history_age_1`.

#### Predict average 2 (3)
This predictor is the average of the two previously logged values of the field (i.e. `(history_age_1 + history_age_2) / 2`
). It is used when there is significant random noise involved in the field, which means that the average of the recent
history is a better predictor of the next value than the previous value on its own would be (for example, in gyroscope
or motor measurements).

#### Predict minthrottle (4)
This predictor subtracts the value of "minthrottle" which is included in the log header. In INAV, motors always
lie in the range of `[minthrottle ... maxthrottle]` when the craft is armed, so this predictor is used for the first
motor value in intraframes.

#### Predict motor[0] (5)
This predictor is set to the value of `motor[0]` which was decoded earlier within the current frame. It is used in
intraframes for every motor after the first one, because the motor commands typically lie in a tight grouping.

#### Predict increment (6)
This predictor assumes that the field will be incremented by 1 unit for every main loop iteration. This is used to
predict the `loopIteration` field, which increases by 1 for every loop iteration.

#### Predict home-coord (7)
This predictor is set to the corresponding latitude or longitude field from the GPS home coordinate (which was logged in
a preceding "H" frame). If no preceding "H" frame exists, the value is marked as invalid.

#### Predict 1500 (8)
This predictor is set to a fixed value of 1500. It is preferred for logging servo values in intraframes, since these
typically lie close to the midpoint of 1500us.

#### Predict vbatref (9)
This predictor is set to the "vbatref" field written in the log header. It is used when logging intraframe battery
voltages in INAV, since these are expected to be broadly similar to the first battery voltage seen during
arming.

#### Predict last main-frame time (10)
This predictor is set to the last logged `time` field from the main frame. This is used when predicting timestamps of
non-main frames (e.g. that might be logging the timing of an event that happened during the main loop cycle, like a GPS
reading).

### Field encoders
The field encoder's job is to use fewer bits to represent values which are closer to zero than for values that are
further from zero. Blackbox supports a range of different encoders, which should be chosen on a per-field basis in order
to minimize the encoded data size. The choice of best encoder is based on the probability distribution of the values
which are to be encoded. For example, if a field is almost always zero, then an encoding should be chosen for it which
can encode that case into a very small number of bits, such as one. Conversely, if a field is normally 8-16 bits large,
it would be wasteful to use an encoder which provided a special short encoded representation for zero values, because
this will increase the encoded length of larger values.

These encoders are presently available:

#### Unsigned variable byte (1)
This is the most straightforward encoding. This encoding uses the lower 7 bits of an encoded byte to store the lower 7
bits of the field's value. The high bit of that encoded byte is set to one if more than 7 bits are required to store the
value. If the value did exceed 7 bits, the lower 7 bits of the value (which were written to the log) are removed from
the value (by right shift), and the encoding process begins again with the new value.

This can be represented by the following algorithm:

```c
while (value > 127) {
    writeByte((uint8_t) (value | 0x80)); // Set the high bit to mean "more bytes follow"
    value >>= 7;
}
writeByte(value);
```

Here are some example values encoded using variable-byte encoding:

| Input value | Output encoding |
| ----------- | --------------- |
| 1           | 0x01            |
| 42          | 0x2A            |
| 127         | 0x7F            |
| 128         | 0x80 0x01       |
| 129         | 0x81 0x01       |
| 23456       | 0xA0 0xB7 0x01  |

#### Signed variable byte (0)
This encoding applies a pre-processing step to fold negative values into positive ones, then the resulting unsigned
number is encoded using unsigned variable byte encoding. The folding is accomplished by "ZigZag" encoding, which is
represented by:

```c
unsigned32 = (signed32 << 1) ^ (signed32 >> 31)
```

ZigZag encoding is preferred against simply casting the signed integer to unsigned, because casting would cause small
negative quantities to appear to be very large unsigned integers, causing the encoded length to be similarly large.
ZigZag encoding ensures that values near zero are still near zero after encoding.

Here are some example integers encoded using ZigZag encoding:

| Input value | ZigZag encoding |
| ----------- | --------------- |
| 0           | 0               |
| -1          | 1               |
| 1           | 2               |
| -2          | 3               |
| 2147483647  | 4294967294      |
| -2147483648 | 4294967295      |

#### Neg 14-bit (3)
The value is negated, treated as an unsigned 14 bit integer, then encoded using unsigned variable byte encoding. This
bizarre encoding is used in INAV for battery pack voltages. This is because battery voltages are measured using a
14-bit ADC, with a predictor which is set to the battery voltage during arming, which is expected to be higher than any
voltage experienced during flight. After the predictor is subtracted, the battery voltage will almost certainly be below
zero.

This results in small encoded values when the voltage is closely below the initial one, at the expense of very large
encoded values if the voltage rises higher than the initial one.

#### Elias delta unsigned 32-bit (4)
Because this encoding produces a bitstream, this is the only encoding for which the encoded value might not be a whole
number of bytes. If the bitstream isn't aligned on a byte boundary by the time the next non-Elias Delta field arrives,
or the end of the frame is reached, the final byte is padded with zeros byte-align the stream. This encoding requires
more CPU time than the other encodings because of the bit juggling involved in writing the bitstream.

When this encoder is chosen to encode all of the values in INAV interframes, it saves about 10% bandwidth
compared to using a mixture of the other encodings, but uses too much CPU time to be practical.

[The basic encoding algorithm is defined on Wikipedia](https://en.wikipedia.org/wiki/Elias_delta_coding). Given these
utility functions:

```c
/* Write `bitCount` bits from the least-significant end of the `bits` integer to the bitstream. The most-significant bit
 * will be written first
 */
void writeBits(uint32_t bits, unsigned int bitCount);

/* Returns the number of bits needed to hold the top-most 1-bit of the integer 'i'. 'i' must not be zero. */
unsigned int numBitsToStoreInteger(uint32_t i);
```

This is our reference implementation of Elias Delta:

```c
// Value must be more than zero
void writeU32EliasDeltaInternal(uint32_t value)
{
    unsigned int valueLen, lengthOfValueLen;

    valueLen = numBitsToStoreInteger(value);
    lengthOfValueLen = numBitsToStoreInteger(valueLen);

    // Use unary to encode the number of bits we'll need to write the length of the value
    writeBits(0, lengthOfValueLen - 1);

    // Now write the length of the value
    writeBits(valueLen, lengthOfValueLen);

    // Having now encoded the position of the top bit of value, write its remaining bits
    writeBits(value, valueLen - 1);
}
```

To this we add a wrapper which allows encoding both the value zero and MAXINT:

```c
void writeU32EliasDelta(uint32_t value)
{
    /* We can't encode value==0, so we need to add 1 to the value before encoding
     *
     * That would make it impossible to encode MAXINT, so use 0xFFFFFFFF as an escape
     * code with an additional bit to choose between MAXINT-1 or MAXINT.
     */
    if (value >= 0xFFFFFFFE) {
        // Write the escape code
        writeU32EliasDeltaInternal(0xFFFFFFFF);
        // Add a one bit after the escape code if we wanted "MAXINT", or a zero if we wanted "MAXINT - 1"
        writeBits(value - 0xFFFFFFFE, 1);
    } else {
        writeU32EliasDeltaInternal(value + 1);
    }
}
```

Here are some reference encoded bit patterns produced by writeU32EliasDelta:

| Input value | Encoded bit string |
| ----------- | ------------------ |
|           0 | 1                  |
|           1 | 0100               |
|           2 | 0101               |
|           3 | 01100              |
|           4 | 01101              |
|           5 | 01110              |
|           6 | 01111              |
|           7 | 00100000           |
|           8 | 00100001           |
|           9 | 00100010           |
|          10 | 00100011           |
|          11 | 00100100           |
|          12 | 00100101           |
|          13 | 00100110           |
|          14 | 00100111           |
|          15 | 001010000          |
|         225 | 00010001100010     |
|  4294967292 | 000001000001111111111111111111111111111101  |
|  4294967293 | 000001000001111111111111111111111111111110  |
|  4294967294 | 0000010000011111111111111111111111111111110 |
|  4294967295 | 0000010000011111111111111111111111111111111 |

Note that the very common value of zero encodes to a single bit, medium-sized values like 225 encode to 14 bits (an
overhead of 4 bits over writing a plain 8 bit value), and typical 32-bit values like 4294967293 encode into 42 bits, an
overhead of 10 bits.

#### Elias delta signed 32-bit (5)
The value is first converted to unsigned using ZigZag encoding, then unsigned Elias-delta encoding is applied.

#### TAG8_8SVB (6)
First, an 8-bit (one byte) header is written. This header has its bits set to zero when the corresponding field (from a
maximum of 8 fields) is set to zero, otherwise the bit is set to one. The least-signficant bit in the header corresponds
to the first field to be written. This header is followed by the values of only the fields which are non-zero, written
using signed variable byte encoding.

This encoding is preferred for groups of fields in interframes which are infrequently updated by the flight controller.
This will mean that their predictions are usually perfect, and so the value to be encoded for each field will normally
be zero. This is common for values like RC inputs and barometer readings, which are updated in only a fraction of main
loop iterations.

For example, given these field values to encode:

```
0, 0, 4, 0, 8
```

This would be encoded:

```
0b00010100, 0x04, 0x08
```

#### TAG2_3S32 (7)
A 2-bit header is written, followed by 3 signed field values of up to 32 bits each. The header value is based on the
maximum size in bits of the three values to be encoded as follows:

| Header value | Maximum field value size | Field range                |
| ------------ | ------------------------ | -------------------------- |
| 0            | 2 bits                   | [-2...1]                   |
| 1            | 4 bits                   | [-8...7]                   |
| 2            | 6 bits                   | [-32...31]                 |
| 3            | Up to 32 bits            | [-2147483648...2147483647] |

If any of the three values requires more than 6 bits to encode, a second, 6-bit header value is written in the lower
bits of the initial header byte. This second header has 2 bits for each of the encoded values which represents how many
bytes are required to encode that value. The least-significant bits of the header represent the first field which is
encoded. The values for each possibility are as follows:

| Header value | Field size | Field range                |
| ------------ | ---------- | -------------------------- |
| 0            | 1 byte     | [-127...128]               |
| 1            | 2 bytes    | [-32768...32767]           |
| 2            | 3 bytes    | [-8388608...8388607]       |
| 3            | 4 bytes    | [-2147483648...2147483647] |

This header is followed by the actual field values in order, written least-significant byte first, using the byte
lengths specified in the header.

So bringing it all together, these encoded bit patterns are possible, where "0" and "1" mean bits fixed to be those
values, "A", "B", and "C" represent the first, second and third fields, and "s" represents the bits of the secondary
header in the case that any field is larger than 6 bits:

```
00AA BBCC,
0100 AAAA BBBB CCCC
10AA AAAA 00BB BBBB 00CC CCCC
11ss ssss (followed by fields of byte lengths specified in the "s" header)
```

This encoding is useful for fields like 3-axis gyroscopes, which are frequently small and typically have similar
magnitudes.

#### TAG8_4S16 (8)
An 8-bit header is written, followed by 4 signed field values of up to 16 bits each. The 8-bit header value has 2 bits
for each of the encoded fields (the least-significant bits represent the first field) which represent the
number of bits required to encode that field as follows:

| Header value | Field value size | Field range      |
| ------------ | ---------------- | ---------------- |
| 0            | 0 bits           | [0...0]          |
| 1            | 4 bits           | [-8...7]         |
| 2            | 8 bits           | [-128...127]     |
| 3            | 16 bits          | [-32768...32767] |

This header is followed by the actual field values in order, written as if the output stream was a bit-stream, with the
most-significant bit of the first field ending up in the most-significant bits of the first written byte. If the number
of nibbles written is odd, the final byte has its least-significant nibble set to zero.

For example, given these field values:

```
13, 0, 4, 2
```

Choosing from the allowable field value sizes, they may be encoded using this many bits each:

```
8, 0, 4, 4
```

The corresponding header values for these lengths would be:

```
2, 0, 1, 1
```

So the header and fields would be encoded together as:

```
0b01010010, 0x0D, 0x42
```

#### NULL (9)
This encoding does not write any bytes to the file. It is used when the predictor will always perfectly predict the
value of the field, so the remainder is always zero. In practice this is only used for the "loopIteration" field in
interframes, which is always perfectly predictable based on the logged frame's position in the sequence of frames and
the "P interval" setting from the header.

## Log file structure
A logging session begins with a log start marker, then a header section which describes the format of the log, then the
log payload data, and finally an optional "log end" event ("E" frame).

A single log file can be comprised of one or more logging sessions. Each session may be preceded and followed by any
amount of non-Blackbox data. This data is ignored by the Blackbox log decoding tools. This allows for the logging device
to be alternately used by the Blackbox and some other system (such as MSP) without requiring the ability to begin a
separate log file for each separate activity.

### Log start marker
The log start marker is "H Product:Blackbox flight data recorder by Nicholas Sherlock\n". This marker is
used to discover the beginning of the flight log if the log begins partway through a file. Because it is such a long
string, it is not expected to occur by accident in any sequence of random bytes from other log device users.

### Log header
The header is comprised of a sequence of lines of plain ASCII text. Each header line has the format `H fieldname:value`
and ends with a '\n'. The overall header does not have a terminator to separate it from the log payload
(the header implicitly ends when a line does not begin with an 'H' character).

The header can contain some of these fields:

#### Data version (required)
When the interpretation of the Blackbox header changes due to Blackbox specification updates, the log version is
incremented to allow backwards-compatibility in the decoder:

```
H Data version:2
```

#### Logging interval
Not every main loop iteration needs to result in a Blackbox logging iteration. When a loop iteration is not logged,
Blackbox is not called, no state is read from the flight controller, and nothing is written to the log. Two header lines
are included to note which main loop iterations will be logged:

##### I interval
This header notes which of the main loop iterations will record an "I" intraframe to the log. If main loop iterations
with indexes divisible by 32 will be logged as "I" frames, the header will be:

```
H I interval: 32
```

The first main loop iteration seen by Blackbox will be numbered with index 0, so the first main loop iteration will
always be logged as an intraframe.

##### P interval
Not every "P" interframe needs to be logged. Blackbox will log a portion of iterations in order to bring the total
portion of logged main frames to a user-chosen fraction. This fraction is called the logging rate. The smallest possible
logging rate is `(1/I interval)` which corresponds to logging only "I" frames at the "I" interval and discarding all
other loop iterations. The maximum logging rate is `1/1`, where every main loop iteration that is not an "I" frame is
logged as a "P" frame. The header records the logging rate fraction in `numerator/denominator` format like so:

```
H P interval:1/2
```

The logging fraction given by `num/denom` should be simplified (i.e. rather than 2/6, a logging rate of 1/3 should
be used).

Given a logging rate of `num/denom` and an I-frame interval of `I_INTERVAL`, the frame type to log for an iteration
of index `iteration` is given by:

```c
if (iteration % I_INTERVAL == 0)
	return 'I';

if ((iteration % I_INTERVAL + num - 1) % denom < num)
	return 'P';

return '.'; // i.e. don't log this iteration
```

For an I-interval of 32, these are the resulting logging patterns at some different P logging rates.

| Logging rate | Main frame pattern                                                | Actual portion logged |
| ------------ | ----------------------------------------------------------------- | --------------------- |
| 1 / 32       | I...............................I...............................I | 0.03                  |
| 1 / 6        | I.....P.....P.....P.....P.....P.I.....P.....P.....P.....P.....P.I | 0.19                  |
| 1 / 3        | I..P..P..P..P..P..P..P..P..P..P.I..P..P..P..P..P..P..P..P..P..P.I | 0.34                  |
| 1 / 2        | I.P.P.P.P.P.P.P.P.P.P.P.P.P.P.P.I.P.P.P.P.P.P.P.P.P.P.P.P.P.P.P.I | 0.50                  |
| 2 / 3        | I.PP.PP.PP.PP.PP.PP.PP.PP.PP.PP.I.PP.PP.PP.PP.PP.PP.PP.PP.PP.PP.I | 0.66                  |
| 5 / 6        | I.PPPPP.PPPPP.PPPPP.PPPPP.PPPPP.I.PPPPP.PPPPP.PPPPP.PPPPP.PPPPP.I | 0.81                  |
| 1 / 1        | IPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPIPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPI | 1.00                  |


#### Firmware type (optional)
Because Blackbox records the internal flight controller state, the interpretation of the logged data will depend
on knowing which flight controller recorded it. To accomodate this, the name of the flight controller should be recorded:

```
H Firmware type:INAV
```

More details should be included to help narrow down the precise flight-controller version (but these are not required):

```
H Firmware revision:c49bd40
H Firmware date:Aug 28 2015 16:49:11
```

#### Field X name (required)
This header is a comma-separated list of the names for the fields in the 'X' frame type:

```
H Field I name:loopIteration,time,axisP[0],axisP[1]...
```

The decoder assumes that the fields in the 'P' frame type will all have the same names as those in the 'I' frame, so
a "Field P name" header does not need to be supplied.

#### Field X signed (optional)
This is a comma-separated list of integers which are set to '1' when their corresponding field's value should be
interpreted as signed after decoding, or '0' otherwise:

```
H Field I signed:0,0,1,1...
```

#### Field X predictor (required)
This is a comma-separated list of integers which specify the predictors for each field in the specified frame type:

```
H Field I predictor:0,0,0,0...
```

#### Field X encoding (required)
This is a comma-separated list of integers which specify the encoding used for each field in the specified frame type:

```
H Field X encoding:1,1,0,0...
```

#### vbatref
This header provides the reference voltage that will be used by predictor #9.

#### minthrottle
This header provides the minimum value sent by INAV to the ESCs when armed, it is used by predictor #4.

#### Additional headers
The decoder ignores headers that it does not understand, so you can freely add any headers that you require in order to
properly interpret the meaning of the logged values.

For example, to create a graphical displays of RC sticks and motor percentages, the Blackbox rendering tool requires
the additional headers "rcRate" and "maxthrottle". In order to convert raw gyroscope, accelerometer and voltage readings
into real-world units, the Blackbox decoder requires the calibration constants "gyro.scale", "acc_1G" and "vbatscale".
These headers might look like:

```
H rcRate:100
H maxthrottle:1980
H gyro.scale:0x3d79c190
H acc_1G:4096
H vbatscale:110
```

### Log payload

The log payload is a concatenated sequence of logged frames. Each frame type which is present in the log payload must
have been previously described in the log header (with Frame X name, etc. headers). Each frame begins with a single
capital letter to specify the type of frame (I, P, etc), which is immediately followed by the frame's field data. There
is no frame length field, checksum, or trailer.

The field data is encoded by taking an array of raw field data, computing the predictor for each field, subtrating this
predictor from the field, then applying the field encoders to each field in sequence to serialize them to the log.

For example, imagine that we are encoding three fields in an intraframe, are using zero-predictors for each field (#0),
and are encoding the values using the unsigned variable byte encoding (#1). For these field values:

```
1, 2, 3
```

We would encode a frame:

```
'I', 0x01, 0x02, 0x03
```

Imagine that we are encoding an array of motor commands in an interframe. We will use the previous motor commands as a
predictor, and encode the resulting values using signed variable byte encoding. The motor command values seen in the
previous logged iteration were:

```
1430, 1500, 1470, 1490
```

And the motor commands to be logged in this iteration are:

```
1635, 1501, 1469, 1532
```

After subtracting the predictors for each field, we will be left with:

```
205, 1, -1, 42
```

We will apply ZigZag encoding to each field, which will give us:

```
410, 2, 1, 84
```

We will use unsigned variable byte encoding to write the resulting values to the log, which will give us:

```
'P', 0x9A, 0x03, 0x02, 0x01, 0x54
```

### Log end marker
The log end marker is an optional Event ("E") frame of type 0xFF whose payload is the string "End of log\0". The
payload ensures that random data does not look like an end-of-log marker by chance. This event signals the tidy ending
of the log. All following bytes until the next log-begin marker (or end of file) should be ignored by the log
decoder.

```
'E', 0xFF, "End of log ", 0x00
```

## Log validation
Any damage experienced to the log during recording is overwhelmingly due to subsequences of bytes being dropped by the
logging device due to overflowing buffers. Accordingly, Blackbox logs do not bother to include any checksums (bytes are
not expected to be damaged by the logging device without changing the length of the message). Because of the tight
bandwidth requirements of logging, neither a frame length field nor frame trailer is recorded that would allow for the
detection of missing bytes.

Instead, the decoder uses a heuristic in order to detect damaged frames. The decoder reads an entire frame from the log
(using the decoder for each field which is the counterpart of the encoder specified in the header), then it checks to
see if the byte immediately following the frame, which should be the beginning of a next frame, is a recognized
frame-type byte (e.g. 'I', 'P', 'E', etc). If that following byte represents a valid frame type, it is assumed that the
decoded frame was the correct length (so was unlikely to have had random ranges of bytes removed from it, which would
have likely altered the frame length). Otherwise, the frame is rejected, and a valid frame-type byte is looked for
immediately after the frame-start byte of the frame that was rejected. A rejected frame causes all subsequent
interframes to be rejected as well, up until the next intraframe.

A frame is also rejected if the "loopIteration" or "time" fields have made unreasonable leaps forward, or moved at
all backwards. This suffices to detect almost all log corruption.
