# Bus and External Device Drivers
Betaflight makes a distinction between external devices and the bus on which they reside. For example each type of gyro will have a device driver which understands the register map of the gyro, and accesses to those registers will be made via a bus driver, either I2C or SPI. A device instance is represented by a `extDevice_t` structure which references a `busDevice_t` structure corresponding to the bus instance via which the device is accesses.

## Bus Agnostic Device Access Routines
There are a common set of device access functions which are bus agnostic. In each case the device instance handle `dev` is passed to indicate the device to access, and from this the appropriate bus instance is selected.

### Access routines where the register is accessed directly
These write routines do not mask the value in `reg`.

```
bool busRawWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data);
```
Write the value `data` to the register offset `reg`.

```
bool busRawWriteRegisterStart(const extDevice_t *dev, uint8_t reg, uint8_t data);
```
Write the value `data` to the register offset `reg`. If the device is on an I2C bus this call is non-blocking and merely starts the access, hence the name suffix, but care should be taken not to call this a second time before the first access has completed.

```
bool busRawReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);
```
Read `length` bytes into the buffer at `*data` from the register offset `reg`.

```
bool busRawReadRegisterBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);
```
Read `length` bytes into the buffer at `*data` from the register offset `reg`. If the device is on an I2C bus this call is non-blocking and merely starts the access, hence the name suffix.

### Write routines where the register number is masked with `0x7f`
It is common to indicate a read from an SPI register by setting but 7 (`0x80`) of the register number. There are therefore a number of routines which clear this bit to indicate a write. I2C register addresses are only 7 bits with an explicit read/write bit.

```
bool busWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data);
```
Write the value `data` to the register offset `reg` logically anded with `0x7f`.

```
bool busWriteRegisterStart(const extDevice_t *dev, uint8_t reg, uint8_t data);
```
Write the value `data` to the register offset `reg` logically anded with `0x7f`. If the device is on an I2C bus this call is non-blocking and merely starts the access, hence the name suffix.

### Read routines where the register is ORed with `0x80`
It is common to indicate a read from an SPI register by setting but 7 (`0x80`) of the register number. There are therefore a number of routines which set this bit to indicate a read. I2C register addresses are only 7 bits with an explicit read/write bit.

```
uint8_t busReadRegister(const extDevice_t *dev, uint8_t reg);
```
Read a single byte from the register offset `reg` logically anded with `0x80`.

```
bool busReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);
```
Read `length` bytes into the buffer at `*data` from the register offset `reg` logically anded with `0x80`.

```
bool busReadRegisterBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);
```
Read `length` bytes into the buffer at `*data` from the register offset `reg` logically anded with `0x80`. If the device is on an I2C bus this call is non-blocking and merely starts the access, hence the name suffix.

## I2C Specific Access
I2C bus accesses are slow and therefore, except during startup device initialisation, the use of blocking accesses should be avoided. Therefore the `bus...Start()` routines should be used which use interrupts to handle the transfer in the background. The barometer driver is a good example of how I2C devices accesses should be performed, with a state machine used so that accesses are started in one state, and the processing of the result of a read, or launching the next write waits until the next state.

It is necessary to register a device as being on an I2C bus in order that it can be accessed.

```
bool i2cBusSetInstance(const extDevice_t *dev, uint32_t device);
```

This registers the external device `dev` with an I2C bus device instance `device`.

```
void i2cBusDeviceRegister(const extDevice_t *dev);
```
A call to `i2cBusDeviceRegister` simply increments a count of the number of I2C devices in use.


## SPI Specific Access
SPI attached devices are accessed at higher speed and therefore may be accessed using blocking read/writes, however longer transfers or accesses requiring multiple transfers are better performed using DMA transfers under interrupt control.

There are a number of SPI specific bus access routines to facilitate such optimisation.

As with I2C it is possible to have multiple devices share a common SPI bus.

It is necessary to register a device as being on an SPI bus in order that it can be accessed.

```
bool spiSetBusInstance(extDevice_t *dev, uint32_t device);
```

This registers the external device `dev` with an SPI bus device instance `device`.

```
void spiSetClkDivisor(const extDevice_t *dev, uint16_t divider);
```
Each device on an SPI bus can use a different SPI bus clock speed and this sets the clock divisor to be used for accesses by the given device.

Two utility routines are provided to determine the `divider` value to use in order to achieve a max SPI clock speed, and to return the actual clock speed corresponding to that divisor.

```
// Determine the divisor to use for a given bus frequency
uint16_t spiCalculateDivider(uint32_t freq);
// Return the SPI clock based on the given divisor
uint32_t spiCalculateClock(uint16_t spiClkDivisor);
```

Access to SPI devices requires that the clock phase/polarity be set appropriately. See [https://en.wikipedia.org/wiki/Serial_Peripheral_Interface](https://en.wikipedia.org/wiki/Serial_Peripheral_Interface).

```
void spiSetClkPhasePolarity(const extDevice_t *dev, bool leadingEdge);
```
If `leadingEdge` is set to true, the default, then data will be clocked on the first rising edge of the clock, or if false, on the second falling edge.

```
void spiDmaEnable(const extDevice_t *dev, bool enable);
```
Certain devices, such as the CC2500 cannot handle the timing of sequential accesses which are being DMAed, so this enables DMA to be enabled (default) or disabled on a per device basis.


In order to support efficient use of SPI it is possible to perform not only single accesses as described above, but also to define a sequence of transfers using an array of `busSegment_t` elements which then comprise a complete transaction. These may be complex, support polling bus status for example before performing a write.

Each `busSegment_t` element passes a union of either a pair of buffer pointers for write/read respectively, a null link structure used to terminate the list. Following is the number of bytes in the transfer, a boolean, `negateCS`, indicating if the SPI CS line should be negated at the end of the segment, and an optional callback routine.

A good example of this is in `m25p16_readBytes()` where a segment list is defined thus:

```
    busSegment_t segments[] = {
            {.u.buffers = {readStatus, readyStatus}, sizeof(readStatus), true, m25p16_callbackReady},
            {.u.buffers = {readBytes, NULL}, fdevice->isLargeFlash ? 5 : 4, false, NULL},
            {.u.buffers = {NULL, buffer}, length, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
    };
```

In the above example the busy status of the FLASH memory is polled in the first element and then `m25p16_callbackReady()` is called which checks the read status. If the device is busy the value `BUS_BUSY` is returned and the element will be repeated under interrupt/DMA control. If the device is ready to accept a new command then `BUS_READY` is returned and the next element is processed. `BUS_ABORT` may also be returned in abort the whole transaction, although this is not currently used.

It can be faster to perform short transfers using polled access rather than setting up DMAs and the rules are as follows to determine if DMA should be use.

1. DMA is enabled on the bus and device
2. All transmit/receive buffers are in memory supporting DMAs
3. One of the following:
	1. There are are at least `SPI_DMA_THRESHOLD` bytes to transfer
	2. There is more than a single element in the segment list
	3. The `negateCS` boolean is set to `false` in the terminating entry of the list.

The ELRS driver in `rx_sx1280.c` is an example of 3.3 where the terminating link has `negateCS` set to `false`. This then ensures that all accesses run in the background without blocking.

```
void spiSequence(const extDevice_t *dev, busSegment_t *segments);
```
This routine queues the given segment list for processing. If the device's bus is already busy then this segment list will be linked to the preceeding one in the queue so that the accesses will automatically proceed one after the other as quickly as possible.

```
void spiWait(const extDevice_t *dev);
```
Block, waiting for completion of the indicated device's bus activity.

```
bool spiIsBusy(const extDevice_t *dev);
```
Return true if the device's bus is busy.


