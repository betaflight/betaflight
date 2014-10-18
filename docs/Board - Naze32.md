# Board - Naze32

The Naze32 target supports all Naze hardware revisions.  Revison 4 and Revision 5 are used and
frequently flown by the primary maintainer.  Previous Naze hardware revisions may have issues,
if found please report via the github issue tracker.

# Serial Ports

| Value | Identifier   | RX        | TX                 | Notes                                                                                       |
| ----- | ------------ | --------- | ------------------ | ------------------------------------------------------------------------------------------- |
| 1     | USART1       | RX  / PA9 | TX  / PA10 / TELEM | TELEM output is always inverted (for FrSky). Internally connected to USB port via CP2102 IC |
| 2     | USART2       | RC3 / PA2 | RC4 / PA3          |                                                                                             |
| 3     | SoftSerial 1 | RC5 / PA6 | RC6 / PA7          |                                                                                             |
| 4     | SoftSerial 2 | RC7 / PB0 | RC8 / PB1          |                                                                                             |

* You cannot use USART1/TX/TX/TELEM pins at the same time. 
* You may encounter flashing problems if you have something connected to the RX/TX pins.  Try disconnecting RX/TX.

