/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

// DEFIO_PORT_<port>_USED_MASK is bitmask of used pins on target
// DEFIO_PORT_<port>_USED_COUNT is count of used pins on target

#if defined(RP2350A)
#define DEFIO_USED_COUNT 30
#elif defined(RP2350B)
#define DEFIO_USED_COUNT 48
#else
#error "Unsupported target MCU type for PICO"
#endif

#undef DEFIO_TAG_MAKE
#define DEFIO_TAG_MAKE(pin) ((ioTag_t)(((1) << DEFIO_PORT_BITSHIFT) | (pin)))

// DEFIO_TAG__P<port><pin> will expand to TAG if defined for target, error is triggered otherwise
// DEFIO_TAG_E__P<port><pin> will expand to TAG if defined, to NONE otherwise (usefull for tables that are CPU-specific)
// DEFIO_REC__P<port><pin> will expand to ioRec* (using DEFIO_REC_INDEX(idx))

#define DEFIO_TAG__P0    DEFIO_TAG_MAKE(0)
#define DEFIO_TAG__P1    DEFIO_TAG_MAKE(1)
#define DEFIO_TAG__P2    DEFIO_TAG_MAKE(2)
#define DEFIO_TAG__P3    DEFIO_TAG_MAKE(3)
#define DEFIO_TAG__P4    DEFIO_TAG_MAKE(4)
#define DEFIO_TAG__P5    DEFIO_TAG_MAKE(5)
#define DEFIO_TAG__P6    DEFIO_TAG_MAKE(6)
#define DEFIO_TAG__P7    DEFIO_TAG_MAKE(7)
#define DEFIO_TAG__P8    DEFIO_TAG_MAKE(8)
#define DEFIO_TAG__P9    DEFIO_TAG_MAKE(9)
#define DEFIO_TAG__P10   DEFIO_TAG_MAKE(10)
#define DEFIO_TAG__P11   DEFIO_TAG_MAKE(11)
#define DEFIO_TAG__P12   DEFIO_TAG_MAKE(12)
#define DEFIO_TAG__P13   DEFIO_TAG_MAKE(13)
#define DEFIO_TAG__P14   DEFIO_TAG_MAKE(14)
#define DEFIO_TAG__P15   DEFIO_TAG_MAKE(15)
#define DEFIO_TAG__P16   DEFIO_TAG_MAKE(16)
#define DEFIO_TAG__P17   DEFIO_TAG_MAKE(17)
#define DEFIO_TAG__P18   DEFIO_TAG_MAKE(18)
#define DEFIO_TAG__P19   DEFIO_TAG_MAKE(19)
#define DEFIO_TAG__P20   DEFIO_TAG_MAKE(20)
#define DEFIO_TAG__P21   DEFIO_TAG_MAKE(21)
#define DEFIO_TAG__P22   DEFIO_TAG_MAKE(22)
#define DEFIO_TAG__P23   DEFIO_TAG_MAKE(23)
#define DEFIO_TAG__P24   DEFIO_TAG_MAKE(24)
#define DEFIO_TAG__P25   DEFIO_TAG_MAKE(25)
#define DEFIO_TAG__P26   DEFIO_TAG_MAKE(26)
#define DEFIO_TAG__P27   DEFIO_TAG_MAKE(27)
#define DEFIO_TAG__P28   DEFIO_TAG_MAKE(28)
#define DEFIO_TAG__P29   DEFIO_TAG_MAKE(29)

#if defined(RP2350B)
#define DEFIO_TAG__P30   DEFIO_TAG_MAKE(30)
#define DEFIO_TAG__P31   DEFIO_TAG_MAKE(31)
#define DEFIO_TAG__P32   DEFIO_TAG_MAKE(32)
#define DEFIO_TAG__P33   DEFIO_TAG_MAKE(33)
#define DEFIO_TAG__P34   DEFIO_TAG_MAKE(34)
#define DEFIO_TAG__P35   DEFIO_TAG_MAKE(35)
#define DEFIO_TAG__P36   DEFIO_TAG_MAKE(36)
#define DEFIO_TAG__P37   DEFIO_TAG_MAKE(37)
#define DEFIO_TAG__P38   DEFIO_TAG_MAKE(38)
#define DEFIO_TAG__P39   DEFIO_TAG_MAKE(39)
#define DEFIO_TAG__P40   DEFIO_TAG_MAKE(40)
#define DEFIO_TAG__P41   DEFIO_TAG_MAKE(41)
#define DEFIO_TAG__P42   DEFIO_TAG_MAKE(42)
#define DEFIO_TAG__P43   DEFIO_TAG_MAKE(43)
#define DEFIO_TAG__P44   DEFIO_TAG_MAKE(44)
#define DEFIO_TAG__P45   DEFIO_TAG_MAKE(45)
#define DEFIO_TAG__P46   DEFIO_TAG_MAKE(46)
#define DEFIO_TAG__P47   DEFIO_TAG_MAKE(47)
#endif

#define DEFIO_TAG_E__P0  DEFIO_TAG__P0
#define DEFIO_TAG_E__P1  DEFIO_TAG__P1
#define DEFIO_TAG_E__P2  DEFIO_TAG__P2
#define DEFIO_TAG_E__P3  DEFIO_TAG__P3
#define DEFIO_TAG_E__P4  DEFIO_TAG__P4
#define DEFIO_TAG_E__P5  DEFIO_TAG__P5
#define DEFIO_TAG_E__P6  DEFIO_TAG__P6
#define DEFIO_TAG_E__P7  DEFIO_TAG__P7
#define DEFIO_TAG_E__P8  DEFIO_TAG__P8
#define DEFIO_TAG_E__P9  DEFIO_TAG__P9
#define DEFIO_TAG_E__P10 DEFIO_TAG__P10
#define DEFIO_TAG_E__P11 DEFIO_TAG__P11
#define DEFIO_TAG_E__P12 DEFIO_TAG__P12
#define DEFIO_TAG_E__P13 DEFIO_TAG__P13
#define DEFIO_TAG_E__P14 DEFIO_TAG__P14
#define DEFIO_TAG_E__P15 DEFIO_TAG__P15
#define DEFIO_TAG_E__P16 DEFIO_TAG__P16
#define DEFIO_TAG_E__P17 DEFIO_TAG__P17
#define DEFIO_TAG_E__P18 DEFIO_TAG__P18
#define DEFIO_TAG_E__P19 DEFIO_TAG__P19
#define DEFIO_TAG_E__P20 DEFIO_TAG__P20
#define DEFIO_TAG_E__P21 DEFIO_TAG__P21
#define DEFIO_TAG_E__P22 DEFIO_TAG__P22
#define DEFIO_TAG_E__P23 DEFIO_TAG__P23
#define DEFIO_TAG_E__P24 DEFIO_TAG__P24
#define DEFIO_TAG_E__P25 DEFIO_TAG__P25
#define DEFIO_TAG_E__P26 DEFIO_TAG__P26
#define DEFIO_TAG_E__P27 DEFIO_TAG__P27
#define DEFIO_TAG_E__P28 DEFIO_TAG__P28
#define DEFIO_TAG_E__P29 DEFIO_TAG__P29

#if defined(RP2350B)
#define DEFIO_TAG_E__P30 DEFIO_TAG__P30
#define DEFIO_TAG_E__P31 DEFIO_TAG__P31
#define DEFIO_TAG_E__P32 DEFIO_TAG__P32
#define DEFIO_TAG_E__P33 DEFIO_TAG__P33
#define DEFIO_TAG_E__P34 DEFIO_TAG__P34
#define DEFIO_TAG_E__P35 DEFIO_TAG__P35
#define DEFIO_TAG_E__P36 DEFIO_TAG__P36
#define DEFIO_TAG_E__P37 DEFIO_TAG__P37
#define DEFIO_TAG_E__P38 DEFIO_TAG__P38
#define DEFIO_TAG_E__P39 DEFIO_TAG__P39
#define DEFIO_TAG_E__P40 DEFIO_TAG__P40
#define DEFIO_TAG_E__P41 DEFIO_TAG__P41
#define DEFIO_TAG_E__P42 DEFIO_TAG__P42
#define DEFIO_TAG_E__P43 DEFIO_TAG__P43
#define DEFIO_TAG_E__P44 DEFIO_TAG__P44
#define DEFIO_TAG_E__P45 DEFIO_TAG__P45
#define DEFIO_TAG_E__P46 DEFIO_TAG__P46
#define DEFIO_TAG_E__P47 DEFIO_TAG__P47
#else
#define DEFIO_TAG_E__P30 DEFIO_TAG__NONE
#define DEFIO_TAG_E__P31 DEFIO_TAG__NONE
#define DEFIO_TAG_E__P32 DEFIO_TAG__NONE
#define DEFIO_TAG_E__P33 DEFIO_TAG__NONE
#define DEFIO_TAG_E__P34 DEFIO_TAG__NONE
#define DEFIO_TAG_E__P35 DEFIO_TAG__NONE
#define DEFIO_TAG_E__P36 DEFIO_TAG__NONE
#define DEFIO_TAG_E__P37 DEFIO_TAG__NONE
#define DEFIO_TAG_E__P38 DEFIO_TAG__NONE
#define DEFIO_TAG_E__P39 DEFIO_TAG__NONE
#define DEFIO_TAG_E__P40 DEFIO_TAG__NONE
#define DEFIO_TAG_E__P41 DEFIO_TAG__NONE
#define DEFIO_TAG_E__P42 DEFIO_TAG__NONE
#define DEFIO_TAG_E__P43 DEFIO_TAG__NONE
#define DEFIO_TAG_E__P44 DEFIO_TAG__NONE
#define DEFIO_TAG_E__P45 DEFIO_TAG__NONE
#define DEFIO_TAG_E__P46 DEFIO_TAG__NONE
#define DEFIO_TAG_E__P47 DEFIO_TAG__NONE
#endif

#define DEFIO_REC__P0    DEFIO_REC_INDEXED(0)
#define DEFIO_REC__P1    DEFIO_REC_INDEXED(1)
#define DEFIO_REC__P2    DEFIO_REC_INDEXED(2)
#define DEFIO_REC__P3    DEFIO_REC_INDEXED(3)
#define DEFIO_REC__P4    DEFIO_REC_INDEXED(4)
#define DEFIO_REC__P5    DEFIO_REC_INDEXED(5)
#define DEFIO_REC__P6    DEFIO_REC_INDEXED(6)
#define DEFIO_REC__P7    DEFIO_REC_INDEXED(7)
#define DEFIO_REC__P8    DEFIO_REC_INDEXED(8)
#define DEFIO_REC__P9    DEFIO_REC_INDEXED(9)
#define DEFIO_REC__P10   DEFIO_REC_INDEXED(10)
#define DEFIO_REC__P11   DEFIO_REC_INDEXED(11)
#define DEFIO_REC__P12   DEFIO_REC_INDEXED(12)
#define DEFIO_REC__P13   DEFIO_REC_INDEXED(13)
#define DEFIO_REC__P14   DEFIO_REC_INDEXED(14)
#define DEFIO_REC__P15   DEFIO_REC_INDEXED(15)
#define DEFIO_REC__P16   DEFIO_REC_INDEXED(16)
#define DEFIO_REC__P17   DEFIO_REC_INDEXED(17)
#define DEFIO_REC__P18   DEFIO_REC_INDEXED(18)
#define DEFIO_REC__P19   DEFIO_REC_INDEXED(19)
#define DEFIO_REC__P20   DEFIO_REC_INDEXED(20)
#define DEFIO_REC__P21   DEFIO_REC_INDEXED(21)
#define DEFIO_REC__P22   DEFIO_REC_INDEXED(22)
#define DEFIO_REC__P23   DEFIO_REC_INDEXED(23)
#define DEFIO_REC__P24   DEFIO_REC_INDEXED(24)
#define DEFIO_REC__P25   DEFIO_REC_INDEXED(25)
#define DEFIO_REC__P26   DEFIO_REC_INDEXED(26)
#define DEFIO_REC__P27   DEFIO_REC_INDEXED(27)
#define DEFIO_REC__P28   DEFIO_REC_INDEXED(28)
#define DEFIO_REC__P29   DEFIO_REC_INDEXED(29)

#if defined(RP2350B)
#define DEFIO_REC__P30   DEFIO_REC_INDEXED(30)
#define DEFIO_REC__P31   DEFIO_REC_INDEXED(31)
#define DEFIO_REC__P32   DEFIO_REC_INDEXED(32)
#define DEFIO_REC__P33   DEFIO_REC_INDEXED(33)
#define DEFIO_REC__P34   DEFIO_REC_INDEXED(34)
#define DEFIO_REC__P35   DEFIO_REC_INDEXED(35)
#define DEFIO_REC__P36   DEFIO_REC_INDEXED(36)
#define DEFIO_REC__P37   DEFIO_REC_INDEXED(37)
#define DEFIO_REC__P38   DEFIO_REC_INDEXED(38)
#define DEFIO_REC__P39   DEFIO_REC_INDEXED(39)
#define DEFIO_REC__P40   DEFIO_REC_INDEXED(40)
#define DEFIO_REC__P41   DEFIO_REC_INDEXED(41)
#define DEFIO_REC__P42   DEFIO_REC_INDEXED(42)
#define DEFIO_REC__P43   DEFIO_REC_INDEXED(43)
#define DEFIO_REC__P44   DEFIO_REC_INDEXED(44)
#define DEFIO_REC__P45   DEFIO_REC_INDEXED(45)
#define DEFIO_REC__P46   DEFIO_REC_INDEXED(46)
#define DEFIO_REC__P47   DEFIO_REC_INDEXED(47)
#endif

// DEFIO_IO_USED_COUNT is number of io pins supported on target
#if defined(RP2350A)
#define DEFIO_IO_USED_COUNT 30
#elif defined(RP2350B)
#define DEFIO_IO_USED_COUNT 48
#endif

// DEFIO_PORT_USED_LIST - comma separated list of bitmask for all used ports.
// DEFIO_PORT_OFFSET_LIST - comma separated list of port offsets (count of pins before this port)
// unused ports on end of list are skipped
#define DEFIO_PORT_USED_COUNT   1
#define DEFIO_PORT_USED_LIST
#define DEFIO_PORT_OFFSET_LIST  0
#define DEFIO_PIN_USED_COUNT    DEFIO_USED_COUNT
