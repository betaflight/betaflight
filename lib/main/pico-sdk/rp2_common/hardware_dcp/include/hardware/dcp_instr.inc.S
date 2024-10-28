/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

.macro INIT
 cdp p4,#0,c0,c0,c0,#0
.endm
.macro ADD0
 cdp p4,#0,c0,c0,c1,#0
.endm
.macro ADD1
 cdp p4,#1,c0,c0,c1,#0
.endm
.macro SUB1
 cdp p4,#1,c0,c0,c1,#1
.endm
.macro SQR0
 cdp p4,#2,c0,c0,c1,#0
.endm
.macro NORM
 cdp p4,#8,c0,c0,c2,#0
.endm
.macro NRDF
 cdp p4,#8,c0,c0,c2,#1
.endm
.macro NRDD
 cdp p4,#8,c0,c0,c0,#1
.endm
.macro NTDC
 cdp p4,#8,c0,c0,c0,#2
.endm
.macro NRDC
 cdp p4,#8,c0,c0,c0,#3
.endm
.macro WXMD rl,rh
 mcrr p4,#0,\rl,\rh,c0
.endm
.macro WYMD rl,rh
 mcrr p4,#0,\rl,\rh,c1
.endm
.macro WEFD rl,rh
 mcrr p4,#0,\rl,\rh,c2
.endm
.macro WXUP rl,rh
 mcrr p4,#1,\rl,\rh,c0
.endm
.macro WYUP rl,rh
 mcrr p4,#1,\rl,\rh,c1
.endm
.macro WXYU rl,rh
 mcrr p4,#1,\rl,\rh,c2
.endm
.macro WXMS rl,rh
 mcrr p4,#2,\rl,\rh,c0
.endm
.macro WXMO rl,rh
 mcrr p4,#3,\rl,\rh,c0
.endm
.macro WXDD rl,rh
 mcrr p4,#4,\rl,\rh,c0
.endm
.macro WXDQ rl,rh
 mcrr p4,#5,\rl,\rh,c0
.endm
.macro WXUC rl,rh
 mcrr p4,#6,\rl,\rh,c0
.endm
.macro WXIC rl,rh
 mcrr p4,#7,\rl,\rh,c0
.endm
.macro WXDC rl,rh
 mcrr p4,#8,\rl,\rh,c0
.endm
.macro WXFC rl,rh
 mcrr p4,#9,\rl,\rh,c2
.endm
.macro WXFM rl,rh
 mcrr p4,#10,\rl,\rh,c0
.endm
.macro WXFD rl,rh
 mcrr p4,#11,\rl,\rh,c0
.endm
.macro WXFQ rl,rh
 mcrr p4,#12,\rl,\rh,c0
.endm
.macro RXVD rt
 mrc p4,#0,\rt,c0,c0,#0
.endm
.macro RCMP rt
 mrc p4,#0,\rt,c0,c0,#1
.endm
.macro RDFA rt
 mrc p4,#0,\rt,c0,c2,#0
.endm
.macro RDFS rt
 mrc p4,#0,\rt,c0,c2,#1
.endm
.macro RDFM rt
 mrc p4,#0,\rt,c0,c2,#2
.endm
.macro RDFD rt
 mrc p4,#0,\rt,c0,c2,#3
.endm
.macro RDFQ rt
 mrc p4,#0,\rt,c0,c2,#4
.endm
.macro RDFG rt
 mrc p4,#0,\rt,c0,c2,#5
.endm
.macro RDIC rt
 mrc p4,#0,\rt,c0,c3,#0
.endm
.macro RDUC rt
 mrc p4,#0,\rt,c0,c3,#1
.endm
.macro RXMD rl,rh
 mrrc p4,#0,\rl,\rh,c8
.endm
.macro RYMD rl,rh
 mrrc p4,#0,\rl,\rh,c9
.endm
.macro REFD rl,rh
 mrrc p4,#0,\rl,\rh,c10
.endm
.macro RXMS rl,rh,s
 mrrc p4,#\s,\rl,\rh,c4
.endm
.macro RYMS rl,rh,s
 mrrc p4,#\s,\rl,\rh,c5
.endm
.macro RXYH rl,rh
 mrrc p4,#1,\rl,\rh,c1
.endm
.macro RYMR rl,rh
 mrrc p4,#2,\rl,\rh,c1
.endm
.macro RXMQ rl,rh
 mrrc p4,#4,\rl,\rh,c1
.endm
.macro RDDA rl,rh
 mrrc p4,#1,\rl,\rh,c0
.endm
.macro RDDS rl,rh
 mrrc p4,#3,\rl,\rh,c0
.endm
.macro RDDM rl,rh
 mrrc p4,#5,\rl,\rh,c0
.endm
.macro RDDD rl,rh
 mrrc p4,#7,\rl,\rh,c0
.endm
.macro RDDQ rl,rh
 mrrc p4,#9,\rl,\rh,c0
.endm
.macro RDDG rl,rh
 mrrc p4,#11,\rl,\rh,c0
.endm
.macro PXVD rt
 mrc2 p4,#0,\rt,c0,c0,#0
.endm
.macro PCMP rt
 mrc2 p4,#0,\rt,c0,c0,#1
.endm
.macro PDFA rt
 mrc2 p4,#0,\rt,c0,c2,#0
.endm
.macro PDFS rt
 mrc2 p4,#0,\rt,c0,c2,#1
.endm
.macro PDFM rt
 mrc2 p4,#0,\rt,c0,c2,#2
.endm
.macro PDFD rt
 mrc2 p4,#0,\rt,c0,c2,#3
.endm
.macro PDFQ rt
 mrc2 p4,#0,\rt,c0,c2,#4
.endm
.macro PDFG rt
 mrc2 p4,#0,\rt,c0,c2,#5
.endm
.macro PDIC rt
 mrc2 p4,#0,\rt,c0,c3,#0
.endm
.macro PDUC rt
 mrc2 p4,#0,\rt,c0,c3,#1
.endm
.macro PXMD rl,rh
 mrrc2 p4,#0,\rl,\rh,c8
.endm
.macro PYMD rl,rh
 mrrc2 p4,#0,\rl,\rh,c9
.endm
.macro PEFD rl,rh
 mrrc2 p4,#0,\rl,\rh,c10
.endm
.macro PXMS rl,rh,s
 mrrc2 p4,#\s,\rl,\rh,c4
.endm
.macro PYMS rl,rh,s
 mrrc2 p4,#\s,\rl,\rh,c5
.endm
.macro PXYH rl,rh
 mrrc2 p4,#1,\rl,\rh,c1
.endm
.macro PYMR rl,rh
 mrrc2 p4,#2,\rl,\rh,c1
.endm
.macro PXMQ rl,rh
 mrrc2 p4,#4,\rl,\rh,c1
.endm
.macro PDDA rl,rh
 mrrc2 p4,#1,\rl,\rh,c0
.endm
.macro PDDS rl,rh
 mrrc2 p4,#3,\rl,\rh,c0
.endm
.macro PDDM rl,rh
 mrrc2 p4,#5,\rl,\rh,c0
.endm
.macro PDDD rl,rh
 mrrc2 p4,#7,\rl,\rh,c0
.endm
.macro PDDQ rl,rh
 mrrc2 p4,#9,\rl,\rh,c0
.endm
.macro PDDG rl,rh
 mrrc2 p4,#11,\rl,\rh,c0
.endm

