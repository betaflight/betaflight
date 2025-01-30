/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Canned instruction sequences for use with the DCP

.macro dcp_fadd_m rz,rx,ry
  WXYU \rx,\ry
  ADD0
  ADD1
  NRDF
  RDFA \rz
.endm

.macro dcp_fsub_m rz,rx,ry
  WXYU \rx,\ry
  ADD0
  SUB1
  NRDF
  RDFS \rz
.endm

.macro dcp_fmul_m rz,rx,ry,ra,rb
  WXYU \rx,\ry
  RXYH \ra,\rb
  umull \ra,\rb,\ra,\rb
  WXFM \ra,\rb
  NRDF
  RDFM \rz
.endm

.macro dcp_fxprod_m rzl,rzh,rx,ry,ra,rb
  WXYU \rx,\ry
  RXYH \ra,\rb
  umull \ra,\rb,\ra,\rb
  WXMO \ra,\rb
  NRDD
  RDDM \rzl,\rzh
.endm

.macro dcp_fdiv_fast_m rz,rx,ry,ra,rb,rc
  WXYU \rx,\ry
  RYMR \ra,\rb
  umull \rb,\rc,\ra,\rb
  mvn \rc,\rc,lsl #2
  smmlar \ra,\rc,\ra,\ra
  smmulr \rc,\rc,\rc
  smmlar \ra,\rc,\ra,\ra
  RXYH \rb,\rc
  umull \ra,\rb,\ra,\rb
  WXFD \rb,\rb
  NRDF
  RDFD \rz
.endm

.macro dcp_fdiv_m rz,rx,ry,ra,rb,rc,rd
  WXYU \rx,\ry
  RYMR \ra,\rb
  umull \rb,\rc,\ra,\rb
  mvn \rc,\rc,lsl #2
  smmlar \ra,\rc,\ra,\ra
  smmulr \rc,\rc,\rc
  smmlar \ra,\rc,\ra,\ra
  RXYH \rb,\rc
  umull \rd,\ra,\ra,\rb
  orr \ra,\ra,\ra,lsr #24
  bic \ra,\ra,\ra,lsr #25
  bic \ra,\ra,#15
  mov \rc,\rc,lsr #7
  mul \rd,\ra,\rc
  rsb \rd,\rd,\rb,lsl #22
  sub \ra,\ra,\rd,lsr #31
  WXFD \ra,\ra
  NRDF
  RDFD \rz
.endm

.macro dcp_fsqrt_fast_m rz,rx,ra,rb,rc,rd
  WXYU \rx,\rx
  SQR0
  RXMQ \rc,\rd
  umull \ra,\rb,\rc,\rc
  umull \ra,\rb,\rb,\rd
  mov \rb,\rb,lsl #3
  sub \rb,\rb,#2147483648
  smmlsr \rc,\rc,\rb,\rc
  umull \ra,\rb,\rc,\rc
  umull \ra,\rb,\rb,\rd
  movs \ra,\ra,lsr #28
  adc \rb,\ra,\rb,lsl #4
  smmulr \ra,\rb,\rc
  sub \rc,\rc,\ra,asr #1
  umull \ra,\rb,\rc,\rd
  WXFQ \ra,\rb
  NRDF
  RDFQ \rz
.endm

.macro dcp_fsqrt_m rz,rx,ra,rb,rc,rd
  WXYU \rx,\rx
  SQR0
  RXMQ \rc,\rd
  umull \ra,\rb,\rc,\rc
  umull \ra,\rb,\rb,\rd
  mov \rb,\rb,lsl #3
  sub \rb,\rb,#2147483648
  smmlsr \rc,\rc,\rb,\rc
  umull \ra,\rb,\rc,\rc
  umull \ra,\rb,\rb,\rd
  movs \ra,\ra,lsr #28
  adc \rb,\ra,\rb,lsl #4
  smmulr \ra,\rb,\rc
  sub \rc,\rc,\ra,asr #1
  umull \ra,\rb,\rc,\rd
  orr \rb,\rb,#63
  mov \ra,\rb,lsr #5
  mul \ra,\ra,\ra
  rsb \ra,\ra,\rd,lsl #18
  bic \rb,\rb,\ra,lsr #26
  WXFQ \ra,\rb
  NRDF
  RDFQ \rz
.endm

.macro dcp_fclassify_m rz,rx
  WXYU \rx,\rx
  RXVD \rz
.endm

.macro dcp_fcmp_m rz,rx,ry
  WXYU \rx,\ry
  ADD0
  RCMP \rz
.endm

.macro dcp_dadd_m rzl,rzh,rxl,rxh,ryl,ryh
  WXUP \rxl,\rxh
  WYUP \ryl,\ryh
  ADD0
  ADD1
  NRDD
  RDDA \rzl,\rzh
.endm

.macro dcp_dsub_m rzl,rzh,rxl,rxh,ryl,ryh
  WXUP \rxl,\rxh
  WYUP \ryl,\ryh
  ADD0
  SUB1
  NRDD
  RDDS \rzl,\rzh
.endm

.macro dcp_dmul_m rzl,rzh,rxl,rxh,ryl,ryh,ra,rb,rc,rd,re,rf,rg
  WXUP \rxl,\rxh
  WYUP \ryl,\ryh
  RXMS \ra,\rb,0
  RYMS \rc,\rd,0
  umull \re,\rf,\ra,\rc
  movs \rg,#0
  umlal \rf,\rg,\ra,\rd
  umlal \rf,\rg,\rb,\rc
  WXMS \re,\rf
  movs \re,#0
  umlal \rg,\re,\rb,\rd
  WXMO \rg,\re
  NRDD
  RDDM \rzl,\rzh
.endm

.macro dcp_ddiv_fast_m rzl,rzh,rxl,rxh,ryl,ryh,ra,rb,rc,rd,re
  WXUP \rxl,\rxh
  WYUP \ryl,\ryh
  RYMR \ra,\rb
  umull \rb,\rc,\ra,\rb
  mvn \rc,\rc,lsl #2
  smmlar \ra,\rc,\ra,\ra
  smmulr \rc,\rc,\rc
  smmlar \ra,\rc,\ra,\ra
  sub \re,\ra,\ra,lsr #31
  RXMS \rc,\rd,0
  smmulr \rb,\re,\rd
  RYMS \rc,\rd,1
  umull \rc,\ra,\rb,\rc
  mla \ra,\rb,\rd,\ra
  RXMS \rc,\rd,4
  sub \ra,\rc,\ra
  smmulr \rc,\ra,\re
  mov \rd,\rb,lsr #4
  adds \ra,\rc,\rb,lsl #28
  adc \rb,\rd,\rc,asr #31
  WXDD \ra,\rb
  NRDD
  RDDD \rzl,\rzh
.endm

.macro dcp_ddiv_m rzl,rzh,rxl,rxh,ryl,ryh,ra,rb,rc,rd,re
  WXUP \rxl,\rxh
  WYUP \ryl,\ryh
  RYMR \ra,\rb
  umull \rb,\rc,\ra,\rb
  mvn \rc,\rc,lsl #2
  smmlar \ra,\rc,\ra,\ra
  smmulr \rc,\rc,\rc
  smmlar \ra,\rc,\ra,\ra
  sub \re,\ra,\ra,lsr #31
  RXMS \rc,\rd,0
  smmulr \rb,\re,\rd
  RYMS \rc,\rd,1
  umull \rc,\ra,\rb,\rc
  mla \ra,\rb,\rd,\ra
  RXMS \rc,\rd,4
  sub \ra,\rc,\ra
  smmulr \rc,\ra,\re
  mov \rd,\rb,lsr #4
  adds \ra,\rc,\rb,lsl #28
  adc \rb,\rd,\rc,asr #31
  orr \ra,\ra,\rb,lsr #21
  bic \ra,\ra,\rb,lsr #22
  bic \ra,\ra,#7
  RYMS \rc,\rd,7
  umull \rd,\re,\ra,\rc
  RYMS \rc,\rd,7
  mla \re,\ra,\rd,\re
  mla \re,\rb,\rc,\re
  RXMS \rc,\rd,0
  sub \re,\re,\rc,lsl #18
  orr \ra,\ra,\re,lsr #29
  sub \ra,\ra,#1
  WXDD \ra,\rb
  NRDD
  RDDD \rzl,\rzh
.endm

.macro dcp_dsqrt_fast_m rzl,rzh,rxl,rxh,ra,rb,rc,rd,re
  WXUP \rxl,\rxh
  SQR0
  RXMQ \rc,\rd
  umull \ra,\rb,\rc,\rc
  umull \ra,\rb,\rb,\rd
  mov \rb,\rb,lsl #3
  sub \rb,\rb,#2147483648
  smmlsr \rc,\rc,\rb,\rc
  umull \ra,\rb,\rc,\rc
  umull \ra,\rb,\rb,\rd
  movs \ra,\ra,lsr #28
  adc \rb,\ra,\rb,lsl #4
  smmulr \ra,\rb,\rc
  sub \rc,\rc,\ra,asr #1
  umull \ra,\rb,\rc,\rd
  mov \rb,\rb,lsr #1
  umull \rd,\ra,\rb,\rb
  RXMS \ra,\re,6
  sub \ra,\ra,\rd
  smmulr \ra,\ra,\rc
  add \rb,\rb,\ra,asr #28
  mov \ra,\ra,lsl #4
  WXDQ \ra,\rb
  NRDD
  RDDQ \rzl,\rzh
.endm

.macro dcp_dsqrt_m rzl,rzh,rxl,rxh,ra,rb,rc,rd,re
  WXUP \rxl,\rxh
  SQR0
  RXMQ \rc,\rd
  umull \ra,\rb,\rc,\rc
  umull \ra,\rb,\rb,\rd
  mov \rb,\rb,lsl #3
  sub \rb,\rb,#2147483648
  smmlsr \rc,\rc,\rb,\rc
  umull \ra,\rb,\rc,\rc
  umull \ra,\rb,\rb,\rd
  movs \ra,\ra,lsr #28
  adc \rb,\ra,\rb,lsl #4
  smmulr \ra,\rb,\rc
  sub \rc,\rc,\ra,asr #1
  umull \ra,\rb,\rc,\rd
  mov \rb,\rb,lsr #1
  umull \rd,\ra,\rb,\rb
  RXMS \ra,\re,6
  sub \rd,\ra,\rd
  smmulr \rd,\rd,\rc
  add \rb,\rb,\rd,asr #28
  mov \rd,\rd,lsl #4
  bic \rd,\rd,#255
  orr \rd,\rd,#128
  umull \re,\rc,\rd,\rd
  mul \re,\rb,\rb
  umlal \rc,\re,\rb,\rd
  umlal \rc,\re,\rb,\rd
  sub \re,\re,\ra
  orr \rd,\rd,\re,lsr #24
  sub \rd,\rd,#11
  WXDQ \rd,\rb
  NRDD
  RDDQ \rzl,\rzh
.endm

.macro dcp_dclassify_m rz,rxl,rxh
  WXUP \rxl,\rxh
  RXVD \rz
.endm

.macro dcp_dcmp_m rz,rxl,rxh,ryl,ryh
  WXUP \rxl,\rxh
  WYUP \ryl,\ryh
  ADD0
  RCMP \rz
.endm

.macro dcp_float2double_m rzl,rzh,rx
  WXYU \rx,\rx
  NRDD
  RDDG \rzl,\rzh
.endm

.macro dcp_double2float_m rz,rxl,rxh
  WXUP \rxl,\rxh
  NRDF
  RDFG \rz
.endm

.macro dcp_int2double_m rzl,rzh,rx
  WXIC \rx,\rx
  ADD0
  SUB1
  NRDD
  RDDS \rzl,\rzh
.endm

.macro dcp_uint2double_m rzl,rzh,rx
  WXUC \rx,\rx
  ADD0
  SUB1
  NRDD
  RDDS \rzl,\rzh
.endm

.macro dcp_int2float_m rz,rx
  WXIC \rx,\rx
  ADD0
  SUB1
  NRDF
  RDFS \rz
.endm

.macro dcp_uint2float_m rz,rx
  WXUC \rx,\rx
  ADD0
  SUB1
  NRDF
  RDFS \rz
.endm

.macro dcp_double2int_m rz,rxl,rxh
  WXDC \rxl,\rxh
  ADD0
  ADD1
  NTDC
  RDIC \rz
.endm

.macro dcp_double2uint_m rz,rxl,rxh
  WXDC \rxl,\rxh
  ADD0
  ADD1
  NTDC
  RDUC \rz
.endm

.macro dcp_float2int_m rz,rx
  WXFC \rx,\rx
  ADD0
  ADD1
  NTDC
  RDIC \rz
.endm

.macro dcp_float2uint_m rz,rx
  WXFC \rx,\rx
  ADD0
  ADD1
  NTDC
  RDUC \rz
.endm

.macro dcp_double2int_r_m rz,rxl,rxh
  WXDC \rxl,\rxh
  ADD0
  ADD1
  NRDC
  RDIC \rz
.endm

.macro dcp_double2uint_r_m rz,rxl,rxh
  WXDC \rxl,\rxh
  ADD0
  ADD1
  NRDC
  RDUC \rz
.endm

.macro dcp_float2int_r_m rz,rx
  WXFC \rx,\rx
  ADD0
  ADD1
  NRDC
  RDIC \rz
.endm

.macro dcp_float2uint_r_m rz,rx
  WXFC \rx,\rx
  ADD0
  ADD1
  NRDC
  RDUC \rz
.endm
