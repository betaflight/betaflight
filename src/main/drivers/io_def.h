#pragma once

#include "common/utils.h"

// return ioTag_t for given pinid
// tag for NONE must be false
#define DEFIO_TAG(pinid) CONCAT(DEFIO_TAG__, pinid)
#define DEFIO_TAG__NONE 0
#define DEFIO_TAG_E(pinid) CONCAT(DEFIO_TAG_E__, pinid)
#define DEFIO_TAG_E__NONE 0


// return ioRec_t or NULL for given pinid
// tags should be preferred, possibly removing it in future
// io_impl.h must be included when this macro is used
#define DEFIO_REC(pinid) CONCAT(DEFIO_REC__, pinid)
#define DEFIO_REC__NONE NULL

#define DEFIO_IO(pinid) (IO_t)DEFIO_REC(pinid)
// TODO - macro to check for pinid NONE (fully in preprocessor)

// get ioRec by index
#define DEFIO_REC_INDEXED(idx) (ioRecs + (idx))

// ioTag_t accessor macros
#define DEFIO_TAG_MAKE(gpioid, pin) ((((gpioid) + 1) << 4) | (pin))
#define DEFIO_TAG_ISEMPTY(tag) (!(tag))
#define DEFIO_TAG_GPIOID(tag) (((tag) >> 4) - 1)
#define DEFIO_TAG_PIN(tag) ((tag) & 0x0f)

// TARGET must define used pins
#include "target.h"
// include template-generated macros for IO pins
#include "io_def_generated.h"

