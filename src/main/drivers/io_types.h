#pragma once

#include <stdint.h>

// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
typedef uint8_t ioTag_t;       // packet tag to specify IO pin
typedef void* IO_t;            // type specifying IO pin. Currently ioRec_t pointer, but this may change

// NONE initializer for ioTag_t variables
#define IOTAG_NONE ((ioTag_t)0)

// NONE initializer for IO_t variable
#define IO_NONE ((IO_t)0)

// both ioTag_t and IO_t are guarantied to be zero if pinid is NONE (no pin)
// this simplifies initialization (globals are zeroed on start) and allows
//  omitting unused fields in structure initializers.
// it is also possible to use IO_t and ioTag_t as boolean value
//   TODO - this may conflict with requirement to generate warning/error on IO_t - ioTag_t assignment
//   IO_t being pointer is only possibility I know of ..

// pin config handling
// pin config is packed into ioConfig_t to decrease memory requirements
// IOCFG_x macros are defined for common combinations for all CPUs; this
//  helps masking CPU differences

typedef uint8_t ioConfig_t;  // packed IO configuration
