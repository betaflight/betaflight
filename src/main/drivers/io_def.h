/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <boost/preprocessor/punctuation/is_begin_parens.hpp>
#include <boost/preprocessor/seq/elem.hpp>
#include <boost/preprocessor/seq/filter.hpp>
#include <boost/preprocessor/variadic/to_seq.hpp>

#include "common/utils.h"

// return ioTag_t for given pinid
// tag for NONE must be false
#define DEFIO_TAG(pinid) CONCAT(DEFIO_TAG__, pinid)
// NONE value must be surrounded by parentheses
#define DEFIO_TAG__NONE (0)
#define DEFIO_TAG_E(pinid) CONCAT(DEFIO_TAG_E__, pinid)
#define DEFIO_TAG_E__NONE (0)

// construct undefined variable if user requests pin that is unavailable on current target
#define DEFIO_UNSUPPORTED_I(port, pin, target) defio_error_P##port##pin##_is_not_supported_on_##target

// macros for variable-sized argument list in IO_TAG_FIRST
// these macros scan passed arguments, using first one that expands to tag (for example PA1, NONE)
// undefined or empty values are ignored
// at least one value must be valid tag (or NONE), otherwise error is treggered
// Internally, anything that is enclosed in parentheses is assumed to be tag. This can be improved,
// if necessary, by changing io_defs_generated implementation and tagging valid values

#define DEFIO_TAG_FIRST(...) DEFIO_TAG(DEFIO_TAG__FIRST_DEFINED(__VA_ARGS__))

// check if parameter expands to tag (tag/error is marked by surrounding it with parentheses)
#define DEFIO_TAG__IS_DEFINED(val) BOOST_PP_IS_BEGIN_PARENS(DEFIO_TAG(val))
// filter seq removing non-tags, then return first element
// NOARGVALID is appended to user-supplied list, so first element is always available
#define DEFIO_TAG__FIRST_DEFINED_FILTER(s, data, elem) DEFIO_TAG__IS_DEFINED(elem)
#define DEFIO_TAG__FIRST_DEFINED_SEQ(seq) BOOST_PP_SEQ_ELEM(0, BOOST_PP_SEQ_FILTER(DEFIO_TAG__FIRST_DEFINED_FILTER, _, seq))
#define DEFIO_TAG__FIRST_DEFINED(...) DEFIO_TAG__FIRST_DEFINED_SEQ(BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__)(NOARGVALID))

// if no pinid is valid, generate error
#define DEFIO_TAG__NOARGVALID (no_valid_pinid_was_passed_to_DEFIO_TAG_FIRST)

// return ioRec_t or NULL for given pinid
// tags should be preferred, possibly removing it in future
// io_impl.h must be included when this macro is used
#define DEFIO_REC(pinid) CONCAT(DEFIO_REC__, pinid)
#define DEFIO_REC__NONE NULL
#define DEFIO_REC__UNSUPPORTED(port, pin) DEFIO_UNSUPPORTED_I(port,pin,TARGET_MCU)

#define DEFIO_IO(pinid) (IO_t)DEFIO_REC(pinid)
// TODO - macro to check for pinid NONE (fully in preprocessor)

// get ioRec by index
#define DEFIO_REC_INDEXED(idx) (ioRecs + (idx))

// ioTag_t accessor macros
#define DEFIO_TAG_MAKE(gpioid, pin) ((ioTag_t)((((gpioid) + 1) << 4) | (pin)))
// DEFIO_TAG__UNSUPPORTED must be something surrounded by parentheses (see IO_TAG_FIRST)
#define DEFIO_TAG__UNSUPPORTED(port, pin) (DEFIO_UNSUPPORTED_I(port,pin,TARGET_MCU))

#define DEFIO_TAG_ISEMPTY(tag) (!(tag))
#define DEFIO_TAG_GPIOID(tag) (((tag) >> 4) - 1)
#define DEFIO_TAG_PIN(tag) ((tag) & 0x0f)

// TARGET must define used pins
#include "target.h"
// include template-generated macros for IO pins
#include "io_def_generated.h"
