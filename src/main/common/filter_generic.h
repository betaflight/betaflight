#pragma once

#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/variadic/to_seq.hpp>
#include <boost/preprocessor/tuple/to_seq.hpp>

#include "filter.h"
#include "filter_magic_h.h"


#define GENERIC_FILTER_VIRTUAL_TYPE(dims) FILTER_TYPE(generic, dims)
#define GENERIC_FILTER_APPLY_FN_TYPE(dims) FILTER_APPLY_FN_TYPE(generic, dims)
#define GENERIC_FILTER_APPLY_AXIS_FN_TYPE(dims) FILTER_APPLY_AXIS_FN_TYPE(generic, dims)

// these functions alias real, filter-typed function;
struct genericFilter_v1 { float dummy[1]; };
typedef struct genericFilter_v1 GENERIC_FILTER_VIRTUAL_TYPE(1);
struct genericFilter_v3 { float dummy[1]; } ;
typedef struct genericFilter_v3 GENERIC_FILTER_VIRTUAL_TYPE(3);

typedef void (GENERIC_FILTER_APPLY_FN_TYPE(1))(GENERIC_FILTER_VIRTUAL_TYPE(1) *filter, float out[1], float in[1]);
typedef void (GENERIC_FILTER_APPLY_FN_TYPE(3))(GENERIC_FILTER_VIRTUAL_TYPE(3) *filter, float out[3], float in[3]);
typedef float (GENERIC_FILTER_APPLY_AXIS_FN_TYPE(1))(GENERIC_FILTER_VIRTUAL_TYPE(1) *filter, float in, int axis);
typedef float (GENERIC_FILTER_APPLY_AXIS_FN_TYPE(3))(GENERIC_FILTER_VIRTUAL_TYPE(3) *filter, float in, int axis);

#define GENERIC_FILTER_MEMBER(name, dims, is_axis)              \
    struct {                                                    \
        BOOST_PP_IIF(is_axis,                                   \
            FILTER_APPLY_AXIS_FN_TYPE(name,dims) *applyAxis,    \
            FILTER_APPLY_FN_TYPE(name, dims) *apply);           \
        FILTER_TYPE(name, dims) filter;                         \
    } name;

#define PP_GENERIC_FILTER_MEMBER(r, data, elem)                               \
    GENERIC_FILTER_MEMBER(elem, BOOST_PP_TUPLE_ELEM(0, data), BOOST_PP_TUPLE_ELEM(1, data))

// filters_seq: (a)(b)(c)
// apply / filter must match fister-specific struct
#define GENERIC_FILTER_TYPE_SEQ(dims, is_axis, filters_seq)                         \
union {                                                                             \
    GENERIC_FILTER_MEMBER(generic, dims, is_axis)                                   \
    BOOST_PP_SEQ_FOR_EACH(PP_GENERIC_FILTER_MEMBER, (dims, is_axis), filters_seq)   \
}

// specify filters as tuple - (a, b, c)
#define GENERIC_FILTER_TYPE(dims, filters_tuple) \
    GENERIC_FILTER_TYPE_SEQ(dims, 0, BOOST_PP_TUPLE_TO_SEQ(filters_tuple))

// specify filters as tuple - (a, b, c)
#define GENERIC_AXISFILTER_TYPE(dims, filters_tuple) \
    GENERIC_FILTER_TYPE_SEQ(dims, 1, BOOST_PP_TUPLE_TO_SEQ(filters_tuple))
