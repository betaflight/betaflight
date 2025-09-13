#pragma once

#include "filter_magic_h.h"


#define DEFINE_FILTER_ZEROINIT(_name_state) \
void CONCAT(_name_state,FilterInit_State)(FILTER_STATE_TYPE(_name_state) *s)  \
{                                                                       \
    for ( int i = 0; i < (int)ARRAYLEN(s->s); i++) {                    \
        s->s[i] = 0.0f;                                                 \
    }                                                                   \
}                                                                       \
/**/

#define DEFINE_FILTER_FN2_DIM0(_name_coef,_name_state)                              \
void CONCAT(_name_state,FilterInit)(FILTER_TYPE(_name_state,1) *filter)             \
{                                                                                   \
    CONCAT(_name_state,FilterInit_State)(filter->state);                            \
}                                                                                   \
void CONCAT(_name_state,FilterUpdateCoeffs)(                                        \
        FILTER_TYPE(_name_state, 1) *filter,                                        \
        const FILTER_COEFFS_TYPE(_name_coef) *coeffs) {                             \
    filter->coeffs = *coeffs;                                                       \
}                                                                                   \
float CONCAT(_name_state,FilterApply)(FILTER_TYPE(_name_state,1) *filter, const float input) \
{                                                                                   \
    return CONCAT(_name_state,FilterApply_cs)(&filter->coeffs, filter->state, input); \
}                                                                                   \
/**/

#define DEFINE_FILTER_FN2_DIM(_name_coef,_name_state,_dim)                          \
void FILTER_INIT_FN_NAME(_name_state,_dim)(FILTER_TYPE(_name_state, _dim) *filter)  \
{                                                                                   \
    for (unsigned i = 0; i < _dim; i++) {                                           \
        CONCAT(_name_state,FilterInit_State)(&filter->state[i]);                    \
    }                                                                               \
}                                                                                   \
void CONCAT3(_name_state,FilterUpdateCoeffs,_dim)(                                  \
        FILTER_TYPE(_name_state, _dim) *filter,                                     \
        const FILTER_COEFFS_TYPE(_name_coef) *coeffs) {                             \
    filter->coeffs = *coeffs;                                                       \
}                                                                                   \
float FILTER_APPLY_AXIS_FN_NAME(_name_state,_dim)(                                \
    FILTER_TYPE(_name_state, _dim) *filter, const float in, int axis)               \
{                                                                                   \
    return CONCAT(_name_state,FilterApply_cs)(&filter->coeffs, &filter->state[axis], in);  \
}                                                                                   \
void  FILTER_APPLY_FN_NAME(_name_state, _dim)(                                      \
FILTER_TYPE(_name_state, _dim) *filter, float out[_dim], const float in[_dim])      \
{                                                                                   \
    for (unsigned i = 0; i < _dim; i++) {                                           \
        out[i] = CONCAT(_name_state,FilterApply_cs)(&filter->coeffs, &filter->state[i], in[i]); \
    }                                                                               \
}                                                                                   \
/**/

#define PP_DEFINE_FILTER_FN2_DIM(r, data, elem) \
    DEFINE_FILTER_FN2_DIM(BOOST_PP_TUPLE_ELEM(0,data),BOOST_PP_TUPLE_ELEM(1,data),elem)

#define DEFINE_FILTER_FN2(_name_coef, _name_state, dim_seq)                         \
    DEFINE_FILTER_ZEROINIT(_name_state);                                            \
    DEFINE_FILTER_FN2_DIM0(_name_coef,_name_state);                                 \
    BOOST_PP_SEQ_FOR_EACH(PP_DEFINE_FILTER_FN2_DIM,                                 \
        (_name_coef,_name_state), dim_seq)                                           \
M_END

#define DEFINE_FILTER_FN(_name, dim_seq) DEFINE_FILTER_FN2(_name,_name,dim_seq);
