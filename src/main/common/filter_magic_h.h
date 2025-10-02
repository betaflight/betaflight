#pragma once

#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/list/to_seq.hpp>
#include <boost/preprocessor/variadic/to_list.hpp>
#include <boost/preprocessor/seq/for_each_i.hpp>
#include <boost/preprocessor/punctuation/comma_if.hpp>
#include <boost/preprocessor/control/if.hpp>

#include "common/utils.h"

// enforce semicolon
#ifndef M_END
#define M_END struct semicolon_needed
#endif

#define FILTER_COEFFS_TYPE(_name) CONCAT(_name,FilterCoeffs_t)
#define FILTER_STATE_TYPE(_name) CONCAT(_name,FilterState_t)
#define FILTER_TYPE(_name, _dim) CONCAT4(_name,FilterVec,_dim,_t)
#define FILTER_TYPE0(_name) CONCAT(_name,Filter_t)
#define FILTER_INIT_FN_NAME(_name, _dim) CONCAT3(_name,FilterInit,_dim)
#define FILTER_INIT_FN_NAME0(_name) CONCAT(_name,FilterInit)
#define FILTER_APPLY_FN_NAME(_name, _dim) CONCAT3(_name,FilterApply,_dim)
#define FILTER_APPLY_FN_TYPE(_name, _dim) CONCAT4(_name,FilterApply,_dim,Fn)
#define FILTER_APPLY_FN_NAME0(_name) CONCAT(_name,FilterApply)
#define FILTER_APPLY_FN_TYPE0(_name) CONCAT(_name,FilterApplyFn)
#define FILTER_APPLY_AXIS_FN_NAME(_name, _dim) CONCAT3(_name,FilterApplyAxis,_dim)
#define FILTER_APPLY_AXIS_FN_TYPE(_name, _dim) CONCAT4(_name,FilterApplyAxis,_dim,Fn)

#define DECLARE_FILTER_TYPE_DIM(_name_coef, _name_state, _dim) \
    typedef struct {                                    \
        FILTER_COEFFS_TYPE(_name_coef) coeffs;           \
        FILTER_STATE_TYPE(_name_state) state[_dim];     \
    } FILTER_TYPE(_name_state, _dim);                   \
/**/

#define DECLARE_FILTER_COEFFS(_name_coef, _coeffs) \
    typedef struct { float c[_coeffs]; } FILTER_COEFFS_TYPE(_name_coef); \
/**/

#define DECLARE_FILTER_STATE(_name_state, _states) \
    typedef struct { float s[_states]; } FILTER_STATE_TYPE(_name_state); \
/**/

#define PP_DECLARE_FILTER_DIM(r, data, elem)                                      \
    DECLARE_FILTER_TYPE_DIM(BOOST_PP_TUPLE_ELEM(0,data), BOOST_PP_TUPLE_ELEM(1,data), elem); \
/**/

// declare filter for each dimension
#define DECLARE_FILTER_TYPE_DIMS(_name_coef, _name_state, _dim_seq)           \
    BOOST_PP_SEQ_FOR_EACH(PP_DECLARE_FILTER_DIM,                               \
        (_name_coef, _name_state), _dim_seq)                                     \
/**/

// create alias for 1D filter
#define DECLARE_FILTER_TYPE_ALIAS0(_name_coef, _name_state)                   \
    typedef FILTER_TYPE(_name_state, 1) FILTER_TYPE0(_name_state);          \
/**/

// common part of filter
#define DECLARE_FILTER_12(_name_coef, _name_state, _coeffs, _states, _dim_seq) \
    DECLARE_FILTER_STATE(_name_state, _states);                                \
    DECLARE_FILTER_TYPE_DIMS(_name_coef, _name_state, _dim_seq);            \
    DECLARE_FILTER_TYPE_ALIAS0(_name_coef, _name_state);                     \
    DECLARE_FILTER_FN2(_name_coef, _name_state, _dim_seq);                     \
/**/

// declare types for standard filter - both coeffs and states are declared
#define DECLARE_FILTER_1(_name_coef, _name_state, _coeffs, _states, _dim_seq)   \
    DECLARE_FILTER_COEFFS(_name_coef, _coeffs);                                 \
    DECLARE_FILTER_12(_name_coef, _name_state, _coeffs, _states, _dim_seq)      \
M_END

#define DECLARE_FILTER(_name, _coeffs, _states, _dim_seq)                       \
    DECLARE_FILTER_1(_name, _name, _coeffs, _states, _dim_seq)  \
/**/

// declare types for derived filter - coeffs are already created
#define DECLARE_FILTER_2(_name_coef, _name_state, _coeffs, _states, _dim_seq)   \
    STATIC_ASSERT(sizeof(FILTER_COEFFS_TYPE(_name_coef)) == sizeof(float) * _coeffs, coef_size_mismatch); \
    DECLARE_FILTER_12(_name_coef, _name_state, _coeffs, _states, _dim_seq)      \
M_END


#define DECLARE_FILTER_INIT_FN2(_name_coef, _name_state, _dim) \
void FILTER_INIT_FN_NAME(_name_state, _dim)(FILTER_TYPE(_name_state, _dim) *filter); \
void _name_state##FilterUpdateCoeffs##_dim(FILTER_TYPE(_name_state, _dim) *filter, const FILTER_COEFFS_TYPE(_name_coef) *coeffs) \
/**/
#define DECLARE_FILTER_INIT_FN0(_name_coef, _name_state) \
void FILTER_INIT_FN_NAME0(_name_state)(FILTER_TYPE(_name_state, 1) *filter); \
void _name_state##FilterUpdateCoeffs(FILTER_TYPE(_name_state, 1) *filter, const FILTER_COEFFS_TYPE(_name_coef) *coeffs) \
/**/

#define DECLARE_FILTER_APPLY_FN2(_name_coef, _name_state, _dim) \
void FILTER_APPLY_FN_NAME(_name_state, _dim)(FILTER_TYPE(_name_state, _dim) *filter, float out[_dim], const float in[_dim]); \
typedef void FILTER_APPLY_FN_TYPE(_name_state, _dim)(FILTER_TYPE(_name_state, _dim) *filter, float out[_dim], const float in[_dim]); \
float FILTER_APPLY_AXIS_FN_NAME(_name_state, _dim)(FILTER_TYPE(_name_state, _dim) *filter, const float in, int axis); \
typedef float FILTER_APPLY_AXIS_FN_TYPE(_name_state, _dim)(FILTER_TYPE(_name_state, _dim) *filter, const float in, int axis) \
/**/

#define DECLARE_FILTER_APPLY_FN0(_name_coef, _name_state) \
float FILTER_APPLY_FN_NAME0(_name_state)(FILTER_TYPE(_name_state, 1) *filter, const float in); \
typedef float FILTER_APPLY_FN_TYPE0(_name_state)(FILTER_TYPE(_name_state, 1) *filter, const float in) \

#define DECLARE_FILTER_FN2_DIM(_name_coef, _name_state, _dim) \
    DECLARE_FILTER_INIT_FN2(_name_coef, _name_state, _dim); \
    DECLARE_FILTER_APPLY_FN2(_name_coef, _name_state, _dim) \
/**/

#define PP_DECLARE_FILTER_FN2_EACH(r, data, elem) \
    DECLARE_FILTER_FN2_DIM(BOOST_PP_TUPLE_ELEM(0,data),BOOST_PP_TUPLE_ELEM(1,data),elem);

#define DECLARE_FILTER_FN0(_name_coef, _name_state) \
    DECLARE_FILTER_INIT_FN0(_name_coef, _name_state); \
    DECLARE_FILTER_APPLY_FN0(_name_coef, _name_state) \
/**/

#define DECLARE_FILTER_FN2(_name_coef, _name_state, _dim_seq) \
    DECLARE_FILTER_FN0(_name_coef, _name_state); \
    BOOST_PP_SEQ_FOR_EACH(PP_DECLARE_FILTER_FN2_EACH,                               \
        (_name_coef, _name_state), _dim_seq)                                     \
/**/

/* ---- param-seq -> signature/args ---- */
/* --- tuple arity (2 vs 3) without BOOST_PP_IF/TUPLE_SIZE --- */
#define PP__CAT_(a,b) a##b
#define PP__CAT(a,b) PP__CAT_(a,b)

#define PP__NARG_(...) PP__ARGN(__VA_ARGS__, 3,2,1,0)
#define PP__ARGN(_1,_2,_3,N,...) N
#define PP__TPL_ARITY(t) PP__NARG_ t  /* (T,name) -> 2 ; (T,name,[n]) -> 3 */

/* decl: (T,name) | (T,name,[n]) */
#define PP__PARAM_DECL_2(T, name)        T name
#define PP__PARAM_DECL_3(T, name, arr)   T name arr
#define PP__PARAM_DECL(t) PP__CAT(PP__PARAM_DECL_, PP__TPL_ARITY(t)) t

/* name only for call site */
#define PP__PARAM_NAME_2(T, name)        name
#define PP__PARAM_NAME_3(T, name, arr)   name
#define PP__PARAM_NAME(t) PP__CAT(PP__PARAM_NAME_, PP__TPL_ARITY(t)) t

/* seq helpers */
#define PP__PARAM_LIST_EACH(r, data, i, elem) BOOST_PP_COMMA_IF(i) PP__PARAM_DECL(elem)
#define PP__PARAM_LIST(seq) BOOST_PP_SEQ_FOR_EACH_I(PP__PARAM_LIST_EACH, ~, seq)

#define PP__ARG_LIST_EACH(r, data, i, elem)   BOOST_PP_COMMA_IF(i) PP__PARAM_NAME(elem)
#define PP__ARG_LIST(seq)   BOOST_PP_SEQ_FOR_EACH_I(PP__ARG_LIST_EACH, ~, seq)

#define FILTER_COEFFS_FN_NAME(_name_coef, _kind) CONCAT3(_name_coef, FilterCoeffs, _kind)
#define FILTER_INIT_KIND_FN_NAME(_name_state, _kind) CONCAT3(_name_state, FilterInit, _kind)
#define FILTER_INIT_KIND_DIM_FN_NAME(_name_state, kind, dim) CONCAT4(_name_state, FilterInit, kind, dim)

/* one dimension */
#define DEFINE_FILTER_INIT_KIND_DIM_P(_coef, _state, _dim, _kind, _params_seq)                 \
    static void MAYBE_UNUSED                                                                    \
    FILTER_INIT_KIND_DIM_FN_NAME(_state, _kind, _dim)(                                          \
        FILTER_TYPE(_state, _dim) *f, PP__PARAM_LIST(_params_seq))                              \
    {                                                                                           \
        FILTER_INIT_FN_NAME(_state, _dim)(f);                                                   \
        FILTER_COEFFS_FN_NAME(_coef, _kind)(&f->coeffs, PP__ARG_LIST(_params_seq));               \
    }

/* drive all dims */
#define PP__DEFINE_FILTER_INIT_KIND_EACH_P(r, data, dim)                                        \
    DEFINE_FILTER_INIT_KIND_DIM_P(                                                              \
        BOOST_PP_TUPLE_ELEM(0, data),  /* _coef  */                                            \
        BOOST_PP_TUPLE_ELEM(1, data),  /* _state */                                            \
        dim,                                                                                   \
        BOOST_PP_TUPLE_ELEM(2, data),  /* _kind  */                                            \
        BOOST_PP_TUPLE_ELEM(3, data))  /* _params_seq */

/* 1D alias + numbered dims */
#define DEFINE_FILTER_INIT_KIND(_coef, _state, _dim_seq, _kind, _params_seq)                    \
    void FILTER_COEFFS_FN_NAME(_coef, _kind)(                                                   \
        FILTER_COEFFS_TYPE(_coef) *coeffs,                                                      \
        PP__PARAM_LIST(_params_seq));                                                            \
    static void MAYBE_UNUSED                                                                    \
    FILTER_INIT_KIND_FN_NAME(_state, _kind)(                                                    \
        FILTER_TYPE(_state, 1) *f, PP__PARAM_LIST(_params_seq))                                 \
    {                                                                                           \
        FILTER_INIT_FN_NAME(_state, 1)(f);                                                      \
        FILTER_COEFFS_FN_NAME(_coef, _kind)(&f->coeffs, PP__ARG_LIST(_params_seq));               \
    }                                                                                           \
    BOOST_PP_SEQ_FOR_EACH(PP__DEFINE_FILTER_INIT_KIND_EACH_P,                                   \
        (_coef, _state, _kind, _params_seq), _dim_seq)                                          \
M_END
