#ifndef QUICKDEVCPP_QUICKDEV_MATH_H_
#define QUICKDEVCPP_QUICKDEV_MATH_H_

#include <quickdev/macros.h>
#include <algorithm>
#include <math.h>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

// =============================================================================================================================================
//! Put value between {-M_PI, M_PI}
template<class __Data>
__Data normalizeEuler( __Data const & value );

// =============================================================================================================================================
//! Get the sign of some value (result is { -1, 0, 1 }
template<class __Data>
inline __Data sign( __Data const & value );

// =============================================================================================================================================
//! Get the sign of some value (result is either -1 or 1)
template<class __Data>
inline __Data signNonzero( __Data const & value );

// =============================================================================================================================================
//! Get the max of two values
template<class __Data>
inline __Data max( __Data const & value1, __Data const & value2 );

// =============================================================================================================================================
//! Get the max of three or more values
template
<
    class __Data,
    class... __Args,
    typename std::enable_if<(sizeof...(__Args) > 1), int>::type = 0
>
inline __Data max( __Data const & value, __Args&&... args );

// =============================================================================================================================================
//! Get the min of two values
template<class __Data>
inline __Data min( __Data const & value1, __Data const & value2 );

// =============================================================================================================================================
//! Get the min of three or more values
template
<
    class __Data,
    class... __Args,
    typename std::enable_if<(sizeof...(__Args) > 1), int>::type = 0
>
inline __Data min( __Data const & value, __Args&&... args );

#include <quickdev/details/math_impl.h>

} // quickdev

#endif // QUICKDEVCPP_QUICKDEV_MATH_H_
