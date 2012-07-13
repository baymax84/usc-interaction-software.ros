#ifndef QUICKDEVCPP_QUICKDEV_MATH_H_
#define QUICKDEVCPP_QUICKDEV_MATH_H_

#include <quickdev/macros.h>
#include <algorithm>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <gsl/gsl_cdf.h>

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
void seedRand( unsigned int const & value );

// =============================================================================================================================================
void seedRand();

// =============================================================================================================================================
//! Get the min of three or more values
template
<
    class __Data,
    class... __Args,
    typename std::enable_if<(sizeof...(__Args) > 1), int>::type = 0
>
inline __Data min( __Data const & value, __Args&&... args );

// =============================================================================================================================================
template<class __Data>
__Data random( __Data const & min, __Data const & max );

// =============================================================================================================================================
template<class __Data>
__Data gaussian_product_mean( __Data const & mean1, __Data const & variance1, __Data const & mean2, __Data const & variance2 );

// =============================================================================================================================================
/*
template
<
    class... __Args
    typename std::enable_if<(sizeof...(__Args) > 2 ), int>::type = 0
>
__Data gaussian_product_mean( __Data const & mean, __Data const & variance, __Args&&... args )
{
    return gaussian_product_mean( mean, variance, gaussian_product_mean( mean, variance, args... ), gaussian_product_variance( variance ) );
}
*/

// =============================================================================================================================================
template<class __Data>
__Data gaussian_product_variance( __Data const & variance1, __Data const & variance2 );

// =============================================================================================================================================
template
<
    class __Data,
    class... __Args,
    typename std::enable_if<(sizeof...(__Args) > 1 ), int>::type = 0
>
__Data gaussian_product_variance( __Data const & mean, __Args&&... args );

// =============================================================================================================================================
template<class __Data>
std::pair<__Data, __Data> gaussian_product( __Data const & mean1, __Data const & variance1, __Data const & mean2, __Data const & variance2 );

// =============================================================================================================================================
double cdf_gaussian( double const & mean, double const & sigma, double const & min, double const & max );

#include <quickdev/details/math_impl.h>

} // quickdev

#endif // QUICKDEVCPP_QUICKDEV_MATH_H_
