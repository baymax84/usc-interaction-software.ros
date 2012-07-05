// =============================================================================================================================================
template<class __Data>
__Data normalizeEuler( __Data const & value )
{
    __Data const clamp = fmod( value, 2 * M_PI );
    if( clamp > M_PI ) return clamp - 2 * M_PI;
    else if( clamp < -M_PI ) return clamp + 2 * M_PI;
    return clamp;
}

// =============================================================================================================================================
//! Get the sign of some value (result is { -1, 0, 1 }
template<class __Data>
inline __Data sign( __Data const & value )
{
    return value > 0 ? 1 : value < 0 ? -1 : 0;
}

//! Get the sign of some value (result is either -1 or 1)
template<class __Data>
inline __Data signNonzero( __Data const & value )
{
    return value > 1 ? 1 : -1;
}

// =============================================================================================================================================
//! Get the max of two values
template<class __Data>
inline __Data max( __Data const & value1, __Data const & value2 )
{
    return std::max( value1, value2 );
}

// =============================================================================================================================================
//! Get the max of three or more values
template
<
    class __Data,
    class... __Args,
    typename std::enable_if<(sizeof...(__Args) > 1), int>::type
>
inline __Data max( __Data const & value, __Args&&... args )
{
    return std::max( value, max( std::forward<__Args>( args )... ) );
}

// =============================================================================================================================================
//! Get the min of two values
template<class __Data>
inline __Data min( __Data const & value1, __Data const & value2 )
{
    return std::min( value1, value2 );
}

// =============================================================================================================================================
//! Get the min of three or more values
template
<
    class __Data,
    class... __Args,
    typename std::enable_if<(sizeof...(__Args) > 1), int>::type
>
inline __Data min( __Data const & value, __Args&&... args )
{
    return std::min( value, min( std::forward<__Args>( args )... ) );
}

// =============================================================================================================================================
template<class __Data>
__Data random( __Data const & min, __Data const & max )
{
    quickdev::seedRand();
    return min + ( max - min ) * double( std::rand() ) / RAND_MAX;
}

// =============================================================================================================================================
template<class __Data>
__Data gaussian_product_mean( __Data const & mean1, __Data const & variance1, __Data const & mean2, __Data const & variance2 )
{
    return ( mean1 * variance2 + mean2 * variance1 ) / ( variance1 + variance2 );
}

// =============================================================================================================================================
template<class __Data>
__Data gaussian_product_variance( __Data const & variance1, __Data const & variance2 )
{
    return ( variance1 * variance2 ) / ( variance1 + variance2 );
}

// =============================================================================================================================================
template
<
    class __Data,
    class... __Args,
    typename std::enable_if<(sizeof...(__Args) > 1 ), int>::type
>
__Data gaussian_product_variance( __Data const & mean, __Args&&... args )
{
    return gaussian_product_mean( mean, gaussian_product_variance( args... ) );
}

// =============================================================================================================================================
template<class __Data>
std::pair<__Data, __Data> gaussian_product( __Data const & mean1, __Data const & variance1, __Data const & mean2, __Data const & variance2 )
{
    return std::pair<__Data, __Data>( gaussian_product_mean( mean1, variance1, mean2, variance2 ), gaussian_product_variance( variance1, variance2 ) );
}
