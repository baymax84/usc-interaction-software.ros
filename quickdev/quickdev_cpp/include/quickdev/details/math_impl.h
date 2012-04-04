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
