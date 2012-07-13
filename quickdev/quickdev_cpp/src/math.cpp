#include <quickdev/math.h>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

// =============================================================================================================================================
void seedRand( unsigned int const & value )
{
    static bool initialized = false;

    if( !initialized )
    {
        srand( value );
        initialized = true;
    }
}

// =============================================================================================================================================
void seedRand()
{
    quickdev::seedRand( time( NULL ) );
}

// =============================================================================================================================================
double cdf_gaussian( double const & mean, double const & sigma, double const & min, double const & max )
{
    return gsl_cdf_gaussian_P( max - mean, sigma ) - gsl_cdf_gaussian_P( min - mean, sigma );
}

} // quickdev
