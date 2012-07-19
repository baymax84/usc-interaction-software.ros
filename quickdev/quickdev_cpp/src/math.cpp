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

// =============================================================================================================================================
double angleBetween( double const & value1, double const & value2 )
{
    double const value1_norm = fmod( value1, 2 * M_PI );
    double const value2_norm = fmod( value2, 2 * M_PI );

    double const angle_diff = value2_norm - value1_norm;
    int const angle_sign = quickdev::sign( angle_diff );
    double const abs_diff = fabs( angle_diff );

    if( abs_diff > M_PI ) return angle_diff - angle_sign * M_PI_2;

    return angle_diff;
}

} // quickdev
