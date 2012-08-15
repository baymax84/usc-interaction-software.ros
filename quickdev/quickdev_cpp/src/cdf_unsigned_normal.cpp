#include <quickdev/cdf_unsigned_normal.h>

double cdf_unsigned_normal( double const & mean, double const & std_dev, double const & value )
{
    // CDF( x ) - CDF( 0 )
    // -------------------
    //     1 - CDF( 0 )

    double const cdf_alpha = gsl_cdf_gaussian_P( 0 - mean, std_dev );

    return ( gsl_cdf_gaussian_P( value - mean, std_dev ) - cdf_alpha ) / ( 1 - cdf_alpha );
}
