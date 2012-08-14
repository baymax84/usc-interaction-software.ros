#include <quickdev/pdf_unsigned_normal.h>

double pdf_unsigned_normal( double const & mean, double const & variance, double const & value )
{
    //   PDF( x )
    // ------------
    // 1 - CDF( 0 )

    return gsl_ran_gaussian_pdf( value - mean, variance ) / ( 1 - gsl_cdf_gaussian_P( 0 - mean, variance ) );
}
