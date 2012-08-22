#include <quickdev/pdf_von_mises.h>

// #############################################################################################################################################
double pdf_von_mises( double const & mean, double const & k, double const & value )
{
    return exp( k * cos( quickdev::normalizeEuler( value - mean ) ) ) / ( 2.0 * M_PI * gsl_sf_bessel_I0( k ) );
}
