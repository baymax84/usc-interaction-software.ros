#include <quickdev/cdf_wrapped_normal.h>

// #############################################################################################################################################
double cdf_wrapped_normal( double const & mean, double const & variance, double const & value )
{
    return cdf_von_mises( mean, 1.0 / variance, value );
}

// #############################################################################################################################################
double cdf_inv_wrapped_normal( double const & mean, double const & variance, double const & value )
{
    return cdf_inv_von_mises( mean, 1.0 / variance, value );
}
