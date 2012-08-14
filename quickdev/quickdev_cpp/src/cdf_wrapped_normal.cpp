#include <quickdev/cdf_wrapped_normal.h>

// #############################################################################################################################################
double cdf_wrapped_normal( double const & mean, double const & std_dev, double const & value )
{
    return cdf_von_mises( mean, 1.0 / std_dev, value );
}

// #############################################################################################################################################
double cdf_inv_wrapped_normal( double const & mean, double const & std_dev, double const & value )
{
    return cdf_inv_von_mises( mean, 1.0 / std_dev, value );
}
