#include <quickdev/cdf_von_mises.h>

// #############################################################################################################################################
double int_indef_von_mises( double const & mean, double const & k, double const & value, double const & precision )
{
    double sum = 0;
    double sum_component = 0;
    int order = 1;

    double const precision_value = pow( 10, int( log10( std::numeric_limits<double>::min() ) * precision ) );

    double const norm_value = quickdev::normalizeEuler( value );
    double const norm_mean = quickdev::normalizeEuler( mean );

    do
    {
        sum_component = gsl_sf_bessel_In( order, k ) * ( sin( (double)order * ( norm_value - norm_mean ) ) / (double) order );
        sum += sum_component;
        order ++;
    }
    while( fabs( sum_component ) > precision_value );

    return ( 1.0 / ( 2.0 * M_PI ) ) * ( ( norm_value - norm_mean ) + ( 2.0 / gsl_sf_bessel_I0( k ) ) * sum );
}

// #############################################################################################################################################
double cdf_von_mises( double const & mean, double const & k, double const & value )
{
    return int_indef_von_mises( mean, k, value ) - int_indef_von_mises( 0, k, -M_PI - mean );
}

// #############################################################################################################################################
double cdf_inv_von_mises( double const & mean, double const & k, double const & value )
{
    return 0;
}
