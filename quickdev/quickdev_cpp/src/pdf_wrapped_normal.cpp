#include <quickdev/pdf_wrapped_normal.h>

// #############################################################################################################################################
double pdf_wrapped_normal( double const & mean, double const & std_dev, double const & value )
{
    return pdf_von_mises( mean, 1.0 / pow( std_dev, 2 ), value );
}
