#include <quickdev/pdf_wrapped_normal.h>

// #############################################################################################################################################
double pdf_wrapped_normal( double const & mean, double const & variance, double const & value )
{
    return pdf_von_mises( mean, 1.0 / variance, value );
}
