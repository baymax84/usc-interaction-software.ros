#ifndef UNSIGNEDNORMAL_CDFUNSIGNEDNORMAL_H_
#define UNSIGNEDNORMAL_CDFUNSIGNEDNORMAL_H_

#include <gsl/gsl_cdf.h>
#include <math.h>

double cdf_unsigned_normal( double const & mean, double const & variance, double const & value );

#endif // UNSIGNEDNORMAL_CDFUNSIGNEDNORMAL_H_
