#ifndef UNSIGNEDNORMAL_PDFUNSIGNEDNORMAL_H_
#define UNSIGNEDNORMAL_PDFUNSIGNEDNORMAL_H_

#include <gsl/gsl_cdf.h>
#include <gsl/gsl_randist.h>
#include <math.h>

double pdf_unsigned_normal( double const & mean, double const & variance, double const & value );

#endif // UNSIGNEDNORMAL_PDFUNSIGNEDNORMAL_H_
