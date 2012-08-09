#ifndef VONMISES_PDFVONMISES_H_
#define VONMISES_PDFVONMISES_H_

#include <gsl/gsl_sf_bessel.h>
#include <iostream>
#include <math.h>
#include <limits>

// #############################################################################################################################################
//! The probability density function of the Von Mises distribution
/*! @param mean the location of the center of the distribution
 *  @param k kappa, the measure of concentration
 *  @param value the value at which to evaluate the CDF
 */
double pdf_von_mises( double const & mean, double const & k, double const & value );

#endif // VONMISES_PDFVONMISES_H_
