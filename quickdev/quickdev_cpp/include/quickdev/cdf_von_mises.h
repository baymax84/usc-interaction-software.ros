#ifndef VONMISES_CDFVONMISES_H_
#define VONMISES_CDFVONMISES_H_

#include <gsl/gsl_sf_bessel.h>
#include <iostream>
#include <math.h>
#include <limits>

// #############################################################################################################################################
//! The upper component of the indefinite integral of the Von Mises distribution
/*! @param k kappa, the measure of concentration
 *  @param value the value at which to evaluate the indefinite integral
 *  @param precision the percentage of the current archiecture's precision to use as a lower-limit when performing iterative calculations
 */
double int_upper_von_mises( double const & k, double const & value, double const & precision = 0.05 );

// #############################################################################################################################################
//! The cumulative distribution function of the zero-centered Von Mises distribution
/*! @param k kappa, the measure of concentration
 *  @param value, the value at which to evaluate the CDF
 */
double cdf_norm_von_mises( double const & k, double const & value );

// #############################################################################################################################################
//! The cumulative distribution function of the Von Mises distribution
/*! @param mean the location of the center of the distribution
 *  @param k kappa, the measure of concentration
 *  @param value the value at which to evaluate the CDF
 */
double cdf_von_mises( double const & mean, double const & k, double const & value );

// #############################################################################################################################################
//! The inverse of the Von Mises CDF
/*! @warning unimplemented
 *  @note used to map a uniform distribution onto the Von Mises PDF and thereby "sample" from it
 *  @param mean the location of the center of the distribution
 *  @param k kappa, the measure of concentration
 *  @param value the value at which to evaluate the CDF
 */
double cdf_inv_von_mises( double const & mean, double const & k, double const & value );

#endif // VONMISES_CDFVONMISES_H_
