#ifndef VONMISES_CDFWRAPPEDNORMAL_H_
#define VONMISES_CDFWRAPPEDNORMAL_H_

#include <quickdev/cdf_von_mises.h>

// #############################################################################################################################################
//! The cumulative distribution function of the wrapped normal distribution
/*! @param mean the location of the center of the distribution
 *  @param std_dev the measure of concentration
 *  @param value the value at which to evaluate the CDF
 */
double cdf_wrapped_normal( double const & mean, double const & std_dev, double const & value );

// #############################################################################################################################################
//! The cumulative distribution function of the Von Mises distribution
/*! @warning depends on unimplemented function: cdf_inv_von_mises
 *  @note used to map a uniform distribution onto the wrapped normal PDF and thereby "sample" from it
 *  @param mean the location of the center of the distribution
 *  @param std_dev the measure of concentration
 *  @param value the value at which to evaluate the CDF
 */
double cdf_inv_wrapped_normal( double const & mean, double const & std_dev, double const & value );

#endif // VONMISES_CDFWRAPPEDNORMAL_H_
