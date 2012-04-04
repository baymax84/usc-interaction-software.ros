/***************************************************************************
 *  include/quickdev/feature.h
 *  --------------------
 *
 *  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of seabee3-ros-pkg nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

#ifndef QUICKDEVCPP_QUICKDEV_FEATURE_H_
#define QUICKDEVCPP_QUICKDEV_FEATURE_H_

#include <quickdev/macros.h>

#include <gsl/gsl_cdf.h>

#include <math.h>
#include <type_traits>
#include <vector>
#include <ostream>
#include <iostream>
#include <sstream>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

template<class __Data>
class Feature;

// #############################################################################################################################################
namespace feature
{

// =============================================================================================================================================
//! Trait to determine if a type is a feature; specialization for non-features
template<class __Data>
struct is_feature
{
    static bool const value = false;
};

// =============================================================================================================================================
//! Trait to determine if a type is a feature; specialization for features
template<class __Data>
struct is_feature<Feature<__Data> >
{
    static bool const value = true;
};

// #############################################################################################################################################

// =============================================================================================================================================
template<class __Data>
struct feature_is_numeric
{
    static bool const value = false;
};

// =============================================================================================================================================
template<class __Data>
struct feature_is_numeric<Feature<__Data> >
{
    static bool const value = std::is_arithmetic<__Data>::value;
};

// #############################################################################################################################################

// =============================================================================================================================================
//! Traits for features
template<class __Data>
struct traits
{
    static bool const is_numeric = feature_is_numeric<__Data>::value;
    static bool const is_feature = is_feature<__Data>::value;
};

// #############################################################################################################################################

// =============================================================================================================================================
//! Mode types used for compile-time switching
namespace mode
{
    //! Distance-specific modes
    namespace distance
    {
        struct EUCLIDIAN{};
        struct EUCLIDIAN_CIRCULAR{};
        struct GAUSSIAN{};
        struct GAUSSIAN_FAST{};
    } // distance
} // mode

} // feature

// #############################################################################################################################################

// =============================================================================================================================================
//! Class used to simplify distance calculations between feature elements and aid in recursion for complex features
template<class __Data>
class FeatureComponent
{
protected:
    __Data const value_;

public:
    // =========================================================================================================================================
    FeatureComponent( __Data const & value );

    QUICKDEV_DECLARE_ACCESSOR2( value_, Value )

    // =========================================================================================================================================
    template
    <
        class __Mode,
        class __OtherData,
        typename std::enable_if<(std::is_same<__Mode, feature::mode::distance::EUCLIDIAN>::value), int>::type = 0
    >
    double distanceToImpl( __OtherData const & other ) const;

    template
    <
        class __Mode,
        class __OtherData,
        typename std::enable_if<(std::is_same<__Mode, feature::mode::distance::EUCLIDIAN_CIRCULAR>::value), int>::type = 0
    >
    double distanceToImpl( __OtherData const & other, double const & min, double const & max ) const;

    template
    <
        class __Mode,
        class __OtherData,
        typename std::enable_if<(std::is_same<__Mode, feature::mode::distance::GAUSSIAN>::value), int>::type = 0
    >
    double distanceToImpl( __OtherData const & other ) const;

    template
    <
        class __Mode,
        class __OtherData,
        typename std::enable_if<(std::is_same<__Mode, feature::mode::distance::GAUSSIAN_FAST>::value), int>::type = 0
    >
    double distanceToImpl( __OtherData const & other ) const;

    // =========================================================================================================================================
    //! Calculates the distance to another feature if this feature is complex
    /*! Recurses via Feature::distanceTo() */
    template
    <
        class __Mode,
        class __MData,
        class __OtherData,
        typename std::enable_if<(feature::is_feature<__MData>::value), int>::type = 0
    >
    double distanceToHelper( FeatureComponent<__OtherData> const & other ) const;

    //! Calculates the distance to another feature if this feature is not complex but the other feature is complex
    /*! Recurses via Feature::distanceTo() */
    template
    <
        class __Mode,
        class __MData,
        class __OtherData,
        typename std::enable_if<(!feature::is_feature<__MData>::value && feature::is_feature<__OtherData>::value), int>::type = 0
    >
    double distanceToHelper( FeatureComponent<__OtherData> const & other ) const;

    //! Calculates the distance to another feature if neither feature is complex
    /*! Bottom level of recursion, if any */
    template
    <
        class __Mode,
        class __MData,
        class __OtherData,
        class... __Args,
        typename std::enable_if<(!feature::is_feature<__MData>::value && !feature::is_feature<__OtherData>::value), int>::type = 0
    >
    double distanceToHelper( FeatureComponent<__OtherData> const & other, __Args&&... args ) const;

    // =========================================================================================================================================
    template
    <
        class __Mode,
        class __OtherData,
        class... __Args
    >
    double distanceTo( FeatureComponent<__OtherData> const & other, __Args&&... args ) const;
};

// #############################################################################################################################################

// =============================================================================================================================================
//! Class used to contain and aid in analysis of n-dimensional numeric or hierarchical feature vectors
template<class __Data>
class Feature
{
public:
    typedef __Data _Data;
    typedef std::vector<__Data> _Storage;

protected:
    _Storage storage_;

public:
    // =========================================================================================================================================
    Feature();

    Feature( Feature<__Data> const & feature );

    //! Construct a Feature from a _Storage object
    Feature( _Storage const & storage );

    //! Construct a Feature from an initializer list
    template<class __MData>
    Feature( std::initializer_list<__MData> storage );

    //! Construct a Feature from some other data type
    template<class __MData>
    Feature( std::vector<__MData> const & storage );

    // =========================================================================================================================================
    typename _Storage::iterator begin();

    typename _Storage::const_iterator cbegin() const;

    typename _Storage::const_iterator begin() const;

    typename _Storage::iterator end();

    typename _Storage::const_iterator cend() const;

    typename _Storage::const_iterator end() const;

    size_t size() const;

    // getStorage()
    QUICKDEV_DECLARE_ACCESSOR2( storage_, Storage )

    // =========================================================================================================================================
    //! Calculates a vector of distances between ordered pairs of components of both features
    /*! Specifically, given an M-dimensional feature and an N-dimensional feature, this function will return a vector of size max( m, n )
     *  where each element is the distance between feature1[i] and feature2[i] (or the distance to the corresponding zero-value if featureX
     *  is not defined for [i]) */
    template
    <
        class __Mode,
        class __OtherData,
        class... __Args
    >
    std::vector<double> getDistanceComponents( Feature<__OtherData> const & other, __Args&&... args ) const;

private:
    // =========================================================================================================================================
    //! Calculates the euclidian distance to another feature
    template
    <
        class __Mode,
        typename std::enable_if<(std::is_same<__Mode, feature::mode::distance::EUCLIDIAN>::value), int>::type = 0,
        class __OtherData
    >
    double distanceToImpl( Feature<__OtherData> const & other ) const;

    template
    <
        class __Mode,
        typename std::enable_if<(std::is_same<__Mode, feature::mode::distance::EUCLIDIAN_CIRCULAR>::value), int>::type = 0,
        class __OtherData
    >
    double distanceToImpl( Feature<__OtherData> const & other, double const & min, double const & max ) const;

    //! Calculates the gaussian distance to another feature
    template
    <
        class __Mode,
        typename std::enable_if<(std::is_same<__Mode, feature::mode::distance::GAUSSIAN>::value), int>::type = 0,
        class __OtherData,
        class __SigmaData,
        typename std::enable_if<(std::is_floating_point<__SigmaData>::value), int>::type = 0
    >
    double distanceToImpl( Feature<__OtherData> const & other, Feature<__SigmaData> const & sigmas, double const & resolution = 1.0 ) const;

    //! Calculates the gaussian distance to another feature using a fast algorithm
    template
    <
        class __Mode,
        typename std::enable_if<(std::is_same<__Mode, feature::mode::distance::GAUSSIAN_FAST>::value), int>::type = 0,
        class __OtherData,
        class __SigmaData,
        typename std::enable_if<(std::is_floating_point<__SigmaData>::value), int>::type = 0
    >
    double distanceToImpl( Feature<__OtherData> const & other, Feature<__SigmaData> const & sigmas, double const & std_dev = 1.0 ) const;

public:
    // =========================================================================================================================================
    //! Calculate the distance from this feature to another feature
    /*! \tparam __Mode the distance algorithm to use
     *  \param other the feature to which the distance should be calculated
     *  \param args the set of arguments, if any, to pass to the distance algorithm */
    template
    <
        class __Mode,
        class __OtherData,
        class... __Args
    >
    double distanceTo( Feature<__OtherData> const & other, __Args&&... args ) const;

    //! Calculate the distance from this feature to a simple numeric feature
    /*! Simply wraps the given numeric value in a feature and forwards all other params to the primary distanceTo() implementation */
    template
    <
        class __Mode,
        class __OtherData,
        class std::enable_if<std::is_arithmetic<__OtherData>::value, int>::type = 0,
        class... __Args
    >
    double distanceTo( __OtherData const & other, __Args&&... args ) const;

    template
    <
        class __OtherData,
        class... __Args
    >
    double distanceTo( Feature<__OtherData> const & other, __Args&&... args ) const;

    // =========================================================================================================================================
    std::string toString() const;

    friend std::ostream & operator<<( std::ostream & out, Feature<__Data> const & feature )
    {
        out << feature.toString();
        return out;
    }
};

// #############################################################################################################################################

#include <quickdev/details/feature_impl.h>

} // quickdev

#endif // QUICKDEVCPP_QUICKDEV_FEATURE_H_
