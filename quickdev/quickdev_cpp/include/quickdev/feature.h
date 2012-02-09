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
#include <deque>
#include <vector>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

/*namespace feature
{
    namespace mode
    {
        struct COPY{};
        struct LVAL{};
        struct RVAL{};
    } // mode
} // feature

template<class __FeatureStorage, class __Mode>
class FeatureStorage{};

template<class __FeatureStorage>
class FeatureStorage<__FeatureStorage, feature::mode::COPY>
{
public:
    FeatureStorage( )
    {
        //
    }
};

template<class __FeatureStorage>
class FeatureStorage<__FeatureStorage, feature::mode::LVAL>
{
public:
    FeatureStorage()
    {
        //
    }
};

template<class __FeatureStorage>
class FeatureStorage<__FeatureStorage, feature::mode::RVAL>
{
public:
    FeatureStorage()
    {
        //
    }
};*/

template<class __Data>
class Feature;

// #############################################################################################################################################
namespace feature
{

//! Trait to determine if a type is a feature; specialization for non-features
template<class __Data>
struct is_feature
{
    const static bool value = false;
};

//! Trait to determine if a type is a feature; specialization for features
template<class __Data>
struct is_feature<Feature<__Data> >
{
    const static bool value = true;
};

template<class __Data>
struct feature_is_numeric
{
    const static bool value = false;
};

template<class __Data>
struct feature_is_numeric<Feature<__Data> >
{
    const static bool value = std::is_arithmetic<__Data>::value;
};

//! Traits for features
template<class __Data>
struct traits
{
    const static bool is_numeric = feature_is_numeric<__Data>::value;
    const static bool is_feature = is_feature<__Data>::value;
};

// #############################################################################################################################################
//! Mode types used for compile-time switching
namespace mode
{
    //! Distance-specific modes
    namespace distance
    {
        struct EUCLIDIAN{};
        struct EUCLIDIAN_CIRCULAR{};
        struct GAUSSIAN{};
    } // distance
} // mode

} // feature

// #############################################################################################################################################
//! Class used to simplify distance calculations between feature elements and aid in recursion for complex features
template<class __Data>
class FeatureComponent
{
protected:
    const __Data value_;

public:
    FeatureComponent( const __Data & value )
    :
        value_( value )
    {
        //
    }

    QUICKDEV_DECLARE_ACCESSOR2( value_, Value )

    template
    <
        class __Mode,
        class __OtherData,
        typename std::enable_if<(std::is_same<__Mode, feature::mode::distance::EUCLIDIAN>::value), int>::type = 0
    >
    double distanceToImpl( __OtherData const & other ) const
    {
        return value_ - other;
    }

    template
    <
        class __Mode,
        class __OtherData,
        typename std::enable_if<(std::is_same<__Mode, feature::mode::distance::EUCLIDIAN_CIRCULAR>::value), int>::type = 0
    >
    double distanceToImpl( __OtherData const & other, double const & min, double const & max ) const
    {
        auto angle1 = other;
        auto angle2 = value_;

        auto const range = max - min;

        // normalize angle1 and angle2 between min and max
        angle1 = fmod( angle1, range ) + min;
        angle2 = fmod( angle2, range ) + min;

        // find min distance between angle1 and angle2
        auto distance = fabs( angle2 - angle1 );
        if( distance > range / 2.0 ) return range - distance;
        return distance;
    }

    template
    <
        class __Mode,
        class __OtherData,
        typename std::enable_if<(std::is_same<__Mode, feature::mode::distance::GAUSSIAN>::value), int>::type = 0
    >
    double distanceToImpl( __OtherData const & other ) const
    {
        return value_ - other;
    }

    //! Calculates the distance to another feature if this feature is complex
    /*! Recurses via Feature::distanceTo() */
    template
    <
        class __Mode,
        class __MData,
        class __OtherData,
        typename std::enable_if<(feature::is_feature<__MData>::value), int>::type = 0
    >
    double distanceToHelper( const FeatureComponent<__OtherData> & other ) const
    {
        //std::cout << "calculating distance between partially non-numeric components" << std::endl;
        return value_.distanceTo<__Mode>( other.getValue() );
    }

    //! Calculates the distance to another feature if this feature is not complex but the other feature is complex
    /*! Recurses via Feature::distanceTo() */
    template
    <
        class __Mode,
        class __MData,
        class __OtherData,
        typename std::enable_if<(!feature::is_feature<__MData>::value && feature::is_feature<__OtherData>::value), int>::type = 0
    >
    double distanceToHelper( const FeatureComponent<__OtherData> & other ) const
    {
        //std::cout << "calculating distance between partially non-numeric components" << std::endl;
        return other.getValue().distanceTo<__Mode>( value_ );
    }

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
    double distanceToHelper( const FeatureComponent<__OtherData> & other, __Args... args ) const
    {
        //std::cout << "calculating distance between numeric components: " << value_ << " and " << other.getValue() << std::endl;
        return distanceToImpl<__Mode>( other.getValue(), args... );
    }

    template<class __Mode, class __OtherData, class... __Args>
    double distanceTo( const FeatureComponent<__OtherData> & other, __Args... args ) const
    {
        return distanceToHelper<__Mode, __Data>( other, args... );
    }
};

// #############################################################################################################################################
//! Class used to contain and aid in analysis of n-dimensional numeric or hierarchical feature vectors
template<class __Data>
class Feature
{
public:
    typedef __Data _Data;
    typedef std::deque<__Data> _Storage;

protected:
    _Storage storage_;

public:
    Feature(){}

    Feature( Feature<__Data> const & feature )
    :
        storage_( feature.storage_ )
    {
        //
    }

    //! Construct a Feature from a _Storage object
    Feature( _Storage const & storage )
    :
        storage_( storage )
    {
        //
    }

    //! Construct a Feature from an initializer list
    Feature( std::initializer_list<__Data> storage )
    :
        storage_( storage )
    {
        //
    }

    //! Construct a Feature from some other data type
    template<class __Storage>
    Feature( std::vector<__Storage> const & storage )
    {
        storage_.resize( storage.size() );
        std::copy( storage.begin(), storage.end(), storage_.begin() );
    }

    //! Construct a Feature from a variadic template of size > 0
    /*! Will fail at compile time if, after args... is expanded, all types do not match __Data */
    template<class... __Args, typename std::enable_if<(sizeof...(__Args) > 0), int>::type = 0>
    Feature( __Args... args )
    :
        // construct our storage via an initializer list containing args...
        storage_( { args... } )
    {
        //
    }

    typename _Storage::iterator begin()
    {
        return storage_.begin();
    }

    typename _Storage::const_iterator cbegin() const
    {
        return storage_.cbegin();
    }

    typename _Storage::const_iterator begin() const
    {
        return cbegin();
    }

    typename _Storage::iterator end()
    {
        return storage_.end();
    }

    typename _Storage::const_iterator cend() const
    {
        return storage_.cend();
    }

    typename _Storage::const_iterator end() const
    {
        return cend();
    }

    size_t size() const
    {
        return storage_.size();
    }

    // getStorage()
    QUICKDEV_DECLARE_ACCESSOR2( storage_, Storage )

    //! Calculates a vector of distances between ordered pairs of components of both features
    /*! Specifically, given an M-dimensional feature and an N-dimensional feature, this function will return a vector of size max( m, n )
     *  where each element is the distance between feature1[i] and feature2[i] (or the distance to the corresponding zero-value if featureX
     *  is not defined for [i]) */
    template<class __Mode, class __OtherData, class... __Args>
    std::vector<double> getDistanceComponents( const Feature<__OtherData> & other, __Args... args ) const
    {
        const auto & storage1 = storage_;
        const auto & storage2 = other.getStorage();

        auto component1_it = storage1.cbegin();
        auto component2_it = storage2.cbegin();

        // say we have:
        // storage1 = { 1, 2, 3 }
        // storage2 = { 1 }
        //
        // we want to compare the elements in the same n-dimensional space, so ideally we would expand storage2 to { 1, 0, 0 }
        // however, rather than resizing the feature, when we reach the last element, we can simply use zero for the remaining comparisons
        bool component1_end = false;
        bool component2_end = false;

        typedef __Data _Component1Data;
        typedef __OtherData _Component2Data;

        const _Component1Data storage1_zero( 0 );
        const _Component2Data storage2_zero( 0 );

        std::vector<double> distance_components;
        distance_components.reserve( std::max( storage1.size(), storage2.size() ) );

        while( true )
        {
            component1_end = storage1.size() == 0 || component1_it == storage1.cend();
            component2_end = storage2.size() == 0 || component2_it == storage2.cend();

            if( component1_end && component2_end ) break;

            FeatureComponent<_Component1Data> component1( component1_end ? storage1_zero : *component1_it );
            FeatureComponent<_Component2Data> component2( component2_end ? storage2_zero : *component2_it );

            distance_components.push_back( component1.distanceTo<__Mode>( component2, args... ) );

            if( !component1_end ) ++component1_it;
            if( !component2_end ) ++component2_it;
        }

        return distance_components;
    }

private:
    //! Calculates the euclidian distance to another feature
    template<
        class __Mode,
        typename std::enable_if<(std::is_same<__Mode, feature::mode::distance::EUCLIDIAN>::value), int>::type = 0,
        class __OtherData
    >
    double distanceToImpl( Feature<__OtherData> const & other ) const
    {
        const auto distance_components = getDistanceComponents<__Mode>( other );

        double total_distance = 0.0;
        for( auto distance_component = distance_components.cbegin(); distance_component != distance_components.cend(); ++distance_component )
        {
            total_distance += fabs( *distance_component );
        }

        return total_distance;
    }

    template<
        class __Mode,
        typename std::enable_if<(std::is_same<__Mode, feature::mode::distance::EUCLIDIAN_CIRCULAR>::value), int>::type = 0,
        class __OtherData
    >
    double distanceToImpl( Feature<__OtherData> const & other, double const & min, double const & max ) const
    {
        const auto distance_components = getDistanceComponents<__Mode>( other, min, max );

        double total_distance = 0.0;
        for( auto distance_component = distance_components.cbegin(); distance_component != distance_components.cend(); ++distance_component )
        {
            total_distance += fabs( *distance_component );
        }

        return total_distance;
    }

    //! Calculates the gaussian distance to another feature
    template<
        class __Mode,
        typename std::enable_if<(std::is_same<__Mode, feature::mode::distance::GAUSSIAN>::value), int>::type = 0,
        class __OtherData,
        class __SigmaData,
        typename std::enable_if<(std::is_floating_point<__SigmaData>::value), int>::type = 0
    >
    double distanceToImpl( Feature<__OtherData> const & other, Feature<__SigmaData> const & sigmas, double const & resolution = 1.0 ) const
    {
        const auto distance_components = getDistanceComponents<__Mode>( other );

        const auto half_resolution = resolution / 2.0;

        double total_distance = 1.0;
        auto sigma = sigmas.cbegin();
        for( auto distance_component = distance_components.cbegin(); distance_component != distance_components.cend(); ++distance_component, ++sigma )
        {
            auto const abs_distance_component = fabs( *distance_component );
            total_distance *= gsl_cdf_gaussian_P( abs_distance_component + half_resolution, *sigma ) - gsl_cdf_gaussian_P( abs_distance_component - half_resolution, *sigma );
        }

        return 1.0 - total_distance;
    }

public:
    //! Calculate the distance from this feature to another feature
    /*! \tparam __Mode the distance algorithm to use
     *  \param other the feature to which the distance should be calculated
     *  \param args the set of arguments, if any, to pass to the distance algorithm */
    template<class __Mode, class __OtherData, class... __Args>
    double distanceTo( const Feature<__OtherData> & other, __Args... args ) const
    {
        return distanceToImpl<__Mode>( other, args... );
    }

    //! Calculate the distance from this feature to a simple numeric feature
    /*! Simply wraps the given numeric value in a feature and forwards all other params to the primary distanceTo() implementation */
    template<class __Mode, class __OtherData, class std::enable_if<std::is_arithmetic<__OtherData>::value, int>::type = 0, class... __Args>
    double distanceTo( const __OtherData & other, __Args... args ) const
    {
        return distanceToImpl<__Mode>( Feature<__OtherData>( other ), args... );
    }

    template<class __OtherData, class... __Args>
    double distanceTo( const Feature<__OtherData> & other, __Args... args ) const
    {
        return distanceTo<feature::mode::distance::EUCLIDIAN>( other, args... );
    }
};

} // quickdev

#endif // QUICKDEVCPP_QUICKDEV_FEATURE_H_
