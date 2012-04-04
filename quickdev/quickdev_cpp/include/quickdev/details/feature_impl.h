#define __FeatureComponent FeatureComponent<__Data>

// =============================================================================================================================================
template<class __Data>
__FeatureComponent::FeatureComponent( __Data const & value )
:
    value_( value )
{
    //
}

// #############################################################################################################################################

// =============================================================================================================================================
template<class __Data>
template
<
    class __Mode,
    class __OtherData,
    typename std::enable_if<(std::is_same<__Mode, feature::mode::distance::EUCLIDIAN>::value), int>::type
>
double __FeatureComponent::distanceToImpl( __OtherData const & other ) const
{
    return value_ - other;
}

// =============================================================================================================================================
template<class __Data>
template
<
    class __Mode,
    class __OtherData,
    typename std::enable_if<(std::is_same<__Mode, feature::mode::distance::EUCLIDIAN_CIRCULAR>::value), int>::type
>
double __FeatureComponent::distanceToImpl( __OtherData const & other, double const & min, double const & max ) const
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

// =============================================================================================================================================
template<class __Data>
template
<
    class __Mode,
    class __OtherData,
    typename std::enable_if<(std::is_same<__Mode, feature::mode::distance::GAUSSIAN>::value), int>::type
>
double __FeatureComponent::distanceToImpl( __OtherData const & other ) const
{
    return value_ - other;
}

// =============================================================================================================================================
template<class __Data>
template
<
    class __Mode,
    class __OtherData,
    typename std::enable_if<(std::is_same<__Mode, feature::mode::distance::GAUSSIAN_FAST>::value), int>::type
>
double __FeatureComponent::distanceToImpl( __OtherData const & other ) const
{
    return value_ - other;
}

// #############################################################################################################################################

// =============================================================================================================================================
template<class __Data>
template
<
    class __Mode,
    class __MData,
    class __OtherData,
    typename std::enable_if<(feature::is_feature<__MData>::value), int>::type
>
double __FeatureComponent::distanceToHelper( FeatureComponent<__OtherData> const & other ) const
{
    //std::cout << "calculating distance between partially non-numeric components" << std::endl;
    return value_.distanceTo<__Mode>( other.getValue() );
}

// =============================================================================================================================================
template<class __Data>
template
<
    class __Mode,
    class __MData,
    class __OtherData,
    typename std::enable_if<(!feature::is_feature<__MData>::value && feature::is_feature<__OtherData>::value), int>::type
>
double __FeatureComponent::distanceToHelper( FeatureComponent<__OtherData> const & other ) const
{
    //std::cout << "calculating distance between partially non-numeric components" << std::endl;
    return other.getValue().distanceTo<__Mode>( value_ );
}

// =============================================================================================================================================
template<class __Data>
template
<
    class __Mode,
    class __MData,
    class __OtherData,
    class... __Args,
    typename std::enable_if<(!feature::is_feature<__MData>::value && !feature::is_feature<__OtherData>::value), int>::type
>
double __FeatureComponent::distanceToHelper( FeatureComponent<__OtherData> const & other, __Args&&... args ) const
{
    //std::cout << "calculating distance between numeric components: " << value_ << " and " << other.getValue() << std::endl;
    return distanceToImpl<__Mode>( other.getValue(), std::forward<__Args>( args )... );
}

// #############################################################################################################################################

// =============================================================================================================================================
template<class __Data>
template
<
    class __Mode,
    class __OtherData,
    class... __Args
>
double __FeatureComponent::distanceTo( FeatureComponent<__OtherData> const & other, __Args&&... args ) const
{
    return distanceToHelper<__Mode, __Data>( other, std::forward<__Args>( args )... );
}

#undef __FeatureComponent

// #############################################################################################################################################

#define __Feature Feature<__Data>

// =============================================================================================================================================

template<class __Data>
__Feature::Feature(){}

// =============================================================================================================================================
template<class __Data>
__Feature::Feature( Feature<__Data> const & feature )
:
    storage_( feature.storage_ )
{
    //
}

// =============================================================================================================================================
template<class __Data>
__Feature::Feature( _Storage const & storage )
:
    storage_( storage )
{
    //
}

// =============================================================================================================================================
template<class __Data>
template<class __MData>
__Feature::Feature( std::initializer_list<__MData> storage )
:
    storage_( storage.size() )
{
    std::copy( storage.begin(), storage.end(), storage_.begin() );
}

// =============================================================================================================================================
template<class __Data>
template<class __MData>
__Feature::Feature( std::vector<__MData> const & storage )
:
    storage_( storage.size() )
{
    std::copy( storage.begin(), storage.end(), storage_.begin() );
}

// #############################################################################################################################################

// =============================================================================================================================================
template<class __Data>
typename __Feature::_Storage::iterator
__Feature::begin()
{
    return storage_.begin();
}

// =============================================================================================================================================
template<class __Data>
typename __Feature::_Storage::const_iterator
__Feature::cbegin() const
{
    return storage_.cbegin();
}

// =============================================================================================================================================
template<class __Data>
typename __Feature::_Storage::const_iterator
__Feature::begin() const
{
    return cbegin();
}

// =============================================================================================================================================
template<class __Data>
typename __Feature::_Storage::iterator
__Feature::end()
{
    return storage_.end();
}

// =============================================================================================================================================
template<class __Data>
typename __Feature::_Storage::const_iterator
__Feature::cend() const
{
    return storage_.cend();
}

// =============================================================================================================================================
template<class __Data>
typename __Feature::_Storage::const_iterator
__Feature::end() const
{
    return cend();
}

// =============================================================================================================================================
template<class __Data>
size_t __Feature::size() const
{
    return storage_.size();
}

// #############################################################################################################################################

// =============================================================================================================================================
template<class __Data>
template<class __Mode, class __OtherData, class... __Args>
std::vector<double> __Feature::getDistanceComponents( Feature<__OtherData> const & other, __Args&&... args ) const
{
    auto const & storage1 = storage_;
    auto const & storage2 = other.getStorage();

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

        distance_components.push_back( component1.distanceTo<__Mode>( component2, std::forward<__Args>( args )... ) );

        if( !component1_end ) ++component1_it;
        if( !component2_end ) ++component2_it;
    }

    return distance_components;
}

// #############################################################################################################################################

// =============================================================================================================================================
template<class __Data>
template
<
    class __Mode,
    typename std::enable_if<(std::is_same<__Mode, feature::mode::distance::EUCLIDIAN>::value), int>::type,
    class __OtherData
>
double __Feature::distanceToImpl( Feature<__OtherData> const & other ) const
{
    auto const distance_components = getDistanceComponents<__Mode>( other );

    double total_distance = 0.0;
    for( auto distance_component = distance_components.cbegin(); distance_component != distance_components.cend(); ++distance_component )
    {
        total_distance += fabs( *distance_component );
    }

    return total_distance;
}

// =============================================================================================================================================
template<class __Data>
template
<
    class __Mode,
    typename std::enable_if<(std::is_same<__Mode, feature::mode::distance::EUCLIDIAN_CIRCULAR>::value), int>::type,
    class __OtherData
>
double __Feature::distanceToImpl( Feature<__OtherData> const & other, double const & min, double const & max ) const
{
    auto const distance_components = getDistanceComponents<__Mode>( other, min, max );

    double total_distance = 0.0;
    for( auto distance_component = distance_components.cbegin(); distance_component != distance_components.cend(); ++distance_component )
    {
        total_distance += fabs( *distance_component );
    }

    return total_distance;
}

// =============================================================================================================================================
template<class __Data>
template
<
    class __Mode,
    typename std::enable_if<(std::is_same<__Mode, feature::mode::distance::GAUSSIAN>::value), int>::type,
    class __OtherData,
    class __SigmaData,
    typename std::enable_if<(std::is_floating_point<__SigmaData>::value), int>::type
>
double __Feature::distanceToImpl( Feature<__OtherData> const & other, Feature<__SigmaData> const & sigmas, double const & resolution ) const
{
    auto const distance_components = getDistanceComponents<__Mode>( other );

    auto const half_resolution = resolution / 2.0;

    double total_distance = 1.0;
    auto sigma = sigmas.cbegin();

    for( auto distance_component = distance_components.cbegin(); distance_component != distance_components.cend(); ++distance_component, ++sigma )
    {
        auto const abs_distance_component = fabs( *distance_component );
        auto const probability_component = gsl_cdf_gaussian_P( abs_distance_component + half_resolution, *sigma ) - gsl_cdf_gaussian_P( abs_distance_component - half_resolution, *sigma );

        total_distance *= probability_component;
    }

    return 1.0 - total_distance;
}

// =============================================================================================================================================
template<class __Data>
template
<
    class __Mode,
    typename std::enable_if<(std::is_same<__Mode, feature::mode::distance::GAUSSIAN_FAST>::value), int>::type,
    class __OtherData,
    class __SigmaData,
    typename std::enable_if<(std::is_floating_point<__SigmaData>::value), int>::type
>
double __Feature::distanceToImpl( Feature<__OtherData> const & other, Feature<__SigmaData> const & sigmas, double const & std_dev ) const
{
    auto const distance_components = getDistanceComponents<__Mode>( other );

    double total_distance = 1.0;
    auto sigma = sigmas.cbegin();

    for( auto distance_component = distance_components.cbegin(); distance_component != distance_components.cend(); ++distance_component, ++sigma )
    {
        auto const abs_distance_component = fabs( *distance_component );
        auto const probability_component = abs_distance_component < *sigma * std_dev ? 1.0 : *sigma * std_dev / abs_distance_component;

        total_distance *= probability_component;
    }

    return 1.0 - total_distance;
}

// #############################################################################################################################################

// =============================================================================================================================================
template<class __Data>
template
<
    class __Mode,
    class __OtherData,
    class... __Args
>
double __Feature::distanceTo( Feature<__OtherData> const & other, __Args&&... args ) const
{
    return distanceToImpl<__Mode>( other, std::forward<__Args>( args )... );
}

// =============================================================================================================================================
template<class __Data>
template
<
    class __Mode,
    class __OtherData,
    class std::enable_if<std::is_arithmetic<__OtherData>::value, int>::type,
    class... __Args
>
double __Feature::distanceTo( __OtherData const & other, __Args&&... args ) const
{
    return distanceToImpl<__Mode>( Feature<__OtherData>( other ), std::forward<__Args>( args )... );
}

// =============================================================================================================================================
template<class __Data>
template
<
    class __OtherData,
    class... __Args
>
double __Feature::distanceTo( Feature<__OtherData> const & other, __Args&&... args ) const
{
    return distanceTo<feature::mode::distance::EUCLIDIAN>( other, std::forward<__Args>( args )... );
}

template<class __Data>
std::string __Feature::toString() const
{
    std::stringstream ss;
    ss << "{ ";

    for( auto elem_it = cbegin(); elem_it != cend(); ++elem_it )
    {
        ss << *elem_it;

        if( elem_it != cend() - 1 ) ss << ", ";
    }
    ss << " }";

    return ss.str();
}
