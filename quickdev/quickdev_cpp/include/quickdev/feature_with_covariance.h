#include <quickdev/matrix.h>

#ifndef QUICKDEVCPP_QUICKDEV_FEATUREWITHCOVARIANCE_H_
#define QUICKDEVCPP_QUICKDEV_FEATUREWITHCOVARIANCE_H_

namespace quickdev
{

template<class __Data, unsigned int __Dim__>
class FeatureWithCovariance
{
public:
    typedef quickdev::SymmetricMatrix<__Dim__, double> _Covariance;
    typedef FeatureWithCovariance<__Data, __Dim__> _FeatureWithCovariance;

protected:
    __Data feature_;
    _Covariance covariance_;

public:
    FeatureWithCovariance( __Data const & feature )
    :
        feature_( feature )
    {
        //
    }

    operator __Data &()
    {
        return feature_;
    }

    operator __Data const &() const
    {
        return feature_;
    }

    // =========================================================================================================================================
    template
    <
        class __Other,
        typename std::enable_if<(std::is_arithmetic<__Other>::value), int>::type = 0
    >
    _FeatureWithCovariance & operator+=( __Other const & value )
    {
        feature_ += value;
        return *this;
    }

    _FeatureWithCovariance & operator+=( _FeatureWithCovariance const & other )
    {
        feature_ += other.getFeature();
        covariance_ += other.getCovariance();
        return *this;
    }

    template
    <
        class __Other,
        typename std::enable_if<(std::is_arithmetic<__Other>::value), int>::type = 0
    >
    _FeatureWithCovariance operator+( __Other const & value ) const
    {
        return _FeatureWithCovariance( getFeature() + value );
    }

    _FeatureWithCovariance operator+( _FeatureWithCovariance const & other ) const
    {
        _FeatureWithCovariance result( getFeature() + other.getFeature() );
        result.getCovariance().copyFrom( getCovariance() + other.getCovariance() );
        return result;
    }

    // =========================================================================================================================================
    template
    <
        class __Other,
        typename std::enable_if<(std::is_arithmetic<__Other>::value), int>::type = 0
    >
    _FeatureWithCovariance & operator-=( __Other const & value )
    {
        feature_ -= value;
        return *this;
    }

    _FeatureWithCovariance & operator-=( _FeatureWithCovariance const & other )
    {
        feature_ -= other.getFeature();
        covariance_ += other.getCovariance();
        return *this;
    }

    template
    <
        class __Other,
        typename std::enable_if<(std::is_arithmetic<__Other>::value), int>::type = 0
    >
    _FeatureWithCovariance operator-( __Other const & value ) const
    {
        return _FeatureWithCovariance( getFeature() - value );
    }

    _FeatureWithCovariance operator-( _FeatureWithCovariance const & other ) const
    {
        _FeatureWithCovariance result( getFeature() - other.getFeature() );
        result.getCovariance().copyFrom( getCovariance() + other.getCovariance() );
        return result;
    }

    // =========================================================================================================================================
    template
    <
        class __Other,
        typename std::enable_if<(std::is_arithmetic<__Other>::value), int>::type = 0
    >
    _FeatureWithCovariance & operator*=( __Other const & value )
    {
        feature_ *= value;
        return *this;
    }

    _FeatureWithCovariance & operator*=( _FeatureWithCovariance const & other )
    {
        feature_ *= other.getFeature();
        covariance_ += other.getCovariance();
        return *this;
    }

    template
    <
        class __Other,
        typename std::enable_if<(std::is_arithmetic<__Other>::value), int>::type = 0
    >
    _FeatureWithCovariance operator*( __Other const & value ) const
    {
        return _FeatureWithCovariance( getFeature() * value );
    }

    _FeatureWithCovariance operator*( _FeatureWithCovariance const & other ) const
    {
        _FeatureWithCovariance result( getFeature() * other.getFeature() );
        result.getCovariance().copyFrom( getCovariance() + other.getCovariance() );
        return result;
    }

    // =========================================================================================================================================
    __Data & getFeature(){ return feature_; }
    __Data const & getFeature() const { return feature_; }

    _Covariance & getCovariance(){ return covariance_; }
    _Covariance const & getCovariance() const { return covariance_; }
};

} // quickdev

#endif // QUICKDEVCPP_QUICKDEV_FEATUREWITHCOVARIANCE_H_
