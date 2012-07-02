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

    __Data & getFeature(){ return feature_; }
    __Data const & getFeature() const { return feature_; }

    _Covariance & getCovariance(){ return covariance_; }
    _Covariance const & getCovariance() const { return covariance_; }
};

} // quickdev

#endif // QUICKDEVCPP_QUICKDEV_FEATUREWITHCOVARIANCE_H_
