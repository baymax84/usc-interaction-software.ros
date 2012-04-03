#ifndef QUICKDEVCPP_QUICKDEV_GAUSSIANPDF_H_
#define QUICKDEVCPP_QUICKDEV_GAUSSIANPDF_H_

#include <quickdev/macros.h>
#include <quickdev/matrix.h>
#include <deque>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

template<size_t __Dim__>
class GaussianPDF
{
public:
    typedef double _Storage;
    typedef quickdev::VectorWrapper<__Dim__, _Storage> _DataPoint;
    typedef quickdev::Matrix<__Dim__, __Dim__, _Storage, quickdev::matrix_base::policies::Symmetric> _Covariance;

protected:
    _DataPoint mean_;
    _Covariance covariance_;

    std::deque<_DataPoint> data_points_;
    _DataPoint sum_;

    bool mean_needs_update_;
    bool covariance_needs_update_;

public:
    template<class... __Args>
    GaussianPDF()
    :
        //mean_( 0 ),
        //covariance_( 0 ),
        sum_( 0 ),
        mean_needs_update_( true ),
        covariance_needs_update_( true )
    {
        update();
    }

    decltype( mean_ ) const & getMean() const
    {
        return mean_;
    }

    decltype( covariance_ ) const & getCovariance() const
    {
        return covariance_;
    }

    void push_back( _DataPoint const & data_point )
    {
        invalidate();
        data_points_.push_back( data_point );
        sum_ += data_point;
    }

    void clear()
    {
        invalidate();
        data_points_.clear();
        sum_.set( 0 );
    }

    _DataPoint const & updateMean()
    {
        if( mean_needs_update_ )
        {
            mean_ = sum_ / data_points_.size();
            mean_needs_update_ = false;
        }
        return getMean();
    }

    _Covariance const & updateCovariance()
    {
        if( covariance_needs_update_ )
        {
            updateMean();

            _DataPoint sum( 0 );
            for( auto data_point = data_points_.cbegin(); data_point != data_points_.cend(); ++data_point )
            {
                auto diff = mean_ - *data_point;
                sum += diff * diff;
            }

            for( size_t i = 0; i < __Dim__; ++i )
            {
                covariance_( i, i ) = sum[i] / data_points_.size();
            }

            covariance_needs_update_ = false;
        }
        return getCovariance();
    }

    void update()
    {
        updateCovariance();
    }

    size_t size() const
    {
        return data_points_.size();
    }

protected:
    void invalidate()
    {
        mean_needs_update_ = true;
        covariance_needs_update_ = true;
    }
};

} // quickdev

#endif // QUICKDEVCPP_QUICKDEV_GAUSSIANPDF_H_
