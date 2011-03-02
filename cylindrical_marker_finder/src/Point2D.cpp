#include "Point2D.hpp"
#include "Point.hpp"
#include <cmath>

Point2D rotatePoint2D( const float theta, const Point2D &c )
{
    const float a = boost::get<0>( c );
    const float b = boost::get<1>( c );
    return boost::make_tuple( a*std::cos(theta)-b*std::sin(theta),
        a*std::sin(theta) + b*std::cos(theta ));
}

double distancePointPoint( const Point2D &a, const Point2D &b )
{
    return distancePointPoint( Point(a), Point(b) );
}
