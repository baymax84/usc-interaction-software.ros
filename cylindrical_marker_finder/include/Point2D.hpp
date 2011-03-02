#ifndef POINT2D_HPP
#define POINT2D_HPP

#include <boost/tuple/tuple.hpp>

typedef boost::tuple<float, float> Point2D;

Point2D rotatePoint2D( const float theta, const Point2D &c );

double distancePointPoint( const Point2D &, const Point2D & );

#endif
