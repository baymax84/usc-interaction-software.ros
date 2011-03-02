#include "SimpleMath.hpp"
#include <limits>
#include <cassert>
#include <cmath>
#include <iostream>

const double PI = 3.141592653589793;

double goodMod( double a, double b )
{
    return (a - b * floor( a/b ) );
}

double degToRad( double angle )
{
	return angle * (PI/180.0);
}

double radToDeg( double angle )
{
	return (angle/(2*PI)) * 360;
}


double exp (const double num, const int e) {
	if (e == 0)
		return 1;
	return (num * exp(num, e - 1));
}

bool valid( const double d )
{
    assert( std::numeric_limits<double>::has_infinity );
    if( d == std::numeric_limits<double>::infinity() ||
        d == -std::numeric_limits<double>::infinity() ||
        d == std::numeric_limits<double>::quiet_NaN() ||
        d == std::numeric_limits<double>::signaling_NaN() )
        return false;
    if( d < -1e100 || d > 1e100 )
        return false;
    else
        return true;
}

void cartesianToPolar(
    const double x, const double y, 
    double * const r, double * const theta)
{
    *r = sqrt( x*x + y*y );
    *theta = radToDeg(atan2(y,x));
    *theta = *theta < 0 ? 360.0 + *theta : *theta;
}
