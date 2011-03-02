#include "Point.hpp"

#include "SimpleMath.hpp"
#include <cmath>
#include <iostream>

std::ostream& Point::put(std::ostream& s) const {
	return (s << "(" << x() << "," << y() << "," << z() << ")");
}

std::istream& operator>>(std::istream& strm, Point& c )
{
	double x,y,z;

	strm >> std::ws;
	strm.ignore(); //skip '('
	strm >> x >> std::ws;
	strm.ignore(); //skip ','
	strm >> y >> std::ws;
	strm.ignore(); //skip ','
	strm >> z >> std::ws;
	strm.ignore(); //skip ')'

	c = Point(x,y,z);
	return strm;
}
std::ostream& operator<<(std::ostream &s, const Point& a)
{
	return a.put(s);
}
double dot( const Point &a, const Point &b )
{
	return a.x()*b.x()+a.y()*b.y()+a.z()*b.z();
}

double angleBetween( const Point &a, const Point &b )
{
	return (180.0/M_PI) * acos( dot(a,b)/(magnitude(a)*magnitude(b)) );
}

Point cross( const Point &a, const Point &b )
{
	return Point( 
			a.y()*b.z() - a.z()*b.y(),
			a.z()*b.x() - a.x()*b.z(),
			a.x()*b.y() - a.y()*b.x() );
}

double slopeXY( const Point &a, const Point &b )
{
	if ( a.x() == b.x() )
        throw std::string("slopeXY returns infinity");
	else if ( a.x() < b.x() )
		return ( b.y() - a.y()) / (b.x() - a.x() );
	else
		return ( a.y() - b.y()) / (a.x() - b.x() );
}

double interceptXY( const Point &a, const Point &b )
{
	if ( a.x() == b.x() )	
        throw std::string("slopeXY returns infinity");
	else if ( a.x() < b.x())
		return ( a.y() - ( slopeXY(a, b) * a.x()) );
	else 
		return ( b.y() - ( slopeXY(a, b) * b.x()) );	
}

bool valid( const Point &d ){ 
    return valid(d.x()) && valid(d.y()) && valid(d.z());}
