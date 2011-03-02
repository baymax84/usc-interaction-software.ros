#ifndef POINT_HPP_
#define POINT_HPP_

#include <iosfwd>
#include <cmath>

#include <boost/operators.hpp>
#include "Point2D.hpp"

//This is loosely based on the mathematical 3 dimentional vector.
//I originally had this object oriented but I thought the functional
//method looked much nicer in formulations

//Note that:
//  Distance between vectors a and b is magnitude( a - b )


//TODO: Maybe this should be in some kind of namespace.  I hope not.

class Point : public boost::equality_comparable1< Point ,
	                 boost::addable1< Point ,
			         boost::subtractable1< Point , 
			         boost::multipliable2< Point , double 
			         > > > >
{
public:
	double m_P[3];

	Point(){}
	virtual ~Point(){}
	Point(const Point &p)
	{
		x()= p.x();
		y()= p.y();
		z()= p.z();
	}
	Point(const Point2D &p)
	{
		x()= p.get<0>();
		y()= p.get<1>();
		z()= 0.0;
	}
	Point & operator=( const Point &p )
	{
		x()=p.x();
		y()=p.y();
		z()=p.z();
		return *this;
	}
	bool operator==( const Point &p ) const
	{
		return p.x()==x() && p.y()==y() && p.z() == z();
	}
	Point operator+=( const Point &p )
	{
		m_P[0]+=p.m_P[0];
		m_P[1]+=p.m_P[1];
		m_P[2]+=p.m_P[2];
		return *this;
	}
	Point operator-=( const Point &p )
	{
		m_P[0]-=p.m_P[0];
		m_P[1]-=p.m_P[1];
		m_P[2]-=p.m_P[2];
		return *this;
	}
	Point operator*=( const double p )
	{
		m_P[0]*=p;
		m_P[1]*=p;
		m_P[2]*=p;
		return *this;
	}
	Point operator-()
	{
		Point p;
		p.m_P[0]= -m_P[0];
		p.m_P[1]= -m_P[1];
		p.m_P[2]= -m_P[2];
		return p;
	}
	Point(const double x, const double y, const double z=0.0)
		{m_P[0]=x; m_P[1]=y; m_P[2]=z;}

	std::ostream& put(std::ostream& s) const;

	double operator[] (const size_t i) const { return m_P[i]; }
	double & operator[] (const size_t i) { return m_P[i]; }

	double x() const { return m_P[0];}
	double y() const { return m_P[1];}
	double z() const { return m_P[2];}

	double& x() { return m_P[0];}
	double& y() { return m_P[1];}
	double& z() { return m_P[2];}
};

inline double magnitude(const Point &p) 
{
	return std::sqrt( p.m_P[0]*p.m_P[0] + p.m_P[1]*p.m_P[1] + p.m_P[2]*p.m_P[2] );
}

inline Point unit(const Point &p) 
{
	return Point( p.x()/magnitude(p), p.y()/magnitude(p), p.z()/magnitude(p) );
}


//Different Point or Vector products
double dot( const Point &a, const Point &b );
Point cross( const Point &a, const Point &b );

//Returns angle between Points in degrees
double angleBetween( const Point &a, const Point &b);

//Slope and intercept for XY plane
double slopeXY( const Point &a, const Point &b );
double interceptXY( const Point &a, const Point &b );

//Does this point lie at infinity?  Does it make sense?
bool valid( const Point &d );

std::ostream& operator<<(std::ostream &s, const Point& a);
std::istream& operator>>(std::istream& strm, Point& c );

//  Distance between vectors a and b is magnitude( a - b )
inline double distancePointPoint( const Point &a, const Point &b )
{
    return magnitude( a - b );
}

#endif
