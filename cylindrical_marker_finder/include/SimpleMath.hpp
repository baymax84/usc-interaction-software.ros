#ifndef SIMPLEMATH_HPP_
#define SIMPLEMATH_HPP_

double degToRad( double angle );
double radToDeg( double angle );

double exp (const double num, const int e);
//Use this instead of fmod please!  fmod doesn't work for negative values
//of a.
double goodMod( double a, double b );

inline bool even( int i ){ return i % 2 == 0; }
inline bool odd( int i ){ return i % 2 != 0; }
//Only on windows systems (and intel compiler).

//Is this number reasonable?  Not at infinity, etc.
bool valid( const double );

enum Quadrant {I=1, II, III, IV};

extern const double PI;

//Resulting theta is between 0 and 360
void cartesianToPolar(
    const double x, const double y, 
    double * const r, double * const theta);


#endif
