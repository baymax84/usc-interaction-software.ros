#include <array>

#ifndef OCULESICS_POINT_3D_H_
#define OCULESICS_POINT_3D_H_

template<class __DataType>
class Point3D
{
public:
	typedef __DataType _DataType;
	typedef std::array<__DataType, 3> _Array;
	
	_Array data_;
	__DataType & x, & y, & z;
	
	Point3D( _Array data = _Array() ) : data_( data ), x( data_[0] ), y( data_[1] ), z( data_[2] )
	{
		//
	}
};

#endif // OCULESICS_POINT_3D_H_
