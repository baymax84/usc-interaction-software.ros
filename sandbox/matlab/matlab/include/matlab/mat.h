/***************************************************************************
 *  include/matlab/mat.h
 *  --------------------
 *
 *  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of usc-ros-pkg nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

#include "matrix.h"
#include <vector>
#include <type_traits>
#include <inttypes.h>
#include <string>
#include <sstream>
#include <iostream>

namespace matlab
{

namespace util
{

//! Simple utility struct to determine if two statically-known data types contain the same value
/*! Specialization for two different values */
template<class __Data, __Data __Id1__, __Data __Id2__>
struct is_same_value
{
	const static bool value = false;
};

//! Simple utility struct to determine if two statically-known data types contain the same value
/*! Specialization for two identical values */
template<class __Data, __Data __Id__>
struct is_same_value<__Data, __Id__, __Id__>
{
	const static bool value = true;
};

}

//! Generic helper for matlab_get_class
/*! If there are no specialized matches for __Data then we say that the ClassID that corresponds to this data type is unknown */
template<class __Data>
struct matlab_get_class_helper
{
	const static mxClassID value = mxUNKNOWN_CLASS;
};

//! Generic helper for matlab_get_type
/*! If there are no specialized matches for __ClassId__ then we say that the type that corresponds to this ClassID is unknown */
template<unsigned int __ClassId__>
struct matlab_get_type_helper
{
	typedef struct UnknownType type;
};

// macro to declare a two-way type conversion between __Type and CLASS
// specifically, we make a specialized matlab_get_class_helper instance to look up CLASS given __Type
// and a specialized matlab_get_type_helper to look up __Type given CLASS
#define DECLARE_MATLAB_TYPE_CONVERSION( __Type, CLASS ) \
template<> \
struct matlab_get_class_helper<__Type> \
{ \
	const static mxClassID value = CLASS; \
}; \
 \
template<> \
struct matlab_get_type_helper<CLASS> \
{ \
	typedef __Type type; \
}

// declare conversions for known types
DECLARE_MATLAB_TYPE_CONVERSION( bool, mxLOGICAL_CLASS );
DECLARE_MATLAB_TYPE_CONVERSION( char, mxCHAR_CLASS );
DECLARE_MATLAB_TYPE_CONVERSION( void, mxVOID_CLASS );
DECLARE_MATLAB_TYPE_CONVERSION( double, mxDOUBLE_CLASS );
DECLARE_MATLAB_TYPE_CONVERSION( float, mxSINGLE_CLASS );
DECLARE_MATLAB_TYPE_CONVERSION( int8_t, mxINT8_CLASS );
DECLARE_MATLAB_TYPE_CONVERSION( uint8_t, mxUINT8_CLASS );
DECLARE_MATLAB_TYPE_CONVERSION( int16_t, mxINT16_CLASS );
DECLARE_MATLAB_TYPE_CONVERSION( uint16_t, mxUINT16_CLASS );
DECLARE_MATLAB_TYPE_CONVERSION( int32_t, mxINT32_CLASS );
DECLARE_MATLAB_TYPE_CONVERSION( uint32_t, mxUINT32_CLASS );
DECLARE_MATLAB_TYPE_CONVERSION( int64_t, mxINT64_CLASS );
DECLARE_MATLAB_TYPE_CONVERSION( uint64_t, mxUINT64_CLASS );

//! Defines the mxClassID that corresponds to the __Data
template<class __Data>
struct matlab_get_class
{
	const static mxClassID value = matlab_get_class_helper<__Data>::value;
};

//! Defines the data type that corresponds to __ClassId__
template<unsigned int __ClassId__>
struct matlab_get_type
{
	typedef typename matlab_get_type_helper<__ClassId__>::type type;
};

//! Generic helper for is_matlab_compatible
template<unsigned int __ClassId__>
struct is_matlab_compatible_helper
{
	const static bool value = true;
};

//! Specialized helper for is_matlab_compatible
/*! mxUKNOWN_CLASS is the only incompatible mxClassID */
template<>
struct is_matlab_compatible_helper<mxUNKNOWN_CLASS>
{
	const static bool value = false;
};

//! Defines whether or not a data type can be stored in a matlab array
/*! Specifically, value is true if the corresponding class to __Data is not mxUKNOWN_CLASS and false otherwise */
template<class __Data>
struct is_matlab_compatible
{
	const static bool value = is_matlab_compatible_helper<matlab_get_class<__Data>::value>::value;
};

//! Type traits for Mats of type __Data
template<class __Data>
struct MatDataTypes
{
	const static bool is_floating_ = std::is_floating_point<__Data>::value;

	const static mxClassID matlab_class_ = matlab_get_class<__Data>::value;
	typedef typename matlab_get_type<matlab_class_>::type _MatlabDataType;
	const static bool is_matlab_compatible_ = is_matlab_compatible<__Data>::value;
};

//! Base Mat class with dimensions but no storage
/*! Stores a Dim and provides assignment operators */
class MatBase
{
public:
	template<class __Data = int>
	struct Dim
	{
		typedef __Data _Data;

		__Data rows_;
		__Data cols_;

		__Data & x_;
		__Data & y_;

		Dim( const __Data & rows = 0, const __Data & cols = 0 )
		:
			rows_( rows ), cols_( cols ), x_( cols_ ), y_( rows_ )
		{
			//
		}

		//! Assign data from an existing Dim<__Data>
		/*! This is necessary because we have non-const reference values ( x_, y_ ) that are not handled automatically by the compiler */
		Dim & operator=( const Dim & other )
		{
			rows_ = other.rows_;
			cols_ = other.cols_;
			return *this;
		}

		friend std::ostream & operator<<( std::ostream & out, const Dim<__Data> & dim )
		{
			out << "dim: (" << dim.rows_ << "x" << dim.cols_ << ") ";
			return out;
		}
	};

	typedef Dim<unsigned int> _Dim;

protected:
	_Dim dim_;

public:
	_Dim::_Data & rows_;
	_Dim::_Data & cols_;

	MatBase( const _Dim & dim )
	:
		dim_( dim ), rows_( dim_.rows_ ), cols_( dim_.cols_ )
	{
		//
	}

	MatBase( const _Dim::_Data & rows, const _Dim::_Data & cols )
	:
		dim_( rows, cols ), rows_( dim_.rows_ ), cols_( dim_.cols_ )
	{
		//
	}

	//! Assign data from an existing MatBase
	/*! This is necessary because we have non-const reference values ( rows_, cols_ ) that are not handled automatically by the compiler */
	MatBase & operator=( const MatBase & other )
	{
		dim_ = other.dim_;
		return *this;
	}

	const _Dim & getDim() const
	{
		return dim_;
	}

	friend std::ostream & operator<<( std::ostream & out, const MatBase & mat )
	{
		out << mat.dim_;
		return out;
	}
};

//! Typedefs for StdMat
template<class __Data>
struct StdMatTypes
{
	typedef std::vector<__Data> _RowVec;
	typedef _RowVec _ColVec;
	typedef std::vector<__Data> _Mat;
};

//! Typedefs for MatlabMat
struct MatlabMatTypes
{
	typedef mxArray _Mat;
	typedef _Mat * _MatPtr;
};

//! A generic Mat class with indexing based on MatlabMat
/*! \sa MatlabMat */
template<class __Data>
class MatlabMat : public MatBase
{
public:
	typedef MatlabMatTypes::_Mat _Mat;
	typedef MatlabMatTypes::_MatPtr _MatPtr;

protected:
	_MatPtr mat_;

public:
	typedef MatBase _Parent;

	//! Wrap an existing const mxArray
	/*! Note that we const_cast to remove the const property.
	 *  When using this constructor, one should use the syntax to preserve const-ness: const MatlabMat<Type>( value ) */
	template<class __Mat, typename std::enable_if<(std::is_same<__Mat, const _Mat>::value), int>::type = 0>
	MatlabMat( __Mat * mat )
	:
		_Parent( mxGetM( mat ), mxGetN( mat ) ), mat_( const_cast<_MatPtr>( mat ) )
	{
		//
	}

	//! Wrap an existing non-const mxArray
	template<class __Mat, typename std::enable_if<(std::is_same<__Mat, _Mat>::value), int>::type = 0>
	MatlabMat( __Mat * mat )
	:
		_Parent( mxGetM( mat ), mxGetN( mat ) ), mat_( mat )
	{
		//
	}

	//! Generic constructor to pass all arguments to parent class
	/*! Note that we also allocate a new mxArray since this constructor is only used to create new data rather than wrap existing data */
	template<class... __ParentArgs>
	MatlabMat( __ParentArgs&&... parent_args )
	:
		_Parent( parent_args... ), mat_( mxCreateNumericMatrix( rows_, cols_, MatDataTypes<__Data>::matlab_class_, mxREAL ) )
	{
		//
	}

	//! Generic constructor to read the desired complexity and pass all other arguments to parent class
	/*! Note that we also allocate a new mxArray since this constructor is only used to create new data rather than wrap existing data */
	template<class... __ParentArgs>
	MatlabMat( mxComplexity complexity, __ParentArgs&&... parent_args )
	:
		_Parent( parent_args... ), mat_( mxCreateNumericMatrix( rows_, cols_, MatDataTypes<__Data>::matlab_class_, complexity ) )
	{
		//
	}

	//! Change the dimensions of the matrix
	/*! \attention Experimental; reallocates storage for this mat; destroys existing data */
	void resize( const MatBase::_Dim & dim )
	{
		resize( dim.rows_, dim.cols_ );
	}

	//! Change the dimensions of the matrix
	/*! \attention Experimental; reallocates storage for this mat; destroys existing data */
	void resize( const MatBase::_Dim::_Data & rows, const MatBase::_Dim::_Data & cols )
	{
		//if( mat_ ) mxFree( mat_ );
		*this = MatlabMat( rows, cols );
	}

	//! Convert our data to a formatted string; specialization enabled if __ClassId__ != mxCHAR_CLASS
	/*! Print out all stored values, showing row vectors and column vectors */
	template<mxClassID __ClassId__, typename std::enable_if<(!util::is_same_value<decltype( __ClassId__ ), __ClassId__, mxCHAR_CLASS>::value), int>::type = 0>
	const std::string dataToString() const
	{
		std::stringstream ss;
		for( unsigned int row = 0; row < rows_; ++row )
		{
			if( rows_ > 1 ) ss << "\n{ ";
			for( unsigned int col = 0; col < cols_; ++col )
			{
				ss << getData()[col * rows_ + row];
				if( cols_ > 1 && col < cols_ - 1 ) ss << ", ";
			}
			if( rows_ > 1 )
			{
				ss << " }";
				if( row < rows_ - 1 ) ss << ", ";
			}
		}
		return ss.str();
	}

	//! Convert our data to a formatted string; specialization enabled if __ClassId__ == mxCHAR_CLASS
	/*! Since we're storing an array of characters, it's possible to compile them directly into an std::string */
	template<mxClassID __ClassId__, typename std::enable_if<(util::is_same_value<decltype( __ClassId__ ), __ClassId__, mxCHAR_CLASS>::value), int>::type = 0>
	const std::string dataToString() const
	{
		return std::string( mxArrayToString( mat_ ) );
	}

	//! Convert our data to a formatted string
	/*! In order to use std::enable_if in a class, we need to use a dependent type in the template declaration even if we're enabling based on a class template value */
	const std::string dataToString() const
	{
		return dataToString<MatDataTypes<__Data>::matlab_class_>();
	}

	const std::string toString() const
	{
		std::stringstream ss;
		ss << getDim() << "data: " << "{ " << dataToString() << " }";
		return ss.str();
	}

	friend std::ostream& operator<<( std::ostream &out, const MatlabMat<__Data> & mat )
	{
		out << mat.toString();
		return out;
	}

	_MatPtr getMat()
	{
		return mat_;
	}

	__Data * getData()
	{
		return mxGetPr( mat_ );
	}

	__Data * const getData() const
	{
		return mxGetPr( mat_ );
	}

	__Data & operator[]( const unsigned int & index )
	{
		return getData()[index];
	}

	const __Data & operator[]( const unsigned int & index ) const
	{
		return getData()[index];
	}

	__Data & at( const unsigned int & row, const unsigned int & col )
	{
		return getData()[col * rows_ + row];
	}

	const __Data & at( const unsigned int & row, const unsigned int & col ) const
	{
		return getData()[col * rows_ + row];
	}

	__Data & first()
	{
		return *getData();
	}

	const __Data & first() const
	{
		return *getData();
	}

	operator mxArray*()
	{
		return getMat();
	}

	operator __Data&()
	{
		return first();
	}

	operator const __Data&() const
	{
		return first();
	}

	operator const std::string() const
	{
		return dataToString();
	}
};

//! A generic Mat class with indexing based on MatlabMat
/*! \sa MatlabMat */
template<class __Data>
class StdMat : public MatBase
{
public:
	typedef StdMatTypes<__Data> _StdMatTypes;
	typedef typename _StdMatTypes::_RowVec _RowVec;
	typedef typename _StdMatTypes::_ColVec _ColVec;
	typedef typename _StdMatTypes::_Mat _Mat;

protected:
	_Mat mat_;

public:
	typedef MatBase _Parent;

	//! Generic constructor to pass all arguments to parent class
	/*! Note that since we inherit from MatBase, as soon as we construct _Parent we can use rows_ and cols_ from MatBase */
	template<class... __ParentArgs>
	StdMat( __ParentArgs... parent_args )
	:
		_Parent( parent_args... ), mat_( rows_ * cols_ )
	{
		//
	}

	//! Convert our data to a formatted string; specialization enabled if __Data != char
	/*! Print out all stored values, showing row vectors and column vectors */
	template<class __MData, typename std::enable_if<(!std::is_same<__MData, char>::value), int>::type = 0>
	const std::string dataToString() const
	{
		std::stringstream ss;
		for( unsigned int row = 0; row < rows_; ++row )
		{
			if( rows_ > 1 ) ss << "\n{ ";
			for( unsigned int col = 0; col < cols_; ++col )
			{
				ss << operator[](col * rows_ + row);
				if( cols_ > 1 && col < cols_ - 1 ) ss << ", ";
			}
			if( rows_ > 1 )
			{
				ss << " }";
				if( row < rows_ - 1 ) ss << ", ";
			}
		}
		return ss.str();
	}

	//! Convert our data to a formatted string; specialization enabled if __Data == char
	/*! Since we're storing an array of characters, it's possible to compile them directly into an std::string */
	template<class __MData, typename std::enable_if<(std::is_same<__MData, char>::value), int>::type = 0>
	const std::string dataToString() const
	{
		return std::string( mat_.begin() );
	}

	//! Convert our data to a formatted string
	/*! In order to use std::enable_if in a class, we need to use a dependent type in the template declaration even if we're enabling based on a class template value */
	const std::string dataToString() const
	{
		return dataToString<__Data>();
	}

	const std::string toString() const
	{
		std::stringstream ss;
		ss << getDim() << "data: " << "{ " << dataToString() << " }";
		return ss.str();
	}

	friend std::ostream& operator<<( std::ostream &out, const StdMat<__Data> & mat )
	{
		out << mat.toString();
		return out;
	}

	_Mat getMat()
	{
		return mat_;
	}

	typename _Mat::iterator getData()
	{
		return mat_.begin();
	}

	typename _Mat::const_iterator getData() const
	{
		return mat_.begin();
	}

	__Data & operator[]( const unsigned int & index )
	{
		return *( getData() + index );
	}

	const __Data & operator[]( const unsigned int & index ) const
	{
		return *( getData() + index );
	}

	__Data & at( const unsigned int & row, const unsigned int & col )
	{
		return *( getData() + ( col * rows_ + row ) );
	}

	const __Data & at( const unsigned int & row, const unsigned int & col ) const
	{
		return *( getData() + ( col * rows_ + row ) );
	}

	__Data & first()
	{
		return *getData();
	}

	const __Data & first() const
	{
		return *getData();
	}

	operator __Data&()
	{
		return first();
	}

	operator const __Data&() const
	{
		return first();
	}

	operator const std::string() const
	{
		return dataToString();
	}
};

//! Adapter for types that are compatible with matlab
/*! To form this adapter, we simply inherit from MatlabMat<__Data>
 *  \sa MatHelper<__Data, false>, MatlabMat */
template<class __Data, bool __IsMatlabCompatible__>
class MatHelper : public MatlabMat<__Data>
{
public:
	typedef MatlabMat<__Data> _Parent;

	//! Generic constructor to pass all arguments to parent class
	template<class... __ParentArgs>
	MatHelper( __ParentArgs... parent_args )
	:
		_Parent( parent_args... )
	{
		//
	}
};

//! Adapter for types that are not compatible with matlab
/*! To form this adapter, we simply inherit from StdMat<__Data>
 *  \sa MatHelper<>, StdMat */
template<class __Data>
class MatHelper<__Data, false> : public StdMat<__Data>
{
public:
	typedef StdMat<__Data> _Parent;

	//! Generic constructor to pass all arguments to parent class
	template<class... __ParentArgs>
	MatHelper( __ParentArgs... parent_args )
	:
		_Parent( parent_args... )
	{
		//
	}
};

//! A generic Mat class; can be matlab-specific
/*! This class will automatically wrap either a MatlabMat or an StdMat;
 *  Specifically, if MatDataTypes<__Data>::is_matlab_compatible_ is true, then this class will wrap a MatlabMat<__Data>
 *  Otherwise, this class will wrap an StdMat<__Data> */
template<class __Data>
class Mat : public MatHelper<__Data, MatDataTypes<__Data>::is_matlab_compatible_>
{
public:
	typedef MatHelper<__Data, MatDataTypes<__Data>::is_matlab_compatible_> _Parent;

	//! Generic constructor to pass all arguments to parent class
	template<class... __ParentArgs>
	Mat( __ParentArgs... parent_args )
	:
		_Parent( parent_args... )
	{
		//
	}
};

}
