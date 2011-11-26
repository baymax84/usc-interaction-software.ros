#include <math.h>
#include <type_traits>
#include <stdint.h>
#include <boost/type_traits/is_base_of.hpp>

#define GET_UNIT_NAMESPACE() \
unit

#define GET_UNIT_NS() \
GET_UNIT_NAMESPACE()

#define DECLARE_UNIT_NAMESPACE() \
namespace GET_UNIT_NAMESPACE()

#define DECLARE_UNIT_NS() \
DECLARE_UNIT_NAMESPACE()

#define DECLARE_UNIT_CONVERSION_LAMBDA( __FromData, __ToData, var_name, conversion... ) \
DECLARE_UNIT_NS() \
{ \
	template<> \
	struct UnitConverter<__FromData, __ToData, __ToData> \
	{ \
		template<class __MFromStorage> \
		static typename std::enable_if<std::is_same<__MFromStorage, __FromData>::value, __ToData>::type \
		convert( const __MFromStorage & var_name ) \
		{ \
			conversion \
		} \
	}; \
}

#define DECLARE_NUMERIC_UNIT_STORAGE_CONVERSION( __FromData, __ToData, __FromStorage, __ToStorage, var_name, conversion ) \
DECLARE_UNIT_NS() \
{ \
	template<> \
	struct UnitConverter<__FromData, __ToData, __ToStorage> \
	{ \
		template<class __MFromStorage> \
		static typename std::enable_if<std::is_same<__MFromStorage, __FromStorage>::value, __ToStorage>::type \
		convert( const __MFromStorage & var_name ) \
		{ \
			return conversion; \
		} \
	}; \
}

#define DECLARE_NUMERIC_UNIT_CONVERSION( __FromData, __ToData, var_name, conversion ) \
DECLARE_UNIT_NS() \
{ \
	template<class __ToStorage> \
	struct UnitConverter<__FromData, __ToData, __ToStorage> \
	{ \
		template<class __FromStorage> \
		static __ToStorage convert( const __FromStorage & var_name ) \
		{ \
			return conversion; \
		} \
	}; \
}

#define DECLARE_SCALAR_UNIT_CONVERSION( __FromData, __ToData, scalar ) \
DECLARE_NUMERIC_UNIT_CONVERSION( __FromData, __ToData, value, value * scalar ); \
DECLARE_NUMERIC_UNIT_CONVERSION( __ToData, __FromData, value, value / ( scalar ) )

#define DECLARE_NUMERIC_UNIT( __Unit, __Storage... ) \
class __Unit : public GET_UNIT_NS()::NumericUnit<__Unit, ## __Storage> \
{ \
public: \
	template<class... __ParentArgs> \
	__Unit( __ParentArgs&&... parent_args ) : GET_UNIT_NS()::NumericUnit<__Unit, ## __Storage>( parent_args... ) {} \
}; \
 \
DECLARE_UNIT_NS(){ \
template<> \
struct is_numeric_helper<__Unit>{ const static bool value = true; }; \
}

DECLARE_UNIT_NS()
{

template<class __FromData, class __ToData, class __ToStorage>
struct UnitConverter{};

template<class __Unit>
struct is_numeric_helper{ const static bool value = false; };

template<class __Unit>
struct is_numeric
{
	const static bool value = is_numeric_helper<__Unit>::value;
};

template<class __Unit>
struct unit_traits
{
	const static bool is_numeric_ = is_numeric<__Unit>::value;
};

class UnitBaseType{};

template<class __Data, class __Storage>
class UnitBase : public UnitBaseType
{
public:
	typedef __Data _Data;
	typedef __Storage _Storage;
	typedef UnitBase<__Data, __Storage> _UnitBase;

protected:
	__Storage value_;

public:
	UnitBase(){}

	UnitBase( const __Storage value )
	:
		value_( value )
	{
		//
	}

	template<class __FromData, class __FromStorage>
	UnitBase( const UnitBase<__FromData, __FromStorage> & other_unit )
	{
		operator=( other_unit );
	}

	_UnitBase & operator=( const __Storage & value )
	{
		value_ = value;
		return *this;
	}

	_UnitBase & operator=( const _UnitBase & other_unit )
	{
		if( this != &other_unit )
		{
			value_ = other_unit.value_;
		}
		return *this;
	}

	template<class __FromData, class __FromStorage>
	_UnitBase & operator=( const UnitBase<__FromData, __FromStorage> & other_unit )
	{
		value_ = UnitConverter<__FromData, __Data, __Storage>::convert( other_unit.getValue() );
		return *this;
	}

	__Storage & value()
	{
		return value_;
	}

	const __Storage & getValue() const
	{
		return value_;
	}

	operator __Storage()
	{
		return value();
	}

	operator __Storage() const
	{
		return getValue();
	}
};

template<class __Data, class __Storage = double>
class NumericUnit : public UnitBase<__Data, __Storage>
{
public:
	typedef UnitBase<__Data, __Storage> _Parent;

	template<class... __ParentArgs>
	NumericUnit( __ParentArgs&&... parent_args )
	:
		_Parent( parent_args... )
	{
		//
	}
};

template<class __Data, bool __IsNumeric__>
class UnitHelper : public UnitBase<__Data, __Data>
{
public:
	typedef UnitBase<__Data, __Data> _Parent;
	typedef _Parent _Unit;

	template<class... __ParentArgs>
	UnitHelper( __ParentArgs&&... parent_args )
	:
		_Parent( parent_args... )
	{
		//
	}

	template<class __OtherData, typename std::enable_if<(!boost::is_base_of<UnitBaseType, __OtherData>::value), int>::type = 0>
	_Unit & operator=( const __OtherData & value )
	{
		operator=( UnitBase<__OtherData, __OtherData>( value ) );
		return *this;
	}

	// unroll any Unit<Unit<Type> > instances
	template<class __OtherData, typename std::enable_if<(boost::is_base_of<UnitBaseType, __OtherData>::value), int>::type = 0>
	_Unit & operator=( const __OtherData & value )
	{
		operator=( value.getValue() );
		return *this;
	}

	/*template<class __ToData>
	typename std::enable_if<(!boost::is_base_of<UnitBaseType, __ToData>::value), __ToData>::type
	operatorCastHelper( const __ToData & value )
	{
		return UnitConverter<__Data, __ToData, __ToData>::convert( value );
	}

	// unroll any Unit<Unit<Type> > instances
	template<class __ToData>
	typename std::enable_if<(boost::is_base_of<UnitBaseType, __ToData>::value), typename __ToData::_Storage>::type
	operatorCastHelper( const __ToData & value )
	{
		return operatorCastHelper( value.getValue() );
	}*/

	template<class __ToData>//, typename std::enable_if<(!boost::is_base_of<UnitBaseType, __ToData>::value), int>::type = 0>
	operator __ToData()
	{
		return operatorCastHelper( this->value_ );
	}
};

template<class __Data>
class UnitHelper<__Data, true> : public NumericUnit<__Data>
{
public:
	typedef NumericUnit<__Data> _Parent;

	template<class... __ParentArgs>
	UnitHelper( __ParentArgs&&... parent_args )
	:
		_Parent( parent_args... )
	{
		//
	}
};

template<class __Data>
class Unit : public UnitHelper< __Data, unit_traits<__Data>::is_numeric_ >
{
public:
	typedef UnitHelper< __Data, unit_traits<__Data>::is_numeric_ > _Parent;

	template<class... __ParentArgs>
	Unit( __ParentArgs&&... parent_args )
	:
		_Parent( parent_args... )
	{
		//
	}
};

template<class __Data>
typename std::enable_if<(!unit_traits<__Data>::is_numeric_), Unit<__Data> >::type
make_unit( const __Data & data )
{
	return Unit<__Data>( data );
}

template<class __Data, class __Storage = double>
typename std::enable_if<(unit_traits<__Data>::is_numeric_), NumericUnit<__Data, __Storage> >::type
make_unit( const __Storage & value )
{
	return NumericUnit<__Data, __Storage>( value );
}

template<class __ToData, class __FromData>
typename std::enable_if<(!unit_traits<__ToData>::is_numeric_ && !unit_traits<__FromData>::is_numeric_), __ToData >::type
convert( const __FromData & data )
{
	Unit<__ToData> result( make_unit( data ) );
	return result.getValue();
}

} // DECLARE_UNIT_NS()
